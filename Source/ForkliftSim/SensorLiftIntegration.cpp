#include "SensorLiftControl.h"
#include "StabilityLab.h"
#include "JointSensors.h"
#include "DynamicsSensors.h"
#include "ForkActor.h"
#include "EngineUtils.h"
#include "LidarSafety.h"
#include "SensorDrive.h"
#include "HandlingControl.h"
#include "MissionNavigation.h"

bool AStabilityLab::IsSensorLiftTest() const{return TestMode.StartsWith(TEXT("mecanum_lift_sensor_"));}
void AStabilityLab::AcknowledgeSensorLift(){if(MissionControl)MissionControl->AckUntilS=TimeS+.5;if(HandlingControl)HandlingControl->AckUntilS=TimeS+.5;if(SensorLift)SensorLift->AckRequested=true;if(VehicleGuard)VehicleGuard->AckRequested=true;if(DriveSensors)DriveSensors->AckRequested=true;}
void AStabilityController::AcknowledgeSensorLift(){for(TActorIterator<AStabilityLab> I(GetWorld());I;++I){I->AcknowledgeSensorLift();break;}}
void AStabilityLab::UpdateSensorLiftControl(double Dt)
{
    if(!bSensorLiftRequested)return;
    if(!SensorLift){FHydraulicCommand Stop;Stop.Direct=true;Stop.PumpEnabled=false;Fork->SetValveCommand(Stop,TimeS+.1);return;}
    SensorLift::FInput In;In.NowS=TimeS;In.Dt=Dt;In.RequestedM=LiftTargetM;In.Acknowledge=SensorLift->AckRequested;SensorLift->AckRequested=false;In.ActuatorExpired=Fork->HydraulicCommandExpired;
    if(JointSensors)
    {
        const auto& S=JointSensors->Model;const auto& H=S.Channels[4].Last;const auto& L=S.Channels[5].Last;
        In.HeightValid=S.Usable(4,TimeS);In.HeightM=H.Value;In.HeightCaptureS=H.CaptureS;In.HeightSequence=H.Sequence;
        In.LimitsValid=S.Usable(5,TimeS);In.LimitCaptureS=L.CaptureS;In.Lower=L.Lower;In.Upper=L.Upper;
    }
    if(DynamicsSensors)
    {
        const auto& S=DynamicsSensors->Model;const auto& P=S.Channels[1].Last;
        In.PressureValid=S.Usable(1,TimeS);In.PressurePa=P.Value[0];In.PressureCaptureS=P.CaptureS;In.PressureClipped=P.Clipped!=0;
    }
    auto& R=*SensorLift;R.LastInput=In;R.Model.Step(In);
    if(bVehicleSafetyRequested&&(!VehicleGuard||VehicleGuard->Decision.LiftHold))
    {
        R.Model.Command=R.Model.Integral=0;R.Model.PumpEnabled=false;R.Model.Holding=true;
        if(In.HeightValid){R.Model.TargetM=In.HeightM;LiftTargetM=In.HeightM;}
        if(!R.Model.Faulted)R.Model.State=TEXT("vehicle_hold");
    }
    if(HandlingControl&&HandlingControl->Model.Faulted){R.Model.Command=R.Model.Integral=0;R.Model.PumpEnabled=false;R.Model.Holding=true;if(In.HeightValid){LiftTargetM=R.Model.TargetM=In.HeightM;}if(!R.Model.Faulted)R.Model.State=TEXT("handling_hold");}
    if(MissionControl&&MissionControl->Model.Holding()){R.Model.Command=R.Model.Integral=0;R.Model.PumpEnabled=false;R.Model.Holding=true;if(In.HeightValid){LiftTargetM=R.Model.TargetM=In.HeightM;}if(!R.Model.Faulted)R.Model.State=TEXT("mission_hold");}
    if(R.Model.Rebased){LiftTargetM=R.Model.TargetM;R.TestRebasedTarget=R.Model.TargetM;}
    R.SawFault|=R.Model.Faulted;R.SawPressure|=R.Model.State==TEXT("pressure_limited");R.SawExpired|=In.ActuatorExpired;R.SawRecovery|=R.Model.Rebased;
    const bool Withhold=TestMode==TEXT("mecanum_lift_sensor_watchdog")&&TimeS>=6&&TimeS<8;
    if(!Withhold)Fork->SetValveCommand(R.Model.Output(),TimeS+R.Model.Spec.LeaseS);
    // Plant values below are diagnostics recorded AFTER the controller call.
    R.Record(TimeS,ForkHeightM,Fork->HydraulicState.Valve,Fork->HydraulicCommandExpired);
}
void AStabilityLab::ControlSensorLiftTest()
{
    Throttle=Strafe=Steering=Lift=0;Brake=1;
    const bool Down=TestMode.EndsWith(TEXT("down"));const bool Fault=TestMode.Contains(TEXT("fault"))||TestMode.EndsWith(TEXT("watchdog"));
    if(TestMode.EndsWith(TEXT("pressure"))){LiftTargetM=TimeS<2?0:TimeS<20?.9:.02;return;}
    if(Fault)
    {
        const double AckAt=Down?32:14,NewGoalAt=Down?36:18;
        LiftTargetM=TimeS<2?0:Down&&TimeS>=20?.02:.9;
        if(SensorLift&&TimeS>=AckAt&&!SensorLift->TestAckSent){SensorLift->AckRequested=true;SensorLift->TestAckSent=true;}
        if(SensorLift&&SensorLift->SawRecovery&&TimeS<NewGoalAt)LiftTargetM=SensorLift->TestRebasedTarget;
        if(TimeS>=NewGoalAt)LiftTargetM=.6;return;
    }
    LiftTargetM=TimeS<2?0:TimeS<20?.9:TimeS<40?.25:.6;
}
bool AStabilityLab::CheckSensorLiftTest() const
{
    if(!SensorLift)return false;const auto& R=*SensorLift;
    bool Pass=R.Model.Armed&&!R.Model.Faulted&&MaxTilt<5&&!CargoState.bFell&&!CargoState.bTipped&&MaxInnerClockErrorS<1e-7;
    if(TestMode.EndsWith(TEXT("pressure")))Pass&=R.SawPressure&&ForkHeightM<.06&&MaxForkHeight<.4;
    else Pass&=FMath::Abs(ForkHeightM-.6)<.025;
    if(TestMode.Contains(TEXT("fault")))Pass&=R.SawFault&&R.SawRecovery;
    if(TestMode.EndsWith(TEXT("watchdog")))Pass&=R.SawFault&&R.SawRecovery&&R.SawExpired;
    UE_LOG(LogTemp,Display,TEXT("SENSOR_LIFT_CHECK %s actual_m=%.6f target_m=%.6f fault=%d recovered=%d pressure_limit=%d watchdog=%d %s"),*TestMode,ForkHeightM,R.Model.TargetM,R.SawFault,R.SawRecovery,R.SawPressure,R.SawExpired,Pass?TEXT("PASS"):TEXT("FAIL"));return Pass;
}
