#include "SensorDrive.h"
#include "StabilityLab.h"
#include "JointSensors.h"
#include "DynamicsSensors.h"
#include "LidarSafety.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"

bool AStabilityLab::IsSensorDriveTest() const{return TestMode.StartsWith(TEXT("mecanum_drive_sensor_"));}
void AStabilityLab::ClearNavigationWalls(){for(AStaticMeshActor* A:NavigationWalls)if(A)A->Destroy();NavigationWalls.Empty();}
void AStabilityLab::ControlSensorDriveTest()
{
    Throttle=Strafe=Steering=Lift=0;Brake=1;LiftTargetM=0;if(!DriveSensors)return;
    if(NavigationWalls.IsEmpty()&&TimeS<1&&!TestMode.EndsWith(TEXT("warehouse")))
    {
        auto Wall=[&](FVector P,FVector Size){auto* A=GetWorld()->SpawnActor<AStaticMeshActor>(P*100,FRotator::ZeroRotator);auto* B=A->GetStaticMeshComponent();B->SetMobility(EComponentMobility::Movable);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));B->SetWorldScale3D(Size);B->SetCollisionProfileName(TEXT("BlockAll"));NavigationWalls.Add(A);};
        // Inner faces agree with the independently authored surveyed fixture.
        Wall(FVector(-4.1,0,1),FVector(.2,12.4,2));Wall(FVector(14.1,0,1),FVector(.2,12.4,2));Wall(FVector(5,-6.1,1),FVector(18,.2,2));Wall(FVector(5,6.1,1),FVector(18,.2,2));
    }
    const bool Fault=TestMode.EndsWith(TEXT("encoder_fault"))||TestMode.EndsWith(TEXT("imu_fault"))||TestMode.EndsWith(TEXT("watchdog"));
    if(TestMode.EndsWith(TEXT("encoder_fault"))&&JointSensors)JointSensors->Model.TimedFaultMask=1;
    if(TestMode.EndsWith(TEXT("imu_fault"))&&DynamicsSensors)DynamicsSensors->Model.FaultMask=1;
    if(TestMode.EndsWith(TEXT("encoder_fault"))&&JointSensors){JointSensors->Model.FaultStartS=6;JointSensors->Model.FaultEndS=8;}
    if(TestMode.EndsWith(TEXT("slip")))
    {
        if(TimeS>=8&&!bFrictionSwitched){ChangeFriction(false,.01-Settings.RoadMu);bFrictionSwitched=true;}
        if(TimeS>=10&&!bFrictionRestored){ChangeFriction(false,.8-Settings.RoadMu);bFrictionRestored=true;}
    }
    if(TimeS>=2&&TimeS<(Fault?12:11))
    {
        Brake=0;
        if(TestMode.EndsWith(TEXT("strafe")))Strafe=.4;
        else {Throttle=TestMode.EndsWith(TEXT("slip"))&&TimeS>=8?.9:.5;if(TestMode.EndsWith(TEXT("curve")))Steering=.4;}
    }
    if(TimeS>=14&&TimeS<14.1){AcknowledgeSensorLift();}
    if(TimeS>=16&&TimeS<23){Brake=0;if(TestMode.EndsWith(TEXT("strafe")))Strafe=-.4;else Throttle=.4;}
}
bool AStabilityLab::CheckSensorDriveTest() const
{
    if(!DriveSensors||!VehicleGuard)return false;const auto& R=*DriveSensors;
    const bool Fault=TestMode.EndsWith(TEXT("fault"))||TestMode.EndsWith(TEXT("watchdog"));
    bool Pass=R.Model.Ready&&!VehicleGuard->Decision.Faulted&&R.Model.Localized&&TimeS-R.Model.LastMatchS<.75&&R.FinalErrorM<.15&&FVector2D(SpeedMps,LateralMps).Size()<.1&&MaxTilt<5&&MaxInnerClockErrorS<1e-7;
    if(Fault)Pass&=R.SawFault&&R.SawRecovery;
    UE_LOG(LogTemp,Display,TEXT("SENSOR_DRIVE_CHECK %s localized=%d map_error_m=%.6f fault_seen=%d recovery=%d speed=%.6f %s"),*TestMode,R.Model.Localized,R.FinalErrorM,R.SawFault,R.SawRecovery,FVector2D(SpeedMps,LateralMps).Size(),Pass?TEXT("PASS"):TEXT("FAIL"));return Pass;
}
