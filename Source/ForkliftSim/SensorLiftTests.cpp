#include "SensorLiftControl.h"
#include "HydraulicCallback.h"
#include "Misc/AutomationTest.h"
namespace
{
    SensorLift::FInput Input(double Time,double Height=.3,double Target=.8)
    {SensorLift::FInput I;I.NowS=I.HeightCaptureS=I.LimitCaptureS=I.PressureCaptureS=Time;I.Dt=.02;I.HeightSequence=uint64(Time*1000)+1;I.HeightValid=I.LimitsValid=I.PressureValid=true;I.HeightM=Height;I.RequestedM=Target;I.PressurePa=2e6;return I;}
    void Arm(SensorLift::FModel& M,double Height=.3,double Target=.8){M.Reset(SensorLift::FSpec(),2.1);for(int I=0;I<=20;++I)M.Step(Input(I*.02,Height,Target));}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDirectValve,"ForkliftSim.SensorLift.DirectValveBypassAndLease",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDirectValve::RunTest(const FString&)
{
    FHydraulicSettings C;FHydraulicState A,B;FHydraulicCommand D;D.Direct=true;D.Spool=.4;
    TestEqual(TEXT("Direct valve ignores physical target and height in command generation"),A.ValveRequest(C,0,2,2.1,D),B.ValveRequest(C,2,0,2.1,D));
    TestEqual(TEXT("Direct spool is exactly the externally measured controller command"),A.ValveRequest(C,0,2,2.1,D),.4);
    A.PressurePa=2e6;A.Valve=.5;D.Spool=0;D.PumpEnabled=false;A.Step(C,.001,2,.4,0,2.1,15000,D);
    TestTrue(TEXT("Pump inhibit stops inlet flow without freezing physical valve response"),A.PumpM3s==0&&A.Valve>0&&A.Valve<.5);
    A.Valve=-.2;A.Step(C,.001,2,.4,-.01,2.1,15000,D);TestTrue(TEXT("Closing a lowering valve retains finite transient return flow"),A.ReturnM3s>0&&A.Valve<0);
    D.Spool=.5;D.PumpEnabled=true;TestEqual(TEXT("Valid command lease passes through"),HydraulicLease(D,1,.9).Spool,.5);const auto Expired=HydraulicLease(D,1,1.01);TestTrue(TEXT("Expired physical-thread command neutralizes valve and inhibits pump"),Expired.Spool==0&&!Expired.PumpEnabled);
    FHydraulicInput In;In.Command=D;In.Reset();TestFalse(TEXT("Reused callback input clears external command"),In.Command.Direct);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLiftMeasuredFeedback,"ForkliftSim.SensorLift.MeasuredFeedbackAndAntiWindup",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FLiftMeasuredFeedback::RunTest(const FString&)
{
    SensorLift::FModel A,B;Arm(A,.2,.6);Arm(B,.8,.6);TestTrue(TEXT("Identical requests produce opposite commands for opposite measured errors"),A.Command>0&&B.Command<0);
    for(int I=21;I<1000;++I)A.Step(Input(I*.02,.2,1.8));TestTrue(TEXT("Saturated controller does not wind up beyond configured integral or spool bounds"),FMath::Abs(A.Integral)<=A.Spec.IntegralCap&&A.Command<=A.Spec.UpCap);
    Arm(B,.3,.6);B.Holding=false;B.Velocity=.2;B.Command=0;auto Near=Input(.42,.3,.305);B.Step(Near);TestEqual(TEXT("Approaching from below may close but cannot open lowering flow before crossing the goal"),B.Command,0.);
    Arm(B,.3,0);B.Holding=false;B.Velocity=-.2;B.Command=0;Near=Input(.42,.3,.295);B.Step(Near);TestEqual(TEXT("Approaching from above may close but cannot pump upward before crossing the goal"),B.Command,0.);
    TestTrue(TEXT("Startup waits for stable fresh sensors"),A.Armed);SensorLift::FModel Wait;Wait.Reset(SensorLift::FSpec(),2.1);Wait.Step(Input(0));TestTrue(TEXT("No actuation before arming"),!Wait.Armed&&Wait.Command==0&&!Wait.PumpEnabled);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLiftFaultAck,"ForkliftSim.SensorLift.TimeoutLatchAndExplicitRecovery",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FLiftFaultAck::RunTest(const FString&)
{
    SensorLift::FModel M;Arm(M);auto I=Input(.5);I.HeightValid=false;M.Step(I);TestTrue(TEXT("Missing height latches stop with pump inhibited"),M.Faulted&&M.Command==0&&!M.PumpEnabled);
    I=Input(.6);I.Acknowledge=true;M.Step(I);TestTrue(TEXT("Premature recovery acknowledgement rejected"),M.Faulted&&M.AckRejected);
    for(int Step=31;Step<=50;++Step)M.Step(Input(Step*.02));TestTrue(TEXT("Fresh sensors cannot restart a latched controller"),M.Faulted&&M.Command==0);
    I=Input(1.02);I.Acknowledge=true;M.Step(I);TestTrue(TEXT("Explicit healthy recovery captures current measured height instead of old goal"),!M.Faulted&&M.Rebased&&FMath::Abs(M.TargetM-.3)<1e-9&&M.Command==0);
    M.Step(Input(1.04,.3,.3));TestEqual(TEXT("Captured target stays stopped"),M.Command,0.);
    I=Input(1.06,.3,.6);I.ActuatorExpired=true;M.Step(I);TestTrue(TEXT("Actuator watchdog is latched too"),M.Faulted&&M.Reason==TEXT("actuator_command_expired"));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLiftDirectionalInterlocks,"ForkliftSim.SensorLift.LimitsAndPressureDirections",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FLiftDirectionalInterlocks::RunTest(const FString&)
{
    SensorLift::FModel M;Arm(M,.8,1);auto I=Input(.5,.8,1);I.Upper=true;M.Step(I);TestTrue(TEXT("Upper limit inhibits upward command and pump"),M.Command==0&&!M.PumpEnabled);
    I=Input(.52,.8,.5);I.Upper=true;M.Step(I);TestTrue(TEXT("Upper limit still permits deliberate lowering"),M.Command<0);
    Arm(M,.2,0);I=Input(.5,.2,0);I.Lower=true;M.Step(I);TestEqual(TEXT("Lower limit blocks downward command"),M.Command,0.);I=Input(.52,.2,.6);I.Lower=true;M.Step(I);TestTrue(TEXT("Lower limit permits raising"),M.Command>0);
    Arm(M,.8,1);for(int Step=25;Step<=40;++Step){I=Input(Step*.02,.8,1);I.PressurePa=7e6;M.Step(I);}TestTrue(TEXT("Pressure inhibits raising without silently commanding a descent"),M.PressureLimited&&M.Command==0&&!M.PumpEnabled&&!M.Faulted);
    I=Input(.82,.8,.5);I.PressurePa=7e6;M.Step(I);TestTrue(TEXT("Operator-requested lowering remains possible under high pressure"),M.Command<0&&!M.PumpEnabled);
    I=Input(.84);I.Lower=I.Upper=true;M.Step(I);TestTrue(TEXT("Contradictory limit states latch a fault"),M.Faulted);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLiftInvalidInput,"ForkliftSim.SensorLift.InvalidClockJumpAndConfiguration",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FLiftInvalidInput::RunTest(const FString&)
{
    SensorLift::FModel M;Arm(M);M.Step(Input(.42,.8));TestTrue(TEXT("Implausible measured height jump cannot command the valve"),M.Faulted&&M.Reason==TEXT("height_jump_or_clock")&&M.Command==0);
    Arm(M);auto I=Input(.42);I.HeightCaptureS=1;M.Step(I);TestTrue(TEXT("Future timestamp is unusable"),M.Faulted&&M.Command==0);
    SensorLift::FSpec S;TestTrue(TEXT("Default controller spec valid"),S.Valid(2.1));S.PressureReleasePa=S.PressureTripPa;TestFalse(TEXT("Pressure hysteresis must have a valid order"),S.Valid(2.1));S=SensorLift::FSpec();S.LeaseS=0;TestFalse(TEXT("Zero lease rejected"),S.Valid(2.1));return true;
}
