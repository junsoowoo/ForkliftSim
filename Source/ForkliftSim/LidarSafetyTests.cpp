#include "LidarSafety.h"
#include "Misc/AutomationTest.h"
namespace
{
    void Scan(LidarSafety::FSensor& S,double T,FVector Point=FVector(100,100,0))
    {
        for(int I=0;I<2;++I){TArray<double> R;R.Init(0,S.Spec.Rays);TArray<LidarSafety::EReturn> V;V.Init(LidarSafety::EReturn::NoReturn,S.Spec.Rays);if(Point.X<50){const FVector P=Point-S.Spec.Mount(I);double A=FMath::RadiansToDegrees(FMath::Atan2(P.Y,P.X));if(I&&A<0)A+=360;const int K=FMath::RoundToInt((A-(I?0:-180))*((S.Spec.Rays-1)/270.));if(K>=0&&K<S.Spec.Rays){R[K]=P.Size2D();V[K]=LidarSafety::EReturn::Valid;}}S.Capture(I,T,R,V);}S.Deliver(T);
    }
    void Ready(LidarSafety::FSensor& S,LidarSafety::FDecision& D){auto Spec=LidarSafety::FSpec();Spec.PacketDrop=Spec.SigmaM=Spec.LatencyS=0;S.Reset(Spec);D=LidarSafety::FDecision();for(int K=0;K<=10;++K){Scan(S,K*.1);D.Step(S,K*.1,true,false,true,false,false);}}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLidarCoverage,"ForkliftSim.LidarSafety.DualCoverageAndConfiguration",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FLidarCoverage::RunTest(const FString&)
{
    LidarSafety::FSpec S;TestTrue(TEXT("Default specification valid"),S.Valid());
    for(int A=0;A<360;++A){const FVector P=FVector(FMath::Cos(FMath::DegreesToRadians(double(A))),FMath::Sin(FMath::DegreesToRadians(double(A))),0)*4;bool Seen=false;for(int I=0;I<2;++I){const FVector D=P-S.Mount(I);double Angle=FMath::RadiansToDegrees(FMath::Atan2(D.Y,D.X));if(I&&Angle<0)Angle+=360;const int K=FMath::RoundToInt((Angle-(I?0:-180))*(S.Rays-1)/270.);Seen|=K>=0&&K<S.Rays&&!S.Masked(I,FMath::Clamp(K,0,S.Rays-1));}TestTrue(TEXT("Outer circle has at least one unmasked calibrated scanner direction"),Seen);}
    S.MountY=S.BodyY;TestFalse(TEXT("Scanner must be outside the calibrated self footprint"),S.Valid());return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLidarDirections,"ForkliftSim.LidarSafety.ProtectiveFieldAllDirections",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FLidarDirections::RunTest(const FString&)
{
    for(FVector P:{FVector(2.4,0,0),FVector(-1.1,0,0),FVector(0,1,0),FVector(0,-1,0),FVector(2.3,1,0)}){LidarSafety::FSensor S;LidarSafety::FDecision D;Ready(S,D);TestFalse(TEXT("Startup arms without an extra acknowledgement"),D.DriveHold);Scan(S,1.1,P);D.Step(S,1.1,true,false,false,false,false);TestTrue(TEXT("Obstacle holds traction but permits stationary vertical handling"),D.DriveHold&&!D.LiftHold&&!D.Faulted&&D.Points>0);Scan(S,1.2);D.Step(S,1.2,true,false,false,false,true);TestTrue(TEXT("Held drive input cannot release the hold"),D.DriveHold&&D.AckRejected);for(int K=13;K<=18;++K){Scan(S,K*.1);D.Step(S,K*.1,true,false,true,false,false);}TestTrue(TEXT("Clear scans alone never restart after an obstacle"),D.DriveHold);Scan(S,1.9);D.Step(S,1.9,true,false,true,false,true);TestFalse(TEXT("Stable clear and neutral explicit acknowledgement release traction"),D.DriveHold);}
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLidarFault,"ForkliftSim.LidarSafety.ScanLossAndEmergencyLatch",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FLidarFault::RunTest(const FString&)
{
    LidarSafety::FSensor S;LidarSafety::FDecision D;Ready(S,D);D.Step(S,1.5,true,false,false,false,false);TestTrue(TEXT("Stale scan holds drive and lift"),D.Faulted&&D.DriveHold&&D.LiftHold);
    for(int K=16;K<=21;++K){Scan(S,K*.1);D.Step(S,K*.1,true,false,true,false,false);}TestTrue(TEXT("Healthy data does not auto clear a fault"),D.Faulted);Scan(S,2.2);D.Step(S,2.2,true,false,true,false,true);TestTrue(TEXT("Acknowledgement clears healthy fault"),!D.Faulted&&!D.DriveHold&&!D.LiftHold);
    D.Step(S,2.21,true,false,false,true,false);TestTrue(TEXT("Emergency input latches whole vehicle stop"),D.Faulted&&D.Emergency&&D.LiftHold);Scan(S,2.3);D.Step(S,2.3,true,false,false,false,true);TestTrue(TEXT("Emergency cannot clear with held drive input"),D.Faulted);Scan(S,2.4);D.Step(S,2.4,true,false,true,false,true);TestFalse(TEXT("Healthy neutral acknowledgement releases simulated emergency"),D.Faulted);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLidarNoise,"ForkliftSim.LidarSafety.MeasurementNoiseBlindZoneAndDelay",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FLidarNoise::RunTest(const FString&)
{
    LidarSafety::FSpec P;P.PacketDrop=0;LidarSafety::FSensor S;S.Reset(P);TArray<double> R;R.Init(1.234,P.Rays);TArray<LidarSafety::EReturn> V;V.Init(LidarSafety::EReturn::Valid,P.Rays);R[0]=.01;S.Capture(0,0,R,V);S.Deliver(.049);TestFalse(TEXT("No scan before configured latency"),S.Channels[0].Have);S.Deliver(.05);TestTrue(TEXT("Too-close return has its own status"),S.Channels[0].Last.States[0]==LidarSafety::EReturn::TooClose);
    for(int K=1;K<P.Rays;++K){const double X=S.Channels[0].Last.Ranges[K];TestTrue(TEXT("Independent six-sigma plus quantization error envelope"),FMath::Abs(X-1.234)<=6*P.SigmaM+.5*P.QuantumM);}
    S.ManualOutage=true;S.Capture(0,.1,R,V);TestEqual(TEXT("Outage counts actual lost scans"),S.Channels[0].Dropped,uint64(1));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLidarManualCreep,"ForkliftSim.LidarSafety.ManualHandlingNeverBypassesFault",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FLidarManualCreep::RunTest(const FString&)
{
    LidarSafety::FSensor S;LidarSafety::FDecision D;Ready(S,D);Scan(S,1.1,FVector(2.4,0,0));D.Step(S,1.1,true,false,false,false,false,true);TestTrue(TEXT("Explicit manual handling permits traction despite a measured obstacle"),!D.DriveHold&&D.State==TEXT("manual_creep"));
    Scan(S,1.2,FVector(2.4,0,0));D.Step(S,1.2,true,false,false,false,false,false);TestTrue(TEXT("Releasing manual handling re-applies obstacle stop"),D.DriveHold);
    D.Step(S,1.21,true,false,false,true,false,true);TestTrue(TEXT("Manual handling cannot override emergency stop"),D.DriveHold&&D.LiftHold&&D.Faulted);return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLidarConcurrentEmergency,"ForkliftSim.LidarSafety.EmergencyWinsConcurrentAcknowledge",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FLidarConcurrentEmergency::RunTest(const FString&)
{
 LidarSafety::FSensor S;LidarSafety::FDecision D;Ready(S,D);Scan(S,1.1);D.Step(S,1.1,true,false,true,true,true);TestTrue(TEXT("Concurrent acknowledgement never clears asserted emergency"),D.Faulted&&D.Emergency&&D.DriveHold&&D.LiftHold);return true;
}
