#include "HandlingControl.h"
#include "LidarSafety.h"
#include "PalletPerception.h"
#include "Misc/AutomationTest.h"
namespace {Handling::FInput Healthy(){Handling::FInput I;I.Now=2;I.Dt=.02;I.Measurements=I.Healthy=I.Camera=I.Geometry=I.Vision=I.HeelValid=I.Aligned=true;I.PerceptionStamp=2;I.Height=.035;return I;}}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandlingLiftGate,"ForkliftSim.Handling.FullInsertionBeforeLift",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FHandlingLiftGate::RunTest(const FString&)
{
    Handling::FModel M;auto I=Healthy();I.Request=Handling::ERequest::PickGround;M.Step(I);I.Request=Handling::ERequest::None;
    for(int K=0;K<80;++K){I.Now+=I.Dt;I.PerceptionStamp=I.Now;M.Step(I);}TestTrue(TEXT("Waits at insertion"),M.State==TEXT("insert"));
    I.Depth[0]=I.Depth[1]=1.08;for(bool& P:I.Presence)P=true;
    for(int K=0;K<30;++K){I.Now+=I.Dt;I.PerceptionStamp=I.Now;M.Step(I);}TestTrue(TEXT("Depth and presence alone never authorize lifting"),M.State==TEXT("insert")&&!M.FullSeen);
    I.Full=true;for(int K=0;K<30;++K){I.Now+=I.Dt;I.PerceptionStamp=I.Now;M.Step(I);}TestTrue(TEXT("Independent fused full evidence and settle authorize lift"),M.State==TEXT("pick_lift")&&M.FullSeen);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandlingFaults,"ForkliftSim.Handling.LatchedFailureAndExplicitRestart",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FHandlingFaults::RunTest(const FString&)
{
    for(int Kind=0;Kind<4;++Kind)
    {
        Handling::FModel M;auto I=Healthy();M.Start(I,Handling::ERequest::PickGround);M.State=TEXT("insert");
        if(Kind==0)I.Camera=false;if(Kind==1)I.Healthy=false;if(Kind==2)I.Obstacle=true;if(Kind==3)I.Manual=true;M.Step(I);
        TestTrue(TEXT("Camera/sensor/obstacle/manual failure latches stop"),M.Faulted&&M.Brake&&M.Velocity.IsNearlyZero()&&!M.Permit);
        I=Healthy();M.Step(I);TestTrue(TEXT("Healthy data does not auto-resume job"),M.Faulted&&!M.Active);
        I.Request=Handling::ERequest::PickGround;M.Step(I);TestTrue(TEXT("Fresh explicit request can restart from beginning"),M.Active&&!M.Faulted&&M.State==TEXT("pick_height"));
    }return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandlingPlaceGate,"ForkliftSim.Handling.UnloadRequiresMeasuredCarrier",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FHandlingPlaceGate::RunTest(const FString&)
{
    Handling::FModel M;M.Spec.RackX=3.2;M.Spec.RackY=0;auto I=Healthy();I.Request=Handling::ERequest::PlaceHigh;M.Step(I);TestFalse(TEXT("Empty forks cannot start place"),M.Active);
    I.Height=.25;I.Load[0]=I.Load[1]=1000;I.HeelDepth[0]=I.HeelDepth[1]=1.09;for(bool& P:I.Presence)P=true;M.Step(I);TestTrue(TEXT("Measured carrier at local surveyed approach accepted"),M.Active);
    I.Request=Handling::ERequest::None;M.State=TEXT("place_lower");I.Height=M.Tier+.035;
    for(int K=0;K<30;++K){I.Now+=I.Dt;M.Step(I);}TestTrue(TEXT("Height alone never authorizes withdrawal"),M.State==TEXT("place_lower"));
    I.Load[0]=I.Load[1]=0;for(int K=0;K<25;++K){I.Now+=I.Dt;M.Step(I);}TestTrue(TEXT("Measured load release and settle authorize withdrawal"),M.State==TEXT("place_withdraw")&&M.TransferSeen);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandlingPermission,"ForkliftSim.Handling.TargetPermissionIsSpatialAndRevocable",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FHandlingPermission::RunTest(const FString&)
{
    Handling::FModel M;M.Last=Healthy();M.Active=M.Permit=true;M.Last.Depth[0]=M.Last.Depth[1]=0;
    TestTrue(TEXT("Only expected measured pallet rectangle permitted"),M.TargetPoint(FVector2D(2.1,0)));TestFalse(TEXT("Side obstacle not exempt"),M.TargetPoint(FVector2D(1,.8)));TestFalse(TEXT("Rear obstacle not exempt"),M.TargetPoint(FVector2D(-1,0)));
    M.Last.Geometry=false;M.Receipt=true;M.Last.HeelValid=false;TestFalse(TEXT("Receipt alone cannot exempt beyond fixed carrier"),M.TargetPoint(FVector2D(2.1,0)));M.Last.HeelValid=true;M.Last.HeelDepth[0]=M.Last.HeelDepth[1]=1.13;TestTrue(TEXT("Fresh measured insertion bounds include actual pallet edge"),M.TargetPoint(FVector2D(2.1,0)));TestFalse(TEXT("Measured carrier does not exempt unrelated forward obstacle"),M.TargetPoint(FVector2D(2.5,0)));
    M.Last.Healthy=false;TestFalse(TEXT("Sensor invalidation revokes permission"),M.TargetPoint(FVector2D(2.1,0)));return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandlingShortGap,"ForkliftSim.Handling.TransientReadingStopsBeforeRecheck",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FHandlingShortGap::RunTest(const FString&)
{
 Handling::FModel M;auto I=Healthy();M.Start(I,Handling::ERequest::PickGround);M.State=TEXT("insert");I.Measurements=false;M.Step(I);TestTrue(TEXT("One optical missing reading holds immediately, no motion"),M.Brake&&M.Velocity.IsNearlyZero()&&M.Active&&M.Permit);I.Now+=.1;I.Measurements=true;M.Step(I);TestTrue(TEXT("Fresh recheck can continue within bounded gap"),M.Active&&!M.Faulted);I.Measurements=false;M.Step(I);I.Now+=.3;M.Step(I);TestTrue(TEXT("Persistent gap latches stop"),M.Faulted&&M.Brake);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandlingAnchorGate,"ForkliftSim.Handling.LockedShapeRejectsOtherBoard",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FHandlingAnchorGate::RunTest(const FString&)
{
 TestTrue(TEXT("Small measured repeat innovation accepted"),PalletPerception::ConsistentAnchor(FVector(2,.1,.13),FVector(2.005,.102,.135),0,.005));TestFalse(TEXT("Lower crossboard cannot replace opening roof"),PalletPerception::ConsistentAnchor(FVector(2,.1,.1),FVector(2,.1,.13),0,0));TestFalse(TEXT("Displaced object cannot refresh same static track"),PalletPerception::ConsistentAnchor(FVector(2,.2,.13),FVector(2,.1,.13),0,0));return true;
}
