#include "MissionNavigation.h"
#include "Misc/AutomationTest.h"
namespace{Mission::FInput Ready(){Mission::FInput I;I.Now=2;I.Dt=.02;I.Healthy=I.Localized=I.Camera=true;I.HandlingSequence=7;return I;}}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionPlannerFootprint,"ForkliftSim.Mission.AsymmetricFootprintAndDetour",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMissionPlannerFootprint::RunTest(const FString&)
{
    Mission::FPlanner P;P.Map.Add({FVector2D(3,-.5),FVector2D(3,.5)});TestTrue(TEXT("Start full footprint clear"),P.Free(FVector2D(0,0)));TestFalse(TEXT("Fork front, not point robot, blocks near wall"),P.Free(FVector2D(1,0)));const auto R=P.Plan(FVector2D(0,0),FVector2D(5,0));TestTrue(TEXT("A star finds non-straight route"),R.Valid&&R.Points.Num()>2&&R.Length>5.2);for(int K=0;K+1<R.Points.Num();++K)TestTrue(TEXT("Every simplified segment has full swept footprint clearance"),P.Clear(R.Points[K],R.Points[K+1]));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionBlockedGoal,"ForkliftSim.Mission.NoRouteAndDynamicGoalBlock",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMissionBlockedGoal::RunTest(const FString&)
{
    Mission::FPlanner P;P.Obstacles.Add(FVector2D(6,0));TestFalse(TEXT("Dynamic point inside goal's fork footprint rejects goal"),P.Plan(FVector2D(0,0),FVector2D(5,0)).Valid);P.Obstacles.Empty();P.Map.Add({FVector2D(3,-6),FVector2D(3,8)});TestFalse(TEXT("No route through a complete wall"),P.Plan(FVector2D(0,0),FVector2D(5,0)).Valid);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionToken,"ForkliftSim.Mission.NewHandlingReceiptRequired",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMissionToken::RunTest(const FString&)
{
    Mission::FModel M;auto I=Ready();I.Request=Mission::ERequest::StartLow;M.Step(I);I.Request=Mission::ERequest::None;for(int K=0;K<24;++K){I.Now+=I.Dt;M.Step(I);}TestTrue(TEXT("New pickup request issued"),M.State==TEXT("pickup"));I.HandlingState=TEXT("complete");I.HandlingFull=I.HandlingTransfer=true;M.Step(I);TestFalse(TEXT("Old complete receipt cannot skip pickup"),M.Carrying);I.HandlingSequence=8;I.Load[0]=I.Load[1]=2000;M.Step(I);TestTrue(TEXT("Fresh full and load receipt enables transport"),M.Carrying&&M.State==TEXT("to_dropoff"));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionResume,"ForkliftSim.Mission.ResumeRevalidatesAndReplans",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMissionResume::RunTest(const FString&)
{
    Mission::FModel M;auto I=Ready();M.Engaged=M.Carrying=true;M.ReferenceLoad=2000;M.State=TEXT("to_dropoff");I.Height=.25;I.Load[0]=I.Load[1]=1000;I.Request=Mission::ERequest::Pause;M.Step(I);TestTrue(TEXT("Pause zeroes commands and keeps checkpoint"),M.Holding()&&M.Checkpoint==TEXT("to_dropoff")&&M.Velocity.IsNearlyZero());I.Request=Mission::ERequest::None;M.Step(I);TestTrue(TEXT("Fresh data alone never resumes"),M.Holding());I.Request=Mission::ERequest::Resume;I.Localized=false;M.Step(I);TestTrue(TEXT("Stale localization rejects resume"),M.Holding());I.Localized=true;M.Step(I);TestTrue(TEXT("Explicit healthy resume invalidates old route"),M.Navigating()&&!M.Route.Valid);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionPartialRecovery,"ForkliftSim.Mission.PartialInsertionCannotBlindlyResume",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMissionPartialRecovery::RunTest(const FString&)
{
    Mission::FModel M;auto I=Ready();M.Engaged=true;M.State=TEXT("pickup");I.HandlingActive=true;I.Request=Mission::ERequest::Pause;M.Step(I);TestTrue(TEXT("Handling pause cancels local actuator owner"),M.CancelHandling&&M.Holding());I.HandlingActive=false;I.HandlingFault=true;I.HandlingFailedStage=TEXT("insert");I.Request=Mission::ERequest::Resume;M.Step(I);TestTrue(TEXT("Partial insertion requires manual recovery"),M.Holding()&&M.Reason==TEXT("partial_insertion_or_transfer_requires_manual_recovery"));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionRetry,"ForkliftSim.Mission.PreInsertionRetryBounded",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMissionRetry::RunTest(const FString&)
{
    Mission::FModel M;auto I=Ready();M.Engaged=true;M.State=TEXT("pickup");M.HandlingBaseline=7;I.HandlingSequence=8;I.HandlingFault=true;I.HandlingFailedStage=TEXT("acquire");I.HandlingReason=TEXT("stage_or_job_timeout");M.Step(I);TestTrue(TEXT("Empty pre-insertion recognition failure gets one retry"),M.Retries==1&&M.State==TEXT("retry_pickup"));I.Now+=1.1;M.Step(I);I.Now+=.02;M.Step(I);I.HandlingSequence=9;I.Now+=.02;M.Step(I);TestTrue(TEXT("Second failure latches, no endless restart"),M.Holding()&&M.Retries==1);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionShield,"ForkliftSim.Mission.DirectionSweepNeverMutesFullFootprint",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMissionShield::RunTest(const FString&)
{
    Mission::FModel M;M.Velocity=FVector2D(.25,0);TestTrue(TEXT("Forward braking sweep protects upcoming point"),M.GuardPoint(FVector2D(2.6,0)));M.Velocity=FVector2D(-.25,0);TestFalse(TEXT("Moving away does not retain irrelevant forward padding"),M.GuardPoint(FVector2D(2.6,0)));for(auto V:{FVector2D(.2,0),FVector2D(-.2,0),FVector2D(0,.2),FVector2D(0,-.2)}){M.Velocity=V;TestTrue(TEXT("Actual fork region remains protected in every direction"),M.GuardPoint(FVector2D(2.1,.6)));}return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionCarrierFilter,"ForkliftSim.Mission.CarrierFilterRequiresCurrentEvidence",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMissionCarrierFilter::RunTest(const FString&)
{
 FMissionRig R;R.Model.Carrying=R.Model.SawPickup=true;R.Model.ReferenceLoad=4000;R.Model.Last.Healthy=R.Model.Last.HandlingFull=true;R.Model.Last.Height=.25;R.Model.Last.Load[0]=R.Model.Last.Load[1]=2000;R.CarrierUntilS=3;TestTrue(TEXT("Confirmed carried geometry is self, not an obstacle"),R.CarriedPoint(FVector2D(1.4,0),2));TestFalse(TEXT("Outside carrier remains protected"),R.CarriedPoint(FVector2D(1.4,-1),2));TestFalse(TEXT("Stale geometry revokes self filter"),R.CarriedPoint(FVector2D(1.4,0),4));R.Model.Last.Load[0]=0;TestFalse(TEXT("Lost support revokes self filter"),R.CarriedPoint(FVector2D(1.4,0),2));return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionGridEndpoint,"ForkliftSim.Mission.ContinuousGoalCanUseAdjacentFreeCell",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMissionGridEndpoint::RunTest(const FString&)
{
 Mission::FPlanner P;P.Obstacles.Add(FVector2D(4,-1.45));const FVector2D Goal(3.24,-2.5);TestTrue(TEXT("Continuous goal is clear"),P.Free(Goal));const auto R=P.Plan(FVector2D(0,0),Goal);TestTrue(TEXT("Blocked nearest grid center does not falsely reject reachable goal"),R.Valid);if(R.Valid)TestTrue(TEXT("Final segment reaches actual surveyed goal"),P.Clear(R.Points[R.Points.Num()-2],Goal));return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMissionAtPickup,"ForkliftSim.Mission.AlreadyAtPickupHandsOffWithoutNavigation",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FMissionAtPickup::RunTest(const FString&)
{
 Mission::FModel M;auto I=Ready();M.Planner.Obstacles.Add(FVector2D(2.15,0));I.Request=Mission::ERequest::StartLow;M.Step(I);TestTrue(TEXT("Stationary known pickup bay directly requests local handling"),M.State==TEXT("request_pickup")&&!M.Navigating()&&M.Brake);I.Request=Mission::ERequest::None;M.Step(I);TestTrue(TEXT("Dock target in ordinary padding does not cause a navigation deadlock"),M.HandlingRequest==Handling::ERequest::PickGround&&M.State==TEXT("pickup"));return true;
}
