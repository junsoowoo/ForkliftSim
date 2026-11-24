#include "CampusFleet.h"
#include "HandlingControl.h"
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCampusTrafficAtomic,"Forklift.CampusFleet.AtomicCrossingAndOpposingRoutes",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCampusTrafficAtomic::RunTest(const FString&)
{
 CampusFleet::FTraffic T;TestTrue(TEXT("First route"),T.Acquire(TEXT("A"),{FVector2D(0,0),FVector2D(10,0)},0));TestFalse(TEXT("Opposing denied"),T.Acquire(TEXT("B"),{FVector2D(10,0),FVector2D(0,0)},0));TestFalse(TEXT("Crossing denied"),T.Acquire(TEXT("B"),{FVector2D(5,-5),FVector2D(5,5)},0));TestEqual(TEXT("No partial acquisitions"),T.Claims.Num(),1);TestTrue(TEXT("Disjoint allowed"),T.Acquire(TEXT("C"),{FVector2D(0,10),FVector2D(10,10)},0));T.Audit(1);TestTrue(TEXT("Unknown occupied route quarantines"),T.Claims[0].Quarantined);T.Release(TEXT("A"),false);TestEqual(TEXT("No unsafe automatic expiry"),T.Claims.Num(),2);T.Release(TEXT("A"),true);TestTrue(TEXT("Confirmed route release permits next"),T.Acquire(TEXT("B"),{FVector2D(10,0),FVector2D(0,0)},1));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCampusGraphSweep,"Forklift.CampusFleet.SurveyedGraphFullFootprint",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCampusGraphSweep::RunTest(const FString&)
{
 CampusFleet::FGraph G;G.Clearance.Spec.Bounds=FBox2D(FVector2D(-10,-10),FVector2D(20,20));G.Nodes={{TEXT("a"),FVector2D(0,0)},{TEXT("b"),FVector2D(0,5)},{TEXT("c"),FVector2D(8,5)},{TEXT("d"),FVector2D(8,0)}};G.Edges={{0,1,4},{1,2,4},{2,3,4}};G.Clearance.Map.Add({FVector2D(4,-3),FVector2D(4,2)});auto P=G.Plan(FVector2D(0,0),FVector2D(8,0));TestTrue(TEXT("Finds authored detour around wall"),P.Valid&&P.Length>9);for(int K=1;K<P.Points.Num();++K)TestTrue(TEXT("Every edge swept footprint clear"),G.Clearance.Clear(P.Points[K-1],P.Points[K]));TestFalse(TEXT("Fork endpoint blocked"),G.Plan(FVector2D(0,0),FVector2D(3,0)).Valid);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCampusFrameRoundtrip,"Forklift.CampusFleet.SurveyFrameAndLatchedStop",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCampusFrameRoundtrip::RunTest(const FString&)
{
 const SensorDrive::FPose Station{28.25,11.27,PI/2},Body{28.25,8,PI/2};auto P=SensorDrive::Compose(SensorDrive::Inverse(Station),Body);TestTrue(TEXT("North approach maps to local negative x"),FMath::Abs(P.X+3.27)<1e-6&&FMath::Abs(P.Y)<1e-6&&FMath::Abs(P.Yaw)<1e-6);auto W=SensorDrive::Compose(Station,P);TestTrue(TEXT("Survey transform inverse"),FMath::Abs(W.X-Body.X)<1e-6&&FMath::Abs(W.Y-Body.Y)<1e-6);CampusFleet::FJob J;J.State=TEXT("to_dropoff");J.Velocity=FVector2D(.1,0);J.Hold(TEXT("sensor_stale"),true);TestTrue(TEXT("Fault latched with checkpoint and zero command"),J.Held()&&J.ResumeState==TEXT("to_dropoff")&&J.Velocity.IsZero()&&J.Brake);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCampusDockBeam,"Forklift.CampusFleet.RaisedForkBeamKeepsWheelAndColumnProtection",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCampusDockBeam::RunTest(const FString&)
{
 Handling::FModel M;M.Active=M.Permit=true;M.Place=true;M.Last.Healthy=true;M.Spec.SurveyRack=true;M.Spec.BeamLeft=-.75;M.Spec.BeamRight=2.25;M.Last.Height=.5;M.Last.Pose={-2.8,0,0};
 TestTrue(TEXT("Surveyed low cross-beam below raised forks"),M.TargetPoint(FVector2D(2.13,0)));
 TestFalse(TEXT("Upright stays protected"),M.TargetPoint(FVector2D(2.13,-.7)));
 TestFalse(TEXT("Upright face with 25 mm sensor allowance stays protected"),M.TargetPoint(FVector2D(2.13,-.666)));
 TestTrue(TEXT("Free beam end outside upright allowance remains associated"),M.TargetPoint(FVector2D(2.13,-.66)));
 TestFalse(TEXT("Unknown geometry outside beam plane stays protected"),M.TargetPoint(FVector2D(1.6,0)));
 M.Last.Height=.35;TestFalse(TEXT("Low fork cannot ignore beam"),M.TargetPoint(FVector2D(2.13,0)));
 M.Last.Height=.5;M.Last.Pose.X=-1.27;TestFalse(TEXT("Wheel stopping region cannot ignore beam"),M.TargetPoint(FVector2D(.60,0)));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCampusGroundSpeed,"Forklift.CampusFleet.ScanVelocityFrameAndFreshness",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCampusGroundSpeed::RunTest(const FString&)
{
 FSensorDriveRig R;R.Model.Ready=true;R.Model.MapToOdom={0,0,FMath::DegreesToRadians(90.)};R.Model.Velocity=FVector(.2,0,.1);R.GroundValid=true;R.GroundStamp=5;R.GroundVelocityWorld=FVector2D(1,0);auto V=R.MeasuredVelocity(5.1);TestTrue(TEXT("Measured world speed transforms to body sideways speed"),FMath::Abs(V.X)<1e-6&&FMath::Abs(V.Y+1)<1e-6);TestTrue(TEXT("Gyro rate retained"),FMath::Abs(V.Z-.1)<1e-6);TestFalse(TEXT("Old scan velocity cannot certify stationary"),R.GroundFresh(5.5));return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCampusDockStoppingDistance,"Forklift.CampusFleet.DockingStopDistanceZeroAndDirection",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCampusDockStoppingDistance::RunTest(const FString&)
{
 TestTrue(TEXT("No directional jump at rest"),CampusFleet::DockingStopOffset(FVector2D::ZeroVector).IsZero());
 TestTrue(TEXT("Microscopic velocity cannot add centimetres to the field"),CampusFleet::DockingStopOffset(FVector2D(0,1e-6)).Size()<1e-6);
 const auto D=CampusFleet::DockingStopOffset(FVector2D(.12,0));TestTrue(TEXT("0.3 s travel plus braking at 0.5 m/s2"),FMath::Abs(D.X-(.12*.3+.12*.12/(2*.5)))<1e-9&&D.Y==0);
 const auto R=CampusFleet::DockingStopOffset(FVector2D(-.12,0));TestTrue(TEXT("Reverse extends the rear only"),R.X<0&&R.Y==0);
 TestTrue(TEXT("Higher speed requires a longer swept stop"),CampusFleet::DockingStopOffset(FVector2D(.2,0)).Size()>D.Size());return true;
}
