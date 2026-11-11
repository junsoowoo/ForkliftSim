#include "SensorDrive.h"
#include "Misc/AutomationTest.h"
namespace
{
    SensorDrive::FInput Input(double T,double Rate=0){SensorDrive::FInput I;I.Now=T;I.Dt=.01;I.Valid=true;I.Gyro=.003;I.Neutral=Rate==0;I.ImuSequence=uint64(T*100+1);for(double& C:I.Capture)C=T-.02;for(double& R:I.Rate)R=Rate;return I;}
    void Calibrate(SensorDrive::FModel& M){for(int K=3;K<100;++K)M.Observe(Input(K*.01));}
    TArray<SensorDrive::FSegment> Map(){return {{{-4,-6},{14,-6}},{{14,-6},{14,6}},{{14,6},{-4,6}},{{-4,6},{-4,-6}}};}
    TArray<FVector2D> Points(SensorDrive::FPose Pose)
    {
        TArray<FVector2D> R;const auto Inv=SensorDrive::Inverse(Pose);for(int K=0;K<50;++K)for(FVector2D P:{FVector2D(-4,-5.+K*.2),FVector2D(-3+K*.2,-6),FVector2D(-3+K*.2,6)}){const auto Q=SensorDrive::Compose(Inv,{P.X,P.Y,0});R.Add(FVector2D(Q.X,Q.Y));}return R;
    }
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDriveOdometry,"ForkliftSim.SensorDrive.MeasuredOdometryAndTorque",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDriveOdometry::RunTest(const FString&)
{
    SensorDrive::FModel M;Calibrate(M);TestTrue(TEXT("Stationary sampled gyro calibrates"),M.Ready);TestTrue(TEXT("Measured bias estimated without configured true bias"),FMath::Abs(M.GyroBias-.003)<1e-6);
    for(int K=100;K<300;++K){M.Observe(Input(K*.01,4));M.Control(.01,FVector2D(.6,0),0,false,1,.6);}TestTrue(TEXT("Finite filtering lag and measured integration"),M.Odom.X>1.1&&M.Odom.X<1.21);TestTrue(TEXT("No gyro drift after calibration"),FMath::Abs(M.Odom.Yaw)<.001);for(double T:M.Torque)TestTrue(TEXT("Sensed rate drives finite torque request"),FMath::IsFinite(T)&&FMath::Abs(T)<=450);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDriveFault,"ForkliftSim.SensorDrive.FaultLeaseAndNeutralRecovery",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDriveFault::RunTest(const FString&)
{
    SensorDrive::FModel M;Calibrate(M);auto I=Input(1);I.Valid=false;M.Observe(I);TestTrue(TEXT("Sensor loss latches"),M.Faulted);for(int K=101;K<150;++K)M.Observe(Input(K*.01));TestTrue(TEXT("Healthy alone never auto-restarts"),M.Faulted);I=Input(1.5,1);I.Ack=true;M.Observe(I);TestTrue(TEXT("Held input rejects acknowledgement"),M.Faulted&&M.AckRejected);I=Input(1.51);I.Ack=true;M.Observe(I);TestTrue(TEXT("Explicit neutral acknowledgement recovers"),M.Ready);I=Input(1.52);I.Expired=true;M.Observe(I);TestTrue(TEXT("Actuator lease expiry latches"),M.Faulted);M.Control(.01,FVector2D(1,1),1,false,1,.6);for(double T:M.Torque)TestEqual(TEXT("Fault commands no motor torque"),T,0.);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDriveRegistration,"ForkliftSim.SensorDrive.SurveyedMapRegistration",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDriveRegistration::RunTest(const FString&)
{
    const SensorDrive::FPose Truth={1,.5,.1};const auto Match=SensorDrive::Match(Points(Truth),Map(),{1.15,.38,.135});TestTrue(TEXT("Three wall measured points correct starting error"),Match.Accepted);TestTrue(TEXT("Position recovered"),FVector2D(Match.Pose.X-Truth.X,Match.Pose.Y-Truth.Y).Size()<.005);TestTrue(TEXT("Heading recovered"),FMath::Abs(Match.Pose.Yaw-Truth.Yaw)<.001);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDriveDegeneracy,"ForkliftSim.SensorDrive.RejectUnobservableAndSparseMap",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDriveDegeneracy::RunTest(const FString&)
{
    TArray<SensorDrive::FSegment> Parallel={{{-10,2},{10,2}},{{-10,3},{10,3}},{{-10,4},{10,4}}};TArray<FVector2D> P;for(int K=0;K<100;++K)P.Add(FVector2D(-3+K*.05,2));TestFalse(TEXT("Single direction cannot observe along-wall translation"),SensorDrive::Match(P,Parallel,{}).Accepted);P.SetNum(10);TestFalse(TEXT("Sparse scans never report localization"),SensorDrive::Match(P,Map(),{}).Accepted);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDriveHistory,"ForkliftSim.SensorDrive.DelayedScanPoseHistory",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDriveHistory::RunTest(const FString&)
{
    SensorDrive::FModel M;M.History={{1,{0,0,3.13}},{2,{1,0,-3.13}}};SensorDrive::FPose P;TestTrue(TEXT("Delayed scan time interpolates"),M.At(1.5,P));TestTrue(TEXT("Translation and wrapped yaw interpolate correctly"),FMath::Abs(P.X-.5)<1e-9&&FMath::Abs(FMath::Abs(P.Yaw)-UE_DOUBLE_PI)<.001);TestFalse(TEXT("Unavailable old history rejected instead of present pose substitution"),M.At(.5,P));const auto I=SensorDrive::Compose(P,SensorDrive::Inverse(P));TestTrue(TEXT("SE2 inverse consistent with Unreal planar axes"),FVector2D(I.X,I.Y).Size()<1e-9&&FMath::Abs(I.Yaw)<1e-9);return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDriveGapRecovery,"ForkliftSim.SensorDrive.StationaryFreshMapFixResolvesObservationGap",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDriveGapRecovery::RunTest(const FString&)
{
 SensorDrive::FModel M;M.Map=Map();M.History={{1,{0,0,0}},{1.1,{0,0,0}}};M.UnobservedS=2;M.Velocity=FVector(.2,0,0);TestTrue(TEXT("Observable map can register moving scan"),M.Correct(Points({}),1,1.1));TestEqual(TEXT("Moving fix does not silently forgive missing motion"),M.UnobservedS,2.);M.Velocity=FVector::ZeroVector;TestFalse(TEXT("Missing scan cannot clear gap"),M.Correct({},1,1.1));TestEqual(TEXT("Gap remains without evidence"),M.UnobservedS,2.);TestTrue(TEXT("Fresh observable stationary fix accepted"),M.Correct(Points({}),1,1.1));TestEqual(TEXT("Stationary relocalization clears unresolved gap"),M.UnobservedS,0.);return true;
}
