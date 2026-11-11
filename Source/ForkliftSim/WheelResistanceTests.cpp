#include "WheelResistance.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelResistanceTest,"ForkliftSim.WheelResistance.PassivityAndUnits",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FWheelResistanceTest::RunTest(const FString& Parameters)
{
    for(double W:{-20.,-1.,-.01,0.,.01,1.,20.})
    for(double Relative:{-20.,-.01,0.,.01,20.})
    for(double N:{0.,5000.,10000.})
    {
        const auto T=WheelResistance::Evaluate(N,.3,W,Relative,.015,1,.05);
        TestTrue(TEXT("Rolling contact never adds power"),T.Rolling*W<=0);
        TestTrue(TEXT("Bearing pair never adds net power"),T.Bearing*Relative<=0);
        if(N==0) TestEqual(TEXT("Airborne wheel has zero rolling resistance"),T.Rolling,0.);
    }
    const auto T=WheelResistance::Evaluate(5000,.3,10,10,.015,1,.05);
    TestEqual(TEXT("Crr*N*R = 22.5 Nm"),T.Rolling,-22.5);
    TestEqual(TEXT("Tc + B*omega = 1.5 Nm"),T.Bearing,-1.5);
    const auto Off=WheelResistance::Evaluate(5000,.3,10,10,0,0,0);
    TestEqual(TEXT("All coefficients off"),Off.Rolling+Off.Bearing,0.);
    const auto Reverse=WheelResistance::Evaluate(5000,.3,-10,-10,.015,1,.05);
    TestEqual(TEXT("Reverse rotation reverses resistance"),Reverse.Rolling+Reverse.Bearing,-T.Rolling-T.Bearing);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelBrakeFrameTest,"ForkliftSim.WheelResistance.BrakeRestFrame",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FWheelBrakeFrameTest::RunTest(const FString&)
{
    const FQuat Parent=FRotator(17,29,-43).Quaternion();
    for(double Angle:{-179.,-75.,-2.,0.,2.,75.,179.})
    {
        const FQuat Child=Parent*FQuat(FVector::ForwardVector,FMath::DegreesToRadians(Angle));
        const FQuat Relative=Parent.Inverse()*Child;
        const double Measured=FMath::RadiansToDegrees(Relative.GetTwistAngle(FVector::ForwardVector));
        const FQuat Target=WheelResistance::TwistTarget(Measured);
        TestTrue(TEXT("Engaging brake captures the existing relative pose with zero spring error"),Target.AngularDistance(Relative)<1e-7);
    }
    TestTrue(TEXT("UE Roll has the opposite sign to quaternion twist"),FMath::Abs(FRotator(0,0,37.).Quaternion().GetTwistAngle(FVector::ForwardVector)+FMath::DegreesToRadians(37.))<1e-12);
    return true;
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FResistanceFiniteStepTest,"ForkliftSim.WheelResistance.FiniteStepEnergy",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FResistanceFiniteStepTest::RunTest(const FString&)
{
    constexpr double Inertia=.479222168;
    const double RawAfter=.1-1000./960/Inertia;
    TestTrue(TEXT("Regression: negative instantaneous power can still create finite-step energy"),.5*Inertia*RawAfter*RawAfter>.5*Inertia*.1*.1);
    for(double W:{-200.,-1.,-.01,0.,.01,1.,200.})for(double H:{1./480,1./960,1./1920})for(double Magnitude:{.01,1.,1000.,10000000.})
    {
        const double Raw=-FMath::Sign(W)*Magnitude,T=WheelResistance::PassiveStepTorque(Raw,W,1./Inertia,H),After=W+T*H/Inertia;
        TestTrue(TEXT("Actual finite-step kinetic energy cannot increase"),After*After<=W*W+1e-9);
        TestTrue(TEXT("Resisting impulse cannot reverse rotation"),After*W>=-1e-9);
        if(FMath::Abs(Raw)<.8*FMath::Abs(W)*Inertia/H)TestTrue(TEXT("Unsaturated physical resistance is retained"),FMath::Abs(T-Raw)<1e-9);
    }
    TestEqual(TEXT("An active torque is not accepted as passive resistance"),WheelResistance::PassiveStepTorque(1.,1.,1.,.01),0.);
    return true;
}
#endif
