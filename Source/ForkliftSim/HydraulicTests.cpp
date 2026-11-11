#include "HydraulicLift.h"
#include "ChainConstitutive.h"
#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FChainPassivity,"ForkliftSim.Hydraulics.UnilateralChainEnergy",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FChainPassivity::RunTest(const FString&)
{
    const double K=1e6,D=6000;
    for(double X:{-.01,0.,.001,.01}) for(double V:{-3.,-.1,0.,.1,3.})
    {
        const auto R=ChainConstitutive::Evaluate(K,D,X,V);
        TestTrue(TEXT("Tension and dissipative power cannot be negative"),R.TensionN>=0 && R.LossW>=-1e-9);
        const double Eps=1e-7;
        const double DE=(ChainConstitutive::Evaluate(K,D,X+V*Eps,0).StoredJ-ChainConstitutive::Evaluate(K,D,X-V*Eps,0).StoredJ)/(2*Eps);
        TestTrue(TEXT("Independent stored energy derivative closes constitutive power"),FMath::Abs(DE+R.LossW-R.TensionN*V)<.3);
    }
    const auto Release=ChainConstitutive::Evaluate(K,D,.001,-3);
    TestTrue(TEXT("Zero-tension unloading dissipates released spring energy only"),Release.TensionN==0 && FMath::Abs(Release.LossW-3000)<1e-9);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHydraulicTest,"ForkliftSim.Hydraulics.PressureFlowPowerHolding",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FHydraulicTest::RunTest(const FString& Parameters)
{
    FHydraulicSettings C; TestTrue(TEXT("Default settings valid"),C.Valid());
    C.AreaM2=0; TestFalse(TEXT("Zero area rejected"),C.Valid()); C=FHydraulicSettings();
    C.Efficiency=1.1; TestFalse(TEXT("Efficiency above unity rejected"),C.Valid()); C=FHydraulicSettings();
    FHydraulicState S;
    for(int I=0;I<4800;++I)
    {
        S.Step(C,1./480,.9,0,0,1.5,15000);
        TestTrue(TEXT("Unilateral gauge pressure with relief limit"),S.PressurePa>=0 && S.PressurePa<=C.ReliefPa);
        TestTrue(TEXT("Pump input power cap"),S.DcW<=C.MotorW+1e-7);
        TestTrue(TEXT("Pump maximum flow"),S.PumpM3s<=C.PumpFlowM3s);
        TestTrue(TEXT("Nonnegative dissipative losses"),S.HeatW>=0);
    }
    TestTrue(TEXT("Blocked cylinder opens ideal relief"),S.ReliefM3s>0 && S.PressurePa==C.ReliefPa);
    S=FHydraulicState(); S.PressurePa=3000000; S.PreviousTarget=.9;
    S.Step(C,.01,.9,.9,0,1.5,15000);
    TestTrue(TEXT("Closed check valve holds pressure, pump off"),S.bHolding && S.PumpM3s==0 && S.PressurePa==3000000);
    C.LeakM3sPa=2e-12; S.Step(C,.01,.9,.85,0,1.5,15000);
    TestTrue(TEXT("Leak depressurizes without hidden re-level servo"),S.PressurePa<3000000 && S.bHolding && S.PumpM3s==0);
    C=FHydraulicSettings(); S=FHydraulicState(); S.PressurePa=3000000; S.PreviousTarget=.9;
    S.Step(C,.01,0,.9,0,1.5,15000);
    TestTrue(TEXT("Gravity return consumes no pump power"),S.ReturnM3s>0 && S.PumpM3s==0 && S.DcW==0 && S.PressurePa<3000000);
    S=FHydraulicState(); S.Step(C,.01,0,.5,10,1.5,15000);
    TestEqual(TEXT("No tensile fluid force / pressure floor"),S.PressurePa,0.);
    // Independent 1-DOF mass (not a substitute for whole-rig contact tests).
    auto Bench=[&](double Mass,double Watts,double Dt,double Leak)
    {
        FHydraulicSettings P; P.MotorW=Watts; P.LeakM3sPa=Leak;
        FHydraulicState H; H.PressurePa=Mass*9.81/P.AreaM2;
        double X=0,V=0,RateSum=0,Duration=0,Hold=0;
        for(int I=0;I<FMath::RoundToInt(20/Dt);++I)
        {
            H.Step(P,Dt,.9,X,V,1.5,15000);
            V+=(H.ForceN-Mass*9.81)/Mass*Dt; X+=V*Dt;
            if(X<0) { X=0; V=FMath::Max(0.,V); }
            if(X>.25 && X<.55 && V>0) { RateSum+=V*Dt; Duration+=Dt; }
            if(I*Dt<12) Hold=X;
        }
        TestTrue(TEXT("Mass bench remains finite and bounded"),FMath::IsFinite(X) && X>.7 && X<1.);
        return FVector(RateSum/FMath::Max(Duration,1e-8),Hold-X,H.DcJ);
    };
    const auto Empty=Bench(150,1200,1./480,0), Load=Bench(575,1200,1./480,0), Low=Bench(575,600,1./480,0), Leak=Bench(575,1200,1./480,2e-12);
    TestTrue(TEXT("Power-limited loaded rise slower"),Load.X<Empty.X*.9 && Low.X<Load.X*.8);
    TestTrue(TEXT("Closed valve vs leaking hold"),FMath::Abs(Load.Y)<.001 && Leak.Y>.01);
    const auto Coarse=Bench(575,1200,1./240,0);
    TestTrue(TEXT("1DOF speed convergence 240/480 Hz"),FMath::Abs(Coarse.X-Load.X)<.005);
    return true;
}
#endif
