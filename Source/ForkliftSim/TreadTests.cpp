#include "TreadModel.h"
#include "ContactLoadSampler.h"
#include "Misc/AutomationTest.h"
#include "Chaos/Evolution/IterationSettings.h"
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTreadFoundationTest,"ForkliftSim.Tread.AreaScaledFoundation",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FTreadFoundationTest::RunTest(const FString&)
{
    FTreadSettings S; S.bEnabled=true; const double R=.15,W=.1,D=.008,V=.02;
    for(int Count:{8,16,32})
    {
        S.Count=Count; const double K=S.PadK(R,W),C=S.PadC(R,W),H=1e-7;
        TestTrue(TEXT("Circumference-integrated stiffness independent of discretization"),FMath::Abs(K*Count-S.FoundationNpm3*2*UE_DOUBLE_PI*R*W)<1e-7);
        TestTrue(TEXT("Damping scales with reference pad area"),FMath::Abs(C*Count-S.FoundationDampingNspm3*2*UE_DOUBLE_PI*R*W)<1e-7);
        const double Derivative=(.5*K*FMath::Square(D+H)-.5*K*FMath::Square(D-H))/(2*H);
        TestTrue(TEXT("Spring force equals stored-energy derivative"),FMath::Abs(Derivative-K*D)<1e-5);
        TestTrue(TEXT("Viscous damping never creates energy"),C*V*V>=0);
        TestTrue(TEXT("Deeper prescribed indentation increases support"),S.FlatSupportN(R,W,.14)>S.FlatSupportN(R,W,.145));
        TestTrue(TEXT("No contact means no force"),S.FlatSupportN(R,W,.151)==0);
    }
    FContactLoadInput I; I.WheelIds[0]=10; I.TreadCounts[0]=2; I.TreadIds[0][0]=11; I.TreadIds[0][1]=12;
    TestEqual(TEXT("Pad resolves to real parent wheel"),I.WheelSlot(12),0);
    TestEqual(TEXT("Unrelated body is not a wheel"),I.WheelSlot(13),INDEX_NONE);
    I.Reset(); TestEqual(TEXT("Reset cannot retain stale pad IDs"),I.WheelSlot(12),INDEX_NONE);
    FContactLoadOutput O; O.TreadNormalNs[0][0]=2; O.Reset(); TestEqual(TEXT("Pad impulse cleared between output windows"),O.TreadNormalNs[0][0],0.);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTreadLimitTest,"ForkliftSim.Tread.ResolutionLimit",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FTreadLimitTest::RunTest(const FString&)
{
    FTreadSettings S; const double R=.15,W=.1;
    for(double H:{.135,.14,.145,.148})
    {
        // Independent midpoint integration of elemental radial equilibrium.
        // Closed form must not be checked by merely calling itself twice.
        constexpr int N=100000; const double Step=2*UE_DOUBLE_PI/N; double Integral=0;
        for(int I=0;I<N;++I)
        {
            const double A=-UE_DOUBLE_PI+(I+.5)*Step,Q=FMath::Cos(A);
            if(Q>0 && R*Q>H) Integral+=(S.FoundationNpm3*R*W*(R*Q-H)/(Q*Q)+S.MassKg*9.81/(2*UE_DOUBLE_PI))*Step;
        }
        const double Exact=S.ContinuousSupportN(R,W,H);
        TestTrue(TEXT("Continuous foundation integral matches independent quadrature"),FMath::Abs(Integral/Exact-1)<1e-6);
        S.Count=16; S.PhaseRad=0; const double Coarse=FMath::Abs(S.FlatSupportN(R,W,H)/Exact-1);
        S.Count=4096; const double Fine=FMath::Abs(S.FlatSupportN(R,W,H)/Exact-1);
        TestTrue(TEXT("Analytic refinement tends toward continuous limit"),Fine<.01 && Fine<Coarse);
        S.Count=16; S.PhaseRad=UE_DOUBLE_PI/16; const double Shifted=S.FlatSupportN(R,W,H);
        S.PhaseRad+=2*UE_DOUBLE_PI/16;
        const double PeriodError=FMath::Abs(S.FlatSupportN(R,W,H)-Shifted);
        AddInfo(FString::Printf(TEXT("Periodic support H=%.3f absolute_error_n=%.12g"),H,PeriodError));
        TestTrue(TEXT("One sector rotation is periodic"),PeriodError<1e-7);
        S.PhaseRad=0;
        TestTrue(TEXT("Coarse solver agreement alone cannot establish convergence"),Coarse>.05);
    }
    TestEqual(TEXT("Above the outer radius the limit has no contact"),S.ContinuousSupportN(R,W,.151),0.);
    FContactLoadInput In; In.TreadCounts[0]=64; In.TreadIds[0][63]=77; In.TreadPhaseRad=.125;
    TestEqual(TEXT("Reference-only 64th pad can be sampled"),In.WheelSlot(77),0);
    In.Reset(); TestEqual(TEXT("Reset clears fixture phase"),In.TreadPhaseRad,0.);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTreadCoupledTest,"ForkliftSim.Tread.CoupledMaterialAndBuffers",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FTreadCoupledTest::RunTest(const FString&)
{
    FTreadSettings S; S.Count=64;
    TestFalse(TEXT("Legacy radial profile rejects runtime 64 sectors"),S.Valid());
    S.bCoupled=true; TestTrue(TEXT("Coupled profile accepts runtime 64 sectors"),S.Valid());
    const double R=.15,W=.1,Length=2*UE_DOUBLE_PI*(R-S.DepthM*.5);
    const double E=S.FoundationNpm3*S.DepthM,G=E/(2*(1+S.Poisson));
    for(int N:{16,32,64})
    {
        S.Count=N;
        TestTrue(TEXT("Cohesive seam series compliance is resolution invariant"),FMath::Abs(N/S.LinkK(R,W)-Length/(G*W*S.DepthM))<1e-12);
        TestTrue(TEXT("Bending series compliance is resolution invariant"),FMath::Abs(N/S.BendK(R,W)-12*Length/(E*W*FMath::Pow(S.DepthM,3)))<1e-12);
        TestTrue(TEXT("Material damping remains passive"),S.PadShearC(R,W)>0 && S.PadRotC(R,W)>0 && S.LinkC(R,W)>0 && S.BendC(R,W)>0);
    }
    S.Count=128; TestFalse(TEXT("128 sectors remain a fixture-only override"),S.Valid());
    FContactLoadInput In; In.bTreadCoupled=true; In.TreadLinkC=42; In.Reset();
    TestFalse(TEXT("Reset clears coupled model flag"),In.bTreadCoupled); TestEqual(TEXT("Reset clears seam damping"),In.TreadLinkC,0.);
    FContactLoadOutput Out;
    TestTrue(TEXT("New contact-point buffers start at zero"),Out.TreadPointMNs[3][127].IsZero());
    Out.TreadPointMNs[3][127]=FVector(1,2,3); Out.Reset();
    TestTrue(TEXT("Reset clears per-sector contact points"),Out.TreadPointMNs[3][127].IsZero());
    In.bAuditWork=true; In.DriveTorqueNm[3]=FVector(2,3,4); In.Reset();
    TestTrue(TEXT("Reset cannot replay a previous rig's torque work"),!In.bAuditWork && In.DriveTorqueNm[3].IsZero());
    Out.DriveWorkJ=123; Out.SteeringJointWorkJ=456; Out.RollingWorkJ=-10; Out.Reset();
    TestTrue(TEXT("Output reuse cannot accumulate old work twice"),Out.DriveWorkJ==0 && Out.SteeringJointWorkJ==0 && Out.RollingWorkJ==0);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FActualSolverRangeTest,"ForkliftSim.Calibration.EngineIterationRange",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FActualSolverRangeTest::RunTest(const FString&)
{
    using Chaos::Private::FIterationSettings;
    TestEqual(TEXT("UE 5.6 stores iteration settings in a signed byte"),FIterationSettings::MaxIterations,127);
    FIterationSettings Requested(512,32,0);
    TestEqual(TEXT("A nominal 512 does not create a higher-quality comparison"),Requested.GetNumPositionIterations(),127);
    FIterationSettings Supported(96,32,0);
    TestEqual(TEXT("Supported comparison really changes solver iterations"),Supported.GetNumPositionIterations(),96);
    return true;
}
#endif
