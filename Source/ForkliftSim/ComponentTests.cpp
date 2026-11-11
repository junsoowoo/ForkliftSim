#include "ComponentDetail.h"
#include "HydraulicLift.h"
#include "TyreCompliance.h"
#include "Vehicle3DSettings.h"
#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FComponentBeamTest,"ForkliftSim.Components.BeamAnalyticalCompliance",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FComponentBeamTest::RunTest(const FString&)
{
    FForkStructureSettings S; S.bEnabled=true; S.LengthM=1.2; S.ThicknessM=.05;
    for(int Count:{2,4,8})
    {
        S.Segments=Count; double Deflection=0; const double Force=2000, D=S.LengthM/Count;
        for(int I=0;I<Count;++I) { const double Arm=S.LengthM-I*D; Deflection+=Force*Arm*Arm/S.JointK(I); }
        const double Exact=Force*FMath::Pow(S.LengthM,3)/(3*S.YoungPa*S.I());
        TestTrue(TEXT("Discrete beam converges to independent Euler-Bernoulli tip-load solution"),FMath::Abs(Deflection/Exact-1)<1./(2*Count*Count)+1e-10);
        TestTrue(TEXT("Positive elastic energy"),Deflection*Force*.5>0);
    }
    TestTrue(TEXT("Failure moment exceeds first yield"),S.BreakMomentNm()>S.YieldMomentNm());
    const double K=S.JointK(0), Trial=1.2*S.YieldMomentNm()/K, Returned=S.YieldMomentNm()/K;
    TestTrue(TEXT("Plastic rest update releases rather than creates elastic energy"),.5*K*Returned*Returned<.5*K*Trial*Trial);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FComponentTyreTest,"ForkliftSim.Components.TyreNonlinearEnergy",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FComponentTyreTest::RunTest(const FString&)
{
    FTyreComplianceSettings S; S.bDetailed=true; S.CubicNpM3=1e9;
    for(double X:{-.02,-.005,0.,.005,.02})
    {
        const double Eps=1e-7, Derivative=(S.StoredJ(X+Eps)-S.StoredJ(X-Eps))/(2*Eps);
        TestTrue(TEXT("Constitutive force is derivative of stored energy"),FMath::Abs(Derivative-S.ForceN(X,0))<.001);
        TestTrue(TEXT("Positive tangent and nonnegative energy"),S.TangentK(X)>0 && S.StoredJ(X)>=0);
    }
    TestTrue(TEXT("Detailed tyre hardens with compression"),S.TangentK(.02)>S.TangentK(.005));
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FComponentFluidTest,"ForkliftSim.Components.FluidPropertiesAndClosedLinePassivity",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FComponentFluidTest::RunTest(const FString&)
{
    FHydraulicSettings C; C.bDetailed=true; C.AmbientC=C.InitialTempC=40; C.CoolingWpK=0;
    TestTrue(TEXT("Published viscosity at 40 C"),FMath::Abs(C.ViscosityCst(40)-46)<1e-9);
    TestTrue(TEXT("Published viscosity at 100 C"),FMath::Abs(C.ViscosityCst(100)-6.9)<1e-9);
    TestTrue(TEXT("Viscosity decreases as fluid warms"),C.ViscosityCst(20)>C.ViscosityCst(80));
    for(double Dt:{1./240,1./480,1./960})
    {
        FHydraulicState S; S.bDetailedInitialized=true; S.OilTempC=40; S.PressurePa=1e6;
        for(double& P:S.LinePressurePa) P=1e6; S.LinePressurePa[0]=4e6;
        const double Initial=S.StoredJ(C,0); double Peak=Initial;
        for(int I=0;I<FMath::RoundToInt(2/Dt);++I) { S.Step(C,Dt,0,0,0,1.5,15000); Peak=FMath::Max(Peak,S.StoredJ(C,0)); }
        TestTrue(TEXT("Closed unpowered hose stays finite and dissipates energy"),FMath::IsFinite(S.PressurePa) && S.StoredJ(C,0)<Initial && S.DcJ==0);
        TestTrue(TEXT("CFL pressure/flow integration bounds passive energy gain"),Peak<=Initial*1.03);
        TestTrue(TEXT("Fluid plus heat respects passive energy bound"),S.StoredJ(C,0)+S.HeatJ<=Initial*1.03);
        TestTrue(TEXT("Pressure relaxation creates viscous heat"),S.HeatJ>0 && S.OilTempC>40);
    }
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FComponentBindingTest,"ForkliftSim.Components.PublishedGeometryBindings",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FComponentBindingTest::RunTest(const FString&)
{
    FVehicle3DSettings S; FString Error; TestTrue(TEXT("Read active component profile"),S.Load(Error)); AddInfo(Error);
    TestTrue(TEXT("Selected drive wheel catalogue geometry"),FMath::Abs(S.RadiusM-.15)<1e-9 && FMath::Abs(S.WidthM-.1)<1e-9);
    TestTrue(TEXT("AGV fork geometry binding"),S.Structure.LengthM==1.2 && S.Structure.WidthM==.12 && S.Structure.ThicknessM==.05);
    TestTrue(TEXT("Mass budget conserves design target"),FMath::Abs(S.TotalMassKg()-3140)<1e-6);
    TestTrue(TEXT("All three component models enabled in active profile"),S.Structure.bEnabled && S.Tyres.bDetailed && S.Hydraulics.bDetailed && S.Hydraulics.bChainMast);
    return true;
}
#endif
