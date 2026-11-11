#include "TyreCompliance.h"
#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTyreComplianceTest,"ForkliftSim.Tyres.CompliancePassivityAndUnits",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FTyreComplianceTest::RunTest(const FString& Parameters)
{
    FTyreComplianceSettings S; TestTrue(TEXT("Default tyre spec valid"),S.Valid());
    TestEqual(TEXT("10 mm deflection at 600 kN/m supports 6 kN"),S.ForceN(.01,0),6000.);
    TestEqual(TEXT("Spring stores 30 J at 10 mm"),S.StoredJ(.01),30.);
    for(double V:{-2.,-.1,0.,.1,2.}) TestTrue(TEXT("Damper dissipates in both directions"),S.DissipationW(V)>=0);
    S.DampingNsPm=-1; TestFalse(TEXT("Negative damping rejected"),S.Valid());
    S=FTyreComplianceSettings(); S.TravelM=.2; TestFalse(TEXT("Unsupported tyre travel rejected"),S.Valid());
    S=FTyreComplianceSettings();
    // Passive implicit spring/damper reference, not a second force on the rig.
    for(double Dt:{1./120,1./240,1./480})
    {
        double X=.02,V=0,E=S.StoredJ(X);
        for(int I=0;I<FMath::RoundToInt(2/Dt);++I)
        {
            V=(500*V-Dt*S.StiffnessNpm*X)/(500+Dt*S.DampingNsPm+Dt*Dt*S.StiffnessNpm);
            X+=Dt*V; const double Next=.5*500*V*V+S.StoredJ(X);
            TestTrue(TEXT("Unforced compliant reference does not create energy"),Next<=E+1e-9); E=Next;
        }
        TestTrue(TEXT("Reference vibration decays"),E<.001);
    }
    return true;
}
#endif
