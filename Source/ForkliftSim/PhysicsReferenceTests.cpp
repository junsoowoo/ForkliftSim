#include "Vehicle3DSettings.h"
#include "PalletLoadActor.h"
#include "Misc/AutomationTest.h"
#include "ValidationMath.h"
#include "ContactLoadSampler.h"
#include "HydraulicCallback.h"
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPhysicsReferenceTest,"ForkliftSim.Validation.FrozenVirtualReference",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FPhysicsReferenceTest::RunTest(const FString& Parameters)
{
    const FVehicle3DSettings S; const FPayloadSettings P;
    TestEqual(TEXT("V1 dry mass kg"),S.TotalMassKg(),2170.);
    TestEqual(TEXT("V1 load kg"),P.PalletMassKg+P.CargoMassKg,425.);
    TestFalse(TEXT("Real cargo contacts, no ideal strap"),P.bCargoSecured);
    TestEqual(TEXT("V1 radius m"),S.RadiusM,.3);
    TestEqual(TEXT("V1 wheelbase m"),S.WheelbaseM,1.6);
    TestEqual(TEXT("V1 track m"),S.TrackM,1.3);
    TestEqual(TEXT("V1 tyre K"),S.Tyres.StiffnessNpm,600000.);
    TestEqual(TEXT("V1 tyre C"),S.Tyres.DampingNsPm,8000.);
    TestEqual(TEXT("V1 tyre mu"),S.TyreMu,.9); TestEqual(TEXT("V1 road mu"),S.RoadMu,.8);
    TestEqual(TEXT("V1 rolling coefficient"),S.RollingResistance,.015);
    TestEqual(TEXT("V1 bearing coulomb"),S.BearingCoulombNm,1.);
    TestEqual(TEXT("V1 bearing viscosity"),S.BearingViscousNms,.05);
    TestEqual(TEXT("V1 drive power per module"),S.Actuators.Drive.PowerW,2000.);
    TestEqual(TEXT("V1 lift area m2"),S.Hydraulics.AreaM2,.002);
    TestEqual(TEXT("V1 pump DC power W"),S.Hydraulics.MotorW,1200.);
    TestEqual(TEXT("V1 pump efficiency"),S.Hydraulics.Efficiency,.8);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FValidationSamplingTest,"ForkliftSim.Validation.TimeWindowAndAccumulatorReset",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FValidationSamplingTest::RunTest(const FString& Parameters)
{
    for(double Dt:{1./30,1./60,1./120}) for(double Offset:{0.,.003,.017})
    {
        double Distance=0,Duration=0,Height=Offset;
        for(int I=0;I<1200;++I)
        {
            const double Next=Height+.17*Dt;
            ValidationMath::RisingWindow(Height,Next,Dt,.25,.55,Distance,Duration); Height=Next;
        }
        TestTrue(TEXT("Fixed window distance without frame overshoot"),FMath::Abs(Distance-.3)<1e-10);
        TestTrue(TEXT("Constant rise independent of sample phase/FPS"),FMath::Abs(Distance/Duration-.17)<1e-10);
    }
    double D=0,T=0; ValidationMath::RisingWindow(.5,.3,.1,.25,.55,D,T);
    TestEqual(TEXT("Descent excluded"),D,0.);
    FContactLoadOutput Loads; Loads.PeakNormalN[0]=20000; Loads.MinNormalN[0]=0; Loads.RefinedContacts=5; Loads.Reset();
    TestEqual(TEXT("Inner peak reset per output interval"),Loads.PeakNormalN[0],0.);
    TestEqual(TEXT("Inner minimum has unset sentinel"),Loads.MinNormalN[0],1e30);
    TestEqual(TEXT("Contact count reset"),Loads.RefinedContacts,0);
    Loads.WheelContact[0].NormalNs=3; Loads.WheelContact[0].BandNs[0]=3; Loads.RefinedEdgeContacts=7; Loads.Reset();
    TestEqual(TEXT("Contact distribution does not leak across output windows"),Loads.WheelContact[0].NormalNs,0.);
    TestEqual(TEXT("Edge refinement count resets"),Loads.RefinedEdgeContacts,0);
    FHydraulicOutput H; H.PeakPressurePa=7e6; H.ReliefVolumeM3=.001; H.Reset();
    TestEqual(TEXT("Hydraulic integrated flow reset"),H.ReliefVolumeM3,0.);
    TestEqual(TEXT("Hydraulic peak reset"),H.PeakPressurePa,0.);
    return true;
}
#endif
