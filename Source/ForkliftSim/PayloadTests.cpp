#include "PalletLoadActor.h"
#include "CargoGeometry.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPayloadGeometryTest,"ForkliftSim.Payload.GeometryValidation",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FPayloadGeometryTest::RunTest(const FString& Parameters)
{
    FString Error; FPayloadSettings S;
    TestFalse(TEXT("Unrestrained cargo is the default"),S.bCargoSecured);
    TestTrue(TEXT("Default fixture valid"),S.Validate(Error));
    S.HoleShiftYM=.3; TestFalse(TEXT("Hole cannot cut outside pallet"),S.Validate(Error));
    S=FPayloadSettings(); S.HoleWidthM=.08; TestFalse(TEXT("Unsupported narrow opening rejected"),S.Validate(Error));
    S=FPayloadSettings(); S.CargoMassKg=-1; TestFalse(TEXT("Negative mass rejected"),S.Validate(Error));
    S=FPayloadSettings(); S.ForkMu=-.1; TestFalse(TEXT("Negative friction rejected"),S.Validate(Error));
    S=FPayloadSettings(); S.ForkShiftYM=.3; TestFalse(TEXT("Fork outside carriage rejected"),S.Validate(Error));
    S=FPayloadSettings(); S.HoleShiftYM=S.ForkShiftYM=.08;
    TestTrue(TEXT("Shifted openings and forks valid"),S.Validate(Error));
    TestEqual(TEXT("Changing holes does not change mass"),S.CargoMassKg+S.PalletMassKg,425.);
    TestEqual(TEXT("Changing holes does not change material coefficient"),S.LeftHoleMu,.45);
    S.CargoStaticMu=.1; S.CargoDynamicMu=.2;
    TestFalse(TEXT("Static coefficient below dynamic rejected"),S.Validate(Error));
    S=FPayloadSettings(); S.CargoWidthM=.01;
    TestFalse(TEXT("Degenerate cargo shape rejected"),S.Validate(Error));
    const TArray<FVector2D> Full={FVector2D(-.3,-.3),FVector2D(.3,-.3),FVector2D(.3,.3),FVector2D(-.3,.3)};
    TestEqual(TEXT("Centered base fully overlaps deck"),CargoGeometry::OverlapFraction(Full,FVector2D(.6,.6)),1.);
    TArray<FVector2D> Half=Full; for(auto& P:Half) P.X+=.6;
    TestTrue(TEXT("Half-overhanging base has half overlap"),FMath::IsNearlyEqual(CargoGeometry::OverlapFraction(Half,FVector2D(.6,.6)),.5,1e-9));
    for(auto& P:Half) P.X+=1;
    TestEqual(TEXT("Off-deck cargo has zero overlap"),CargoGeometry::OverlapFraction(Half,FVector2D(.6,.6)),0.);
    return true;
}
#endif
