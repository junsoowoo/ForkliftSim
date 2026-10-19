#include "LiftModel.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLiftUnitsTest, "ForkliftSim.Model.SIUnits", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FLiftUnitsTest::RunTest(const FString& Parameters)
{
    TestEqual(TEXT("100 kg gravity is 98100 Unreal force units"), ForkliftUnits::NewtonToUnreal(100 * 9.81), 98100.0);
    TestEqual(TEXT("1 Nm is 10000 Unreal torque units"), ForkliftUnits::NewtonMetreToUnreal(1), 10000.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLiftLimitsTest, "ForkliftSim.Model.ForceAndAccelerationLimits", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FLiftLimitsTest::RunTest(const FString& Parameters)
{
    FLiftSettings S;
    S.MaxLiftForceN = 1500;
    FLiftForceController C;
    FLiftSensorReading R;
    R.bValid = true;
    double PreviousCommand = 0;
    double Force = 0;
    for (int32 I = 0; I < 1000; ++I)
    {
        Force = C.Step(S, 0.65, R, 0.01);
        TestTrue(TEXT("Positive force limited by actuator"), Force >= 0 && Force <= S.MaxLiftForceN);
        TestTrue(TEXT("Command acceleration limited"), FMath::Abs(C.GetCommandVelocityMps() - PreviousCommand) <= S.MaxLiftAccelerationMps2 * .01 + 1e-9);
        PreviousCommand = C.GetCommandVelocityMps();
    }
    TestEqual(TEXT("Stalled command saturates"), Force, S.MaxLiftForceN);
    R.bValid = false;
    TestEqual(TEXT("Invalid measurement produces no drive command"), C.Step(S, .65, R, .01), 0.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLiftSettingsTest, "ForkliftSim.Model.ConfigurationValidation", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FLiftSettingsTest::RunTest(const FString& Parameters)
{
    FLiftSettings S;
    FString Error;
    TestTrue(TEXT("Default settings valid"), S.Validate(Error));
    S.SensorTimeoutS = S.SensorDelayS;
    TestFalse(TEXT("Timeout must exceed delivery latency"), S.Validate(Error));
    S = FLiftSettings();
    S.CarriageMassKg = -1;
    TestFalse(TEXT("Negative mass rejected"), S.Validate(Error));
    return true;
}
#endif
