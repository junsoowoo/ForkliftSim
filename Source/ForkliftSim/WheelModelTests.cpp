#include "WheelModel.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace
{
    FWheelVehicleModel Drive(double Mu, double Seconds = 3, double Step = 1. / 240.)
    {
        FWheelVehicleModel M; M.Settings.Friction = Mu; M.Reset();
        for (int I = 0; I < FMath::RoundToInt(Seconds / Step); ++I) M.Step(1, 0, 0, Step);
        return M;
    }
    double StopDistance(double Mu)
    {
        FWheelVehicleModel M; M.Settings.Friction = Mu; M.Reset(3);
        for (int I = 0; I < 4800 && M.VelocityMps.Size() > .01; ++I) M.Step(0, 1, 0, 1. / 240.);
        return M.VelocityMps.Size() > .01 ? -1 : M.PositionM.X;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelSettingsTest, "ForkliftSim.Wheels.SettingsAndLoads", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FWheelSettingsTest::RunTest(const FString& Parameters)
{
    FWheelVehicleModel M; FString Error; TestTrue(TEXT("Defaults valid"), M.Settings.Validate(Error)); M.Reset();
    double Load = 0; for (auto& W : M.Wheels) Load += W.NormalForceN;
    TestEqual(TEXT("Loads sum to vehicle weight; wheels not counted twice"), Load, M.Settings.VehicleMassKg * 9.81);
    TestEqual(TEXT("Cylinder spin inertia"), M.Settings.WheelInertia(), .9375);
    M.Settings.WheelRadiusM = 0; TestFalse(TEXT("Zero radius rejected"), M.Settings.Validate(Error));
    M.Settings = FWheelSettings(); M.Settings.ComForwardM = 2; TestFalse(TEXT("Invalid CoM rejected"), M.Settings.Validate(Error));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelTractionTest, "ForkliftSim.Wheels.TractionAndFreeSpin", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FWheelTractionTest::RunTest(const FString& Parameters)
{
    auto Dry = Drive(.8), Ice = Drive(.08), Free = Drive(0);
    AddInfo(FString::Printf(TEXT("3s: dry=%.4f m/s low_grip=%.4f m/s; dry_slip=%.4f low_grip_slip=%.4f m/s"),
        Dry.ForwardSpeed(), Ice.ForwardSpeed(), Dry.Wheels[0].SlipSpeedMps, Ice.Wheels[0].SlipSpeedMps));
    TestTrue(TEXT("Dry floor accelerates faster"), Dry.ForwardSpeed() > Ice.ForwardSpeed() + 1);
    TestTrue(TEXT("Low grip produces wheelspin"), Ice.Wheels[0].SlipSpeedMps > 2);
    TestTrue(TEXT("Dry contact rolls without significant slip"), FMath::Abs(Dry.Wheels[0].SlipSpeedMps) < .05);
    bool Symmetric = FMath::Abs(Dry.YawRad) < 1e-8 && FMath::Abs(Dry.PositionM.Y) < 1e-8;
    for (const auto& W : Dry.Wheels) Symmetric &= FMath::Abs(W.LateralForceN) < .01;
    TestTrue(TEXT("Straight drive has no spurious lateral tyre forces or yaw"), Symmetric);
    TestTrue(TEXT("Zero friction cannot translate chassis"), Free.VelocityMps.Size() < 1e-8);
    TestTrue(TEXT("Motor still spins with zero traction"), Free.Wheels[0].OmegaRadS > 10);
    const auto OneSecond = Drive(.8, 1);
    const auto& S = OneSecond.Settings;
    const double ExpectedAcceleration = (2*S.DriveTorqueNm/S.WheelRadiusM - S.RollingResistance*S.VehicleMassKg*9.81)
        / (S.VehicleMassKg + 4*S.WheelInertia()/(S.WheelRadiusM*S.WheelRadiusM));
    AddInfo(FString::Printf(TEXT("Straight rolling acceleration: analytic=%.5f simulated=%.5f m/s2"), ExpectedAcceleration, OneSecond.ForwardSpeed()));
    TestTrue(TEXT("Rolling acceleration agrees with force balance including wheel inertia"), FMath::Abs(OneSecond.ForwardSpeed() - ExpectedAcceleration) < .03);
    FWheelVehicleModel Heavy, Light; Heavy.Settings.WheelMassKg = 60; Heavy.Settings.Friction = Light.Settings.Friction = 0;
    Heavy.Reset(); Light.Reset(); Heavy.Step(1, 0, 0, 1./240.); Light.Step(1, 0, 0, 1./240.);
    TestEqual(TEXT("Double spin inertia halves angular acceleration"), Heavy.Wheels[0].OmegaRadS * 2, Light.Wheels[0].OmegaRadS);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelBrakeTest, "ForkliftSim.Wheels.BrakingAndEnergy", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FWheelBrakeTest::RunTest(const FString& Parameters)
{
    const double Dry = StopDistance(.8), Ice = StopDistance(.08);
    AddInfo(FString::Printf(TEXT("From 3 m/s: dry stop=%.4f m low-grip stop=%.4f m"), Dry, Ice));
    TestTrue(TEXT("Both cases stop"), Dry > 0 && Ice > 0);
    TestTrue(TEXT("Low grip has a longer stopping distance"), Ice > Dry * 3);
    FWheelVehicleModel M; M.Reset(3); double Previous = M.KineticEnergyJ(); bool EnergyOK = true;
    for (int I = 0; I < 1200; ++I)
    {
        M.Step(0, 1, 0, 1./240.); const double E = M.KineticEnergyJ();
        EnergyOK &= E <= Previous + 1e-5; Previous = E;
    }
    TestTrue(TEXT("Brakes do not add kinetic energy"), EnergyOK);
    TestTrue(TEXT("Stopped brakes do not reverse vehicle"), FMath::Abs(M.ForwardSpeed()) < .001);
    FWheelVehicleModel Coast; Coast.Reset(3);
    for (int I = 0; I < 2400; ++I) Coast.Step(0, 0, 0, 1./240.);
    TestTrue(TEXT("Rolling resistance slows coasting"), Coast.ForwardSpeed() > 0 && Coast.ForwardSpeed() < 2);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelCircleTest, "ForkliftSim.Wheels.FrictionBudget", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FWheelCircleTest::RunTest(const FString& Parameters)
{
    FWheelVehicleModel M; M.Settings.Friction = .12; M.Reset(2); bool Valid = true;
    for (int I = 0; I < 1200; ++I)
    {
        M.Step(I < 600 ? 1 : 0, I >= 600 ? 1 : 0, .8, 1./240.);
        for (const auto& W : M.Wheels)
            Valid &= FMath::Sqrt(W.LongForceN * W.LongForceN + W.LateralForceN * W.LateralForceN)
                <= M.Settings.Friction * W.NormalForceN + 1e-5;
    }
    TestTrue(TEXT("Combined longitudinal/lateral force stays inside mu*N"), Valid);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelSteerTest, "ForkliftSim.Wheels.RearSteeringAndReverse", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FWheelSteerTest::RunTest(const FString& Parameters)
{
    FWheelVehicleModel Right, Left, Reverse; Right.Reset(1); Left.Reset(1); Reverse.Reset();
    for (int I = 0; I < 480; ++I)
    { Right.Step(.1, 0, .5, 1./240.); Left.Step(.1, 0, -.5, 1./240.); Reverse.Step(-1, 0, 0, 1./240.); }
    TestTrue(TEXT("Right command rotates chassis right with rear wheels left"), Right.YawRad > .1 && Right.Wheels[2].SteerRad < 0);
    TestTrue(TEXT("Left command rotates chassis left"), Left.YawRad < -.1);
    TestTrue(TEXT("Inside rear wheel has greater steer angle"), FMath::Abs(Right.Wheels[3].SteerRad) > FMath::Abs(Right.Wheels[2].SteerRad));
    TestTrue(TEXT("Reverse torque drives backwards"), Reverse.ForwardSpeed() < -1);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FWheelConvergenceTest, "ForkliftSim.Wheels.TimeStepConvergence", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FWheelConvergenceTest::RunTest(const FString& Parameters)
{
    const auto A = Drive(.8, 3, 1./120.), B = Drive(.8, 3, 1./240.), C = Drive(.8, 3, 1./480.);
    AddInfo(FString::Printf(TEXT("dt 1/120,1/240,1/480: x=%.5f,%.5f,%.5f m; v=%.5f,%.5f,%.5f m/s"),
        A.PositionM.X, B.PositionM.X, C.PositionM.X, A.ForwardSpeed(), B.ForwardSpeed(), C.ForwardSpeed()));
    TestTrue(TEXT("Position converges within 5 cm"), FMath::Abs(B.PositionM.X - C.PositionM.X) < .05);
    TestTrue(TEXT("Velocity converges within 0.02 m/s"), FMath::Abs(B.ForwardSpeed() - C.ForwardSpeed()) < .02);
    TestTrue(TEXT("Coarser timestep remains consistent"), FMath::Abs(A.PositionM.X - C.PositionM.X) < .1);
    return true;
}
#endif
