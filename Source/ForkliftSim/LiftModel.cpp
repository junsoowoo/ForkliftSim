#include "LiftModel.h"
#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

bool FLiftSettings::Load(FString& Error)
{
    FString Text;
    if (!FFileHelper::LoadFileToString(Text, *(FPaths::ProjectConfigDir() / TEXT("ForkliftLab.json"))))
    {
        Error = TEXT("Cannot read Config/ForkliftLab.json");
        return false;
    }
    TSharedPtr<FJsonObject> Json;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Json) || !Json.IsValid())
    {
        Error = TEXT("Invalid ForkliftLab.json");
        return false;
    }
    auto Read = [&](const TCHAR* Key, double& Out)
    {
        if (!Json->TryGetNumberField(Key, Out))
        {
            Error = FString::Printf(TEXT("Missing numeric setting: %s"), Key);
            return false;
        }
        return true;
    };
    if (!(Read(TEXT("carriage_mass_kg"), CarriageMassKg)
        && Read(TEXT("pallet_mass_kg"), PalletMassKg)
        && Read(TEXT("payload_mass_kg"), PayloadMassKg)
        && Read(TEXT("max_lift_force_n"), MaxLiftForceN)
        && Read(TEXT("max_lift_speed_mps"), MaxLiftSpeedMps)
        && Read(TEXT("max_lift_acceleration_mps2"), MaxLiftAccelerationMps2)
        && Read(TEXT("lift_target_m"), LiftTargetM)
        && Read(TEXT("lift_travel_m"), LiftTravelM)
        && Read(TEXT("guide_coulomb_friction_n"), GuideFrictionN)
        && Read(TEXT("guide_viscous_damping_ns_per_m"), GuideDampingNsPerM)
        && Read(TEXT("contact_friction"), ContactFriction)
        && Read(TEXT("velocity_kp_ns_per_m"), VelocityKp)
        && Read(TEXT("velocity_ki_n_per_m"), VelocityKi)
        && Read(TEXT("sensor_rate_hz"), SensorRateHz)
        && Read(TEXT("sensor_resolution_m"), SensorResolutionM)
        && Read(TEXT("sensor_delay_s"), SensorDelayS)
        && Read(TEXT("sensor_timeout_s"), SensorTimeoutS))) return false;
    return Validate(Error);
}

bool FLiftSettings::Validate(FString& Error) const
{
    const double Values[] = { CarriageMassKg, PalletMassKg, PayloadMassKg,
        MaxLiftForceN, MaxLiftSpeedMps, MaxLiftAccelerationMps2, LiftTargetM,
        LiftTravelM, GuideFrictionN, GuideDampingNsPerM, ContactFriction,
        VelocityKp, VelocityKi, SensorRateHz, SensorResolutionM, SensorDelayS, SensorTimeoutS };
    for (double V : Values)
    {
        if (!FMath::IsFinite(V) || V < 0.0)
        {
            Error = TEXT("Settings must be finite, non-negative SI values");
            return false;
        }
    }
    if (CarriageMassKg < 1 || PalletMassKg < 1 || MaxLiftForceN <= 0
        || MaxLiftSpeedMps <= 0 || MaxLiftAccelerationMps2 <= 0
        || LiftTargetM <= 0 || LiftTravelM <= LiftTargetM
        || SensorRateHz < 1 || SensorRateHz > 120 || SensorResolutionM <= 0
        || SensorTimeoutS <= SensorDelayS + 1.0 / SensorRateHz)
    {
        Error = TEXT("Invalid mass, actuator range or sensor timing configuration");
        return false;
    }
    Error.Empty();
    return true;
}

double FLiftForceController::Step(const FLiftSettings& S, double TargetHeightM,
                                 const FLiftSensorReading& R, double Dt)
{
    if (!R.bValid || Dt <= 0.0) return 0.0;
    const double DesiredVelocity = FMath::Clamp(0.6 * (TargetHeightM - R.HeightM),
                                              -S.MaxLiftSpeedMps, S.MaxLiftSpeedMps);
    CommandVelocityMps += FMath::Clamp(DesiredVelocity - CommandVelocityMps,
                                     -S.MaxLiftAccelerationMps2 * Dt,
                                      S.MaxLiftAccelerationMps2 * Dt);
    const double Error = CommandVelocityMps - R.VelocityMps;
    // Feedforward knows only carriage mass. Payload ground truth is never used by control.
    const double FeedforwardN = S.CarriageMassKg * ForkliftUnits::GravityMps2;
    const double CandidateIntegral = FMath::Clamp(IntegralN + S.VelocityKi * Error * Dt,
                                                -S.MaxLiftForceN, S.MaxLiftForceN);
    const double CandidateForce = FeedforwardN + S.VelocityKp * Error + CandidateIntegral;
    if ((CandidateForce >= 0 && CandidateForce <= S.MaxLiftForceN)
        || (CandidateForce > S.MaxLiftForceN && Error < 0)
        || (CandidateForce < 0 && Error > 0)) IntegralN = CandidateIntegral;
    // Single-acting lift: gravity lowers it; the actuator does not pull downward.
    return FMath::Clamp(CandidateForce, 0.0, S.MaxLiftForceN);
}
