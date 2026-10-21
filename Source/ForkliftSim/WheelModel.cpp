#include "WheelModel.h"
#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
    double Cross(FVector2D A, FVector2D B) { return A.X * B.Y - A.Y * B.X; }
    FVector2D Perp(FVector2D V) { return FVector2D(-V.Y, V.X); }
}

bool FWheelSettings::Load(FString& Error)
{
    FString Text;
    TSharedPtr<FJsonObject> Json;
    if (!FFileHelper::LoadFileToString(Text, *(FPaths::ProjectConfigDir() / TEXT("WheelLab.json")))
        || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Json) || !Json.IsValid())
    { Error = TEXT("Cannot read valid Config/WheelLab.json"); return false; }
    auto Read = [&](const TCHAR* Key, double& Value)
    {
        if (Json->TryGetNumberField(Key, Value)) return true;
        Error = FString::Printf(TEXT("Missing numeric wheel setting: %s"), Key); return false;
    };
    if (!(Read(TEXT("vehicle_mass_kg"), VehicleMassKg) && Read(TEXT("yaw_inertia_kgm2"), YawInertiaKgM2)
        && Read(TEXT("wheel_mass_kg"), WheelMassKg) && Read(TEXT("wheel_radius_m"), WheelRadiusM)
        && Read(TEXT("wheel_width_m"), WheelWidthM) && Read(TEXT("wheel_inertia_factor"), WheelInertiaFactor)
        && Read(TEXT("wheelbase_m"), WheelbaseM) && Read(TEXT("track_m"), TrackM)
        && Read(TEXT("com_forward_m"), ComForwardM) && Read(TEXT("drive_torque_nm"), DriveTorqueNm)
        && Read(TEXT("brake_torque_nm"), BrakeTorqueNm) && Read(TEXT("motor_max_rad_s"), MotorMaxRadS)
        && Read(TEXT("friction"), Friction) && Read(TEXT("rolling_resistance"), RollingResistance)
        && Read(TEXT("max_steer_rad"), MaxSteerRad) && Read(TEXT("steer_rate_rad_s"), SteerRateRadS))) return false;
    return Validate(Error);
}

bool FWheelSettings::Validate(FString& Error) const
{
    const double Positive[] = { VehicleMassKg, YawInertiaKgM2, WheelMassKg, WheelRadiusM, WheelWidthM,
        WheelInertiaFactor, WheelbaseM, TrackM, DriveTorqueNm, BrakeTorqueNm, MotorMaxRadS, MaxSteerRad, SteerRateRadS };
    for (double V : Positive) if (!FMath::IsFinite(V) || V <= 0)
    { Error = TEXT("Mass, geometry, inertia and actuator settings must be finite and positive"); return false; }
    if (!FMath::IsFinite(ComForwardM) || FMath::Abs(ComForwardM) >= WheelbaseM * .45
        || !FMath::IsFinite(Friction) || Friction < 0 || Friction > 2
        || !FMath::IsFinite(RollingResistance) || RollingResistance < 0 || RollingResistance > .1
        || VehicleMassKg <= 4 * WheelMassKg || WheelInertiaFactor > 1
        || MaxSteerRad > .7 || TrackM * FMath::Tan(MaxSteerRad) >= WheelbaseM)
    { Error = TEXT("Invalid wheel mass, static load distribution, friction or steering geometry"); return false; }
    Error.Empty(); return true;
}

FVector2D FWheelVehicleModel::Rotate(FVector2D V, double A)
{ const double C = FMath::Cos(A), S = FMath::Sin(A); return FVector2D(C * V.X - S * V.Y, S * V.X + C * V.Y); }

void FWheelVehicleModel::UpdateGeometry()
{
    for (int I = 0; I < 4; ++I)
    {
        const bool Front = I < 2;
        const double Y = (I % 2 == 0 ? -.5 : .5) * Settings.TrackM;
        Wheels[I].LocalPosition = FVector2D((Front ? .5 : -.5) * Settings.WheelbaseM - Settings.ComForwardM, Y);
        const double FrontFraction = .5 + Settings.ComForwardM / Settings.WheelbaseM;
        Wheels[I].NormalForceN = Settings.VehicleMassKg * 9.81 * (Front ? FrontFraction : 1 - FrontFraction) * .5;
        // Rear Ackermann: front tyres roll along +X, rear inside wheel steers more.
        const double T = FMath::Tan(SteeringRad);
        Wheels[I].SteerRad = Front ? 0 : FMath::Atan(T / (1 + Y * T / Settings.WheelbaseM));
    }
}

void FWheelVehicleModel::Reset(double Speed)
{
    PositionM = FVector2D::ZeroVector; VelocityMps = FVector2D(Speed, 0);
    YawRad = YawRateRadS = SteeringRad = TimeS = 0;
    for (auto& W : Wheels) { W = FWheelContact(); W.OmegaRadS = Speed / Settings.WheelRadiusM; }
    UpdateGeometry();
}

void FWheelVehicleModel::Step(double Throttle, double Brake, double Steering, double Dt)
{
    if (!FMath::IsFinite(Dt) || Dt <= 0 || Dt > .01) return;
    Throttle = FMath::Clamp(Throttle, -1., 1.); Brake = FMath::Clamp(Brake, 0., 1.);
    const double TargetSteer = -FMath::Clamp(Steering, -1., 1.) * Settings.MaxSteerRad;
    SteeringRad += FMath::Clamp(TargetSteer - SteeringRad, -Settings.SteerRateRadS * Dt, Settings.SteerRateRadS * Dt);
    UpdateGeometry();
    const double InvM = 1 / Settings.VehicleMassKg, InvYawI = 1 / Settings.YawInertiaKgM2;
    const double InvWheelI = 1 / Settings.WheelInertia(), Radius = Settings.WheelRadiusM;
    FVector2D Lever[4], Forward[4], Side[4], AccumImpulse[4];
    double AccumBrake[4] = {}, BrakeBudget[4] = {};
    for (int I = 0; I < 4; ++I)
    {
        auto& W = Wheels[I];
        Lever[I] = Rotate(W.LocalPosition, YawRad);
        Forward[I] = Rotate(FVector2D(1, 0), YawRad + W.SteerRad); Side[I] = Perp(Forward[I]);
        AccumImpulse[I] = FVector2D::ZeroVector;
        // Torque roll-off in the commanded rotation direction, not a velocity teleport/clamp.
        const double MotorFactor = FMath::Clamp((Settings.MotorMaxRadS - W.OmegaRadS * FMath::Sign(Throttle))
            / (Settings.MotorMaxRadS * .2), 0., 1.);
        W.AppliedDriveNm = I < 2 && Brake < .01 ? Throttle * Settings.DriveTorqueNm * MotorFactor : 0;
        W.OmegaRadS += W.AppliedDriveNm * InvWheelI * Dt;
        BrakeBudget[I] = (Brake * Settings.BrakeTorqueNm + Settings.RollingResistance * W.NormalForceN * Radius) * Dt;
    }
    // Coupled contact + wheel-brake impulse solver. The accumulated tyre impulse is
    // projected onto a shared Coulomb friction disk, preventing double use of grip.
    // Relaxed simultaneous updates preserve left/right symmetry. Sequential wheel
    // updates can leave arbitrary self-equilibrating lateral forces in straight travel.
    for (int Iteration = 0; Iteration < 64; ++Iteration)
    {
        FVector2D Deltas[4];
        for (int I = 0; I < 4; ++I)
        {
            auto& W = Wheels[I];
            const double NewBrake = FMath::Clamp(AccumBrake[I] - W.OmegaRadS / InvWheelI, -BrakeBudget[I], BrakeBudget[I]);
            W.OmegaRadS += (NewBrake - AccumBrake[I]) * InvWheelI; AccumBrake[I] = NewBrake;
            const FVector2D V = VelocityMps + Perp(Lever[I]) * YawRateRadS;
            const double A = Cross(Lever[I], Forward[I]), B = Cross(Lever[I], Side[I]);
            const double Kxx = InvM + A * A * InvYawI + Radius * Radius * InvWheelI;
            const double Kyy = InvM + B * B * InvYawI;
            const double Ex = Radius * W.OmegaRadS - FVector2D::DotProduct(V, Forward[I]);
            const double Ey = -FVector2D::DotProduct(V, Side[I]);
            FVector2D NewImpulse = AccumImpulse[I] + FVector2D(Ex / Kxx, Ey / Kyy);
            const double Budget = Settings.Friction * W.NormalForceN * Dt;
            if (NewImpulse.SizeSquared() > Budget * Budget) NewImpulse *= Budget / NewImpulse.Size();
            Deltas[I] = (NewImpulse - AccumImpulse[I]) * .5;
            AccumImpulse[I] += Deltas[I];
        }
        FVector2D TotalImpulse = FVector2D::ZeroVector;
        double TotalYawImpulse = 0;
        for (int I = 0; I < 4; ++I)
        {
            const FVector2D Delta = Deltas[I];
            const FVector2D WorldImpulse = Forward[I] * Delta.X + Side[I] * Delta.Y;
            TotalImpulse += WorldImpulse; TotalYawImpulse += Cross(Lever[I], WorldImpulse);
            Wheels[I].OmegaRadS -= Radius * Delta.X * InvWheelI;
        }
        VelocityMps += TotalImpulse * InvM; YawRateRadS += TotalYawImpulse * InvYawI;
    }
    for (int I = 0; I < 4; ++I)
    {
        auto& W = Wheels[I];
        W.LongForceN = AccumImpulse[I].X / Dt; W.LateralForceN = AccumImpulse[I].Y / Dt;
        const double ContactSpeed = FVector2D::DotProduct(VelocityMps + Perp(Lever[I]) * YawRateRadS, Forward[I]);
        W.SlipSpeedMps = Radius * W.OmegaRadS - ContactSpeed;
        W.SlipRatio = W.SlipSpeedMps / FMath::Max(.1, FMath::Max(FMath::Abs(ContactSpeed), FMath::Abs(Radius * W.OmegaRadS)));
        W.AngleRad = FMath::Fmod(W.AngleRad + W.OmegaRadS * Dt, 2 * PI);
    }
    PositionM += VelocityMps * Dt; YawRad = FMath::UnwindRadians(YawRad + YawRateRadS * Dt); TimeS += Dt;
}

double FWheelVehicleModel::ForwardSpeed() const
{ return FVector2D::DotProduct(VelocityMps, Rotate(FVector2D(1, 0), YawRad)); }

double FWheelVehicleModel::KineticEnergyJ() const
{
    double E = .5 * Settings.VehicleMassKg * VelocityMps.SizeSquared() + .5 * Settings.YawInertiaKgM2 * YawRateRadS * YawRateRadS;
    for (const auto& W : Wheels) E += .5 * Settings.WheelInertia() * W.OmegaRadS * W.OmegaRadS;
    return E;
}
