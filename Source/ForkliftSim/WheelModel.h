#pragma once
#include "CoreMinimal.h"

// SI-only planar rigid body model. Z/roll/pitch are constrained by a flat floor.
struct FWheelSettings
{
    double VehicleMassKg = 2200; // Includes the translational mass of all four wheels.
    double YawInertiaKgM2 = 1400;
    double WheelMassKg = 30;
    double WheelRadiusM = .25;
    double WheelWidthM = .18;
    double WheelInertiaFactor = .5; // I = factor * wheel mass * radius^2.
    double WheelbaseM = 1.6;
    double TrackM = 1.1;
    double ComForwardM = .2; // Ahead of axle midpoint; static load distribution only.
    double DriveTorqueNm = 650; // Each FRONT wheel, torque at the axle after gearing.
    double BrakeTorqueNm = 900; // Each wheel; simplified four-wheel service brake.
    double MotorMaxRadS = 24;
    double Friction = .8; // One Coulomb coefficient; shared longitudinal/lateral budget.
    double RollingResistance = .015;
    double MaxSteerRad = .5235987756;
    double SteerRateRadS = 1.0471975512;
    double WheelInertia() const { return WheelInertiaFactor * WheelMassKg * WheelRadiusM * WheelRadiusM; }
    bool Load(FString& Error);
    bool Validate(FString& Error) const;
};

struct FWheelContact
{
    double OmegaRadS = 0;
    double AngleRad = 0;
    double SteerRad = 0;
    double NormalForceN = 0;
    double LongForceN = 0;
    double LateralForceN = 0;
    double SlipSpeedMps = 0; // r*omega - contact-point longitudinal speed.
    double SlipRatio = 0; // Regularized denominator >= 0.1 m/s.
    double AppliedDriveNm = 0;
    FVector2D LocalPosition = FVector2D::ZeroVector;
};

class FWheelVehicleModel
{
public:
    FWheelSettings Settings;
    FVector2D PositionM = FVector2D::ZeroVector;
    FVector2D VelocityMps = FVector2D::ZeroVector;
    double YawRad = 0;
    double YawRateRadS = 0;
    double SteeringRad = 0;
    double TimeS = 0;
    FWheelContact Wheels[4]; // FL (-Y), FR (+Y), RL, RR.
    void Reset(double InitialSpeedMps = 0);
    void Step(double Throttle, double Brake, double Steering, double Dt);
    double ForwardSpeed() const;
    double KineticEnergyJ() const;
    static FVector2D Rotate(FVector2D V, double Angle);
private:
    void UpdateGeometry();
};
