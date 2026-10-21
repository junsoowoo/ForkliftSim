#pragma once
#include "CoreMinimal.h"

struct FVehicle3DSettings
{
    bool bSwerve = true;
    double YawRateDegS = 45;
    double AccelerationLimitMps2 = 1.5;
    double WheelSpeedGain = 350;
    double SweepRadiusM() const { return FMath::Sqrt(RadiusM*RadiusM+WidthM*WidthM*.25); }
    double MastXM() const { return WheelbaseM*.5+SweepRadiusM()+.10; }
    double ChassisMassKg = 1800;
    double WheelMassKg = 50;
    double KnuckleMassKg = 5;
    double ForkMassKg = 150;
    double RadiusM = .30;
    double WidthM = .18;
    double WheelbaseM = 1.60;
    double TrackM = 1.30;
    double TyreMu = .9;
    double RoadMu = .8;
    double DriveNm = 450;
    double BrakeNm = 900;
    double MaxSpeedMps = 3;
    double MaxSteerDeg = 30;
    double SteerRateDegS = 30;
    double SteeringTorqueNm = 5000;
    double ChassisComOffsetM = 0;
    double ForkTravelM = 1.5;
    double ForkSpeedMps = .25;
    double ForkForceN = 15000;
    double EffectiveMu() const { return FMath::Min(TyreMu, RoadMu); }
    double TotalMassKg() const { return ChassisMassKg + 4 * (WheelMassKg + KnuckleMassKg) + ForkMassKg; }
    bool Load(FString& Error);
};
