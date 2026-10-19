#pragma once
#include "CoreMinimal.h"

namespace ForkliftUnits
{
    constexpr double GravityMps2 = 9.81;
    constexpr double CmPerM = 100.0;
    inline double NewtonToUnreal(double ForceN) { return ForceN * 100.0; }
    inline double NewtonMetreToUnreal(double TorqueNm) { return TorqueNm * 10000.0; }
}

// SI units. Direct vertical actuator without hydraulic pulley ratios.
struct FLiftSettings
{
    double CarriageMassKg = 100.0;
    double PalletMassKg = 25.0;
    double PayloadMassKg = 250.0;
    double MaxLiftForceN = 6000.0;
    double MaxLiftSpeedMps = 0.25;
    double MaxLiftAccelerationMps2 = 0.5;
    double LiftTargetM = 0.65;
    double LiftTravelM = 1.0;
    double GuideFrictionN = 80.0;
    double GuideDampingNsPerM = 150.0;
    double ContactFriction = 0.6;
    double VelocityKp = 1000.0;
    double VelocityKi = 1200.0;
    double SensorRateHz = 20.0;
    double SensorResolutionM = 0.001;
    double SensorDelayS = 0.05;
    double SensorTimeoutS = 0.3;
    bool Load(FString& Error);
    bool Validate(FString& Error) const;
};

struct FLiftSensorReading
{
    double TimestampS = -1.0;
    double HeightM = 0.0;
    double VelocityMps = 0.0;
    bool bValid = false;
};

class FLiftForceController
{
public:
    double Step(const FLiftSettings& Settings, double TargetHeightM,
                const FLiftSensorReading& Reading, double DeltaS);
    void Reset() { IntegralN = 0.0; CommandVelocityMps = 0.0; }
    double GetCommandVelocityMps() const { return CommandVelocityMps; }
private:
    double IntegralN = 0.0;
    double CommandVelocityMps = 0.0;
};
