#pragma once
#include "CoreMinimal.h"

// UE chassis axes: +X forward, +Y right, positive yaw turns right.
// Pure kinematics supplies targets only; Chaos contact and motor torque move the rig.
namespace Swerve
{
    struct FTarget { double AngleDeg=0, SpeedMps=0; };
    inline FTarget Solve(FVector2D PositionM,FVector2D TranslationMps,double YawRadS,double CurrentDeg)
    {
        const FVector2D V=TranslationMps+FVector2D(-YawRadS*PositionM.Y,YawRadS*PositionM.X);
        FTarget T; T.SpeedMps=V.Size(); T.AngleDeg=CurrentDeg;
        if(T.SpeedMps<1e-5) return T; // Keep the last orientation at rest.
        T.AngleDeg=FMath::RadiansToDegrees(FMath::Atan2(V.Y,V.X));
        if(FMath::Abs(FMath::FindDeltaAngleDegrees(CurrentDeg,T.AngleDeg))>90.000001)
        { T.AngleDeg=FMath::UnwindDegrees(T.AngleDeg+180); T.SpeedMps=-T.SpeedMps; }
        return T;
    }
    inline double Slew(double Current,double Target,double MaxStep)
    { return FMath::UnwindDegrees(Current+FMath::Clamp(FMath::FindDeltaAngleDegrees(Current,Target),-MaxStep,MaxStep)); }
}
