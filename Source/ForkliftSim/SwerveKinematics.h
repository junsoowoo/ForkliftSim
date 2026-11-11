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
    // Release translation and yaw together, preserving the last turn curvature.
    // Independent ramps can demand an abrupt near-stationary spin while braking.
    inline double DecelerationYaw(double Yaw,double BeforeMps,double AfterMps)
    { return BeforeMps>1e-8?Yaw*FMath::Clamp(AfterMps/BeforeMps,0.,1.):0.; }
    inline double DesaturationScale(const FTarget* Targets,int32 Count,double WheelLimitMps)
    {
        double Peak=WheelLimitMps;
        for(int32 I=0;I<Count;++I) Peak=FMath::Max(Peak,FMath::Abs(Targets[I].SpeedMps));
        return WheelLimitMps/Peak;
    }
    // Change the common chassis yaw command only as fast as ALL modules can
    // steer. Slewing four unrelated angles can make them demand incompatible
    // circles and scrub/brake the vehicle during an S-curve reversal.
    inline double LimitYawForSteering(const FVector2D* Positions,const double* PreviousAngles,int32 Count,
        FVector2D Translation,double CurrentYaw,double DesiredYaw,double MaxAngleStepDeg,
        const double* ActualAngles=nullptr,double MaxTrackingErrorDeg=180)
    {
        auto Fits=[&](double Yaw)
        {
            for(int32 I=0;I<Count;++I)
            {
                const double Target=Solve(Positions[I],Translation,Yaw,PreviousAngles[I]).AngleDeg;
                if(FMath::Abs(FMath::FindDeltaAngleDegrees(PreviousAngles[I],Target))>MaxAngleStepDeg+1e-7) return false;
                if(ActualAngles && FMath::Abs(FMath::FindDeltaAngleDegrees(ActualAngles[I],Target))>MaxTrackingErrorDeg+1e-7) return false;
            }
            return true;
        };
        if(Fits(DesiredYaw)) return DesiredYaw;
        // Pure sideways starts require the separate stationary alignment phase.
        if(!Fits(CurrentYaw)) return CurrentYaw;
        double Low=0,High=1;
        for(int I=0;I<20;++I)
        { const double Mid=(Low+High)*.5; if(Fits(FMath::Lerp(CurrentYaw,DesiredYaw,Mid))) Low=Mid; else High=Mid; }
        return FMath::Lerp(CurrentYaw,DesiredYaw,Low);
    }
}
