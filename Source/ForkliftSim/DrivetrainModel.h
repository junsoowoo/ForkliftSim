#pragma once
#include "CoreMinimal.h"

// System-level, four-quadrant electric actuator. SI mechanical shaft quantities,
// not a winding/current/voltage or battery model. G = motor speed / output speed.
namespace ElectricDrive
{
    struct FSpec
    {
        double PeakNm=45, PowerW=2000, MaxRpm=1800, Ratio=12, GearEfficiency=.9, MotorEfficiency=.9;
        double ResponseS=.04, RotorInertiaKgM2=.0004;
        bool Valid() const
        {
            for(double V:{PeakNm,PowerW,MaxRpm,Ratio,GearEfficiency,MotorEfficiency,ResponseS,RotorInertiaKgM2})
                if(!FMath::IsFinite(V)) return false;
            return PeakNm>0 && PeakNm<=500 && PowerW>0 && PowerW<=50000 && MaxRpm>=100 && MaxRpm<=20000
                && Ratio>=1 && Ratio<=300 && GearEfficiency>.1 && GearEfficiency<=1 && MotorEfficiency>.1 && MotorEfficiency<=1
                && ResponseS>=0 && ResponseS<=1 && RotorInertiaKgM2>=0 && RotorInertiaKgM2<=.01;
        }
    };
    struct FResult
    {
        double OutputNm=0, LimitNm=0, MotorNm=0, MotorRpm=0, ShaftW=0, DcW=0, LossW=0;
    };
    inline double Lag(double State,double Target,double Dt,double Tau)
    { return Tau>0?State+(Target-State)*(1-FMath::Exp(-FMath::Max(0.,Dt)/Tau)):Target; }
    inline double ShaftLimit(const FSpec& S,double MotorOmega,bool Motoring)
    {
        const double Speed=FMath::Abs(MotorOmega), MaxOmega=S.MaxRpm*PI/30;
        const double Base=FMath::Min(S.PeakNm,S.PowerW/FMath::Max(Speed,1e-9));
        // A defined 10% soft run-out before maximum motoring speed. Braking
        // remains possible above that speed; there is no artificial speed clamp.
        return Base*(Motoring?FMath::Clamp((MaxOmega-Speed)/(.1*MaxOmega),0.,1.):1.);
    }
    inline double OutputLimit(const FSpec& S,double Omega,bool Motoring,double SafetyLimit)
    { return FMath::Min(SafetyLimit,ShaftLimit(S,Omega*S.Ratio,Motoring)*S.Ratio*(Motoring?S.GearEfficiency:1/S.GearEfficiency)); }
    inline FResult Measure(double OutputNm,double Omega,const FSpec& S,double Limit)
    {
        FResult R; R.OutputNm=OutputNm; R.LimitNm=Limit; R.MotorRpm=Omega*S.Ratio*30/PI;
        const bool Motoring=OutputNm*Omega>=0;
        R.MotorNm=OutputNm/(S.Ratio*(Motoring?S.GearEfficiency:1/S.GearEfficiency));
        R.ShaftW=R.MotorNm*Omega*S.Ratio;
        R.DcW=Motoring?R.ShaftW/S.MotorEfficiency:R.ShaftW*S.MotorEfficiency;
        R.LossW=FMath::Max(0.,R.DcW-OutputNm*Omega);
        return R;
    }
    inline FResult Step(double RequestNm,double Omega,const FSpec& S,double SafetyLimit,double Dt,bool Enabled,double& ShaftStateNm)
    {
        if(!Enabled) { ShaftStateNm=0; return Measure(0,Omega,S,OutputLimit(S,Omega,true,SafetyLimit)); }
        RequestNm=FMath::Clamp(RequestNm,-SafetyLimit,SafetyLimit);
        const bool DemandMotoring=RequestNm*Omega>=0;
        const double Conversion=S.Ratio*(DemandMotoring?S.GearEfficiency:1/S.GearEfficiency);
        ShaftStateNm=Lag(ShaftStateNm,RequestNm/Conversion,Dt,S.ResponseS);
        const bool Motoring=ShaftStateNm*Omega>=0;
        const double Limit=OutputLimit(S,Omega,Motoring,SafetyLimit);
        const double OutputConversion=S.Ratio*(Motoring?S.GearEfficiency:1/S.GearEfficiency);
        const double Output=FMath::Clamp(ShaftStateNm*OutputConversion,-Limit,Limit);
        ShaftStateNm=Output/OutputConversion; // no hidden wind-up beyond the envelope
        return Measure(Output,Omega,S,Limit);
    }
    // Backlash play operator at the gearbox output, not random measurement noise.
    inline double Backlash(double Previous,double Input,double WidthDeg)
    {
        const double D=FMath::FindDeltaAngleDegrees(Previous,Input), Half=WidthDeg*.5;
        return FMath::UnwindDegrees(Previous+FMath::Clamp(D-Half,0.,180.)+FMath::Clamp(D+Half,-180.,0.));
    }
}
struct FDrivetrainSettings
{
    bool bEnabled=true;
    ElectricDrive::FSpec Drive, Steer;
    double BacklashDeg=.2;
    FDrivetrainSettings()
    {
        Steer.PeakNm=60; Steer.PowerW=800; Steer.Ratio=100; Steer.GearEfficiency=.85;
        Steer.MotorEfficiency=.85; Steer.ResponseS=.03; Steer.RotorInertiaKgM2=.0002;
    }
    bool Load(FString& Error);
    bool Valid() const { return Drive.Valid() && Steer.Valid() && FMath::IsFinite(BacklashDeg) && BacklashDeg>=0 && BacklashDeg<=3; }
};
