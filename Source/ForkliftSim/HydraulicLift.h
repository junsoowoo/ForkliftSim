#pragma once
#include "CoreMinimal.h"

// SI, gauge pressure. One direct 1:1 single-acting cylinder; gravity return.
// Effective bulk modulus includes hose/entrained-air compliance, not pure oil.
struct FHydraulicSettings
{
    bool bEnabled=true;
    double AreaM2=.002, DeadVolumeM3=.002, BulkPa=200000000;
    double ReliefPa=7500000, PumpFlowM3s=.0005, MotorW=1200, Efficiency=.8;
    double ReturnM3sSqrtPa=.00000025, LeakM3sPa=0;
    double SealN=80, ViscousNsPm=400, ValveResponseS=.08, ValveGainPerM=6, DeadbandM=.003;
    bool bDetailed=false, bChainMast=false;
    double ChainRatio=2, MastMassKg=30, ChainK=1000000, ChainD=6000;
    double HoseLengthM=1.5, HoseBoreM=.008, PumpVolumeM3=.001;
    double OilDensityKgM3=856, Nu40Cst=46, Nu100Cst=6.9;
    double InitialTempC=40, AmbientC=25, ThermalMassJpK=16000, CoolingWpK=12;
    double ViscosityCst(double TempC) const;
    bool Load(FString& Error);
    bool Valid() const;
};
// External valve control bypasses ONLY the legacy GT position servo. Physical
// stroke, velocity, pressure, chains, relief and force integration still apply.
struct FHydraulicCommand
{
    bool Direct=false,PumpEnabled=true;double Spool=0;
};
inline FHydraulicCommand HydraulicLease(FHydraulicCommand Command,double ValidUntilS,double NowS)
{if(Command.Direct&&(!FMath::IsFinite(ValidUntilS)||NowS>ValidUntilS)){Command.Spool=0;Command.PumpEnabled=false;}return Command;}
struct FHydraulicState
{
    double PressurePa=0, Valve=0, PreviousTarget=0;
    double PumpM3s=0, ReturnM3s=0, LeakM3s=0, ReliefM3s=0;
    double ForceN=0, DcW=0, HydraulicW=0, MechanicalW=0, HeatW=0;
    double DcJ=0, MechanicalJ=0, HeatJ=0;
    double OilTempC=40, ViscosityCst=46, PumpPressurePa=0, PeakLinePressurePa=0, LineHeatW=0;
    double LinePressurePa[5]={0,0,0,0,0}, LineFlowM3s[4]={0,0,0,0};
    double ChainTensionN=0, ChainStretchM=0, ChainStoredJ=0, MastHeightM=0;
    double ChainLossW=0, ChainLossJ=0;
    bool bDetailedInitialized=false, bPressureFloorHit=false;
    bool bHolding=true;
    // No mass/gravity compensation and no force based on declared payload mass.
    double ValveRequest(const FHydraulicSettings& C,double Target,double Height,double Travel,const FHydraulicCommand& Control);
    void Step(const FHydraulicSettings& C,double Dt,double Target,double Height,double Velocity,double Travel,double ForceCapN,const FHydraulicCommand& Control=FHydraulicCommand());
    void StepDetailed(const FHydraulicSettings& C,double Dt,double Target,double Height,double Velocity,double Travel,double ForceCapN,const FHydraulicCommand& Control=FHydraulicCommand());
    double StoredJ(const FHydraulicSettings& C,double Height) const
    {
        const double CylinderHeight=C.bChainMast?MastHeightM:Height;
        const double Volume=C.DeadVolumeM3+C.AreaM2*FMath::Max(0.,CylinderHeight);
        if(!C.bDetailed) return .5*Volume*PressurePa*PressurePa/C.BulkPa;
        const double A=PI*C.HoseBoreM*C.HoseBoreM*.25, Dx=C.HoseLengthM/4;
        double E=0;
        for(int I=0;I<5;++I) E+=.5*(I==0?C.PumpVolumeM3+A*Dx*.5:I==4?Volume+A*Dx*.5:A*Dx)*LinePressurePa[I]*LinePressurePa[I]/C.BulkPa;
        for(double Q:LineFlowM3s) E+=.5*C.OilDensityKgM3*Dx/A*Q*Q;
        return E+ChainStoredJ;
    }
};
