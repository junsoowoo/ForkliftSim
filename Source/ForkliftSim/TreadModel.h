#pragma once
#include "CoreMinimal.h"
#include <cmath>

// Discrete, bonded radial foundation. Coefficients are design assumptions, NOT
// a continuum rubber law. Area scaling keeps the law independent of pad count.
struct FTreadSettings
{
    bool bEnabled=false;
    bool bCoupled=false;
    int32 Count=16;
    // Fixture-only rotation of the tessellation relative to the wheel core.
    double PhaseRad=0;
    double DepthM=.05, TravelM=.025, MassKg=6;
    double FoundationNpm3=1.e8, FoundationDampingNspm3=2.e5;
    double Poisson=.49; // design assumption, not a measured Vulkollan value
    double PadArea(double R,double W) const { return 2*UE_DOUBLE_PI*R*W/Count; }
    double PadK(double R,double W) const { return FoundationNpm3*PadArea(R,W); }
    double PadC(double R,double W) const { return FoundationDampingNspm3*PadArea(R,W); }
    double ShearRatio() const { return 1/(2*(1+Poisson)); }
    double PadShearK(double R,double W) const { return PadK(R,W)*ShearRatio(); }
    double PadShearC(double R,double W) const { return PadC(R,W)*ShearRatio(); }
    double PadRotK(double R,double W) const { return PadK(R,W)*FMath::Square(2*UE_DOUBLE_PI*R/Count)/12; }
    double PadRotC(double R,double W) const { return PadC(R,W)*FMath::Square(2*UE_DOUBLE_PI*R/Count)/12; }
    double LinkK(double R,double W) const { return FoundationNpm3*DepthM*ShearRatio()*W*DepthM/(2*UE_DOUBLE_PI*(R-DepthM*.5)/Count); }
    double LinkC(double R,double W) const { return FoundationDampingNspm3*DepthM*ShearRatio()*W*DepthM/(2*UE_DOUBLE_PI*(R-DepthM*.5)/Count); }
    double BendK(double R,double W) const { return FoundationNpm3*DepthM*W*FMath::Pow(DepthM,3)/(12*(2*UE_DOUBLE_PI*(R-DepthM*.5)/Count)); }
    double BendC(double R,double W) const { return FoundationDampingNspm3*DepthM*W*FMath::Pow(DepthM,3)/(12*(2*UE_DOUBLE_PI*(R-DepthM*.5)/Count)); }
    bool Valid() const
    {
        return Count>=8 && Count<=(bCoupled?64:32) && Count%4==0 && FMath::IsFinite(Poisson) && Poisson>=0 && Poisson<.5 && FMath::IsFinite(DepthM) && DepthM>=.02 && DepthM<=.08
            && FMath::IsFinite(TravelM) && TravelM>=.005 && TravelM<DepthM*.8
            && FMath::IsFinite(MassKg) && MassKg>=1 && MassKg<=20
            && FMath::IsFinite(FoundationNpm3) && FoundationNpm3>=1.e7 && FoundationNpm3<=1.e9
            && FMath::IsFinite(FoundationDampingNspm3) && FoundationDampingNspm3>=0 && FoundationDampingNspm3<=1.e6;
    }
    // Fixed hub, zero friction, horizontal floor. Finite convex sectors,
    // radial-only travel: N = k*overlap/cos(theta)^2 + pad weight.
    // This is independent of the Chaos solver and includes sector extent.
    double FlatSupportN(double R,double W,double HubHeight) const
    {
        double F=0;
        for(int I=0;I<Count;++I)
        {
            // UE 5.6 UnwindRadians<T> subtracts float UE_TWO_PI even for T=double.
            const double Angle=FMath::Abs(std::remainder(2*UE_DOUBLE_PI*I/Count+PhaseRad,2*UE_DOUBLE_PI)), Down=FMath::Cos(Angle);
            const double Extent=R*FMath::Cos(FMath::Max(0.,Angle-UE_DOUBLE_PI/Count));
            const double Overlap=Extent-HubHeight;
            if(Down>1e-6 && Overlap>0) F+=PadK(R,W)*Overlap/(Down*Down)+MassKg/Count*9.81;
        }
        return F;
    }
    // Zero-width radial-foundation limit, NOT measured rubber behaviour.
    // Integrate k_A R W (R sec(theta)-H sec(theta)^2) over the contact arc.
    double ContinuousSupportN(double R,double W,double H) const
    {
        if(H>=R) return 0;
        if(H<=0) return TNumericLimits<double>::Max();
        const double A=FMath::Acos(H/R), Tan=FMath::Tan(A);
        return 2*FoundationNpm3*R*W*(R*FMath::Loge(1/FMath::Cos(A)+Tan)-H*Tan)+MassKg*9.81*A/UE_DOUBLE_PI;
    }
};
