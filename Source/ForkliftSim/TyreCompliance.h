#pragma once
#include "CoreMinimal.h"
#include "TreadModel.h"

// One-axis rim/belt compliance in SERIES with rigid belt/road contact.
// These are diagnostics for the implicit Chaos joint, never extra road forces.
struct FTyreComplianceSettings
{
    bool bEnabled=true;
    double StiffnessNpm=600000, DampingNsPm=8000, TravelM=.05;
    bool bDetailed=false;
    FTreadSettings Tread;
    double CubicNpM3=0, ShearNpm=800000, ShearDampingNsPm=4000, ShearTravelM=.005;
    bool Load(FString& Error);
    bool Valid() const
    {
        return Tread.Valid() && (!Tread.bEnabled || (bEnabled && bDetailed)) && FMath::IsFinite(StiffnessNpm) && StiffnessNpm>=100000 && StiffnessNpm<=5000000
            && FMath::IsFinite(DampingNsPm) && DampingNsPm>=0 && DampingNsPm<=50000
            && FMath::IsFinite(TravelM) && TravelM>=.01 && TravelM<=.06
            && FMath::IsFinite(CubicNpM3) && CubicNpM3>=0 && CubicNpM3<=1e11
            && FMath::IsFinite(ShearNpm) && ShearNpm>=100000 && ShearNpm<=5e6
            && FMath::IsFinite(ShearDampingNsPm) && ShearDampingNsPm>=0 && ShearDampingNsPm<=50000
            && FMath::IsFinite(ShearTravelM) && ShearTravelM>=.001 && ShearTravelM<=.02;
    }
    double ForceN(double CompressionM,double RateMps) const { return StiffnessNpm*CompressionM+(bDetailed?CubicNpM3*FMath::Pow(CompressionM,3):0)+DampingNsPm*RateMps; }
    double TangentK(double D) const { return StiffnessNpm+(bDetailed?3*CubicNpM3*D*D:0); }
    double StoredJ(double CompressionM) const { return .5*StiffnessNpm*CompressionM*CompressionM+(bDetailed?.25*CubicNpM3*FMath::Pow(CompressionM,4):0); }
    double DissipationW(double RateMps) const { return DampingNsPm*RateMps*RateMps; }
};
