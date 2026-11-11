#pragma once
#include "CoreMinimal.h"

// Published geometry/strength and explicitly identified design assumptions.
// A component reference does not identify the OEM supplier of an AGV.
struct FForkStructureSettings
{
    bool bEnabled=false;
    double LengthM=1.4, WidthM=.12, ThicknessM=.07;
    double YoungPa=210e9, YieldPa=825e6, BreakPa=1100e6, DampingRatio=.08;
    double DensityKgM3=7850;
    double FailurePlasticDeg=10;
    double HardeningRatio=.05;
    double BladeMassKg() const { return LengthM*WidthM*ThicknessM*DensityKgM3; }
    int32 Segments=4;
    double I() const { return WidthM*FMath::Pow(ThicknessM,3)/12.; }
    double SectionModulusM3() const { return WidthM*ThicknessM*ThicknessM/6.; }
    double JointK(int32 Index) const { return YoungPa*I()/(LengthM/Segments)*(Index==0?2:1); }
    double YieldMomentNm() const { return YieldPa*SectionModulusM3(); }
    double BreakMomentNm() const { return BreakPa*SectionModulusM3(); }
    bool Load(FString& Error);
    bool Valid() const;
};
