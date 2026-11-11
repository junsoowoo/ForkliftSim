#pragma once
#include "CoreMinimal.h"

// Solver impulse diagnostics only: no synthetic area, pressure, extra force or
// force clipping. Bins are across the rotating tyre's axle (- / middle / +).
struct FWheelContactData
{
    double NormalNs=0, BandNs[3]={0,0,0};
    FVector PointWeightedMNs=FVector::ZeroVector, NormalImpulseNs=FVector::ZeroVector;
    FVector NormalMomentNms=FVector::ZeroVector;
    int32 PeakActivePoints=0;
    void AddPoint(const FVector& PointM,const FVector& CenterM,const FVector& Axis,
        const FVector& OutwardNormal,double J,double HalfWidthM)
    {
        if(J<=0 || HalfWidthM<=0) return;
        NormalNs+=J; PointWeightedMNs+=PointM*J; NormalImpulseNs+=OutwardNormal*J;
        NormalMomentNms+=FVector::CrossProduct(PointM-CenterM,OutwardNormal*J);
        const double U=FVector::DotProduct(PointM-CenterM,Axis)/HalfWidthM;
        BandNs[U< -1./3?0:U>1./3?2:1]+=J;
    }
    void Add(const FWheelContactData& B)
    {
        NormalNs+=B.NormalNs; PointWeightedMNs+=B.PointWeightedMNs; NormalImpulseNs+=B.NormalImpulseNs;
        NormalMomentNms+=B.NormalMomentNms; PeakActivePoints=FMath::Max(PeakActivePoints,B.PeakActivePoints);
        for(int I=0;I<3;++I) BandNs[I]+=B.BandNs[I];
    }
};
