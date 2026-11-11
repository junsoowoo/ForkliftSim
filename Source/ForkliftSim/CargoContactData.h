#pragma once
#include "CoreMinimal.h"

// Physics-thread output: impulses in N*s, weighted points in m*N*s.
// All vectors are world-space and impulses act ON the cargo.
struct FCargoContactData
{
    FVector ImpulseNs=FVector::ZeroVector, FrictionImpulseNs=FVector::ZeroVector;
    FVector NormalImpulseNs=FVector::ZeroVector, PointWeightedMNs=FVector::ZeroVector;
    double NormalNs=0, FrictionMagnitudeNs=0, GroundNormalNs=0, OtherNormalNs=0;
    int32 Points=0;
    void Add(const FCargoContactData& O)
    {
        ImpulseNs+=O.ImpulseNs; FrictionImpulseNs+=O.FrictionImpulseNs; NormalImpulseNs+=O.NormalImpulseNs;
        PointWeightedMNs+=O.PointWeightedMNs; NormalNs+=O.NormalNs; FrictionMagnitudeNs+=O.FrictionMagnitudeNs;
        GroundNormalNs+=O.GroundNormalNs; OtherNormalNs+=O.OtherNormalNs; Points+=O.Points;
    }
};
