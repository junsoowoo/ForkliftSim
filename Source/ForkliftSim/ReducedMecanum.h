#pragma once
#include "CoreMinimal.h"

// Rank-one, velocity-regularized Coulomb contact. Passive roller coordinates
// are eliminated; this does not reproduce discrete barrel impacts or spin.
namespace ReducedMecanum
{
    // The equivalent roller law is a bottom/tread model. A cap hitting a step
    // wall is an ordinary solid contact, not another load-bearing roller patch.
    inline bool IsRollingContact(FVector Normal,FVector Axle)
    { return Normal.Z>.5 && FMath::Abs(FVector::DotProduct(Normal,Axle))<.5; }
    inline FVector ContactDirection(FVector Axle,FVector Normal,double Hand)
    { return (FVector::CrossProduct(Axle,Normal)+Hand*FVector::VectorPlaneProject(Axle,Normal)).GetSafeNormal(); }
    struct FBrakeStep {double Impulse=0,Shear=0;};
    // Dissipative dashpot followed by an implicit elastic/plastic bristle.
    // The total impulse stays in the Coulomb interval. Return mapping acts on
    // the spring impulse, never stores the clipped damper force as elastic energy.
    inline FBrakeStep BrakeStep(double V,double OldShear,double InvMass,double Dt,double K,double C,double ForceLimit)
    {
        FBrakeStep Out;if(Dt<=0||InvMass<=0||K<=0)return Out;
        const double L=FMath::Max(0.,ForceLimit)*Dt;
        const double D=FMath::Clamp(-C*Dt*V/(1+C*Dt*InvMass),-L,L);
        const double VD=V+InvMass*D;
        const double Trial=-K*Dt*(OldShear+Dt*VD)/(1+K*Dt*Dt*InvMass);
        const double Spring=FMath::Clamp(Trial,-L-D,L-D);
        Out.Impulse=D+Spring;Out.Shear=-Spring/(K*Dt);return Out;
    }
    // A wheel is constrained to its braked shaft and chassis. The free-body
    // predictor is NOT its final displacement. Update storage with the solved
    // contact motion, bounded by the available spring energy minus port work.
    inline double SolvedShear(double OldShear,double MeanSlip,double Dt,double K,double ForceLimit,double PortWork)
    {
        if(Dt<=0||K<=0)return 0;
        const double Budget=FMath::Max(0.,.5*K*OldShear*OldShear-PortWork);
        const double MaxShear=FMath::Min(FMath::Max(0.,ForceLimit)/K,FMath::Sqrt(2*Budget/K));
        return FMath::Clamp(OldShear+Dt*MeanSlip,-MaxShear,MaxShear);
    }
    inline FVector Force(FVector Slip,FVector DriveDirection,double NormalN,double Mu,double InverseMass,double Dt)
    {
        if(NormalN<=0 || Mu<=0 || InverseMass<=0 || Dt<=0)return FVector::ZeroVector;
        const double V=FVector::DotProduct(Slip,DriveDirection);
        const double Magnitude=FMath::Clamp(-.8*V/(InverseMass*Dt),-Mu*NormalN,Mu*NormalN);
        return DriveDirection*Magnitude;
    }
}
