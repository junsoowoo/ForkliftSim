#pragma once
#include "CoreMinimal.h"

namespace RigidSpin
{
    // Bearing couple needed to carry a freely spinning symmetric roller with
    // the hub's rotating axis. For W = hubW + spin*a, L = Iperp*hubW + c*a,
    // and dL/dt = c*(hubW x a). It has no component along the free axle.
    inline FVector BearingTransport(FQuat MassQ,FVector W,FVector I,FVector HubW)
    {
        const FVector A=MassQ.GetAxisX();
        const double Perp=(I.Y+I.Z)*.5;
        return (I.X*FVector::DotProduct(W,A)-Perp*FVector::DotProduct(HubW,A))*FVector::CrossProduct(HubW,A);
    }
    // Exact torque-free flow of a symmetric top, in its principal mass frame.
    // Q maps mass coordinates to world; W is world angular velocity. This
    // advances both pose and velocity, preserving the body's L and kinetic E.
    // It never reads a system momentum/energy residual or rescales a speed.
    inline bool Symmetric(FQuat& Q,FVector& W,FVector I,double Dt)
    {
        if(I.GetMin()<=0 || Dt<0 || !FMath::IsFinite(Dt)) return false;
        int Axis=-1;
        for(int K=0;K<3;++K) if(FMath::Abs(I[(K+1)%3]-I[(K+2)%3])<I.GetMax()*1e-6) {Axis=K;break;}
        if(Axis<0) return false;
        const double Parallel=I[Axis],Perp=(I[(Axis+1)%3]+I[(Axis+2)%3])*.5;
        FVector LocalAxis=FVector::ZeroVector;LocalAxis[Axis]=1;
        const FVector L=Q.RotateVector(I*Q.UnrotateVector(W));
        const double Magnitude=L.Size(),Axial=FVector::DotProduct(L,Q.RotateVector(LocalAxis));
        if(Magnitude>1e-15) Q=FQuat(L/Magnitude,Magnitude/Perp*Dt)*Q*FQuat(LocalAxis,(1/Parallel-1/Perp)*Axial*Dt);
        Q.Normalize();W=Q.RotateVector(Q.UnrotateVector(L)/I);
        return true;
    }
}
