#pragma once
#include "CoreMinimal.h"

// SI units. A low-speed linear transition avoids a discontinuous sign torque
// near rest; this is a dissipative approximation, not a static-friction solver.
namespace WheelResistance
{
    // A negative instantaneous power is insufficient for an explicit torque:
    // an oversized impulse can cross zero and increase rotational KE. Limit
    // the resisting angular impulse using the wheel's actual world inertia.
    // Ordinary (unsaturated) Crr torques are unchanged.
    inline double PassiveStepTorque(double TorqueNm,double Omega,double InverseInertia,double Dt)
    {
        if(Dt<=0||InverseInertia<=0||TorqueNm*Omega>=0)return 0;
        const double Cap=.8*FMath::Abs(Omega)/(InverseInertia*Dt);
        return FMath::Clamp(TorqueNm,-Cap,Cap);
    }
    inline FQuat TwistTarget(double MeasuredDegrees)
    {
        // GetCurrentTwist is the signed quaternion twist of Frame1 relative
        // to Frame2. FRotator's positive Roll produces NEGATIVE quaternion X;
        // passing that angle through Roll would reverse the captured rest pose.
        return FQuat(FVector::ForwardVector,FMath::DegreesToRadians(MeasuredDegrees));
    }
    struct FTorques { double Rolling=0, Bearing=0; };
    inline FTorques Evaluate(double LoadN,double RadiusM,double OmegaRadS,
        double BearingRelativeRadS,double Crr,double BearingCoulombNm,double BearingViscousNms)
    {
        constexpr double TransitionRadS=.5;
        FTorques T;
        T.Rolling=-Crr*FMath::Max(0.,LoadN)*RadiusM*FMath::Clamp(OmegaRadS/TransitionRadS,-1.,1.);
        T.Bearing=-BearingCoulombNm*FMath::Clamp(BearingRelativeRadS/TransitionRadS,-1.,1.)-BearingViscousNms*BearingRelativeRadS;
        return T;
    }
}
