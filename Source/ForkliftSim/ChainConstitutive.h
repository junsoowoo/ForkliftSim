#pragma once
#include "CoreMinimal.h"

namespace ChainConstitutive
{
    struct FResponse { double TensionN, StoredJ, LossW; };
    inline FResponse Evaluate(double K,double D,double Stretch,double Rate)
    {
        // Preserve the existing unilateral Kelvin-Voigt force. When the rope
        // unloads to zero tension, D*rate^2 alone OVERCOUNTS its damping.
        const double Elastic=K*FMath::Max(0.,Stretch);
        const double Tension=FMath::Max(0.,K*Stretch+D*Rate);
        return {Tension,.5*K*FMath::Square(FMath::Max(0.,Stretch)),(Tension-Elastic)*Rate};
    }
}
