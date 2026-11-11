#pragma once
#include "CoreMinimal.h"
namespace ValidationMath
{
    // Clip a measured rising position interval to a fixed height window.
    // Fractional boundary frames avoid endpoint sample-phase bias.
    inline void RisingWindow(double Previous,double Current,double Dt,double Low,double High,double& Distance,double& Duration)
    {
        if(Dt<=0 || Current<=Previous || High<=Low) return;
        const double Inside=FMath::Max(0.,FMath::Min(Current,High)-FMath::Max(Previous,Low));
        Distance+=Inside; Duration+=Dt*Inside/(Current-Previous);
    }
}
