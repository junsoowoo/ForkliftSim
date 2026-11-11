#pragma once
#include "CoreMinimal.h"

namespace CameraMounts
{
    // Above the blades, pitched down: both entry channels can be observed
    // before insertion. A carried load can occlude this view; consumers must
    // expose that loss instead of substituting a pallet actor transform.
    inline const FVector FrontCm(5,-10,10);
    inline const FRotator FrontRotation(-10,0,0);
    inline const FVector RearCm(-60,0,70);
}
