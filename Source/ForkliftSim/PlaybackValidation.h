#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PlaybackValidation.generated.h"
UCLASS()
class APlaybackValidation : public AActor
{
 GENERATED_BODY()
public:
 APlaybackValidation();virtual void Tick(float Dt) override;
 double Age=0,MaxDt=0,MaxGap=0,StartWall=0,OriginalRenderLimit=-1;int Stage=0,RateMask=0;bool Done=false;FVector Start;
};
