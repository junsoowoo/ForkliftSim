#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Vehicle3DSettings.h"
#include "ChassisActor.generated.h"

UCLASS()
class FORKLIFTSIM_API AChassisActor : public AActor
{
    GENERATED_BODY()
public:
    AChassisActor();
    void Configure(const FVehicle3DSettings& S);
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UStaticMeshComponent> Body;
};
