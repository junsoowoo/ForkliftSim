#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "JX2000AExterior.generated.h"

class UStaticMeshComponent;

/** Chinese product 95 exterior. No drivetrain, collision or lift behaviour is implied. */
UCLASS(Blueprintable)
class FORKLIFTSIM_API AJX2000AExterior : public AActor
{
    GENERATED_BODY()
public:
    AJX2000AExterior();
    virtual void OnConstruction(const FTransform& Transform) override;

    UFUNCTION(CallInEditor, Category="JX-2000A")
    void RebuildExterior();

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="JX-2000A")
    TArray<TObjectPtr<UStaticMeshComponent>> ExteriorParts;
};
