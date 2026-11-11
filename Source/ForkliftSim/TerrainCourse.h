#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TerrainCourse.generated.h"

UCLASS()
class FORKLIFTSIM_API ATerrainCourse : public AActor
{
    GENERATED_BODY()
public:
    ATerrainCourse();
    void Configure(const FString& Preset,double Mu,double ApproachOffsetM=0);
    void SetMu(double Mu);
    UPROPERTY() TArray<TObjectPtr<class UStaticMeshComponent>> Surfaces;
    UPROPERTY() TArray<TObjectPtr<class UPhysicalMaterial>> Materials;
    FString Mode;
    double SlopeDeg=6, StepM=.025;
private:
    void Box(FVector TopCenterM,FVector SizeM,FRotator Rotation,FLinearColor Color);
};
