#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Vehicle3DSettings.h"
#include "ForkActor.generated.h"

UCLASS()
class FORKLIFTSIM_API AForkActor : public AActor
{
    GENERATED_BODY()
public:
    AForkActor();
    void Configure(class UStaticMeshComponent* Chassis,const FVehicle3DSettings& S);
    void SetLiftTarget(double HeightM);
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UStaticMeshComponent> Carriage;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UPhysicsConstraintComponent> Guide;
    double TravelM=1.5;
};
