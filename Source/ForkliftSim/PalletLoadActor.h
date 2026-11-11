#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PalletLoadActor.generated.h"

struct FPayloadSettings
{
    double PalletMassKg=25, CargoMassKg=400, LengthM=1.2, WidthM=1.2, HoleHeightM=.18;
    double HoleSpacingM=.6, HoleWidthM=.32, HoleShiftYM=0, ForkSpacingM=.6, ForkShiftYM=0;
    double CargoXM=0, CargoYM=0, CargoHeightM=.6, PalletYM=0, InitialDepthM=-.25;
    double ForkMu=.45, LeftHoleMu=.45, RightHoleMu=.45, CargoMu=.5;
    double CargoLengthM=.65, CargoWidthM=.65;
    double PalletTopStaticMu=.6, PalletTopDynamicMu=.5, CargoStaticMu=.6, CargoDynamicMu=.5;
    bool bCargoSecured=false;
    // Local Fab presentation uses board-sized contact boxes, never one solid hull.
    bool bWarehouseAssets=false;
    FVector PalletDatumM=FVector::ZeroVector, CargoBoundsMinM=FVector::ZeroVector;
    double PalletTopM=0;
    FString PalletMeshPath,CargoMeshPath;
    TArray<FVector> BoardCentersM,BoardSizesM;
    bool UseWarehouseGeometry(FString& Error);
    bool Load(FString& Error);
    bool Validate(FString& Error) const;
};

UCLASS()
class FORKLIFTSIM_API APalletLoadActor : public AActor
{
    GENERATED_BODY()
public:
    APalletLoadActor();
    void Configure(const FPayloadSettings& S);
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    FVector LoadCOM() const;
    double LoadMass() const;
    UPROPERTY() TObjectPtr<class UStaticMeshComponent> Deck;
    UPROPERTY() TObjectPtr<class AStaticMeshActor> Cargo;
    UPROPERTY() TObjectPtr<class UPhysicsConstraintComponent> Strap;
};
