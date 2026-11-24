#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/HUD.h"
#include "WarehouseCampus.generated.h"
class UHierarchicalInstancedStaticMeshComponent;
UCLASS()
class FORKLIFTSIM_API AWarehouseMapGroup : public AActor
{
    GENERATED_BODY()
public:
    AWarehouseMapGroup();
    UPROPERTY(VisibleAnywhere) TObjectPtr<UHierarchicalInstancedStaticMeshComponent> Instances;
    UPROPERTY(EditAnywhere,Category="Layout") TObjectPtr<UStaticMesh> LayoutMesh;
    UPROPERTY(EditAnywhere,Category="Layout") TObjectPtr<UMaterialInterface> LayoutMaterial;
    UPROPERTY(EditAnywhere,Category="Layout") TArray<FTransform> LayoutTransforms;
    UPROPERTY(EditAnywhere,Category="Layout") TArray<FString> LayoutRecordIds;
    bool RemoveRecord(const FString& Id);
    UPROPERTY(EditAnywhere,Category="Layout") bool Blocking=false;
    UPROPERTY(EditAnywhere,Category="Layout") bool CastLayoutShadow=false;
    UPROPERTY(EditAnywhere,Category="Layout") bool InvisibleProxy=false;
    UFUNCTION(BlueprintCallable,CallInEditor,Category="Layout") void Rebuild();
    virtual void OnConstruction(const FTransform& Transform) override;
};
UCLASS()
class FORKLIFTSIM_API AWarehouseMapController : public APlayerController
{
    GENERATED_BODY()
public:
    virtual void BeginPlay() override;
    bool ManualVehicle=true;
    virtual void PlayerTick(float DeltaSeconds) override;
    virtual void SetupInputComponent() override;
private:
    bool InitialView=false,RoofHidden=true,CaptureRequested=false;
    double StartedWall=0,FrameWall=0,FrameSum=0;int Frames=0;
    FString CaptureName,CapturePath;
    void StartJob();void StartFleet();void PauseJob();void ResumeJob();void CancelJob();void LowTier();void HighTier();
    void NextVehicle();void FollowVehicle();void NextLoad();void SensorRays();void ForkRays();void CameraToggle();void RearMode();void Estop();void Ack();
    void View(int Mode);void Overview();void Yard();void Interior();void Service();void ToggleRoof();
};
UCLASS()
class FORKLIFTSIM_API AWarehouseMapHUD : public AHUD
{
    GENERATED_BODY()
public: virtual void DrawHUD() override;
};
UCLASS()
class FORKLIFTSIM_API AWarehouseMapGameMode : public AGameModeBase
{
    GENERATED_BODY()
public: AWarehouseMapGameMode();virtual void BeginPlay() override;
};
