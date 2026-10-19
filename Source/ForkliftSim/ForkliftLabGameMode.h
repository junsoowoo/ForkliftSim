#pragma once
#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/HUD.h"
#include "ForkliftLabGameMode.generated.h"

UCLASS()
class FORKLIFTSIM_API AForkliftLabGameMode : public AGameModeBase
{
    GENERATED_BODY()
public:
    AForkliftLabGameMode();
    virtual void BeginPlay() override;
};

UCLASS()
class FORKLIFTSIM_API AForkliftLabController : public APlayerController
{
    GENERATED_BODY()
public:
    virtual void BeginPlay() override;
    virtual void SetupInputComponent() override;
private:
    void StartCycle();
    void Reset();
    void ChangeMass();
    void ChangeFriction();
    void ChangeForce();
    void FailSensor();
    void ChangeCamera();
    UPROPERTY() TObjectPtr<class ACameraActor> LabCamera;
    bool bSideView = false;
};

UCLASS()
class FORKLIFTSIM_API AForkliftLabHUD : public AHUD
{
    GENERATED_BODY()
public:
    virtual void DrawHUD() override;
};
