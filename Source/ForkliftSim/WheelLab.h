#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/HUD.h"
#include "WheelModel.h"
#include "WheelLab.generated.h"

UCLASS()
class FORKLIFTSIM_API AWheelLab : public AActor
{
    GENERATED_BODY()
public:
    AWheelLab();
    virtual void BeginPlay() override;
    virtual void Tick(float Dt) override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    void ResetLab();
    void CycleFriction();
    void CycleMass();
    void CycleTorque();
    void FlushTelemetry();
    FWheelVehicleModel Model;
    double Throttle = 0, Brake = 0, Steering = 0, AccelerationMps2 = 0;
    FString Error;
private:
    UPROPERTY() TArray<TObjectPtr<USceneComponent>> SteerPivots;
    UPROPERTY() TArray<TObjectPtr<USceneComponent>> SpinPivots;
    TArray<FString> Rows;
    FString CsvPath, TestMode;
    double Accumulator = 0, NextRecord = 0, NextFlush = 5, MaximumSpeed = 0;
    double MaxSlip = 0, BrakeStartX = 0, StopDistanceM = -1;
    bool bCapture = false, bCaptured = false, bFinished = false, bBraking = false;
    void BuildVisuals();
    void Record();
    void CheckTest();
};

UCLASS()
class FORKLIFTSIM_API AWheelLabController : public APlayerController
{
    GENERATED_BODY()
public:
    virtual void BeginPlay() override;
    virtual void PlayerTick(float Dt) override;
    virtual void SetupInputComponent() override;
private:
    UPROPERTY() TObjectPtr<class ACameraActor> LabCamera;
    bool bSideView = false;
    void Reset(); void Friction(); void Mass(); void Torque(); void ChangeCamera();
};

UCLASS()
class FORKLIFTSIM_API AWheelLabHUD : public AHUD
{
    GENERATED_BODY()
public:
    virtual void DrawHUD() override;
};
