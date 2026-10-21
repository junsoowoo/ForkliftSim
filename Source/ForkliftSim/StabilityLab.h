#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/HUD.h"
#include "Vehicle3DSettings.h"
#include "StabilityLab.generated.h"

UCLASS()
class FORKLIFTSIM_API AStabilityLab : public AActor
{
    GENERATED_BODY()
public:
    AStabilityLab();
    virtual void BeginPlay() override;
    virtual void Tick(float Dt) override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    void ResetRig();
    void ChangeFriction(bool Tyre, double Delta);
    void ToggleRoadGrip();
    bool FrictionMaterialsMatch() const;
    void CycleStress();
    FVehicle3DSettings Settings;
    UPROPERTY() TObjectPtr<class AChassisActor> Chassis;
    UPROPERTY() TObjectPtr<class AForkActor> Fork;
    UPROPERTY() TArray<TObjectPtr<class AWheelActor>> Wheels;
    double Throttle=0, Brake=0, Steering=0, Lift=0, SteerDeg=0, LiftTargetM=0;
    double Strafe=0, TargetYawRadS=0;
    double TargetHeadingDeg=0;
    bool bAligning=false;
    double AlignmentErrorDeg=0;
    FVector2D TargetVelocity=FVector2D::ZeroVector;
    double TimeS=0, SpeedMps=0, LateralMps=0, YawDegS=0, TiltDeg=0, RollDeg=0, PitchDeg=0, ForkHeightM=0;
    double MaxTilt=0, MaxLateralSlip=0, MaxYawRate=0, MaxSpeed=0, MassKg=0, AccelerationMps2=0;
    int Grounded=0, MinGrounded=4;
    FString Error, TestMode, FrictionNotice;
    double FrictionNoticeUntil=0;
private:
    UPROPERTY() TObjectPtr<class UPhysicalMaterial> TyreMaterial;
    UPROPERTY() TObjectPtr<class UPhysicalMaterial> RoadMaterial;
    UPROPERTY() TObjectPtr<class UStaticMeshComponent> Road;
    TArray<FString> Rows;
    FString CsvPath;
    double NextRecord=0, NextFlush=5, ForkBaseZ=0, SettledTilt=0, SettledSpeed=0, MaxForkHeight=0;
    double NormalComOffset=0, NormalMaxSpeed=3, NormalSteerRate=30, NormalMaxSteer=30, MaxWheelSurfaceSpeed=0;
    double CaptureTimeS=3;
    bool bFrictionSwitched=false, bFrictionRestored=false, bFrictionKeysPassed=false;
    double InitialRoadMu=.8, MaxLiveSlip=0;
    double MinPoweredTurnSpeed=TNumericLimits<double>::Max();
    double MaxSideSpeed=0, MaxModuleError=0;
    FVector SwerveStart=FVector::ZeroVector;
    void DriveSwerve(double Dt);
    FVector PreviousVelocity=FVector::ZeroVector;
    bool bSeeded=false, bFinished=false, bCapture=false, bCaptured=false, bStress=false;
    void ApplyMaterials();
    void SeedSpeed(double Speed);
    void FinishTest();
    void Flush();
};

UCLASS()
class FORKLIFTSIM_API AStabilityController : public APlayerController
{
    GENERATED_BODY()
public:
    virtual void BeginPlay() override;
    virtual void PlayerTick(float Dt) override;
    virtual void SetupInputComponent() override;
    bool TestFrictionBindings();
private:
    UPROPERTY() TObjectPtr<class ACameraActor> LabCamera;
    void Reset(); void TyreDown(); void TyreUp(); void RoadDown(); void RoadUp(); void Stress();
    void RoadPreset();
    void ToggleView();
    bool bTopView=false;
    void RepeatTyreDown(); void RepeatTyreUp(); void RepeatRoadDown(); void RepeatRoadUp();
    void FrictionInput(bool Tyre,double Delta,bool Repeat);
    double NextFrictionRepeat=0;
};

UCLASS()
class FORKLIFTSIM_API AStabilityHUD : public AHUD
{
    GENERATED_BODY()
public:
    virtual void DrawHUD() override;
};
