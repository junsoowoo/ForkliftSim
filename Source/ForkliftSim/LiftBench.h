#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "LiftModel.h"
#include "LiftBench.generated.h"

class UStaticMeshComponent;
class UPhysicsConstraintComponent;
class UPhysicalMaterial;

enum class ELiftLabPhase : uint8 { Ready, Raising, Holding, Lowering, Complete, Fault };

UCLASS()
class FORKLIFTSIM_API ALiftBench : public AActor
{
    GENERATED_BODY()
public:
    ALiftBench();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    void StartCycle();
    void ResetBench();
    void CyclePayload();
    void CycleFriction();
    void CycleForce();
    void ToggleSensorFailure();
    const TCHAR* PhaseLabel() const;

    FLiftSettings Settings;
    FLiftSensorReading Sensor;
    ELiftLabPhase Phase = ELiftLabPhase::Ready;
    FString FaultReason;
    FString TelemetryPath;
    double SimTimeS = 0;
    double ActualHeightM = 0;
    double ActualVelocityMps = 0;
    double ActualAccelerationMps2 = 0;
    double TargetHeightM = 0;
    double AppliedLiftForceN = 0;
    double GuideResistanceN = 0;
    double PalletRiseM = 0;
    double MaxPalletRiseM = 0;
    bool bSensorFailure = false;
private:
    UPROPERTY() TObjectPtr<UStaticMeshComponent> Mast;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> Carriage;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> LeftFork;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> RightFork;
    UPROPERTY() TObjectPtr<UPhysicsConstraintComponent> Rail;
    UPROPERTY() TObjectPtr<AActor> LoadActor;
    UPROPERTY() TObjectPtr<UStaticMeshComponent> Pallet;
    UPROPERTY() TObjectPtr<UPhysicalMaterial> ContactMaterial;
    FLiftForceController Controller;
    TArray<FLiftSensorReading> SensorQueue;
    TArray<FString> CsvRows;
    FString TestMode;
    FVector InitialCarriagePosition;
    FVector InitialPalletPosition;
    double LastSensorSampleS = -1;
    double LastCsvSampleS = -1;
    double LastActualVelocityMps = 0;
    double HoldTimeS = 0;
    double StallTimeS = 0;
    double MaxPalletDriftM = 0;
    bool bTestFinished = false;
    bool bCaptureRequested = false;
    bool bCaptureMode = false;
    void ConfigurePhysics();
    void BuildPallet();
    void SampleSensors();
    void UpdateMission(double DeltaS);
    void EnterFault(const FString& Reason);
    void FlushTelemetry();
    void EvaluateSmokeTest();
};
