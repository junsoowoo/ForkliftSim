#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Vehicle3DSettings.h"
#include "DynamicsSensors.h"
#include "ForkActor.generated.h"

UCLASS()
class FORKLIFTSIM_API AForkTineActor : public AActor
{
    GENERATED_BODY()
public:
    AForkTineActor();
    void Configure(class UStaticMeshComponent* Carriage,double MassKg,double Mu,const FForkStructureSettings& Spec=FForkStructureSettings());
    void UpdateStructure();
    FVector Heel() const;
    FVector Tip() const;
    FForkStructureSettings Structure;
    double MaxStressPa=0, TipDeflectionM=0, PlasticDeg=0;
    double PlasticRelaxationJ=0,FractureReleaseJ=0;
    double ElasticEnergyJ() const;
    bool bFailed=false;
    UPROPERTY() TArray<TObjectPtr<class UStaticMeshComponent>> Sections;
    UPROPERTY() TArray<TObjectPtr<class UPhysicsConstraintComponent>> Bends;
    UPROPERTY() TObjectPtr<class UStaticMeshComponent> Body;
    UPROPERTY() TObjectPtr<class UPhysicsConstraintComponent> Mount;
private:
    UPROPERTY() TObjectPtr<class UStaticMeshComponent> Carrier;
    FVector HeelLocal=FVector::ZeroVector;
    TArray<double> RestPitchDeg;
    TArray<double> AccumulatedPlasticDeg;
};

UCLASS()
class FORKLIFTSIM_API AForkActor : public AActor
{
    GENERATED_BODY()
public:
    AForkActor();
    void Configure(class UStaticMeshComponent* Chassis,const FVehicle3DSettings& S,const struct FPayloadSettings* Payload=nullptr);
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    UPROPERTY() TArray<TObjectPtr<AForkTineActor>> Tines;
    void SetLiftTarget(double HeightM);
    void UpdateHydraulics();
    void SetValveCommand(const FHydraulicCommand& Command,double ValidUntilS);
    bool bPressureSensor=false;double PressureSamplePeriodS=.01;
    TArray<DynamicsSensor::FTruth> PressureSamples;
    FHydraulicCommand ValveControl;double ValveControlValidUntilS=0;bool HydraulicCommandExpired=false;
    FHydraulicSettings HydraulicSettings;
    FHydraulicState HydraulicState;
    double HydraulicHeightM=0, HydraulicVelocityMps=0;
    double HydraulicFramePeakPressurePa=0, HydraulicFramePeakDcW=0, HydraulicFrameReliefM3=0;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UStaticMeshComponent> Carriage;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UPhysicsConstraintComponent> Guide;
    UPROPERTY() TObjectPtr<class UStaticMeshComponent> MovingMast;
    UPROPERTY() TObjectPtr<class UPhysicsConstraintComponent> MastGuide;
    double TravelM=1.5;
private:
    UPROPERTY() TObjectPtr<class UStaticMeshComponent> LiftBase;
    class FHydraulicCallback* HydraulicCallback=nullptr;
    double TargetM=0, BaseZCm=0, ForceCapN=15000, InitialPressurePa=0;
};
