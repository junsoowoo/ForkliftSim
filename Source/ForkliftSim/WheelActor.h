#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/StaticMeshComponent.h"
#include "Vehicle3DSettings.h"
#include "WheelActor.generated.h"

UCLASS()
class FORKLIFTSIM_API UWheelTyreComponent : public UStaticMeshComponent
{
    GENERATED_BODY()
public:
    void BuildRollingCollision();
    virtual class UBodySetup* GetBodySetup() override;
private:
    UPROPERTY() TObjectPtr<class UBodySetup> RollingCollision;
};

UCLASS()
class FORKLIFTSIM_API AWheelActor : public AActor
{
    GENERATED_BODY()
public:
    AWheelActor();
    void Configure(class UStaticMeshComponent* Chassis,const FVehicle3DSettings& S,bool Rear,class UPhysicalMaterial* Material);
    void Drive(double TorqueNm,double BrakeNm,double SteeringDeg);
    void SetMu(class UPhysicalMaterial* Material);
    void Measure(double RadiusM,double WidthM);
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UStaticMeshComponent> Knuckle;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UStaticMeshComponent> Tyre;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UPhysicsConstraintComponent> SteeringJoint;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UPhysicsConstraintComponent> Axle;
    UPROPERTY() TObjectPtr<class UPhysicsConstraintComponent> ChassisCollisionFilter;
    bool bRear=false, bGrounded=false;
    bool bSteerable=false;
    double SteerTargetDeg=0, SpeedTargetMps=0;
    double OmegaRadS=0, LongSlipMps=0, LateralSlipMps=0, ClearanceM=0, SteerActualDeg=0;
    double DriveTorqueNm=0, BrakeLimitNm=0;
    int32 ChassisContactCount=0;
private:
    UPROPERTY() TObjectPtr<class UStaticMeshComponent> MotorStator;
    UFUNCTION() void OnTyreHit(UPrimitiveComponent* HitComponent,AActor* OtherActor,UPrimitiveComponent* OtherComponent,FVector NormalImpulse,const FHitResult& Hit);
};
