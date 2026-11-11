#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/StaticMeshComponent.h"
#include "Vehicle3DSettings.h"
#include "WheelContactData.h"
#include "WheelActor.generated.h"

UCLASS()
class FORKLIFTSIM_API UWheelTyreComponent : public UStaticMeshComponent
{
    GENERATED_BODY()
public:
    void BuildRollingCollision(int32 Segments);
    void BuildTreadSector(double RadiusCm,double DepthCm,double WidthCm,int32 Count,UStaticMesh* SharedMesh=nullptr);
    void BuildMecanumRoller(double WheelRadiusCm,double RollerRadiusCm,double HalfLengthCm,UStaticMesh* SharedMesh=nullptr);
    void UseMecanumSphereContact(int32 Count);
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
    void Configure(class UStaticMeshComponent* Chassis,const FVehicle3DSettings& S,bool Rear,class UPhysicalMaterial* Material,int32 WheelSlot=0);
    void SetBrakeLimit(double LimitNm) {BrakeLimitNm=LimitNm;UpdateBrakeServo(LimitNm);}
    void Drive(double TorqueNm,double BrakeNm,double SteeringDeg);
    void Actuate(const FVehicle3DSettings& S,double Dt,double RequestNm,double BrakeNm,double SteeringDeg,bool Powered);
    void SetMu(class UPhysicalMaterial* Material);
    void Measure(double RadiusM,double WidthM);
    void UpdateLoad(double ImpulseNs,int32 Contacts,double PhysicsDt);
    void ApplyResistance(const FVehicle3DSettings& Settings,bool Enabled);
    void BuildTread(const FVehicle3DSettings& S,class UPhysicalMaterial* Material);
    void MeasureTread(double RadiusM,double WidthM);
    TArray<UStaticMeshComponent*> PhysicalBodies() const;
    void BuildMecanum(const FVehicle3DSettings& S,class UPhysicalMaterial* Material,int Slot);
    void BuildReducedMecanum(const FVehicle3DSettings& S,class UPhysicalMaterial* Material,int Slot);
    void MeasureMecanum();
    void ResistRollers(bool Enabled);
    bool bMecanum=false,bSphereContact=false,bReducedMecanum=false;
    double ReducedContactLossJ=0,ReducedContactElasticJ=0;
    double RollerHand=0,RollerBearingPowerW=0;
    FMecanumSettings MecanumSpec;
    UPROPERTY() TArray<TObjectPtr<UWheelTyreComponent>> Rollers;
    UPROPERTY() TArray<TObjectPtr<class UPhysicsConstraintComponent>> RollerJoints;
    TArray<double> RollerNormalN,RollerOmega,RollerBearingNm;
    TArray<double> RollerAnchorErrorM;
    TArray<FVector> RollerLocalCenters;
    TArray<FVector> RollerContactPointM;
    UPROPERTY() TArray<TObjectPtr<UWheelTyreComponent>> TreadPads;
    UPROPERTY() TArray<TObjectPtr<class UPhysicsConstraintComponent>> TreadJoints;
    UPROPERTY() TArray<TObjectPtr<class UPhysicsConstraintComponent>> TreadLinks;
    TArray<double> TreadCompressionM,TreadRateMps;
    TArray<double> TreadShearM,TreadRotationRad;
    TArray<double> TreadNormalN;
    TArray<FVector> TreadContactPointM;
    double TreadMaxCompressionM=0,TreadMinCompressionM=0,TreadStoredJ=0,TreadSupportN=0;
    double TreadShearJ=0,TreadRotationJ=0,TreadLinkJ=0,TreadMaxShearM=0,TreadMaxRotationRad=0;
    bool bTreadBottomed=false;
    double NormalLoadN=0, NormalLoadFilteredN=0;
    double ContactInnerPeakN=0;
    FVector ContactFrictionForceN=FVector::ZeroVector;
    double RollingTorqueNm=0, BearingTorqueNm=0, ResistancePowerW=0;
    int32 FloorContacts=0;
    FWheelContactData ContactData;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UStaticMeshComponent> Knuckle;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UStaticMeshComponent> Tyre;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UStaticMeshComponent> BeltCarrier;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UPhysicsConstraintComponent> ComplianceGuide;
    UPROPERTY() TObjectPtr<class UPhysicsConstraintComponent> ShearGuide;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UPhysicsConstraintComponent> BrakeJoint;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UPhysicsConstraintComponent> SteeringJoint;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Physics") TObjectPtr<class UPhysicsConstraintComponent> Axle;
    UPROPERTY() TObjectPtr<class UPhysicsConstraintComponent> ChassisCollisionFilter;
    bool bRear=false, bGrounded=false;
    bool bSteerable=false;
    double SteerTargetDeg=0, SpeedTargetMps=0;
    double SolverSlipAlongSum=0,SolverSlipAcrossSum=0,SolverSlipWeight=0,FrictionUsedNs=0;
    double OmegaRadS=0, LongSlipMps=0, LateralSlipMps=0, ClearanceM=0, SteerActualDeg=0;
    double DriveTorqueNm=0, BrakeLimitNm=0;
    double BrakeElasticJ=0,BrakePlasticWorkJ=0;
    ElectricDrive::FResult DriveMotor;
    double DriveRequestNm=0, SteerAvailableNm=0, SteerMeasuredNm=0, SteerOmegaRadS=0, SteerMotorRpm=0;
    double SteerFilteredDeg=0, SteerGearDeg=0, ContactMu=0, ContactSteerNm=0;
    FTyreComplianceSettings Compliance;
    double CompressionM=0, CompressionRateMps=0, AxleSupportN=0, SpringEstimateN=0, SpringEnergyJ=0;
    double ShearXM=0, ShearYM=0, EstimatedPatchLengthM=0, EstimatedPeakPressurePa=0;
    double LoadedRadiusM=.3, RollingRadiusM=0;
    bool bRollingRadiusValid=false, bTyreTravelLimit=false;
    int32 ChassisContactCount=0;
private:
    void UpdateBrakeServo(double TotalLimitNm);
    bool bBrakeAngleActive=false;
    double BrakeHoldAngleDeg=0;
    ECollisionChannel TreadChannel=ECC_GameTraceChannel1;
    double DriveShaftStateNm=0;
    double SteeringPositionGain=50000000, SteeringVelocityGain=2000000;
    bool bSteerOverspeed=false;
    UPROPERTY() TObjectPtr<class UStaticMeshComponent> MotorStator;
    UFUNCTION() void OnTyreHit(UPrimitiveComponent* HitComponent,AActor* OtherActor,UPrimitiveComponent* OtherComponent,FVector NormalImpulse,const FHitResult& Hit);
};
