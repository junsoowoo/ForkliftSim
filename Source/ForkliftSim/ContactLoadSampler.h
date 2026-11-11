#pragma once
#include "Chaos/SimCallbackObject.h"
#include "CargoContactData.h"
#include "WheelContactData.h"
#include "DrivetrainModel.h"
#include "JointSensors.h"
#include "DynamicsSensors.h"

struct FContactLoadInput : Chaos::FSimCallbackInput
{
    bool bJointSensors=false;double JointSamplePeriodS=.01,JointForkBaseM=0;
    bool bDynamicsSensors=false;double DynamicsPeriodS=.01;FVector ImuOffsetM=FVector(0,0,.4),SensorGravityMps2=FVector(0,0,-9.81);
    int32 WheelIds[4]={INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE};
    // Runtime coupled ring <=64; the one-wheel reference fixture also uses 96.
    int32 TreadIds[4][128], TreadCounts[4]={0,0,0,0};
    double TreadPadC=0,TreadRestRadiusM=0,TreadPhaseRad=0;
    double TreadShearC=0,TreadRotC=0,TreadLinkC=0,TreadBendC=0;
    bool bTreadCoupled=false;
    bool bMecanum=false,bReducedMecanum=false;
    double ReducedTyreMu=.9,ReducedRoadMu=.8,ReducedSurfaceMu[8]={};
    bool ReducedServiceBrake[4]={};
    double ReducedRollerCoulombNm=.03,ReducedRollerViscousNms=.002,ReducedRollerRadiusM=.02;
    bool bBearingPredictor=false;
    bool bExactRollerSpin=false;
    bool bMecanumDetailedMotor=true;
    bool bMecanumControl=false,bMecanumPowered=false,bMecanumResistance=true;
    bool bMecanumSensorTorque=false;double MecanumSensorTorqueNm[4]={},MecanumLeaseUntilS=0;
    ElectricDrive::FSpec MecanumMotor;
    double MecanumTargetMps[4]={},MecanumLoadN[4]={};
    double MecanumRadiusM=.15,MecanumGain=350,MecanumTorqueLimitNm=450,MecanumCrr=.015;
    double MecanumBearingCoulomb=1,MecanumBearingViscous=.05;
    bool bAuditWork=false;
    // Isolated material bench only: constant wrench at the particle origin,
    // updated at each inner step (a fixed GT world point would add force lag).
    int32 CalibrationBodyId=INDEX_NONE;
    FVector CalibrationForceN=FVector::ZeroVector,CalibrationTorqueNm=FVector::ZeroVector;
    FVector DriveTorqueNm[4]={FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector};
    FVector RollingTorqueNm[4]={FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector};
    FContactLoadInput() { for(auto& Row:TreadIds) for(int32& Id:Row) Id=INDEX_NONE; }
    int32 WheelSlot(int32 Id) const
    {
        if(Id==INDEX_NONE) return INDEX_NONE;
        for(int I=0;I<4;++I) { if(WheelIds[I]==Id) return I; for(int J=0;J<TreadCounts[I];++J) if(TreadIds[I][J]==Id) return I; }
        return INDEX_NONE;
    }
    int32 PadSlot(int32 Wheel,int32 Id) const { for(int J=0;J<TreadCounts[Wheel];++J) if(TreadIds[Wheel][J]==Id) return J; return INDEX_NONE; }
    int32 BenchHubId=INDEX_NONE,BenchRollerIds[16]={},BenchRollerCount=0;
    int32 BenchIds[32]={}; int32 BenchCount=0; FVector BenchOriginM=FVector::ZeroVector;
    int32 RoadId=INDEX_NONE;
    int32 RackIds[2]={INDEX_NONE,INDEX_NONE};
    int32 ForkCarriageId=INDEX_NONE,MastStageId=INDEX_NONE,ForkDampingCount=0;
    int32 ForkDampingParent[16]={},ForkDampingChild[16]={};double ForkDampingC[16]={};
    int32 ChassisId=INDEX_NONE, KnuckleIds[4]={INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE};
    int32 CarrierIds[4]={INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE};
    double TyreDampingNsPm=0;
    bool bTyreEnabled=false;
    bool bSmoothTyreContacts=false;
    bool bSmoothTyreEdges=false;
    bool bBrakeHolding[4]={false,false,false,false};
    double TyreRadiusCm=30, TyreHalfWidthCm=9;
    int32 SurfaceIds[8]={INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE};
    // Up to eight physical sections per fork; aggregate every section contact.
    int32 TineIds[16]={INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE,INDEX_NONE}, PalletId=INDEX_NONE, CargoId=INDEX_NONE;
    void Reset() { bMecanumSensorTorque=false;MecanumLeaseUntilS=0;for(double& T:MecanumSensorTorqueNm)T=0;bDynamicsSensors=false;DynamicsPeriodS=.01;ImuOffsetM=FVector(0,0,.4);SensorGravityMps2=FVector(0,0,-9.81);bJointSensors=false;JointSamplePeriodS=.01;JointForkBaseM=0;ForkCarriageId=MastStageId=INDEX_NONE;ForkDampingCount=0;RackIds[0]=RackIds[1]=INDEX_NONE;bReducedMecanum=false;bExactRollerSpin=false;bBearingPredictor=false;BenchHubId=INDEX_NONE;BenchRollerCount=0;bMecanumControl=false;bMecanumPowered=false;bMecanumResistance=true; BenchCount=0; bMecanum=false; CalibrationBodyId=INDEX_NONE; CalibrationForceN=CalibrationTorqueNm=FVector::ZeroVector; bAuditWork=false; for(FVector& T:DriveTorqueNm) T=FVector::ZeroVector; for(FVector& T:RollingTorqueNm) T=FVector::ZeroVector; TreadShearC=TreadRotC=TreadLinkC=TreadBendC=0; bTreadCoupled=false; for(auto& Row:TreadIds) for(int32& Id:Row) Id=INDEX_NONE; for(int32& Count:TreadCounts) Count=0; TreadPadC=TreadRestRadiusM=TreadPhaseRad=0; for(bool& Holding:bBrakeHolding) Holding=false; for(int32& Id:WheelIds) Id=INDEX_NONE; for(int32& Id:CarrierIds) Id=INDEX_NONE; for(int32& Id:KnuckleIds) Id=INDEX_NONE; for(int32& Id:SurfaceIds) Id=INDEX_NONE; for(int32& Id:TineIds) Id=INDEX_NONE; RoadId=PalletId=CargoId=ChassisId=INDEX_NONE; TyreDampingNsPm=0; bTyreEnabled=bSmoothTyreContacts=bSmoothTyreEdges=false; TyreRadiusCm=30; TyreHalfWidthCm=9; }
};
struct FContactLoadOutput : Chaos::FSimCallbackOutput
{
    TArray<JointSensor::FTruth> JointSamples;
    TArray<DynamicsSensor::FTruth> DynamicsSamples;
    double ReducedLossJ[4]={},ReducedElasticJ[4]={};
    double PalletRackNs=0;
    double ContactNormalWorkJ=0,ContactTangentWorkJ=0,ContactCorrectionPseudoJ=0,ForkDampingEstimateJ=0;
    int32 ContactWorkPoints=0;
    double PalletPhysicsMassKg=0,CargoPhysicsMassKg=0;
    double ImpulseNs[4]={0,0,0,0};
    double PeakNormalN[4]={0,0,0,0}, MinNormalN[4]={1e30,1e30,1e30,1e30};
    int32 RefinedContacts=0, RefinedEdgeContacts=0;
    FWheelContactData WheelContact[4];
    double TreadNormalNs[4][128]={};
    FVector TreadPointMNs[4][128]={};
    int32 Contacts[4]={0,0,0,0};
    FVector WheelMomentNs[4]={FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector};
    FVector WheelFrictionImpulseNs[4]={FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector};
    double WheelMuNs[4]={0,0,0,0};
    FVector SteerImpulseNms[4]={FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector};
    FVector AxleImpulseNs[4]={FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector,FVector::ZeroVector};
    double TyreDissipatedJ=0, TyrePeakCompressionM=0;
    ElectricDrive::FResult MecanumMotor[4];
    double MecanumRequestNm[4]={},MecanumRollingNm[4]={};
    bool bMecanumMotorValid=false;
    bool bDriveCommandExpired=false;
    double BearingPredictorWorkJ=0,BenchPredictorWorkJ=0;
    double ShaftJointWorkJ=0;
    double DriveWorkJ=0,RollingWorkJ=0,SteeringJointWorkJ=0;
    double RollerBearingImpulseNms[4][128]={};
    double RollerBearingWorkJ[4]={0,0,0,0};
    double SlipAlongM2Ns[4]={},SlipAcrossM2Ns[4]={},SlipWeightNs[4]={},FrictionUsedNs[4]={};
    FVector BenchL=FVector::ZeroVector;
    double BenchEnergyJ=0;
    bool bBenchValid=false;
    double IntervalS=0, CompletedInnerS=0, MaxInnerDt=0;
    int32 InnerSteps=0;
    double ForkVerticalNs[2]={0,0}, ForkNormalNs[2]={0,0}, ForkFrictionNs[2]={0,0}, PalletGroundNs=0;
    FVector ForkPointWeighted[2]={FVector::ZeroVector,FVector::ZeroVector};
    FCargoContactData Cargo;
    FContactLoadOutput() {for(auto& Row:TreadPointMNs) for(FVector& P:Row) P=FVector::ZeroVector;}
    void Reset() { bDriveCommandExpired=false;DynamicsSamples.Reset();JointSamples.Reset();PalletPhysicsMassKg=CargoPhysicsMassKg=0;ContactNormalWorkJ=ContactTangentWorkJ=ContactCorrectionPseudoJ=ForkDampingEstimateJ=0;ContactWorkPoints=0;PalletRackNs=0;for(double& V:ReducedLossJ)V=0;for(double& V:ReducedElasticJ)V=0;bMecanumMotorValid=false; BenchL=FVector::ZeroVector;BenchEnergyJ=0;bBenchValid=false;for(int I=0;I<4;++I) SlipAlongM2Ns[I]=SlipAcrossM2Ns[I]=SlipWeightNs[I]=FrictionUsedNs[I]=0; for(auto& Row:RollerBearingImpulseNms) for(double& V:Row) V=0; for(double& V:RollerBearingWorkJ) V=0; BearingPredictorWorkJ=BenchPredictorWorkJ=ShaftJointWorkJ=0;DriveWorkJ=RollingWorkJ=SteeringJointWorkJ=0; for(double& J:ImpulseNs) J=0; for(int32& N:Contacts) N=0; for(FVector& M:WheelMomentNs) M=FVector::ZeroVector; for(FVector& M:SteerImpulseNms) M=FVector::ZeroVector; for(FVector& M:AxleImpulseNs) M=FVector::ZeroVector; for(double& M:WheelMuNs) M=0; InnerSteps=0; IntervalS=CompletedInnerS=MaxInnerDt=PalletGroundNs=TyreDissipatedJ=TyrePeakCompressionM=0;
        for(int I=0;I<2;++I) { ForkVerticalNs[I]=ForkNormalNs[I]=ForkFrictionNs[I]=0; ForkPointWeighted[I]=FVector::ZeroVector; } Cargo=FCargoContactData();
        for(auto& Row:TreadNormalNs) for(double& N:Row) N=0; for(auto& Row:TreadPointMNs) for(FVector& P:Row) P=FVector::ZeroVector; for(int I=0;I<4;++I) { PeakNormalN[I]=0; MinNormalN[I]=1e30; WheelFrictionImpulseNs[I]=FVector::ZeroVector; WheelContact[I]=FWheelContactData(); } RefinedContacts=RefinedEdgeContacts=0; }
};
// No UObject access on the physics thread. Unique particle IDs and a marshalled
// output queue make reset/destruction independent of game-thread actor lifetime.
// UE 5.6 registers the common callback list through the Presimulate flag, even
// when the useful work is done only in PostSolve.
class FContactLoadSampler : public Chaos::TSimCallbackObject<FContactLoadInput,FContactLoadOutput,Chaos::ESimCallbackOptions::Presimulate | Chaos::ESimCallbackOptions::PreIntegrate | Chaos::ESimCallbackOptions::PostIntegrate | Chaos::ESimCallbackOptions::PostSolve | Chaos::ESimCallbackOptions::ContactModification>
{
    void CaptureJointSensors(double Dt);
    void CaptureDynamicsSensors(double Dt,const FVector* ForkImpulseNs);
    double DynamicsClockS=0,NextDynamicsS=0,PreviousImuS=0,LoadWindowS=0,LoadImpulseNs[2]={};
    FVector PreviousImuVelocity=FVector::ZeroVector;bool ImuSeeded=false;
    double JointClockS=0,NextJointSampleS=0,JointShaftRad[4]={};
    struct FAuditVelocity {FVector V,W,Com;Chaos::FPBDRigidParticleHandle* Body=nullptr;};
    TMap<int32,FAuditVelocity> AuditVelocities;
    bool AuditTracks(int32 Id) const;
    void CaptureAuditVelocities();
    void MeasureContactWork(double Dt);
    void ApplyReducedContact();
    void CompleteReducedContact(double Dt);
    double ReducedNormalN[4]={},ReducedMu[4]={},ReducedShearM[4]={};
    FVector ReducedPointCm[4]={},ReducedNormal[4]={};
    Chaos::FPBDRigidParticleHandle* ReducedBodies[4]={};
    Chaos::FVec3 ReducedAddedA[4]={},ReducedAddedW[4]={};
    Chaos::FPBDRigidParticleHandle* ReducedSolvedBodies[4]={};
    FVector ReducedBeforeV[4]={},ReducedDirection[4]={},ReducedArm[4]={},ReducedImpulse[4]={};
    double ReducedOldShear[4]={},ReducedForceLimit[4]={};bool ReducedWasBraking[4]={};
    double ReducedPriorNormalN[4]={};bool ReducedSteadySupport[4]={};
    FVector BenchBeforeL=FVector::ZeroVector,BenchIntegratedL=FVector::ZeroVector;
    FVector BenchIntegrationDrift=FVector::ZeroVector,BenchConstraintDrift=FVector::ZeroVector;
    double BenchAuditTime=0,BenchAuditLogged=0;
    bool bBenchStep=false;
    FVector MeasureBenchMomentum(bool Predicted);
    void ApplyBearingKick(bool Predicted,double HalfDt);
    void IntegrateRollerSpin(bool Before);
    double BearingKickDt=0,BearingKickMainJ=0,BearingKickBenchJ=0;
    Chaos::FPBDRigidParticleHandle* PredictorBodies[85]={};
    FVector PredictorForceN[85]={},PredictorTorqueNm[85]={};bool PredictorMain[85]={};int PredictorCount=0;
    Chaos::FPBDRigidParticleHandle* MotorBodies[5]={};
    Chaos::FVec3 MotorAddedW[5]={};int MotorBodyCount=0;
    double MotorShaftStateNm[4]={};
    FVector AppliedMotorNm[4]={},AppliedRollingNm[4]={};
    Chaos::FPBDRigidParticleHandle* CalibrationBody=nullptr;
    Chaos::FVec3 CalibrationAddedA=Chaos::FVec3(0),CalibrationAddedW=Chaos::FVec3(0);
    virtual void OnPreIntegrate_Internal() override;
    virtual void OnPostIntegrate_Internal() override;
    virtual void OnPreSimulate_Internal() override {}
    virtual void OnContactModification_Internal(Chaos::FCollisionContactModifier& Modifier) override;
    virtual void OnPostSolve_Internal() override;
};


