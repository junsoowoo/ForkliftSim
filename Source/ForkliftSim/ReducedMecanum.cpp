#include "ReducedMecanum.h"
#include "WheelActor.h"
#include "Vehicle3DHelpers.h"
#include "ContactLoadSampler.h"
#include "PBDRigidsSolver.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

void AWheelActor::BuildReducedMecanum(const FVehicle3DSettings& S,UPhysicalMaterial* Material,int Slot)
{
    bReducedMecanum=true;MecanumSpec=S.Mecanum;RollerHand=Mecanum::Hand(Slot);
    if(Slot==0)UE_LOG(LogTemp,Display,TEXT("REDUCED_SUPPORT profile=%s scope=rigid_equivalent_cylinder_not_PU_deformation"),FParse::Param(FCommandLine::Get(),TEXT("NativeReducedSupport"))?TEXT("legacy_native_convex"):TEXT("analytic_finite_cylinder_box_v2"));
    const FVector P=Tyre->GetComponentLocation();const FQuat Q=Tyre->GetComponentQuat();
    const FVector AxleAxis=Knuckle->GetRightVector();
    const double HubMass=S.WheelMassKg-S.Mecanum.Count*S.Mecanum.RollerMassKg;
    const double R=S.Mecanum.HubRadiusM,W=S.Mecanum.HubWidthM;
    FVector Inertia(HubMass*(3*R*R+W*W)/12,HubMass*(3*R*R+W*W)/12,HubMass*R*R/2);
    UStaticMesh* Mesh=nullptr;
    for(int I=0;I<S.Mecanum.Count;++I)
    {
        const double Angle=2*PI*I/S.Mecanum.Count;
        const FVector Radial=Q.RotateVector(FVector(FMath::Cos(Angle),FMath::Sin(Angle),0));
        const FVector Axis=(-FVector::CrossProduct(AxleAxis,Radial)+RollerHand*AxleAxis).GetSafeNormal();
        const FQuat Pose=FRotationMatrix::MakeFromXZ(Axis,Radial).ToQuat();
        auto* B=NewObject<UWheelTyreComponent>(this);AddInstanceComponent(B);B->SetMobility(EComponentMobility::Movable);B->RegisterComponent();
        B->SetWorldLocationAndRotation(P+Radial*(S.RadiusM-S.Mecanum.RollerRadiusM)*100,Pose);
        B->BuildMecanumRoller(S.RadiusM*100,S.Mecanum.RollerRadiusM*100,S.Mecanum.RollerHalfLengthM*100,Mesh);Mesh=B->GetStaticMesh();
        // Cook once to retain the authored component masses and locked-assembly
        // inertia. These components become collision-free visuals before play.
        Vehicle3D::Dynamic(B,S.Mecanum.RollerMassKg);
        const FVector PartI=B->GetInertiaTensor()/10000.;
        const FQuat Relative=Q.Inverse()*Pose*B->BodyInstance.GetMassSpaceLocal().GetRotation();
        const FVector Arm=Q.UnrotateVector(B->GetCenterOfMass()-P)/100.;
        for(int K=0;K<3;++K)
        {
            FVector E=FVector::ZeroVector;E[K]=1;
            const FVector A=Relative.UnrotateVector(E);
            Inertia[K]+=FVector::DotProduct(A,PartI*A)+S.Mecanum.RollerMassKg*(Arm.SizeSquared()-Arm[K]*Arm[K]);
        }
        B->SetSimulatePhysics(false);B->BodyInstance.bAutoWeld=false;B->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        B->AttachToComponent(Tyre,FAttachmentTransformRules::KeepWorldTransform);
        Vehicle3D::Color(B,I%2?FLinearColor(.09,.13,.15):FLinearColor(.045,.065,.08));
    }
    FPhysicsCommand::ExecuteWrite(Tyre->BodyInstance.GetPhysicsActor(),[Inertia](const FPhysicsActorHandle& A)
    {
        FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(A,Inertia*10000.);
        FPhysicsInterface::SetComLocalPose_AssumesLocked(A,FTransform::Identity);
    });
    // The full-radius hidden cylinder supplies only contact/support geometry.
    // The visible hub retains the original external dimensions.
    Tyre->SetVisibility(false,false);
    auto* Hub=NewObject<UStaticMeshComponent>(this);AddInstanceComponent(Hub);
    Hub->SetMobility(EComponentMobility::Movable);Hub->SetStaticMesh(Tyre->GetStaticMesh());Hub->BodyInstance.bAutoWeld=false;
    Hub->SetCollisionEnabled(ECollisionEnabled::NoCollision);Hub->RegisterComponent();
    Hub->SetWorldLocationAndRotation(P,Q);Hub->SetWorldScale3D(FVector(2*R,2*R,W));
    Hub->AttachToComponent(Tyre,FAttachmentTransformRules::KeepWorldTransform);Vehicle3D::Color(Hub,FLinearColor(.22,.30,.34));
    UE_LOG(LogTemp,Display,TEXT("REDUCED_MECANUM slot=%d mass_kg=%.3f inertia_kgm2=%s physical_rollers=0 visual_rollers=%d"),Slot,Tyre->GetMass(),*Inertia.ToString(),S.Mecanum.Count);
}

void FContactLoadSampler::ApplyReducedContact()
{
    const auto* In=GetConsumerInput_Internal();if(!In || !In->bReducedMecanum)return;
    auto* RigidSolver=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
    const double Remaining=FMath::Max(0.,GetDeltaTime_Internal()-GetProducerOutputData_Internal().CompletedInnerS);
    const double Dt=FMath::Min(Remaining,RigidSolver->GetMaxDeltaTime_External());if(Dt<=0)return;
    Chaos::FPBDRigidParticleHandle* Hubs[4]={};Chaos::FPBDRigidParticleHandle* Stator=nullptr;
    for(auto& P:RigidSolver->GetParticles().GetNonDisabledDynamicView())
    {
        const int Id=P.UniqueIdx().Idx;
        if(Id==In->ChassisId)Stator=P.Handle()->CastToRigidParticle();
        for(int I=0;I<4;++I)if(Id==In->WheelIds[I])Hubs[I]=P.Handle()->CastToRigidParticle();
    }
    if(!Stator)return;
    constexpr double ShearK=200000,ShearC=2000; // assumed aggregate contact compliance (N/m, Ns/m)
    auto& Output=GetProducerOutputData_Internal();
    for(int I=0;I<4;++I)
    {
        ReducedSteadySupport[I]=ReducedNormalN[I]>0 && FMath::Abs(ReducedNormalN[I]-ReducedPriorNormalN[I])<.05*FMath::Max(100.,ReducedNormalN[I]);
        ReducedPriorNormalN[I]=ReducedNormalN[I];
    }
    for(int I=0;I<4;++I)if(!Hubs[I] || ReducedNormalN[I]<=0)
    {Output.ReducedLossJ[I]+=.5*ShearK*ReducedShearM[I]*ReducedShearM[I];ReducedShearM[I]=0;Output.ReducedElasticJ[I]=0;}
    for(int I=0;I<4;++I)if(Hubs[I] && ReducedNormalN[I]>0)
    {
        auto* B=Hubs[I];const FVector N=ReducedNormal[I];
        const FVector Direction=ReducedMecanum::ContactDirection(Stator->GetR().GetAxisY(),N,Mecanum::Hand(I));
        if(Direction.IsNearlyZero())continue;
        const FVector Com=B->GetX()+B->GetR().RotateVector(B->CenterOfMass());
        const FVector Arm=(ReducedPointCm[I]-Com)/100.;
        const FVector V=FVector(B->GetV())/100.+FVector::CrossProduct(FVector(B->GetW()),Arm);
        const FQuat MassQ=FQuat(B->GetR())*FQuat(B->RotationOfMass());
        const FVector Moment=MassQ.UnrotateVector(FVector::CrossProduct(Arm,Direction));
        const double InverseMass=1./B->M()+FVector::DotProduct(Moment,FVector(B->InvI())*10000.*Moment);
        const double OldShearState=ReducedShearM[I];
        const double Limit=ReducedMu[I]*ReducedNormalN[I]*Dt;
        const FVector Free=FVector::CrossProduct(N,Direction).GetSafeNormal();double FreeImpulse=0;
        auto PointKick=[&](FVector J)
        {
            const FVector DW=MassQ.RotateVector(FVector(B->InvI())*10000.*MassQ.UnrotateVector(FVector::CrossProduct(Arm,J)));
            return J/B->M()+FVector::CrossProduct(DW,Arm);
        };
        if(In->bMecanumResistance)
        {
            const double VF=FVector::DotProduct(V,Free),RR=In->ReducedRollerRadiusM;
            const double InvFree=FVector::DotProduct(Free,PointKick(Free));
            const double Loss=In->ReducedRollerCoulombNm/RR*FMath::Clamp(VF/.02,-1.,1.)+In->ReducedRollerViscousNms/(RR*RR)*VF;
            const double Cap=FMath::Min(Limit,.8*FMath::Abs(VF)/InvFree);
            FreeImpulse=FMath::Clamp(-Loss*Dt,-Cap,Cap);
        }
        // Apply the two tangent directions sequentially in the body's exact
        // point inverse-mass matrix. This retains their rotational cross term.
        const FVector AfterFree=V+PointKick(Free*FreeImpulse);
        const double TractionLimit=FMath::Sqrt(FMath::Max(0.,Limit*Limit-FreeImpulse*FreeImpulse));
        double TractionImpulse=0;
        if(In->ReducedServiceBrake[I] && ReducedMu[I]>0)
        {
            const auto Step=ReducedMecanum::BrakeStep(FVector::DotProduct(AfterFree,Direction),ReducedShearM[I],InverseMass,Dt,ShearK,ShearC,TractionLimit/Dt);
            TractionImpulse=Step.Impulse;ReducedShearM[I]=Step.Shear;
        }
        else
        {
            ReducedShearM[I]=0;TractionImpulse=FMath::Clamp(-.8*FVector::DotProduct(AfterFree,Direction)/InverseMass,-TractionLimit,TractionLimit);
        }
        const FVector F=(Direction*TractionImpulse+Free*FreeImpulse)/Dt;
        ReducedSolvedBodies[I]=B;ReducedBeforeV[I]=V;ReducedDirection[I]=Direction;ReducedArm[I]=Arm;
        ReducedImpulse[I]=F*Dt;
        // Preserve the actual old signed state, not the predicted return state.
        ReducedOldShear[I]=OldShearState;
        ReducedForceLimit[I]=TractionLimit/Dt;ReducedWasBraking[I]=In->ReducedServiceBrake[I];
        const auto OldA=B->Acceleration(),OldW=B->AngularAcceleration();
        B->AddForce(F*100.);B->AddTorque(FVector::CrossProduct(Arm,F)*10000.);
        ReducedBodies[I]=B;ReducedAddedA[I]=B->Acceleration()-OldA;ReducedAddedW[I]=B->AngularAcceleration()-OldW;
        auto& Out=GetProducerOutputData_Internal();Out.WheelFrictionImpulseNs[I]+=F*Dt;Out.WheelMomentNs[I]+=FVector::CrossProduct(Arm,F)*Dt;Out.FrictionUsedNs[I]+=F.Size()*Dt;
        const double Slip=FVector::DotProduct(V,Direction);
        Out.SlipAlongM2Ns[I]+=Slip*Slip*ReducedNormalN[I]*Dt;Out.SlipWeightNs[I]+=ReducedNormalN[I]*Dt;
    }
}

void FContactLoadSampler::CompleteReducedContact(double Dt)
{
    constexpr double K=200000;
    auto& Out=GetProducerOutputData_Internal();
    for(int I=0;I<4;++I)if(auto* B=ReducedSolvedBodies[I])
    {
        const FVector After=FVector(B->GetV())/100.+FVector::CrossProduct(FVector(B->GetW()),ReducedArm[I]);
        const FVector Mean=.5*(ReducedBeforeV[I]+After);
        const double Work=FVector::DotProduct(ReducedImpulse[I],Mean),OldE=.5*K*FMath::Square(ReducedOldShear[I]);
        // Constraint-compatible stiction is a near-rest branch, not an impact
        // integrator. During shocks retain the passive free-body predictor.
        const bool NearRest=ReducedBeforeV[I].Size()<.3 && After.Size()<.3 && FVector(B->GetW()).Size()<.5;
        static const bool Diagnostics=FParse::Param(FCommandLine::Get(),TEXT("ReducedContactDiagnostics"));
        static int DiagnosticSteps=0;
        if(Diagnostics && I==0 && ++DiagnosticSteps%960==0)
            UE_LOG(LogTemp,Display,TEXT("REDUCED_STICTION before_mps=%.6f after_mps=%.6f omega_rad_s=%.6f near_rest=%d steady_load=%d shear_m=%.8f normal_n=%.3f"),ReducedBeforeV[I].Size(),After.Size(),FVector(B->GetW()).Size(),NearRest?1:0,ReducedSteadySupport[I]?1:0,ReducedOldShear[I],ReducedPriorNormalN[I]);
        double LossWork=Work;
        if(ReducedWasBraking[I] && NearRest)
            ReducedShearM[I]=ReducedMecanum::SolvedShear(ReducedOldShear[I],FVector::DotProduct(Mean,ReducedDirection[I]),Dt,K,ReducedForceLimit[I],Work);
        else
        {
            const FQuat MassQ=FQuat(B->GetR())*FQuat(B->RotationOfMass());
            const FVector Moment=MassQ.UnrotateVector(FVector::CrossProduct(ReducedArm[I],ReducedImpulse[I]));
            const double Kick=.5*(ReducedImpulse[I].SizeSquared()/B->M()+FVector::DotProduct(Moment,FVector(B->InvI())*10000.*Moment));
            LossWork=FVector::DotProduct(ReducedImpulse[I],ReducedBeforeV[I])+Kick;
        }
        Out.ReducedElasticJ[I]=.5*K*FMath::Square(ReducedShearM[I]);
        Out.ReducedLossJ[I]+=FMath::Max(0.,OldE-Out.ReducedElasticJ[I]-LossWork);
        ReducedSolvedBodies[I]=nullptr;
    }
}
