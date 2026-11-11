#include "ContactLoadSampler.h"
#include "PBDRigidsSolver.h"
#include "Chaos/PBDRigidsEvolutionGBF.h"
#include "Chaos/PBDCollisionConstraints.h"

bool FContactLoadSampler::AuditTracks(int32 Id) const
{
    const auto* In=GetConsumerInput_Internal();if(!In || Id==INDEX_NONE)return false;
    if(Id==In->ChassisId||Id==In->ForkCarriageId||Id==In->MastStageId||Id==In->PalletId||Id==In->CargoId)return true;
    for(int I=0;I<4;++I)if(Id==In->WheelIds[I]||Id==In->KnuckleIds[I]||Id==In->CarrierIds[I])return true;
    for(int I:In->TineIds)if(Id==I)return true;
    return In->WheelSlot(Id)!=INDEX_NONE;
}
void FContactLoadSampler::CaptureAuditVelocities()
{
    AuditVelocities.Reset();const auto* In=GetConsumerInput_Internal();if(!In || !In->bAuditWork)return;
    auto* S=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
    for(auto& P:S->GetParticles().GetNonDisabledDynamicView())if(AuditTracks(P.UniqueIdx().Idx))
    {
        auto* B=P.Handle()->CastToRigidParticle();if(!B)continue;
        FAuditVelocity V;V.V=FVector(B->GetV())/100.;V.W=FVector(B->GetW());V.Com=(FVector(B->GetP())+FQuat(B->GetQ()).RotateVector(FVector(B->CenterOfMass())))/100.;V.Body=B;
        AuditVelocities.Add(P.UniqueIdx().Idx,V);
    }
}
void FContactLoadSampler::MeasureContactWork(double Dt)
{
    const auto* In=GetConsumerInput_Internal();if(!In||!In->bAuditWork||Dt<=0)return;
    auto& Out=GetProducerOutputData_Internal();auto* S=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
    if(const auto* V=AuditVelocities.Find(In->PalletId))Out.PalletPhysicsMassKg=V->Body->M();
    if(const auto* V=AuditVelocities.Find(In->CargoId))Out.CargoPhysicsMassKg=V->Body->M();
    auto MeanVelocity=[&](int Id,const FVector& Point)
    {
        const auto* V=AuditVelocities.Find(Id);if(!V)return FVector::ZeroVector;
        const auto* B=V->Body;const FVector Com=(FVector(B->GetP())+FQuat(B->GetQ()).RotateVector(FVector(B->CenterOfMass())))/100.;
        return .5*(V->V+FVector::CrossProduct(V->W,Point-V->Com)+FVector(B->GetV())/100.+FVector::CrossProduct(FVector(B->GetW()),Point-Com));
    };
    for(const auto* H:S->GetEvolution()->GetCollisionConstraints().GetConstConstraintHandles())
    {
        if(!H)continue;const auto& C=H->GetContact();if(!C.IsEnabled()||!C.IsCurrent()||C.GetIsProbe())continue;
        const int A=C.GetParticle0()->UniqueIdx().Idx,B=C.GetParticle1()->UniqueIdx().Idx;
        if(!AuditVelocities.Contains(A)&&!AuditVelocities.Contains(B))continue;
        for(int K=0;K<C.NumManifoldPoints();++K)
        {
            const auto& R=C.GetManifoldPointResult(K);if(!C.IsManifoldPointActive(K)||!R.bIsValid)continue;
            const auto& P=C.GetManifoldPoint(K).ContactPoint;
            const FVector P0=FVector(C.GetShapeWorldTransform0().TransformPosition(Chaos::FVec3(P.ShapeContactPoints[0])))/100.;
            const FVector P1=FVector(C.GetShapeWorldTransform1().TransformPosition(Chaos::FVec3(P.ShapeContactPoints[1])))/100.;
            const FVector N=FVector(C.GetShapeWorldTransform1().TransformVector(Chaos::FVec3(P.ShapeContactNormal))).GetSafeNormal();
            const FVector J=FVector(R.NetImpulse)/100.,JN=N*FVector::DotProduct(J,N),V=MeanVelocity(A,P0)-MeanVelocity(B,P1);
            Out.ContactNormalWorkJ+=FVector::DotProduct(JN,V);Out.ContactTangentWorkJ+=FVector::DotProduct(J-JN,V);
            // Positional correction is NOT physical friction loss. Keep signed
            // pseudo-work visible beside the still-unresolved solver residual.
            const double Correction=FVector::DotProduct(FVector(R.NetPushOut)/(100.*Dt),V);
            Out.ContactCorrectionPseudoJ+=Correction;++Out.ContactWorkPoints;
            if(FMath::Abs(Correction)>500.)
            {
                auto Role=[&](int Id){if(Id==In->ChassisId)return TEXT("chassis");if(Id==In->PalletId)return TEXT("pallet");if(Id==In->CargoId)return TEXT("cargo");if(Id==In->ForkCarriageId)return TEXT("carriage");if(Id==In->MastStageId)return TEXT("mast");if(In->WheelSlot(Id)!=INDEX_NONE)return TEXT("wheel");for(int T:In->TineIds)if(T==Id)return TEXT("tine");return TEXT("other");};
                UE_LOG(LogTemp,Display,TEXT("CONTACT_ENERGY_SPIKE ids=%d/%d roles=%s/%s pseudo_j=%.6f point_m=%s normal=%s impulse_ns=%s relative_mps=%s"),A,B,Role(A),Role(B),Correction,*P0.ToString(),*N.ToString(),*J.ToString(),*V.ToString());
            }
        }
    }
    for(int I=0;I<In->ForkDampingCount;++I)
    {
        const auto* A=AuditVelocities.Find(In->ForkDampingParent[I]);const auto* B=AuditVelocities.Find(In->ForkDampingChild[I]);if(!A||!B)continue;
        const FVector Axis=FQuat(A->Body->GetQ()).GetAxisY();
        const FVector Mean=.5*(A->W-B->W+FVector(A->Body->GetW())-FVector(B->Body->GetW()));
        Out.ForkDampingEstimateJ+=In->ForkDampingC[I]*FMath::Square(FVector::DotProduct(Mean,Axis))*Dt;
    }
}
