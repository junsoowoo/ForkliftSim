#include "ContactLoadSampler.h"
#include "PBDRigidsSolver.h"
#include "Chaos/PBDRigidsEvolutionGBF.h"
#include "Chaos/PBDCollisionConstraints.h"
#include "Chaos/ContactModification.h"
#include "WheelContactGeometry.h"
#include "WheelResistance.h"
#include "RigidSpin.h"
#include <cmath>
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/ScopeExit.h"
#include "ReducedMecanum.h"

FVector FContactLoadSampler::MeasureBenchMomentum(bool Predicted)
{
    const auto* In=GetConsumerInput_Internal();
    if(!In) return FVector::ZeroVector;
    auto* RigidSolver=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
    FVector L=FVector::ZeroVector;
    for(auto& P:RigidSolver->GetParticles().GetNonDisabledDynamicView())
    {
        bool Wanted=false;for(int I=0;I<In->BenchCount;++I) Wanted |= P.UniqueIdx().Idx==In->BenchIds[I];
        if(!Wanted) continue;
        const auto* B=P.Handle()->CastToRigidParticle();if(!B) continue;
        const FQuat R=Predicted?B->GetQ():B->GetR();
        const FQuat Q=R*FQuat(B->RotationOfMass());
        const FVector Com=((Predicted?B->GetP():B->GetX())+R.RotateVector(B->CenterOfMass()))/100.;
        L+=Q.RotateVector(FVector(B->I())/10000.*Q.UnrotateVector(B->GetW()))+FVector::CrossProduct(Com-In->BenchOriginM,FVector(B->GetV())/100.*B->M());
    }
    return L;
}

void FContactLoadSampler::ApplyBearingKick(bool Predicted,double HalfDt)
{
    const auto* In=GetConsumerInput_Internal();if(!In || HalfDt<=0) return;
    auto* RigidSolver=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
    Chaos::FPBDRigidParticleHandle* Hubs[5]={};Chaos::FPBDRigidParticleHandle* Rollers[5][16]={};
    for(auto& P:RigidSolver->GetParticles().GetNonDisabledDynamicView())
    {
        const int Id=P.UniqueIdx().Idx;auto* B=P.Handle()->CastToRigidParticle();if(!B) continue;
        for(int I=0;I<4;++I)
        {
            if(Id==In->WheelIds[I]) Hubs[I]=B;
            for(int J=0;J<FMath::Min(In->TreadCounts[I],16);++J) if(Id==In->TreadIds[I][J]) Rollers[I][J]=B;
        }
        if(Id==In->BenchHubId) Hubs[4]=B;
        for(int J=0;J<In->BenchRollerCount;++J) if(Id==In->BenchRollerIds[J]) Rollers[4][J]=B;
    }
    PredictorCount=0;
    auto Wrench=[&](Chaos::FPBDRigidParticleHandle* B,FVector F,FVector T,bool Main)
    {
        int K=0;while(K<PredictorCount && PredictorBodies[K]!=B) ++K;
        if(K==PredictorCount) {check(PredictorCount<85);PredictorBodies[K]=B;PredictorForceN[K]=PredictorTorqueNm[K]=FVector::ZeroVector;PredictorMain[K]=Main;++PredictorCount;}
        PredictorForceN[K]+=F;PredictorTorqueNm[K]+=T;
    };
    auto Com=[&](const Chaos::FPBDRigidParticleHandle* B) {const FQuat Q=Predicted?B->GetQ():B->GetR();return ((Predicted?B->GetP():B->GetX())+Q.RotateVector(B->CenterOfMass()))/100.;};
    for(int I=0;I<5;++I) if(Hubs[I])
    {
        auto* H=Hubs[I];const FVector C=Com(H),Omega=H->GetW();
        const int N=I<4?FMath::Min(In->TreadCounts[I],16):In->BenchRollerCount;
        for(int J=0;J<N;++J) if(Rollers[I][J])
        {
            auto* B=Rollers[I][J];const FVector R=Com(B)-C;
            const FVector F=B->M()*FVector::CrossProduct(Omega,FVector::CrossProduct(Omega,R));
            const FQuat MassQ=(Predicted?B->GetQ():B->GetR())*FQuat(B->RotationOfMass());
            const FVector T=In->bExactRollerSpin?RigidSpin::BearingTransport(MassQ,B->GetW(),FVector(B->I())/10000.,Omega):FVector::ZeroVector;
            // Coincident internal force pair at the roller COM. The second
            // half kick evaluates the force at the predicted pose. No transform
            // override, measured momentum error or global energy correction.
            Wrench(B,F,T,I<4);Wrench(H,-F,FVector::CrossProduct(R,-F)-T,I<4);
        }
    }
    for(int I=0;I<PredictorCount;++I)
    {
        auto* B=PredictorBodies[I];const FQuat Q=(Predicted?B->GetQ():B->GetR())*FQuat(B->RotationOfMass());
        const FVector V0=FVector(B->GetV())/100.,W0=B->GetW();
        const FVector DV=PredictorForceN[I]*(HalfDt/B->M());
        const FVector DW=Q.RotateVector(FVector(B->InvI())*Q.UnrotateVector(PredictorTorqueNm[I]*10000.*HalfDt));
        B->SetV((V0+DV)*100.);B->SetW(W0+DW);
        const double Work=FVector::DotProduct(PredictorForceN[I],V0+DV*.5)*HalfDt+FVector::DotProduct(PredictorTorqueNm[I],W0+DW*.5)*HalfDt;
        if(PredictorMain[I]) BearingKickMainJ+=Work;else BearingKickBenchJ+=Work;
    }
}

void FContactLoadSampler::IntegrateRollerSpin(bool Before)
{
    const auto* In=GetConsumerInput_Internal();if(!In) return;
    auto* RigidSolver=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
    const double Remaining=FMath::Max(0.,GetDeltaTime_Internal()-GetProducerOutputData_Internal().CompletedInnerS);
    const double MaxDt=RigidSolver->GetMaxDeltaTime_External();const double Dt=MaxDt>0?FMath::Min(Remaining,MaxDt):Remaining;
    for(auto& P:RigidSolver->GetParticles().GetNonDisabledDynamicView())
    {
        const int Id=P.UniqueIdx().Idx;bool Roller=false;
        for(int I=0;I<4;++I) for(int J=0;J<In->TreadCounts[I];++J) Roller |= Id==In->TreadIds[I][J];
        for(int J=0;J<In->BenchRollerCount;++J) Roller |= Id==In->BenchRollerIds[J];
        if(!Roller) continue;
        auto* B=P.Handle()->CastToRigidParticle();if(!B) continue;
        if(Before) B->SetGyroscopicTorqueEnabled(false);
        else
        {
            FQuat Q=FQuat(B->GetR())*FQuat(B->RotationOfMass());FVector W=B->GetW();
            if(!RigidSpin::Symmetric(Q,W,FVector(B->I()),Dt)) {checkf(false,TEXT("Exact roller spin requires an axisymmetric physical inertia"));continue;}
            B->SetTransformPQCom(B->PCom(),Q);B->SetW(W);
        }
    }
}

void FContactLoadSampler::OnPreIntegrate_Internal()
{
    for(auto& B:ReducedBodies)B=nullptr;
    PredictorCount=0;MotorBodyCount=0;
    CalibrationBody=nullptr; const auto* In=GetConsumerInput_Internal();
    bBenchStep=In && In->BenchCount>0;
    if(bBenchStep) BenchBeforeL=MeasureBenchMomentum(false);
    if(In && In->bMecanum && In->bMecanumControl)
    {
        auto* DriveSolver=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
        auto& Out=GetProducerOutputData_Internal();
        const double Remaining=FMath::Max(0.,GetDeltaTime_Internal()-Out.CompletedInnerS);
        const double MaxDt=DriveSolver->GetMaxDeltaTime_External();
        const double Dt=MaxDt>0?FMath::Min(Remaining,MaxDt):Remaining;
        Chaos::FPBDRigidParticleHandle* Hubs[4]={};Chaos::FPBDRigidParticleHandle* Stator=nullptr;
        for(auto& P:DriveSolver->GetParticles().GetNonDisabledDynamicView())
        {
            if(P.UniqueIdx().Idx==In->ChassisId) Stator=P.Handle()->CastToRigidParticle();
            for(int I=0;I<4;++I) if(P.UniqueIdx().Idx==In->WheelIds[I]) Hubs[I]=P.Handle()->CastToRigidParticle();
        }
        auto Torque=[&](Chaos::FPBDRigidParticleHandle* B,FVector T)
        {
            int K=0;while(K<MotorBodyCount && MotorBodies[K]!=B) ++K;
            if(K==MotorBodyCount) {check(MotorBodyCount<5);MotorBodies[K]=B;MotorAddedW[K]=Chaos::FVec3(0);++MotorBodyCount;}
            const auto Before=B->AngularAcceleration();B->AddTorque(T*10000.);MotorAddedW[K]+=B->AngularAcceleration()-Before;
        };
        if(Stator && Dt>0) for(int I=0;I<4;++I) if(Hubs[I])
        {
            const FVector Axis=Stator->GetQ().GetAxisY();
            const double Omega=FVector::DotProduct(FVector(Hubs[I]->GetW()),Axis);
            const double Relative=Omega-FVector::DotProduct(FVector(Stator->GetW()),Axis);
            const bool Expired=In->bMecanumSensorTorque&&JointClockS>In->MecanumLeaseUntilS;
            Out.bDriveCommandExpired|=Expired;
            const bool Powered=In->bMecanumPowered&&!Expired;
            // In sensor mode the external controller supplies measured-feedback
            // torque. True relative RPM remains a PLANT input for motor limits,
            // back-EMF, work and losses, never a substitute feedback measurement.
            const double Requested=In->bMecanumSensorTorque?In->MecanumSensorTorqueNm[I]:(In->MecanumTargetMps[I]-Relative*In->MecanumRadiusM)*In->MecanumGain;
            const double Request=Powered?FMath::Clamp(Requested,-In->MecanumTorqueLimitNm,In->MecanumTorqueLimitNm):0;
            auto Result=In->bMecanumDetailedMotor?ElectricDrive::Step(Request,Relative,In->MecanumMotor,In->MecanumTorqueLimitNm,Dt,Powered,MotorShaftStateNm[I]):ElectricDrive::Measure(Request,Relative,In->MecanumMotor,In->MecanumTorqueLimitNm);
            const auto Loss=WheelResistance::Evaluate(In->bReducedMecanum?ReducedNormalN[I]:In->MecanumLoadN[I],In->MecanumRadiusM,Omega,Relative,In->bMecanumResistance?In->MecanumCrr:0,0,0);
            const FQuat MassQ=FQuat(Hubs[I]->GetR())*FQuat(Hubs[I]->RotationOfMass());
            const FVector MassAxis=MassQ.UnrotateVector(Axis);
            const double InvAxis=FVector::DotProduct(MassAxis,FVector(Hubs[I]->InvI())*10000.*MassAxis);
            const double Rolling=WheelResistance::PassiveStepTorque(Loss.Rolling,Omega,InvAxis,Dt);
            AppliedMotorNm[I]=Axis*Result.OutputNm;AppliedRollingNm[I]=Axis*Rolling;
            Torque(Hubs[I],AppliedMotorNm[I]+AppliedRollingNm[I]);Torque(Stator,-AppliedMotorNm[I]);
            Out.MecanumMotor[I]=Result;Out.MecanumRequestNm[I]=Request;Out.MecanumRollingNm[I]=Rolling;Out.bMecanumMotorValid=true;
        }
    }
    if(In && In->bReducedMecanum)ApplyReducedContact();
    BearingKickDt=0;BearingKickMainJ=BearingKickBenchJ=0;
    if(In && In->bMecanum && In->bBearingPredictor)
    {
        auto* RigidSolver=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
        const double Remaining=FMath::Max(0.,GetDeltaTime_Internal()-GetProducerOutputData_Internal().CompletedInnerS);
        const double MaxDt=RigidSolver->GetMaxDeltaTime_External();
        BearingKickDt=MaxDt>0?FMath::Min(Remaining,MaxDt):Remaining;
        ApplyBearingKick(false,BearingKickDt*.5);
    }
    if(In && In->bMecanum && In->bExactRollerSpin) IntegrateRollerSpin(true);
    if(!In || In->CalibrationBodyId==INDEX_NONE) return;
    auto* RigidSolver=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
    for(auto& P:RigidSolver->GetParticles().GetNonDisabledDynamicView()) if(P.UniqueIdx().Idx==In->CalibrationBodyId)
    {
        CalibrationBody=P.Handle()->CastToRigidParticle(); if(!CalibrationBody) return;
        const auto A=CalibrationBody->Acceleration(),W=CalibrationBody->AngularAcceleration();
        const FVector F=In->CalibrationForceN*100;
        const FVector Arm=-CalibrationBody->GetR().RotateVector(FVector(CalibrationBody->CenterOfMass()));
        CalibrationBody->AddForce(F);
        CalibrationBody->AddTorque(FVector::CrossProduct(Arm,F)+In->CalibrationTorqueNm*10000);
        CalibrationAddedA=CalibrationBody->Acceleration()-A; CalibrationAddedW=CalibrationBody->AngularAcceleration()-W;
        return;
    }
}
void FContactLoadSampler::OnPostIntegrate_Internal()
{
    for(int I=0;I<4;++I)if(ReducedBodies[I]){auto* B=ReducedBodies[I];B->SetAcceleration(B->Acceleration()-ReducedAddedA[I]);B->SetAngularAcceleration(B->AngularAcceleration()-ReducedAddedW[I]);ReducedBodies[I]=nullptr;}
    for(int I=0;I<MotorBodyCount;++I) MotorBodies[I]->SetAngularAcceleration(MotorBodies[I]->AngularAcceleration()-MotorAddedW[I]);
    MotorBodyCount=0;
    const auto* In=GetConsumerInput_Internal();if(In && In->bMecanum && In->bExactRollerSpin) IntegrateRollerSpin(false);
    if(BearingKickDt>0) ApplyBearingKick(true,BearingKickDt*.5);
    if(bBenchStep) {BenchIntegratedL=MeasureBenchMomentum(true);BenchIntegrationDrift+=BenchIntegratedL-BenchBeforeL;}
    CaptureAuditVelocities();
    if(!CalibrationBody) return;
    CalibrationBody->SetAcceleration(CalibrationBody->Acceleration()-CalibrationAddedA);
    CalibrationBody->SetAngularAcceleration(CalibrationBody->AngularAcceleration()-CalibrationAddedW);
}

void FContactLoadSampler::OnContactModification_Internal(Chaos::FCollisionContactModifier& Modifier)
{
    const auto* In=GetConsumerInput_Internal();
    if(!In) return;
    auto IsRoad=[&](int32 Id) { if(Id==In->RoadId) return true; for(int32 S:In->SurfaceIds) if(S!=INDEX_NONE && Id==S) return true; return false; };
    auto* RigidSolver=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
    for(auto* Handle:RigidSolver->GetEvolution()->GetCollisionConstraints().GetConstraintHandles())
    {
        if(!Handle) continue;
        auto& Constraint=Handle->GetContact();
        if(!Constraint.IsEnabled() || !Constraint.IsCurrent() || Constraint.GetIsProbe()) continue;
        Chaos::FContactPairModifier Pair(&Constraint,Modifier);
        const auto Bodies=Pair.GetParticlePair();
        if(!Bodies[0] || !Bodies[1]) continue;
        const int Slot0=In->WheelSlot(Bodies[0]->UniqueIdx().Idx), Slot1=In->WheelSlot(Bodies[1]->UniqueIdx().Idx);
        // Bonded pads share one wheel: exclude only its own pad/pad and pad/rim
        // pairs. Other wheels, chassis, pallets and obstacles still collide.
        if(Slot0!=INDEX_NONE && Slot0==Slot1) { Pair.Disable(); continue; }
        const bool ReducedPair=In->bReducedMecanum && ((Slot0!=INDEX_NONE && IsRoad(Bodies[1]->UniqueIdx().Idx)) || (Slot1!=INDEX_NONE && IsRoad(Bodies[0]->UniqueIdx().Idx)));
        ON_SCOPE_EXIT
        {
            if(ReducedPair)
            {
                const int W=Slot0!=INDEX_NONE?0:1,G=1-W;const FVector Axis=Pair.GetParticleRotation(W).GetAxisZ();bool Rolling=false;
                for(int K=0;K<Pair.GetNumContacts();++K)if(!Pair.IsContactPointDisabled(K))Rolling|=ReducedMecanum::IsRollingContact(Pair.GetWorldNormal(K)*(W==0?1.:-1.),Axis);
                double Mu=In->ReducedRoadMu;for(int I=0;I<8;++I)if(In->SurfaceIds[I]==Bodies[G]->UniqueIdx().Idx)Mu=In->ReducedSurfaceMu[I];
                Mu=Rolling?0:FMath::Min(In->ReducedTyreMu,Mu);Pair.ModifyDynamicFriction(Mu);Pair.ModifyStaticFriction(Mu);
            }
        };
        if(!In->bSmoothTyreContacts) continue;
        int WheelIndex=INDEX_NONE, RoadIndex=INDEX_NONE, WheelSlot=INDEX_NONE;
        for(int I=0;I<2;++I)
        {
            if(IsRoad(Bodies[I]->UniqueIdx().Idx)) RoadIndex=I;
            const int J=In->WheelSlot(Bodies[I]->UniqueIdx().Idx); if(J!=INDEX_NONE) { WheelIndex=I; WheelSlot=J; }
        }
        if(WheelIndex==INDEX_NONE || RoadIndex==INDEX_NONE || WheelIndex==RoadIndex) continue;
        // The segmented model owns its real deformed convex surfaces. Projecting
        // them back onto the undeformed full-radius cylinder would erase it.
        if(In->TreadCounts[WheelSlot]>0) continue;
        const auto* Geometry=Pair.GetContactGeometry(RoadIndex);
        static const bool Diagnose=FParse::Param(FCommandLine::Get(),TEXT("ReducedContactDiagnostics"));
        static thread_local int ProbeCount=0;
        if(Diagnose && In->bReducedMecanum && Bodies[RoadIndex]->UniqueIdx().Idx!=In->RoadId && ProbeCount==0)
        {++ProbeCount;UE_LOG(LogTemp,Display,TEXT("SUPPORT_GEOMETRY id=%d type=%d min=%s max=%s"),Bodies[RoadIndex]->UniqueIdx().Idx,Geometry?int(Geometry->GetType()):-1,Geometry?*FVector(Geometry->BoundingBox().Min()).ToString():TEXT("none"),Geometry?*FVector(Geometry->BoundingBox().Max()).ToString():TEXT("none"));}
        // Analytic box faces and axle-parallel edges. Skew edges, vertices,
        // meshes and wheel sidewalls retain the original Chaos narrow phase.
        if(!Geometry || Geometry->GetType()!=Chaos::ImplicitObjectType::Box) continue;
        const auto Bounds=Geometry->BoundingBox();
        const auto RoadTM=Pair.GetShapeToWorld(RoadIndex);
        const FVector Center=Pair.GetParticlePosition(WheelIndex);
        const FVector Axis=Pair.GetParticleRotation(WheelIndex).GetAxisZ();
        for(int K=0;K<Pair.GetNumContacts();++K)
        {
            if(Pair.IsContactPointDisabled(K)) continue;
            const double Sign=WheelIndex==0?1:-1;
            const FVector LocalN=RoadTM.InverseTransformVectorNoScale(Pair.GetWorldNormal(K)*Sign);
            int D=0; for(int J=1;J<3;++J) if(FMath::Abs(LocalN[J])>FMath::Abs(LocalN[D])) D=J;
            FVector FaceN=FVector::ZeroVector; FaceN[D]=LocalN[D]>=0?1:-1;
            FVector N=RoadTM.TransformVectorNoScale(FaceN);
            Chaos::FVec3 Old0,Old1; Pair.GetWorldContactLocations(K,Old0,Old1);
            const FVector OldRoad=RoadTM.InverseTransformPositionNoScale(RoadIndex==0?Old0:Old1);
            if(Diagnose && ProbeCount<12 && In->bReducedMecanum && Center.X>800 && FMath::Abs(Center.Y)<20 && FMath::Abs(LocalN.Y)>.1)
            {++ProbeCount;const auto* WG=Pair.GetContactGeometry(WheelIndex);const auto WT=Pair.GetShapeToWorld(WheelIndex);UE_LOG(LogTemp,Display,TEXT("SUPPORT_PROBE center=%s axis=%s road=%s min=%s max=%s normal=%s wheel_bounds=%s/%s shape_center=%s shape_axis=%s"),*RoadTM.InverseTransformPositionNoScale(Center).ToString(),*RoadTM.InverseTransformVectorNoScale(Axis).ToString(),*OldRoad.ToString(),*FVector(Bounds.Min()).ToString(),*FVector(Bounds.Max()).ToString(),*LocalN.ToString(),*FVector(WG->BoundingBox().Min()).ToString(),*FVector(WG->BoundingBox().Max()).ToString(),*RoadTM.InverseTransformPositionNoScale(WT.GetTranslation()).ToString(),*RoadTM.InverseTransformVectorNoScale(WT.GetRotation().GetAxisZ()).ToString());}
            FVector Pw,Pr,EdgeW,EdgeR,EdgeN;
            bool Edge=In->bSmoothTyreEdges && WheelContactGeometry::ProjectEdge(
                RoadTM.InverseTransformPositionNoScale(Center),RoadTM.InverseTransformVectorNoScale(Axis),OldRoad,
                FVector(Bounds.Min()),FVector(Bounds.Max()),In->TyreRadiusCm,In->TyreHalfWidthCm,EdgeW,EdgeR,EdgeN);
            if(!Edge && In->bReducedMecanum && In->bSmoothTyreEdges)
                Edge=WheelContactGeometry::ProjectCylinderEdge(RoadTM.InverseTransformPositionNoScale(Center),RoadTM.InverseTransformVectorNoScale(Axis),OldRoad,
                    FVector(Bounds.Min()),FVector(Bounds.Max()),In->TyreRadiusCm,In->TyreHalfWidthCm,EdgeW,EdgeR,EdgeN);
            if(Edge)
            {
                Pw=RoadTM.TransformPositionNoScale(EdgeW); Pr=RoadTM.TransformPositionNoScale(EdgeR);
                N=RoadTM.TransformVectorNoScale(EdgeN);
                ++GetProducerOutputData_Internal().RefinedEdgeContacts;
            }
            else
            {
                if(FMath::Abs(LocalN[D])<.995) continue;
                if(In->bReducedMecanum)Pw=WheelContactGeometry::CylinderSupport(Center,Axis,N,WheelIndex==0?Old0:Old1,In->TyreRadiusCm,In->TyreHalfWidthCm);
                else if(!WheelContactGeometry::Project(Center,Axis,N,WheelIndex==0?Old0:Old1,In->TyreRadiusCm,In->TyreHalfWidthCm,Pw)) continue;
                FVector LocalP=RoadTM.InverseTransformPositionNoScale(Pw);
                bool Interior=true;
                for(int J=0;J<3;++J) if(J!=D) Interior &= LocalP[J]>Bounds.Min()[J]+.1 && LocalP[J]<Bounds.Max()[J]-.1
                    && OldRoad[J]>Bounds.Min()[J]+.1 && OldRoad[J]<Bounds.Max()[J]-.1;
                if(!Interior) continue;
                LocalP[D]=FaceN[D]>0?Bounds.Max()[D]:Bounds.Min()[D];
                Pr=RoadTM.TransformPositionNoScale(LocalP);
            }
            auto& ManifoldPoint=Constraint.GetManifoldPoints()[K];
            const bool KeepAnchor=In->bBrakeHolding[WheelSlot] && ManifoldPoint.Flags.bHasStaticFrictionAnchor;
            const auto Anchor0=ManifoldPoint.ShapeAnchorPoints[0], Anchor1=ManifoldPoint.ShapeAnchorPoints[1];
            Pair.ModifyWorldNormal(N*Sign,K);
            Pair.ModifyWorldContactLocations(WheelIndex==0?Pw:Pr,WheelIndex==0?Pr:Pw,K);
            // UE resets anchor locations above, but leaves the 'has anchor'
            // flag set. That would erase the tangential slip demand and lose
            // propulsion/braking. A newly projected smooth contact must use
            // the solver's contact-velocity friction path (as spheres do).
            // A near-stationary, service-braked tyre needs its EXISTING static
            // friction anchors. ModifyWorldContactLocations resets those local
            // points even though the engine flag survives. Preserve the pair
            // only for braking hold; Chaos still enforces the real mu*N cone.
            // Rolling contacts retain the velocity path (no stored anchor energy).
            if(KeepAnchor) { ManifoldPoint.ShapeAnchorPoints[0]=Anchor0; ManifoldPoint.ShapeAnchorPoints[1]=Anchor1; }
            ManifoldPoint.Flags.bHasStaticFrictionAnchor=KeepAnchor;
            ++GetProducerOutputData_Internal().RefinedContacts;
        }
    }
}

void FContactLoadSampler::OnPostSolve_Internal()
{
    const auto* Input=GetConsumerInput_Internal();
    if(!Input || Input->RoadId==INDEX_NONE) return;
    auto IsRoad=[&](int32 Id) { if(Id==Input->RoadId) return true; for(int32 Surface:Input->SurfaceIds) if(Surface!=INDEX_NONE && Id==Surface) return true; return false; };
    auto& Output=GetProducerOutputData_Internal();
    // One output belongs to the outer solver step. PostSolve may execute several
    // times for its inner substeps; accumulate impulses, but count this time ONCE.
    Output.IntervalS=GetDeltaTime_Internal();
    auto* RigidSolver=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
    // Match UE 5.6 AdvanceOneTimeStepTask's inner time slicing. Manifold
    // pushout is converted to impulse with the INNER dt, not the output dt.
    const double Remaining=FMath::Max(0.,Output.IntervalS-Output.CompletedInnerS);
    const double MaxInner=RigidSolver->GetMaxDeltaTime_External();
    const double InnerDt=MaxInner>0?FMath::Min(Remaining,MaxInner):Remaining;
    CaptureJointSensors(InnerDt);
    CompleteReducedContact(InnerDt);
    if(bBenchStep)
    {
        BenchConstraintDrift+=MeasureBenchMomentum(true)-BenchIntegratedL;BenchAuditTime+=InnerDt;
        if(BenchAuditTime-BenchAuditLogged>=1)
        {
            BenchAuditLogged=BenchAuditTime;
            UE_LOG(LogTemp,Display,TEXT("MECANUM_STAGE time=%.4f integration_delta_l=%s constraint_delta_l=%s"),BenchAuditTime,*BenchIntegrationDrift.ToString(),*BenchConstraintDrift.ToString());
        }
    }
    Output.CompletedInnerS+=InnerDt;
    ++Output.InnerSteps;
    Output.MaxInnerDt=FMath::Max(Output.MaxInnerDt,InnerDt);
    auto RotationStep=[](const Chaos::FPBDRigidParticleHandle* B)
    {
        FQuat D=FQuat(B->GetQ())*FQuat(B->GetR()).Inverse();
        if(D.W<0) D=D*(-1.);
        return D.ToRotationVector();
    };
    Output.BearingPredictorWorkJ+=BearingKickMainJ;Output.BenchPredictorWorkJ+=BearingKickBenchJ;
    if(Input->bAuditWork && InnerDt>0)
    {
        FVector StatorTorque=FVector::ZeroVector;
        for(int I=0;I<4;++I) StatorTorque-=Input->bMecanumControl?AppliedMotorNm[I]:Input->DriveTorqueNm[I];
        for(auto& Particle:RigidSolver->GetParticles().GetNonDisabledDynamicView())
        {
            const auto* B=Particle.Handle()->CastToRigidParticle(); if(!B) continue;
            const int Id=B->UniqueIdx().Idx; const FVector Angle=RotationStep(B);
            if(Id==Input->ChassisId) Output.DriveWorkJ+=FVector::DotProduct(StatorTorque,Angle);
            for(int I=0;I<4;++I) if(Id==Input->WheelIds[I])
            {
                Output.DriveWorkJ+=FVector::DotProduct(Input->bMecanumControl?AppliedMotorNm[I]:Input->DriveTorqueNm[I],Angle);
                Output.RollingWorkJ+=FVector::DotProduct(Input->bMecanumControl?AppliedRollingNm[I]:Input->RollingTorqueNm[I],Angle);
            }
        }
    }
    // UE 5.6's GT GetConstraintForce exposes the LAST inner-step impulse via
    // JointConstraintProxy, not a frame-averaged torque. Integrate PT impulses
    // here, just like road contact impulses, before dividing by the outer time.
    const auto& Joints=RigidSolver->GetEvolution()->GetJointCombinedConstraints();
    FVector HubPosition[4]={},HubAxis[4]={}; bool HaveHub[4]={false,false,false,false};
    for(const auto* Container:{&Joints.LinearConstraints,&Joints.NonLinearConstraints})
    for(const auto* Joint:Container->GetConstConstraintHandles())
    {
        if(!Joint || !Joint->IsConstraintEnabled()) continue;
        const auto Particles=Joint->GetConstrainedParticles();
        if(!Particles[0] || !Particles[1]) continue;
        const int32 A=Particles[0]->UniqueIdx().Idx, B=Particles[1]->UniqueIdx().Idx;
        if(Input->bAuditWork && InnerDt>0)
        {
            const auto& S=Joint->GetSettings();
            const bool TorqueOnly=S.LinearMotionTypes[0]==Chaos::EJointMotionType::Free && S.LinearMotionTypes[1]==Chaos::EJointMotionType::Free && S.LinearMotionTypes[2]==Chaos::EJointMotionType::Free && S.AngularMotionTypes[0]==Chaos::EJointMotionType::Free && S.AngularMotionTypes[1]==Chaos::EJointMotionType::Free && S.AngularMotionTypes[2]==Chaos::EJointMotionType::Free;
            bool Hub=false;for(int I=0;I<4;++I) Hub |= A==Input->WheelIds[I] || B==Input->WheelIds[I];
            if(TorqueOnly && Hub)
            {
                const auto* P0=Particles[0]->CastToRigidParticle();const auto* P1=Particles[1]->CastToRigidParticle();
                if(P0 && P1) Output.ShaftJointWorkJ+=FVector::DotProduct(FVector(Joint->GetAngularImpulse())/10000.,RotationStep(P0)-RotationStep(P1))/InnerDt;
            }
        }
        for(int I=0;I<4;++I) if(Input->TreadCounts[I]>0)
        {
            const bool First=A==Input->WheelIds[I], Second=B==Input->WheelIds[I];
            if(!First && !Second) continue;
            const auto* Hub=Particles[First?0:1]->CastToRigidParticle();
            if(!Hub) continue;
            HubPosition[I]=FVector(Hub->GetP()); HubAxis[I]=FVector(Hub->GetQ().GetAxisZ()); HaveHub[I]=true;
            const int Pad=Input->PadSlot(I,First?B:A); if(Pad==INDEX_NONE || InnerDt<=0) continue;
            const auto* Tread=Particles[First?1:0]->CastToRigidParticle(); if(!Tread) continue;
            if(Input->bMecanum)
            {
                const FVector Axis=Tread->GetQ().GetAxisX();
                // Angular impulse acts on solver particle 0. Keep the signed
                // work, including any positive numerical remainder.
                const double J=FVector::DotProduct(FVector(Joint->GetAngularImpulse())/10000.,Axis)*(First?-1:1);
                Output.RollerBearingImpulseNms[I][Pad]+=J;
                Output.RollerBearingWorkJ[I]+=J*FVector::DotProduct(Axis,RotationStep(Tread)-RotationStep(Hub))/InnerDt;
                continue;
            }
            const double Angle=2*UE_DOUBLE_PI*Pad/Input->TreadCounts[I]+Input->TreadPhaseRad; const FVector Local(FMath::Cos(Angle),FMath::Sin(Angle),0);
            const double Before=FVector::DotProduct(FVector(Hub->GetR().UnrotateVector(Tread->GetX()-Hub->GetX())),Local)/100.;
            const double After=FVector::DotProduct(FVector(Hub->GetQ().UnrotateVector(Tread->GetP()-Hub->GetP())),Local)/100.;
            // The native drive damps WORLD connector displacement, projected
            // onto its CURRENT frame. Differencing coordinates in two rotating
            // frames is a different rate and corrupts rolling-loss estimates.
            const FVector HubAnchor0=FVector(Hub->GetX())+Hub->GetR().RotateVector(Local*(Input->TreadRestRadiusM*100));
            const FVector HubAnchor1=FVector(Hub->GetP())+Hub->GetQ().RotateVector(Local*(Input->TreadRestRadiusM*100));
            const FVector ConnectorRate=(FVector(Tread->GetP())-HubAnchor1-FVector(Tread->GetX())+HubAnchor0)/(100*InnerDt);
            const double Rate=FVector::DotProduct(ConnectorRate,Hub->GetQ().RotateVector(Local));
            Output.TyreDissipatedJ+=Input->TreadPadC*Rate*Rate*InnerDt;
            if(Input->bTreadCoupled)
            {
                const FVector Tangent(-FMath::Sin(Angle),FMath::Cos(Angle),0);
                const double ShearRate=FVector::DotProduct(ConnectorRate,Hub->GetQ().RotateVector(Tangent));
                const FQuat R0=Hub->GetR().Inverse()*Tread->GetR(),R1=Hub->GetQ().Inverse()*Tread->GetQ();
                const double RotationRate=std::remainder(2*(FMath::Atan2(R1.Z,R1.W)-FMath::Atan2(R0.Z,R0.W)),2*UE_DOUBLE_PI)/InnerDt;
                Output.TyreDissipatedJ+=(Input->TreadShearC*ShearRate*ShearRate+Input->TreadRotC*RotationRate*RotationRate)*InnerDt;
            }
            Output.TyrePeakCompressionM=FMath::Max(Output.TyrePeakCompressionM,Input->TreadRestRadiusM-After);
        }
        if(Input->bTreadCoupled && InnerDt>0)
        {
            const int Slot=Input->WheelSlot(A);
            if(Slot!=INDEX_NONE && Input->WheelSlot(B)==Slot)
            {
                const int I=Input->PadSlot(Slot,A),J=Input->PadSlot(Slot,B),N=Input->TreadCounts[Slot];
                if(I!=INDEX_NONE && J!=INDEX_NONE && ((I+1)%N==J || (J+1)%N==I))
                {
                    const auto* P0=Particles[0]->CastToRigidParticle(); const auto* P1=Particles[1]->CastToRigidParticle();
                    if(P0 && P1)
                    {
                        const double Half=UE_DOUBLE_PI/N,Sign=(I+1)%N==J?1:-1;
                        const double Rc=Input->TreadRestRadiusM*100;
                        const FVector A0(Rc*(FMath::Cos(Half)-1),Sign*Rc*FMath::Sin(Half),0),A1(A0.X,-A0.Y,0);
                        const FVector D0=FVector(P1->GetX()+P1->GetR().RotateVector(A1)-P0->GetX()-P0->GetR().RotateVector(A0));
                        const FVector D1=FVector(P1->GetP()+P1->GetQ().RotateVector(A1)-P0->GetP()-P0->GetQ().RotateVector(A0));
                        const FVector V=P0->GetQ().UnrotateVector(D1-D0)/(100*InnerDt);
                        const FQuat Q0=P0->GetR().Inverse()*P1->GetR(),Q1=P0->GetQ().Inverse()*P1->GetQ();
                        const double W=std::remainder(2*(FMath::Atan2(Q1.Z,Q1.W)-FMath::Atan2(Q0.Z,Q0.W)),2*UE_DOUBLE_PI)/InnerDt;
                        Output.TyreDissipatedJ+=(Input->TreadLinkC*(V.X*V.X+V.Y*V.Y)+Input->TreadBendC*W*W)*InnerDt;
                    }
                }
            }
        }
        const int32 Other=A==Input->ChassisId?B:B==Input->ChassisId?A:INDEX_NONE;
        for(int I=0;I<4;++I) if(Other!=INDEX_NONE && Other==Input->KnuckleIds[I])
        {
            const FVector Impulse=FVector(Joint->GetAngularImpulse())/10000.;
            Output.SteerImpulseNms[I]+=Impulse;
            const auto* P0=Particles[0]->CastToRigidParticle(); const auto* P1=Particles[1]->CastToRigidParticle();
            // Container impulse acts on particle 0 (solver particle order is reversed).
            if(Input->bAuditWork && !Input->bMecanum && P0 && P1 && InnerDt>0)
                Output.SteeringJointWorkJ+=FVector::DotProduct(Impulse,(RotationStep(P0)-RotationStep(P1))/InnerDt);
        }
        for(int I=0;I<4;++I)
        {
            const bool Forward=A==Input->KnuckleIds[I] && B==Input->CarrierIds[I];
            const bool Backward=B==Input->KnuckleIds[I] && A==Input->CarrierIds[I];
            if(!Forward && !Backward) continue;
            Output.AxleImpulseNs[I]+=FVector(Joint->GetLinearImpulse())/100.;
            const auto* Hub=Particles[Forward?0:1]->CastToRigidParticle();
            const auto* Belt=Particles[Forward?1:0]->CastToRigidParticle();
            if(!Hub || !Belt || InnerDt<=0 || !Input->bTyreEnabled) continue;
            // The implicit position drive damps connector displacement over dt.
            // PostSolve precedes X<-P commit: GetV is not that displacement rate.
            const double Before=FVector::DotProduct(FVector(Belt->GetX()-Hub->GetX()),FVector(Hub->GetR().GetAxisZ()))/100.;
            const double After=FVector::DotProduct(FVector(Belt->GetP()-Hub->GetP()),FVector(Hub->GetQ().GetAxisZ()))/100.;
            const FVector Delta=FVector(Belt->GetP()-Hub->GetP()-Belt->GetX()+Hub->GetX());
            const double Rate=FVector::DotProduct(Delta,FVector(Hub->GetQ().GetAxisZ()))/(100*InnerDt);
            Output.TyreDissipatedJ+=Input->TyreDampingNsPm*Rate*Rate*InnerDt;
            Output.TyrePeakCompressionM=FMath::Max(Output.TyrePeakCompressionM,After);
        }
    }
    if(Input->BenchCount>0)
    {
        Output.BenchL=FVector::ZeroVector;Output.BenchEnergyJ=0;Output.bBenchValid=false;
        for(auto& P:RigidSolver->GetParticles().GetNonDisabledDynamicView())
        {
            bool Wanted=false;for(int I=0;I<Input->BenchCount;++I) Wanted |= P.UniqueIdx().Idx==Input->BenchIds[I];
            if(!Wanted) continue;
            const auto* B=P.Handle()->CastToRigidParticle();if(!B) continue;
            const FQuat Q=FQuat(B->GetQ())*FQuat(B->RotationOfMass());
            const FVector W=Q.UnrotateVector(FVector(B->GetW())),I=FVector(B->I())/10000.;
            const FVector Com=(B->GetP()+B->GetQ().RotateVector(B->CenterOfMass()))/100.;
            const FVector V=FVector(B->GetV())/100.;
            Output.BenchL+=Q.RotateVector(I*W)+FVector::CrossProduct(Com-Input->BenchOriginM,V*B->M());
            Output.BenchEnergyJ+=.5*(FVector::DotProduct(W,I*W)+B->M()*V.SizeSquared());Output.bBenchValid=true;
        }
    }
    MeasureContactWork(InnerDt);
    const auto& Handles=RigidSolver->GetEvolution()->GetCollisionConstraints().GetConstConstraintHandles();
    double StepImpulseNs[4]={0,0,0,0},ReducedSupportNs[4]={};
    FVector SensorForkImpulseNs[2]={};
    FVector StepPoints[4]={},StepNormals[4]={};double StepPointWeights[4]={};
    int32 ActivePoints[4]={0,0,0,0};
    for(const auto* Handle:Handles)
    {
        if(!Handle) continue;
        const auto& C=Handle->GetContact();
        if(!C.IsEnabled() || !C.IsCurrent() || C.GetIsProbe()) continue;
        const int32 Id0=C.GetParticle0()->UniqueIdx().Idx, Id1=C.GetParticle1()->UniqueIdx().Idx;
        const int32 WheelId=IsRoad(Id0)?Id1:IsRoad(Id1)?Id0:INDEX_NONE;
        const double J=FMath::Abs(Chaos::FVec3::DotProduct(Chaos::FVec3(C.AccumulatedImpulse),C.CalculateWorldContactNormal()))/100.;
        for(int32 I=0;I<4;++I) if(WheelId!=INDEX_NONE && Input->WheelSlot(WheelId)==I)
        {
            if(!Input->bReducedMecanum){Output.ImpulseNs[I]+=J;Output.WheelMuNs[I]+=J*C.GetDynamicFriction();StepImpulseNs[I]+=J;}
            if(J>0) ++Output.Contacts[I];
            for(int K=0;K<C.NumManifoldPoints();++K)
            {
                const auto& R=C.GetManifoldPointResult(K);
                if(!C.IsManifoldPointActive(K) || !R.bIsValid || InnerDt<=1e-8) continue;
                const auto& P=C.GetManifoldPoint(K).ContactPoint;
                const FVector Impulse=(FVector(R.NetImpulse)+FVector(R.NetPushOut)/InnerDt)/100.;
                const FVector Position=C.GetShapeWorldTransform0().TransformPosition(Chaos::FVec3(P.ShapeContactPoints[0]));
                const FVector Center=Id0==WheelId?FVector(C.GetParticle0()->GetX()):FVector(C.GetParticle1()->GetX());
                const FVector Normal=C.GetShapeWorldTransform1().TransformVector(Chaos::FVec3(P.ShapeContactNormal));
                const FVector Tangent=Impulse-Normal*FVector::DotProduct(Impulse,Normal);
                Output.WheelFrictionImpulseNs[I]+=Tangent*(Id0==WheelId?1:-1);
                Output.WheelMomentNs[I]+=FVector::CrossProduct((Position-(HaveHub[I]?HubPosition[I]:Center))/100.,Tangent)*(Id0==WheelId?1:-1);
                const double NormalJ=FVector::DotProduct(Impulse,Normal);
                if(NormalJ>1e-10)
                {
                    // Use the SAME constraint pose as the reported contact;
                    // previous-step X/R would invent a moving lever arm.
                    const auto& WheelTM=Id0==WheelId?C.GetShapeWorldTransform0():C.GetShapeWorldTransform1();
                    const auto& RoadTM=Id0==WheelId?C.GetShapeWorldTransform1():C.GetShapeWorldTransform0();
                    const FVector RoadPoint=RoadTM.TransformPosition(Chaos::FVec3(P.ShapeContactPoints[Id0==WheelId?1:0]));
                    Output.WheelContact[I].AddPoint(RoadPoint/100.,HaveHub[I]?HubPosition[I]/100.:FVector(WheelTM.GetTranslation())/100.,HaveHub[I]?HubAxis[I]:FVector(WheelTM.GetRotation().GetAxisZ()),Normal*(Id0==WheelId?1:-1),NormalJ,Input->TyreHalfWidthCm/100.);
                    if(Input->bReducedMecanum)
                    {
                        Output.ImpulseNs[I]+=NormalJ;StepImpulseNs[I]+=NormalJ;
                        const FVector SupportNormal=Normal*(Id0==WheelId?1:-1);
                        if(ReducedMecanum::IsRollingContact(SupportNormal,HaveHub[I]?HubAxis[I]:FVector(WheelTM.GetRotation().GetAxisZ())))
                        {ReducedSupportNs[I]+=NormalJ;StepPointWeights[I]+=NormalJ;StepPoints[I]+=RoadPoint*NormalJ;StepNormals[I]+=SupportNormal*NormalJ;}
                        double Mu=Input->ReducedRoadMu;const int GroundId=Id0==WheelId?Id1:Id0;
                        for(int G=0;G<8;++G)if(Input->SurfaceIds[G]==GroundId)Mu=Input->ReducedSurfaceMu[G];
                        ReducedMu[I]=FMath::Min(Input->ReducedTyreMu,Mu);Output.WheelMuNs[I]+=NormalJ*ReducedMu[I];
                    }
                    const int Pad=Input->PadSlot(I,WheelId); if(Pad!=INDEX_NONE) {Output.TreadNormalNs[I][Pad]+=NormalJ;Output.TreadPointMNs[I][Pad]+=RoadPoint/100.*NormalJ;}
                    if(Input->bMecanum && !Input->bReducedMecanum)
                    {
                        const auto* B=(Id0==WheelId?C.GetParticle0():C.GetParticle1())->CastToRigidParticle();
                        const auto* Ground=(Id0==WheelId?C.GetParticle1():C.GetParticle0())->CastToRigidParticle();
                        auto PointV=[](const Chaos::FPBDRigidParticleHandle* Body,FVector Point)
                        {
                            if(!Body) return FVector::ZeroVector;
                            const FVector Com=Body->GetP()+Body->GetQ().RotateVector(Body->CenterOfMass());
                            return (FVector(Body->GetV())+FVector::CrossProduct(FVector(Body->GetW()),Point-Com))/100.;
                        };
                        if(B)
                        {
                            const FVector Point=WheelTM.TransformPosition(Chaos::FVec3(P.ShapeContactPoints[Id0==WheelId?0:1]));
                            const FVector Relative=PointV(B,Point)-PointV(Ground,RoadPoint);
                            const FVector Axis=FVector::VectorPlaneProject(FVector(B->GetQ().GetAxisX()),Normal).GetSafeNormal();
                            const FVector Across=FVector::CrossProduct(Normal,Axis).GetSafeNormal();
                            Output.SlipAlongM2Ns[I]+=FMath::Square(FVector::DotProduct(Relative,Axis))*NormalJ;
                            Output.SlipAcrossM2Ns[I]+=FMath::Square(FVector::DotProduct(Relative,Across))*NormalJ;
                            Output.SlipWeightNs[I]+=NormalJ;
                            Output.FrictionUsedNs[I]+=Tangent.Size();
                        }
                    }
                    ++ActivePoints[I];
                }
            }
        }
        if(Input->PalletId==INDEX_NONE) continue;
        const int32 Other=Id0==Input->PalletId?Id1:Id1==Input->PalletId?Id0:INDEX_NONE;
        if(Other!=INDEX_NONE && IsRoad(Other)) Output.PalletGroundNs+=J;
        if(Other!=INDEX_NONE && (Other==Input->RackIds[0] || Other==Input->RackIds[1]))for(int K=0;K<C.NumManifoldPoints();++K)
        {
            const auto& R=C.GetManifoldPointResult(K);if(!C.IsManifoldPointActive(K)||!R.bIsValid||InnerDt<=1e-8)continue;
            const FVector N=C.GetShapeWorldTransform1().TransformVector(Chaos::FVec3(C.GetManifoldPoint(K).ContactPoint.ShapeContactNormal));
            Output.PalletRackNs+=FMath::Abs(FVector::DotProduct((FVector(R.NetImpulse)+FVector(R.NetPushOut)/InnerDt)/100.,N));
        }
        if(Input->CargoId!=INDEX_NONE)
        {
            const int32 CargoOther=Id0==Input->CargoId?Id1:Id1==Input->CargoId?Id0:INDEX_NONE;
            if(CargoOther!=INDEX_NONE && IsRoad(CargoOther)) Output.Cargo.GroundNormalNs+=J;
            else if(CargoOther!=INDEX_NONE && CargoOther!=Input->PalletId) Output.Cargo.OtherNormalNs+=J;
            if(CargoOther==Input->PalletId) for(int K=0;K<C.NumManifoldPoints();++K)
            {
                const auto& Result=C.GetManifoldPointResult(K);
                if(!C.IsManifoldPointActive(K) || !Result.bIsValid || InnerDt<=1e-8) continue;
                const auto& Point=C.GetManifoldPoint(K).ContactPoint;
                const FVector Impulse=(FVector(Result.NetImpulse)+FVector(Result.NetPushOut)/InnerDt)/100.;
                const FVector Normal=C.GetShapeWorldTransform1().TransformVector(Chaos::FVec3(Point.ShapeContactNormal));
                const double Sign=Id0==Input->CargoId?1:-1;
                const double NormalJ=FVector::DotProduct(Impulse,Normal);
                const FVector Tangent=Impulse-Normal*NormalJ;
                Output.Cargo.ImpulseNs+=Impulse*Sign;
                Output.Cargo.NormalImpulseNs+=Normal*NormalJ*Sign;
                Output.Cargo.FrictionImpulseNs+=Tangent*Sign;
                Output.Cargo.NormalNs+=FMath::Abs(NormalJ);
                Output.Cargo.FrictionMagnitudeNs+=Tangent.Size();
                const FVector Position=C.GetShapeWorldTransform0().TransformPosition(Chaos::FVec3(Point.ShapeContactPoints[0]));
                Output.Cargo.PointWeightedMNs+=Position/100.*FMath::Abs(NormalJ);
                if(FMath::Abs(NormalJ)>0) ++Output.Cargo.Points;
            }
        }
        for(int I=0;I<2;++I) if(Other!=INDEX_NONE && [&]() { for(int J=0;J<8;++J) if(Other==Input->TineIds[I*8+J]) return true; return false; }())
        {
            // A box/box pair has several support points. Using the constraint's
            // closest point for its total impulse gives a false CoP at the tip.
            for(int K=0;K<C.NumManifoldPoints();++K)
            {
                const auto& Result=C.GetManifoldPointResult(K);
                if(!C.IsManifoldPointActive(K) || !Result.bIsValid || InnerDt<=1e-8) continue;
                const auto& Point=C.GetManifoldPoint(K).ContactPoint;
                const FVector Impulse=(FVector(Result.NetImpulse)+FVector(Result.NetPushOut)/InnerDt)/100.;
                const FVector Normal=C.GetShapeWorldTransform1().TransformVector(Chaos::FVec3(Point.ShapeContactNormal));
                const double NormalJ=FMath::Abs(FVector::DotProduct(Impulse,Normal));
                const FVector Position=C.GetShapeWorldTransform0().TransformPosition(Chaos::FVec3(Point.ShapeContactPoints[0]));
                Output.ForkVerticalNs[I]+=(Id0==Input->PalletId?1:-1)*Impulse.Z;
                SensorForkImpulseNs[I]+=(Id0==Input->PalletId?1:-1)*Impulse;
                Output.ForkNormalNs[I]+=NormalJ;
                Output.ForkFrictionNs[I]+=(Impulse-Normal*FVector::DotProduct(Impulse,Normal)).Size();
                Output.ForkPointWeighted[I]+=Position/100.*NormalJ;
            }
        }
    }
    if(InnerDt>1e-8) for(int I=0;I<4;++I) if(Input->WheelIds[I]!=INDEX_NONE)
    {
        Output.WheelContact[I].PeakActivePoints=FMath::Max(Output.WheelContact[I].PeakActivePoints,ActivePoints[I]);
        const double N=StepImpulseNs[I]/InnerDt;
        if(Input->bReducedMecanum)
        {
            ReducedNormalN[I]=ReducedSupportNs[I]/InnerDt;
            if(StepPointWeights[I]>1e-8){ReducedPointCm[I]=StepPoints[I]/StepPointWeights[I];ReducedNormal[I]=StepNormals[I].GetSafeNormal();}
        }
        Output.PeakNormalN[I]=FMath::Max(Output.PeakNormalN[I],N);
        Output.MinNormalN[I]=FMath::Min(Output.MinNormalN[I],N);
    }
    CaptureDynamicsSensors(InnerDt,SensorForkImpulseNs);
}
