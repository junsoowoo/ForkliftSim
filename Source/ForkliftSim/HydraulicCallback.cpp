#include "HydraulicCallback.h"
#include "PBDRigidsSolver.h"
#include "ChainConstitutive.h"

void FHydraulicCallback::OnPreIntegrate_Internal()
{
    Bodies[0]=Bodies[1]=Bodies[2]=nullptr;
    const auto* In=GetConsumerInput_Internal();
    if(!In || In->ChassisId==INDEX_NONE || In->CarriageId==INDEX_NONE) return;
    auto* RigidSolver=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
    for(auto& P:RigidSolver->GetParticles().GetNonDisabledDynamicView())
    {
        if(P.UniqueIdx().Idx==In->CarriageId) Bodies[0]=P.Handle()->CastToRigidParticle();
        if(P.UniqueIdx().Idx==In->ChassisId) Bodies[1]=P.Handle()->CastToRigidParticle();
        if(P.UniqueIdx().Idx==In->MastId) Bodies[2]=P.Handle()->CastToRigidParticle();
    }
    const bool Chained=In->Settings.bChainMast;
    if(!Bodies[0] || !Bodies[1] || (Chained && !Bodies[2])) { Bodies[0]=Bodies[1]=Bodies[2]=nullptr; return; }
    auto& Out=GetProducerOutputData_Internal();
    const double Remaining=FMath::Max(0.,double(GetDeltaTime_Internal())-Out.CompletedS);
    const double MaxDt=RigidSolver->GetMaxDeltaTime_External();
    const double Dt=MaxDt>0?FMath::Min(Remaining,MaxDt):Remaining;
    if(Dt<=0) { Bodies[0]=Bodies[1]=nullptr; return; }
    Out.CompletedS+=Dt;
    const bool Expired=In->Command.Direct&&(!FMath::IsFinite(In->CommandValidUntilS)||SensorClockS+Dt>In->CommandValidUntilS);
    const auto Command=HydraulicLease(In->Command,In->CommandValidUntilS,SensorClockS+Dt);Out.CommandExpired|=Expired;
    if(!bInitialized) { State.PressurePa=In->InitialPressurePa; bInitialized=true; }
    const FVector Axis=Bodies[1]->GetR().GetAxisZ(), Point=Bodies[0]->GetX();
    const FVector Relative=Point-FVector(Bodies[1]->GetX());
    const double H=(FVector::DotProduct(Relative,Axis)-In->BaseZCm)/100.;
    auto OriginVelocity=[](const Chaos::FPBDRigidParticleHandle* B)
    { return FVector(B->GetV())-FVector::CrossProduct(FVector(B->GetW()),B->GetR().RotateVector(FVector(B->CenterOfMass()))); };
    const FVector BaseV=OriginVelocity(Bodies[1])+FVector::CrossProduct(FVector(Bodies[1]->GetW()),Relative);
    const double V=FVector::DotProduct(OriginVelocity(Bodies[0])-BaseV,Axis)/100.;
    double Forces[3]={0,0,0};
    if(Chained)
    {
        const auto& C=In->Settings; const double R=C.ChainRatio;
        const FVector MastRelative=FVector(Bodies[2]->GetX()-Bodies[1]->GetX());
        const double MastH=(FVector::DotProduct(MastRelative,Axis)-In->MastBaseZCm)/100.;
        const FVector MastBaseV=OriginVelocity(Bodies[1])+FVector::CrossProduct(FVector(Bodies[1]->GetW()),MastRelative);
        const double MastV=FVector::DotProduct(OriginVelocity(Bodies[2])-MastBaseV,Axis)/100.;
        State.Step(C,Dt,In->TargetM/R,MastH,MastV,In->TravelM/R,In->ForceCapN*R,Command);
        const double Stretch=R*MastH-H, Rate=R*MastV-V;
        const auto Response=ChainConstitutive::Evaluate(C.ChainK,C.ChainD,Stretch,Rate);
        const double Tension=Response.TensionN;
        State.ChainLossW=Response.LossW; State.ChainLossJ+=Response.LossW*Dt;
        // Rope on carriage +T; movable pulley -R*T; fixed rope end +(R-1)*T.
        // The cylinder acts on the actual movable mast body, not the carriage.
        Forces[0]=Tension; Forces[1]=-State.ForceN+(R-1)*Tension; Forces[2]=State.ForceN-R*Tension;
        State.ChainTensionN=Tension; State.ChainStretchM=FMath::Max(0.,Stretch);
        State.ChainStoredJ=Response.StoredJ; State.MastHeightM=MastH;
    }
    else
    {
        State.Step(In->Settings,Dt,In->TargetM,H,V,In->TravelM,In->ForceCapN,Command);
        Forces[0]=State.ForceN; Forces[1]=-State.ForceN;
    }
    for(int I=0;I<(Chained?3:2);++I)
    {
        auto* B=Bodies[I];
        const Chaos::FVec3 Force=Axis*(Forces[I]*100);
        const auto OldA=B->Acceleration(), OldW=B->AngularAcceleration();
        B->AddForce(Force);
        // Both forces act on the SAME line/point. Include reaction moment on
        // the chassis, rather than accidentally applying both forces at COMs.
        const FVector COM=FVector(B->GetX())+B->GetR().RotateVector(FVector(B->CenterOfMass()));
        B->AddTorque(FVector::CrossProduct(Point-COM,FVector(Force)));
        AddedA[I]=B->Acceleration()-OldA; AddedW[I]=B->AngularAcceleration()-OldW;
    }
    Out.State=State; Out.HeightM=H; Out.VelocityMps=V;
    Out.PeakPressurePa=FMath::Max(Out.PeakPressurePa,State.PressurePa);
    Out.PeakDcW=FMath::Max(Out.PeakDcW,State.DcW);
    Out.ReliefVolumeM3+=State.ReliefM3s*Dt;
    SensorClockS+=Dt;
    if(In->bPressureSensor&&In->PressureSamplePeriodS>0&&SensorClockS+1e-9>=NextSensorS)
    {
        NextSensorS=(FMath::FloorToDouble((SensorClockS+1e-9)/In->PressureSamplePeriodS)+1)*In->PressureSamplePeriodS;
        DynamicsSensor::FTruth T;T.Channel=1;T.TimeS=SensorClockS;T.Value[0]=State.PressurePa;Out.SensorSamples.Add(T);
    }
}
void FHydraulicCallback::OnPostIntegrate_Internal()
{
    // Game-thread forces persist across inner steps. Remove JUST our temporary
    // accelerations after integration so they do not accumulate every substep.
    for(int I=0;I<3;++I) if(Bodies[I])
    {
        Bodies[I]->SetAcceleration(Bodies[I]->Acceleration()-AddedA[I]);
        Bodies[I]->SetAngularAcceleration(Bodies[I]->AngularAcceleration()-AddedW[I]);
    }
}
