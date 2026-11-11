#include "ContactLoadSampler.h"
#include "PBDRigidsSolver.h"

void FContactLoadSampler::CaptureJointSensors(double Dt)
{
    const auto* In=GetConsumerInput_Internal();if(!In||!In->bJointSensors||Dt<=0)return;
    JointClockS+=Dt;
    auto* S=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
    const Chaos::FPBDRigidParticleHandle* Hubs[4]={};const Chaos::FPBDRigidParticleHandle* Mounts[4]={};
    const Chaos::FPBDRigidParticleHandle* Body=nullptr;const Chaos::FPBDRigidParticleHandle* Carriage=nullptr;
    for(auto& P:S->GetParticles().GetNonDisabledDynamicView())
    {
        const int Id=P.UniqueIdx().Idx;const auto* R=P.Handle()->CastToRigidParticle();
        if(Id==In->ChassisId)Body=R;if(Id==In->ForkCarriageId)Carriage=R;
        for(int I=0;I<4;++I){if(Id==In->WheelIds[I])Hubs[I]=R;if(Id==In->KnuckleIds[I])Mounts[I]=R;}
    }
    if(!Body||!Carriage)return;
    for(int I=0;I<4;++I)if(!Hubs[I]||!Mounts[I])return;
    for(int I=0;I<4;++I)JointShaftRad[I]+=JointSensor::ShaftStep(FQuat(Hubs[I]->GetR()),FQuat(Hubs[I]->GetQ()),FQuat(Mounts[I]->GetR()),FQuat(Mounts[I]->GetQ()));
    if(JointClockS+1e-9<NextJointSampleS)return;
    const double Period=In->JointSamplePeriodS;
    if(Period<=0)return;
    NextJointSampleS=(FMath::FloorToDouble((JointClockS+1e-9)/Period)+1)*Period;
    JointSensor::FTruth T;T.TimeS=JointClockS;
    for(int I=0;I<4;++I)T.AngleRad[I]=JointShaftRad[I];
    T.HeightM=FQuat(Body->GetQ()).UnrotateVector(FVector(Carriage->GetP()-Body->GetP())).Z/100.-In->JointForkBaseM;
    GetProducerOutputData_Internal().JointSamples.Add(T);
}

void FContactLoadSampler::CaptureDynamicsSensors(double Dt,const FVector* ForkImpulseNs)
{
    const auto* In=GetConsumerInput_Internal();if(!In||!In->bDynamicsSensors||Dt<=0||In->DynamicsPeriodS<=0)return;
    DynamicsClockS+=Dt;
    auto* RigidSolver=static_cast<Chaos::FPBDRigidsSolver*>(GetSolver());
    const Chaos::FPBDRigidParticleHandle* Body=nullptr;
    for(auto& P:RigidSolver->GetParticles().GetNonDisabledDynamicView())if(P.UniqueIdx().Idx==In->ChassisId){Body=P.Handle()->CastToRigidParticle();break;}
    if(!Body)return;
    const FQuat Rotation(Body->GetQ());const FVector Omega(Body->GetW());
    for(int I=0;I<2;++I)LoadImpulseNs[I]+=FVector::DotProduct(ForkImpulseNs[I],Rotation.GetAxisZ());
    LoadWindowS+=Dt;
    if(DynamicsClockS+1e-9<NextDynamicsS)return;
    NextDynamicsS=(FMath::FloorToDouble((DynamicsClockS+1e-9)/In->DynamicsPeriodS)+1)*In->DynamicsPeriodS;
    const FVector Velocity=DynamicsSensor::PointVelocity(FVector(Body->GetV())/100.,Omega,Rotation,In->ImuOffsetM);
    auto& Output=GetProducerOutputData_Internal();
    if(ImuSeeded&&DynamicsClockS>PreviousImuS)
    {
        const FVector Acc=DynamicsSensor::SpecificForce(Rotation,Velocity-PreviousImuVelocity,DynamicsClockS-PreviousImuS,In->SensorGravityMps2);
        const FVector Gyro=Rotation.UnrotateVector(Omega);
        DynamicsSensor::FTruth T;T.TimeS=DynamicsClockS;T.Channel=0;
        for(int A=0;A<3;++A){T.Value[A]=Acc[A];T.Value[A+3]=Gyro[A];}Output.DynamicsSamples.Add(T);
    }
    PreviousImuVelocity=Velocity;PreviousImuS=DynamicsClockS;ImuSeeded=true;
    for(int I=0;I<2;++I)
    {
        DynamicsSensor::FTruth T;T.Channel=I+2;T.TimeS=DynamicsClockS;T.Value[0]=LoadImpulseNs[I]/LoadWindowS;
        Output.DynamicsSamples.Add(T);LoadImpulseNs[I]=0;
    }
    LoadWindowS=0;
}
