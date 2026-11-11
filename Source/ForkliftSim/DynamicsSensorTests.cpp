#include "DynamicsSensors.h"
#include "ContactLoadSampler.h"
#include "HydraulicCallback.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDynamicsImuFrames,"ForkliftSim.DynamicsSensors.SpecificForceFramesAndFreeFall",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDynamicsImuFrames::RunTest(const FString&)
{
    const FVector Gravity(0,0,-9.81);const double Dt=.01;
    TestTrue(TEXT("Stationary level accelerometer reads positive gravity, not zero"),DynamicsSensor::SpecificForce(FQuat::Identity,FVector::ZeroVector,Dt,Gravity).Equals(FVector(0,0,9.81),1e-9));
    TestTrue(TEXT("Free fall has zero specific force"),DynamicsSensor::SpecificForce(FQuat::Identity,Gravity*Dt,Dt,Gravity).IsNearlyZero(1e-9));
    const FQuat Tilt(FVector::RightVector,UE_DOUBLE_PI/2);
    TestTrue(TEXT("Gravity rotates into tilted sensor axes"),DynamicsSensor::SpecificForce(Tilt,FVector::ZeroVector,Dt,Gravity).Equals(FVector(-9.81,0,0),1e-8));
    TestTrue(TEXT("Translation acceleration adds to apparent gravity"),DynamicsSensor::SpecificForce(FQuat::Identity,FVector(.02,0,0),Dt,Gravity).Equals(FVector(2,0,9.81),1e-9));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDynamicsLeverArm,"ForkliftSim.DynamicsSensors.RotatingSensorLeverArm",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDynamicsLeverArm::RunTest(const FString&)
{
    const FVector Offset(1,0,0),Omega(0,0,2);const double Dt=.0001;
    const FVector A=DynamicsSensor::PointVelocity(FVector::ZeroVector,Omega,FQuat::Identity,Offset);
    const FQuat Q(FVector::UpVector,2*Dt);
    const FVector B=DynamicsSensor::PointVelocity(FVector::ZeroVector,Omega,Q,Offset);
    const FVector Acc=DynamicsSensor::SpecificForce(Q,B-A,Dt,FVector::ZeroVector);
    TestTrue(TEXT("Offset sensor sees centripetal acceleration w^2*r while COM is stationary"),Acc.Equals(FVector(-4,0,0),.001));
    TestTrue(TEXT("COM-mounted sensor has no lever velocity"),DynamicsSensor::PointVelocity(FVector(1,2,3),Omega,Q,FVector::ZeroVector).Equals(FVector(1,2,3),1e-9));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDynamicsTransport,"ForkliftSim.DynamicsSensors.TransportSaturationAndRecovery",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDynamicsTransport::RunTest(const FString&)
{
    DynamicsSensor::FSpec Spec;Spec.Dropout=Spec.AccSigma=Spec.GyroSigma=Spec.PressureSigma=Spec.LoadSigma=0;
    DynamicsSensor::FModel M;M.Reset(Spec);M.FaultMask=15;M.FaultStartS=.5;M.FaultEndS=1.5;
    for(int Step=0;Step<=2000;++Step)
    {
        const double Now=Step*.001;
        for(int I=0;I<4;++I){DynamicsSensor::FTruth T;T.Channel=I;T.TimeS=Now;T.Value[0]=I==1?-100000:1e8;M.Capture(T);}
        M.Deliver(Now);
        if(Now<Spec.LatencyS)TestFalse(TEXT("No packet before latency"),M.Channels[0].Have);
        if(Now>.8&&Now<1.5)for(int I=0;I<4;++I)TestFalse(TEXT("Old data is unusable during outage"),M.Usable(I,Now));
        if(Now>1.55)for(int I=0;I<4;++I)TestTrue(TEXT("Only new fresh packets restore validity"),M.Usable(I,Now)&&Now-M.Channels[I].Last.CaptureS<.04);
    }
    TestEqual(TEXT("Acceleration saturates at positive full scale"),M.Channels[0].Last.Value[0],Spec.AccRange);
    TestEqual(TEXT("Gauge pressure has a zero floor"),M.Channels[1].Last.Value[0],0.);
    TestTrue(TEXT("Saturation is explicit"),(M.Channels[0].Last.Clipped&1)!=0&&(M.Channels[1].Last.Clipped&1)!=0);
    TestTrue(TEXT("Loss counted, bounded queue"),M.Channels[0].Dropped>90&&M.QueueOverflows==0&&M.Pending.Num()<20);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDynamicsNoise,"ForkliftSim.DynamicsSensors.NoiseUnitsReplayAndSourceClock",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDynamicsNoise::RunTest(const FString&)
{
    DynamicsSensor::FSpec Spec;Spec.Dropout=0;DynamicsSensor::FModel A,B;A.Reset(Spec);B.Reset(Spec);
    double Sum[4]={};int Count[4]={};
    for(int Step=0;Step<=4000;++Step)
    {
        const double Now=Step*.01;
        for(int I=0;I<4;++I)
        {
            DynamicsSensor::FTruth T;T.Channel=I;T.TimeS=Now;
            for(int Axis=0;Axis<DynamicsSensor::Width(I);++Axis)T.Value[Axis]=I==0?(Axis<3?3:.4):I==1?3e6:2000;
            A.Capture(T);B.Capture(T);
        }
        A.Deliver(Now);B.Deliver(Now);
        for(int I=0;I<4;++I)if(A.Channels[I].Have)
        {
            const auto& P=A.Channels[I].Last;
            for(int Axis=0;Axis<DynamicsSensor::Width(I);++Axis)
            {
                TestEqual(TEXT("Reproducible per-axis virtual measurements"),P.Value[Axis],B.Channels[I].Last.Value[Axis]);
                TestTrue(TEXT("Independent six-sigma plus half-quantum error envelope"),FMath::Abs(P.Value[Axis]-P.Truth[Axis]-Spec.Bias(I,Axis))<=6*Spec.Sigma(I,Axis)+Spec.Quantum(I,Axis)*.5+1e-8);
            }
            Sum[I]+=P.Value[0]-P.Truth[0];++Count[I];
        }
    }
    for(int I=0;I<4;++I){TestTrue(TEXT("Declared bias remains observable in each unit"),FMath::Abs(Sum[I]/Count[I]-Spec.Bias(I,0))<Spec.Sigma(I,0)*.1+Spec.Quantum(I,0)*.5);TestEqual(TEXT("100 Hz acquisition independent of delivery"),A.Channels[I].Sequence,uint64(4001));}
    const auto Sequence=A.Channels[0].Sequence;DynamicsSensor::FTruth Old;Old.TimeS=2;A.Capture(Old);TestEqual(TEXT("Out-of-order samples rejected"),A.Channels[0].Sequence,Sequence);
    A.Reset(Spec);TestFalse(TEXT("Reset clears old measurement"),A.Channels[0].Have);TestTrue(TEXT("Reset clears queue"),A.Pending.IsEmpty());return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDynamicsReset,"ForkliftSim.DynamicsSensors.ConfigurationAndCallbackReset",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDynamicsReset::RunTest(const FString&)
{
    DynamicsSensor::FSpec Spec;TestTrue(TEXT("Default design valid"),Spec.Valid());Spec.AccQuantum=0;TestFalse(TEXT("Zero quantization rejected"),Spec.Valid());Spec=DynamicsSensor::FSpec();Spec.StaleS=.01;TestFalse(TEXT("Timeout below latency rejected"),Spec.Valid());
    FContactLoadInput In;In.bDynamicsSensors=true;In.Reset();TestFalse(TEXT("Reused input does not retain enabled sensor"),In.bDynamicsSensors);
    FContactLoadOutput Out;Out.DynamicsSamples.Add(DynamicsSensor::FTruth());Out.Reset();TestTrue(TEXT("Contact output reset removes old sensor packets"),Out.DynamicsSamples.IsEmpty());
    FHydraulicOutput Hydro;Hydro.SensorSamples.Add(DynamicsSensor::FTruth());Hydro.Reset();TestTrue(TEXT("Hydraulic output reset removes old pressure packets"),Hydro.SensorSamples.IsEmpty());return true;
}
