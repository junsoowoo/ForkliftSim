#include "JointSensors.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJointShaftGeometry,"ForkliftSim.JointSensors.RelativeShaftGeometry",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FJointShaftGeometry::RunTest(const FString&)
{
    double Sum=0;FQuat Previous=FQuat::Identity,Mount=FQuat::Identity;
    for(int I=1;I<=500;++I)
    {
        const FQuat NextMount=FRotator(I*.3,I*.7,I*.1).Quaternion();
        const FQuat Hub=NextMount*FQuat(FVector::RightVector,I*.1);
        Sum+=JointSensor::ShaftStep(Previous,Hub,Mount,NextMount);Previous=Hub;Mount=NextMount;
    }
    TestTrue(TEXT("Many complete shaft turns survive chassis yaw/pitch/roll and quaternion wrapping"),FMath::Abs(Sum-50)<1e-5);
    const FQuat A=FRotator(20,30,40).Quaternion(),B=FRotator(-40,80,110).Quaternion(),Fixed(FVector::RightVector,.8);
    TestTrue(TEXT("Chassis-only motion does not create encoder pulses"),FMath::Abs(JointSensor::ShaftStep(A*Fixed,B*Fixed,A,B))<1e-6);
    TestTrue(TEXT("Reverse shaft rotation retains its sign"),JointSensor::ShaftStep(FQuat::Identity,FQuat(FVector::RightVector,-.2),FQuat::Identity,FQuat::Identity)<-.199999);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJointEncoderCounts,"ForkliftSim.JointSensors.CountsAndSampleClock",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FJointEncoderCounts::RunTest(const FString&)
{
    JointSensor::FSpec S;S.EncoderNoiseCounts=S.Dropout=0;JointSensor::FModel M;M.Reset(S,2.1);
    for(int I=0;I<=1000;++I)
    {
        JointSensor::FTruth T;T.TimeS=I*.001;T.HeightM=.5;
        for(int J=0;J<4;++J)T.AngleRad[J]=(J%2?-1:1)*2*UE_DOUBLE_PI*T.TimeS;
        M.Capture(T);M.Deliver(T.TimeS);
    }
    const auto& P=M.Channels[0].Last;const auto& N=M.Channels[1].Last;
    TestTrue(TEXT("Decoded count equals independent shaft-turn formula"),P.Counts==int64(FMath::RoundToDouble(P.CaptureS*S.CountsPerTurn)));
    TestEqual(TEXT("Opposite physical wheel rotations produce opposite counts"),N.Counts,-P.Counts);
    TestTrue(TEXT("Rate uses source capture intervals, not receiver/render intervals"),FMath::Abs(P.Rate-2*UE_DOUBLE_PI)<.16);
    TestEqual(TEXT("100 Hz acquisition includes time zero"),M.Channels[0].Sequence,uint64(101));
    TestEqual(TEXT("50 Hz height acquisition has its own clock"),M.Channels[4].Sequence,uint64(51));
    TestTrue(TEXT("Sensor delivery cannot precede configured latency"),P.DeliveryS-P.CaptureS+1e-9>=S.LatencyS);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJointSensorOutage,"ForkliftSim.JointSensors.DelayOutageRecovery",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FJointSensorOutage::RunTest(const FString&)
{
    JointSensor::FSpec S;S.Dropout=0;JointSensor::FModel M;M.Reset(S,2.1);M.TimedFaultMask=63;M.FaultStartS=.5;M.FaultEndS=1.5;
    bool Stale=false,Recovered=false;
    for(int I=0;I<=2000;++I)
    {
        JointSensor::FTruth T;T.TimeS=I*.001;T.HeightM=.4;for(double& A:T.AngleRad)A=T.TimeS*4;
        M.Capture(T);M.Deliver(T.TimeS);
        if(T.TimeS>.8&&T.TimeS<1.5){Stale=true;TestFalse(TEXT("A transport outage invalidates old readings after timeout"),M.Usable(0,T.TimeS));}
        if(T.TimeS>1.55){Recovered=true;TestTrue(TEXT("New delayed readings recover without replaying outage backlog"),M.Usable(0,T.TimeS)&&T.TimeS-M.Channels[0].Last.CaptureS<.04);}
    }
    TestTrue(TEXT("Both outage and recovery phases were exercised"),Stale&&Recovered);
    TestTrue(TEXT("Transport loss is counted"),M.Channels[0].Dropped>90);
    TestTrue(TEXT("No unbounded packet queue"),M.Pending.Num()<20&&M.QueueOverflows==0);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJointLimits,"ForkliftSim.JointSensors.LimitHysteresisAndDebounce",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FJointLimits::RunTest(const FString&)
{
    JointSensor::FSwitch S;S.Update(true,false,0,.02);TestFalse(TEXT("Startup contact is unknown until debounce"),S.Initialized);
    S.Update(true,false,.01,.02);TestFalse(TEXT("A short contact does not assert"),S.Initialized);
    S.Update(true,false,.02,.02);TestTrue(TEXT("Sustained contact asserts"),S.Initialized&&S.Stable);
    S.Update(false,false,.03,.02);TestTrue(TEXT("Hysteresis band retains asserted contact"),S.Stable);
    S.Update(false,true,.04,.02);TestTrue(TEXT("Release is debounced too"),S.Stable);
    S.Update(false,true,.061,.02);TestFalse(TEXT("Stable release opens the contact"),S.Stable);
    JointSensor::FSpec Spec;TestTrue(TEXT("Default sensor design is valid"),Spec.Validate(2.1));Spec.StaleS=.01;TestFalse(TEXT("Timeout shorter than normal delivery must be rejected"),Spec.Validate(2.1));
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FJointNoise,"ForkliftSim.JointSensors.SeededHeightNoiseAndReset",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FJointNoise::RunTest(const FString&)
{
    JointSensor::FSpec S;S.Dropout=0;JointSensor::FModel A,B;A.Reset(S,2.1);B.Reset(S,2.1);double Sum=0;int Count=0;
    for(int I=0;I<=4000;++I)
    {
        JointSensor::FTruth T;T.TimeS=I*.02;T.HeightM=1.234;A.Capture(T);B.Capture(T);A.Deliver(T.TimeS);B.Deliver(T.TimeS);
        if(A.Channels[4].Have){const double V=A.Channels[4].Last.Value;TestEqual(TEXT("Fixed-seed sensor replay"),V,B.Channels[4].Last.Value);TestTrue(TEXT("Independent 6-sigma plus quantization error envelope"),FMath::Abs(V-T.HeightM-S.HeightBiasM)<6*S.HeightSigmaM+S.HeightQuantumM*.5);Sum+=V-T.HeightM;++Count;}
    }
    TestTrue(TEXT("Declared height bias remains observable"),FMath::Abs(Sum/Count-S.HeightBiasM)<.0001);
    A.Reset(S,2.1);TestFalse(TEXT("Reset cannot reuse a previous run's valid sample"),A.Channels[4].Have);TestEqual(TEXT("Reset clears pending packets"),A.Pending.Num(),0);
    return true;
}
