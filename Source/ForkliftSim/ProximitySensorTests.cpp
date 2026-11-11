#include "ProximitySensors.h"
#include "ForkActor.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FProximityValidity,"ForkliftSim.ProximitySensors.RangeBlindZoneAndNoReturn",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FProximityValidity::RunTest(const FString&)
{
    ProximitySensor::FSpec S;S.OpticalDropout=S.PacketDropout=S.SigmaM=S.BiasM=0;S.LatencyS=0;
    ProximitySensor::FModel M;M.Reset(S);ProximitySensor::FTruth T;T.Hit=true;T.TimeS=0;T.RangeM=.01;M.Capture(T);M.Deliver(0);
    TestTrue(TEXT("A too-close packet is received but has no usable distance"),M.TransportValid(0,0)&&!M.MeasurementValid(0,0)&&M.Channels[0].Last.Reading==ProximitySensor::EReading::TooClose);
    T.TimeS=.1;T.Hit=false;M.Capture(T);M.Deliver(.1);TestTrue(TEXT("No return differs from stale transport"),M.TransportValid(0,.1)&&M.Channels[0].Last.Reading==ProximitySensor::EReading::NoReturn);
    T.TimeS=.2;T.Hit=true;T.RangeM=.42;M.Capture(T);M.Deliver(.2);TestTrue(TEXT("Valid metric range"),M.MeasurementValid(0,.2)&&FMath::Abs(M.Channels[0].Last.RangeM-.42)<1e-9);
    T.TimeS=.3;T.RangeM=2;M.Capture(T);M.Deliver(.3);TestTrue(TEXT("Beyond measurement range is not a zero distance"),!M.MeasurementValid(0,.3)&&M.Channels[0].Last.Reading==ProximitySensor::EReading::TooFar);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FProximityPresence,"ForkliftSim.ProximitySensors.PresenceDebounceBothEdges",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FProximityPresence::RunTest(const FString&)
{
    ProximitySensor::FSpec S;S.PacketDropout=S.OpticalDropout=S.LatencyS=0;S.DebounceS=.05;ProximitySensor::FModel M;M.Reset(S);
    auto Feed=[&](double Time,bool Hit){ProximitySensor::FTruth T;T.Channel=4;T.TimeS=Time;T.Hit=Hit;T.RangeM=.02;M.Capture(T);M.Deliver(Time);};
    Feed(0,false);TestFalse(TEXT("Startup switch is unknown before stable observation"),M.MeasurementValid(4,0));Feed(.05,false);TestTrue(TEXT("Clear beam is a valid OFF reading"),M.MeasurementValid(4,.05)&&!M.Channels[4].Last.Detected);
    Feed(.1,true);Feed(.12,false);TestFalse(TEXT("Short occlusion does not assert"),M.Channels[4].Last.Detected);
    Feed(.2,true);Feed(.25,true);TestTrue(TEXT("Sustained object asserts"),M.Channels[4].Last.Detected);
    Feed(.3,false);TestTrue(TEXT("Release also waits for debounce"),M.Channels[4].Last.Detected);Feed(.35,false);TestFalse(TEXT("Sustained clear releases"),M.Channels[4].Last.Detected);
    Feed(1,true);TestFalse(TEXT("Long acquisition gap cannot satisfy debounce"),M.MeasurementValid(4,1));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FProximityTransport,"ForkliftSim.ProximitySensors.TransportRecoveryAndOpticalLoss",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FProximityTransport::RunTest(const FString&)
{
    ProximitySensor::FSpec S;S.PacketDropout=S.OpticalDropout=0;ProximitySensor::FModel M;M.Reset(S);M.FaultMask=255;
    for(int Step=0;Step<=200;++Step)
    {
        const double Now=Step*.05;for(int I=0;I<8;++I){ProximitySensor::FTruth T;T.Channel=I;T.TimeS=Now;T.Hit=true;T.RangeM=.1;M.Capture(T);}M.Deliver(Now);
        if(Now<.03)TestFalse(TEXT("No premature delivery"),M.Channels[0].Have);
        if(Now>6.4&&Now<8)for(int I=0;I<8;++I)TestFalse(TEXT("All transport channels become stale"),M.TransportValid(I,Now));
        if(Now>8.2)for(int I=0;I<8;++I)TestTrue(TEXT("Fresh data restores measurement, not replay"),M.MeasurementValid(I,Now)&&Now-M.Channels[I].Last.Truth.TimeS<.11);
    }
    TestTrue(TEXT("Fault losses counted and queue bounded"),M.Channels[0].Dropped>=40&&M.Pending.Num()<16&&M.QueueOverflows==0);
    S.OpticalDropout=1;M.Reset(S);ProximitySensor::FTruth T;T.TimeS=0;T.Hit=true;T.RangeM=.1;M.Capture(T);M.Deliver(.1);TestTrue(TEXT("Optical dropout has valid transport but invalid measurement"),M.TransportValid(0,.1)&&!M.MeasurementValid(0,.1)&&M.Channels[0].Last.Reading==ProximitySensor::EReading::Dropout);
    M.Reset(S);TestFalse(TEXT("Reset removes previous episode"),M.Channels[0].Have);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FProximityNoise,"ForkliftSim.ProximitySensors.RangeNoiseAndConfiguration",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FProximityNoise::RunTest(const FString&)
{
    ProximitySensor::FSpec S;TestTrue(TEXT("Default spec valid"),S.Valid());S.PacketDropout=S.OpticalDropout=0;ProximitySensor::FModel A,B;A.Reset(S);B.Reset(S);double Sum=0;int N=0;
    for(int I=0;I<4000;++I){ProximitySensor::FTruth T;T.TimeS=I*.05;T.Hit=true;T.RangeM=.432;A.Capture(T);B.Capture(T);A.Deliver(T.TimeS);B.Deliver(T.TimeS);if(A.MeasurementValid(0,T.TimeS)){const auto& P=A.Channels[0].Last;TestEqual(TEXT("Fixed seed replay"),P.RangeM,B.Channels[0].Last.RangeM);TestTrue(TEXT("Independent six-sigma plus half-quantum envelope"),FMath::Abs(P.RangeM-P.Truth.RangeM-S.BiasM)<=6*S.SigmaM+S.QuantumM*.5+1e-9);Sum+=P.RangeM-P.Truth.RangeM;++N;}}
    TestTrue(TEXT("Declared bias visible"),FMath::Abs(Sum/N-S.BiasM)<.0001);S.MinM=2;TestFalse(TEXT("Reversed range rejected"),S.Valid());S=ProximitySensor::FSpec();S.StaleS=.01;TestFalse(TEXT("Impossible stale threshold rejected"),S.Valid());return true;
}
