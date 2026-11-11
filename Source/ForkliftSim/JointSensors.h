#pragma once
#include "CoreMinimal.h"
#include "Dom/JsonObject.h"

namespace JointSensor
{
    struct FTruth {double TimeS=0,AngleRad[4]={},HeightM=0;};
    struct FSpec
    {
        int CountsPerTurn=4096,Seed=4217;
        double EncoderHz=100,HeightHz=50,LimitHz=100,EncoderNoiseCounts=.1;
        double HeightSigmaM=.0005,HeightBiasM=.0005,HeightQuantumM=.001;
        double LatencyS=.02,Dropout=.001,StaleS=.25,LimitTripM=.010,LimitReleaseM=.015,DebounceS=.02;
        double MaxHz() const {return FMath::Max3(EncoderHz,HeightHz,LimitHz);}
        bool Validate(double TravelM) const;
        bool Load(FString& Error,double TravelM);
    };
    // Exact relative attitude change removes chassis yaw/roll before shaft
    // unwrapping. The physics inner step is small enough to stay below pi.
    inline double ShaftStep(FQuat HubBefore,FQuat HubAfter,FQuat MountBefore,FQuat MountAfter)
    {
        FQuat Delta=(MountAfter.Inverse()*HubAfter)*(MountBefore.Inverse()*HubBefore).Inverse();
        Delta.Normalize();if(Delta.W<0)Delta=Delta*(-1.);
        return Delta.ToRotationVector().Y;
    }
    struct FSwitch
    {
        bool Raw=false,Stable=false,Seeded=false,Initialized=false;
        double SinceS=0;
        void Update(bool Trip,bool Release,double NowS,double DebounceS);
    };
    struct FSample
    {
        int Channel=0;uint64 Sequence=0;int64 Counts=0;
        double CaptureS=0,DeliveryS=0,Value=0,Rate=0,TruthAngle=0,TruthHeight=0;
        bool RateValid=false,Initialized=true,Clipped=false,Lower=false,Upper=false;
    };
    struct FChannel {FSample Last;bool Have=false;uint64 Sequence=0,Dropped=0;};
    struct FModel
    {
        FSpec Spec;double TravelM=2.1;
        FChannel Channels[6];FSwitch Switches[2];
        TArray<FSample> Pending;
        FRandomStream Random[6];double NextCaptureS[6]={},PreviousEncoderS[4]={};
        int64 PreviousCounts[4]={};bool EncoderSeeded[4]={};
        uint64 QueueOverflows=0;bool ManualOutage=false;
        uint32 TimedFaultMask=0;double FaultStartS=0,FaultEndS=0;
        void Reset(const FSpec& NewSpec,double NewTravelM);
        bool Fault(int Channel,double NowS) const;
        void Capture(const FTruth& Truth);
        void Deliver(double NowS);
        bool Usable(int Channel,double NowS) const;
        const TCHAR* Status(int Channel,double NowS) const;
    };
}

class FArchive;
// Measures physical joints only. No navigation, motor command or safety
// interlock consumes these outputs yet; truth stays in the diagnostic CSV.
struct FJointSensorRig
{
    JointSensor::FModel Model;
    double RadiusM=.15,NextCsvS=0,NextFlushS=0;
    FString CsvPath;TUniquePtr<FArchive> Csv;
    ~FJointSensorRig();
    static TSharedPtr<FJointSensorRig> Create(double TravelM,double RadiusM,FString& Error);
    void Update(double NowS);
    void ToggleOutage(){Model.ManualOutage=!Model.ManualOutage;}
    TSharedRef<FJsonObject> Telemetry(double NowS) const;
};
