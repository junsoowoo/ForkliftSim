#pragma once
#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
#include "JointSensors.h"

class AStabilityLab;
class AForkTineActor;
class UPrimitiveComponent;
class FArchive;
namespace ProximitySensor
{
    constexpr int Count=8;
    inline const TCHAR* Id(int I){static const TCHAR* Names[]={TEXT("heel_left"),TEXT("heel_right"),TEXT("tip_left"),TEXT("tip_right"),TEXT("presence_left_near"),TEXT("presence_right_near"),TEXT("presence_left_far"),TEXT("presence_right_far")};return Names[I];}
    struct FSpec
    {
        double Hz=20,MinM=.02,MaxM=1.5,PresenceMaxM=.15;
        double SigmaM=.001,BiasM=.001,QuantumM=.001,OpticalDropout=.001;
        double LatencyS=.03,StaleS=.3,PacketDropout=.001,DebounceS=.05;
        double ForwardOffsetM=.01,RangeAboveTopM=.03,PresenceAboveTopM=.001,NearFraction=.2,FarFraction=.8;
        int Seed=7251;
        bool Valid() const;bool Load(FString& Error);
    };
    struct FRay {FVector OriginM=FVector::ZeroVector,Direction=FVector::ForwardVector;};
    // Diagnostic ray result carries no object class, actor ID or insertion depth.
    struct FTruth {int Channel=0;double TimeS=0,RangeM=0;bool Hit=false;FRay Ray;};
    enum class EReading:uint8 {Valid,NoReturn,TooClose,TooFar,Dropout,Initializing};
    const TCHAR* ReadingName(EReading R);
    struct FPacket
    {
        FTruth Truth;uint64 Sequence=0;double DeliveryS=0,RangeM=0;
        EReading Reading=EReading::Initializing;bool Detected=false;
    };
    struct FChannel {FPacket Last;bool Have=false;uint64 Sequence=0,Dropped=0;FRandomStream Random;JointSensor::FSwitch Switch;double LastSourceS=-1;};
    struct FModel
    {
        FSpec Spec;FChannel Channels[Count];TArray<FPacket> Pending;
        bool ManualOutage=false;uint32 FaultMask=0;double FaultStartS=6,FaultEndS=8;uint64 QueueOverflows=0;
        void Reset(const FSpec& S);bool Fault(int I,double NowS) const;
        void Capture(const FTruth& T);void Deliver(double NowS);
        bool TransportValid(int I,double NowS) const;bool MeasurementValid(int I,double NowS) const;
        const TCHAR* Status(int I,double NowS) const;
    };
    // Read-locked Chaos simulation shapes include welded children and PhysicsOnly
    // solids without enabling any collision/query profile. Query-only copies excluded.
    TArray<TWeakObjectPtr<UPrimitiveComponent>> Targets(AStabilityLab& Lab);
    FTruth Trace(const TArray<TWeakObjectPtr<UPrimitiveComponent>>& Bodies,FRay Ray,double MaxM);
    FRay Mount(const AForkTineActor& Tine,int Kind,const FSpec& Spec);
}
struct FProximitySensorRig
{
    ProximitySensor::FModel Model;TArray<TWeakObjectPtr<UPrimitiveComponent>> Bodies;
    TUniquePtr<FArchive> Csv;FString CsvPath;double NextCaptureS=0,NextCsvS=0,NextFlushS=0;
    bool ShowRays=false,Validated=false;
    ~FProximitySensorRig();
    static TSharedPtr<FProximitySensorRig> Create(FString& Error);
    void Update(AStabilityLab& Lab);
    void ValidateScene(AStabilityLab& Lab);
    TSharedRef<FJsonObject> Telemetry(double NowS) const;
};
