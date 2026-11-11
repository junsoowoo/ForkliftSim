#pragma once
#include "CoreMinimal.h"
#include "Dom/JsonObject.h"

namespace DynamicsSensor
{
    // SI; IMU axes follow Unreal chassis X-forward, Y-right, Z-up (left-handed).
    // Values are specific force xyz then angular rate xyz, or one scalar.
    struct FTruth {int Channel=0;double TimeS=0,Value[6]={};};
    struct FSpec
    {
        double Hz=100,LatencyS=.02,StaleS=.25,Dropout=.001;
        double AccSigma=.02,AccBias=.01,AccQuantum=.001,AccRange=78.48;
        double GyroSigma=.001,GyroBias=.0005,GyroQuantum=.0001,GyroRange=8.72664626;
        double PressureSigma=3000,PressureBias=5000,PressureQuantum=1000,PressureRange=16000000;
        double LoadSigma=2,LoadBias=1,LoadQuantum=1,LoadRange=20000;
        FVector MountFromComM=FVector(0,0,.4);int Seed=9321;
        bool Valid() const;
        bool Load(FString& Error);
        double Sigma(int Channel,int Axis) const;
        double Bias(int Channel,int Axis) const;
        double Quantum(int Channel,int Axis) const;
        double Range(int Channel,int Axis) const;
    };
    inline FVector SpecificForce(FQuat SensorToWorld,FVector DeltaPointVelocityMps,double Dt,FVector GravityMps2)
    {return SensorToWorld.UnrotateVector(DeltaPointVelocityMps/Dt-GravityMps2);}
    inline FVector PointVelocity(FVector ComVelocityMps,FVector WorldOmega,FQuat BodyToWorld,FVector OffsetM)
    {return ComVelocityMps+FVector::CrossProduct(WorldOmega,BodyToWorld.RotateVector(OffsetM));}
    inline const TCHAR* Id(int I){static const TCHAR* Names[]={TEXT("imu"),TEXT("lift_pressure"),TEXT("fork_load_left"),TEXT("fork_load_right")};return Names[I];}
    inline int Width(int I){return I==0?6:1;}
    struct FPacket
    {
        int Channel=0;uint64 Sequence=0;double CaptureS=0,DeliveryS=0,Value[6]={},Truth[6]={};uint8 Clipped=0;
    };
    struct FChannel {FPacket Last;bool Have=false;uint64 Sequence=0,Dropped=0;double NextS=0,LastSourceS=-1;FRandomStream Random;};
    struct FModel
    {
        FSpec Spec;FChannel Channels[4];TArray<FPacket> Pending;
        bool ManualOutage=false;uint32 FaultMask=0;double FaultStartS=6,FaultEndS=8;uint64 QueueOverflows=0;
        void Reset(const FSpec& Settings);
        bool Fault(int I,double NowS) const;
        void Capture(const FTruth& Truth);
        void Deliver(double NowS);
        bool Usable(int I,double NowS) const;
        const TCHAR* Status(int I,double NowS) const;
    };
}
class FArchive;
struct FDynamicsSensorRig
{
    DynamicsSensor::FModel Model;TUniquePtr<FArchive> Csv;FString CsvPath;double NextCsvS=0,NextFlushS=0;
    ~FDynamicsSensorRig();
    static TSharedPtr<FDynamicsSensorRig> Create(FString& Error);
    void Update(double NowS);
    TSharedRef<FJsonObject> Telemetry(double NowS) const;
};
