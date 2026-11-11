#pragma once
#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
class AStabilityLab;class UPrimitiveComponent;class FArchive;
namespace LidarSafety
{
    enum class EReturn:uint8 {Valid,NoReturn,TooClose,Dropout,SelfMasked};
    struct FSpec
    {
        int Rays=270,Seed=1967;double Hz=10,MinM=.05,MaxM=8,SigmaM=.003,QuantumM=.001,PacketDrop=.001,RayDrop=0,LatencyS=.05,StaleS=.35,ClearS=.3;
        double MountX=.8,MountY=.57,MountZ=-.27,BodyX=.78,BodyY=.55,Front=2.05,Rear=.78,HalfWidth=.65,Padding=.65,MaxSpeed=1,MaxYaw=.6;
        bool Load(FString& Error);bool Valid() const;
        FVector Mount(int I) const{return FVector(I?-MountX:MountX,I?MountY:-MountY,MountZ);}
        double Angle(int I,int K) const{return FMath::DegreesToRadians((I?0.:-180.)+270.*K/(Rays-1));}
        bool Masked(int I,int K) const;
    };
    struct FScan {int Id=0;uint64 Sequence=0;double CaptureS=0,DeliveryS=0;TArray<double> Ranges;TArray<EReturn> States;};
    struct FChannel {bool Have=false;uint64 Sequence=0,Dropped=0;FRandomStream Random;FScan Last;};
    struct FSensor
    {
        FSpec Spec;FChannel Channels[2];TArray<FScan> Pending;bool ManualOutage=false;uint32 FaultMask=0;double FaultStartS=6,FaultEndS=8;
        void Reset(const FSpec& S);bool Fault(int I,double Now) const;
        void Capture(int I,double Now,const TArray<double>& TrueRanges,const TArray<EReturn>& TrueStates);
        void Deliver(double Now);bool Fresh(int I,double Now) const;
    };
    struct FDecision
    {
        bool DriveHold=true,LiftHold=false,Faulted=false,Emergency=false,AckRejected=false,EverReady=false,HaveRun=false;
        bool Obstacle=false,DataHealthy=false;int Points=0;double ClearanceM=0,ClearSinceS=-1,HealthySinceS=-1;FString State=TEXT("initializing"),Reason;
        bool CreepWasActive=false;
        void Step(const FSensor& Sensor,double Now,bool CriticalHealthy,bool LiftFault,bool Neutral,bool Estop,bool Acknowledge,bool ManualCreep=false,bool DriveFault=false,const TFunction<bool(FVector2D)>& TargetPermit=nullptr,const TFunction<bool(FVector2D)>& NavigationField=nullptr);
    };
}
struct FLidarSafetyRig
{
    LidarSafety::FSensor Sensor;LidarSafety::FDecision Decision;
    bool AckRequested=false,EstopRequested=false,ShowRays=false;
    double NextScanS=0,NextCsvS=0,NextFlushS=0;FString CsvPath;TUniquePtr<FArchive> Csv;
    bool SawObstacle=false,SawFault=false,SawRecovered=false;
    bool TestAckSent=false,SawFaultRecovery=false,TestRangePass=false;uint32 TestBits=0;double RecoveryHeightM=0;
    ~FLidarSafetyRig();static TSharedPtr<FLidarSafetyRig> Create(FString& Error);
    void Update(AStabilityLab& Lab);TSharedRef<FJsonObject> Telemetry(double Now) const;
};
