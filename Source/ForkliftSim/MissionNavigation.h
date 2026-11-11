#pragma once
#include "CoreMinimal.h"
#include "SensorDrive.h"
#include "HandlingControl.h"
#include "Dom/JsonObject.h"
class AStabilityLab;class FArchive;
namespace Mission
{
    enum class ERequest {None,StartLow,StartHigh,Pause,Resume,Cancel};
    struct FSpec
    {
        FBox2D Bounds=FBox2D(FVector2D(-4,-5.5),FVector2D(12,7.5));
        double Cell=.2,Front=2.15,Rear=.8,HalfWidth=.65,Margin=.2,EmptySpeed=.25,LoadedSpeed=.18;
        double FixAge=1.5,ObservationTtl=2,ReplanS=1,LeaseS=.15,NavTimeout=120,Timeout=600;int MaxRetries=1;
        FVector2D Pickup=FVector2D::ZeroVector,Dropoff=FVector2D(3.24,-2.5);
        bool Load(FString& Error);
    };
    struct FObstacle {FVector2D Point;double Stamp=0;};
    struct FPlan {bool Valid=false;FString Reason;TArray<FVector2D> Points;int Expanded=0;double Length=0;};
    struct FPlanner
    {
        FSpec Spec;TArray<SensorDrive::FSegment> Map;TArray<FVector2D> Obstacles;
        bool Free(FVector2D P) const;bool Clear(FVector2D A,FVector2D B) const;
        FPlan Plan(FVector2D Start,FVector2D Goal) const;
    };
    struct FInput
    {
        double Now=0,Dt=0,Height=0,Load[2]={},FixAge=0;SensorDrive::FPose Pose;FVector Velocity=FVector::ZeroVector;
        bool Healthy=false,Localized=false,VehicleFault=false,Manual=false,GuardHold=true,Obstacle=false,Camera=false;
        uint64 HandlingSequence=0;bool HandlingActive=false,HandlingFault=false,HandlingFull=false,HandlingTransfer=false;
        FString HandlingState,HandlingReason,HandlingFailedStage;SensorDrive::FPose HandlingStartPose;
        ERequest Request=ERequest::None;
    };
    struct FModel
    {
        FSpec Spec;FPlanner Planner;FInput Last;FString State=TEXT("idle"),Reason,Checkpoint;
        bool Engaged=false,High=false,Carrying=false,Brake=true,HoldLift=false,AwaitingHandling=false;
        bool SawPickup=false,SawDropoff=false;int Retries=0,Waypoint=0,Replans=0;uint64 HandlingBaseline=0;
        double StartS=0,StageS=0,StableS=0,NextPlanS=0,HeightTarget=0,ReferenceLoad=0,BestDistance=1e9,ProgressS=0,PausedS=0;
        FPlan Route;FVector2D Velocity=FVector2D::ZeroVector;double YawRate=0;Handling::ERequest HandlingRequest=Handling::ERequest::None;
        bool CancelHandling=false;SensorDrive::FPose PreviousPose;bool HavePose=false;
        bool Navigating() const{return Engaged&&(State==TEXT("to_pickup")||State==TEXT("to_dropoff"));}
        bool Holding() const{return Engaged&&(State==TEXT("paused")||State==TEXT("fault_latched")||State==TEXT("blocked")||State==TEXT("cancelled"));}
        void Step(const FInput& I);void Pause(const TCHAR* Why,bool Fault=false);void Phase(const TCHAR* Next,double Now);
        bool Resume(const FInput& I);bool GuardPoint(FVector2D P) const;
    };
}
struct FMissionRig
{
    Mission::FModel Model;Mission::ERequest Request=Mission::ERequest::None;bool SelectedHigh=false;
    double AckUntilS=-1;double CommandUntilS=0,NextMapS=0,NextCsvS=0,NextFlushS=0,PlannerMs=0;
    double CarrierFrontM=.7,CarrierBackM=2.15,CarrierUntilS=-1;
    bool CarriedPoint(FVector2D P,double Now) const;
    TMap<FIntPoint,Mission::FObstacle> Dynamic;uint64 LastScan[2]={};TUniquePtr<FArchive> Csv;FString CsvPath;
    ~FMissionRig();static TSharedPtr<FMissionRig> Create(FString& Error);
    void ObserveMap(AStabilityLab& Lab);void Update(AStabilityLab& Lab,double Dt);void Apply(AStabilityLab& Lab);void Record(AStabilityLab& Lab);
    bool GuardPoint(FVector2D P) const;bool NavField(double Now) const;TSharedRef<FJsonObject> Telemetry() const;
};
