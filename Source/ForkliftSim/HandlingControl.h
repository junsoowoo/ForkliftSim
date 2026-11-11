#pragma once
#include "CoreMinimal.h"
#include "SensorDrive.h"
#include "Dom/JsonObject.h"
class AStabilityLab;class FArchive;
namespace Handling
{
    enum class ERequest {None,PickGround,PickLow,PickHigh,PlaceLow,PlaceHigh,Cancel};
    struct FSpec
    {
        double Speed=.12,Strafe=.06,Yaw=.08,Depth=1.075,Withdraw=1.50,Carry=.25,Clear=.15;
        double StageTimeout=45,JobTimeout=140,Dwell=.4,Stale=.35,MaxApproach=2.5;
        double RackX=6.95,RackY=-2.5,Low=.35,High=1.95,RackDepth=1.45,RackWidth=1.8,ForkCenter=1.4104257;
        bool Load(FString& Error);
    };
    // Only measured/fused sensor values and surveyed dimensions. No actor or plant state.
    struct FInput
    {
        double Now=0,Dt=0,PerceptionStamp=-1,Height=0,Load[2]={},HeelDepth[2]={},Depth[2]={},Uncertainty=0,Y=0,Z=0,Yaw=0;
        SensorDrive::FPose Pose;FVector Velocity=FVector::ZeroVector;
        bool Measurements=false,Healthy=false,Camera=false,Geometry=false,Vision=false,Full=false,Aligned=false,Presence[4]={},HeelValid=false;
        bool Fault=false,Obstacle=false,Manual=false,Cancel=false;ERequest Request=ERequest::None;
    };
    struct FModel
    {
        FSpec Spec;FString State=TEXT("idle"),Reason,FailedStage;bool Active=false,Faulted=false,Place=false,Receipt=false,FullSeen=false,TransferSeen=false;
        double StartS=0,StageS=0,StableS=0,Tier=0,HeightTarget=0,ReferenceLoad=0,VisionLostS=-1,AlignmentSince=-1;
        SensorDrive::FPose StartPose,WithdrawPose;FVector2D Velocity=FVector2D::ZeroVector;double YawRate=0;
        bool Brake=true,Permit=false;FInput Last;
        bool Working() const{return Active||Faulted;}
        void Step(const FInput& In);void Stop(const TCHAR* Why);void Transition(const TCHAR* Next,double Now);
        bool Stable(bool Value,double Dt);bool Start(const FInput& In,ERequest Request);
        bool TargetPoint(FVector2D BodyPoint) const;
    };
}
struct FHandlingRig
{
    Handling::FModel Model;Handling::ERequest Request=Handling::ERequest::None;bool Cancel=false,GuardReleased=false;uint64 JobSequence=0;
    TUniquePtr<FArchive> Csv;FString CsvPath;double NextCsv=0,NextFlush=0,AckUntilS=-1;
    ~FHandlingRig();static TSharedPtr<FHandlingRig> Create(FString& Error);
    void Update(AStabilityLab& Lab,double Dt);void Apply(AStabilityLab& Lab);TSharedRef<FJsonObject> Telemetry() const;
};
