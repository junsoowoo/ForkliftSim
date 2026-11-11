#pragma once
#include "CoreMinimal.h"
#include "CameraSensors.h"
#include "SensorDrive.h"
#include "Dom/JsonObject.h"
class AStabilityLab;class FArchive;
namespace PalletPerception
{
    struct FSpec
    {
        double Spacing=.45,OpeningWidth=.354,OpeningHeight=.093,SupportHalfWidth=.45,Width=1.011;
        double FrameStaleS=.5,TrackMaxS=4,TrackMaxDistanceM=.7,MinConfidence=.65,FullDepthM=1.02,DwellS=.15;
        double ForkLength=1.2,ForkWidth=.12,ForkThickness=.05,ForkSpacing=.45;
        FVector CameraOffsetM=FVector(.05,-.1,.1);double CameraPitchDeg=-10;
        bool Load(FString& Error);
    };
    struct FDetection
    {
        bool Valid=false;double CaptureS=0,DeliveryS=0,Confidence=0,DistanceM=0,CenterY=0,CenterZ=0,Yaw=0,Rms=0;
        double HoleWidth[2]={},HoleY[2]={};int FaceEvidence=0,GapEvidence=0,CensoredEvidence=0;uint64 Sequence=0;
        int PlaneCandidates=0,SupportPass=0,RoofPass=0,GapPass=0,EdgePass=0;
    };
    inline bool ConsistentAnchor(FVector A,FVector B,double YawA,double YawB){return FVector2D(A-B).Size()<.06&&FMath::Abs(A.Z-B.Z)<.012&&FMath::Abs(FMath::UnwindRadians(YawA-YawB))<.02;}
    FDetection Detect(const DepthCamera::FFrame& Frame,const FSpec& Spec);
    struct FSignals
    {
        double Now=0;bool Fresh=false,VisionFresh=false,Tracked=false,Alignment=false,Presence[4]={},Loaded=false,UnilateralLoad=false;
        double Depth[2]={},UncertaintyM=0;
    };
    struct FDecision
    {
        FString State=TEXT("unknown"),Reason=TEXT("waiting_for_measurements"),Candidate;double Since=-1;
        bool ConfirmedFull=false,LastFullBeforeSupport=false;double LastFullS=-1;
        void Step(const FSignals& In,const FSpec& Spec);
    };
}
struct FPalletPerceptionRig
{
    PalletPerception::FSpec Spec;PalletPerception::FDetection Detection;PalletPerception::FDecision Decision;
    DepthCamera::FFrame LastFrame;uint64 LastSequence=0;bool Tracked=false,Aligned=false,SignalsFresh=false,CameraEnabled=false,RangeFused=false;
    double NextDetectionS=0;
    double Depth[2]={},OffsetY=0,OffsetZ=0,UncertaintyM=0,NextCsvS=0,NextFlushS=0;
    bool Locked=false;FVector LockedPoint=FVector::ZeroVector;double LockedYaw=0;
    bool HaveAnchor=false;SensorDrive::FPose AnchorBody;FVector AnchorPlanePoint=FVector::ZeroVector;double AnchorYaw=0,AnchorHeight=0;
    struct FHeight {double Time=0,Value=0;};TArray<FHeight> Heights;
    bool HeightAt(double Time,double& H) const;
    TUniquePtr<FArchive> Csv;FString CsvPath;
    ~FPalletPerceptionRig();static TSharedPtr<FPalletPerceptionRig> Create(FString& Error);
    void Update(AStabilityLab& Lab);TSharedRef<FJsonObject> Telemetry(double Now) const;
};
