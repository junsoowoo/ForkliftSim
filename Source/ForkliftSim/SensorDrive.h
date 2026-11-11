#pragma once
#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
class AStabilityLab;class FArchive;
namespace SensorDrive
{
    struct FPose {double X=0,Y=0,Yaw=0;};
    FPose Compose(FPose A,FPose B);FPose Inverse(FPose P);FPose Advance(FPose P,FVector V,double Dt);
    struct FSegment {FVector2D A,B;};
    struct FMatch {bool Accepted=false;FPose Pose;int Points=0;double Rms=0,Observability=0;};
    FMatch Match(const TArray<FVector2D>& Points,const TArray<FSegment>& Map,FPose Initial);
    struct FInput
    {
        double Now=0,Dt=0;bool Valid=false,Neutral=true,Ack=false,Expired=false;
        double Rate[4]={},Capture[5]={},Gyro=0;uint64 ImuSequence=0;
    };
    struct FSpec
    {
        double FilterS=.05,CalibrateS=.4,HealthyS=.3,LeaseS=.1;
        double WheelKp=350,WheelKi=120,IntegralNm=100,MaxTorqueNm=450;
        double Radius=.15,Wheelbase=.938,Track=.7,Acceleration=.6,MaxSpeed=1,MaxYaw=.6;
        bool Load(FString& Error);
    };
    struct FHistory {double Time=0;FPose Pose;};
    struct FModel
    {
        FSpec Spec;bool DataHealthy=false,Calibrated=false,Faulted=false,Ready=false,AckRejected=false;
        FString State=TEXT("calibrating"),Reason;
        double NowS=0,UnobservedS=0,HealthySince=-1,CalibrationSince=-1,BiasSum=0,GyroBias=0,LastStamp=-1,LastGyro=0,LastImuStamp=-1;
        int BiasCount=0;uint64 LastImuSequence=0;
        FVector Velocity=FVector::ZeroVector;double Wheel[4]={},Torque[4]={},Integral[4]={},Targets[4]={};
        FVector2D Target=FVector2D::ZeroVector;double YawTarget=0,HeadingTarget=0;
        FPose Odom,MapToOdom;TArray<FHistory> History;
        bool Localized=false;double LastMatchS=-1;FMatch Registration;uint64 LastScanSequence[2]={};
        TArray<FSegment> Map;FString MapId;
        void Observe(const FInput& In);
        void Control(double Dt,FVector2D Command,double Yaw,bool Hold,double SpeedCap,double YawCap);
        FPose Predicted(double Now) const;bool At(double Stamp,FPose& P) const;
        FPose MapPose(double Now) const{return Compose(MapToOdom,Predicted(Now));}
        bool Correct(const TArray<FVector2D>& Points,double Stamp,double Now);
    };
}
struct FSensorDriveRig
{
    SensorDrive::FModel Model;bool AckRequested=false,ActuatorExpired=false;
    TUniquePtr<FArchive> Csv;FString CsvPath;double NextCsv=0,NextFlush=0;
    bool SawFault=false,SawRecovery=false;double PeakErrorM=0,FinalErrorM=0;
    double CommandUntilS=0,CommandTorque[4]={};bool CommandPowered=false;
    ~FSensorDriveRig();static TSharedPtr<FSensorDriveRig> Create(FString& Error);
    void Observe(AStabilityLab& Lab,double Dt);void Correct(AStabilityLab& Lab);
    void Drive(AStabilityLab& Lab,double Dt,class FContactLoadSampler* Sampler);void Record(AStabilityLab& Lab);
    TSharedRef<FJsonObject> Telemetry(double Now) const;
};
