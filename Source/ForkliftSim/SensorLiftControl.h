#pragma once
#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
#include "HydraulicLift.h"

// This controller's public input has no actor, mass, physical position/velocity,
// hydraulic plant state or sensor ground-truth fields.
namespace SensorLift
{
    struct FSpec
    {
        double Kp=4,Ki=.4,Kd=1.5,IntegralCap=.2,UpCap=.65,DownCap=.22,SlewPerS=3;
        double DeadbandM=.006,ReopenM=.012,VelocityTauS=.15,MaxMeasuredSpeed=.6;
        double MaxAgeS=.25,HealthyS=.2,LeaseS=.10,UpperMarginM=.04;
        double PressureTripPa=6900000,PressureReleasePa=6500000,PressureDebounceS=.1;
        double SettleDwellS=.2;
        double MaxLowerMps=.06,LowerApproachPerS=.8,ReturnGainMpsSqrtPa=.00025;
        bool Load(FString& Error);bool Valid(double TravelM) const;
    };
    struct FInput
    {
        double NowS=0,Dt=0,RequestedM=0,HeightM=0,HeightCaptureS=0,LimitCaptureS=0,PressureCaptureS=0,PressurePa=0;
        uint64 HeightSequence=0;bool HeightValid=false,LimitsValid=false,PressureValid=false,PressureClipped=false;
        bool Lower=false,Upper=false,Acknowledge=false,ActuatorExpired=false;
    };
    struct FModel
    {
        FSpec Spec;double TravelM=2.1,TargetM=0,Command=0,Integral=0,Velocity=0,HealthySinceS=-1,PressureSinceS=-1;
        double PreviousHeightM=0,PreviousHeightCaptureS=0,SettledSinceS=-1;uint64 PreviousHeightSequence=0;
        bool Armed=false,Faulted=false,PressureLimited=false,Holding=true,VelocitySeeded=false,Rebased=false,AckRejected=false,PumpEnabled=false;
        FString State=TEXT("initializing"),Reason;
        void Reset(const FSpec& S,double Travel);
        void Step(const FInput& In);
        FHydraulicCommand Output() const {FHydraulicCommand C;C.Direct=true;C.Spool=Command;C.PumpEnabled=PumpEnabled;return C;}
        void Stop(const TCHAR* FaultReason);
    };
}
class FArchive;
struct FSensorLiftRig
{
    SensorLift::FModel Model;SensorLift::FInput LastInput;
    bool AckRequested=false,TestAckSent=false;double TestRebasedTarget=0,NextCsvS=0,NextFlushS=0;
    bool SawFault=false,SawPressure=false,SawExpired=false,SawRecovery=false;
    FString CsvPath;TUniquePtr<FArchive> Csv;
    ~FSensorLiftRig();static TSharedPtr<FSensorLiftRig> Create(double TravelM,double NominalReturnGain,FString& Error);
    void Record(double NowS,double ActualHeightM,double ActualValve,bool ActuatorExpired);
    TSharedRef<FJsonObject> Telemetry() const;
};
