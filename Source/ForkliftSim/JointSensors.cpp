#include "JointSensors.h"
#include "DynamicsSensors.h"
#include "ProximitySensors.h"
#include "LidarSafety.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/FileManager.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/Archive.h"
#include "StabilityLab.h"
#include "EngineUtils.h"

namespace
{
    double Gaussian(FRandomStream& R){return FMath::Sqrt(-2*FMath::Loge(FMath::Max(1e-9,double(R.FRand()))))*FMath::Cos(2*UE_DOUBLE_PI*R.FRand());}
    const TCHAR* Id(int I){static const TCHAR* Names[]={TEXT("encoder_fl"),TEXT("encoder_fr"),TEXT("encoder_rl"),TEXT("encoder_rr"),TEXT("fork_height"),TEXT("fork_limits")};return Names[I];}
    void Write(FArchive& A,const FString& S){FTCHARToUTF8 U(*S);A.Serialize((void*)U.Get(),U.Length());}
    void Number(const TSharedRef<FJsonObject>& J,const TCHAR* Key,double Value,bool Valid=true)
    {if(Valid && FMath::IsFinite(Value))J->SetNumberField(Key,Value);else J->SetField(Key,MakeShared<FJsonValueNull>());}
}
bool JointSensor::FSpec::Validate(double TravelM) const
{
    const double Values[]={EncoderHz,HeightHz,LimitHz,EncoderNoiseCounts,HeightSigmaM,HeightBiasM,HeightQuantumM,LatencyS,Dropout,StaleS,LimitTripM,LimitReleaseM,DebounceS,TravelM};
    for(double V:Values)if(!FMath::IsFinite(V))return false;
    return CountsPerTurn>=64 && CountsPerTurn<=1048576 && EncoderHz>=10 && EncoderHz<=200 && HeightHz>=10 && HeightHz<=100 && LimitHz>=10 && LimitHz<=200 && EncoderNoiseCounts>=0 && EncoderNoiseCounts<=1 && HeightSigmaM>=0 && HeightSigmaM<=.01 && FMath::Abs(HeightBiasM)<=.02 && HeightQuantumM>=.00001 && HeightQuantumM<=.01 && LatencyS>=0 && LatencyS<=.2 && Dropout>=0 && Dropout<=1 && StaleS>LatencyS+2/FMath::Min3(EncoderHz,HeightHz,LimitHz) && StaleS<=2 && LimitTripM>=0 && LimitReleaseM>LimitTripM && LimitReleaseM<TravelM*.25 && DebounceS>=0 && DebounceS<=.2;
}
bool JointSensor::FSpec::Load(FString& Error,double TravelM)
{
    FString Text;TSharedPtr<FJsonObject> J;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("JointSensors.json"))) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J){Error=TEXT("Cannot read JointSensors.json");return false;}
    double Counts=0,SeedValue=0;
    auto Read=[&](const TCHAR* Key,double& V){return J->TryGetNumberField(Key,V);};
    if(!Read(TEXT("encoder_counts_per_turn"),Counts)||!Read(TEXT("seed"),SeedValue)||!Read(TEXT("encoder_hz"),EncoderHz)||!Read(TEXT("height_hz"),HeightHz)||!Read(TEXT("limit_hz"),LimitHz)||!Read(TEXT("encoder_phase_noise_counts"),EncoderNoiseCounts)||!Read(TEXT("height_sigma_m"),HeightSigmaM)||!Read(TEXT("height_bias_m"),HeightBiasM)||!Read(TEXT("height_quantum_m"),HeightQuantumM)||!Read(TEXT("latency_s"),LatencyS)||!Read(TEXT("packet_dropout"),Dropout)||!Read(TEXT("stale_s"),StaleS)||!Read(TEXT("limit_trip_m"),LimitTripM)||!Read(TEXT("limit_release_m"),LimitReleaseM)||!Read(TEXT("limit_debounce_s"),DebounceS)||!FMath::IsFinite(Counts)||Counts!=FMath::FloorToDouble(Counts)||Counts<64||Counts>1048576||!FMath::IsFinite(SeedValue)||SeedValue<0||SeedValue>2147483000||SeedValue!=FMath::FloorToDouble(SeedValue))
    {Error=TEXT("Missing/invalid JointSensors.json fields");return false;}
    CountsPerTurn=int(Counts);Seed=int(SeedValue);
    if(!Validate(TravelM)){Error=TEXT("Joint sensor specification outside supported bounds");return false;}return true;
}
void JointSensor::FSwitch::Update(bool Trip,bool Release,double NowS,double DebounceS)
{
    const bool Next=Raw?!Release:Trip;
    if(!Seeded||Next!=Raw){Raw=Next;SinceS=NowS;Seeded=true;}
    if(NowS-SinceS+1e-9>=DebounceS){Stable=Raw;Initialized=true;}
}
void JointSensor::FModel::Reset(const FSpec& NewSpec,double NewTravelM)
{
    *this=FModel();Spec=NewSpec;TravelM=NewTravelM;
    for(int I=0;I<6;++I)Random[I].Initialize(Spec.Seed+I*101);
}
bool JointSensor::FModel::Fault(int Channel,double NowS) const
{return ManualOutage || ((TimedFaultMask&(1u<<Channel)) && NowS>=FaultStartS && NowS<FaultEndS);}
void JointSensor::FModel::Capture(const FTruth& Truth)
{
    if(!FMath::IsFinite(Truth.TimeS)||!FMath::IsFinite(Truth.HeightM))return;
    for(double A:Truth.AngleRad)if(!FMath::IsFinite(A))return;
    for(int I=0;I<6;++I)
    {
        if(Truth.TimeS+1e-9<NextCaptureS[I])continue;
        const double Hz=I<4?Spec.EncoderHz:I==4?Spec.HeightHz:Spec.LimitHz;
        NextCaptureS[I]=(FMath::FloorToDouble((Truth.TimeS+1e-9)*Hz)+1)/Hz;
        FSample S;S.Channel=I;S.Sequence=++Channels[I].Sequence;S.CaptureS=Truth.TimeS;S.DeliveryS=Truth.TimeS+Spec.LatencyS;S.TruthHeight=Truth.HeightM;
        if(I<4)
        {
            S.TruthAngle=Truth.AngleRad[I];S.Counts=int64(FMath::RoundToDouble(S.TruthAngle/(2*UE_DOUBLE_PI)*Spec.CountsPerTurn+Spec.EncoderNoiseCounts*Gaussian(Random[I])));
            S.Value=double(S.Counts)*(2*UE_DOUBLE_PI)/Spec.CountsPerTurn;
            const double Dt=Truth.TimeS-PreviousEncoderS[I];S.RateValid=EncoderSeeded[I] && Dt>0;
            if(S.RateValid)S.Rate=double(S.Counts-PreviousCounts[I])*(2*UE_DOUBLE_PI)/(Spec.CountsPerTurn*Dt);
            PreviousCounts[I]=S.Counts;PreviousEncoderS[I]=Truth.TimeS;EncoderSeeded[I]=true;
        }
        else if(I==4)
        {
            const double Raw=FMath::RoundToDouble((Truth.HeightM+Spec.HeightBiasM+Spec.HeightSigmaM*Gaussian(Random[I]))/Spec.HeightQuantumM)*Spec.HeightQuantumM;
            S.Value=FMath::Clamp(Raw,0.,TravelM);S.Clipped=Raw<0||Raw>TravelM;
        }
        else
        {
            Switches[0].Update(Truth.HeightM<=Spec.LimitTripM,Truth.HeightM>=Spec.LimitReleaseM,Truth.TimeS,Spec.DebounceS);
            Switches[1].Update(Truth.HeightM>=TravelM-Spec.LimitTripM,Truth.HeightM<=TravelM-Spec.LimitReleaseM,Truth.TimeS,Spec.DebounceS);
            S.Initialized=Switches[0].Initialized&&Switches[1].Initialized;S.Lower=Switches[0].Stable;S.Upper=Switches[1].Stable;
        }
        if(Fault(I,Truth.TimeS)||Random[I].FRand()<Spec.Dropout){++Channels[I].Dropped;continue;}
        if(Pending.Num()>=1024){++QueueOverflows;++Channels[I].Dropped;continue;}
        Pending.Add(S);
    }
}
void JointSensor::FModel::Deliver(double NowS)
{
    for(int I=0;I<Pending.Num();)
    {
        FSample S=Pending[I];if(S.DeliveryS>NowS+1e-9){++I;continue;}
        Pending.RemoveAt(I,1,EAllowShrinking::No);
        auto& C=Channels[S.Channel];if(Fault(S.Channel,NowS)){++C.Dropped;continue;}
        S.DeliveryS=NowS;C.Last=S;C.Have=true;
    }
}
bool JointSensor::FModel::Usable(int Channel,double NowS) const
{const auto& C=Channels[Channel];return C.Have && C.Last.Initialized && NowS>=C.Last.CaptureS-1e-9 && NowS-C.Last.CaptureS<=Spec.StaleS;}
const TCHAR* JointSensor::FModel::Status(int Channel,double NowS) const
{const auto& C=Channels[Channel];if(!C.Have)return TEXT("waiting");if(NowS-C.Last.CaptureS>Spec.StaleS)return TEXT("stale");if(!C.Last.Initialized)return TEXT("initializing");return TEXT("valid");}

FJointSensorRig::~FJointSensorRig(){if(Csv)Csv->Close();}
void AStabilityLab::ToggleJointSensorOutage(){if(JointSensors)JointSensors->ToggleOutage();if(DynamicsSensors)DynamicsSensors->Model.ManualOutage=!DynamicsSensors->Model.ManualOutage;if(ProximitySensors)ProximitySensors->Model.ManualOutage=!ProximitySensors->Model.ManualOutage;if(VehicleGuard)VehicleGuard->Sensor.ManualOutage=!VehicleGuard->Sensor.ManualOutage;}
void AStabilityController::ToggleJointSensorOutage(){for(TActorIterator<AStabilityLab> I(GetWorld());I;++I){I->ToggleJointSensorOutage();break;}}
TSharedPtr<FJointSensorRig> FJointSensorRig::Create(double TravelM,double WheelRadiusM,FString& Error)
{
    JointSensor::FSpec Spec;if(!Spec.Load(Error,TravelM))return nullptr;
    auto R=MakeShared<FJointSensorRig>();R->RadiusM=WheelRadiusM;R->Model.Reset(Spec,TravelM);
    FString FaultName;if(FParse::Value(FCommandLine::Get(),TEXT("JointSensorFault="),FaultName))
    {
        for(int I=0;I<6;++I)if(FaultName==Id(I)||FaultName==TEXT("all"))R->Model.TimedFaultMask|=1u<<I;
        if(!R->Model.TimedFaultMask){Error=TEXT("Unknown JointSensorFault channel");return nullptr;}
        R->Model.FaultStartS=6;R->Model.FaultEndS=8;
        FParse::Value(FCommandLine::Get(),TEXT("JointSensorFaultStart="),R->Model.FaultStartS);FParse::Value(FCommandLine::Get(),TEXT("JointSensorFaultEnd="),R->Model.FaultEndS);
        if(!FMath::IsFinite(R->Model.FaultStartS)||!FMath::IsFinite(R->Model.FaultEndS)||R->Model.FaultStartS<0||R->Model.FaultEndS<=R->Model.FaultStartS){Error=TEXT("Invalid JointSensorFault window");return nullptr;}
    }
    R->CsvPath=FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir()/TEXT("Telemetry")/FString::Printf(TEXT("JointSensors-%s-%s.csv"),*FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")),*FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(8)));
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(R->CsvPath),true);R->Csv.Reset(IFileManager::Get().CreateFileWriter(*R->CsvPath));
    if(!R->Csv){Error=TEXT("Cannot create joint sensor CSV");return nullptr;}
    Write(*R->Csv,TEXT("snapshot_sim_s,id,status,valid,sequence,capture_sim_s,delivery_sim_s,age_s,counts,position_rad,angular_velocity_rad_s,height_m,lower_active,upper_active,truth_angle_rad,truth_height_m,clipped,transport_fault,dropped\n"));
    UE_LOG(LogTemp,Display,TEXT("JOINT_SENSOR_CSV %s"),*R->CsvPath);return R;
}
void FJointSensorRig::Update(double NowS)
{
    Model.Deliver(NowS);if(!Csv||NowS+1e-9<NextCsvS)return;NextCsvS=NowS+.05;
    for(int I=0;I<6;++I)
    {
        const auto& C=Model.Channels[I];const auto& S=C.Last;const bool Valid=Model.Usable(I,NowS);
        auto N=[](double V){return FString::Printf(TEXT("%.9f"),V);};
        TArray<FString> Cells={N(NowS),Id(I),Model.Status(I,NowS),Valid?TEXT("1"):TEXT("0"),FString::Printf(TEXT("%llu"),C.Have?S.Sequence:0),C.Have?N(S.CaptureS):TEXT(""),C.Have?N(S.DeliveryS):TEXT(""),C.Have?N(NowS-S.CaptureS):TEXT(""),Valid&&I<4?FString::Printf(TEXT("%lld"),S.Counts):TEXT(""),Valid&&I<4?N(S.Value):TEXT(""),Valid&&I<4&&S.RateValid?N(S.Rate):TEXT(""),Valid&&I==4?N(S.Value):TEXT(""),Valid&&I==5?(S.Lower?TEXT("1"):TEXT("0")):TEXT(""),Valid&&I==5?(S.Upper?TEXT("1"):TEXT("0")):TEXT(""),C.Have&&I<4?N(S.TruthAngle):TEXT(""),C.Have?N(S.TruthHeight):TEXT(""),S.Clipped?TEXT("1"):TEXT("0"),Model.Fault(I,NowS)?TEXT("1"):TEXT("0"),FString::Printf(TEXT("%llu"),C.Dropped)};
        Write(*Csv,FString::Join(Cells,TEXT(","))+TEXT("\n"));
    }
    if(NowS>=NextFlushS){Csv->Flush();NextFlushS=NowS+5;}
}
TSharedRef<FJsonObject> FJointSensorRig::Telemetry(double NowS) const
{
    auto Root=MakeShared<FJsonObject>();Root->SetStringField(TEXT("source"),TEXT("simulated_physical_joint_sensors_v1"));Root->SetStringField(TEXT("clock"),TEXT("physics_simulation_seconds"));Root->SetBoolField(TEXT("controller_connected"),false);Root->SetBoolField(TEXT("safety_rated"),false);Number(Root,TEXT("queue_overflows"),Model.QueueOverflows);
    TArray<TSharedPtr<FJsonValue>> Encoders;
    for(int I=0;I<6;++I)
    {
        const auto& C=Model.Channels[I];const auto& S=C.Last;const bool Valid=Model.Usable(I,NowS);auto J=MakeShared<FJsonObject>();
        J->SetStringField(TEXT("id"),Id(I));J->SetStringField(TEXT("status"),Model.Status(I,NowS));J->SetBoolField(TEXT("valid"),Valid);
        Number(J,TEXT("sequence"),C.Have?S.Sequence:0);Number(J,TEXT("capture_sim_s"),S.CaptureS,C.Have);Number(J,TEXT("delivery_sim_s"),S.DeliveryS,C.Have);Number(J,TEXT("age_s"),NowS-S.CaptureS,C.Have);
        Number(J,TEXT("latency_s"),Model.Spec.LatencyS);Number(J,TEXT("stale_s"),Model.Spec.StaleS);Number(J,TEXT("dropped_packets"),C.Dropped);J->SetBoolField(TEXT("injected_transport_fault_diagnostic"),Model.Fault(I,NowS));
        if(I<4)
        {
            Number(J,TEXT("target_hz"),Model.Spec.EncoderHz);Number(J,TEXT("counts_per_turn"),Model.Spec.CountsPerTurn);Number(J,TEXT("nominal_radius_m"),RadiusM);Number(J,TEXT("counts"),double(S.Counts),Valid);Number(J,TEXT("position_rad"),S.Value,Valid);Number(J,TEXT("angular_velocity_rad_s"),S.Rate,Valid&&S.RateValid);Number(J,TEXT("nominal_surface_mps"),S.Rate*RadiusM,Valid&&S.RateValid);Encoders.Add(MakeShared<FJsonValueObject>(J));
        }
        else if(I==4){Number(J,TEXT("target_hz"),Model.Spec.HeightHz);Number(J,TEXT("height_m"),S.Value,Valid);Number(J,TEXT("quantum_m"),Model.Spec.HeightQuantumM);J->SetBoolField(TEXT("range_clipped"),Valid&&S.Clipped);Root->SetObjectField(TEXT("fork_height"),J);}
        else
        {
            Number(J,TEXT("target_hz"),Model.Spec.LimitHz);Number(J,TEXT("debounce_s"),Model.Spec.DebounceS);
            Number(J,TEXT("lower_trip_height_m"),Model.Spec.LimitTripM);Number(J,TEXT("lower_release_height_m"),Model.Spec.LimitReleaseM);Number(J,TEXT("upper_trip_height_m"),Model.TravelM-Model.Spec.LimitTripM);Number(J,TEXT("upper_release_height_m"),Model.TravelM-Model.Spec.LimitReleaseM);
            if(Valid){J->SetBoolField(TEXT("lower_active"),S.Lower);J->SetBoolField(TEXT("upper_active"),S.Upper);}else{J->SetField(TEXT("lower_active"),MakeShared<FJsonValueNull>());J->SetField(TEXT("upper_active"),MakeShared<FJsonValueNull>());}
            Root->SetObjectField(TEXT("fork_limits"),J);
        }
    }
    Root->SetArrayField(TEXT("encoders"),Encoders);return Root;
}

void AStabilityLab::ToggleProximityRays(){if(ProximitySensors)ProximitySensors->ShowRays=!ProximitySensors->ShowRays;}
void AStabilityController::ToggleProximityRays(){for(TActorIterator<AStabilityLab> I(GetWorld());I;++I){I->ToggleProximityRays();break;}}
