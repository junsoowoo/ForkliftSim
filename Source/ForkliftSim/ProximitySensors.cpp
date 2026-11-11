#include "ProximitySensors.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/FileManager.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/Archive.h"

namespace
{
    double Gaussian(FRandomStream& R){return FMath::Sqrt(-2*FMath::Loge(FMath::Max(1e-9,double(R.FRand()))))*FMath::Cos(2*UE_DOUBLE_PI*R.FRand());}
    void Write(FArchive& A,const FString& S){FTCHARToUTF8 U(*S);A.Serialize((void*)U.Get(),U.Length());}
    void Number(const TSharedRef<FJsonObject>& J,const TCHAR* K,double V,bool Valid=true){if(Valid&&FMath::IsFinite(V))J->SetNumberField(K,V);else J->SetField(K,MakeShared<FJsonValueNull>());}
    void Vector(const TSharedRef<FJsonObject>& J,const TCHAR* K,FVector V){TArray<TSharedPtr<FJsonValue>> A;for(int I=0;I<3;++I)A.Add(MakeShared<FJsonValueNumber>(V[I]));J->SetArrayField(K,A);}
}
bool ProximitySensor::FSpec::Valid() const
{
    const double Values[]={Hz,MinM,MaxM,PresenceMaxM,SigmaM,BiasM,QuantumM,OpticalDropout,LatencyS,StaleS,PacketDropout,DebounceS,ForwardOffsetM,RangeAboveTopM,PresenceAboveTopM,NearFraction,FarFraction};for(double V:Values)if(!FMath::IsFinite(V))return false;
    return Hz>=5&&Hz<=50&&MinM>=.001&&MinM<.1&&MaxM>MinM&&MaxM<=3&&PresenceMaxM>=.03&&PresenceMaxM<=.3&&SigmaM>=0&&SigmaM<=.01&&FMath::Abs(BiasM)<=.02&&QuantumM>=.0001&&QuantumM<=.01&&OpticalDropout>=0&&OpticalDropout<=1&&PacketDropout>=0&&PacketDropout<=1&&LatencyS>=0&&LatencyS<=.2&&StaleS>LatencyS+2/Hz&&StaleS<=2&&DebounceS>=0&&DebounceS<=.2&&ForwardOffsetM>=0&&ForwardOffsetM<=.05&&RangeAboveTopM>=.005&&RangeAboveTopM<=.1&&PresenceAboveTopM>=0&&PresenceAboveTopM<=.005&&NearFraction>.05&&FarFraction<.95&&FarFraction-NearFraction>.3;
}
bool ProximitySensor::FSpec::Load(FString& Error)
{
    FString Text;TSharedPtr<FJsonObject> J;if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("ProximitySensors.json")))||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J){Error=TEXT("Cannot read ProximitySensors.json");return false;}
    struct FField {const TCHAR* Key;double* Value;};FField Fields[]={
        {TEXT("hz"),&Hz},{TEXT("min_range_m"),&MinM},{TEXT("max_range_m"),&MaxM},{TEXT("presence_max_m"),&PresenceMaxM},
        {TEXT("sigma_m"),&SigmaM},{TEXT("bias_m"),&BiasM},{TEXT("quantum_m"),&QuantumM},{TEXT("optical_dropout"),&OpticalDropout},
        {TEXT("latency_s"),&LatencyS},{TEXT("stale_s"),&StaleS},{TEXT("packet_dropout"),&PacketDropout},{TEXT("presence_debounce_s"),&DebounceS},
        {TEXT("forward_offset_m"),&ForwardOffsetM},{TEXT("range_above_tine_top_m"),&RangeAboveTopM},{TEXT("presence_above_tine_top_m"),&PresenceAboveTopM},{TEXT("near_fraction"),&NearFraction},{TEXT("far_fraction"),&FarFraction}};
    for(const auto& F:Fields)if(!J->TryGetNumberField(F.Key,*F.Value)){Error=TEXT("Missing proximity sensor field");return false;}
    double SeedValue=0;if(!J->TryGetNumberField(TEXT("seed"),SeedValue)||!FMath::IsFinite(SeedValue)||SeedValue<0||SeedValue>2147483000||SeedValue!=FMath::FloorToDouble(SeedValue)||!Valid()){Error=TEXT("Invalid proximity specification");return false;}Seed=int(SeedValue);return true;
}
const TCHAR* ProximitySensor::ReadingName(EReading R)
{switch(R){case EReading::Valid:return TEXT("valid");case EReading::NoReturn:return TEXT("no_return");case EReading::TooClose:return TEXT("too_close");case EReading::TooFar:return TEXT("too_far");case EReading::Dropout:return TEXT("dropout");default:return TEXT("initializing");}}
void ProximitySensor::FModel::Reset(const FSpec& S){*this=FModel();Spec=S;for(int I=0;I<Count;++I)Channels[I].Random.Initialize(S.Seed+173*I);}
bool ProximitySensor::FModel::Fault(int I,double NowS) const{return ManualOutage||((FaultMask&(1u<<I))&&NowS>=FaultStartS&&NowS<FaultEndS);}
void ProximitySensor::FModel::Capture(const FTruth& T)
{
    if(T.Channel<0||T.Channel>=Count||!FMath::IsFinite(T.TimeS)||T.TimeS<0||!FMath::IsFinite(T.RangeM)||T.RangeM<0)return;
    auto& C=Channels[T.Channel];if(T.TimeS<=C.LastSourceS)return;const double PreviousSourceS=C.LastSourceS;C.LastSourceS=T.TimeS;
    FPacket P;P.Truth=T;P.Sequence=++C.Sequence;P.DeliveryS=T.TimeS+Spec.LatencyS;
    if(C.Random.FRand()<Spec.OpticalDropout){P.Reading=EReading::Dropout;C.Switch=JointSensor::FSwitch();}
    else if(T.Channel>=4)
    {
        const bool Present=T.Hit&&T.RangeM<=Spec.PresenceMaxM;
        // A gap in acquisition cannot satisfy a continuous debounce interval.
        if(PreviousSourceS>=0&&T.TimeS-PreviousSourceS>2.5/Spec.Hz)C.Switch=JointSensor::FSwitch();
        C.Switch.Update(Present,!Present,T.TimeS,Spec.DebounceS);P.Detected=C.Switch.Stable;P.Reading=C.Switch.Initialized?EReading::Valid:EReading::Initializing;
    }
    else if(!T.Hit)P.Reading=EReading::NoReturn;
    else if(T.RangeM<Spec.MinM)P.Reading=EReading::TooClose;
    else if(T.RangeM>Spec.MaxM)P.Reading=EReading::TooFar;
    else
    {
        P.RangeM=FMath::RoundToDouble((T.RangeM+Spec.BiasM+Spec.SigmaM*Gaussian(C.Random))/Spec.QuantumM)*Spec.QuantumM;
        P.Reading=P.RangeM<Spec.MinM?EReading::TooClose:P.RangeM>Spec.MaxM?EReading::TooFar:EReading::Valid;
    }
    if(Fault(T.Channel,T.TimeS)||C.Random.FRand()<Spec.PacketDropout){++C.Dropped;return;}
    if(Pending.Num()>=1024){++C.Dropped;++QueueOverflows;return;}Pending.Add(P);
}
void ProximitySensor::FModel::Deliver(double NowS)
{
    for(int I=0;I<Pending.Num();){auto P=Pending[I];if(P.DeliveryS>NowS+1e-9){++I;continue;}Pending.RemoveAt(I,1,EAllowShrinking::No);auto& C=Channels[P.Truth.Channel];if(Fault(P.Truth.Channel,NowS)){++C.Dropped;continue;}P.DeliveryS=NowS;C.Last=P;C.Have=true;}
}
bool ProximitySensor::FModel::TransportValid(int I,double NowS) const{const auto& C=Channels[I];return C.Have&&NowS>=C.Last.Truth.TimeS-1e-9&&NowS-C.Last.Truth.TimeS<=Spec.StaleS;}
bool ProximitySensor::FModel::MeasurementValid(int I,double NowS) const{return TransportValid(I,NowS)&&Channels[I].Last.Reading==EReading::Valid;}
const TCHAR* ProximitySensor::FModel::Status(int I,double NowS) const{return !Channels[I].Have?TEXT("waiting"):TransportValid(I,NowS)?TEXT("valid"):TEXT("stale");}
FProximitySensorRig::~FProximitySensorRig(){if(Csv)Csv->Close();}
TSharedPtr<FProximitySensorRig> FProximitySensorRig::Create(FString& Error)
{
    ProximitySensor::FSpec S;if(!S.Load(Error))return nullptr;auto R=MakeShared<FProximitySensorRig>();R->Model.Reset(S);
    FString Name;if(FParse::Value(FCommandLine::Get(),TEXT("ProximitySensorFault="),Name))
    {
        for(int I=0;I<ProximitySensor::Count;++I)if(Name==TEXT("all")||Name==ProximitySensor::Id(I))R->Model.FaultMask|=1u<<I;
        if(!R->Model.FaultMask){Error=TEXT("Unknown ProximitySensorFault channel");return nullptr;}
    }
    R->CsvPath=FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir()/TEXT("Telemetry")/FString::Printf(TEXT("ProximitySensors-%s-%s.csv"),*FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")),*FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(8)));
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(R->CsvPath),true);R->Csv.Reset(IFileManager::Get().CreateFileWriter(*R->CsvPath));if(!R->Csv){Error=TEXT("Cannot create proximity CSV");return nullptr;}
    Write(*R->Csv,TEXT("snapshot_sim_s,id,status,transport_valid,measurement_valid,reading_status,sequence,capture_sim_s,delivery_sim_s,age_s,range_m,detected,truth_hit,truth_range_m,origin_x_m,origin_y_m,origin_z_m,direction_x,direction_y,direction_z,dropped,transport_fault\n"));
    UE_LOG(LogTemp,Display,TEXT("PROXIMITY_SENSOR_CSV %s"),*R->CsvPath);return R;
}
TSharedRef<FJsonObject> FProximitySensorRig::Telemetry(double NowS) const
{
    auto Root=MakeShared<FJsonObject>();Root->SetStringField(TEXT("source"),TEXT("simulated_fork_proximity_sensors_v1"));Root->SetStringField(TEXT("geometry"),TEXT("simple_physics_bodies_nearest_return_no_actor_semantics"));Root->SetBoolField(TEXT("controller_connected"),false);Root->SetBoolField(TEXT("safety_rated"),false);Number(Root,TEXT("queue_overflows"),Model.QueueOverflows);
    TArray<TSharedPtr<FJsonValue>> Sensors;
    for(int I=0;I<ProximitySensor::Count;++I)
    {
        const auto& C=Model.Channels[I];const auto& P=C.Last;const bool Transport=Model.TransportValid(I,NowS),Valid=Model.MeasurementValid(I,NowS);auto J=MakeShared<FJsonObject>();
        J->SetStringField(TEXT("id"),ProximitySensor::Id(I));J->SetStringField(TEXT("kind"),I<4?TEXT("range"):TEXT("presence"));J->SetStringField(TEXT("status"),Model.Status(I,NowS));J->SetStringField(TEXT("reading_status"),Transport?ProximitySensor::ReadingName(P.Reading):TEXT("unavailable"));J->SetBoolField(TEXT("transport_valid"),Transport);J->SetBoolField(TEXT("measurement_valid"),Valid);
        Number(J,TEXT("sequence"),C.Have?P.Sequence:0);Number(J,TEXT("capture_sim_s"),P.Truth.TimeS,C.Have);Number(J,TEXT("delivery_sim_s"),P.DeliveryS,C.Have);Number(J,TEXT("age_s"),NowS-P.Truth.TimeS,C.Have);Number(J,TEXT("target_hz"),Model.Spec.Hz);Number(J,TEXT("latency_s"),Model.Spec.LatencyS);Number(J,TEXT("stale_s"),Model.Spec.StaleS);Number(J,TEXT("dropped_packets"),C.Dropped);
        J->SetBoolField(TEXT("injected_transport_fault_diagnostic"),Model.Fault(I,NowS));Number(J,TEXT("min_m"),I<4?Model.Spec.MinM:0);Number(J,TEXT("max_m"),I<4?Model.Spec.MaxM:Model.Spec.PresenceMaxM);
        if(I<4)Number(J,TEXT("range_m"),P.RangeM,Valid);else if(Valid)J->SetBoolField(TEXT("detected"),P.Detected);else J->SetField(TEXT("detected"),MakeShared<FJsonValueNull>());
        if(C.Have){Vector(J,TEXT("diagnostic_origin_world_m"),P.Truth.Ray.OriginM);Vector(J,TEXT("diagnostic_beam_world"),P.Truth.Ray.Direction);}Sensors.Add(MakeShared<FJsonValueObject>(J));
    }
    Root->SetArrayField(TEXT("sensors"),Sensors);return Root;
}
// Acquisition and CSV writing are in ProximitySampling.cpp.
