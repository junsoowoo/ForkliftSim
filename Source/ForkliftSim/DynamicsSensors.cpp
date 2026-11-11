#include "DynamicsSensors.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/Archive.h"
#include "HAL/FileManager.h"

namespace
{
    double Gaussian(FRandomStream& R){return FMath::Sqrt(-2*FMath::Loge(FMath::Max(1e-9,double(R.FRand()))))*FMath::Cos(2*UE_DOUBLE_PI*R.FRand());}
    void Write(FArchive& A,const FString& S){FTCHARToUTF8 U(*S);A.Serialize((void*)U.Get(),U.Length());}
    void Number(const TSharedRef<FJsonObject>& J,const TCHAR* Key,double V,bool Valid=true)
    {if(Valid&&FMath::IsFinite(V))J->SetNumberField(Key,V);else J->SetField(Key,MakeShared<FJsonValueNull>());}
}
bool DynamicsSensor::FSpec::Valid() const
{
    const double Values[]={Hz,LatencyS,StaleS,Dropout,AccSigma,AccBias,AccQuantum,AccRange,GyroSigma,GyroBias,GyroQuantum,GyroRange,PressureSigma,PressureBias,PressureQuantum,PressureRange,LoadSigma,LoadBias,LoadQuantum,LoadRange};
    for(double V:Values)if(!FMath::IsFinite(V))return false;
    if(MountFromComM.ContainsNaN()||MountFromComM.Size()>2||Hz<10||Hz>200||LatencyS<0||LatencyS>.2||StaleS<=LatencyS+2/Hz||StaleS>2||Dropout<0||Dropout>1)return false;
    for(int I=0;I<4;++I)for(int A=0;A<Width(I);++A)
        if(Sigma(I,A)<0||Sigma(I,A)>Range(I,A)*.1||Quantum(I,A)<=0||Quantum(I,A)>Range(I,A)*.1||Range(I,A)<=0||FMath::Abs(Bias(I,A))>Range(I,A)*.1)return false;
    return true;
}
double DynamicsSensor::FSpec::Sigma(int I,int A) const{return I==0?(A<3?AccSigma:GyroSigma):I==1?PressureSigma:LoadSigma;}
double DynamicsSensor::FSpec::Bias(int I,int A) const{return I==0?(A<3?AccBias:GyroBias):I==1?PressureBias:LoadBias;}
double DynamicsSensor::FSpec::Quantum(int I,int A) const{return I==0?(A<3?AccQuantum:GyroQuantum):I==1?PressureQuantum:LoadQuantum;}
double DynamicsSensor::FSpec::Range(int I,int A) const{return I==0?(A<3?AccRange:GyroRange):I==1?PressureRange:LoadRange;}
bool DynamicsSensor::FSpec::Load(FString& Error)
{
    FString Text;TSharedPtr<FJsonObject> J;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("DynamicsSensors.json")))||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J){Error=TEXT("Cannot read DynamicsSensors.json");return false;}
    struct FField {const TCHAR* Key;double* Value;};
    FField Fields[]={ {TEXT("hz"),&Hz},{TEXT("latency_s"),&LatencyS},{TEXT("stale_s"),&StaleS},{TEXT("packet_dropout"),&Dropout},
        {TEXT("acc_sigma_mps2"),&AccSigma},{TEXT("acc_bias_mps2"),&AccBias},{TEXT("acc_quantum_mps2"),&AccQuantum},{TEXT("acc_range_mps2"),&AccRange},
        {TEXT("gyro_sigma_rad_s"),&GyroSigma},{TEXT("gyro_bias_rad_s"),&GyroBias},{TEXT("gyro_quantum_rad_s"),&GyroQuantum},{TEXT("gyro_range_rad_s"),&GyroRange},
        {TEXT("pressure_sigma_pa"),&PressureSigma},{TEXT("pressure_bias_pa"),&PressureBias},{TEXT("pressure_quantum_pa"),&PressureQuantum},{TEXT("pressure_range_pa"),&PressureRange},
        {TEXT("load_sigma_n"),&LoadSigma},{TEXT("load_bias_n"),&LoadBias},{TEXT("load_quantum_n"),&LoadQuantum},{TEXT("load_range_n"),&LoadRange},
        {TEXT("mount_com_x_m"),&MountFromComM.X},{TEXT("mount_com_y_m"),&MountFromComM.Y},{TEXT("mount_com_z_m"),&MountFromComM.Z}};
    for(const auto& F:Fields)if(!J->TryGetNumberField(F.Key,*F.Value)){Error=TEXT("Missing dynamics sensor field");return false;}
    double SeedValue=0;if(!J->TryGetNumberField(TEXT("seed"),SeedValue)||!FMath::IsFinite(SeedValue)||SeedValue<0||SeedValue>2147483000||SeedValue!=FMath::FloorToDouble(SeedValue)||!Valid()){Error=TEXT("Invalid dynamics sensor configuration");return false;}
    Seed=int(SeedValue);return true;
}
void DynamicsSensor::FModel::Reset(const FSpec& Settings)
{*this=FModel();Spec=Settings;for(int I=0;I<4;++I)Channels[I].Random.Initialize(Spec.Seed+I*137);}
bool DynamicsSensor::FModel::Fault(int I,double NowS) const
{return ManualOutage||((FaultMask&(1u<<I))&&NowS>=FaultStartS&&NowS<FaultEndS);}
void DynamicsSensor::FModel::Capture(const FTruth& T)
{
    if(T.Channel<0||T.Channel>=4||!FMath::IsFinite(T.TimeS)||T.TimeS<0)return;
    auto& C=Channels[T.Channel];if(T.TimeS<=C.LastSourceS||T.TimeS+1e-9<C.NextS)return;
    for(int A=0;A<Width(T.Channel);++A)if(!FMath::IsFinite(T.Value[A]))return;
    C.LastSourceS=T.TimeS;C.NextS=(FMath::FloorToDouble((T.TimeS+1e-9)*Spec.Hz)+1)/Spec.Hz;
    FPacket P;P.Channel=T.Channel;P.Sequence=++C.Sequence;P.CaptureS=T.TimeS;P.DeliveryS=T.TimeS+Spec.LatencyS;
    for(int A=0;A<Width(T.Channel);++A)
    {
        P.Truth[A]=T.Value[A];const double Q=Spec.Quantum(T.Channel,A),Limit=Spec.Range(T.Channel,A),Low=T.Channel==1?0:-Limit;
        const double Raw=FMath::RoundToDouble((T.Value[A]+Spec.Bias(T.Channel,A)+Spec.Sigma(T.Channel,A)*Gaussian(C.Random))/Q)*Q;
        P.Value[A]=FMath::Clamp(Raw,Low,Limit);if(Raw<Low||Raw>Limit)P.Clipped|=1u<<A;
    }
    if(Fault(T.Channel,T.TimeS)||C.Random.FRand()<Spec.Dropout){++C.Dropped;return;}
    if(Pending.Num()>=1024){++C.Dropped;++QueueOverflows;return;}Pending.Add(P);
}
void DynamicsSensor::FModel::Deliver(double NowS)
{
    for(int I=0;I<Pending.Num();)
    {
        auto P=Pending[I];if(P.DeliveryS>NowS+1e-9){++I;continue;}Pending.RemoveAt(I,1,EAllowShrinking::No);
        auto& C=Channels[P.Channel];if(Fault(P.Channel,NowS)){++C.Dropped;continue;}
        P.DeliveryS=NowS;C.Last=P;C.Have=true;
    }
}
bool DynamicsSensor::FModel::Usable(int I,double NowS) const
{const auto& C=Channels[I];return C.Have&&NowS>=C.Last.CaptureS-1e-9&&NowS-C.Last.CaptureS<=Spec.StaleS;}
const TCHAR* DynamicsSensor::FModel::Status(int I,double NowS) const
{return !Channels[I].Have?TEXT("waiting"):Usable(I,NowS)?TEXT("valid"):TEXT("stale");}

FDynamicsSensorRig::~FDynamicsSensorRig(){if(Csv)Csv->Close();}
TSharedPtr<FDynamicsSensorRig> FDynamicsSensorRig::Create(FString& Error)
{
    DynamicsSensor::FSpec Spec;if(!Spec.Load(Error))return nullptr;
    auto R=MakeShared<FDynamicsSensorRig>();R->Model.Reset(Spec);
    FString Name;if(FParse::Value(FCommandLine::Get(),TEXT("DynamicsSensorFault="),Name))
    {
        for(int I=0;I<4;++I)if(Name==DynamicsSensor::Id(I)||Name==TEXT("all"))R->Model.FaultMask|=1u<<I;
        if(!R->Model.FaultMask){Error=TEXT("Unknown DynamicsSensorFault channel");return nullptr;}
    }
    R->CsvPath=FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir()/TEXT("Telemetry")/FString::Printf(TEXT("DynamicsSensors-%s-%s.csv"),*FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")),*FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(8)));
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(R->CsvPath),true);R->Csv.Reset(IFileManager::Get().CreateFileWriter(*R->CsvPath));
    if(!R->Csv){Error=TEXT("Cannot open dynamics sensor CSV");return nullptr;}
    Write(*R->Csv,TEXT("snapshot_sim_s,id,status,valid,sequence,capture_sim_s,delivery_sim_s,age_s,value_0,value_1,value_2,value_3,value_4,value_5,truth_0,truth_1,truth_2,truth_3,truth_4,truth_5,clipped_mask,dropped,transport_fault\n"));
    UE_LOG(LogTemp,Display,TEXT("DYNAMICS_SENSOR_CSV %s"),*R->CsvPath);return R;
}
void FDynamicsSensorRig::Update(double NowS)
{
    Model.Deliver(NowS);if(!Csv||NowS+1e-9<NextCsvS)return;NextCsvS=NowS+.05;
    for(int I=0;I<4;++I)
    {
        const auto& C=Model.Channels[I];const auto& P=C.Last;const bool Valid=Model.Usable(I,NowS);
        auto N=[](double V){return FString::Printf(TEXT("%.9f"),V);};
        TArray<FString> Cells={N(NowS),DynamicsSensor::Id(I),Model.Status(I,NowS),Valid?TEXT("1"):TEXT("0"),FString::Printf(TEXT("%llu"),C.Have?P.Sequence:0),C.Have?N(P.CaptureS):TEXT(""),C.Have?N(P.DeliveryS):TEXT(""),C.Have?N(NowS-P.CaptureS):TEXT("")};
        for(int A=0;A<6;++A)Cells.Add(Valid&&A<DynamicsSensor::Width(I)?N(P.Value[A]):TEXT(""));
        for(int A=0;A<6;++A)Cells.Add(C.Have&&A<DynamicsSensor::Width(I)?N(P.Truth[A]):TEXT(""));
        Cells.Add(Valid?FString::FromInt(P.Clipped):TEXT(""));Cells.Add(FString::Printf(TEXT("%llu"),C.Dropped));Cells.Add(Model.Fault(I,NowS)?TEXT("1"):TEXT("0"));
        Write(*Csv,FString::Join(Cells,TEXT(","))+TEXT("\n"));
    }
    if(NowS>=NextFlushS){Csv->Flush();NextFlushS=NowS+5;}
}
TSharedRef<FJsonObject> FDynamicsSensorRig::Telemetry(double NowS) const
{
    auto Root=MakeShared<FJsonObject>();Root->SetStringField(TEXT("source"),TEXT("simulated_dynamics_sensors_v1"));Root->SetBoolField(TEXT("controller_connected"),false);Root->SetBoolField(TEXT("safety_rated"),false);Root->SetStringField(TEXT("imu_frame"),TEXT("UE_left_handed_X_forward_Y_right_Z_up"));Number(Root,TEXT("queue_overflows"),Model.QueueOverflows);
    TArray<TSharedPtr<FJsonValue>> Mount;for(int A=0;A<3;++A)Mount.Add(MakeShared<FJsonValueNumber>(Model.Spec.MountFromComM[A]));Root->SetArrayField(TEXT("imu_mount_from_chassis_com_m"),Mount);
    TArray<TSharedPtr<FJsonValue>> Sensors;
    for(int I=0;I<4;++I)
    {
        const auto& C=Model.Channels[I];const auto& P=C.Last;const bool Valid=Model.Usable(I,NowS);auto J=MakeShared<FJsonObject>();
        J->SetStringField(TEXT("id"),DynamicsSensor::Id(I));J->SetStringField(TEXT("status"),Model.Status(I,NowS));J->SetBoolField(TEXT("valid"),Valid);
        Number(J,TEXT("sequence"),C.Have?P.Sequence:0);Number(J,TEXT("capture_sim_s"),P.CaptureS,C.Have);Number(J,TEXT("delivery_sim_s"),P.DeliveryS,C.Have);Number(J,TEXT("age_s"),NowS-P.CaptureS,C.Have);Number(J,TEXT("target_hz"),Model.Spec.Hz);Number(J,TEXT("latency_s"),Model.Spec.LatencyS);Number(J,TEXT("stale_s"),Model.Spec.StaleS);Number(J,TEXT("dropped_packets"),C.Dropped);
        J->SetBoolField(TEXT("injected_transport_fault_diagnostic"),Model.Fault(I,NowS));Number(J,TEXT("clipped_mask"),P.Clipped,Valid);
        static const TCHAR* Keys[]={TEXT("specific_force_x_mps2"),TEXT("specific_force_y_mps2"),TEXT("specific_force_z_mps2"),TEXT("gyro_x_rad_s"),TEXT("gyro_y_rad_s"),TEXT("gyro_z_rad_s")};
        for(int A=0;A<DynamicsSensor::Width(I);++A)Number(J,I==0?Keys[A]:I==1?TEXT("gauge_pressure_pa"):TEXT("fork_axis_force_n"),P.Value[A],Valid);
        J->SetStringField(TEXT("measurement"),I==0?TEXT("interval_delta_point_velocity_minus_gravity_and_body_angular_rate"):I==1?TEXT("cylinder_port_gauge_pressure"):TEXT("ideal_instrumented_fork_pallet_contact_projected_on_chassis_up"));
        Sensors.Add(MakeShared<FJsonValueObject>(J));
    }
    Root->SetArrayField(TEXT("sensors"),Sensors);return Root;
}
