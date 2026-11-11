#include "LidarSafety.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/Archive.h"
#include "HAL/FileManager.h"

bool LidarSafety::FSpec::Valid() const
{
    for(double V:{Hz,MinM,MaxM,SigmaM,QuantumM,PacketDrop,RayDrop,LatencyS,StaleS,ClearS,MountX,MountY,MountZ,BodyX,BodyY,Front,Rear,HalfWidth,Padding,MaxSpeed,MaxYaw})if(!FMath::IsFinite(V))return false;
    return Rays>=90&&Rays<=720&&Hz>=5&&Hz<=20&&MinM>0&&MinM<.2&&MaxM>=6&&MaxM<=15&&SigmaM>=0&&SigmaM<=.02&&QuantumM>=.0001&&QuantumM<=.01&&PacketDrop>=0&&PacketDrop<=1&&RayDrop>=0&&RayDrop<=.01&&LatencyS>=0&&LatencyS<=.15&&StaleS>LatencyS+1/Hz&&StaleS<=.5&&ClearS>=.2&&ClearS<=1&&MountX>BodyX&&MountX<=1&&MountY>BodyY&&MountY<=.7&&MountZ>=-.35&&MountZ<=.2&&BodyX>=.7&&BodyY>=.49&&Front>=1.94&&Front<=2.4&&Rear>=BodyX&&Rear<=1&&HalfWidth>=BodyY&&HalfWidth<=.9&&Padding>=.5&&Padding<=1&&MaxSpeed>0&&MaxSpeed<=1&&MaxYaw>0&&MaxYaw<=.6;
}
bool LidarSafety::FSpec::Load(FString& Error)
{
    FString Text;TSharedPtr<FJsonObject> J;if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("LidarSafety.json")))||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J){Error=TEXT("Cannot read LidarSafety.json");return false;}
    struct FField{const TCHAR* K;double* V;};FField Fields[]={ {TEXT("hz"),&Hz},{TEXT("min_range_m"),&MinM},{TEXT("max_range_m"),&MaxM},{TEXT("sigma_m"),&SigmaM},{TEXT("quantum_m"),&QuantumM},{TEXT("packet_dropout"),&PacketDrop},{TEXT("ray_dropout"),&RayDrop},{TEXT("latency_s"),&LatencyS},{TEXT("stale_s"),&StaleS},{TEXT("clear_dwell_s"),&ClearS},{TEXT("mount_x_m"),&MountX},{TEXT("mount_y_m"),&MountY},{TEXT("mount_z_m"),&MountZ},{TEXT("body_half_x_m"),&BodyX},{TEXT("body_half_y_m"),&BodyY},{TEXT("footprint_front_m"),&Front},{TEXT("footprint_rear_m"),&Rear},{TEXT("footprint_half_width_m"),&HalfWidth},{TEXT("protect_padding_m"),&Padding},{TEXT("max_drive_mps"),&MaxSpeed},{TEXT("max_yaw_rad_s"),&MaxYaw}};
    for(const auto& F:Fields)if(!J->TryGetNumberField(F.K,*F.V)){Error=TEXT("Missing LiDAR configuration field");return false;}
    double N=0,SeedValue=0;if(!J->TryGetNumberField(TEXT("rays"),N)||!J->TryGetNumberField(TEXT("seed"),SeedValue)||!FMath::IsFinite(N)||N<90||N>720||N!=FMath::FloorToDouble(N)||!FMath::IsFinite(SeedValue)||SeedValue<0||SeedValue>2147483000||SeedValue!=FMath::FloorToDouble(SeedValue)){Error=TEXT("Invalid LiDAR count/seed");return false;}Rays=int(N);Seed=int(SeedValue);if(!Valid()){Error=TEXT("LiDAR safety specification out of supported bounds");return false;}return true;
}
bool LidarSafety::FSpec::Masked(int I,int K) const
{
    const FVector O=Mount(I),D(FMath::Cos(Angle(I,K)),FMath::Sin(Angle(I,K)),0);
    return FMath::LineBoxIntersection(FBox(FVector(-BodyX,-BodyY,-1),FVector(BodyX,BodyY,1)),O*FVector(1,1,0),O*FVector(1,1,0)+D*MaxM,D*MaxM);
}
void LidarSafety::FSensor::Reset(const FSpec& S){*this=FSensor();Spec=S;for(int I=0;I<2;++I)Channels[I].Random.Initialize(S.Seed+I*113);}
bool LidarSafety::FSensor::Fault(int I,double Now) const{return ManualOutage||((FaultMask&(1u<<I))&&Now>=FaultStartS&&Now<FaultEndS);}
void LidarSafety::FSensor::Capture(int I,double Now,const TArray<double>& Ranges,const TArray<EReturn>& States)
{
    if(I<0||I>1||Ranges.Num()!=Spec.Rays||States.Num()!=Spec.Rays)return;auto& C=Channels[I];FScan S;S.Id=I;S.CaptureS=Now;S.DeliveryS=Now+Spec.LatencyS;S.Sequence=++C.Sequence;S.Ranges.SetNumZeroed(Spec.Rays);S.States=States;
    for(int K=0;K<Spec.Rays;++K)
    {
        if(States[K]==EReturn::SelfMasked)continue;
        if(C.Random.FRand()<Spec.RayDrop){S.States[K]=EReturn::Dropout;continue;}
        if(States[K]!=EReturn::Valid)continue;
        if(Ranges[K]<Spec.MinM){S.States[K]=EReturn::TooClose;continue;}
        const double G=FMath::Sqrt(-2*FMath::Loge(FMath::Max(1e-9,double(C.Random.FRand()))))*FMath::Cos(2*UE_DOUBLE_PI*C.Random.FRand());
        const double V=FMath::RoundToDouble((Ranges[K]+Spec.SigmaM*G)/Spec.QuantumM)*Spec.QuantumM;
        if(V<Spec.MinM)S.States[K]=EReturn::TooClose;else if(V>Spec.MaxM)S.States[K]=EReturn::NoReturn;else S.Ranges[K]=V;
    }
    if(Fault(I,Now)||C.Random.FRand()<Spec.PacketDrop||Pending.Num()>=64){++C.Dropped;return;}Pending.Add(MoveTemp(S));
}
void LidarSafety::FSensor::Deliver(double Now)
{
    for(int I=0;I<Pending.Num();){if(Pending[I].DeliveryS>Now+1e-9){++I;continue;}auto S=MoveTemp(Pending[I]);Pending.RemoveAt(I,1,EAllowShrinking::No);auto& C=Channels[S.Id];if(Fault(S.Id,Now)){++C.Dropped;continue;}S.DeliveryS=Now;C.Last=MoveTemp(S);C.Have=true;}
}
bool LidarSafety::FSensor::Fresh(int I,double Now) const{return Channels[I].Have&&Now>=Channels[I].Last.CaptureS-1e-9&&Now-Channels[I].Last.CaptureS<=Spec.StaleS;}
void LidarSafety::FDecision::Step(const FSensor& S,double Now,bool CriticalHealthy,bool LiftFault,bool Neutral,bool Estop,bool Ack,bool ManualCreep,bool DriveFault,const TFunction<bool(FVector2D)>& TargetPermit,const TFunction<bool(FVector2D)>& NavigationField)
{
    if(CreepWasActive&&!ManualCreep)DriveHold=true;
    CreepWasActive=false;
    AckRejected=false;Emergency|=Estop;DataHealthy=CriticalHealthy&&S.Fresh(0,Now)&&S.Fresh(1,Now);Obstacle=false;Points=0;ClearanceM=S.Spec.MaxM;bool Unknown=false;
    if(DataHealthy)for(int I=0;I<2;++I)
    {
        const auto& Scan=S.Channels[I].Last;
        if(Scan.States.Num()!=S.Spec.Rays||Scan.Ranges.Num()!=S.Spec.Rays){Unknown=true;continue;}
        for(int K=0;K<S.Spec.Rays;++K)
        {
            const auto R=Scan.States[K];if(R==EReturn::Dropout){Unknown=true;continue;}if(R==EReturn::TooClose){
                // The whole unresolved [0,min_range] ray segment must remain
                // inside the already authorized local target envelope.
                const double A=S.Spec.Angle(I,K);const FVector O=S.Spec.Mount(I),E=O+FVector(FMath::Cos(A),FMath::Sin(A),0)*S.Spec.MinM;
                if(TargetPermit&&TargetPermit(FVector2D(O.X,O.Y))&&TargetPermit(FVector2D(E.X,E.Y)))continue;
                Obstacle=true;++Points;ClearanceM=0;continue;
            }if(R!=EReturn::Valid)continue;
            const double D=Scan.Ranges[K];if(!FMath::IsFinite(D)||D<S.Spec.MinM||D>S.Spec.MaxM){Unknown=true;continue;}
            const double A=S.Spec.Angle(I,K);const FVector P=S.Spec.Mount(I)+FVector(FMath::Cos(A)*D,FMath::Sin(A)*D,0);
            if(TargetPermit&&TargetPermit(FVector2D(P.X,P.Y)))continue;
            const double DX=FMath::Max3(-S.Spec.Rear-P.X,0.,P.X-S.Spec.Front),DY=FMath::Max(0.,FMath::Abs(P.Y)-S.Spec.HalfWidth);
            ClearanceM=FMath::Min(ClearanceM,FMath::Sqrt(DX*DX+DY*DY));
            if(NavigationField?NavigationField(FVector2D(P.X,P.Y)):(P.X>=-S.Spec.Rear-S.Spec.Padding&&P.X<=S.Spec.Front+S.Spec.Padding&&FMath::Abs(P.Y)<=S.Spec.HalfWidth+S.Spec.Padding)){Obstacle=true;++Points;}
        }
    }
    DataHealthy&=!Unknown;
    if(DataHealthy){if(HealthySinceS<0)HealthySinceS=Now;}else HealthySinceS=-1;
    if(DataHealthy&&!Obstacle){if(ClearSinceS<0)ClearSinceS=Now;}else ClearSinceS=-1;
    if(Emergency||(!DataHealthy&&EverReady)||LiftFault||DriveFault)
    {Faulted=true;Reason=Emergency?TEXT("emergency_stop"):DriveFault?TEXT("drive_controller_fault"):LiftFault?TEXT("lift_controller_fault"):Unknown?TEXT("scan_unknown"):TEXT("required_sensor_stale");}
    if(Obstacle)DriveHold=true;
    if(Faulted)
    {
        DriveHold=LiftHold=true;State=TEXT("fault_latched");
        // Fresh inputs can be acknowledged into an obstacle drive hold, allowing
        // stationary manual handling. A blocked aisle never releases traction.
        if(Ack&&!Estop&&DataHealthy&&Neutral&&Now-HealthySinceS>=S.Spec.ClearS){Faulted=Emergency=false;LiftHold=false;DriveHold=true;Reason.Empty();State=TEXT("recovery_hold");}
        else {AckRejected=Ack;return;}
    }
    if(!DataHealthy){DriveHold=true;LiftHold=false;State=TEXT("initializing");return;}
    EverReady=true;
    if(ManualCreep){DriveHold=LiftHold=false;CreepWasActive=true;State=TEXT("manual_creep");return;}
    if(Obstacle){DriveHold=true;LiftHold=false;State=TEXT("obstacle_hold");AckRejected=Ack;return;}
    if(DriveHold)
    {
        const bool Clear=ClearSinceS>=0&&Now-ClearSinceS>=S.Spec.ClearS;
        if(Clear&&Neutral&&(Ack||!HaveRun)){DriveHold=false;HaveRun=true;State=TEXT("ready");}
        else {AckRejected=Ack;State=TEXT("clear_wait_ack");}
    }
    else State=TEXT("ready");
}
namespace{void Write(FArchive& A,const FString& S){FTCHARToUTF8 U(*S);A.Serialize((void*)U.Get(),U.Length());}}
FLidarSafetyRig::~FLidarSafetyRig(){if(Csv)Csv->Close();}
TSharedPtr<FLidarSafetyRig> FLidarSafetyRig::Create(FString& Error)
{
    LidarSafety::FSpec S;if(!S.Load(Error))return nullptr;auto R=MakeShared<FLidarSafetyRig>();R->Sensor.Reset(S);
    FString Fault;if(FParse::Value(FCommandLine::Get(),TEXT("LidarFault="),Fault)){R->Sensor.FaultMask=Fault==TEXT("front")?1:Fault==TEXT("rear")?2:Fault==TEXT("all")?3:0;if(!R->Sensor.FaultMask){Error=TEXT("Unknown LiDAR fault");return nullptr;}}
    R->CsvPath=FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir()/TEXT("Telemetry")/FString::Printf(TEXT("LidarSafety-%s-%s.csv"),*FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")),*FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(8)));IFileManager::Get().MakeDirectory(*FPaths::GetPath(R->CsvPath),true);R->Csv.Reset(IFileManager::Get().CreateFileWriter(*R->CsvPath));if(!R->Csv){Error=TEXT("Cannot open LiDAR CSV");return nullptr;}
    Write(*R->Csv,TEXT("sim_s,state,reason,drive_hold,lift_hold,fault_latched,obstacle,points,clearance_m,front_sequence,rear_sequence,front_capture_s,rear_capture_s,front_fresh,rear_fresh,ack_rejected,truth_speed_mps,truth_side_mps,truth_fork_m\n"));UE_LOG(LogTemp,Display,TEXT("LIDAR_SAFETY_CSV %s"),*R->CsvPath);return R;
}
TSharedRef<FJsonObject> FLidarSafetyRig::Telemetry(double Now) const
{
    auto J=MakeShared<FJsonObject>();J->SetStringField(TEXT("source"),TEXT("simulated_dual_planar_lidar_v1"));J->SetStringField(TEXT("state"),Decision.State);J->SetStringField(TEXT("reason"),Decision.Reason);J->SetBoolField(TEXT("drive_hold"),Decision.DriveHold);J->SetBoolField(TEXT("lift_hold"),Decision.LiftHold);J->SetBoolField(TEXT("fault_latched"),Decision.Faulted);J->SetBoolField(TEXT("obstacle"),Decision.Obstacle);J->SetBoolField(TEXT("ack_rejected"),Decision.AckRejected);J->SetBoolField(TEXT("safety_rated"),false);J->SetNumberField(TEXT("clearance_m"),Decision.ClearanceM);J->SetNumberField(TEXT("protected_points"),Decision.Points);
    if(!Decision.DataHealthy)J->SetField(TEXT("clearance_m"),MakeShared<FJsonValueNull>());
    J->SetBoolField(TEXT("data_healthy"),Decision.DataHealthy);
    J->SetNumberField(TEXT("field_front_m"),Sensor.Spec.Front+Sensor.Spec.Padding);J->SetNumberField(TEXT("field_rear_m"),Sensor.Spec.Rear+Sensor.Spec.Padding);J->SetNumberField(TEXT("field_half_width_m"),Sensor.Spec.HalfWidth+Sensor.Spec.Padding);
    TArray<TSharedPtr<FJsonValue>> Scans;
    for(int I=0;I<2;++I)
    {
        const auto& C=Sensor.Channels[I];auto S=MakeShared<FJsonObject>();const bool Fresh=Sensor.Fresh(I,Now);
        S->SetStringField(TEXT("id"),I?TEXT("rear_right"):TEXT("front_left"));S->SetBoolField(TEXT("fresh"),Fresh);
        S->SetNumberField(TEXT("sequence"),C.Have?C.Last.Sequence:0);S->SetNumberField(TEXT("capture_sim_s"),C.Last.CaptureS);S->SetNumberField(TEXT("delivery_sim_s"),C.Last.DeliveryS);S->SetNumberField(TEXT("age_s"),C.Have?Now-C.Last.CaptureS:0);
        S->SetNumberField(TEXT("latency_s"),Sensor.Spec.LatencyS);S->SetNumberField(TEXT("stale_s"),Sensor.Spec.StaleS);S->SetNumberField(TEXT("target_hz"),Sensor.Spec.Hz);S->SetNumberField(TEXT("dropped_packets"),C.Dropped);
        S->SetNumberField(TEXT("start_deg"),I?0:-180);S->SetNumberField(TEXT("fov_deg"),270);S->SetNumberField(TEXT("min_range_m"),Sensor.Spec.MinM);S->SetNumberField(TEXT("max_range_m"),Sensor.Spec.MaxM);
        TArray<TSharedPtr<FJsonValue>> Mount;const FVector M=Sensor.Spec.Mount(I);for(int A=0;A<3;++A)Mount.Add(MakeShared<FJsonValueNumber>(M[A]));S->SetArrayField(TEXT("calibrated_mount_chassis_m"),Mount);
        TArray<TSharedPtr<FJsonValue>> Ranges,States;
        for(int K=0;K<Sensor.Spec.Rays;++K)
        {
            const bool Have=Fresh&&C.Last.States.IsValidIndex(K);const int Code=Have?int(C.Last.States[K]):-1;
            States.Add(MakeShared<FJsonValueNumber>(Code));Ranges.Add(Have&&Code==0?TSharedPtr<FJsonValue>(MakeShared<FJsonValueNumber>(C.Last.Ranges[K])):TSharedPtr<FJsonValue>(MakeShared<FJsonValueNull>()));
        }
        S->SetArrayField(TEXT("ranges_m"),Ranges);S->SetArrayField(TEXT("return_codes"),States);Scans.Add(MakeShared<FJsonValueObject>(S));
    }
    J->SetArrayField(TEXT("scanners"),Scans);return J;
}
