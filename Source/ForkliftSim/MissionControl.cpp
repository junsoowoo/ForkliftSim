#include "MissionNavigation.h"
#include "StabilityLab.h"
#include "LidarSafety.h"
#include "JointSensors.h"
#include "DynamicsSensors.h"
#include "SensorLiftControl.h"
#include "CameraSensors.h"
#include "EngineUtils.h"
#include "HAL/PlatformTime.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Serialization/Archive.h"
namespace
{
    void Write(FArchive& A,const FString& Text){FTCHARToUTF8 U(*Text);A.Serialize((void*)U.Get(),U.Length());}
    double DistanceToMap(FVector2D P,const TArray<SensorDrive::FSegment>& Map)
    {
        double Best=1e9;for(const auto& L:Map){const auto V=L.B-L.A;const double T=FMath::Clamp(FVector2D::DotProduct(P-L.A,V)/FMath::Max(1e-10,V.SizeSquared()),0.,1.);Best=FMath::Min(Best,(P-L.A-V*T).SizeSquared());}return FMath::Sqrt(Best);
    }
}
FMissionRig::~FMissionRig(){if(Csv)Csv->Close();}
TSharedPtr<FMissionRig> FMissionRig::Create(FString& Error)
{
    auto R=MakeShared<FMissionRig>();if(!R->Model.Spec.Load(Error))return nullptr;
    if(FParse::Param(FCommandLine::Get(),TEXT("MissionFixture")))R->Model.Spec.Pickup=FVector2D(2,0);
    R->CsvPath=FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir()/TEXT("Telemetry")/FString::Printf(TEXT("Mission-%s-%s.csv"),*FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")),*FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(8)));IFileManager::Get().MakeDirectory(*FPaths::GetPath(R->CsvPath),true);R->Csv.Reset(IFileManager::Get().CreateFileWriter(*R->CsvPath));if(!R->Csv){Error=TEXT("Cannot create mission CSV");return nullptr;}
    Write(*R->Csv,TEXT("sim_s,state,reason,checkpoint,engaged,high,carrying,pickup_receipt,dropoff_receipt,healthy,localized,fix_age_s,map_x_m,map_y_m,map_yaw_rad,height_m,left_load_n,right_load_n,vx_command_mps,vy_command_mps,yaw_command_rad_s,brake,hold_lift,retries,replans,waypoint,path_points,path_length_m,dynamic_cells,planner_ms,handling_sequence,handling_state,handling_fault,handling_full,handling_transfer,command_until_s\n"));UE_LOG(LogTemp,Display,TEXT("MISSION_CSV %s"),*R->CsvPath);return R;
}
void FMissionRig::ObserveMap(AStabilityLab& L)
{
    if(L.HandlingControl)
    {
        const auto& H=L.HandlingControl->Model.Last;bool Presence=true;for(bool V:H.Presence)Presence&=V;
        if(H.HeelValid&&H.Healthy&&H.Load[0]>80&&H.Load[1]>80&&(Presence||(Model.Carrying&&Model.SawPickup&&L.HandlingControl->Model.FullSeen&&FMath::Abs(L.Settings.ForkFaceXM()+L.Settings.Structure.LengthM-(H.HeelDepth[0]+H.HeelDepth[1])*.5-CarrierFrontM-.12)<.08)))
        {const double Front=L.Settings.ForkFaceXM()+L.Settings.Structure.LengthM-(H.HeelDepth[0]+H.HeelDepth[1])*.5;CarrierFrontM=Front-.12;CarrierBackM=Front+1.2158514+.12;CarrierUntilS=L.TimeS+.3;}
    }
    if(!L.DriveSensors||!L.VehicleGuard)return;auto& D=L.DriveSensors->Model;Model.Planner.Map=D.Map;
    if(L.TimeS<NextMapS)return;NextMapS=L.TimeS+.2;
    for(auto It=Dynamic.CreateIterator();It;++It)if(L.TimeS-It->Value.Stamp>Model.Spec.ObservationTtl)It.RemoveCurrent();
    if(!D.Ready||!D.Localized||L.TimeS-D.LastMatchS>Model.Spec.FixAge)return;
    const auto& S=L.VehicleGuard->Sensor;
    for(int I=0;I<2;++I)
    {
        if(!S.Fresh(I,L.TimeS))continue;const auto& Scan=S.Channels[I].Last;if(Scan.Sequence==LastScan[I])continue;LastScan[I]=Scan.Sequence;
        SensorDrive::FPose O;if(!D.At(Scan.CaptureS,O))continue;const auto P=SensorDrive::Compose(D.MapToOdom,O);const double C=FMath::Cos(P.Yaw),Sn=FMath::Sin(P.Yaw);
        for(int K=0;K<Scan.States.Num();K+=2)if(Scan.States[K]==LidarSafety::EReturn::Valid)
        {
            const double A=S.Spec.Angle(I,K);const FVector B=S.Spec.Mount(I)+FVector(FMath::Cos(A),FMath::Sin(A),0)*Scan.Ranges[K];const FVector2D W(P.X+C*B.X-Sn*B.Y,P.Y+Sn*B.X+C*B.Y);
            if(!Model.Spec.Bounds.IsInside(W)||DistanceToMap(W,D.Map)<.10)continue;
            const FIntPoint Cell(FMath::FloorToInt(W.X/.15),FMath::FloorToInt(W.Y/.15));Dynamic.Add(Cell,{W,Scan.CaptureS});
        }
    }
    if(Model.Carrying)
    {
        const auto P=D.MapPose(L.TimeS);const double C=FMath::Cos(P.Yaw),Sn=FMath::Sin(P.Yaw);
        for(auto It=Dynamic.CreateIterator();It;++It){const auto V=It->Value.Point-FVector2D(P.X,P.Y);if(CarriedPoint(FVector2D(C*V.X+Sn*V.Y,-Sn*V.X+C*V.Y),L.TimeS))It.RemoveCurrent();}
    }
    Model.Planner.Obstacles.Reset();for(const auto& E:Dynamic)Model.Planner.Obstacles.Add(E.Value.Point);
}
void FMissionRig::Update(AStabilityLab& L,double Dt)
{
    const bool Withhold=L.TestMode==TEXT("mecanum_mission_watchdog")&&L.MissionTestInjected&&L.TimeS<L.MissionTestEventS+1;
    if(Withhold)return;
    ObserveMap(L);Mission::FInput I;I.Now=L.TimeS;I.Dt=Dt;I.Request=Request;Request=Mission::ERequest::None;
    I.Manual=FMath::Abs(L.Throttle)>.01||FMath::Abs(L.Strafe)>.01||FMath::Abs(L.Steering)>.01||FMath::Abs(L.Lift)>.01||L.Brake>0;
    I.Camera=L.bCameraViews&&L.CameraSensors;I.Healthy=L.HandlingControl&&L.HandlingControl->Model.Last.Healthy;
    if(L.JointSensors)I.Height=L.JointSensors->Model.Channels[4].Last.Value;
    if(L.DynamicsSensors)for(int K=0;K<2;++K){I.Load[K]=L.DynamicsSensors->Model.Channels[K+2].Last.Value[0];I.Healthy&=L.DynamicsSensors->Model.Usable(K+2,I.Now)&&!L.DynamicsSensors->Model.Channels[K+2].Last.Clipped;}
    if(L.DriveSensors){const auto& D=L.DriveSensors->Model;I.Pose=D.MapPose(I.Now);I.Velocity=D.Velocity;I.FixAge=D.LastMatchS<0?1e6:I.Now-D.LastMatchS;I.Localized=D.Ready&&D.Localized&&I.FixAge<=Model.Spec.FixAge;I.Healthy&=D.Ready;I.VehicleFault|=D.Faulted;}
    if(L.VehicleGuard){I.VehicleFault|=L.VehicleGuard->Decision.Faulted||L.VehicleGuard->Decision.Emergency;I.GuardHold=L.VehicleGuard->Decision.DriveHold;I.Obstacle=L.VehicleGuard->Decision.Obstacle;}
    if(L.SensorLift)I.VehicleFault|=L.SensorLift->Model.Faulted||L.SensorLift->Model.PressureLimited;
    if(L.HandlingControl)
    {
        const auto& H=L.HandlingControl->Model;I.HandlingSequence=L.HandlingControl->JobSequence;I.HandlingState=H.State;I.HandlingReason=H.Reason;I.HandlingFailedStage=H.FailedStage;I.HandlingStartPose=H.StartPose;
        I.HandlingActive=H.Active;I.HandlingFault=H.Faulted;I.HandlingFull=H.FullSeen;I.HandlingTransfer=H.TransferSeen;
    }
    if(I.Now<=AckUntilS)
    {
        if(Model.State==TEXT("cancelled")&&I.Healthy&&!I.VehicleFault&&!I.HandlingFault&&!I.Manual&&I.Velocity.Size2D()<.025){Model.Engaged=false;Model.State=TEXT("idle");Model.Reason.Empty();}
    }
    const bool WasCarrying=Model.Carrying;const FString Before=Model.State;const double Begin=FPlatformTime::Seconds();Model.Step(I);PlannerMs=(FPlatformTime::Seconds()-Begin)*1000;
    if(!WasCarrying&&Model.Carrying){NextMapS=0;ObserveMap(L);}
    CommandUntilS=I.Now+Model.Spec.LeaseS;
    if(L.HandlingControl){if(Model.CancelHandling)L.HandlingControl->Cancel=true;if(Model.HandlingRequest!=Handling::ERequest::None)L.HandlingControl->Request=Model.HandlingRequest;}
    if(Model.Navigating()&&L.VehicleGuard&&!I.VehicleFault&&!L.VehicleGuard->EstopRequested&&I.Velocity.Size2D()<.025)L.VehicleGuard->AckRequested=true;
    if(Before!=Model.State)UE_LOG(LogTemp,Display,TEXT("MISSION_STAGE %s time=%.3f state=%s checkpoint=%s reason=%s"),*L.TestMode,L.TimeS,*Model.State,*Model.Checkpoint,*Model.Reason);
}
void FMissionRig::Apply(AStabilityLab& L)
{
    if(!Model.Engaged){Record(L);return;}
    if(L.TimeS>CommandUntilS){Model.Pause(TEXT("mission_command_expired"),true);if(L.HandlingControl&&L.HandlingControl->Model.Active){L.HandlingControl->Model.Stop(TEXT("mission_command_expired"));}}
    if(Model.Navigating()&&L.VehicleGuard&&L.VehicleGuard->Decision.Obstacle){Model.Pause(TEXT("protective_obstacle_stop"));UE_LOG(LogTemp,Display,TEXT("MISSION_PROTECT_STOP time=%.3f carrier_until=%.3f points=%d"),L.TimeS,CarrierUntilS,L.VehicleGuard->Decision.Points);}
    if(Model.Navigating()||Model.Holding()||Model.State.StartsWith(TEXT("request_"))||Model.State==TEXT("retry_pickup"))
    {L.Throttle=Model.Velocity.X;L.Strafe=Model.Velocity.Y;L.Steering=Model.YawRate/.6;L.Brake=Model.Brake?1:0;L.Lift=0;L.LiftTargetM=Model.HeightTarget;}
    Record(L);
}
bool FMissionRig::NavField(double Now) const{return Model.Navigating()&&Model.Last.Healthy&&Model.Last.Localized&&Now<=CommandUntilS;}
bool FMissionRig::CarriedPoint(FVector2D P,double Now) const
{
    const auto& I=Model.Last;return Model.Carrying&&Model.SawPickup&&I.Healthy&&Now<=CarrierUntilS&&I.Height>.12&&I.Height<.36&&I.Load[0]>80&&I.Load[1]>80&&I.Load[0]+I.Load[1]>Model.ReferenceLoad*.55&&P.X>=CarrierFrontM&&P.X<=CarrierBackM&&FMath::Abs(P.Y)<.62;
}
bool FMissionRig::GuardPoint(FVector2D P) const{return Model.GuardPoint(P);}
void FMissionRig::Record(AStabilityLab& L)
{
    if(!Csv||L.TimeS<NextCsvS)return;NextCsvS=L.TimeS+.05;const auto& M=Model;const auto& I=M.Last;
    auto N=[](double V){return FString::Printf(TEXT("%.8f"),V);};auto B=[](bool V){return V?TEXT("1"):TEXT("0");};
    TArray<FString> R={N(L.TimeS),M.State,M.Reason,M.Checkpoint,B(M.Engaged),B(M.High),B(M.Carrying),B(M.SawPickup),B(M.SawDropoff),B(I.Healthy),B(I.Localized),N(I.FixAge),N(I.Pose.X),N(I.Pose.Y),N(I.Pose.Yaw),N(I.Height),N(I.Load[0]),N(I.Load[1]),N(M.Velocity.X),N(M.Velocity.Y),N(M.YawRate),B(M.Brake),B(M.HoldLift),FString::FromInt(M.Retries),FString::FromInt(M.Replans),FString::FromInt(M.Waypoint),FString::FromInt(M.Route.Points.Num()),N(M.Route.Length),FString::FromInt(Dynamic.Num()),N(PlannerMs),FString::Printf(TEXT("%llu"),I.HandlingSequence),I.HandlingState,B(I.HandlingFault),B(I.HandlingFull),B(I.HandlingTransfer),N(CommandUntilS)};
    Write(*Csv,FString::Join(R,TEXT(","))+TEXT("\n"));if(L.TimeS>=NextFlushS){Csv->Flush();NextFlushS=L.TimeS+5;}
}
TSharedRef<FJsonObject> FMissionRig::Telemetry() const
{
    const auto& M=Model;auto J=MakeShared<FJsonObject>();J->SetStringField(TEXT("source"),TEXT("measured_grid_mission_v1"));J->SetBoolField(TEXT("ground_truth_feedback"),false);J->SetStringField(TEXT("state"),M.State);J->SetStringField(TEXT("reason"),M.Reason);J->SetStringField(TEXT("checkpoint"),M.Checkpoint);
    J->SetBoolField(TEXT("engaged"),M.Engaged);J->SetBoolField(TEXT("high"),M.High);J->SetBoolField(TEXT("selected_high"),SelectedHigh);J->SetBoolField(TEXT("carrying"),M.Carrying);J->SetBoolField(TEXT("pickup_receipt"),M.SawPickup);J->SetBoolField(TEXT("dropoff_receipt"),M.SawDropoff);J->SetBoolField(TEXT("brake"),M.Brake);J->SetBoolField(TEXT("hold_lift"),M.HoldLift);J->SetBoolField(TEXT("localized"),M.Last.Localized);J->SetBoolField(TEXT("sensors_healthy"),M.Last.Healthy);
    J->SetNumberField(TEXT("localization_age_s"),M.Last.FixAge);J->SetNumberField(TEXT("measured_x_m"),M.Last.Pose.X);J->SetNumberField(TEXT("measured_y_m"),M.Last.Pose.Y);J->SetNumberField(TEXT("measured_yaw_rad"),M.Last.Pose.Yaw);if(!M.Last.Localized){for(const TCHAR* K:{TEXT("measured_x_m"),TEXT("measured_y_m"),TEXT("measured_yaw_rad")})J->SetField(K,MakeShared<FJsonValueNull>());}
    J->SetNumberField(TEXT("height_target_m"),M.HeightTarget);
    J->SetNumberField(TEXT("vx_command_mps"),M.Velocity.X);J->SetNumberField(TEXT("vy_command_mps"),M.Velocity.Y);J->SetNumberField(TEXT("yaw_command_rad_s"),M.YawRate);J->SetNumberField(TEXT("retries"),M.Retries);J->SetNumberField(TEXT("replans"),M.Replans);J->SetNumberField(TEXT("waypoint"),M.Waypoint);J->SetNumberField(TEXT("dynamic_cells"),Dynamic.Num());J->SetNumberField(TEXT("planner_ms"),PlannerMs);J->SetNumberField(TEXT("path_length_m"),M.Route.Length);J->SetBoolField(TEXT("carrier_self_filter"),CarriedPoint(FVector2D((CarrierFrontM+CarrierBackM)*.5,0),M.Last.Now));
    TArray<TSharedPtr<FJsonValue>> Path;for(auto P:M.Route.Points){TArray<TSharedPtr<FJsonValue>> Pair;Pair.Add(MakeShared<FJsonValueNumber>(P.X));Pair.Add(MakeShared<FJsonValueNumber>(P.Y));Path.Add(MakeShared<FJsonValueArray>(Pair));}J->SetArrayField(TEXT("path_m"),Path);
    for(const auto& Pair:{TPair<FString,FVector2D>(TEXT("pickup_approach_m"),M.Spec.Pickup),TPair<FString,FVector2D>(TEXT("dropoff_approach_m"),M.Spec.Dropoff)}){TArray<TSharedPtr<FJsonValue>> V;V.Add(MakeShared<FJsonValueNumber>(Pair.Value.X));V.Add(MakeShared<FJsonValueNumber>(Pair.Value.Y));J->SetArrayField(Pair.Key,V);}return J;
}
void AStabilityLab::RequestMission(int Id){if(MissionControl)MissionControl->Request=static_cast<Mission::ERequest>(Id);}
namespace{AStabilityLab* Find(AStabilityController* C){for(TActorIterator<AStabilityLab> L(C->GetWorld());L;++L)return *L;return nullptr;}}
void AStabilityController::StartMission(){if(auto* L=Find(this))if(L->MissionControl)L->RequestMission(L->MissionControl->SelectedHigh?2:1);}
void AStabilityController::PauseMission(){if(auto* L=Find(this))L->RequestMission(3);}
void AStabilityController::ResumeMission(){if(auto* L=Find(this))L->RequestMission(4);}
void AStabilityController::SelectMissionLow(){if(auto* L=Find(this))if(L->MissionControl&&!L->MissionControl->Model.Engaged)L->MissionControl->SelectedHigh=false;}
void AStabilityController::SelectMissionHigh(){if(auto* L=Find(this))if(L->MissionControl&&!L->MissionControl->Model.Engaged)L->MissionControl->SelectedHigh=true;}
