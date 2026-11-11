#include "HandlingControl.h"
#include "StabilityLab.h"
#include "PalletPerception.h"
#include "JointSensors.h"
#include "DynamicsSensors.h"
#include "ProximitySensors.h"
#include "SensorLiftControl.h"
#include "LidarSafety.h"
#include "MissionNavigation.h"
#include "Serialization/Archive.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "EngineUtils.h"
namespace {void Write(FArchive& A,const FString& S){FTCHARToUTF8 U(*S);A.Serialize((void*)U.Get(),U.Length());}}
FHandlingRig::~FHandlingRig(){if(Csv)Csv->Close();}
TSharedPtr<FHandlingRig> FHandlingRig::Create(FString& Error)
{
    auto R=MakeShared<FHandlingRig>();if(!R->Model.Spec.Load(Error))return nullptr;
    R->CsvPath=FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir()/TEXT("Telemetry")/FString::Printf(TEXT("Handling-%s-%s.csv"),*FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")),*FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(8)));IFileManager::Get().MakeDirectory(*FPaths::GetPath(R->CsvPath),true);R->Csv.Reset(IFileManager::Get().CreateFileWriter(*R->CsvPath));if(!R->Csv){Error=TEXT("Cannot create handling CSV");return nullptr;}
    Write(*R->Csv,TEXT("sim_s,state,reason,active,fault,place,tier_m,healthy,measurements,geometry,vision,perception_age_s,full,receipt,transfer,measured_height_m,target_height_m,left_load_n,right_load_n,left_depth_m,right_depth_m,uncertainty_m,lateral_error_m,height_error_m,yaw_error_rad,measured_x_m,measured_y_m,measured_yaw_rad,vx_command_mps,vy_command_mps,yaw_command_rad_s,brake,target_permission\n"));UE_LOG(LogTemp,Display,TEXT("HANDLING_CSV %s"),*R->CsvPath);return R;
}
void FHandlingRig::Update(AStabilityLab& L,double Dt)
{
    Handling::FInput I;I.Now=L.TimeS;I.Dt=Dt;I.Request=Request;Request=Handling::ERequest::None;I.Cancel=Cancel;Cancel=false;
    I.Manual=FMath::Abs(L.Throttle)>.01||FMath::Abs(L.Strafe)>.01||FMath::Abs(L.Steering)>.01||FMath::Abs(L.Lift)>.01||L.Brake>0;
    I.Healthy=L.JointSensors&&L.DynamicsSensors&&L.DriveSensors&&L.SensorLift&&L.VehicleGuard&&L.ProximitySensors;
    if(L.JointSensors){I.Healthy&=L.JointSensors->Model.Usable(4,I.Now)&&L.JointSensors->Model.Usable(5,I.Now);I.Height=L.JointSensors->Model.Channels[4].Last.Value;}
    if(L.DynamicsSensors)for(int K=0;K<2;++K){I.Healthy&=L.DynamicsSensors->Model.Usable(K+2,I.Now)&&!L.DynamicsSensors->Model.Channels[K+2].Last.Clipped;I.Load[K]=L.DynamicsSensors->Model.Channels[K+2].Last.Value[0];}
    if(L.ProximitySensors)
    {
        auto& P=L.ProximitySensors->Model;I.HeelValid=true;
        for(int K=0;K<8;++K)I.Healthy&=P.TransportValid(K,I.Now);
        for(int K=0;K<4;++K){I.Presence[K]=P.MeasurementValid(K+4,I.Now)&&P.Channels[K+4].Last.Detected;}
        for(int K=0;K<2;++K){I.HeelValid&=P.MeasurementValid(K,I.Now);I.HeelDepth[K]=1.2-P.Spec.ForwardOffsetM-P.Channels[K].Last.RangeM;}
    }
    if(L.DriveSensors){auto& D=L.DriveSensors->Model;I.Healthy&=D.Ready&&D.UnobservedS<.1;I.Pose=D.MapPose(I.Now);I.Velocity=D.Velocity;I.Fault|=D.Faulted;}
    if(L.SensorLift){I.Healthy&=L.SensorLift->Model.Armed;I.Fault|=L.SensorLift->Model.Faulted||L.SensorLift->Model.PressureLimited;}
    if(L.VehicleGuard)I.Fault|=L.VehicleGuard->Decision.Faulted||L.VehicleGuard->Decision.Emergency;
    if(L.PalletRecognition)
    {
        const auto& P=*L.PalletRecognition;I.Camera=P.CameraEnabled;I.PerceptionStamp=P.LastFrame.CaptureS;I.Vision=P.Detection.Valid&&P.Detection.Sequence==P.LastSequence&&I.Now-P.Detection.CaptureS<=P.Spec.FrameStaleS;
        I.Measurements=P.SignalsFresh;I.Geometry=(I.Vision||P.Tracked)&&P.Decision.State!=TEXT("supported")&&P.Decision.State!=TEXT("uneven_support");I.Full=P.Decision.ConfirmedFull;I.Aligned=P.Aligned;
        for(int K=0;K<2;++K)I.Depth[K]=P.Depth[K];I.Uncertainty=P.UncertaintyM;I.Y=P.OffsetY;I.Z=P.OffsetZ;
        I.Yaw=P.Tracked&&L.DriveSensors?FMath::UnwindRadians(P.AnchorYaw-L.DriveSensors->Model.Predicted(I.Now).Yaw):P.Detection.Yaw;
    }
    Model.Spec.RackX=L.RackSettings.X;Model.Spec.RackY=L.RackSettings.Y;Model.Spec.Low=L.RackSettings.Low;Model.Spec.High=L.RackSettings.High;Model.Spec.RackDepth=L.RackSettings.Depth;Model.Spec.RackWidth=L.RackSettings.Width;
    if(I.Now<=AckUntilS&&!Model.Active&&Model.Faulted&&I.Healthy&&!I.Fault&&!I.Manual&&I.Velocity.Size2D()<.025){Model.Faulted=false;Model.State=TEXT("idle");Model.Reason.Empty();AckUntilS=-1;}
    const bool WasActive=Model.Active;const FString Before=Model.State;Model.Step(I);if(!WasActive&&Model.Active){GuardReleased=false;++JobSequence;}
    if(L.PalletRecognition)
    {
        auto& P=*L.PalletRecognition;
        if(Model.State==TEXT("insert")&&!P.Locked&&P.HaveAnchor&&P.Aligned&&P.Detection.Valid&&L.TimeS-P.Detection.CaptureS<.5&&P.Detection.DistanceM<1.8){P.Locked=true;P.LockedPoint=P.AnchorPlanePoint;P.LockedYaw=P.AnchorYaw;}
        if(!Model.Active||Model.State!=TEXT("insert"))P.Locked=false;
    }
if(Before!=Model.State)UE_LOG(LogTemp,Display,TEXT("HANDLING_STAGE %s time=%.3f state=%s reason=%s"),*L.TestMode,I.Now,*Model.State,*Model.Reason);
    // An explicit new job authorizes clearing a healthy stationary docking hold.
    // Unexpected obstacles latch the job below and cannot auto-resume.
    if(Model.Active&&Model.Permit&&!Model.Brake&&L.VehicleGuard&&!L.VehicleGuard->Decision.DriveHold)GuardReleased=true;
    if(Model.Active&&!GuardReleased&&L.VehicleGuard&&!I.Fault&&!L.VehicleGuard->EstopRequested&&I.Velocity.Size2D()<.025)L.VehicleGuard->AckRequested=true;
}
void FHandlingRig::Apply(AStabilityLab& L)
{
    auto& M=Model;
    if(M.Active&&M.Permit&&L.VehicleGuard&&L.VehicleGuard->Decision.Obstacle){
        const auto& S=L.VehicleGuard->Sensor;int Count=0;for(int A=0;A<2;++A){const auto& Scan=S.Channels[A].Last;for(int K=0;K<Scan.States.Num();++K)if(Scan.States[K]==LidarSafety::EReturn::Valid){const double Angle=S.Spec.Angle(A,K);const FVector P=S.Spec.Mount(A)+FVector(FMath::Cos(Angle)*Scan.Ranges[K],FMath::Sin(Angle)*Scan.Ranges[K],0);if(P.X>=-S.Spec.Rear-S.Spec.Padding&&P.X<=S.Spec.Front+S.Spec.Padding&&FMath::Abs(P.Y)<=S.Spec.HalfWidth+S.Spec.Padding&&!M.TargetPoint(FVector2D(P.X,P.Y))&&Count++<5)UE_LOG(LogTemp,Display,TEXT("HANDLING_BLOCKED_POINT %.4f %.4f"),P.X,P.Y);}}
        M.Stop(TEXT("non_target_obstacle"));
    }
    if(M.Working()){L.Throttle=M.Velocity.X;L.Strafe=M.Velocity.Y;L.Steering=M.YawRate/.6;L.Brake=M.Brake?1:0;L.Lift=0;L.LiftTargetM=M.HeightTarget;}
    if(!Csv||L.TimeS<NextCsv)return;NextCsv=L.TimeS+.05;
    auto N=[](double V){return FString::Printf(TEXT("%.8f"),V);};auto B=[](bool V){return V?TEXT("1"):TEXT("0");};auto& I=M.Last;
    TArray<FString> R={N(I.Now),M.State,M.Reason,B(M.Active),B(M.Faulted),B(M.Place),N(M.Tier),B(I.Healthy),B(I.Measurements),B(I.Geometry),B(I.Vision),N(I.Now-I.PerceptionStamp),B(I.Full),B(M.Receipt),B(M.TransferSeen),N(I.Height),N(M.HeightTarget),N(I.Load[0]),N(I.Load[1]),N(I.Depth[0]),N(I.Depth[1]),N(I.Uncertainty),N(I.Y),N(I.Z),N(I.Yaw),N(I.Pose.X),N(I.Pose.Y),N(I.Pose.Yaw),N(M.Velocity.X),N(M.Velocity.Y),N(M.YawRate),B(M.Brake),B(M.Permit)};
    Write(*Csv,FString::Join(R,TEXT(","))+TEXT("\n"));if(L.TimeS>=NextFlush){Csv->Flush();NextFlush=L.TimeS+5;}
}
TSharedRef<FJsonObject> FHandlingRig::Telemetry() const
{
    const auto& M=Model;auto J=MakeShared<FJsonObject>();J->SetStringField(TEXT("source"),TEXT("measured_local_handling_v1"));J->SetBoolField(TEXT("ground_truth_feedback"),false);J->SetStringField(TEXT("state"),M.State);J->SetStringField(TEXT("reason"),M.Reason);
    J->SetBoolField(TEXT("active"),M.Active);J->SetBoolField(TEXT("fault_latched"),M.Faulted);J->SetBoolField(TEXT("place"),M.Place);J->SetBoolField(TEXT("full_seen"),M.FullSeen);J->SetBoolField(TEXT("transfer_seen"),M.TransferSeen);J->SetBoolField(TEXT("load_receipt"),M.Receipt);J->SetBoolField(TEXT("target_permission"),M.Permit);J->SetBoolField(TEXT("brake"),M.Brake);J->SetBoolField(TEXT("sensors_healthy"),M.Last.Healthy);
    J->SetNumberField(TEXT("tier_m"),M.Tier);J->SetNumberField(TEXT("target_height_m"),M.HeightTarget);J->SetNumberField(TEXT("vx_command_mps"),M.Velocity.X);J->SetNumberField(TEXT("vy_command_mps"),M.Velocity.Y);J->SetNumberField(TEXT("yaw_command_rad_s"),M.YawRate);return J;
}
void AStabilityLab::RequestHandling(int RequestId){if(MissionControl&&MissionControl->Model.Engaged){if(RequestId==6)MissionControl->Request=Mission::ERequest::Cancel;else return;}if(HandlingControl){HandlingControl->Request=static_cast<Handling::ERequest>(RequestId);}}
namespace {void Request(AStabilityController* C,int Id){for(TActorIterator<AStabilityLab> L(C->GetWorld());L;++L){L->RequestHandling(Id);break;}}}
void AStabilityController::PickGround(){Request(this,1);}void AStabilityController::PickLow(){Request(this,2);}void AStabilityController::PickHigh(){Request(this,3);}void AStabilityController::PlaceLow(){Request(this,4);}void AStabilityController::PlaceHigh(){Request(this,5);}void AStabilityController::CancelHandling(){Request(this,6);}
