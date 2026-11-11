#include "PalletPerception.h"
#include "StabilityLab.h"
#include "JointSensors.h"
#include "DynamicsSensors.h"
#include "ProximitySensors.h"
#include "CameraMounts.h"
#include "MissionNavigation.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/PlatformTime.h"
#include "Serialization/Archive.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonSerializer.h"
namespace
{
    void Write(FArchive& A,const FString& S){FTCHARToUTF8 U(*S);A.Serialize((void*)U.Get(),U.Length());}
    void Number(TSharedRef<FJsonObject> J,const TCHAR* K,double V,bool Valid=true){if(Valid&&FMath::IsFinite(V))J->SetNumberField(K,V);else J->SetField(K,MakeShared<FJsonValueNull>());}
}
FPalletPerceptionRig::~FPalletPerceptionRig(){if(Csv)Csv->Close();}
TSharedPtr<FPalletPerceptionRig> FPalletPerceptionRig::Create(FString& Error)
{
    auto R=MakeShared<FPalletPerceptionRig>();if(!R->Spec.Load(Error))return nullptr;R->Spec.CameraOffsetM=CameraMounts::FrontCm/100.;R->Spec.CameraPitchDeg=CameraMounts::FrontRotation.Pitch;
    R->CsvPath=FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir()/TEXT("Telemetry")/FString::Printf(TEXT("PalletPerception-%s-%s.csv"),*FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")),*FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(8)));IFileManager::Get().MakeDirectory(*FPaths::GetPath(R->CsvPath),true);R->Csv.Reset(IFileManager::Get().CreateFileWriter(*R->CsvPath));if(!R->Csv){Error=TEXT("Cannot create pallet perception CSV");return nullptr;}
    Write(*R->Csv,TEXT("sim_s,state,reason,sequence,vision_valid,vision_age_s,confidence,plane_distance_m,center_y_m,center_z_m,yaw_rad,left_width_m,right_width_m,tracked,signals_fresh,aligned,depth_left_m,depth_right_m,uncertainty_m,lateral_error_m,height_error_m,last_full_before_support,left_load_n,right_load_n\n"));UE_LOG(LogTemp,Display,TEXT("PALLET_PERCEPTION_CSV %s"),*R->CsvPath);return R;
}
bool FPalletPerceptionRig::HeightAt(double Time,double& H) const
{
    if(Heights.IsEmpty()||Time<Heights[0].Time||Time>Heights.Last().Time)return false;
    for(int I=1;I<Heights.Num();++I)if(Heights[I].Time>=Time){const auto& A=Heights[I-1];const auto& B=Heights[I];H=FMath::Lerp(A.Value,B.Value,(Time-A.Time)/(B.Time-A.Time));return true;}H=Heights.Last().Value;return true;
}
void FPalletPerceptionRig::Update(AStabilityLab& Lab)
{
    const double Now=Lab.TimeS;double Height=0;bool HeightValid=Lab.JointSensors&&Lab.JointSensors->Model.Usable(4,Now);
    if(HeightValid){const auto& H=Lab.JointSensors->Model.Channels[4].Last;Height=H.Value;if(Heights.IsEmpty()||H.CaptureS>Heights.Last().Time)Heights.Add({H.CaptureS,H.Value});while(Heights.Num()>1&&Now-Heights[0].Time>5)Heights.RemoveAt(0,1,EAllowShrinking::No);}
    double Load[2]={};bool LoadsValid=Lab.DynamicsSensors.IsValid();if(Lab.DynamicsSensors)for(int I=0;I<2;++I){LoadsValid&=Lab.DynamicsSensors->Model.Usable(I+2,Now)&&!Lab.DynamicsSensors->Model.Channels[I+2].Last.Clipped;Load[I]=Lab.DynamicsSensors->Model.Channels[I+2].Last.Value[0];}
    const bool Loaded=LoadsValid&&Load[0]>80&&Load[1]>80,Unilateral=LoadsValid&&FMath::Max(Load[0],Load[1])>80&&FMath::Min(Load[0],Load[1])<20;
    const auto* Handling=Lab.HandlingControl.Get();const bool LocalVision=!Handling||!Handling->Model.Active||Handling->Model.State==TEXT("pick_height")||Handling->Model.State==TEXT("acquire")||Handling->Model.State==TEXT("align")||Handling->Model.State==TEXT("insert")||Handling->Model.State==TEXT("pick_lift");
    // Loaded support and surveyed transfer/withdrawal use their own measured
    // receipts, ranges and loads. Do not run a hole search against carried
    // cargo or fabricate fresh geometry when that search is not required.
    if(Loaded||Unilateral||!LocalVision)HaveAnchor=false;
    const bool Enabled=Lab.bCameraViews&&Lab.CameraSensors;CameraEnabled=Enabled;
    if(Enabled&&Lab.CameraSensors->ReadFront(LastFrame,LastSequence))
    {
        LastSequence=LastFrame.Sequence;PalletPerception::FDetection Attempt;Attempt.Sequence=LastFrame.Sequence;Attempt.CaptureS=LastFrame.CaptureS;Attempt.DeliveryS=LastFrame.DeliveryS;const bool Navigating=Lab.MissionControl&&Lab.MissionControl->Model.Navigating();if(!Loaded&&!Unilateral&&LocalVision&&(!Navigating||Now>=NextDetectionS)){const double Begin=FPlatformTime::Seconds();Attempt=PalletPerception::Detect(LastFrame,Spec);if(FParse::Param(FCommandLine::Get(),TEXT("MissionProfile")))UE_LOG(LogTemp,Display,TEXT("PALLET_DETECTION_TIME sim=%.6f ms=%.3f"),Now,(FPlatformTime::Seconds()-Begin)*1000);NextDetectionS=Now+1.;}
        if(Locked&&HaveAnchor&&Attempt.Valid&&Lab.DriveSensors)
        {
            SensorDrive::FPose Capture;double H=Height;
            if(!Lab.DriveSensors->Model.At(LastFrame.CaptureS,Capture)||!HeightAt(LastFrame.CaptureS,H))Attempt.Valid=false;
            else
            {
                const FVector N(FMath::Cos(Attempt.Yaw),FMath::Sin(Attempt.Yaw),0),T(-N.Y,N.X,0),P=N*Attempt.DistanceM+T*Attempt.CenterY;
                const auto W=SensorDrive::Compose(Capture,{P.X+Lab.Settings.ForkFaceXM()-.03+Spec.CameraOffsetM.X,P.Y+Spec.CameraOffsetM.Y,Attempt.Yaw});
                if(!PalletPerception::ConsistentAnchor(FVector(W.X,W.Y,Attempt.CenterZ+H),LockedPoint,W.Yaw,LockedYaw))Attempt.Valid=false;
            }
        }
        const FString FramePath=CsvPath+TEXT(".frame.json");
        if(Lab.IsPalletPerceptionTest()&&Now>(Lab.TestMode.EndsWith(TEXT("high"))?16:4)&&!IFileManager::Get().FileExists(*FramePath))
        {
            TArray<uint8> Bytes;Bytes.Append(reinterpret_cast<const uint8*>(LastFrame.DepthM.GetData()),LastFrame.DepthM.Num()*sizeof(float));FFileHelper::SaveArrayToFile(Bytes,*(FramePath+TEXT(".depth.bin")));FFileHelper::SaveArrayToFile(LastFrame.Validity,*(FramePath+TEXT(".validity.bin")));
            auto J=MakeShared<FJsonObject>();J->SetNumberField(TEXT("width"),LastFrame.Spec.Width);J->SetNumberField(TEXT("height"),LastFrame.Spec.Height);J->SetNumberField(TEXT("fov_deg"),LastFrame.Spec.FovDeg);J->SetNumberField(TEXT("pitch_deg"),Spec.CameraPitchDeg);J->SetNumberField(TEXT("capture_s"),LastFrame.CaptureS);J->SetNumberField(TEXT("sequence"),LastSequence);J->SetBoolField(TEXT("detected"),Attempt.Valid);J->SetNumberField(TEXT("planes"),Attempt.PlaneCandidates);J->SetNumberField(TEXT("support_pass"),Attempt.SupportPass);J->SetNumberField(TEXT("roof_pass"),Attempt.RoofPass);J->SetNumberField(TEXT("gap_pass"),Attempt.GapPass);J->SetNumberField(TEXT("edge_pass"),Attempt.EdgePass);FString Text;auto Writer=TJsonWriterFactory<>::Create(&Text);FJsonSerializer::Serialize(J,Writer);FFileHelper::SaveStringToFile(Text,*FramePath);UE_LOG(LogTemp,Display,TEXT("PALLET_DEPTH_FRAME %s"),*FramePath);
        }
        if(!Attempt.Valid&&HaveAnchor)
        {
            // Retain a bounded blind-zone track only when the target is now
            // near the camera. Losing a visible, distant pallet invalidates it.
            bool Coherent=false;
            if(Lab.DriveSensors&&Lab.DriveSensors->Model.Ready)
            {
                auto Current=Lab.DriveSensors->Model.Predicted(Now);Lab.DriveSensors->Model.At(LastFrame.CaptureS,Current);
                const auto Local=SensorDrive::Compose(SensorDrive::Inverse(Current),{AnchorPlanePoint.X,AnchorPlanePoint.Y,AnchorYaw});double H=Height;HeightAt(LastFrame.CaptureS,H);
                const FVector Center(Local.X-(Lab.Settings.ForkFaceXM()-.03+Spec.CameraOffsetM.X),Local.Y-Spec.CameraOffsetM.Y,AnchorPlanePoint.Z-H),Normal(FMath::Cos(Local.Yaw),FMath::Sin(Local.Yaw),0),Tangent(-Normal.Y,Normal.X,0);
                int EvidencePixels=0,SurfacePoints=0;for(uint8 V:LastFrame.Validity)EvidencePixels+=V==uint8(DepthCamera::EValidity::Valid)||V==uint8(DepthCamera::EValidity::TooClose);
                const double Fx=LastFrame.Spec.Width/(2*FMath::Tan(FMath::DegreesToRadians(LastFrame.Spec.FovDeg)*.5));const auto Rotation=FRotator(Spec.CameraPitchDeg,0,0).Quaternion();
                for(int Y=2;Y<LastFrame.Spec.Height;Y+=5)for(int X=2;X<LastFrame.Spec.Width;X+=5){const int K=Y*LastFrame.Spec.Width+X;if(LastFrame.Validity[K]!=uint8(DepthCamera::EValidity::Valid))continue;const double D=LastFrame.DepthM[K];const FVector P=Rotation.RotateVector(FVector(D,(X-(LastFrame.Spec.Width-1)*.5)*D/Fx,((LastFrame.Spec.Height-1)*.5-Y)*D/Fx)),Delta=P-Center;SurfacePoints+=FMath::Abs(FVector::DotProduct(Delta,Normal))<.05&&FMath::Abs(FVector::DotProduct(Delta,Tangent))<.6&&FMath::Abs(Delta.Z)<.10;}
                const bool RecentMission=Lab.MissionControl&&Lab.MissionControl->Model.Engaged&&!Locked&&Now-Detection.CaptureS<.3;
                Coherent=Center.X<(Locked?2.:RecentMission?3.:1.3)&&EvidencePixels>500&&(Center.X<.55||SurfacePoints>=12);
            }
            HaveAnchor=Coherent;
        }
        if(Attempt.Valid){Detection=Attempt;HaveAnchor=false;}
        SensorDrive::FPose AtCapture;double H;
        if(Attempt.Valid&&!Loaded&&!Unilateral&&Lab.DriveSensors&&Lab.DriveSensors->Model.Ready&&Lab.DriveSensors->Model.At(Detection.CaptureS,AtCapture)&&HeightAt(Detection.CaptureS,H))
        {
            AnchorBody=AtCapture;AnchorHeight=H;const FVector N(FMath::Cos(Detection.Yaw),FMath::Sin(Detection.Yaw),0),T(-N.Y,N.X,0);const FVector P=N*Detection.DistanceM+T*Detection.CenterY;
            const auto World=SensorDrive::Compose(AtCapture,{P.X+Lab.Settings.ForkFaceXM()-.03+Spec.CameraOffsetM.X,P.Y+Spec.CameraOffsetM.Y,Detection.Yaw});AnchorPlanePoint=FVector(World.X,World.Y,Detection.CenterZ+H);AnchorYaw=World.Yaw;HaveAnchor=true;
        }
    }
    PalletPerception::FSignals In;In.Now=Now;In.Loaded=Loaded;In.UnilateralLoad=Unilateral;
    In.VisionFresh=Enabled&&Detection.Valid&&Detection.Sequence==LastSequence&&Now-Detection.CaptureS<=Spec.FrameStaleS;
    FVector P=FVector::ZeroVector;double Yaw=Detection.Yaw;Tracked=false;UncertaintyM=.025;
    if(HaveAnchor&&Enabled&&HeightValid&&Lab.DriveSensors&&Lab.DriveSensors->Model.Ready&&Now-Detection.CaptureS<=Spec.TrackMaxS)
    {
        const auto Current=Lab.DriveSensors->Model.Predicted(Now);const double Distance=FVector2D(Current.X-AnchorBody.X,Current.Y-AnchorBody.Y).Size();
        if(Distance<=Spec.TrackMaxDistanceM)
        {
            const auto Local=SensorDrive::Compose(SensorDrive::Inverse(Current),{AnchorPlanePoint.X,AnchorPlanePoint.Y,AnchorYaw});P=FVector(Local.X-(Lab.Settings.ForkFaceXM()-.03)-Spec.CameraOffsetM.X,Local.Y-Spec.CameraOffsetM.Y,AnchorPlanePoint.Z-Height);Yaw=Local.Yaw;Tracked=true;UncertaintyM=.025+.06*Distance+.003*(Now-Detection.CaptureS);
        }
    }
    if(!Tracked&&In.VisionFresh){const FVector N(FMath::Cos(Yaw),FMath::Sin(Yaw),0),T(-N.Y,N.X,0);P=N*Detection.DistanceM+T*Detection.CenterY;P.Z=Detection.CenterZ;}
    In.Tracked=Tracked;In.UncertaintyM=UncertaintyM;
    SignalsFresh=HeightValid&&LoadsValid&&Lab.ProximitySensors;bool Blocked=false;
    if(Lab.ProximitySensors)for(int I=0;I<8;++I)
    {
        const auto& M=Lab.ProximitySensors->Model;SignalsFresh&=M.TransportValid(I,Now)&&M.Channels[I].Last.Reading!=ProximitySensor::EReading::Dropout;
        if(I>=4){SignalsFresh&=M.MeasurementValid(I,Now);In.Presence[I-4]=M.Channels[I].Last.Detected;}
        else if(I<2)Blocked|=M.Channels[I].Last.Reading==ProximitySensor::EReading::TooClose;
    }
    OffsetY=P.Y+Spec.CameraOffsetM.Y;OffsetZ=P.Z+Spec.CameraOffsetM.Z+.15;
    const double LateralMargin=(Spec.OpeningWidth-Spec.ForkWidth)*.5-.01,VerticalMargin=(Spec.OpeningHeight-Spec.ForkThickness)*.5-.005;
    Aligned=!Blocked&&(In.VisionFresh||Tracked)&&FMath::Abs(OffsetY)<LateralMargin&&FMath::Abs(OffsetZ)<VerticalMargin&&FMath::Abs(Yaw)<FMath::DegreesToRadians(8.);
    const FVector N(FMath::Cos(Yaw),FMath::Sin(Yaw),0);const double D=N.X*P.X+N.Y*P.Y;
    for(int I=0;I<2;++I){const double ForkY=(I?1:-1)*Spec.ForkSpacing*.5-Spec.CameraOffsetM.Y;Depth[I]=Spec.ForkLength+.03-Spec.CameraOffsetM.X-(D-N.Y*ForkY)/FMath::Max(.5,N.X);In.Depth[I]=Depth[I];}
    RangeFused=false;
    if((In.VisionFresh||Tracked)&&SignalsFresh&&Lab.ProximitySensors&&!Loaded&&!Unilateral)
    {
        const auto& M=Lab.ProximitySensors->Model;bool Associated=true;double Measured[2]={};
        for(int I=0;I<2;++I){Associated&=M.MeasurementValid(I,Now);Measured[I]=Spec.ForkLength-M.Spec.ForwardOffsetM-M.Channels[I].Last.RangeM;Associated&=FMath::Abs(Measured[I]-Depth[I])<.08;}
        if(Associated){RangeFused=true;for(int I=0;I<2;++I)In.Depth[I]=Depth[I]=.8*Measured[I]+.2*Depth[I];UncertaintyM=FMath::Min(UncertaintyM,.03+.01*FMath::Abs(Yaw));In.UncertaintyM=UncertaintyM;}
    }
    In.Alignment=Aligned;In.Fresh=SignalsFresh;Decision.Step(In,Spec);
    if(!Csv||Now<NextCsvS)return;NextCsvS=Now+.05;
    auto Num=[](double V){return FString::Printf(TEXT("%.8f"),V);};auto B=[](bool V){return V?TEXT("1"):TEXT("0");};
    TArray<FString> R={Num(Now),Decision.State,Decision.Reason,FString::Printf(TEXT("%llu"),Detection.Sequence),B(In.VisionFresh),Num(Now-Detection.CaptureS),Num(Detection.Confidence),Num(Detection.DistanceM),Num(Detection.CenterY),Num(Detection.CenterZ),Num(Detection.Yaw),Num(Detection.HoleWidth[0]),Num(Detection.HoleWidth[1]),B(Tracked),B(SignalsFresh),B(Aligned),Num(Depth[0]),Num(Depth[1]),Num(UncertaintyM),Num(OffsetY),Num(OffsetZ),B(Decision.LastFullBeforeSupport),Num(Load[0]),Num(Load[1])};Write(*Csv,FString::Join(R,TEXT(","))+TEXT("\n"));if(Now>=NextFlushS){Csv->Flush();NextFlushS=Now+5;}
}
TSharedRef<FJsonObject> FPalletPerceptionRig::Telemetry(double Now) const
{
    auto J=MakeShared<FJsonObject>();const bool Fresh=CameraEnabled&&Detection.Valid&&Detection.Sequence==LastSequence&&Now-Detection.CaptureS<=Spec.FrameStaleS,Geometry=SignalsFresh&&(Fresh||Tracked)&&Decision.State!=TEXT("supported")&&Decision.State!=TEXT("uneven_support");
    J->SetStringField(TEXT("source"),TEXT("depth_template_fork_sensor_fusion_v1"));J->SetStringField(TEXT("state"),Decision.State);J->SetStringField(TEXT("reason"),Decision.Reason);J->SetBoolField(TEXT("ground_truth_input"),false);J->SetBoolField(TEXT("vision_fresh"),Fresh);J->SetBoolField(TEXT("tracked"),Tracked);J->SetBoolField(TEXT("signals_fresh"),SignalsFresh);J->SetBoolField(TEXT("aligned"),Aligned&&Geometry);J->SetBoolField(TEXT("fully_inserted"),Decision.ConfirmedFull);J->SetBoolField(TEXT("last_full_before_support"),Decision.LastFullBeforeSupport);
    J->SetBoolField(TEXT("heel_range_fused"),RangeFused&&Geometry);
    Number(J,TEXT("sequence"),Detection.Sequence);Number(J,TEXT("capture_sim_s"),Detection.CaptureS,Detection.Sequence>0);Number(J,TEXT("age_s"),Now-Detection.CaptureS,Detection.Sequence>0);Number(J,TEXT("confidence"),Detection.Confidence,Fresh);Number(J,TEXT("plane_distance_m"),Detection.DistanceM,Fresh);Number(J,TEXT("hole_center_y_m"),Detection.CenterY,Fresh);Number(J,TEXT("hole_center_z_m"),Detection.CenterZ,Fresh);Number(J,TEXT("yaw_rad"),Detection.Yaw,Fresh);Number(J,TEXT("left_opening_width_m"),Detection.HoleWidth[0],Fresh);Number(J,TEXT("right_opening_width_m"),Detection.HoleWidth[1],Fresh);Number(J,TEXT("nominal_opening_height_m"),Spec.OpeningHeight);
    Number(J,TEXT("depth_left_m"),Depth[0],Geometry);Number(J,TEXT("depth_right_m"),Depth[1],Geometry);Number(J,TEXT("depth_uncertainty_m"),UncertaintyM,Geometry);Number(J,TEXT("lateral_error_m"),OffsetY,Geometry);Number(J,TEXT("height_error_m"),OffsetZ,Geometry);Number(J,TEXT("valid_gap_samples"),Detection.GapEvidence,Fresh);Number(J,TEXT("censored_gap_samples"),Detection.CensoredEvidence,Fresh);return J;
}
