#include "CameraSensors.h"
#include "StabilityLab.h"
#include "CameraMounts.h"
#include "PalletPerception.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "Vehicle3DHelpers.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/Texture2D.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Math/Float16Color.h"
#include "RHIGPUReadback.h"
#include "RenderingThread.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/FileManager.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace
{
    struct FCameraPacket
    {
        uint64 Sequence=0;double CaptureS=0,DeliveryS=0,ForkHeightM=0,ExpectedZ=0,IdealCenterZ=0;
        FTransform Pose;TArray<float> DepthM,RangeM;TArray<uint8> Validity;TArray<FColor> RGB,Preview;
        int Counts[5]={};
    };
    struct FCameraChannel
    {
        DepthCamera::FSpec Spec;bool bRGBD=false,bCopyNextTick=false,bInFlight=false,bDecoded=false;
        TAtomic<bool> Submitted{false},CopyFailed{false},PollQueued{false},ReadComplete{false};
        TArray<uint8> RawPixels;TArray<double> RayFactors;
        TWeakObjectPtr<USceneCaptureComponent2D> Capture;TWeakObjectPtr<UTextureRenderTarget2D> Target;TWeakObjectPtr<AStaticMeshActor> Fixture;
        TUniquePtr<FRHIGPUTextureReadback> Readback;TSharedPtr<FCameraPacket> Pending,Latest;FRandomStream Random;
        uint64 Sequence=0;double NextS=0;int Skipped=0,ValidChecks=0,NearChecks=0,FarChecks=0,Failures=0;double MaxErrorM=0;
    };
    using FChannelPtr=TSharedPtr<FCameraChannel,ESPMode::ThreadSafe>;
    struct FPixelUpload { FUpdateTextureRegion2D Region;TArray<FColor> Pixels; };
    void Upload(UTexture2D* Image,const TArray<FColor>& Pixels,int W,int H)
    {
        if(!Image)return;
        auto Data=MakeShared<FPixelUpload,ESPMode::ThreadSafe>();Data->Region=FUpdateTextureRegion2D(0,0,0,0,W,H);Data->Pixels=Pixels;
        Image->UpdateTextureRegions(0,1,&Data->Region,W*4,4,reinterpret_cast<uint8*>(Data->Pixels.GetData()),[Data](uint8*,const FUpdateTextureRegion2D*){});
    }
    UTexture2D* Image(const DepthCamera::FSpec& S,bool Depth)
    {
        auto* T=UTexture2D::CreateTransient(S.Width,S.Height,PF_B8G8R8A8);
        T->SRGB=true;T->NeverStream=true;T->Filter=Depth?TF_Nearest:TF_Bilinear;T->UpdateResource();return T;
    }
    FColor Preview(const DepthCamera::FSample& P,const DepthCamera::FSpec& S)
    {
        if(P.Validity==DepthCamera::EValidity::TooClose)return FColor(210,30,200);
        if(P.Validity==DepthCamera::EValidity::Dropout)return FColor(65,65,65);
        if(P.Validity!=DepthCamera::EValidity::Valid)return FColor(8,12,20);
        const double T=1-FMath::Clamp((P.RangeM-S.MinM)/(S.MaxM-S.MinM),0.,1.);
        auto C=[](double V){return uint8(255*FMath::Clamp(V,0.,1.));};
        return FColor(C(1.5-FMath::Abs(4*T-3)),C(1.5-FMath::Abs(4*T-2)),C(1.5-FMath::Abs(4*T-1)));
    }
    FColor Color(const FFloat16Color& P)
    {
        // The paired capture contains linear HDR RGB + optical Z, not final LDR.
        // This fixed display curve changes color presentation only.
        auto Tone=[](float V){V=FMath::Max(0.f,V);return FMath::Clamp((V*(2.51f*V+.03f))/(V*(2.43f*V+.59f)+.14f),0.f,1.f);};
        return FLinearColor(Tone(P.R.GetFloat()),Tone(P.G.GetFloat()),Tone(P.B.GetFloat()),1).ToFColorSRGB();
    }
    bool LoadSpecs(DepthCamera::FSpec& F,DepthCamera::FSpec& R,FString& Error)
    {
        FString Text;TSharedPtr<FJsonObject> J;
        if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("Cameras.json"))) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J) || !J)
        {Error=TEXT("Cannot read Config/Cameras.json");return false;}
        double W=0,H=0,Seed=0;
        auto Read=[&](const TCHAR* K,double& V){return J->TryGetNumberField(K,V) && FMath::IsFinite(V);};
        if(!Read(TEXT("width"),W)||!Read(TEXT("height"),H)||!Read(TEXT("seed"),Seed)||!Read(TEXT("hz"),F.Hz)||
           !Read(TEXT("front_fov_deg"),F.FovDeg)||!Read(TEXT("front_min_range_m"),F.MinM)||!Read(TEXT("front_max_range_m"),F.MaxM)||
           !Read(TEXT("noise_sigma_base_m"),F.SigmaBaseM)||!Read(TEXT("noise_sigma_per_m2"),F.SigmaPerM2)||!Read(TEXT("quantization_m"),F.QuantumM)||
           !Read(TEXT("dropout_probability"),F.Dropout)||!Read(TEXT("latency_s"),F.LatencyS))
        {Error=TEXT("Missing/nonfinite camera parameter");return false;}
        if(W!=FMath::FloorToDouble(W)||H!=FMath::FloorToDouble(H)||W<64||W>640||H<36||H>480||Seed<0||Seed>2147483000||Seed!=FMath::FloorToDouble(Seed))
        {Error=TEXT("Camera dimensions or seed outside supported bounds");return false;}
        F.Width=W;F.Height=H;F.Seed=Seed;R=F;R.Seed+=1;
        if(!Read(TEXT("rear_fov_deg"),R.FovDeg)||!Read(TEXT("rear_min_range_m"),R.MinM)||!Read(TEXT("rear_max_range_m"),R.MaxM))
        {Error=TEXT("Missing rear camera parameters");return false;}
        for(const auto* S:{&F,&R})if(S->Hz<1||S->Hz>20||S->FovDeg<20||S->FovDeg>130||S->MinM<.05||S->MaxM<=S->MinM||S->MaxM>30||S->SigmaBaseM<0||S->SigmaBaseM>.1||S->SigmaPerM2<0||S->SigmaPerM2>.01||S->QuantumM<=0||S->QuantumM>.05||S->Dropout<0||S->Dropout>.5||S->LatencyS<0||S->LatencyS>.5)
        {Error=TEXT("Camera range/noise/rate/latency outside supported bounds");return false;}
        return true;
    }
    FChannelPtr Channel(AStabilityLab& Lab,const DepthCamera::FSpec& S,bool Rear)
    {
        auto C=MakeShared<FCameraChannel,ESPMode::ThreadSafe>();C->Spec=S;C->bRGBD=Rear;C->Random.Initialize(S.Seed);C->NextS=Lab.TimeS+(Rear?.5/S.Hz:0);
        // Intrinsics do not change between frames. Cache exact ray lengths,
        // avoiding trigonometry per pixel on every delivered depth frame.
        C->RayFactors.SetNumUninitialized(S.Width*S.Height);
        for(int Y=0;Y<S.Height;++Y)for(int X=0;X<S.Width;++X)C->RayFactors[Y*S.Width+X]=DepthCamera::RayFactor(S,X,Y);
        auto* Target=NewObject<UTextureRenderTarget2D>(&Lab);Target->RenderTargetFormat=Rear?RTF_RGBA16f:RTF_R32f;
        Target->ClearColor=FLinearColor::Black;Target->TargetGamma=1;Target->InitAutoFormat(S.Width,S.Height);Target->UpdateResourceImmediate(true);
        auto* Capture=NewObject<USceneCaptureComponent2D>(&Lab);Lab.AddInstanceComponent(Capture);
        auto* Parent=Rear?static_cast<USceneComponent*>(Lab.Chassis->Body):static_cast<USceneComponent*>(Lab.Fork->Carriage);
        Capture->SetupAttachment(Parent);Capture->SetAbsolute(false,false,true);Capture->bCaptureEveryFrame=false;Capture->bCaptureOnMovement=false;
        Capture->CaptureSource=Rear?ESceneCaptureSource::SCS_SceneColorSceneDepth:ESceneCaptureSource::SCS_SceneDepth;
        Capture->TextureTarget=Target;Capture->FOVAngle=S.FovDeg;Capture->MaxViewDistanceOverride=2500;
        Capture->bOverride_CustomNearClippingPlane=true;Capture->CustomNearClippingPlane=1;
        Capture->ShowFlags.SetMotionBlur(false);Capture->ShowFlags.SetTemporalAA(false);Capture->ShowFlags.SetDynamicShadows(false);
        Capture->ShowFlags.SetLumenGlobalIllumination(false);Capture->ShowFlags.SetLumenReflections(false);Capture->ShowFlags.SetScreenSpaceReflections(false);
        Capture->ShowFlags.SetAmbientOcclusion(false);Capture->ShowFlags.SetVolumetricFog(false);Capture->ShowFlags.SetLightShafts(false);Capture->LODDistanceFactor=2;
        Capture->RegisterComponent();Capture->SetWorldScale3D(FVector::OneVector);
        Capture->SetWorldLocation(Parent->GetComponentLocation()+Parent->GetComponentQuat().RotateVector(Rear?CameraMounts::RearCm:CameraMounts::FrontCm));
        Capture->SetWorldRotation(Parent->GetComponentQuat()*(Rear?FRotator(-8,180,0).Quaternion():CameraMounts::FrontRotation.Quaternion()));
        C->Capture=Capture;C->Target=Target;C->Readback=MakeUnique<FRHIGPUTextureReadback>(Rear?TEXT("RearRGBD"):TEXT("FrontDepth"));
        if(Rear){Lab.RearViewCapture=Capture;Lab.RearViewTexture=Target;Lab.RearColorImage=Image(S,false);Lab.RearDepthImage=Image(S,true);}
        else {Lab.FrontViewCapture=Capture;Lab.FrontViewTexture=Target;Lab.FrontDepthImage=Image(S,true);}
        if(FParse::Param(FCommandLine::Get(),TEXT("CameraValidate")))
        {
            auto* Fixture=Lab.GetWorld()->SpawnActor<AStaticMeshActor>();auto* Box=Fixture->GetStaticMeshComponent();Box->SetMobility(EComponentMobility::Movable);
            Box->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));
            Box->SetCollisionEnabled(ECollisionEnabled::NoCollision);Box->SetWorldScale3D(FVector(.01,1.2,1.2));
            Vehicle3D::Color(Box,Rear?FLinearColor(.1,.8,.2):FLinearColor(.8,.2,.1));C->Fixture=Fixture;
            // Isolated metric calibration: the pitched front lens can otherwise
            // see the real floor before a distant calibration plane. Normal
            // operation and pallet-scene tests retain all scene occluders.
            Capture->PrimitiveRenderMode=ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;Capture->ShowOnlyActorComponents(Fixture);
        }
        return C;
    }
    void Validate(FCameraChannel& C,const FCameraPacket& P)
    {
        if(P.ExpectedZ<=0)return;
        const double E=FMath::Abs(P.IdealCenterZ-P.ExpectedZ);C.MaxErrorM=FMath::Max(C.MaxErrorM,E);
        const auto V=static_cast<DepthCamera::EValidity>(P.Validity[C.Spec.Width/2+C.Spec.Width*(C.Spec.Height/2)]);
        if(E>.025){++C.Failures;return;}
        if(P.ExpectedZ<C.Spec.MinM){++C.NearChecks;if(V!=DepthCamera::EValidity::TooClose)++C.Failures;}
        else if(P.ExpectedZ>C.Spec.MaxM){++C.FarChecks;if(V!=DepthCamera::EValidity::TooFar)++C.Failures;}
        else {++C.ValidChecks;if(V!=DepthCamera::EValidity::Valid && V!=DepthCamera::EValidity::Dropout)++C.Failures;}
    }
    void Poll(const FChannelPtr& C,AStabilityLab& Lab)
    {
        if(C->bInFlight && C->Submitted.Load() && !C->ReadComplete.Load() && !C->PollQueued.Exchange(true))
        {
            // Staging texture Lock/Unlock requires the render thread on D3D11.
            // Copy only ready bytes there; noise/processing stays on game thread.
            ENQUEUE_RENDER_COMMAND(CameraSensorPoll)([C](FRHICommandListImmediate&)
            {
                if(C->CopyFailed.Load())C->ReadComplete.Store(true);
                else if(C->Readback->IsReady())
                {
                    int Pitch=0,Height=0;const void* Bytes=C->Readback->Lock(Pitch,&Height);
                    const int Bpp=C->bRGBD?sizeof(FFloat16Color):sizeof(float),W=C->Spec.Width,H=C->Spec.Height;
                    if(Bytes && Pitch>=W && Height>=H)
                    {
                        C->RawPixels.SetNumUninitialized(W*H*Bpp);
                        for(int Y=0;Y<H;++Y)FMemory::Memcpy(C->RawPixels.GetData()+Y*W*Bpp,static_cast<const uint8*>(Bytes)+Y*Pitch*Bpp,W*Bpp);
                    }
                    else C->CopyFailed.Store(true);
                    if(Bytes)C->Readback->Unlock();
                    C->ReadComplete.Store(true);
                }
                C->PollQueued.Store(false);
            });
        }
        if(C->bInFlight && C->ReadComplete.Load())
        {
            C->bInFlight=false;
            if(C->CopyFailed.Load()){C->Pending.Reset();++C->Skipped;return;}
            const int RowPitch=C->Spec.Width;const void* Data=C->RawPixels.GetData();
            const auto& S=C->Spec;auto& P=*C->Pending;
            const int Count=S.Width*S.Height;P.DepthM.SetNumZeroed(Count);P.RangeM.SetNumZeroed(Count);P.Validity.SetNum(Count);P.Preview.SetNum(Count);
            if(C->bRGBD)P.RGB.SetNum(Count);
            for(int Y=0;Y<S.Height;++Y)for(int X=0;X<S.Width;++X)
            {
                const int I=Y*S.Width+X,Source=Y*RowPitch+X;
                const double Z=C->bRGBD?static_cast<const FFloat16Color*>(Data)[Source].A.GetFloat():static_cast<const float*>(Data)[Source];
                if(I==S.Width/2+S.Width*(S.Height/2))P.IdealCenterZ=Z*.01;
                const auto Sample=DepthCamera::Measure(Z,C->RayFactors[I],S,C->Random);
                P.DepthM[I]=Sample.DepthM;P.RangeM[I]=Sample.RangeM;P.Validity[I]=static_cast<uint8>(Sample.Validity);++P.Counts[P.Validity[I]];
                P.Preview[I]=Preview(Sample,S);if(C->bRGBD)P.RGB[I]=Color(static_cast<const FFloat16Color*>(Data)[Source]);
            }
            C->bDecoded=true;
        }
        if(C->bDecoded && C->Pending && Lab.TimeS>=C->Pending->CaptureS+C->Spec.LatencyS)
        {
            C->bDecoded=false;
            if(Lab.TimeS-C->Pending->CaptureS>.75){C->Pending.Reset();++C->Skipped;return;}
            C->Pending->DeliveryS=Lab.TimeS;Validate(*C,*C->Pending);C->Latest=MoveTemp(C->Pending);
            Upload(C->bRGBD?Lab.RearDepthImage.Get():Lab.FrontDepthImage.Get(),C->Latest->Preview,C->Spec.Width,C->Spec.Height);
            if(C->bRGBD)Upload(Lab.RearColorImage,C->Latest->RGB,C->Spec.Width,C->Spec.Height);
        }
        // Copy next tick, after the preceding deferred capture was enqueued.
        // Never ReadPixels / FlushRenderingCommands or wait for the GPU here.
        if(C->bCopyNextTick)
        {
            C->bCopyNextTick=false;C->bInFlight=true;C->Submitted.Store(false);C->CopyFailed.Store(false);C->ReadComplete.Store(false);
            auto* Resource=C->Target->GameThread_GetRenderTargetResource();
            ENQUEUE_RENDER_COMMAND(CameraSensorReadback)([C,Resource](FRHICommandListImmediate& Cmd)
            {
                if(Resource && Resource->GetRenderTargetTexture())C->Readback->EnqueueCopy(Cmd,Resource->GetRenderTargetTexture());
                else C->CopyFailed.Store(true);
                C->Submitted.Store(true);
            });
        }
    }
    void Capture(const FChannelPtr& C,AStabilityLab& Lab)
    {
        C->NextS=Lab.TimeS+1/C->Spec.Hz;C->Pending=MakeShared<FCameraPacket>();auto& P=*C->Pending;
        P.Sequence=++C->Sequence;P.CaptureS=Lab.TimeS;P.Pose=C->Capture->GetComponentTransform();P.ForkHeightM=Lab.ForkHeightM;
        if(C->Fixture.IsValid())
        {
            P.ExpectedZ=Lab.TimeS<2?(C->bRGBD?3.:2.):Lab.TimeS<4?.15:Lab.TimeS<6?(C->Spec.MaxM+1):2.;
            C->Fixture->SetActorLocationAndRotation(P.Pose.GetLocation()+P.Pose.GetRotation().GetAxisX()*(P.ExpectedZ*100+.5),P.Pose.GetRotation());
        }
        C->Capture->CaptureSceneDeferred();C->bCopyNextTick=true;
    }
}
struct FCameraSensorState { FChannelPtr Front,Rear;bool bValidation=false; };

bool FCameraSensorRig::ReadFront(DepthCamera::FFrame& Out,uint64 AfterSequence) const
{
    if(!State||!State->Front->Latest||State->Front->Latest->Sequence<=AfterSequence)return false;
    const auto& P=*State->Front->Latest;Out.Spec=State->Front->Spec;Out.Sequence=P.Sequence;Out.CaptureS=P.CaptureS;Out.DeliveryS=P.DeliveryS;Out.DepthM=P.DepthM;Out.Validity=P.Validity;return true;
}

namespace
{
    TSharedRef<FJsonObject> Json(const FCameraChannel& C,double Now,bool Enabled)
    {
        const auto& S=C.Spec;auto J=MakeShared<FJsonObject>();
        J->SetStringField(TEXT("type"),C.bRGBD?TEXT("rgbd"):TEXT("tof_depth"));J->SetStringField(TEXT("model"),TEXT("generic-tof-range-noise-v1"));
        J->SetStringField(TEXT("depth_definition"),TEXT("optical_z_m"));J->SetStringField(TEXT("status"),!Enabled?TEXT("disabled"):!C.Latest?TEXT("waiting"):Now-C.Latest->CaptureS>.5?TEXT("stale"):TEXT("running"));
        J->SetBoolField(TEXT("safety_rated"),false);J->SetNumberField(TEXT("width"),S.Width);J->SetNumberField(TEXT("height"),S.Height);J->SetNumberField(TEXT("target_hz"),S.Hz);
        const double Focal=S.Width/(2*FMath::Tan(FMath::DegreesToRadians(S.FovDeg)*.5));
        J->SetNumberField(TEXT("fx"),Focal);J->SetNumberField(TEXT("fy"),Focal);J->SetNumberField(TEXT("cx"),(S.Width-1)*.5);J->SetNumberField(TEXT("cy"),(S.Height-1)*.5);
        J->SetNumberField(TEXT("min_range_m"),S.MinM);J->SetNumberField(TEXT("max_range_m"),S.MaxM);J->SetNumberField(TEXT("configured_latency_s"),S.LatencyS);J->SetNumberField(TEXT("skipped_frames"),C.Skipped);
        if(!C.Latest){J->SetField(TEXT("center_range_m"),MakeShared<FJsonValueNull>());return J;}
        const auto& P=*C.Latest;const int Mid=S.Width/2+S.Width*(S.Height/2);
        J->SetNumberField(TEXT("sequence"),double(P.Sequence));J->SetNumberField(TEXT("capture_sim_s"),P.CaptureS);J->SetNumberField(TEXT("delivery_sim_s"),P.DeliveryS);J->SetNumberField(TEXT("age_s"),FMath::Max(0.,Now-P.CaptureS));
        J->SetNumberField(TEXT("valid_fraction"),double(P.Counts[0])/(S.Width*S.Height));J->SetStringField(TEXT("center_status"),DepthCamera::Name(static_cast<DepthCamera::EValidity>(P.Validity[Mid])));
        if(P.Validity[Mid]==0){J->SetNumberField(TEXT("center_range_m"),P.RangeM[Mid]);J->SetNumberField(TEXT("center_depth_m"),P.DepthM[Mid]);}
        else {J->SetField(TEXT("center_range_m"),MakeShared<FJsonValueNull>());J->SetField(TEXT("center_depth_m"),MakeShared<FJsonValueNull>());}
        const FVector Pos=P.Pose.GetLocation()/100.;const FQuat Q=P.Pose.GetRotation();
        J->SetArrayField(TEXT("capture_position_world_m"),{MakeShared<FJsonValueNumber>(Pos.X),MakeShared<FJsonValueNumber>(Pos.Y),MakeShared<FJsonValueNumber>(Pos.Z)});
        J->SetArrayField(TEXT("capture_rotation_xyzw"),{MakeShared<FJsonValueNumber>(Q.X),MakeShared<FJsonValueNumber>(Q.Y),MakeShared<FJsonValueNumber>(Q.Z),MakeShared<FJsonValueNumber>(Q.W)});
        J->SetStringField(TEXT("pose_scope"),TEXT("simulator mount truth for diagnostics; not SLAM or estimated localization"));
        J->SetStringField(TEXT("pose_axes"),TEXT("Unreal camera X forward, Y right, Z up; image u right, v down; optical depth forward"));
        for(int I=1;I<5;++I)J->SetNumberField(FString::Printf(TEXT("%s_pixels"),DepthCamera::Name(static_cast<DepthCamera::EValidity>(I))),P.Counts[I]);
        if(C.bRGBD){J->SetNumberField(TEXT("rgb_sequence"),double(P.Sequence));J->SetNumberField(TEXT("depth_sequence"),double(P.Sequence));J->SetBoolField(TEXT("registered_same_exposure"),true);}
        return J;
    }
}
FCameraSensorRig::~FCameraSensorRig()
{if(State)for(auto C:{State->Front,State->Rear})if(C && C->Fixture.IsValid())C->Fixture->Destroy();}
TSharedPtr<FCameraSensorRig> FCameraSensorRig::Create(AStabilityLab& Lab,FString& Error)
{
    DepthCamera::FSpec F,R;if(!LoadSpecs(F,R,Error))return nullptr;
    if(FParse::Param(FCommandLine::Get(),TEXT("CameraRearDepth")))Lab.bRearDepthView=true;
    auto Rig=MakeShared<FCameraSensorRig>();Rig->State=MakeShared<FCameraSensorState>();
    Rig->State->Front=Channel(Lab,F,false);Rig->State->Rear=Channel(Lab,R,true);Rig->State->bValidation=FParse::Param(FCommandLine::Get(),TEXT("CameraValidate"));
    UE_LOG(LogTemp,Display,TEXT("CAMERA_SENSORS front=depth rear=rgbd width=%d height=%d hz=%.1f latency_s=%.3f capture_passes=2 async_readback=1"),F.Width,F.Height,F.Hz,F.LatencyS);
    return Rig;
}
void FCameraSensorRig::Update(AStabilityLab& Lab)
{
    if(!State)return;
    for(auto C:{State->Front,State->Rear})Poll(C,Lab);
    auto F=State->Front,R=State->Rear;
    const bool FD=!F->Pending && Lab.TimeS>=F->NextS,RD=!R->Pending && Lab.TimeS>=R->NextS;
    if(FD && (!RD || F->NextS<=R->NextS))Capture(F,Lab);else if(RD)Capture(R,Lab);
}
TSharedRef<FJsonObject> FCameraSensorRig::Telemetry(double NowS,bool Enabled) const
{
    auto J=MakeShared<FJsonObject>();J->SetStringField(TEXT("source"),TEXT("unreal_render_depth_with_sensor_errors"));
    J->SetObjectField(TEXT("front"),Json(*State->Front,NowS,Enabled));J->SetObjectField(TEXT("rear"),Json(*State->Rear,NowS,Enabled));return J;
}
void FCameraSensorRig::Draw(AStabilityLab& Lab,AHUD* HUD,UCanvas* Canvas)
{
    if(!HUD || !Canvas)return;
    const float W=FMath::Min(320.f,Canvas->ClipX*.25f),H=W*State->Front->Spec.Height/State->Front->Spec.Width,X=Canvas->ClipX-W-12;float Y=12;
    for(int I=0;I<2;++I)
    {
        const auto C=I==0?State->Front:State->Rear;const auto& S=C->Spec;auto P=C->Latest;
        HUD->DrawRect(FLinearColor(.02,.03,.045,.96),X-4,Y-2,W+8,H+65);
        const FString Title=I==0?FString::Printf(TEXT("FRONT ToF | lift %.2f m"),Lab.ForkHeightM):Lab.bRearDepthView?TEXT("REAR RGB-D | depth [O]"):TEXT("REAR RGB-D | RGB [O]");
        HUD->DrawText(Title,FLinearColor(.3,.9,.85),X+4,Y,GEngine->GetSmallFont(),.8f);
        UTexture2D* T=I==0?Lab.FrontDepthImage.Get():Lab.bRearDepthView?Lab.RearDepthImage.Get():Lab.RearColorImage.Get();
        if(P && T)Canvas->K2_DrawTexture(T,FVector2D(X,Y+20),FVector2D(W,H),FVector2D::ZeroVector,FVector2D(1,1),FLinearColor::White,BLEND_Opaque,0,FVector2D(.5,.5));
        FString Status=TEXT("WAITING FOR SENSOR FRAME");
        if(P)
        {
            const int Mid=S.Width/2+S.Width*(S.Height/2);
            const FString Center=P->Validity[Mid]==0?FString::Printf(TEXT("%.2f m"),P->RangeM[Mid]):FString(DepthCamera::Name(static_cast<DepthCamera::EValidity>(P->Validity[Mid])));
            Status=FString::Printf(TEXT("center %s | valid %.0f%% | age %.0fms"),*Center,100.*P->Counts[0]/(S.Width*S.Height),1000*(Lab.TimeS-P->CaptureS));
            if(Lab.TimeS-P->CaptureS>.5)Status=TEXT("STALE | ")+Status;
        }
        HUD->DrawText(Status,FLinearColor(.9,.9,.9),X+3,Y+H+23,GEngine->GetSmallFont(),.72f);
        if(I==0&&Lab.PalletRecognition)
        {
            const auto& R=*Lab.PalletRecognition;const auto& D=R.Detection;
            HUD->DrawText(FString::Printf(TEXT("Pallet: %s | %s"),*R.Decision.State,R.Tracked?TEXT("bounded track"):TEXT("depth evidence")),FLinearColor(.2,.95,.6),X+3,Y+H+52,GEngine->GetSmallFont(),.65f);
            if(P&&D.Valid&&P->Sequence==D.Sequence&&Lab.TimeS-D.CaptureS<.5)
            {
                const double Fx=S.Width/(2*FMath::Tan(FMath::DegreesToRadians(S.FovDeg)*.5));const FVector N(FMath::Cos(D.Yaw),FMath::Sin(D.Yaw),0),Tangent(-N.Y,N.X,0);
                auto Project=[&](double U,double V){const FVector Optical=CameraMounts::FrontRotation.Quaternion().UnrotateVector(N*D.DistanceM+Tangent*U+FVector(0,0,V));return FVector2D(X+W*(.5+Fx*Optical.Y/Optical.X/S.Width),Y+20+H*(.5-Fx*Optical.Z/Optical.X/S.Height));};
                for(int Hole=0;Hole<2;++Hole){FVector2D Points[4];const double HW=D.HoleWidth[Hole]*.5,HH=R.Spec.OpeningHeight*.5;Points[0]=Project(D.HoleY[Hole]-HW,D.CenterZ-HH);Points[1]=Project(D.HoleY[Hole]+HW,D.CenterZ-HH);Points[2]=Project(D.HoleY[Hole]+HW,D.CenterZ+HH);Points[3]=Project(D.HoleY[Hole]-HW,D.CenterZ+HH);for(int J=0;J<4;++J)HUD->DrawLine(Points[J].X,Points[J].Y,Points[(J+1)%4].X,Points[(J+1)%4].Y,FLinearColor(.1,1,.4),1);}
            }
        }
        HUD->DrawText(FString::Printf(TEXT("%.1f-%.1fm | red near / blue far / purple invalid-near"),S.MinM,S.MaxM),FLinearColor(.6,.7,.8),X+3,Y+H+39,GEngine->GetSmallFont(),.66f);
        Y+=H+73;
    }
}
void FCameraSensorRig::WriteReport(const FString& Folder,double NowS,bool Enabled) const
{
    IFileManager::Get().MakeDirectory(*Folder,true);auto J=Telemetry(NowS,Enabled);bool Pass=true;
    for(auto C:{State->Front,State->Rear})
    {
        const FString Name=C->bRGBD?TEXT("rear"):TEXT("front");
        if(State->bValidation)
        {
            const bool OK=C->ValidChecks>=2 && C->NearChecks>=2 && C->FarChecks>=2 && C->Failures==0;Pass &= OK;
            auto V=MakeShared<FJsonObject>();V->SetBoolField(TEXT("pass"),OK);V->SetNumberField(TEXT("valid_checks"),C->ValidChecks);V->SetNumberField(TEXT("near_checks"),C->NearChecks);V->SetNumberField(TEXT("far_checks"),C->FarChecks);V->SetNumberField(TEXT("failures"),C->Failures);V->SetNumberField(TEXT("max_ideal_depth_error_m"),C->MaxErrorM);J->SetObjectField(Name+TEXT("_validation"),V);
        }
        if(!C->Latest)continue;
        const auto& P=*C->Latest;FString CSV=TEXT("x,y,optical_depth_m,radial_range_m,validity\n");
        for(int Y=0;Y<C->Spec.Height;++Y)for(int X=0;X<C->Spec.Width;++X)
        {
            const int I=Y*C->Spec.Width+X;
            if(P.Validity[I]==0)CSV+=FString::Printf(TEXT("%d,%d,%.6f,%.6f,valid\n"),X,Y,P.DepthM[I],P.RangeM[I]);
            else CSV+=FString::Printf(TEXT("%d,%d,,,%s\n"),X,Y,DepthCamera::Name(static_cast<DepthCamera::EValidity>(P.Validity[I])));
        }
        FFileHelper::SaveStringToFile(CSV,*(Folder/(Name+TEXT("-depth.csv"))),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    }
    FString Text;FJsonSerializer::Serialize(J,TJsonWriterFactory<>::Create(&Text));FFileHelper::SaveStringToFile(Text,*(Folder/TEXT("cameras.json")),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    if(State->bValidation)UE_LOG(LogTemp,Display,TEXT("CAMERA_VALIDATION %s front_checks=%d/%d/%d rear_checks=%d/%d/%d"),Pass?TEXT("PASS"):TEXT("FAIL"),State->Front->ValidChecks,State->Front->NearChecks,State->Front->FarChecks,State->Rear->ValidChecks,State->Rear->NearChecks,State->Rear->FarChecks);
    UE_LOG(LogTemp,Display,TEXT("CAMERA_REPORT %s"),*Folder);
}
