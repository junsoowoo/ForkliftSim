#include "PalletPerception.h"
#include "StabilityLab.h"
#include "PalletLoadActor.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/World.h"
bool AStabilityLab::IsPalletPerceptionTest() const{return TestMode.StartsWith(TEXT("mecanum_pallet_vision_"));}
void AStabilityLab::ControlPalletPerceptionTest()
{
    Throttle=Strafe=Steering=Lift=0;Brake=1;LiftTargetM=TestMode.EndsWith(TEXT("high"))?1.835:.035;
    if(!PalletRecognition)return;
    if(!PalletTestConfigured&&TestMode.EndsWith(TEXT("box")))
    {
        PalletTestConfigured=true;auto* A=GetWorld()->SpawnActor<AStaticMeshActor>(FVector(Fork->Tines[0]->Tip().X+85,0,30),FRotator::ZeroRotator);PalletVisionDistractor=A;auto* B=A->GetStaticMeshComponent();B->SetMobility(EComponentMobility::Movable);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));B->SetWorldScale3D(FVector(1.2,1,.6));B->SetCollisionProfileName(TEXT("BlockAll"));
    }
    if(!PalletTestConfigured&&Pallet)
    {
        PalletTestConfigured=true;
        if(TestMode.EndsWith(TEXT("yaw"))||TestMode.EndsWith(TEXT("shift"))||TestMode.EndsWith(TEXT("high")))
        {
            const FVector Before=Pallet->GetActorLocation();
            if(TestMode.EndsWith(TEXT("high")))
            {
                auto* A=GetWorld()->SpawnActor<AStaticMeshActor>(Before-FVector(0,0,PayloadSettings.PalletDatumM.Z*100+5),FRotator::ZeroRotator);PalletVisionDistractor=A;auto* B=A->GetStaticMeshComponent();B->SetMobility(EComponentMobility::Movable);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));B->SetWorldScale3D(FVector(1.5,1.3,.1));B->SetCollisionProfileName(TEXT("BlockAll"));
            }
        }
    }
    if(TestMode.EndsWith(TEXT("insert"))&&TimeS>=2&&TimeS<12){Brake=0;Throttle=1;}
    if(TestMode.EndsWith(TEXT("disabled"))&&TimeS>5)bCameraViews=false;
    const auto& R=*PalletRecognition;
    if(R.Detection.Valid&&TimeS-R.Detection.CaptureS<.5&&R.Detection.Sequence==R.LastSequence)
    {
        PalletTestSawDetection=true;++PalletTestFrames;
        if(Pallet&&!TestMode.EndsWith(TEXT("insert")))
        {
            // Diagnostics only. Detection and insertion never receive actor poses.
            const FTransform CameraPose=FrontViewCapture?FrontViewCapture->GetComponentTransform():FTransform::Identity;
            const FVector Center=Pallet->GetActorLocation()/100.;const FVector Face=Center-Pallet->GetActorForwardVector()*PayloadSettings.LengthM*.5;
            const FVector Origin=CameraPose.GetLocation()/100.;const FVector Level=Fork->Carriage->GetComponentQuat().UnrotateVector(Face-Origin);
            const double Distance=FMath::Cos(R.Detection.Yaw)*Level.X+FMath::Sin(R.Detection.Yaw)*Level.Y;PalletTestMaxDistanceError=FMath::Max(PalletTestMaxDistanceError,FMath::Abs(R.Detection.DistanceM-Distance));
        }
    }
    PalletTestSawPartial|=R.Decision.State==TEXT("partial");PalletTestSawFull|=R.Decision.ConfirmedFull;
    if(TestMode.EndsWith(TEXT("insert"))&&R.Decision.ConfirmedFull&&Pallet)
    {
        const FVector N=Pallet->GetActorForwardVector(),Face=Pallet->GetActorLocation()/100.-N*PayloadSettings.LengthM*.5;double MinDepth=100;
        for(AForkTineActor* T:Fork->Tines)MinDepth=FMath::Min(MinDepth,FVector::DotProduct(T->Tip()/100.-Face,N));
        PalletTestMaxDistanceError=FMath::Max(PalletTestMaxDistanceError,FMath::Max(0.,R.Spec.FullDepthM-.03-MinDepth));
    }
}
bool AStabilityLab::CheckPalletPerceptionTest() const
{
    if(!PalletRecognition)return false;const bool Negative=TestMode.EndsWith(TEXT("box"));
    bool Pass=Negative?!PalletTestSawDetection:PalletTestSawDetection&&PalletTestFrames>=3&&PalletTestMaxDistanceError<.08;
    if(TestMode.EndsWith(TEXT("insert")))Pass&=PalletTestSawPartial&&PalletTestSawFull&&PalletTestMaxDistanceError<1e-5;
    if(TestMode.EndsWith(TEXT("yaw"))||TestMode.EndsWith(TEXT("shift")))Pass&=PalletRecognition->Decision.State==TEXT("misaligned");
    if(TestMode.EndsWith(TEXT("disabled")))Pass&=PalletRecognition->Decision.State==TEXT("unknown");
    if(TestMode.EndsWith(TEXT("high")))Pass&=Pallet&&Pallet->GetActorLocation().Z>170&&ForkHeightM>1.7&&TimeS-PalletRecognition->Detection.CaptureS<.5&&PalletRecognition->Detection.Valid;
    if(TestMode.EndsWith(TEXT("yaw")))Pass&=Pallet&&FMath::Abs(Pallet->GetActorRotation().Yaw)>8;
    if(TestMode.EndsWith(TEXT("shift")))Pass&=Pallet&&Pallet->GetActorLocation().Y>12;
    UE_LOG(LogTemp,Display,TEXT("PALLET_PERCEPTION_CHECK %s detected=%d frames=%d partial=%d full=%d max_distance_error_m=%.6f final_state=%s %s"),*TestMode,PalletTestSawDetection,PalletTestFrames,PalletTestSawPartial,PalletTestSawFull,PalletTestMaxDistanceError,*PalletRecognition->Decision.State,Pass?TEXT("PASS"):TEXT("FAIL"));return Pass;
}
