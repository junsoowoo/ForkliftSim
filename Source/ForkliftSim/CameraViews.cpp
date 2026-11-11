#include "StabilityLab.h"
#include "CameraSensors.h"
#include "PalletPerception.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/World.h"

void AStabilityLab::ConfigureCameraViews()
{
    CameraSensors.Reset();
    if(IsValid(FrontViewCapture))FrontViewCapture->DestroyComponent();
    if(IsValid(RearViewCapture))RearViewCapture->DestroyComponent();
    FrontViewCapture=nullptr;RearViewCapture=nullptr;FrontViewTexture=nullptr;RearViewTexture=nullptr;
    FrontDepthImage=nullptr;RearColorImage=nullptr;RearDepthImage=nullptr;
    if(!CameraPostTickHandle.IsValid())
        CameraPostTickHandle=FWorldDelegates::OnWorldPostActorTick.AddWeakLambda(this,[this](UWorld* W,ELevelTick,float){if(W==GetWorld())UpdateCameraViews();});
    if(!bCameraViews || !Chassis || !Fork)return;
    FString CameraError;CameraSensors=FCameraSensorRig::Create(*this,CameraError);
    if(!CameraSensors){bCameraViews=false;UE_LOG(LogTemp,Error,TEXT("CAMERA_CONFIG %s"),*CameraError);}
}
void AStabilityLab::UpdateCameraViews(){if(CameraSensors && bCameraViews)CameraSensors->Update(*this);if(PalletRecognition)PalletRecognition->Update(*this);}
void AStabilityLab::ToggleCameraViews()
{
    bCameraViews=!bCameraViews;
    if(bCameraViews){if(!CameraSensors)ConfigureCameraViews();bHydraulicView=false;}
}
void AStabilityLab::ToggleRearCameraMode(){bRearDepthView=!bRearDepthView;}
void AStabilityLab::DrawCameraViews(AHUD* ViewHUD,UCanvas* ViewCanvas)
{if(CameraSensors && bCameraViews)CameraSensors->Draw(*this,ViewHUD,ViewCanvas);}
void AStabilityLab::ShutdownCameraViews()
{
    if(CameraPostTickHandle.IsValid()){FWorldDelegates::OnWorldPostActorTick.Remove(CameraPostTickHandle);CameraPostTickHandle.Reset();}
    CameraSensors.Reset();
}
