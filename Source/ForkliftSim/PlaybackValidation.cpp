#include "PlaybackValidation.h"
#include "SimulationPlayback.h"
#include "CampusOperations.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "Components/InputComponent.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/WorldSettings.h"
#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "Serialization/JsonSerializer.h"
APlaybackValidation::APlaybackValidation(){PrimaryActorTick.bCanEverTick=true;PrimaryActorTick.TickGroup=TG_PostPhysics;}
void APlaybackValidation::Tick(float Dt)
{
 Super::Tick(Dt);if(Done)return;if(OriginalRenderLimit<0)OriginalRenderLimit=SimulationPlayback::RenderLimit();Age+=Dt;MaxDt=FMath::Max(MaxDt,double(Dt));if(StartWall==0)StartWall=FPlatformTime::Seconds();
 auto* O=ACampusOperations::Find(GetWorld());if(!O||!O->Active())return;auto* V=O->Active();const bool Keys=FParse::Param(FCommandLine::Get(),TEXT("PlaybackKeys"));
 const double R=GetWorld()->GetWorldSettings()->TimeDilation;RateMask|=R==1?1:R==2?2:R==4?4:0;
 auto Finish=[&](bool Pass,const FString& Why){Done=true;auto J=MakeShared<FJsonObject>();J->SetBoolField(TEXT("pass"),Pass);J->SetStringField(TEXT("reason"),Why);J->SetStringField(TEXT("source"),TEXT("native_playback_v1"));J->SetNumberField(TEXT("requested_rate"),R);J->SetNumberField(TEXT("rate_mask"),RateMask);J->SetNumberField(TEXT("sim_s"),Age);J->SetNumberField(TEXT("observed_wall_s"),FPlatformTime::Seconds()-StartWall);J->SetNumberField(TEXT("max_world_step_s"),MaxDt);J->SetNumberField(TEXT("max_clock_gap_s"),MaxGap);J->SetNumberField(TEXT("actual_ratio_session"),V->ProgressRatio());J->SetObjectField(TEXT("frame"),V->BuildWebFrame());FString Text;FJsonSerializer::Serialize(J,TJsonWriterFactory<>::Create(&Text));const FString Folder=FPaths::ProjectSavedDir()/TEXT("Validation/Playback");IFileManager::Get().MakeDirectory(*Folder,true);FString Name;FParse::Value(FCommandLine::Get(),TEXT("PlaybackCase="),Name);FFileHelper::SaveStringToFile(Text,*(Folder/(Name+TEXT(".json"))),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);UE_LOG(LogTemp,Display,TEXT("PLAYBACK_TEST %s case=%s reason=%s"),Pass?TEXT("PASS"):TEXT("FAIL"),*Name,*Why);FGenericPlatformMisc::RequestExitWithStatus(false,Pass?0:1);};
 auto Press=[&](FKey Key){auto* PC=GetWorld()->GetFirstPlayerController();if(!PC||!PC->InputComponent)return false;for(const auto& B:PC->InputComponent->KeyBindings)if(B.Chord.Key==Key&&B.KeyEvent==IE_Pressed){B.KeyDelegate.Execute(Key);return true;}return false;};
 for(AStabilityLab* B:O->Vehicles){MaxGap=FMath::Max(MaxGap,FMath::Abs(B->TimeS-B->ValidationState.PhysicsSampleS));if(FMath::Abs(B->TimeS-O->Age)>.001||B->ValidationState.MaxInnerStepS>1./480.+1e-6){Finish(false,TEXT("world_vehicle_or_physics_clock_mismatch"));return;}}
 if(MaxDt>1./30.+1e-6){Finish(false,TEXT("control_step_cap_exceeded"));return;}
 if(Keys&&Age>2.1&&Stage==0){if(!Press(EKeys::F6)){Finish(false,TEXT("missing_F6_binding"));return;}Start=V->Chassis->GetActorLocation();O->QueueSelected();Stage=1;}
 if(Keys&&Age>4&&Stage==1){if(!Press(EKeys::F7)){Finish(false,TEXT("missing_F7_binding"));return;}Stage=2;}
 if(Keys&&Age>6&&Stage==2){if(!Press(EKeys::F5)){Finish(false,TEXT("missing_F5_binding"));return;}Stage=3;}
 if(Age>(Keys?9:6))
 {
  bool Healthy=MaxGap<.05&&(R<4||SimulationPlayback::RenderLimit()==0||SimulationPlayback::RenderLimit()>=120);for(AStabilityLab* B:O->Vehicles)Healthy&=B->DriveSensors&&B->DriveSensors->Model.Ready&&!B->DriveSensors->Model.Faulted&&B->DriveSensors->Model.Localized;
  if(Keys){const auto* J=O->Job(V->VehicleId);Healthy&=FMath::IsNearlyEqual(SimulationPlayback::RenderLimit(),OriginalRenderLimit,.01)&&RateMask==7&&J&&J->Engaged&&!J->Held()&&(V->Chassis->GetActorLocation()-Start).Size()>15;}
  Finish(Healthy,Keys?TEXT("real input bindings 2x/4x/1x during sensed physical job, clocks and safety intact"):TEXT("rendered native world/vehicle/physics clock and sensor freshness remain bounded at requested speed"));
 }
}
