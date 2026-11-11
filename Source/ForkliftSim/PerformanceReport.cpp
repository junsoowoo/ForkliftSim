#include "StabilityLab.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/App.h"
#include "HAL/FileManager.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Components/SceneCaptureComponent2D.h"
#include "ForkActor.h"
#include "ChassisActor.h"
#include "CameraMounts.h"
#include "CameraSensors.h"
void AStabilityLab::WritePerformanceReport()
{
    if(PerformanceFrameMs.IsEmpty())return;
    auto Samples=PerformanceFrameMs;Samples.Sort();double Sum=0;for(double V:Samples)Sum+=V;
    auto J=MakeShared<FJsonObject>();
    J->SetStringField(TEXT("model"),Settings.Mecanum.bReduced?TEXT("reduced"):TEXT("detailed"));
    J->SetNumberField(TEXT("physics_hz"),1./SelectedPhysicsStepS);J->SetNumberField(TEXT("samples"),Samples.Num());
    J->SetBoolField(TEXT("camera_views"),bCameraViews);
    J->SetNumberField(TEXT("camera_hz_each"),bCameraViews?8:0);
    if(CameraSensors)
    {
        auto CameraData=CameraSensors->Telemetry(TimeS,bCameraViews);
        J->SetObjectField(TEXT("camera_sensors"),CameraData);
        J->SetNumberField(TEXT("camera_hz_each"),bCameraViews?CameraData->GetObjectField(TEXT("front"))->GetNumberField(TEXT("target_hz")):0);
        const FString CameraFolder=FPaths::ProjectSavedDir()/TEXT("Sensors")/FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S"));
        CameraSensors->WriteReport(CameraFolder,TimeS,bCameraViews);
    }
    if(bCameraViews && FrontViewCapture && RearViewCapture && Fork && Chassis)
    {
        const FVector FrontOffset=Fork->Carriage->GetComponentQuat().UnrotateVector(FrontViewCapture->GetComponentLocation()-Fork->Carriage->GetComponentLocation());
        const FVector RearOffset=Chassis->Body->GetComponentQuat().UnrotateVector(RearViewCapture->GetComponentLocation()-Chassis->Body->GetComponentLocation());
        J->SetNumberField(TEXT("front_mount_error_cm"),(FrontOffset-CameraMounts::FrontCm).Size());
        J->SetNumberField(TEXT("rear_mount_error_cm"),(RearOffset-CameraMounts::RearCm).Size());
        J->SetNumberField(TEXT("fork_height_m"),ForkHeightM);
        J->SetNumberField(TEXT("front_camera_world_height_m"),FrontViewCapture->GetComponentLocation().Z/100.);
        J->SetNumberField(TEXT("rear_camera_world_height_m"),RearViewCapture->GetComponentLocation().Z/100.);
    }
    double TailSum=0;const int TailCount=FMath::Min(120,PerformanceFrameMs.Num());
    for(int I=PerformanceFrameMs.Num()-TailCount;I<PerformanceFrameMs.Num();++I)TailSum+=PerformanceFrameMs[I];
    J->SetNumberField(TEXT("tail_120_mean_fps"),1000*TailCount/TailSum);
    J->SetNumberField(TEXT("mean_frame_ms"),Sum/Samples.Num());J->SetNumberField(TEXT("mean_fps"),1000*Samples.Num()/Sum);
    J->SetNumberField(TEXT("median_frame_ms"),Samples[Samples.Num()/2]);J->SetNumberField(TEXT("p95_frame_ms"),Samples[FMath::Min(Samples.Num()-1,FMath::FloorToInt(Samples.Num()*.95))]);
    J->SetNumberField(TEXT("actual_realtime_ratio"),RealtimeRatio);J->SetBoolField(TEXT("benchmark"),FApp::IsBenchmarking());
    J->SetStringField(TEXT("scope"),TEXT("World game-tick wall intervals after first simulation second; not GPU timing or a long-running editor PIE guarantee."));
    FString Text;FJsonSerializer::Serialize(J,TJsonWriterFactory<>::Create(&Text));
    const FString Folder=FPaths::ProjectSavedDir()/TEXT("Validation/Performance");IFileManager::Get().MakeDirectory(*Folder,true);
    const FString File=Folder/FString::Printf(TEXT("Runtime-%s-%dHz-%s.json"),Settings.Mecanum.bReduced?TEXT("reduced"):TEXT("detailed"),FMath::RoundToInt(1./SelectedPhysicsStepS),*FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")));
    FFileHelper::SaveStringToFile(Text,*File,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    UE_LOG(LogTemp,Display,TEXT("PERFORMANCE_REPORT mean_fps=%.2f p95_ms=%.2f realtime=%.3f file=%s"),1000*Samples.Num()/Sum,Samples[FMath::Min(Samples.Num()-1,FMath::FloorToInt(Samples.Num()*.95))],RealtimeRatio,*File);
}
