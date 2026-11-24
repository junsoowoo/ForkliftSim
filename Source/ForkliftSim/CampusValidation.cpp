#include "CampusOperations.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "WheelActor.h"
#include "LidarSafety.h"
#include "ProximitySensors.h"
#include "PalletPerception.h"
#include "CameraSensors.h"
#include "SensorLiftControl.h"
#include "JointSensors.h"
#include "Engine/StaticMeshActor.h"
#include "UnrealClient.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "Serialization/JsonSerializer.h"

void ACampusOperations::FinishValidation(bool Pass,const FString& Reason)
{
    auto J=MakeShared<FJsonObject>();J->SetBoolField(TEXT("pass"),Pass);J->SetStringField(TEXT("reason"),Reason);J->SetStringField(TEXT("case"),TestCase);J->SetNumberField(TEXT("entry_yaw_deg"),EntryYaw);J->SetNumberField(TEXT("sim_s"),Age);
    J->SetNumberField(TEXT("vehicles"),Vehicles.Num());J->SetNumberField(TEXT("registered_loads"),Inventory.Loads.Num());J->SetNumberField(TEXT("physical_loads"),PhysicalLoads.Num());J->SetNumberField(TEXT("slots"),Inventory.Slots.Num());J->SetNumberField(TEXT("idle_peer_drift_m"),IdleDrift);J->SetBoolField(TEXT("saw_vision"),SawVision);J->SetBoolField(TEXT("saw_full_insertion"),SawFull);J->SetBoolField(TEXT("saw_lift"),SawLift);J->SetBoolField(TEXT("saw_physical_vehicle_support"),SawVehicleSupport);J->SetBoolField(TEXT("peer_physics_ray_hit"),SawPeer);J->SetNumberField(TEXT("max_fork_load_n"),MaxForkLoad);J->SetNumberField(TEXT("max_wheel_load_n"),MaxWheelLoad);J->SetNumberField(TEXT("final_pallet_rise_m"),FinalRise);J->SetNumberField(TEXT("truth_depth_m"),TestDepth);
    J->SetStringField(TEXT("scope"),TEXT("Native physical campus foundation. Scripted operator test inputs and truth-only assertions; not autonomous navigation/dispatch."));
    TArray<TSharedPtr<FJsonValue>> List;for(AStabilityLab* V:Vehicles)
    {auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("id"),V->VehicleId);O->SetStringField(TEXT("error"),V->Error);O->SetBoolField(TEXT("drive_ready"),V->DriveSensors&&V->DriveSensors->Model.Ready);O->SetBoolField(TEXT("localized"),V->DriveSensors&&V->DriveSensors->Model.Localized&&V->TimeS-V->DriveSensors->Model.LastMatchS<.75);
    if(V->DriveSensors){const auto& M=V->DriveSensors->Model;O->SetNumberField(TEXT("match_points"),M.Registration.Points);O->SetNumberField(TEXT("match_rms_m"),M.Registration.Rms);O->SetNumberField(TEXT("match_age_s"),V->TimeS-M.LastMatchS);}if(V->CameraSensors)O->SetObjectField(TEXT("cameras"),V->CameraSensors->Telemetry(V->TimeS,V->bCameraViews));if(V->SensorLift)O->SetObjectField(TEXT("lift_control"),V->SensorLift->Telemetry());O->SetNumberField(TEXT("fps"),V->GameFPS());O->SetNumberField(TEXT("sim_wall_ratio"),V->ProgressRatio());if(V->Chassis){const auto P=V->Chassis->GetActorLocation()/100.;O->SetNumberField(TEXT("x_m"),P.X);O->SetNumberField(TEXT("y_m"),P.Y);}List.Add(MakeShared<FJsonValueObject>(O));}J->SetArrayField(TEXT("vehicles_state"),List);
    if(auto* V=Active();V&&V->PalletRecognition&&!V->PalletRecognition->LastFrame.DepthM.IsEmpty())
    {
        const auto& F=V->PalletRecognition->LastFrame;const auto D=PalletPerception::Detect(F,V->PalletRecognition->Spec);auto Debug=MakeShared<FJsonObject>();
        Debug->SetNumberField(TEXT("planes"),D.PlaneCandidates);Debug->SetNumberField(TEXT("support"),D.SupportPass);Debug->SetNumberField(TEXT("roof"),D.RoofPass);Debug->SetNumberField(TEXT("gap"),D.GapPass);Debug->SetNumberField(TEXT("edge"),D.EdgePass);Debug->SetBoolField(TEXT("valid"),D.Valid);J->SetObjectField(TEXT("last_vision_diagnostic"),Debug);
        TArray<uint8> Bytes;Bytes.Append(reinterpret_cast<const uint8*>(F.DepthM.GetData()),F.DepthM.Num()*sizeof(float));Bytes.Append(F.Validity);const FString Folder=FPaths::ProjectSavedDir()/TEXT("Validation/Campus");IFileManager::Get().MakeDirectory(*Folder,true);FFileHelper::SaveArrayToFile(Bytes,*(Folder/TEXT("MeasuredFront.bin")));
    }
    FString Text;FJsonSerializer::Serialize(J,TJsonWriterFactory<>::Create(&Text));const FString D=FPaths::ProjectSavedDir()/TEXT("Validation/Campus");IFileManager::Get().MakeDirectory(*D,true);FFileHelper::SaveStringToFile(Text,*(D/FString::Printf(TEXT("Foundation-%s-%d.json"),*TestCase,EntryYaw)),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    UE_LOG(LogTemp,Display,TEXT("CAMPUS_FOUNDATION %s case=%s yaw=%d sim=%.2f reason=%s fork_n=%.1f idle_m=%.4f vision=%d full=%d"),Pass?TEXT("PASS"):TEXT("FAIL"),*TestCase,EntryYaw,Age,*Reason,MaxForkLoad,IdleDrift,SawVision,SawFull);SetActorTickEnabled(false);FGenericPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
}
void ACampusOperations::ValidateStep(float Dt)
{
    if(Vehicles.Num()!=3)return;for(AStabilityLab* V:Vehicles)if(!V->Error.IsEmpty()){FinishValidation(false,V->VehicleId+TEXT(": ")+V->Error);return;}
    auto* V=Active();if(!V||!V->Chassis||!V->Pallet)return;
    if(TestStage!=LastLoggedStage){UE_LOG(LogTemp,Display,TEXT("CAMPUS_STAGE stage=%.0f sim=%.2f depth=%.3f height=%.3f"),TestStage,Age,TestDepth,V->ForkHeightM);LastLoggedStage=TestStage;}
    for(int I=1;I<Vehicles.Num();++I){auto* B=Vehicles[I].Get();B->Throttle=B->Steering=B->Strafe=B->Lift=0;B->Brake=1;if(Age>2)IdleDrift=FMath::Max(IdleDrift,FVector2D(B->Chassis->GetActorLocation()-IdleStart[I]).Size()/100.);}
    if(Age>90){FinishValidation(false,TEXT("simulation timeout"));return;}
    if(V->PalletRecognition){SawVision|=V->PalletRecognition->Detection.Valid;SawFull|=V->PalletRecognition->Decision.ConfirmedFull;}
    MaxForkLoad=FMath::Max(MaxForkLoad,V->PayloadState.FilteredN[0]+V->PayloadState.FilteredN[1]);MaxWheelLoad=FMath::Max(MaxWheelLoad,V->TotalNormalLoadN);
    FinalRise=V->PayloadState.HeightM;SawLift|=FinalRise>.25;SawVehicleSupport|=V->Pallet->HasRecentVehicleSupport();TestDepth=FMath::Min(V->PayloadState.DepthM[0],V->PayloadState.DepthM[1]);
    if(Age>2&&!SawPeer)
    {
        const auto Bodies=ProximitySensor::Targets(*V);ProximitySensor::FRay Ray;Ray.OriginM=Vehicles[1]->Chassis->GetActorLocation()/100.;Ray.Direction=(Vehicles[2]->Chassis->GetActorLocation()/100.-Ray.OriginM).GetSafeNormal();
        Ray.OriginM+=Ray.Direction*1.2;const auto R=ProximitySensor::Trace(Bodies,Ray,8);SawPeer=R.Hit&&R.RangeM>3&&R.RangeM<5.5;
    }
    if(Age>2&&Age-Dt<=2)FScreenshotRequest::RequestScreenshot(FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir()/TEXT("Validation/Campus/FoundationView.png")),true,false);
    if(TestCase==TEXT("inventory"))
    {
        const FString A=TEXT("S01-B1-S-L2-1"),B=TEXT("S01-B2-S-L2-1");auto* P=V->Pallet.Get();
        if(TestStage==0&&Age>3)
        {
            if(Inventory.Slots[A].Pallet!=TEXT("PICK-01")||!P->HasRecentRackSupport()){FinishValidation(false,TEXT("physical shelf support was not confirmed"));return;}
            const FVector Bottom=Inventory.Slots[B].CenterM+FVector(0,0,.015);P->Deck->SetEnableGravity(false);P->Deck->SetPhysicsLinearVelocity(FVector::ZeroVector);P->Deck->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector);P->SetActorLocationAndRotation((Bottom+PayloadSpec.PalletDatumM)*100,FRotator::ZeroRotator,false,nullptr,ETeleportType::TeleportPhysics);
            auto* Cargo=P->Cargo->GetStaticMeshComponent();Cargo->SetEnableGravity(false);Cargo->SetPhysicsLinearVelocity(FVector::ZeroVector);Cargo->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector);Cargo->SetWorldLocation((Bottom+FVector(0,0,.502))*100,false,nullptr,ETeleportType::TeleportPhysics);P->LastRackSupportS=-1;TestStage=1;StageStarted=Age;
        }
        if(TestStage==1&&Age-StageStarted>1)
        {const bool Pass=Inventory.Slots[A].Pallet.IsEmpty()&&Inventory.Slots[B].Pallet.IsEmpty()&&Inventory.Loads[TEXT("PICK-01")].Slot.IsEmpty()&&!P->HasRecentRackSupport();FinishValidation(Pass,Pass?TEXT("shelf contact confirms placement; pickup clears source; floating geometry cannot claim slot"):TEXT("inventory claimed unsupported geometry"));}
        return;
    }
    if(TestCase==TEXT("rackvision"))
    {
        V->LiftTargetM=1.95;V->Lift=0;V->Throttle=V->Strafe=V->Steering=0;V->Brake=1;
        if(V->JointSensors&&V->PalletRecognition&&V->ForkHeightM>1.91&&V->ForkHeightM<1.99&&Age>4)
        {const auto& D=V->PalletRecognition->Detection;if(D.Valid&&V->TimeS-D.CaptureS<.6&&FMath::Abs(D.CenterZ+.24)<.06){FinishValidation(true,TEXT("fresh measured upper-tier pallet recognition at 1.95 m fork height"));return;}}
        if(Age>40)FinishValidation(false,TEXT("upper-tier measured detection timeout"));return;
    }
    if(TestCase==TEXT("vision")){if(Age>3)FinishValidation(SawVision,SawVision?TEXT("measured front-depth recognition"):TEXT("no valid front-depth detection"));return;}
    if(TestCase==TEXT("smoke"))
    {
        if(Age<5)return;bool Ready=SawPeer&&IdleDrift<.08;TSet<AActor*> Parts;
        for(AStabilityLab* B:Vehicles){DepthCamera::FFrame F;Ready&=B->CameraSensors&&B->CameraSensors->ReadFront(F,0)&&B->TimeS-F.CaptureS<.5;Ready&=B->DriveSensors&&B->DriveSensors->Model.Ready&&B->DriveSensors->Model.Localized&&B->TimeS-B->DriveSensors->Model.LastMatchS<.75&&B->VehicleGuard&&B->PalletRecognition&&B->CameraSensors;Ready&=B->Chassis->GetOwner()==B&&B->Wheels.Num()==4&&!Parts.Contains(B->Chassis);Parts.Add(B->Chassis);}
        const auto Bodies=ProximitySensor::Targets(*V);ProximitySensor::FRay Ray;Ray.OriginM=FVector(24,10.6,.25);Ray.Direction=FVector(1,0,0);const auto Hit=ProximitySensor::Trace(Bodies,Ray,3);Ready&=Hit.Hit&&FMath::Abs(Hit.RangeM-1.94)<.03;
        Ray.OriginM=FVector(27.2,10.6,1);const auto Gap=ProximitySensor::Trace(Bodies,Ray,.5);Ready&=!Gap.Hit;
        // Promotion must remove all static render + obstacle entries for a stock ID.
        const FString Stock=LoadOrder[3];Ready&=SelectLoad(Stock)&&PhysicalLoads.Contains(Stock)&&Inventory.Loads[Stock].Physical;
        FinishValidation(Ready,Ready?TEXT("ownership, sensor readiness, peer ray, instanced rack hit/gap, stock promotion"):TEXT("foundation condition failed"));return;
    }
    if(SawFull&&TestDepth<.95&&!SawLift){FinishValidation(false,TEXT("full insertion claim precedes physical insertion"));return;}
    if(TestCase!=TEXT("handling")){FinishValidation(false,TEXT("unknown validation case"));return;}
    V->Throttle=V->Strafe=V->Steering=V->Lift=0;V->Brake=1;V->bCampusTestCreep=true;
    if(TestStage==0)
    {
        // Scripted operator aligns the blade height from the real depth view
        // before driving into the channel. Do not lift an unconfirmed insertion.
        if(Age>2&&V->DriveSensors&&V->DriveSensors->Model.Ready&&V->PalletRecognition&&V->JointSensors)
        {
            const auto& P=*V->PalletRecognition;
            if(P.Detection.Valid&&P.Detection.Sequence==P.LastSequence&&V->TimeS-P.Detection.CaptureS<.35)
            {
                const double H=V->JointSensors->Model.Channels[4].Last.Value;
                V->LiftTargetM=FMath::Clamp(H+P.OffsetZ,0.,.08);
                if(FMath::Abs(P.OffsetZ)<.008&&P.Aligned&&P.HaveAnchor&&P.Detection.DistanceM<1.8){auto& Track=*V->PalletRecognition;Track.Locked=true;Track.LockedPoint=Track.AnchorPlanePoint;Track.LockedYaw=Track.AnchorYaw;TestStage=1;StageStarted=Age;}
            }
        }
        if(Age>25)FinishValidation(false,TEXT("measured operator height alignment timeout"));return;
    }
    if(TestStage==1)
    {
        V->Throttle=1;V->Brake=0;
        if(TestDepth>=1.07){TestStage=2;StageStarted=Age;}
        if(Age-StageStarted>30){FinishValidation(false,TEXT("insertion timeout"));return;}
    }
    else if(TestStage==2)
    {
        if(Age-StageStarted>.7&&V->PalletRecognition&&V->PalletRecognition->Decision.ConfirmedFull){V->PalletRecognition->Locked=false;V->LiftTargetM=.4;V->Lift=0;}
        if(V->ForkHeightM>.38&&V->JointSensors&&FMath::Abs(V->JointSensors->Model.Channels[4].Last.Value-.4)<.02&&Age-StageStarted>3){TestStage=3;StageStarted=Age;}
        if(Age-StageStarted>15){FinishValidation(false,TEXT("lift timeout"));return;}
    }
    else if(TestStage==3)
    {if(Age-StageStarted<2){V->Throttle=.8;V->Brake=0;}else if(Age-StageStarted>3){TestStage=4;StageStarted=Age;}}
    else if(TestStage==4)
    {
        V->LiftTargetM=0;V->Lift=0;if(V->SensorLift&&V->SensorLift->Model.State==TEXT("lower_limit")&&V->ForkHeightM<.02&&FMath::Abs(FinalRise)<.03&&Age-StageStarted>1){TestStage=5;StageStarted=Age;}
        if(Age-StageStarted>30){FinishValidation(false,TEXT("lowering timeout"));return;}
    }
    else if(TestStage==5)
    {
        V->Throttle=-1;V->Brake=0;if(TestDepth<.01&&Age-StageStarted>1)
        {const bool Pass=SawVision&&SawFull&&SawLift&&SawVehicleSupport&&MaxForkLoad>2500&&IdleDrift<.08&&FMath::Abs(FinalRise)<.05;FinishValidation(Pass,Pass?TEXT("four-way physical insertion, measured full evidence, lift, carry, lower and withdraw"):TEXT("handling evidence incomplete"));}
        else if(Age-StageStarted>30)FinishValidation(false,TEXT("withdrawal timeout"));
    }
}
