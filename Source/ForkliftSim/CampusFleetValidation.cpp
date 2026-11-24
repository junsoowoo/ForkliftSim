#include "CampusOperations.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "HandlingControl.h"
#include "LidarSafety.h"
#include "DynamicsSensors.h"
#include "PalletPerception.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "Serialization/JsonSerializer.h"
#include "Engine/GameViewportClient.h"
#include "Engine/StaticMeshActor.h"
void ACampusOperations::FleetValidationStep(float Dt)
{
    if(TestCase==TEXT("board_playback"))return;
    if(TestCase.StartsWith(TEXT("ops_"))){OperationsValidation(Dt);return;}
    if(TestCase.StartsWith(TEXT("board_"))){TaskValidationStep(Dt);return;}
    if(TestStage==99)return;auto* V=Active();if(!V||!V->DriveSensors)return;
    auto Finish=[&](bool Pass,const FString& Why)
    {
        auto J=FleetTelemetry();J->SetBoolField(TEXT("pass"),Pass);J->SetStringField(TEXT("case"),TestCase);J->SetStringField(TEXT("reason"),Why);J->SetNumberField(TEXT("sim_s"),Age);J->SetNumberField(TEXT("observed_game_fps"),V->GameFPS());J->SetNumberField(TEXT("sim_wall_ratio"),V->ProgressRatio());J->SetNumberField(TEXT("max_fork_load_n"),MaxForkLoad);J->SetNumberField(TEXT("braking_displacement_m"),TestDepth);if(TestMinClearance<1e8)J->SetNumberField(TEXT("minimum_obstacle_clearance_m"),TestMinClearance);J->SetNumberField(TEXT("max_vehicle_tilt_deg"),MaxWheelLoad);J->SetBoolField(TEXT("saw_measured_vision"),SawVision);J->SetBoolField(TEXT("saw_measured_full"),SawFull);J->SetStringField(TEXT("scope"),TEXT("Actual native campus with measured GPU camera, encoder/IMU/LiDAR and torque/valve control. Authored station fixtures only at initialization. Not OEM qualification or six-vehicle throughput."));
        J->SetNumberField(TEXT("peak_loaded_transit_mps"),TestPeakLoadedMps);
        J->SetNumberField(TEXT("peak_empty_transit_mps"),TestPeakEmptyMps);
        J->SetNumberField(TEXT("obstacle_entry_mps"),TestObstacleEntryMps);
        FString Text;FJsonSerializer::Serialize(J,TJsonWriterFactory<>::Create(&Text));const FString Folder=FPaths::ProjectSavedDir()/TEXT("Validation/Campus");IFileManager::Get().MakeDirectory(*Folder,true);FFileHelper::SaveStringToFile(Text,*(Folder/(TEXT("Fleet-")+TestCase+TEXT(".json"))),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
        UE_LOG(LogTemp,Display,TEXT("CAMPUS_FLEET %s case=%s sim_s=%.2f reason=%s notice=%s"),Pass?TEXT("PASS"):TEXT("FAIL"),*TestCase,Age,*Why,*Notice);TestStage=99;FGenericPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
    };
    if(V->PalletRecognition){SawVision|=V->PalletRecognition->Detection.Valid;SawFull|=V->PalletRecognition->Decision.ConfirmedFull;}MaxForkLoad=FMath::Max(MaxForkLoad,V->PayloadState.FilteredN[0]+V->PayloadState.FilteredN[1]);MaxWheelLoad=FMath::Max(MaxWheelLoad,V->TiltDeg);
    if(TestCase==TEXT("performance")){if(Age>12)Finish(true,TEXT("idle 3-vehicle rendered measurement; report FPS and sim/wall separately"));return;}
    if(TestStage==0&&Age>3)
    {
        if(Jobs.IsEmpty()){if(TestCase==TEXT("rack_transfer")){if(SelectLoad(TEXT("LOAD-S01-B1-S-L1-1")))QueueSelected();}else if(TestCase==TEXT("transfer")||TestCase==TEXT("upper")||TestCase==TEXT("rack_transfer")||TestCase==TEXT("full_transfer"))QueueSelected();else QueueAll();}
        if(!Jobs.IsEmpty()){StartPosition=V->Chassis->GetActorLocation();TestStage=.5;StageStarted=Age;}
        if(Age>12){Finish(false,TEXT("queue_not_accepted"));return;}
    }
    auto* J=Job(V->VehicleId);if(!J)return;
    const double BodySpeedMps=FVector2D(V->SpeedMps,V->LateralMps).Size();
    if(J->Travelling())
    {
        double& Peak=J->Carrying?TestPeakLoadedMps:TestPeakEmptyMps;
        Peak=FMath::Max(Peak,BodySpeedMps);
    }
    if(!SawPeer&&J->State==TEXT("dropoff")&&V->HandlingControl&&V->HandlingControl->Model.State==TEXT("place_lower")){SawPeer=true;FScreenshotRequest::RequestScreenshot(FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir()/TEXT("Validation/Campus/FleetShelf.png")),true,false);}
    const auto P=V->DriveSensors->Model.MapPose(V->TimeS);const double Distance=(V->Chassis->GetActorLocation()-StartPosition).Size()/100.;
    if(TestCase==TEXT("dispatch")||TestCase==TEXT("gate"))
    {
        if(J->Held()){Finish(false,TEXT("dispatch held: ")+J->Reason);return;}
        if(TestCase==TEXT("gate")&&P.X>2){const auto Actual=V->Chassis->GetActorLocation()/100.;Finish(J->Travelling()&&FVector2D(P.X-Actual.X,P.Y-Actual.Y).Size()<.15,TEXT("measured route from outdoor bay through actual warehouse portal; estimated pose checked against truth only for qualification"));return;}
        if(TestCase==TEXT("dispatch")&&Distance>3&&J->Travelling())
        {bool Wait=false;for(const auto& Pair:Jobs)Wait|=Pair.Value.State==TEXT("queued")&&Pair.Value.Reason.StartsWith(TEXT("traffic_wait:"));Finish(Jobs.Num()==3&&Traffic.Claims.Num()==1&&Wait&&V->DriveSensors->Model.Localized&&Age-V->DriveSensors->Model.LastMatchS<1.5,TEXT("three real jobs; exclusive intersecting corridor and measured path progress"));return;}
    }
    else if(TestCase==TEXT("detour"))
    {
        if(TestStage==.5&&J->Travelling())
        {TestStage=1;StageStarted=Age;const auto Here=FVector2D(P.X,P.Y);const auto Direction=(J->ToPick.Points[J->Waypoint]-Here).GetSafeNormal();const auto At=Here+Direction*8;V->GuardObstacle=GetWorld()->SpawnActor<AStaticMeshActor>(FVector(At.X*100,At.Y*100,25),FRotator::ZeroRotator);auto* B=V->GuardObstacle->GetStaticMeshComponent();B->SetMobility(EComponentMobility::Movable);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));B->SetWorldScale3D(FVector(.8,1,.5));B->SetCollisionProfileName(TEXT("BlockAll"));}
        if(J->Held()){Finish(false,TEXT("detour held: ")+J->Reason);return;}
        if(J->Replans>0&&Distance>3){Finish(J->Travelling()&&J->ToPick.Points.Num()>2,TEXT("actual sensed obstacle causes local full-footprint detour before protective field"));return;}
        if(Age>45){Finish(false,TEXT("no proactive sensed detour"));return;}
    }
    else if(TestCase==TEXT("obstacle")||TestCase==TEXT("resume"))
    {
        if(TestStage==.5&&J->Travelling()&&BodySpeedMps>=.85&&V->SpeedMps>=.8)
        {
            TestObstacleEntryMps=BodySpeedMps;
            TestStage=1;StageStarted=Age;StartPosition=V->Chassis->GetActorLocation();TestDepth=0;FinalRise=0;V->GuardObstacle=GetWorld()->SpawnActor<AStaticMeshActor>(V->Chassis->GetActorLocation()+FVector(355,0,-24),FRotator::ZeroRotator);auto* B=V->GuardObstacle->GetStaticMeshComponent();B->SetMobility(EComponentMobility::Movable);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));B->SetWorldScale3D(FVector(.3,1.2,.5));B->SetCollisionProfileName(TEXT("BlockAll"));
        }
        if(TestStage==1)
        {const bool Still=FMath::Abs(V->SpeedMps)<.03&&FMath::Abs(V->LateralMps)<.03;FinalRise=Still?FinalRise+Dt:0;TestDepth=FMath::Max(TestDepth,(V->Chassis->GetActorLocation()-StartPosition).Size()/100.);if(V->GuardObstacle){const double Front=V->GuardObstacle->GetActorLocation().X/100.-.15;for(AForkTineActor* T:V->Fork->Tines)TestMinClearance=FMath::Min(TestMinClearance,Front-T->Tip().X/100.);}}
        if(TestStage==1&&Age-StageStarted>1&&FinalRise>.3)
        {
            bool Quarantined=false;for(const auto& C:Traffic.Claims)Quarantined|=C.Owner==V->VehicleId&&C.Quarantined;
            if(!J->Held()||FMath::Abs(V->SpeedMps)>.03||!Quarantined||TestMinClearance<.02){Finish(false,TEXT("physical obstacle did not stop and quarantine"));return;}
            if(TestCase==TEXT("obstacle")){Finish(true,TEXT("actual physical obstacle causes stop and retained corridor"));return;}
            V->GuardObstacle->Destroy();V->GuardObstacle=nullptr;TestStage=2;StageStarted=Age;
        }
        if(TestStage==1&&Age-StageStarted>8){Finish(false,TEXT("obstacle brake did not settle within 8 s"));return;}
        if(TestStage==2&&Age-StageStarted>.8)
        {V->AcknowledgeSensorLift();ResumeSelected();if(J->Travelling()){TestStage=3;StageStarted=Age;StartPosition=V->Chassis->GetActorLocation();}}
        if(TestStage==3&&Distance>.3){Finish(!J->Held(),TEXT("explicit acknowledgement, measured obstacle replan and resumed torque motion"));return;}
        if(Age>60){Finish(false,TEXT("obstacle recovery timeout: ")+Notice);return;}
    }
    else if(TestCase==TEXT("sensor_stop")||TestCase==TEXT("watchdog"))
    {
        if(TestStage==.5&&Distance>.3){TestStage=1;StageStarted=Age;FinalRise=0;TestDepth=0;StartPosition=V->Chassis->GetActorLocation();if(TestCase==TEXT("sensor_stop"))V->VehicleGuard->Sensor.ManualOutage=true;}
        if(TestStage==1)
        {
            bool Quarantined=false;for(const auto& C:Traffic.Claims)Quarantined|=C.Owner==V->VehicleId&&C.Quarantined;
            TestDepth=FMath::Max(TestDepth,(V->Chassis->GetActorLocation()-StartPosition).Size()/100.);
            const bool Still=FMath::Abs(V->SpeedMps)<.03&&FMath::Abs(V->LateralMps)<.03;FinalRise=Still?FinalRise+Dt:0;
            if(Age-StageStarted>1&&FinalRise>.3){Finish(J->Held()&&Quarantined&&TestDepth<1&&J->Reason==(TestCase==TEXT("watchdog")?TEXT("fleet_command_expired"):TEXT("sensor_or_localization_unavailable")),TEXT("bounded stable physical stop; motion lease/sensor fault latched and corridor quarantined"));return;}
            if(Age-StageStarted>8){Finish(false,TEXT("physical brake did not settle within 8 s"));return;}
        }
    }
    else if(TestCase==TEXT("transfer")||TestCase==TEXT("upper")||TestCase==TEXT("rack_transfer")||TestCase==TEXT("full_transfer"))
    {
        if(J->Held()){Finish(false,TEXT("transfer held: ")+J->Reason);return;}
        if(J->State==TEXT("complete"))
        {
            const auto& L=Inventory.Loads[J->Load];
            const bool ReachedCruise=TestCase!=TEXT("transfer")||TestPeakLoadedMps>=.75;
            Finish(ReachedCruise&&J->PickupReceipt&&J->DropoffReceipt&&L.Slot==J->Destination&&LoadOrder.Num()==35&&SawFull&&MaxForkLoad>1000&&!V->CargoState.bFell&&MaxWheelLoad<10,
                ReachedCruise?TEXT("sensor-controlled pickup, transfer, shelf unload, withdrawal and return"):TEXT("loaded transit did not reach 0.75 m/s"));
            return;
        }
    }
    if(Age>(TestCase==TEXT("full_transfer")?900:360))Finish(false,TEXT("native_timeout state=")+J->State+TEXT(" reason=")+J->Reason);
}
