#include "WarehouseCampus.h"
#include "SimulationPlayback.h"
#include "PlaybackValidation.h"
#include "GameFramework/WorldSettings.h"
#include "CampusOperations.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "LidarSafety.h"
#include "SensorDrive.h"
#include "PalletPerception.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/InputComponent.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/SpectatorPawn.h"
#include "GameFramework/SpectatorPawnMovement.h"
#include "HAL/PlatformTime.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "UnrealClient.h"
#include "Camera/PlayerCameraManager.h"

AWarehouseMapGroup::AWarehouseMapGroup()
{
    PrimaryActorTick.bCanEverTick=false;
    Instances=CreateDefaultSubobject<UHierarchicalInstancedStaticMeshComponent>(TEXT("LayoutInstances"));RootComponent=Instances;
    Instances->SetMobility(EComponentMobility::Static);Instances->SetCanEverAffectNavigation(false);
}
void AWarehouseMapGroup::Rebuild()
{
    Instances->ClearInstances();Instances->SetStaticMesh(LayoutMesh);
    if(LayoutMaterial)Instances->SetMaterial(0,LayoutMaterial);
    Instances->SetCollisionProfileName(Blocking?TEXT("BlockAll"):TEXT("NoCollision"));
    Instances->SetCollisionEnabled(Blocking?ECollisionEnabled::QueryAndPhysics:ECollisionEnabled::NoCollision);
    Instances->SetCastShadow(CastLayoutShadow);Instances->SetGenerateOverlapEvents(false);Instances->SetVisibility(!InvisibleProxy,true);
    if(LayoutMesh&&!LayoutTransforms.IsEmpty())Instances->AddInstances(LayoutTransforms,false,false,false);
}
bool AWarehouseMapGroup::RemoveRecord(const FString& Id)
{
    bool Changed=false;for(int I=LayoutRecordIds.Num()-1;I>=0;--I)if(LayoutRecordIds[I]==Id&&LayoutTransforms.IsValidIndex(I)){LayoutRecordIds.RemoveAt(I);LayoutTransforms.RemoveAt(I);Changed=true;}if(Changed)Rebuild();return Changed;
}
void AWarehouseMapGroup::OnConstruction(const FTransform& Transform){Super::OnConstruction(Transform);Rebuild();}
AWarehouseMapGameMode::AWarehouseMapGameMode()
{
    PlayerControllerClass=AWarehouseMapController::StaticClass();DefaultPawnClass=ASpectatorPawn::StaticClass();HUDClass=AWarehouseMapHUD::StaticClass();
}
void AWarehouseMapGameMode::BeginPlay()
{
    Super::BeginPlay();if(FParse::Param(FCommandLine::Get(),TEXT("PlaybackValidation")))GetWorld()->SpawnActor<APlaybackValidation>();FString Capture;
    if(!FParse::Value(FCommandLine::Get(),TEXT("CampusCapture="),Capture)&&!FParse::Param(FCommandLine::Get(),TEXT("CampusPreview")))GetWorld()->SpawnActor<ACampusOperations>();
}
void AWarehouseMapController::BeginPlay()
{
    Super::BeginPlay();SetInputMode(FInputModeGameOnly());bShowMouseCursor=false;StartedWall=FPlatformTime::Seconds();
    FParse::Value(FCommandLine::Get(),TEXT("CampusCapture="),CaptureName);
    for(TActorIterator<AActor> It(GetWorld());It;++It)if(It->ActorHasTag(TEXT("CampusRoof")))It->SetActorHiddenInGame(RoofHidden);
}
void AWarehouseMapController::View(int Mode)
{
    ManualVehicle=false;if(!GetPawn())return;FVector P,T;if(PlayerCameraManager)PlayerCameraManager->SetFOV(Mode==0?65:80);
    switch(Mode){case 1:P=FVector(-4700,-800,1800);T=FVector(-1300,2400,0);break;case 2:P=FVector(1200,2450,550);T=FVector(7200,2500,180);break;case 3:P=FVector(500,5200,1600);T=FVector(1200,3900,0);break;case 4:P=FVector(4100,1900,360);T=FVector(4700,1200,100);break;default:P=FVector(3000,-4900,9000);T=FVector(3000,2300,0);break;}
    GetPawn()->SetActorLocation(P);SetControlRotation((T-P).Rotation());
}
void AWarehouseMapController::Overview(){View(0);}void AWarehouseMapController::Yard(){View(1);}void AWarehouseMapController::Interior(){View(2);}void AWarehouseMapController::Service(){View(3);}
void AWarehouseMapController::ToggleRoof(){RoofHidden=!RoofHidden;for(TActorIterator<AActor> It(GetWorld());It;++It)if(It->ActorHasTag(TEXT("CampusRoof")))It->SetActorHiddenInGame(RoofHidden);}
void AWarehouseMapController::SetupInputComponent()
{
    Super::SetupInputComponent();SimulationPlayback::Bind(InputComponent,GetWorld());
    InputComponent->BindKey(EKeys::Enter,IE_Pressed,this,&AWarehouseMapController::StartJob);InputComponent->BindKey(EKeys::F,IE_Pressed,this,&AWarehouseMapController::StartFleet);InputComponent->BindKey(EKeys::BackSpace,IE_Pressed,this,&AWarehouseMapController::PauseJob);InputComponent->BindKey(EKeys::RightBracket,IE_Pressed,this,&AWarehouseMapController::ResumeJob);InputComponent->BindKey(EKeys::Zero,IE_Pressed,this,&AWarehouseMapController::CancelJob);InputComponent->BindKey(EKeys::PageDown,IE_Pressed,this,&AWarehouseMapController::LowTier);InputComponent->BindKey(EKeys::PageUp,IE_Pressed,this,&AWarehouseMapController::HighTier);
    InputComponent->BindKey(EKeys::L,IE_Pressed,this,&AWarehouseMapController::ForkRays);
    InputComponent->BindKey(EKeys::Tab,IE_Pressed,this,&AWarehouseMapController::NextVehicle);InputComponent->BindKey(EKeys::End,IE_Pressed,this,&AWarehouseMapController::FollowVehicle);InputComponent->BindKey(EKeys::G,IE_Pressed,this,&AWarehouseMapController::NextLoad);InputComponent->BindKey(EKeys::I,IE_Pressed,this,&AWarehouseMapController::SensorRays);InputComponent->BindKey(EKeys::P,IE_Pressed,this,&AWarehouseMapController::CameraToggle);InputComponent->BindKey(EKeys::O,IE_Pressed,this,&AWarehouseMapController::RearMode);InputComponent->BindKey(EKeys::X,IE_Pressed,this,&AWarehouseMapController::Estop);InputComponent->BindKey(EKeys::Y,IE_Pressed,this,&AWarehouseMapController::Ack);
    InputComponent->BindKey(EKeys::One,IE_Pressed,this,&AWarehouseMapController::Overview);InputComponent->BindKey(EKeys::Two,IE_Pressed,this,&AWarehouseMapController::Yard);InputComponent->BindKey(EKeys::Three,IE_Pressed,this,&AWarehouseMapController::Interior);InputComponent->BindKey(EKeys::Four,IE_Pressed,this,&AWarehouseMapController::Service);InputComponent->BindKey(EKeys::Home,IE_Pressed,this,&AWarehouseMapController::Overview);InputComponent->BindKey(EKeys::H,IE_Pressed,this,&AWarehouseMapController::ToggleRoof);
}
void AWarehouseMapController::PlayerTick(float Dt)
{
    Super::PlayerTick(Dt);const double Now=FPlatformTime::Seconds(),Elapsed=Now-StartedWall;
    auto* Ops=ACampusOperations::Find(GetWorld());
    if(Ops&&!InitialView&&GetPawn()){InitialView=true;ManualVehicle=true;}
    if(Ops)
    {
        ResetIgnoreMoveInput();if(ManualVehicle)SetIgnoreMoveInput(true);
        if(!Ops->Validation&&!Ops->FleetTest)for(AStabilityLab* V:Ops->Vehicles)
        {
            const bool Active=V==Ops->Active()&&ManualVehicle;
            V->Throttle=Active?double(IsInputKeyDown(EKeys::W))-double(IsInputKeyDown(EKeys::S)):0;
            V->Steering=Active?double(IsInputKeyDown(EKeys::D))-double(IsInputKeyDown(EKeys::A)):0;
            V->Strafe=Active?double(IsInputKeyDown(EKeys::C))-double(IsInputKeyDown(EKeys::Z)):0;
            V->Lift=Active?double(IsInputKeyDown(EKeys::E))-double(IsInputKeyDown(EKeys::Q)):0;
            V->Brake=IsInputKeyDown(EKeys::SpaceBar)&&Active?1:Ops->VehicleEngaged(*V)?0:!Active?1:0;V->bCampusManualInput=FMath::Abs(V->Throttle)+FMath::Abs(V->Steering)+FMath::Abs(V->Strafe)+FMath::Abs(V->Lift)>.01||(Active&&IsInputKeyDown(EKeys::SpaceBar));
        }
        auto* V=Ops->Active();if(ManualVehicle&&V&&V->Chassis&&GetPawn())
        {const FVector Target=V->Chassis->GetActorLocation()+FVector(0,0,100);const FVector P=Target+FVector(-650,-650,350);GetPawn()->SetActorLocation(P);SetControlRotation((Target-P).Rotation());}
    }
    if(!InitialView&&GetPawn()){View(CaptureName==TEXT("Yard")?1:CaptureName==TEXT("Interior")?2:CaptureName==TEXT("Service")?3:CaptureName==TEXT("Rack")?4:0);InitialView=true;}
    if(auto* P=Cast<ASpectatorPawn>(GetPawn()))if(auto* M=Cast<USpectatorPawnMovement>(P->GetMovementComponent()))M->MaxSpeed=IsInputKeyDown(EKeys::LeftShift)?6000:1800;
    if(FrameWall>0&&Elapsed>1){FrameSum+=Now-FrameWall;++Frames;}FrameWall=Now;
    if(!CaptureName.IsEmpty()&&Elapsed>6&&!CaptureRequested)
    {
        CaptureRequested=true;const FString Folder=FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir()/TEXT("Validation/Campus"));IFileManager::Get().MakeDirectory(*Folder,true);CapturePath=Folder/(TEXT("Campus-")+CaptureName+TEXT(".png"));FScreenshotRequest::RequestScreenshot(CapturePath,true,false);
        bool GeometryPass=true;for(const FVector P:{FVector(50,25,0),FVector(-12,25,0),FVector(18,12,0),FVector(96,43,0),FVector(-20,-6,0)}){FHitResult H;GeometryPass&=GetWorld()->LineTraceSingleByChannel(H,P*100+FVector(0,0,200),P*100-FVector(0,0,100),ECC_Visibility)&&FMath::Abs(H.ImpactPoint.Z)<1;}
        for(double Y:{12.,38.}){FHitResult H;GeometryPass&=!GetWorld()->LineTraceSingleByChannel(H,FVector(-100,Y*100,100),FVector(100,Y*100,100),ECC_Visibility);}FHitResult Wall;GeometryPass&=GetWorld()->LineTraceSingleByChannel(Wall,FVector(-100,2500,100),FVector(100,2500,100),ECC_Visibility);
        UE_LOG(LogTemp,Display,TEXT("CAMPUS_GEOMETRY %s floors=5 vehicle_portals=2 solid_wall=1"),GeometryPass?TEXT("PASS"):TEXT("FAIL"));if(!GeometryPass){FGenericPlatformMisc::RequestExitWithStatus(false,1);return;}
        int Groups=0,Count=0;for(TActorIterator<AWarehouseMapGroup> It(GetWorld());It;++It){++Groups;Count+=It->Instances->GetInstanceCount();}
        const FString Report=FString::Printf(TEXT("{\"scope\":\"map-only rendered walkthrough; no fleet physics or autonomy\",\"groups\":%d,\"instances\":%d,\"mean_fps\":%.3f,\"samples\":%d}"),Groups,Count,Frames/FMath::Max(.001,FrameSum),Frames);FFileHelper::SaveStringToFile(Report,*(Folder/(CaptureName+TEXT("-performance.json"))),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    }
    if(CaptureRequested&&Elapsed>8&&IFileManager::Get().FileExists(*CapturePath)){UE_LOG(LogTemp,Display,TEXT("CAMPUS_CAPTURE PASS %s"),*CapturePath);FGenericPlatformMisc::RequestExit(false);}
    if(!CaptureName.IsEmpty()&&Elapsed>40){UE_LOG(LogTemp,Error,TEXT("CAMPUS_CAPTURE failed"));FGenericPlatformMisc::RequestExitWithStatus(false,1);}
}
void AWarehouseMapController::StartJob(){if(auto* O=ACampusOperations::Find(GetWorld()))O->QueueSelected();}
void AWarehouseMapController::StartFleet(){if(auto* O=ACampusOperations::Find(GetWorld()))O->QueueAll();}
void AWarehouseMapController::PauseJob(){if(auto* O=ACampusOperations::Find(GetWorld()))O->PauseSelected();}
void AWarehouseMapController::ResumeJob(){if(auto* O=ACampusOperations::Find(GetWorld()))O->ResumeSelected();}
void AWarehouseMapController::CancelJob(){if(auto* O=ACampusOperations::Find(GetWorld()))O->CancelSelected();}
void AWarehouseMapController::LowTier(){if(auto* O=ACampusOperations::Find(GetWorld())){O->HighTier=false;O->Notice=TEXT("New jobs: lower tier");}}
void AWarehouseMapController::HighTier(){if(auto* O=ACampusOperations::Find(GetWorld())){O->HighTier=true;O->Notice=TEXT("New jobs: upper tier");}}
void AWarehouseMapController::NextVehicle(){if(auto* O=ACampusOperations::Find(GetWorld())){O->SelectNext();ManualVehicle=true;}}
void AWarehouseMapController::FollowVehicle(){ManualVehicle=true;}
void AWarehouseMapController::NextLoad(){if(auto* O=ACampusOperations::Find(GetWorld()))O->CycleLoad();}
void AWarehouseMapController::SensorRays(){if(auto* O=ACampusOperations::Find(GetWorld()))if(auto* V=O->Active())V->ToggleLidarRays();}
void AWarehouseMapController::ForkRays(){if(auto* O=ACampusOperations::Find(GetWorld()))if(auto* V=O->Active())V->ToggleProximityRays();}
void AWarehouseMapController::CameraToggle(){if(auto* O=ACampusOperations::Find(GetWorld()))if(auto* V=O->Active())V->ToggleCameraViews();}
void AWarehouseMapController::RearMode(){if(auto* O=ACampusOperations::Find(GetWorld()))if(auto* V=O->Active())V->ToggleRearCameraMode();}
void AWarehouseMapController::Estop(){if(auto* O=ACampusOperations::Find(GetWorld()))if(auto* V=O->Active())V->EmergencyVehicleStop();}
void AWarehouseMapController::Ack(){if(auto* O=ACampusOperations::Find(GetWorld()))if(auto* V=O->Active())V->AcknowledgeSensorLift();}
void AWarehouseMapHUD::DrawHUD()
{
    Super::DrawHUD();if(!Canvas)return;
    if(auto* O=ACampusOperations::Find(GetWorld()))
    {
        const float S=FMath::Clamp(Canvas->SizeX/1600.f,.6f,1.1f);DrawRect(FLinearColor(.025,.04,.06,.9),18*S,18*S,1120*S,284*S);
        auto Text=[&](const FString& T,int Y,FLinearColor C=FLinearColor::White){DrawText(T,C,32*S,Y*S,nullptr,.95f*S);};
        Text(FString::Printf(TEXT("CAMPUS F1 / %d PHYSICAL AGVs / FLEET 5-6"),O->Vehicles.Num()),28,FLinearColor(.35,.9,.83));
        Text(TEXT("TAB vehicle | END drive | 1-4 fly views | H roof | G next load | I lidar / L fork rays | P cameras | O rear RGB/depth"),52);
        Text(TEXT("ENTER queue / F fleet | PgDn/Up tier | BACKSPACE pause / ] resume / 0 hold | Manual W/S A/D Z/C Q/E SPACE"),74);
        if(auto* V=O->Active())
        {
            Text(FString::Printf(TEXT("%s | target %s | speed %.2f / side %.2f m/s | lift %.2f m | FPS %.1f | sim/wall %.2fx"),*V->VehicleId,*V->CampusLoadId,V->SpeedMps,V->LateralMps,V->ForkHeightM,V->GameFPS(),V->ProgressRatio()),100);
            Text(FString::Printf(TEXT("Fork load L/R %.0f / %.0f N | wheel total %.0f N | %s"),V->PayloadState.FilteredN[0],V->PayloadState.FilteredN[1],V->TotalNormalLoadN,V->VehicleGuard?*V->VehicleGuard->Decision.State:TEXT("guard unavailable")),124);
            if(V->DriveSensors){const auto& M=V->DriveSensors->Model;const auto P=M.MapPose(V->TimeS);Text(FString::Printf(TEXT("Estimated pose %.2f / %.2f m / %.1f deg | %s"),P.X,P.Y,FMath::RadiansToDegrees(P.Yaw),M.Localized&&V->TimeS-M.LastMatchS<.75?TEXT("fresh map match"):TEXT("known bay + odometry; fix unavailable")),148);}
            int Occupied=0,Reserved=0,Blocked=0;for(const auto& Pair:O->Inventory.Slots){Occupied+=!Pair.Value.Pallet.IsEmpty();Reserved+=!Pair.Value.ReservedBy.IsEmpty();Blocked+=Pair.Value.Blocked;}
            Text(FString::Printf(TEXT("Pallet %s | physical %d / registered %d | slots %d occupied / %d reserved / %d blocked / %d total"),V->PalletRecognition?*V->PalletRecognition->Decision.State:TEXT("unavailable"),O->PhysicalLoads.Num(),O->Inventory.Loads.Num(),Occupied,Reserved,Blocked,O->Inventory.Slots.Num()),172);
            V->DrawCameraViews(this,Canvas);
        }
        FString Status=O->Notice;if(auto* V=O->Active())if(auto* J=O->Job(V->VehicleId))Status=J->Id+TEXT(" / ")+J->State+TEXT(" / ")+J->Reason;Text(O->Error.IsEmpty()?Status:O->Error,198,FLinearColor(1,.8,.35));Text(O->Traffic.Sections?TEXT("Section windows; occupied/stale space retained | X stop / Y acknowledge"):TEXT("Whole-job reservation; occupied/stale space retained | X stop / Y acknowledge"),222);Text(FString::Printf(TEXT("F5 1x / F6 2x / F7 4x | TARGET %.0fx | actual %.2fx | %s"),GetWorld()->GetWorldSettings()->TimeDilation,O->Active()?O->Active()->ProgressRatio():0,SimulationPlayback::Limited(GetWorld(),O->Active()&&O->Active()->GameFPS()>0?1./O->Active()->GameFPS():0)?TEXT("PERFORMANCE LIMITED - physics step preserved"):TEXT("bounded physics/control clock")),246);
        return;
    }
    const float S=FMath::Clamp(Canvas->SizeX/1600.f,.65f,1.2f);DrawRect(FLinearColor(.025,.04,.06,.86),18*S,18*S,1020*S,100*S);
    DrawText(TEXT("WAREHOUSE CAMPUS / F1  100 x 50 m = 5,000 m2 INDOOR"),FLinearColor(.35,.9,.83),32*S,28*S,nullptr,S);
    DrawText(TEXT("MAP PREVIEW | 3 parked displays | 8 charging/parking bays | fleet control: next stage"),FLinearColor::White,32*S,53*S,nullptr,.9f*S);
    DrawText(TEXT("WASD fly | Q/E down/up | Mouse look | SHIFT fast | 1 overview  2 yard  3 indoor  4 services | H roof"),FLinearColor(.8,.85,.9),32*S,78*S,nullptr,.85f*S);
}
