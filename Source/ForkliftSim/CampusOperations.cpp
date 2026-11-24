#include "CampusOperations.h"
#include "WarehouseCampus.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "WheelActor.h"
#include "LidarSafety.h"
#include "ProximitySensors.h"
#include "PalletPerception.h"
#include "DynamicsSensors.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "WebTelemetry.h"
#include "WebTaskInbox.h"
#include "Serialization/JsonSerializer.h"
namespace
{
    FVector Vec(const TSharedPtr<FJsonObject>& O,const TCHAR* K){const auto& A=O->GetArrayField(K);return FVector(A[0]->AsNumber(),A[1]->AsNumber(),A[2]->AsNumber());}
    bool Read(const FString& Name,TSharedPtr<FJsonObject>& J){FString T;return FFileHelper::LoadFileToString(T,*(FPaths::ProjectConfigDir()/Name))&&FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(T),J)&&J.IsValid();}
    void Save(const FString& Name,const TSharedRef<FJsonObject>& J){FString T;FJsonSerializer::Serialize(J,TJsonWriterFactory<>::Create(&T));const FString D=FPaths::ProjectSavedDir()/TEXT("Validation/Campus");IFileManager::Get().MakeDirectory(*D,true);FFileHelper::SaveStringToFile(T,*(D/Name),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);}
}
ACampusOperations::ACampusOperations(){PrimaryActorTick.bCanEverTick=true;PrimaryActorTick.TickGroup=TG_PrePhysics;}
ACampusOperations* ACampusOperations::Find(UWorld* W){for(TActorIterator<ACampusOperations> I(W);I;++I)return *I;return nullptr;}
AStabilityLab* ACampusOperations::Active() const{return Vehicles.IsValidIndex(Selected)?Vehicles[Selected].Get():nullptr;}
bool ACampusOperations::LoadCampus()
{
    TSharedPtr<FJsonObject> J,Map;if(!Read(TEXT("WarehouseCampus.json"),J)||!Read(TEXT("CampusSurvey.json"),Map)||!PayloadSpec.Load(Error)||!PayloadSpec.UseCampusGeometry(Error)){Error=TEXT("Cannot read campus layout/survey/pallet");return false;}
    InitialVehicles=J->GetIntegerField(TEXT("initial_vehicles"));
    for(const auto& V:J->GetArrayField(TEXT("parking"))){const auto& C=V->AsObject()->GetArrayField(TEXT("center_m"));ParkingStartM.Add(FVector(C[0]->AsNumber(),C[1]->AsNumber(),0));Chargers.Add({V->AsObject()->GetStringField(TEXT("id")),TEXT(""),FVector2D(C[0]->AsNumber(),C[1]->AsNumber())});}
    if(InitialVehicles<1||InitialVehicles>ParkingStartM.Num()||InitialVehicles>8){Error=TEXT("Initial vehicle count outside reserved bays");return false;}
    for(const auto& V:J->GetArrayField(TEXT("slots")))
    {
        const auto O=V->AsObject();CampusInventory::FSlot S;S.Id=O->GetStringField(TEXT("id"));S.CenterM=Vec(O,TEXT("center_m"));
        SlotApproaches.Add(S.Id,Vec(O,TEXT("approach_m")));SlotYaws.Add(S.Id,FMath::DegreesToRadians(O->GetNumberField(TEXT("approach_yaw_deg"))));
        const double Sub=FCString::Atoi(*S.Id.Right(1));const double Left=Sub==1?-.75:-2.25,Right=Left+3;SlotBeamSides.Add(S.Id,O->GetNumberField(TEXT("approach_yaw_deg"))>0?FVector2D(-Right,-Left):FVector2D(Left,Right));
        if(O->GetBoolField(TEXT("initial_visual_stock"))){CampusInventory::FLoad L;L.Id=TEXT("LOAD-")+S.Id;L.CenterM=S.CenterM;L.Slot=S.Id;S.Pallet=L.Id;Inventory.Loads.Add(L.Id,L);LoadOrder.Add(L.Id);}Inventory.Slots.Add(S.Id,S);
    }
    for(const auto& V:J->GetArrayField(TEXT("stations")))
    {const auto O=V->AsObject();CampusInventory::FLoad L;L.Id=O->GetStringField(TEXT("id"));L.CenterM=Vec(O,TEXT("center_m"));KnownStations.Add(L.Id,L.CenterM);Inventory.Loads.Add(L.Id,L);LoadOrder.Insert(L.Id,FMath::Min(LoadOrder.Num(),FCString::Atoi(*L.Id.Right(2))-1));}
    for(const auto& V:Map->GetArrayField(TEXT("segments")))
    {const auto O=V->AsObject();const auto& A=O->GetArrayField(TEXT("a"));const auto& B=O->GetArrayField(TEXT("b"));Survey.Add({FVector2D(A[0]->AsNumber(),A[1]->AsNumber()),FVector2D(B[0]->AsNumber(),B[1]->AsNumber())});}
    TMap<FString,int> NodeIds;for(const auto& Value:J->GetArrayField(TEXT("route_nodes"))){const auto O=Value->AsObject();const auto& P=O->GetArrayField(TEXT("xy_m"));FString Id=O->GetStringField(TEXT("id"));NodeIds.Add(Id,Graph.Nodes.Num());Graph.Nodes.Add({Id,FVector2D(P[0]->AsNumber(),P[1]->AsNumber())});}
    for(const auto& Value:J->GetArrayField(TEXT("route_edges"))){const auto O=Value->AsObject();Graph.Edges.Add({NodeIds.FindChecked(O->GetStringField(TEXT("a"))),NodeIds.FindChecked(O->GetStringField(TEXT("b"))),O->GetNumberField(TEXT("nominal_width_m"))});}
    for(const auto& Pair:Inventory.Loads){const auto P=Pair.Value.CenterM;for(double X:{-.55,0.,.55})for(double Y:{-.55,0.,.55})Graph.Clearance.Obstacles.Add(FVector2D(P.X+X,P.Y+Y));}
    Graph.Clearance.Map=Survey;Graph.Clearance.Spec.Bounds=FBox2D(FVector2D(-39,-11),FVector2D(99,49));Graph.Clearance.Spec.Front=2.05;Graph.Clearance.Spec.Rear=.8;Graph.Clearance.Spec.HalfWidth=.65;Graph.Clearance.Spec.Margin=.15;
    if(Inventory.Slots.Num()!=256||LoadOrder.Num()!=35||Survey.Num()<12){Error=TEXT("Invalid campus record counts");return false;}return true;
}
void ACampusOperations::RemovePresentation(const FString& Id){for(TActorIterator<AWarehouseMapGroup> I(GetWorld());I;++I)I->RemoveRecord(Id);}
APalletLoadActor* ACampusOperations::ActivateLoad(const FString& Id)
{
    if(auto* Found=PhysicalLoads.Find(Id))return Found->Get();auto* R=Inventory.Loads.Find(Id);if(!R)return nullptr;
    auto* P=GetWorld()->SpawnActor<APalletLoadActor>((R->CenterM+PayloadSpec.PalletDatumM+FVector(0,0,.002))*100,FRotator::ZeroRotator);
    if(!P)return nullptr;P->SetOwner(this);P->Configure(PayloadSpec);P->InventoryPalletId=Id+TEXT("-PALLET");P->InventoryCargoId=Id+TEXT("-CARGO");P->Tags.Add(FName(*P->InventoryPalletId));P->Cargo->Tags.Add(FName(*P->InventoryCargoId));P->Tags.Add(FName(*Id));
    RemovePresentation(Id);R->Physical=true;PhysicalLoads.Add(Id,P);return P;
}
void ACampusOperations::BeginPlay()
{
    Super::BeginPlay();Validation=FParse::Param(FCommandLine::Get(),TEXT("CampusFoundationTest"));FParse::Value(FCommandLine::Get(),TEXT("CampusCase="),TestCase);FParse::Value(FCommandLine::Get(),TEXT("CampusEntryYaw="),EntryYaw);
    FleetTest=FParse::Param(FCommandLine::Get(),TEXT("CampusFleetTest"));if(FleetTest)FParse::Value(FCommandLine::Get(),TEXT("CampusFleetCase="),TestCase);
    if(!LoadCampus()){UE_LOG(LogTemp,Error,TEXT("CAMPUS_INIT %s"),*Error);if(Validation)FinishValidation(false,Error);return;}
    LoadTaskConfig();LoadOperationsConfig();if(!Error.IsEmpty()){UE_LOG(LogTemp,Error,TEXT("CAMPUS_TASK_CONFIG %s"),*Error);return;}
    if(!Validation)SetupRouteReferences();
    for(TActorIterator<AStaticMeshActor> I(GetWorld());I;++I)if(I->ActorHasTag(TEXT("CampusRoad")))Floors.Add(I->GetStaticMeshComponent());
    if(Floors.Num()!=3){Error=TEXT("Campus needs exactly three shared physical floor slabs; rebuild campus map");UE_LOG(LogTemp,Error,TEXT("CAMPUS_INIT %s"),*Error);if(Validation)FinishValidation(false,Error);return;}
    if(Validation&&(TestCase==TEXT("handling")||TestCase==TEXT("vision")))Inventory.Loads[TEXT("PICK-01")].CenterM=FVector(10,18,0);
    if(Validation&&(TestCase==TEXT("inventory")||TestCase==TEXT("rackvision")))Inventory.Loads[TEXT("PICK-01")].CenterM=Inventory.Slots[TEXT("S01-B1-S-L2-1")].CenterM;
    if(FleetTest&&(TestCase==TEXT("transfer")||TestCase==TEXT("upper")||TestCase==TEXT("board_cycle"))){Inventory.Loads[TEXT("PICK-01")].CenterM=FVector(26.75,6.2,0);KnownStations[TEXT("PICK-01")]=FVector(26.75,6.2,0);ParkingStartM[0]=FVector(23.65,6.2,0);HighTier=TestCase==TEXT("upper");}
    if(FleetTest&&TestCase==TEXT("board_cycle")){Inventory.Loads[TEXT("PICK-02")].CenterM=FVector(26.75,4.8,0);KnownStations[TEXT("PICK-02")]=FVector(26.75,4.8,0);}
    if(FleetTest&&TestCase==TEXT("rack_transfer"))ParkingStartM[0]=FVector(26.75,8,0);
    if(FleetTest&&TestCase.StartsWith(TEXT("ops_parallel")))for(int I=0;I<3;++I){const double Y=I==0?8.:I==1?16.:39.;ParkingStartM[I]=FVector(-20,Y,0);Inventory.Loads[LoadOrder[I]].CenterM=FVector(-10,Y,0);KnownStations[LoadOrder[I]]=FVector(-10,Y,0);}
    if(FleetTest&&TestCase==TEXT("ops_cross")){ParkingStartM[0]=FVector(-24,10,0);ParkingStartM[1]=FVector(-16,5,0);Inventory.Loads[LoadOrder[0]].CenterM=KnownStations[LoadOrder[0]]=FVector(-6,10,0);Inventory.Loads[LoadOrder[1]].CenterM=KnownStations[LoadOrder[1]]=FVector(-12.9,23,0);}
    if(FleetTest&&TestCase==TEXT("ops_six"))InitialVehicles=6;
    if(FleetTest&&TestCase.StartsWith(TEXT("ops_charge"))){ParkingStartM[0]=FVector(-28,5,0);BatterySpec.Resume=.25004;}
    for(int I=0;I<InitialVehicles;++I)
    {
        const FString Id=FString::Printf(TEXT("AGV-%02d"),I+1),Load=LoadOrder[I];auto* P=ActivateLoad(Load);Inventory.Reserve(Load,Id);
        FVector Bay=ParkingStartM[I]*100;double Yaw=0;
        if(Validation&&(TestCase==TEXT("handling")||TestCase==TEXT("vision"))&&I==0){Yaw=EntryYaw;Bay=FVector(1000,1800,0)-FRotator(0,Yaw,0).RotateVector(FVector(TestCase==TEXT("vision")?270.:260.,0,0));}
        if(Validation&&TestCase==TEXT("rackvision")&&I==0){Yaw=90;EntryYaw=90;Bay=Inventory.Loads[TEXT("PICK-01")].CenterM*100-FVector(0,270,195);}
        const FTransform Spawn(FRotator(0,Yaw,0),Bay);
        auto* V=GetWorld()->SpawnActorDeferred<AStabilityLab>(AStabilityLab::StaticClass(),Spawn,this,nullptr,ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
        V->bCampusVehicle=true;V->VehicleId=Id;V->Campus=this;V->CampusLoadId=Load;V->Pallet=P;V->CampusSpawn=Spawn;V->CampusSurfaces=Floors;V->FinishSpawning(Spawn);
        Vehicles.Add(V);auto& Battery=Batteries.Add(Id);Battery.Init(BatterySpec);V->AddTickPrerequisiteActor(this);V->bCampusSelected=I==0;V->Brake=1;
        RemovePresentation(TEXT("DISPLAY-")+Id);IdleStart.Add(V->Chassis?V->Chassis->GetActorLocation():FVector::ZeroVector);
    }
    if(FleetTest&&TestCase.StartsWith(TEXT("ops_charge")))Batteries[TEXT("AGV-01")].StoredWh=BatterySpec.CapacityWh()*.24999;
    if(FleetTest&&TestCase==TEXT("ops_critical"))Batteries[TEXT("AGV-01")].StoredWh=BatterySpec.CapacityWh()*.049;
    WriteInventorySnapshot();
    FleetWeb=FWebTelemetryPublisher::Create();if(FleetWeb)FleetWeb->NewRun();
    if(FleetWeb&&BoardEnabled()){TaskInbox=MakeShared<FWebTaskInbox,ESPMode::ThreadSafe>();TaskInbox->Base=FleetWeb->GetBaseUrl();TaskInbox->World=FleetWeb->GetWorldId();TaskInbox->Run=TaskBoard.Run;}
    Notice=TEXT("Manual campus: TAB vehicle | G target load | END follow | 1-4 inspect");
    UE_LOG(LogTemp,Display,TEXT("CAMPUS_READY vehicles=%d loads=%d physical_loads=%d slots=%d survey_segments=%d ground_truth_control=0"),Vehicles.Num(),Inventory.Loads.Num(),PhysicalLoads.Num(),Inventory.Slots.Num(),Survey.Num());
}
void ACampusOperations::ConfigureVehicle(AStabilityLab& V)
{
    if(V.DriveSensors){auto& M=V.DriveSensors->Model;M.Map=Survey;M.MapId=Validation?TEXT("warehouse-campus-f1-authored-static-survey"):TEXT("warehouse-campus-f1-authored-static-survey+route-bollards-v1");M.MapToOdom={V.CampusSpawn.GetLocation().X/100.,V.CampusSpawn.GetLocation().Y/100.,FMath::DegreesToRadians(V.CampusSpawn.Rotator().Yaw)};}
}
void ACampusOperations::SelectNext()
{
    if(Vehicles.IsEmpty())return;for(AStabilityLab* V:Vehicles){V->Throttle=V->Strafe=V->Steering=V->Lift=0;if(!VehicleEngaged(*V)){V->Throttle=V->Strafe=V->Steering=V->Lift=0;V->Brake=1;}V->bCampusSelected=false;}Selected=(Selected+1)%Vehicles.Num();Active()->bCampusSelected=true;
}
bool ACampusOperations::SelectLoad(const FString& Id)
{
    auto* V=Active();if(!V)return false;if(VehicleEngaged(*V)){Notice=TEXT("Pause/recover the active job before changing its reserved target");return false;}if(Id==V->CampusLoadId)return true;
    if(!V->DynamicsSensors||!V->DynamicsSensors->Model.Usable(2,V->TimeS)||!V->DynamicsSensors->Model.Usable(3,V->TimeS)){Notice=TEXT("Wait for healthy measured fork loads before changing target");return false;}
    const auto& Forces=V->DynamicsSensors->Model.Channels;
    if(Forces[2].Last.Value[0]+Forces[3].Last.Value[0]>100||V->PayloadState.FilteredN[0]+V->PayloadState.FilteredN[1]>100||FMath::Abs(V->SpeedMps)>.05||FMath::Abs(V->LateralMps)>.05){Notice=TEXT("Stop and lower/release supported load before changing target");return false;}
    if(!Inventory.Reserve(Id,V->VehicleId)){Notice=TEXT("Target reserved by another vehicle or unavailable");return false;}
    auto* P=ActivateLoad(Id);if(!P){Inventory.Release(Id,V->VehicleId);return false;}
    const auto* PreviousLoad=Inventory.Loads.Find(V->CampusLoadId);if(PreviousLoad&&PreviousLoad->ReservedBy==V->VehicleId&&!Inventory.Release(V->CampusLoadId,V->VehicleId)){Inventory.Release(Id,V->VehicleId);Notice=TEXT("Previous load still moving; wait before changing target");return false;}V->CampusLoadId=Id;V->Pallet=P;V->PayloadState=AStabilityLab::FPayloadState();V->PayloadState.Start=P->GetActorLocation();V->CargoState=AStabilityLab::FCargoState();V->CargoState.InitialRelativeM=P->Deck->GetComponentQuat().UnrotateVector(P->Cargo->GetActorLocation()-P->GetActorLocation())/100.;
    if(V->PalletRecognition){V->PalletRecognition->HaveAnchor=false;V->PalletRecognition->Locked=false;V->PalletRecognition->Tracked=false;V->PalletRecognition->Detection={};V->PalletRecognition->Decision={};}
    Notice=TEXT("Target reserved: ")+Id+TEXT("; perception still requires measured depth evidence");return true;
}
void ACampusOperations::CycleLoad()
{
    auto* V=Active();if(!V)return;const int From=LoadOrder.IndexOfByKey(V->CampusLoadId);
    for(int I=1;I<=LoadOrder.Num();++I){const FString Id=LoadOrder[(FMath::Max(0,From)+I)%LoadOrder.Num()];const auto& R=Inventory.Loads[Id];if(R.ReservedBy.IsEmpty()||R.ReservedBy==V->VehicleId){SelectLoad(Id);return;}}
}
void ACampusOperations::ReconcileInventory()
{
    // Simulation-side inventory observation, not a perception input. Geometry and
    // motion decide occupancy only after dwelling; a reservation is never proof of placement.
    for(auto& Pair:PhysicalLoads)
    {
        auto* P=Pair.Value.Get();if(!P||!P->Deck)continue;auto& L=Inventory.Loads[Pair.Key];const FVector Bottom=(P->GetActorLocation()-P->Deck->GetComponentQuat().RotateVector(PayloadSpec.PalletDatumM*100))/100.;L.CenterM=Bottom;
        L.Moving=P->Deck->GetPhysicsLinearVelocity().Size()>5||P->Deck->GetPhysicsAngularVelocityInRadians().Size()>.05;
        FString Candidate;for(const auto& S:Inventory.Slots)if(FVector2D(Bottom-S.Value.CenterM).Size()<.10&&FMath::Abs(Bottom.Z-S.Value.CenterM.Z)<.025&&P->Deck->GetUpVector().Z>.998){Candidate=S.Key;break;}
        if(Candidate!=L.Slot)Inventory.Depart(L.Id);
        bool Held=P->HasRecentVehicleSupport();for(AStabilityLab* V:Vehicles)if(V->Pallet==P)Held|=V->PayloadState.FilteredN[0]+V->PayloadState.FilteredN[1]>80;
        if(L.Moving||Candidate.IsEmpty()||Held||!P->HasRecentRackSupport()){StableSince.Remove(L.Id);continue;}
        double& Since=StableSince.FindOrAdd(L.Id,Age);if(Age-Since>.4&&!Inventory.Place(L.Id,Candidate,L.ReservedBy))Notice=TEXT("Placement conflict: ")+L.Id+TEXT(" / ")+Candidate;
    }
}
void ACampusOperations::WriteInventorySnapshot()
{
    auto J=MakeShared<FJsonObject>();J->SetStringField(TEXT("source"),TEXT("unreal-campus-inventory-authority"));J->SetStringField(TEXT("scope"),TEXT("simulation bookkeeping; not a perception/localization measurement or fleet web feed"));J->SetNumberField(TEXT("sim_s"),Age);
    TArray<TSharedPtr<FJsonValue>> Slots,Loads,VehiclesJson;
    for(const auto& Pair:Inventory.Slots){const auto& S=Pair.Value;auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("id"),S.Id);O->SetStringField(TEXT("pallet"),S.Pallet);O->SetStringField(TEXT("reserved_by"),S.ReservedBy);O->SetStringField(TEXT("state"),S.Blocked?TEXT("blocked"):!S.Pallet.IsEmpty()?TEXT("occupied"):!S.ReservedBy.IsEmpty()?TEXT("reserved"):TEXT("empty"));Slots.Add(MakeShared<FJsonValueObject>(O));}
    for(const auto& Pair:Inventory.Loads){const auto& L=Pair.Value;auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("id"),L.Id);O->SetStringField(TEXT("pallet_id"),L.Id+TEXT("-PALLET"));O->SetStringField(TEXT("cargo_id"),L.Id+TEXT("-CARGO"));O->SetStringField(TEXT("slot"),L.Slot);O->SetStringField(TEXT("reserved_by"),L.ReservedBy);O->SetBoolField(TEXT("physical"),L.Physical);O->SetBoolField(TEXT("moving"),L.Moving);if(auto* P=PhysicalLoads.Find(Pair.Key))O->SetBoolField(TEXT("rack_contact_support"),(*P)->HasRecentRackSupport());Loads.Add(MakeShared<FJsonValueObject>(O));}
    for(AStabilityLab* V:Vehicles){auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("id"),V->VehicleId);O->SetStringField(TEXT("target_load"),V->CampusLoadId);if(V->DriveSensors)O->SetStringField(TEXT("drive_csv"),V->DriveSensors->CsvPath);if(V->PalletRecognition)O->SetStringField(TEXT("perception_csv"),V->PalletRecognition->CsvPath);if(V->ProximitySensors)O->SetStringField(TEXT("proximity_csv"),V->ProximitySensors->CsvPath);VehiclesJson.Add(MakeShared<FJsonValueObject>(O));}
    J->SetArrayField(TEXT("slots"),Slots);J->SetArrayField(TEXT("loads"),Loads);J->SetArrayField(TEXT("vehicles"),VehiclesJson);Save(TEXT("InventoryState.json"),J);
}
void ACampusOperations::Tick(float Dt)
{
    Super::Tick(Dt);Age+=Dt;if(!Error.IsEmpty())return;
    if(Age>=NextInventory){NextInventory=Age+.1;ReconcileInventory();}
    if(Age>=NextSnapshot){NextSnapshot=Age+1;WriteInventorySnapshot();}
    ObserveTraffic();ServiceEnergy(Dt);ServiceTasks(Dt);Traffic.Audit(Age);PublishFleet();if(FleetTest)FleetValidationStep(Dt);
    if(Validation)ValidateStep(Dt);
}
void ACampusOperations::EndPlay(const EEndPlayReason::Type Reason)
{
    if(TaskInbox){TaskInbox->Stop();TaskInbox.Reset();}TaskBoard.Close(Age);SaveTasks();if(FleetWeb&&Active()){auto Frame=Active()->BuildWebFrame();Frame->SetObjectField(TEXT("fleet"),FleetTelemetry());FleetWeb->PrepareStopFrame(Frame);}
    // Explicit owned lifetime. Removing a vehicle never destroys shared stock.
    for(AActor* A:NavigationReferences)if(IsValid(A))A->Destroy();NavigationReferences.Empty();if(FleetWeb){FleetWeb->Stop();FleetWeb.Reset();}for(AStabilityLab* V:Vehicles)if(IsValid(V))V->Destroy();Vehicles.Empty();for(auto& P:PhysicalLoads)if(IsValid(P.Value))P.Value->Destroy();PhysicalLoads.Empty();Super::EndPlay(Reason);
}
