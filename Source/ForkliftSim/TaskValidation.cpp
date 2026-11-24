#include "CampusOperations.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "LidarSafety.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "Serialization/JsonSerializer.h"
void ACampusOperations::TaskValidationStep(float Dt)
{
 if(TestStage==99||!Active())return;auto* V=Active();
 auto Finish=[&](bool Pass,const FString& Why){SaveTasks();auto J=TasksTelemetry();J->SetBoolField(TEXT("pass"),Pass);J->SetStringField(TEXT("case"),TestCase);J->SetStringField(TEXT("reason"),Why);J->SetStringField(TEXT("scope"),TEXT("Native Unreal physical vehicles and actual measured controls; initialization-only fixture stations, never teleported completion."));J->SetNumberField(TEXT("observed_game_fps"),V->GameFPS());J->SetNumberField(TEXT("sim_wall_ratio"),V->ProgressRatio());FString Text;FJsonSerializer::Serialize(J,TJsonWriterFactory<>::Create(&Text));const FString Folder=FPaths::ProjectSavedDir()/TEXT("Validation/Tasks");IFileManager::Get().MakeDirectory(*Folder,true);FFileHelper::SaveStringToFile(Text,*(Folder/(TestCase+TEXT(".json"))),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);UE_LOG(LogTemp,Display,TEXT("TASK_BOARD %s case=%s reason=%s"),Pass?TEXT("PASS"):TEXT("FAIL"),*TestCase,*Why);TestStage=99;FGenericPlatformMisc::RequestExitWithStatus(false,Pass?0:1);};
 if(TestCase==TEXT("board_scenario")){if(Age>5){bool Pass=FMath::Abs(PayloadSpec.CargoMassKg-600)<.01&&FMath::Abs(ScenarioRoad-.3)<.001;for(const auto& P:PhysicalLoads)Pass&=FMath::Abs(P.Value->Cargo->GetStaticMeshComponent()->GetMass()-600)<1;for(AStabilityLab* B:Vehicles)Pass&=B->FrictionMaterialsMatch()&&FMath::Abs(B->Settings.RoadMu-.3)<.001;Finish(Pass,TEXT("actual cargo rigid-body mass and physical road/tyre materials match selected experiment"));}return;}
 if(TestStage==0&&Age>3&&TestCase!=TEXT("board_web"))
 {
  TArray<WarehouseTasks::FRequest> Requests;const int Count=TestCase==TEXT("board_batch")?10:2;
  for(int I=0;I<Count;++I){WarehouseTasks::FRequest R;R.Load=LoadOrder[I];R.Priority=TestCase==TEXT("board_batch")&&I==Count-1?9:0;if(TestCase!=TEXT("board_batch"))R.PreferredVehicle=TEXT("AGV-01");Requests.Add(R);}TArray<FString> Ids;FString Why;if(!AddTasks(Requests,Ids,Why)){Finish(false,Why);return;}TestStage=1;StageStarted=Age;
 }
 if(TestCase==TEXT("board_batch"))
 {
  int Assigned=0;TSet<FString> Owners,Slots;bool Good=true;for(const auto& T:TaskBoard.Tasks)if(T.Assigned>=0){++Assigned;Owners.Add(T.Vehicle);Slots.Add(T.Destination);Good&=T.State!=TEXT("failed");}
  if(Assigned==3&&Age-StageStarted>5){Finish(Good&&TaskBoard.Tasks.Num()==10&&Owners.Num()==3&&Slots.Num()==3&&TaskBoard.Tasks.Last().Assigned>=0,TEXT("ten actual queue entries, priority dispatch, three unique vehicles/slots and seven pending requests"));return;}
 }
 else if(TestCase==TEXT("board_cycle"))
 {
  for(const auto& T:TaskBoard.Tasks)if(T.State==TEXT("failed")||T.State==TEXT("paused")){Finish(false,T.Reason);return;}
  if(TaskBoard.Tasks.Num()==2&&TaskBoard.Tasks[0].State==TEXT("completed")&&TaskBoard.Tasks[1].State==TEXT("completed")){bool Good=true;for(const auto& T:TaskBoard.Tasks)Good&=T.Vehicle==TEXT("AGV-01")&&T.Picked&&T.Dropped&&T.Distance>1&&T.PeakForkN>1000&&Inventory.Loads[T.Load].Slot==T.Destination;Finish(Good&&TaskBoard.Tasks[0].Job!=TaskBoard.Tasks[1].Job,TEXT("same physical vehicle automatically completes two distinct sensed pickup/unload/return jobs; receipts, inventory and measured KPI retained"));return;}
 }
 else if(TestCase==TEXT("board_fault"))
 {
  if(!TaskBoard.Tasks.IsEmpty()&&TaskBoard.Tasks[0].Distance>.15&&TestStage==1){V->VehicleGuard->Sensor.ManualOutage=true;TestStage=2;StageStarted=Age;}
  if(TestStage==2&&Age-StageStarted>2){const auto& T=TaskBoard.Tasks[0];if(FMath::Abs(V->SpeedMps)<.03&&FMath::Abs(V->LateralMps)<.03){bool Q=false;for(const auto& C:Traffic.Claims)Q|=C.Owner==V->VehicleId&&C.Quarantined;Finish(T.State==TEXT("failed")&&T.RecoveryRequired&&T.Stops==1&&TaskBoard.Tasks[1].Pending()&&Q,TEXT("actual sensor outage holds failed task/vehicle/corridor and prevents automatic reuse"));return;}if(Age-StageStarted>8){Finish(false,TEXT("brake_not_settled"));return;}}
 }
 else if(TestCase==TEXT("board_web"))
 {
  bool Cancelled=false,Moved=false;for(const auto& T:TaskBoard.Tasks){Cancelled|=T.State==TEXT("cancelled");Moved|=T.Distance>.15;}
  if(TaskBoard.Tasks.Num()==10&&Age>15&&Moved&&Cancelled&&CommandReceipts.Num()>=2){Finish(TaskBoard.Tasks[0].State!=TEXT("failed"),TEXT("browser-shaped HTTP request acknowledged by Unreal and assigned to physical vehicle with measured motion"));return;}
 }
 if(Age>(TestCase==TEXT("board_cycle")?900:90))Finish(false,TEXT("task_qualification_timeout"));
}
