#include "CampusOperations.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "WheelActor.h"
#include "LidarSafety.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "Serialization/JsonSerializer.h"
void ACampusOperations::OperationsValidation(float Dt)
{
 if(TestStage==99||!Active())return;auto* V=Active();
 auto Finish=[&](bool Pass,const FString& Why){SaveTasks();auto J=TasksTelemetry();J->SetBoolField(TEXT("pass"),Pass);J->SetStringField(TEXT("case"),TestCase);J->SetStringField(TEXT("reason"),Why);J->SetNumberField(TEXT("observed_game_fps"),V->GameFPS());J->SetNumberField(TEXT("sim_wall_ratio"),V->ProgressRatio());J->SetStringField(TEXT("scope"),TEXT("Native physical rigs, actual sensor velocity, integrated motor/pump energy. Initialization-only routing and near-threshold SOC fixtures; no position or task-completion teleport."));FString T;FJsonSerializer::Serialize(J,TJsonWriterFactory<>::Create(&T));const FString Folder=FPaths::ProjectSavedDir()/TEXT("Validation/Operations");IFileManager::Get().MakeDirectory(*Folder,true);FFileHelper::SaveStringToFile(T,*(Folder/(TestCase+TEXT(".json"))),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);UE_LOG(LogTemp,Display,TEXT("OPERATIONS %s case=%s reason=%s"),Pass?TEXT("PASS"):TEXT("FAIL"),*TestCase,*Why);TestStage=99;FGenericPlatformMisc::RequestExitWithStatus(false,Pass?0:1);};
 if(TestStage==0&&Age>3)
 {
  StartPosition=V->Chassis->GetActorLocation();TArray<WarehouseTasks::FRequest> R;int Count=TestCase==TEXT("ops_six")?6:TestCase==TEXT("ops_cross")?2:TestCase.StartsWith(TEXT("ops_parallel"))?3:1;
  for(int I=0;I<Count;++I){WarehouseTasks::FRequest Q;Q.Load=LoadOrder[I];Q.PreferredVehicle=Vehicles[I]->VehicleId;if(TestCase.StartsWith(TEXT("ops_parallel")))Q.Destination=FString::Printf(TEXT("S01-B%d-S-L1-2"),I+1);R.Add(Q);}TArray<FString> Ids;FString Why;if(!AddTasks(R,Ids,Why)){Finish(false,Why);return;}TestStage=1;StageStarted=Age;
 }
 if(TestCase.StartsWith(TEXT("ops_parallel")))
 {
  for(const auto& P:Jobs)if(P.Value.Held()){Finish(false,P.Value.Reason);return;}
  if(Age>16){const bool Sections=TestCase.EndsWith(TEXT("sections"));int Moved=0;for(const auto& T:TaskBoard.Tasks)Moved+=T.Distance>1;Finish(Sections?(PeakMoving>=3&&Moved==3):(PeakMoving==1&&Moved==1),TEXT("identical three-job physical fixture: compare concurrent measured motion under whole-job versus section-window reservation"));return;}
 }
 else if(TestCase==TEXT("ops_cross"))
 {
  for(const auto& T:TaskBoard.Tasks){if(T.State==TEXT("failed")||T.State==TEXT("paused")){Finish(false,T.Reason);return;}SawPeer|=T.TrafficWait>.5;}
  if(TaskBoard.Tasks.Num()==2&&TaskBoard.Tasks[0].Distance>11&&TaskBoard.Tasks[1].Distance>.3&&PeakMoving>=2){Finish(SawPeer,TEXT("crossing waits while occupied, then second real vehicle proceeds after first clears tail while first job remains active"));return;}
 }
 else if(TestCase==TEXT("ops_six"))
 {
  if(Age>16){bool Good=Vehicles.Num()==6&&Batteries.Num()==6;int Wheels=0,Assigned=0;double Energy=0,Distance=0;for(const AStabilityLab* B:Vehicles){const auto& D=B->DriveSensors->Model;Good&=D.Ready&&D.Localized&&B->TimeS-D.LastMatchS<1.5;Wheels+=B->Wheels.Num();Energy+=Batteries[B->VehicleId].ConsumedWh;}for(const auto& T:TaskBoard.Tasks){Assigned+=T.Assigned>=0;Distance+=T.Distance;}Finish(Good&&Wheels==24&&Assigned==6&&Energy>0&&Distance>.15,TEXT("six independent physical/sensor rigs, 24 driven wheels, six actual assignments and observed motion/energy; not six-vehicle long-run throughput"));return;}
 }
 else if(TestCase.StartsWith(TEXT("ops_charge")))
 {
  auto& B=Batteries[V->VehicleId];auto* J=Job(V->VehicleId);
  if(TestCase==TEXT("ops_charge_interlock"))
  {
   if(TestStage==1&&J&&J->State==TEXT("charging")&&B.ChargeSeconds>.2){V->VehicleGuard->Sensor.ManualOutage=true;TestStage=2;StageStarted=Age;TestDepth=B.ChargedWh;}
   if(TestStage==2&&Age-StageStarted>1){Finish(J&&J->Held()&&!B.Charging&&B.ChargedWh-TestDepth<.3&&TaskBoard.Tasks[0].Pending()&&FMath::Abs(V->SpeedMps)<.03,TEXT("real sensor outage opens virtual charging contactor, holds charger/vehicle and prevents pending transport dispatch"));return;}
  }
  else if(CompletedCharges==1&&TaskBoard.Tasks[0].Assigned>=0&&TaskBoard.Tasks[0].Distance>.15){bool Clear=true;for(const auto& C:Chargers)Clear&=C.Owner!=V->VehicleId;Finish(Clear&&B.ChargedWh>0&&B.ConsumedWh>0&&B.Sessions==1,TEXT("low-SOC vehicle drives to reserved charger, charges only at measured stationary dock, releases charger and receives queued transport"));return;}
  if(J&&J->Held()&&TestStage==1){Finish(false,J->Reason);return;}
 }
 else if(TestCase==TEXT("ops_critical")&&Age>6)
 {
  const auto& B=Batteries[V->VehicleId];bool Zero=true;for(const AWheelActor* W:V->Wheels)Zero&=FMath::Abs(W->DriveTorqueNm)<.1;
  Finish(B.CriticalLatched&&!ReadyForTask(*V)&&TaskBoard.Tasks[0].Pending()&&Zero&&FMath::Abs(V->SpeedMps)<.03,TEXT("critical SOC latches drive hold and forbids both new transport and automatic fault recovery"));return;
 }
 if(Age>120)Finish(false,TEXT("operations_qualification_timeout"));
}
