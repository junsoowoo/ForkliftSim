#include "CampusOperations.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "HandlingControl.h"
#include "DynamicsSensors.h"
#include "WebTelemetry.h"
#include "WebTaskInbox.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/FileManager.h"
#include "Serialization/JsonSerializer.h"
namespace
{
 int64 UtcMs(){const auto T=FDateTime::UtcNow();return T.ToUnixTimestamp()*1000+T.GetMillisecond();}
 TSharedRef<FJsonObject> TaskJson(const WarehouseTasks::FTask& T,double Now)
 {
  auto O=MakeShared<FJsonObject>();
  struct S{const TCHAR* K;const FString* V;};for(const auto& F:TArray<S>{{TEXT("id"),&T.Id},{TEXT("load_id"),&T.Load},{TEXT("destination"),&T.Destination},{TEXT("vehicle_id"),&T.Vehicle},{TEXT("job_id"),&T.Job},{TEXT("state"),&T.State},{TEXT("reason"),&T.Reason},{TEXT("phase"),&T.Phase}})O->SetStringField(F.K,*F.V);
  struct N{const TCHAR* K;double V;};for(const auto& F:TArray<N>{{TEXT("priority"),double(T.Priority)},{TEXT("tier"),double(T.Tier)},{TEXT("created_s"),T.Created},{TEXT("assigned_s"),T.Assigned},{TEXT("ended_s"),T.Ended},{TEXT("elapsed_s"),T.Elapsed},{TEXT("queue_wait_s"),T.Pending()?Now-T.Created:T.QueueWait},{TEXT("traffic_wait_s"),T.TrafficWait},{TEXT("held_s"),T.Held},{TEXT("travel_s"),T.Travel},{TEXT("handling_s"),T.Handling},{TEXT("distance_est_m"),T.Distance},{TEXT("velocity_residual_s"),T.SlipS},{TEXT("pose_gap_s"),T.PoseGapS},{TEXT("peak_fork_load_n"),T.PeakForkN},{TEXT("stop_count"),double(T.Stops)},{TEXT("recovery_count"),double(T.Recoveries)},{TEXT("replans"),double(T.Replans)},{TEXT("tyre_mu_min"),T.Assigned>=0?T.TyreMuMin:0},{TEXT("tyre_mu_max"),T.TyreMuMax},{TEXT("road_mu_min"),T.Assigned>=0?T.RoadMuMin:0},{TEXT("road_mu_max"),T.RoadMuMax}})O->SetNumberField(F.K,F.V);
  O->SetBoolField(TEXT("recovery_required"),T.RecoveryRequired);O->SetBoolField(TEXT("pickup_receipt"),T.Picked);O->SetBoolField(TEXT("dropoff_receipt"),T.Dropped);return O;
 }
 FString Quote(FString V){V.ReplaceInline(TEXT("\""),TEXT("\"\""));return TEXT("\"")+V+TEXT("\"");}
}
void ACampusOperations::LoadTaskConfig()
{
 TaskBoard.Run=FGuid::NewGuid().ToString(EGuidFormats::Digits);TaskFolder=FPaths::ProjectSavedDir()/TEXT("Operations")/TaskBoard.Run;
 FString ConfigPath=FPaths::ProjectConfigDir()/TEXT("CampusOperations.json"),Override;
 if(FParse::Value(FCommandLine::Get(),TEXT("CampusScenario="),Override))ConfigPath=FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(),Override);
 FString Text;TSharedPtr<FJsonObject> J;
 if(!FFileHelper::LoadFileToString(Text,*ConfigPath)||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J){Error=TEXT("Cannot read CampusOperations.json");return;}
 double Cargo=400;bool Auto=true;
 if(!J->TryGetStringField(TEXT("scenario_id"),TaskBoard.Scenario)||TaskBoard.Scenario.IsEmpty()||TaskBoard.Scenario.Len()>64||!J->TryGetNumberField(TEXT("cargo_mass_kg"),Cargo)||!J->TryGetNumberField(TEXT("tyre_mu"),ScenarioTyre)||!J->TryGetNumberField(TEXT("road_mu"),ScenarioRoad)||!J->TryGetBoolField(TEXT("auto_dispatch"),Auto)||!FMath::IsFinite(Cargo)||Cargo<1||Cargo>1000||!FMath::IsFinite(ScenarioTyre)||ScenarioTyre<0||ScenarioTyre>2||!FMath::IsFinite(ScenarioRoad)||ScenarioRoad<0||ScenarioRoad>2){Error=TEXT("Invalid campus experiment configuration");return;}
 BoardAuto=Auto;if(BoardEnabled())PayloadSpec.CargoMassKg=Cargo;
}
bool ACampusOperations::AddTasks(const TArray<WarehouseTasks::FRequest>& Requests,TArray<FString>& Ids,FString& Why)
{
 if(!BoardEnabled()){Why=TEXT("task_board_disabled_in_legacy_fixture");return false;}
 for(const auto& R:Requests){const auto* L=Inventory.Loads.Find(R.Load);if(!L||L->Moving||(!R.Destination.IsEmpty()&&(!Inventory.Slots.Contains(R.Destination)||R.Destination==L->Slot))){Why=TEXT("unknown_busy_load_or_invalid_destination:")+R.Load;return false;}
  if(!R.PreferredVehicle.IsEmpty()&&!Vehicles.ContainsByPredicate([&](const auto& V){return V->VehicleId==R.PreferredVehicle;})){Why=TEXT("unknown_preferred_vehicle");return false;}
  for(const auto& P:Jobs)if(P.Value.Engaged&&P.Value.Load==R.Load){Why=TEXT("load_already_in_vehicle_job");return false;}}
 if(!TaskBoard.Add(Requests,Age,Ids,Why))return false;Why=FString::Printf(TEXT("Registered %d tasks; waiting for eligible vehicle"),Ids.Num());SaveTasks();return true;
}
void ACampusOperations::ConsumeTaskCommands()
{
 if(!TaskInbox)return;TaskInbox->Poll();auto Commands=MoveTemp(TaskInbox->Commands);TaskInbox->Commands.Reset();
 for(const auto& J:Commands)
 {
  FString Id,World,Run,Kind,Why;double Expires=0;TArray<FString> Ids;bool Accepted=false;
  if(!J->TryGetStringField(TEXT("request_id"),Id)||Id.IsEmpty()||Id.Len()>100)continue;
  if(SeenCommands.Contains(Id))continue;
  if(!J->TryGetStringField(TEXT("world_id"),World)||World!=FleetWeb->GetWorldId()||!J->TryGetStringField(TEXT("run_id"),Run)||Run!=TaskBoard.Run)continue;
  if(!J->TryGetNumberField(TEXT("expires_at_ms"),Expires)||Expires<UtcMs())Why=TEXT("command_expired");
  else if(J->TryGetStringField(TEXT("kind"),Kind)&&Kind==TEXT("enqueue"))
  {
   const TArray<TSharedPtr<FJsonValue>>* A=nullptr;TArray<WarehouseTasks::FRequest> Requests;bool Valid=J->TryGetArrayField(TEXT("tasks"),A)&&A->Num()>0&&A->Num()<=10;
   if(Valid)for(const auto& V:*A){if(V->Type!=EJson::Object){Valid=false;break;}auto O=V->AsObject();WarehouseTasks::FRequest R;double Priority=0,Tier=1;
    if(!O->TryGetStringField(TEXT("load_id"),R.Load)||!O->TryGetStringField(TEXT("destination"),R.Destination)||!O->TryGetNumberField(TEXT("priority"),Priority)||Priority!=FMath::FloorToDouble(Priority)||Priority<0||Priority>9||!O->TryGetNumberField(TEXT("tier"),Tier)||(Tier!=1&&Tier!=2)){Valid=false;break;}R.Priority=int(Priority);R.Tier=int(Tier);Requests.Add(R);}
   if(Valid)Accepted=AddTasks(Requests,Ids,Why);else Why=TEXT("invalid_batch_schema");
  }
  else if(Kind==TEXT("cancel_pending")){FString IdToCancel;if(J->TryGetStringField(TEXT("task_id"),IdToCancel))Accepted=TaskBoard.CancelPending(IdToCancel,Age,Why);else Why=TEXT("missing_task_id");}
  else Why=TEXT("unsupported_command");
  auto Receipt=MakeShared<FJsonObject>();Receipt->SetStringField(TEXT("request_id"),Id);Receipt->SetStringField(TEXT("status"),Accepted?TEXT("accepted"):TEXT("rejected"));Receipt->SetStringField(TEXT("reason"),Why);TArray<TSharedPtr<FJsonValue>> A;for(const auto& T:Ids)A.Add(MakeShared<FJsonValueString>(T));Receipt->SetArrayField(TEXT("task_ids"),A);SeenCommands.Add(Id,Receipt);CommandReceipts.Add(MakeShared<FJsonValueObject>(Receipt));
  if(CommandReceipts.Num()>512)CommandReceipts.RemoveAt(0);SaveTasks();
 }
}
void ACampusOperations::ServiceTasks(double Dt)
{
 if(!BoardEnabled())return;ConsumeTaskCommands();
 for(auto& T:TaskBoard.Tasks)if(T.Assigned>=0&&(!T.Terminal()||T.RecoveryRequired))
 {
  auto* J=Jobs.Find(T.Vehicle);AStabilityLab* V=nullptr;for(AStabilityLab* Candidate:Vehicles)if(Candidate->VehicleId==T.Vehicle)V=Candidate;
  if(!J||!V||J->Id!=T.Job)continue;WarehouseTasks::FSample S;S.Now=Age;S.Dt=Dt;S.TrafficWaiting=J->TrafficWaiting;S.State=J->State;S.Reason=J->Reason;S.Engaged=J->Engaged;S.Picked=J->PickupReceipt;S.Dropped=J->DropoffReceipt;T.Replans=J->Replans;
  if(V->DriveSensors){const auto& D=V->DriveSensors->Model;S.Localized=D.Localized&&V->TimeS-D.LastMatchS<1.5;S.GroundFresh=V->DriveSensors->GroundFresh(V->TimeS);S.WheelSpeed=FVector2D(D.Velocity).Size();S.GroundSpeed=FVector2D(V->DriveSensors->MeasuredVelocity(V->TimeS)).Size();S.VelocityResidual=(FVector2D(D.Velocity)-FVector2D(V->DriveSensors->MeasuredVelocity(V->TimeS))).Size();}
  if(V->DynamicsSensors&&V->DynamicsSensors->Model.Usable(2,V->TimeS)&&V->DynamicsSensors->Model.Usable(3,V->TimeS))S.ForkLoad=V->DynamicsSensors->Model.Channels[2].Last.Value[0]+V->DynamicsSensors->Model.Channels[3].Last.Value[0];S.TyreMu=V->Settings.TyreMu;S.RoadMu=V->Settings.RoadMu;TaskBoard.Observe(T,S);
 }
 if(BoardAuto&&Age>=NextDispatch)
 {
  NextDispatch=Age+.5;PlanningBudget=8;ScheduleCharges();auto Order=TaskBoard.PendingOrder();
  if(!Order.IsEmpty())for(AStabilityLab* V:Vehicles)if(ReadyForTask(*V))Inventory.Release(V->CampusLoadId,V->VehicleId);
  for(int Index:Order)
  {
   auto& T=TaskBoard.Tasks[Index];AStabilityLab* Best=nullptr;CampusFleet::FJob BestJob;double Score=1e30;FString Why=TEXT("no_healthy_stationary_empty_vehicle");
   for(AStabilityLab* V:Vehicles)if(ReadyForTask(*V)&&(T.PreferredVehicle.IsEmpty()||T.PreferredVehicle==V->VehicleId))
   {
    CampusFleet::FJob Plan;FString CandidateReason;
    if(!BuildJob(*V,T.Load,T.Destination,T.Tier==2,Plan,CandidateReason)){Why=CandidateReason;continue;}
    const double Cost=(Plan.ToPick.Length+Plan.ToReturn.Length)/CampusFleet::EmptyCruiseMps
        +Plan.ToDrop.Length/CampusFleet::LoadedCruiseMps+90.;
    if(Cost<Score){Score=Cost;Best=V;BestJob=Plan;}
   }
   if(Best&&CommitJob(*Best,BestJob,Why)){auto* J=Job(Best->VehicleId);TaskBoard.Assign(Index,Best->VehicleId,J->Id,J->Destination,Age);}else T.Reason=Why;
  }
 }
 // Reserve eligible queued jobs in task priority/FIFO order, rather than actor tick order.
 TArray<int> ClaimOrder;for(int I=0;I<TaskBoard.Tasks.Num();++I){const auto& T=TaskBoard.Tasks[I];const auto* J=Jobs.Find(T.Vehicle);if(T.Assigned>=0&&J&&J->Id==T.Job&&J->State==TEXT("queued"))ClaimOrder.Add(I);}
 ClaimOrder.Sort([&](int A,int B){return TaskBoard.Tasks[A].Priority==TaskBoard.Tasks[B].Priority?A<B:TaskBoard.Tasks[A].Priority>TaskBoard.Tasks[B].Priority;});
 for(int I:ClaimOrder){const auto& T=TaskBoard.Tasks[I];const auto& J=Jobs[T.Vehicle];for(AStabilityLab* V:Vehicles)if(V->VehicleId==T.Vehicle)ClaimRoute(*V,J);}
 if(Age>=NextTaskSave){NextTaskSave=Age+1;SaveTasks();}
}
TSharedRef<FJsonObject> ACampusOperations::TasksTelemetry() const
{
 auto J=MakeShared<FJsonObject>();J->SetStringField(TEXT("source"),TEXT("unreal_task_board_v1"));J->SetStringField(TEXT("qualification_case"),FleetTest?TestCase:TEXT(""));J->SetNumberField(TEXT("schema"),1);J->SetStringField(TEXT("run_id"),TaskBoard.Run);J->SetStringField(TEXT("scenario_id"),TaskBoard.Scenario);J->SetBoolField(TEXT("enabled"),BoardEnabled());J->SetBoolField(TEXT("auto_dispatch"),BoardAuto);J->SetNumberField(TEXT("sim_s"),Age);J->SetNumberField(TEXT("cargo_mass_kg"),PayloadSpec.CargoMassKg);J->SetNumberField(TEXT("tyre_mu_initial"),ScenarioTyre);J->SetNumberField(TEXT("road_mu_initial"),ScenarioRoad);J->SetStringField(TEXT("velocity_residual_definition"),TEXT("Time above 0.05 m/s difference between wheel and scan-derived body velocity vectors. Includes sensor/filter lag, not direct tyre-contact slip."));J->SetStringField(TEXT("distance_definition"),TEXT("Integral of fresh LiDAR-registration speed; excludes gaps and low-speed noise. Not ground-truth distance."));J->SetStringField(TEXT("assignment_policy"),TEXT("priority descending, FIFO; minimum nominal route travel time among eligible vehicles; fixed handling allowance, not learned/OEM cycle prediction"));
 TArray<TSharedPtr<FJsonValue>> A;double Cycle=0,Lead=0,Wait=0,Distance=0,Held=0,Slip=0;int Complete=0,Failed=0,Pending=0,Active=0,Cancelled=0,Interrupted=0,Stops=0;
 for(const auto& T:TaskBoard.Tasks){A.Add(MakeShared<FJsonValueObject>(TaskJson(T,Age)));Complete+=T.State==TEXT("completed");Failed+=T.State==TEXT("failed");Pending+=T.Pending();Cancelled+=T.State==TEXT("cancelled");Interrupted+=T.State==TEXT("interrupted");Active+=!T.Terminal()&&!T.Pending();if(T.State==TEXT("completed")){Cycle+=T.Ended-T.Assigned;Lead+=T.Ended-T.Created;Wait+=T.QueueWait+T.TrafficWait;}Distance+=T.Distance;Held+=T.Held;Slip+=T.SlipS;Stops+=T.Stops;}
 J->SetArrayField(TEXT("tasks"),A);auto K=MakeShared<FJsonObject>();struct N{const TCHAR* K;double V;};for(const auto& F:TArray<N>{{TEXT("submitted"),double(TaskBoard.Tasks.Num())},{TEXT("pending"),double(Pending)},{TEXT("active"),double(Active)},{TEXT("completed"),double(Complete)},{TEXT("failed"),double(Failed)},{TEXT("cancelled"),double(Cancelled)},{TEXT("interrupted"),double(Interrupted)},{TEXT("stop_events"),double(Stops)},{TEXT("distance_est_m"),Distance},{TEXT("held_s"),Held},{TEXT("velocity_residual_s"),Slip},{TEXT("throughput_per_sim_hour"),Age>0?Complete*3600./Age:0}})K->SetNumberField(F.K,F.V);
 if(Complete){K->SetNumberField(TEXT("mean_cycle_s"),Cycle/Complete);K->SetNumberField(TEXT("mean_lead_s"),Lead/Complete);K->SetNumberField(TEXT("mean_wait_s"),Wait/Complete);}if(Complete+Failed)K->SetNumberField(TEXT("resolved_success_fraction"),double(Complete)/(Complete+Failed));J->SetObjectField(TEXT("kpi"),K);
 A.Reset();for(AStabilityLab* V:Vehicles){double Busy=0,Waiting=0;for(const auto& T:TaskBoard.Tasks)if(T.Vehicle==V->VehicleId){Busy+=T.Travel+T.Handling;Waiting+=T.TrafficWait+T.Held;}auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("vehicle_id"),V->VehicleId);O->SetNumberField(TEXT("busy_s"),Busy);O->SetNumberField(TEXT("wait_s"),Waiting);O->SetNumberField(TEXT("utilization_fraction"),Age>0?FMath::Clamp(Busy/Age,0.,1.):0);A.Add(MakeShared<FJsonValueObject>(O));}J->SetArrayField(TEXT("vehicles"),A);
 A.Reset();for(const auto& Id:LoadOrder){const auto& L=Inventory.Loads[Id];auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("id"),Id);O->SetStringField(TEXT("slot"),L.Slot);O->SetBoolField(TEXT("moving"),L.Moving);O->SetStringField(TEXT("reserved_by"),L.ReservedBy);A.Add(MakeShared<FJsonValueObject>(O));}J->SetArrayField(TEXT("loads"),A);
 A.Reset();for(int I=FMath::Max(0,TaskBoard.Events.Num()-100);I<TaskBoard.Events.Num();++I){const auto& E=TaskBoard.Events[I];auto O=MakeShared<FJsonObject>();O->SetNumberField(TEXT("sim_s"),E.Time);O->SetStringField(TEXT("task_id"),E.Task);O->SetStringField(TEXT("state"),E.State);O->SetStringField(TEXT("reason"),E.Reason);A.Add(MakeShared<FJsonValueObject>(O));}J->SetArrayField(TEXT("events"),A);J->SetArrayField(TEXT("command_receipts"),CommandReceipts);J->SetObjectField(TEXT("energy"),EnergyTelemetry());J->SetStringField(TEXT("traffic_policy"),Traffic.Sections?TEXT("section_windows"):TEXT("whole_job"));J->SetNumberField(TEXT("peak_measured_moving"),PeakMoving);return J;
}
void ACampusOperations::SaveTasks()
{
 if(TaskFolder.IsEmpty()||!BoardEnabled())return;IFileManager::Get().MakeDirectory(*TaskFolder,true);
 if(LastEnergySave<0||Age-LastEnergySave>=.99){const FString P=TaskFolder/TEXT("energy.csv");FString E;if(!IFileManager::Get().FileExists(*P))E=TEXT("run_id,sim_s,vehicle_id,mode,soc,stored_wh,consumed_wh,charged_wh,power_w,charging,critical_latched,charge_sessions\n");for(AStabilityLab* V:Vehicles)if(const auto* B=Batteries.Find(V->VehicleId))E+=FString::Printf(TEXT("%s,%.8f,%s,%s,%.10f,%.8f,%.8f,%.8f,%.4f,%d,%d,%d\n"),*TaskBoard.Run,Age,*V->VehicleId,*B->Mode,B->Soc(BatterySpec),B->StoredWh,B->ConsumedWh,B->ChargedWh,B->PowerW,B->Charging?1:0,B->CriticalLatched?1:0,B->Sessions);FFileHelper::SaveStringToFile(E,*P,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,&IFileManager::Get(),FILEWRITE_Append);LastEnergySave=Age;}
FString Text;FJsonSerializer::Serialize(TasksTelemetry(),TJsonWriterFactory<>::Create(&Text));FFileHelper::SaveStringToFile(Text,*(TaskFolder/TEXT("session.json")),FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
 FString Csv=TEXT("run_id,scenario_id,task_id,load_id,destination,vehicle_id,state,reason,priority,created_s,assigned_s,ended_s,queue_wait_s,traffic_wait_s,held_s,travel_s,handling_s,distance_est_m,velocity_residual_s,pose_gap_s,peak_fork_load_n,stop_count,recovery_count,replans,cargo_mass_kg,tyre_mu_min,tyre_mu_max,road_mu_min,road_mu_max\n");
 for(const auto& T:TaskBoard.Tasks){TArray<FString> R={Quote(TaskBoard.Run),Quote(TaskBoard.Scenario),Quote(T.Id),Quote(T.Load),Quote(T.Destination),Quote(T.Vehicle),Quote(T.State),Quote(T.Reason)};for(double V:{double(T.Priority),T.Created,T.Assigned,T.Ended,T.Pending()?Age-T.Created:T.QueueWait,T.TrafficWait,T.Held,T.Travel,T.Handling,T.Distance,T.SlipS,T.PoseGapS,T.PeakForkN,double(T.Stops),double(T.Recoveries),double(T.Replans),PayloadSpec.CargoMassKg,T.Assigned>=0?T.TyreMuMin:0,T.TyreMuMax,T.Assigned>=0?T.RoadMuMin:0,T.RoadMuMax})R.Add(FString::Printf(TEXT("%.8f"),V));Csv+=FString::Join(R,TEXT(","))+TEXT("\n");}FFileHelper::SaveStringToFile(Csv,*(TaskFolder/TEXT("tasks.csv")),FFileHelper::EEncodingOptions::ForceUTF8);
 Text=TEXT("run_id,sim_s,task_id,state,reason\n");for(const auto& E:TaskBoard.Events)Text+=Quote(TaskBoard.Run)+TEXT(",")+FString::SanitizeFloat(E.Time)+TEXT(",")+Quote(E.Task)+TEXT(",")+Quote(E.State)+TEXT(",")+Quote(E.Reason)+TEXT("\n");FFileHelper::SaveStringToFile(Text,*(TaskFolder/TEXT("events.csv")),FFileHelper::EEncodingOptions::ForceUTF8);
}
