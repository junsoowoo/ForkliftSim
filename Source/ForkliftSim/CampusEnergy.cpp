#include "CampusOperations.h"
#include "StabilityLab.h"
#include "WheelActor.h"
#include "ForkActor.h"
#include "JointSensors.h"
#include "DynamicsSensors.h"
#include "LidarSafety.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Serialization/JsonSerializer.h"
void ACampusOperations::LoadOperationsConfig()
{
 FString Text,Policy;TSharedPtr<FJsonObject> J;
 if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("FleetOperations.json")))||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J){Error=TEXT("Cannot read FleetOperations.json");return;}
 double Count=0;const TSharedPtr<FJsonObject>* B=nullptr;
 if(!J->TryGetNumberField(TEXT("vehicle_count"),Count)||Count!=FMath::FloorToDouble(Count)||Count<3||Count>6||!J->TryGetStringField(TEXT("traffic_policy"),Policy)||(Policy!=TEXT("whole_job")&&Policy!=TEXT("section_windows"))||!J->TryGetObjectField(TEXT("battery"),B)){Error=TEXT("Invalid fleet count/traffic policy/battery config");return;}
 auto& S=BatterySpec;bool Good=(*B)->TryGetBoolField(TEXT("enabled"),S.Enabled);
 struct F{const TCHAR* Key;double* Value;};for(auto Field:TArray<F>{{TEXT("nominal_voltage_v"),&S.Voltage},{TEXT("capacity_ah"),&S.Ah},{TEXT("initial_soc"),&S.Initial},{TEXT("charge_below_soc"),&S.Low},{TEXT("resume_soc"),&S.Resume},{TEXT("critical_soc"),&S.Critical},{TEXT("auxiliary_w"),&S.AuxW},{TEXT("charger_ac_w"),&S.ChargeW},{TEXT("charger_efficiency"),&S.Efficiency}})Good&=(*B)->TryGetNumberField(Field.Key,*Field.Value);
 if(!Good||!S.Valid()){Error=TEXT("Invalid fleet battery coefficients");return;}
 int Override=int(Count);FParse::Value(FCommandLine::Get(),TEXT("CampusVehicles="),Override);FParse::Value(FCommandLine::Get(),TEXT("CampusTraffic="),Policy);
 if(Override<3||Override>6||Override>ParkingStartM.Num()||(Policy!=TEXT("whole_job")&&Policy!=TEXT("section_windows"))){Error=TEXT("Invalid campus fleet override");return;}
 if(BoardEnabled()){InitialVehicles=Override;Traffic.Sections=Policy==TEXT("section_windows");}else S.Enabled=false;
 if(TestCase==TEXT("ops_parallel_whole"))Traffic.Sections=false;
}
bool ACampusOperations::BatteryReady(const AStabilityLab& V) const
{const auto* B=Batteries.Find(V.VehicleId);return !BatterySpec.Enabled||(B&&B->Dispatch(BatterySpec));}
void ACampusOperations::ServiceEnergy(double Dt)
{
 for(AStabilityLab* V:Vehicles)
 {
  auto* B=Batteries.Find(V->VehicleId);if(!B)continue;double MotorJ=0;for(const AWheelActor* W:V->Wheels)MotorJ+=W->DcConsumedJ;
  const auto* J=Jobs.Find(V->VehicleId);bool Docked=J&&J->Engaged&&!J->Held()&&J->Kind==TEXT("charge")&&J->State==TEXT("charging");
  if(Docked)
  {
   const auto& D=V->DriveSensors->Model;const auto P=D.MapPose(V->TimeS);
   Docked=D.Ready&&D.Localized&&V->TimeS-D.LastMatchS<.5&&V->DriveSensors->Stationary(V->TimeS,.02)&&FVector2D(P.X-J->Pickup.X,P.Y-J->Pickup.Y).Size()<.08&&FMath::Abs(P.Yaw)<.025&&V->JointSensors->Model.Usable(4,V->TimeS)&&V->JointSensors->Model.Channels[4].Last.Value<.1&&V->DynamicsSensors->Model.Usable(2,V->TimeS)&&V->DynamicsSensors->Model.Usable(3,V->TimeS)&&V->DynamicsSensors->Model.Channels[2].Last.Value[0]+V->DynamicsSensors->Model.Channels[3].Last.Value[0]<100&&!V->VehicleGuard->Decision.Faulted;
   bool Owned=false;for(const auto& C:Chargers)Owned|=C.Id==J->Charger&&C.Owner==V->VehicleId;Docked&=Owned;
  }
  B->Step(BatterySpec,Dt,MotorJ,V->Fork->HydraulicState.DcJ,Docked);if(J&&J->Engaged&&J->Kind==TEXT("charge")&&!J->Held()&&!Docked&&!B->CriticalLatched)B->Mode=TEXT("to_charger");
 }
}
void ACampusOperations::ScheduleCharges()
{
 if(!BatterySpec.Enabled||!BoardAuto)return;
 for(AStabilityLab* V:Vehicles)
 {
  auto& B=Batteries[V->VehicleId];if(B.Soc(BatterySpec)>BatterySpec.Low||B.CriticalLatched||!ReadyForTask(*V,true))continue;
  const auto P=V->DriveSensors->Model.MapPose(V->TimeS);const FVector2D Here(P.X,P.Y);int Best=-1;double Cost=1e30;Mission::FPlan Plan;
  for(int I=0;I<Chargers.Num();++I)
  {
   const auto& C=Chargers[I];if(!C.Owner.IsEmpty())continue;bool Clear=true;
   for(const AStabilityLab* Peer:Vehicles)if(Peer!=V){const auto& D=Peer->DriveSensors->Model;if(!D.Ready||!D.Localized||Peer->TimeS-D.LastMatchS>1.5){Clear=false;break;}const auto Q=D.MapPose(Peer->TimeS);if(FVector2D(Q.X-C.Position.X,Q.Y-C.Position.Y).Size()<4.7){Clear=false;break;}}
   if(!Clear)continue;auto Candidate=AssignmentPlan(Here,C.Position);if(Candidate.Valid&&Candidate.Length<Cost){Cost=Candidate.Length;Best=I;Plan=Candidate;}
  }
  if(Best<0){B.Mode=TEXT("waiting_charger");B.Reason=TEXT("no_free_reachable_charger");continue;}
  auto& C=Chargers[Best];C.Owner=V->VehicleId;CampusFleet::FJob J;J.Kind=TEXT("charge");J.Charger=C.Id;J.Vehicle=V->VehicleId;J.Id=FString::Printf(TEXT("CHARGE-%04d"),++JobNumber);J.Pickup=C.Position;J.ToPick=Plan;J.Started=Age;J.Engaged=true;Jobs.Add(V->VehicleId,J);Inventory.Release(V->CampusLoadId,V->VehicleId);++B.Sessions;B.Mode=TEXT("to_charger");B.Reason.Empty();
 }
}
bool ACampusOperations::UpdateCharge(AStabilityLab& V,CampusFleet::FJob& J,double Dt)
{
 if(J.Kind!=TEXT("charge")||J.State==TEXT("queued")||J.Travelling())return false;
 auto& B=Batteries[V.VehicleId];const auto P=V.DriveSensors->Model.MapPose(V.TimeS);
 if(J.State==TEXT("align_charge"))
 {
  J.YawRate=FMath::Clamp(-P.Yaw,-.15,.15);J.Brake=FMath::Abs(P.Yaw)<.02;V.LiftTargetM=0;
  if(J.Brake&&V.DriveSensors->Stationary(V.TimeS,.02)&&V.JointSensors->Model.Channels[4].Last.Value<.1){J.StableS+=Dt;if(J.StableS>.5){J.State=TEXT("charging");J.StageS=Age;J.StableS=0;}}else J.StableS=0;return true;
 }
 if(J.State==TEXT("charging"))
 {
  if(B.Soc(BatterySpec)>=BatterySpec.Resume){J.State=TEXT("complete");J.Engaged=false;Traffic.Release(V.VehicleId,true);for(auto& C:Chargers)if(C.Id==J.Charger&&C.Owner==V.VehicleId)C.Owner.Empty();++CompletedCharges;B.Mode=TEXT("available");B.Reason.Empty();return true;}
  if(!B.Charging&&Age-J.StageS>1){J.Hold(TEXT("charger_docking_interlock_lost"),true);B.Reason=J.Reason;}return true;
 }
 return true;
}
TSharedRef<FJsonObject> ACampusOperations::EnergyTelemetry() const
{
 auto J=MakeShared<FJsonObject>();J->SetStringField(TEXT("source"),TEXT("unreal_motor_hydraulic_energy_v1"));J->SetBoolField(TEXT("enabled"),BatterySpec.Enabled);J->SetNumberField(TEXT("capacity_wh"),BatterySpec.CapacityWh());J->SetNumberField(TEXT("nominal_voltage_v"),BatterySpec.Voltage);J->SetNumberField(TEXT("charge_below_soc"),BatterySpec.Low);J->SetNumberField(TEXT("resume_soc"),BatterySpec.Resume);J->SetNumberField(TEXT("completed_charges"),CompletedCharges);J->SetStringField(TEXT("scope"),TEXT("Integrated positive motor DC and pump DC energy plus assumed auxiliaries; constant nominal voltage. No regenerative recovery, cell/BMS/thermal chemistry or real charger communications."));
 TArray<TSharedPtr<FJsonValue>> A;for(AStabilityLab* V:Vehicles)if(const auto* B=Batteries.Find(V->VehicleId))
 {auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("vehicle_id"),V->VehicleId);O->SetStringField(TEXT("mode"),B->Mode);O->SetStringField(TEXT("reason"),B->Reason);O->SetNumberField(TEXT("soc"),B->Soc(BatterySpec));O->SetNumberField(TEXT("stored_wh"),B->StoredWh);O->SetNumberField(TEXT("consumed_wh"),B->ConsumedWh);O->SetNumberField(TEXT("charged_wh"),B->ChargedWh);O->SetNumberField(TEXT("unserved_wh"),B->UnservedWh);O->SetNumberField(TEXT("power_w"),B->PowerW);O->SetNumberField(TEXT("charge_s"),B->ChargeSeconds);O->SetNumberField(TEXT("charge_sessions"),B->Sessions);O->SetBoolField(TEXT("charging"),B->Charging);O->SetBoolField(TEXT("critical_latched"),B->CriticalLatched);A.Add(MakeShared<FJsonValueObject>(O));}J->SetArrayField(TEXT("vehicles"),A);
 A.Reset();for(const auto& C:Chargers){auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("id"),C.Id);O->SetStringField(TEXT("owner"),C.Owner);O->SetArrayField(TEXT("position_m"),{MakeShared<FJsonValueNumber>(C.Position.X),MakeShared<FJsonValueNumber>(C.Position.Y)});A.Add(MakeShared<FJsonValueObject>(O));}J->SetArrayField(TEXT("chargers"),A);return J;
}
