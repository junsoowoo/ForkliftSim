#include "CampusOperations.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "WebTelemetry.h"
#include "HandlingControl.h"
#include "LidarSafety.h"
#include "Dom/JsonObject.h"
TSharedRef<FJsonObject> ACampusOperations::FleetTelemetry() const
{
    auto F=MakeShared<FJsonObject>();F->SetObjectField(TEXT("task_board"),TasksTelemetry());F->SetStringField(TEXT("kind"),TEXT("campus_fleet_v1"));F->SetStringField(TEXT("selected_vehicle"),Active()?Active()->VehicleId:TEXT(""));F->SetStringField(TEXT("reservation_policy"),Traffic.Sections?TEXT("section_windows; current and exit legs atomically reserved; passed tail released; occupied/stale footprints retained"):TEXT("atomic_whole_job_corridors; conservative concurrency; stale claims quarantine"));F->SetNumberField(TEXT("sim_s"),Age);
    TArray<TSharedPtr<FJsonValue>> A;
    for(AStabilityLab* V:Vehicles)if(V&&V->Chassis)
    {
        auto O=V->BuildWebFrame();O->SetStringField(TEXT("vehicle_id"),V->VehicleId);O->SetStringField(TEXT("target_load_id"),V->CampusLoadId);
        auto J=MakeShared<FJsonObject>();if(const auto* Job=Jobs.Find(V->VehicleId))
        {J->SetStringField(TEXT("kind"),Job->Kind);J->SetStringField(TEXT("charger_id"),Job->Charger);J->SetBoolField(TEXT("traffic_waiting"),Job->TrafficWaiting);J->SetStringField(TEXT("id"),Job->Id);J->SetStringField(TEXT("state"),Job->State);J->SetStringField(TEXT("reason"),Job->Reason);J->SetStringField(TEXT("load_id"),Job->Load);J->SetStringField(TEXT("destination"),Job->Destination);J->SetBoolField(TEXT("engaged"),Job->Engaged);J->SetBoolField(TEXT("carrying"),Job->Carrying);J->SetBoolField(TEXT("pickup_receipt"),Job->PickupReceipt);J->SetBoolField(TEXT("dropoff_receipt"),Job->DropoffReceipt);J->SetNumberField(TEXT("waypoint"),Job->Waypoint);J->SetNumberField(TEXT("replans"),Job->Replans);J->SetNumberField(TEXT("elapsed_s"),Age-Job->Started);J->SetNumberField(TEXT("waiting_s"),Job->State==TEXT("queued")?Age-Job->Started:0);J->SetNumberField(TEXT("command_lease_remaining_s"),FMath::Max(0.,Job->LeaseUntil-V->TimeS));TArray<TSharedPtr<FJsonValue>> P;const FString Stage=Job->Held()?Job->ResumeState:Job->State;const bool Picking=Stage==TEXT("queued")||Stage==TEXT("to_pickup")||Stage==TEXT("to_charger")||Stage==TEXT("align_pickup")||Stage==TEXT("pickup");const bool Returning=Stage==TEXT("align_return")||Stage==TEXT("returning")||Stage==TEXT("complete");J->SetStringField(TEXT("path_phase"),Picking?TEXT("pickup"):Returning?TEXT("return"):TEXT("dropoff"));const auto& R=Picking?Job->ToPick:Returning?Job->ToReturn:Job->ToDrop;for(auto Vtx:R.Points)P.Add(MakeShared<FJsonValueArray>(TArray<TSharedPtr<FJsonValue>>{MakeShared<FJsonValueNumber>(Vtx.X),MakeShared<FJsonValueNumber>(Vtx.Y)}));J->SetArrayField(TEXT("path_m"),P);}
        else J->SetStringField(TEXT("state"),TEXT("manual_idle"));O->SetObjectField(TEXT("campus_job"),J);O->SetStringField(TEXT("scenario"),TEXT("campus_fleet"));A.Add(MakeShared<FJsonValueObject>(O));
    }F->SetArrayField(TEXT("vehicles"),A);
    A.Reset();for(const auto& Pair:Inventory.Slots){const auto& S=Pair.Value;auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("id"),S.Id);O->SetStringField(TEXT("pallet_id"),S.Pallet);O->SetStringField(TEXT("reserved_by"),S.ReservedBy);O->SetBoolField(TEXT("blocked"),S.Blocked);O->SetArrayField(TEXT("center_m"),{MakeShared<FJsonValueNumber>(S.CenterM.X),MakeShared<FJsonValueNumber>(S.CenterM.Y),MakeShared<FJsonValueNumber>(S.CenterM.Z)});A.Add(MakeShared<FJsonValueObject>(O));}F->SetArrayField(TEXT("slots"),A);
    A.Reset();for(const auto& C:Traffic.Claims){auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("owner"),C.Owner);O->SetBoolField(TEXT("quarantined"),C.Quarantined);O->SetNumberField(TEXT("heartbeat_age_s"),Age-C.Heartbeat);O->SetNumberField(TEXT("corridors"),C.Corridors.Num());TArray<TSharedPtr<FJsonValue>> Boxes;for(const auto& B:C.Corridors)Boxes.Add(MakeShared<FJsonValueArray>(TArray<TSharedPtr<FJsonValue>>{MakeShared<FJsonValueNumber>(B.Min.X),MakeShared<FJsonValueNumber>(B.Min.Y),MakeShared<FJsonValueNumber>(B.Max.X),MakeShared<FJsonValueNumber>(B.Max.Y)}));O->SetArrayField(TEXT("bounds_m"),Boxes);A.Add(MakeShared<FJsonValueObject>(O));}F->SetArrayField(TEXT("traffic_claims"),A);
    int Waiting=0;for(const auto& Pair:Jobs)Waiting+=Pair.Value.State==TEXT("queued")||Pair.Value.TrafficWaiting;F->SetNumberField(TEXT("waiting_jobs"),Waiting);A.Reset();for(const auto& P:Traffic.WaitingFor){auto O=MakeShared<FJsonObject>();O->SetStringField(TEXT("vehicle_id"),P.Key);O->SetStringField(TEXT("blocked_by"),P.Value);A.Add(MakeShared<FJsonValueObject>(O));}F->SetArrayField(TEXT("traffic_waits"),A);A.Reset();for(const auto& Id:Traffic.Deadlock())A.Add(MakeShared<FJsonValueString>(Id));F->SetArrayField(TEXT("deadlock_vehicles"),A);F->SetNumberField(TEXT("physical_loads"),PhysicalLoads.Num());F->SetStringField(TEXT("inventory_source"),TEXT("simulation bookkeeping; never sensor truth feedback"));return F;
}
void ACampusOperations::PublishFleet()
{
    if(!FleetWeb||!FleetWeb->IsDue()||!Active()||Age<NextFleetWeb)return;NextFleetWeb=Age+.2;
    auto Frame=Active()->BuildWebFrame();Frame->SetStringField(TEXT("scenario"),TEXT("campus_fleet"));Frame->SetObjectField(TEXT("fleet"),FleetTelemetry());FleetWeb->Send(Frame);
}
