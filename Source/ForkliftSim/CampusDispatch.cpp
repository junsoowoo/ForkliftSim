#include "CampusOperations.h"
#include "WarehouseCampus.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "HandlingControl.h"
#include "JointSensors.h"
#include "DynamicsSensors.h"
#include "SensorLiftControl.h"
#include "LidarSafety.h"
#include "PalletPerception.h"
#include "ProximitySensors.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "WebTelemetry.h"
#include "HAL/PlatformTime.h"
namespace
{
    bool Ready(const AStabilityLab& V,bool Fix=true)
    {
        if(!V.DriveSensors||!V.DriveSensors->Model.Ready||!V.SensorLift||!V.SensorLift->Model.Armed||V.SensorLift->Model.Faulted||!V.VehicleGuard||V.VehicleGuard->Decision.Faulted||!V.JointSensors||!V.DynamicsSensors)return false;
        const auto& D=V.DriveSensors->Model;if(D.Faulted||D.UnobservedS>.1||(Fix&&(!D.Localized||V.TimeS-D.LastMatchS>1.5)))return false;
        for(int I:{2,3})if(!V.DynamicsSensors->Model.Usable(I,V.TimeS)||V.DynamicsSensors->Model.Channels[I].Last.Clipped)return false;
        return V.JointSensors->Model.Usable(4,V.TimeS);
    }
    FVector2D Position(const AStabilityLab& V){const auto P=V.DriveSensors->Model.MapPose(V.TimeS);return FVector2D(P.X,P.Y);}
    double Load(const AStabilityLab& V){return V.DynamicsSensors?V.DynamicsSensors->Model.Channels[2].Last.Value[0]+V.DynamicsSensors->Model.Channels[3].Last.Value[0]:0;}
    CampusFleet::FGraph ObservedGraph(const ACampusOperations& O,const AStabilityLab& V)
    {
        CampusFleet::FGraph G=O.Graph;if(!V.VehicleGuard||!V.DriveSensors)return G;const auto& S=V.VehicleGuard->Sensor;const auto& D=V.DriveSensors->Model;
        for(int I=0;I<2;++I)if(S.Fresh(I,V.TimeS))
        {
            const auto& Scan=S.Channels[I].Last;SensorDrive::FPose Local;if(!D.At(Scan.CaptureS,Local))continue;const auto Pose=SensorDrive::Compose(D.MapToOdom,Local);const double C=FMath::Cos(Pose.Yaw),Sn=FMath::Sin(Pose.Yaw);
            for(int K=0;K<Scan.States.Num();++K)if(Scan.States[K]==LidarSafety::EReturn::Valid)
            {const double A=S.Spec.Angle(I,K);const auto B=S.Spec.Mount(I)+FVector(FMath::Cos(A),FMath::Sin(A),0)*Scan.Ranges[K];if(O.FleetPermit(V,FVector2D(B)))continue;G.Clearance.Obstacles.Add(FVector2D(Pose.X+C*B.X-Sn*B.Y,Pose.Y+Sn*B.X+C*B.Y));}
        }return G;
    }
    void RefreshCarrier(AStabilityLab& V,CampusFleet::FJob& J)
    {
        if(!J.Carrying||!J.PickupReceipt||!V.ProximitySensors||!V.JointSensors||!V.DynamicsSensors||!V.JointSensors->Model.Usable(4,V.TimeS)||!V.DynamicsSensors->Model.Usable(2,V.TimeS)||!V.DynamicsSensors->Model.Usable(3,V.TimeS)||Load(V)<J.ReferenceLoad*.55)return;
        const double Height=V.JointSensors->Model.Channels[4].Last.Value;if(Height<.12||Height>.36)return;
        const auto& R=V.ProximitySensors->Model;double Depth[2]={},Stamp=V.TimeS;
        for(int I=0;I<2;++I){if(!R.MeasurementValid(I,V.TimeS))return;Stamp=FMath::Min(Stamp,R.Channels[I].Last.Truth.TimeS);Depth[I]=1.2-R.Spec.ForwardOffsetM-R.Channels[I].Last.RangeM;}
        if(V.TimeS-Stamp>.15||FMath::Min(Depth[0],Depth[1])<.99||FMath::Abs(Depth[0]-Depth[1])>.08)return;
        const double Front=V.Settings.ForkFaceXM()+1.2-(Depth[0]+Depth[1])*.5;
        if(J.CarrierUntil>=0&&FMath::Abs(Front-.12-J.CarrierFront)>.08)return;
        J.CarrierFront=Front-.12;J.CarrierBack=Front+1.22;J.CarrierUntil=Stamp+.25;
    }
    void Phase(CampusFleet::FJob& J,const TCHAR* State,double Now){J.State=State;J.Reason.Empty();J.StageS=Now;J.StableS=0;J.Waypoint=1;J.BestDistance=1e9;J.LastProgressS=Now;J.HandlingStarted=false;UE_LOG(LogTemp,Display,TEXT("CAMPUS_JOB %s %s state=%s"),*J.Id,*J.Vehicle,State);}
    Mission::FPlan& Path(CampusFleet::FJob& J){return (J.State==TEXT("to_pickup")||J.State==TEXT("to_charger"))?J.ToPick:J.State==TEXT("returning")?J.ToReturn:J.ToDrop;}
    TArray<FVector2D> Whole(const CampusFleet::FJob& J){TArray<FVector2D> P=J.ToPick.Points;P.Append(J.ToDrop.Points);P.Append(J.ToReturn.Points);return P;}
}
void ACampusOperations::SetupRouteReferences()
{
    // Physical 0.5 m surveyed targets, instanced without changing collision or
    // sensor return semantics. The outermost column stays clear of lane x=96.
    auto* Group=GetWorld()->SpawnActor<AWarehouseMapGroup>();Group->SetOwner(this);Group->LayoutMesh=LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube"));Group->Blocking=true;NavigationReferences.Add(Group);
    for(double X:{-20.,-12.,-4.,10.,22.,44.,61.,78.,98.8})for(double Y:{3.,21.,29.,47.})
    {
        if(X==10&&Y==47)continue;
        Group->LayoutTransforms.Add(FTransform(FQuat::Identity,FVector(X*100,Y*100,35),FVector(.5,.5,.7)));Group->LayoutRecordIds.Add(TEXT(""));
        const FVector2D Min(X-.25,Y-.25),Max(X+.25,Y+.25);Survey.Append({{Min,FVector2D(Max.X,Min.Y)},{FVector2D(Max.X,Min.Y),Max},{Max,FVector2D(Min.X,Max.Y)},{FVector2D(Min.X,Max.Y),Min}});
    }
    Group->Rebuild();Graph.Clearance.Map=Survey;
}
bool ACampusOperations::VehicleEngaged(const AStabilityLab& V) const{const auto* J=Jobs.Find(V.VehicleId);return J&&J->Engaged;}
bool ACampusOperations::ReadyForTask(const AStabilityLab& V,bool ForCharge) const
{return (ForCharge||BatteryReady(V))&&V.HandlingControl&&!V.bCampusManualInput&&!VehicleEngaged(V)&&Ready(V)&&V.DriveSensors->Stationary(V.TimeS)&&Load(V)<100&&!V.HandlingControl->Model.Faulted&&!V.HandlingControl->Model.Active&&!V.SensorLift->Model.PressureLimited&&V.JointSensors->Model.Channels[4].Last.Value<=.36;}
bool ACampusOperations::BuildJob(AStabilityLab& V,const FString& LoadId,const FString& Destination,bool High,CampusFleet::FJob& J,FString& Why) const
{
 const auto* LoadRecord=Inventory.Loads.Find(LoadId);if(!LoadRecord||LoadRecord->Moving||(!LoadRecord->ReservedBy.IsEmpty()&&LoadRecord->ReservedBy!=V.VehicleId)){Why=TEXT("load_busy_or_unknown");return false;}const auto& L=*LoadRecord;
 J=CampusFleet::FJob();J.Vehicle=V.VehicleId;J.Load=L.Id;J.Started=Age;J.Engaged=true;
 if(!L.Slot.IsEmpty()){J.PickCenter=Inventory.Slots[L.Slot].CenterM;J.PickTier=J.PickCenter.Z;J.Pickup=FVector2D(SlotApproaches[L.Slot]);J.PickYaw=SlotYaws[L.Slot];}
 else if(const auto* P=KnownStations.Find(L.Id)){J.PickCenter=*P;J.Pickup=FVector2D(P->X-3.1,P->Y);}
 else{Why=TEXT("no_surveyed_pickup_station");return false;}
 J.ToPick=AssignmentPlan(Position(V),J.Pickup);if(!J.ToPick.Valid){Why=TEXT("pickup_route:")+J.ToPick.Reason;return false;}
 TArray<FString> Candidates;Inventory.Slots.GetKeys(Candidates);Candidates.Sort([&](const FString& A,const FString& B){const double DA=(FVector2D(SlotApproaches[A])-J.Pickup).SizeSquared(),DB=(FVector2D(SlotApproaches[B])-J.Pickup).SizeSquared();return FMath::IsNearlyEqual(DA,DB)?A<B:DA<DB;});
 if(!Destination.IsEmpty()){if(!Inventory.Slots.Contains(Destination)){Why=TEXT("unknown_destination");return false;}Candidates={Destination};}
 for(const auto& Id:Candidates){const auto& S=Inventory.Slots[Id];if(Id==L.Slot||S.Blocked||!S.Pallet.IsEmpty()||!S.ReservedBy.IsEmpty()||(Destination.IsEmpty()&&FMath::Abs(S.CenterM.Z-(High?1.95:.35))>.01))continue;
  auto To=AssignmentPlan(J.Pickup,FVector2D(SlotApproaches[Id]));if(!To.Valid)continue;const int Index=Vehicles.IndexOfByKey(&V);auto Return=AssignmentPlan(FVector2D(SlotApproaches[Id]),FVector2D(ParkingStartM[Index]));if(!Return.Valid)continue;
  J.Destination=Id;J.DropCenter=S.CenterM;J.Dropoff=FVector2D(SlotApproaches[Id]);J.DropYaw=SlotYaws[Id];J.DropTier=S.CenterM.Z;J.ToDrop=MoveTemp(To);J.ToReturn=MoveTemp(Return);break;}
 if(J.Destination.IsEmpty()){Why=BoardEnabled()&&PlanningBudget<=0?TEXT("assignment_planning_budget_wait"):TEXT("destination_occupied_reserved_or_unreachable");return false;}return true;
}
bool ACampusOperations::CommitJob(AStabilityLab& V,CampusFleet::FJob J,FString& Why)
{
 if(!ReadyForTask(V)||!Inventory.Reserve(J.Load,V.VehicleId)){Why=TEXT("vehicle_or_load_no_longer_available");return false;}
 const int Previous=Selected;Selected=Vehicles.IndexOfByKey(&V);const bool Bound=SelectLoad(J.Load);Selected=Previous;
 if(!Bound){Inventory.Release(J.Load,V.VehicleId);Why=Notice;return false;}
 if(!Inventory.ReserveSlot(J.Destination,V.VehicleId)){Inventory.Release(J.Load,V.VehicleId);Why=TEXT("destination_no_longer_available");return false;}
 J.Id=FString::Printf(TEXT("JOB-%04d"),++JobNumber);Jobs.Add(V.VehicleId,J);Notice=J.Id+TEXT(" queued: ")+J.Load+TEXT(" -> ")+J.Destination;return true;
}
void ACampusOperations::QueueSelected()
{
 auto* V=Active();if(!V)return;
 if(BoardEnabled()){WarehouseTasks::FRequest R;R.Load=V->CampusLoadId;R.Tier=HighTier?2:1;R.PreferredVehicle=V->VehicleId;TArray<FString> Ids;AddTasks({R},Ids,Notice);return;}
 if(!ReadyForTask(*V)){Notice=TEXT("Queue requires stationary healthy empty vehicle and fresh localization");return;}
 CampusFleet::FJob J;FString Why;if(!BuildJob(*V,V->CampusLoadId,TEXT(""),HighTier,J,Why)||!CommitJob(*V,J,Why))Notice=Why;
}
void ACampusOperations::QueueAll()
{
 if(BoardEnabled()){TArray<WarehouseTasks::FRequest> R;for(AStabilityLab* V:Vehicles){WarehouseTasks::FRequest Q;Q.Load=V->CampusLoadId;Q.Tier=HighTier?2:1;R.Add(Q);}TArray<FString> Ids;AddTasks(R,Ids,Notice);return;}
 const int Old=Selected;for(int I=0;I<Vehicles.Num();++I){Selected=I;QueueSelected();}Selected=Old;
}
void ACampusOperations::PauseSelected(){if(auto* V=Active())if(auto* J=Job(V->VehicleId)){J->Hold(TEXT("operator_pause"));if(V->HandlingControl->Model.Active)V->HandlingControl->Model.Stop(TEXT("fleet_pause"));}}
void ACampusOperations::CancelSelected()
{
    PauseSelected();auto* V=Active();if(!V)return;auto* J=Job(V->VehicleId);if(!J)return;
    if(J->Kind==TEXT("charge")){J->State=TEXT("cancelled");J->Reason=TEXT("charge_cancel_manual_recovery");if(Ready(*V)&&V->DriveSensors->Stationary(V->TimeS)&&Load(*V)<100){for(auto& C:Chargers)if(C.Owner==V->VehicleId)C.Owner.Empty();Traffic.Release(V->VehicleId,true);J->Engaged=false;}return;}
    J->State=TEXT("cancelled");J->Reason=TEXT("loaded_or_unhealthy_cancel_retains_reservations; manual recovery required");
    if(Ready(*V)&&V->DriveSensors->Stationary(V->TimeS)&&Load(*V)<100)
    {auto& Slot=Inventory.Slots[J->Destination];if(Slot.ReservedBy==V->VehicleId)Slot.ReservedBy.Empty();Traffic.Release(V->VehicleId,true);J->Engaged=false;J->Reason=TEXT("cancelled_empty_stationary; peer footprint remains protected");}
}
void ACampusOperations::ResumeSelected()
{
    auto* V=Active();if(!V)return;auto* J=Job(V->VehicleId);if(!J||!J->Held())return;
    if(!Ready(*V)||!V->DriveSensors->Stationary(V->TimeS)||V->HandlingControl->Model.Faulted){Notice=TEXT("Resume requires Y acknowledge, healthy stationary sensors and fresh map fix");return;}
    // Faulted local insertion cannot restart blindly halfway inside a pallet.
    if(J->ResumeState==TEXT("pickup")||J->ResumeState==TEXT("dropoff")){Notice=TEXT("Interrupted insertion requires manual recovery and a new job; no blind resume");return;}
    const FString State=J->ResumeState;
    if(State==TEXT("to_pickup")||State==TEXT("to_dropoff")||State==TEXT("returning")||State==TEXT("to_charger"))
    {
        auto Copy=*J;Copy.State=State;Copy.Waypoint=1;auto& R=Path(Copy);const FVector2D Goal=(State==TEXT("to_pickup")||State==TEXT("to_charger"))?J->Pickup:State==TEXT("to_dropoff")?J->Dropoff:FVector2D(ParkingStartM[Vehicles.IndexOfByKey(V)]);
        R=ObservedGraph(*this,*V).Plan(Position(*V),Goal);if(!R.Valid){Notice=TEXT("Resume replan blocked by current measured obstacles: ")+R.Reason;return;}
        for(auto& C:Traffic.Claims)if(C.Owner==V->VehicleId){C.Quarantined=false;C.Heartbeat=Age;}
        if(!ClaimRoute(*V,Copy)){for(auto& C:Traffic.Claims)if(C.Owner==V->VehicleId)C.Quarantined=true;Notice=TEXT("Resume waits for conflicting route: ")+Traffic.Blocker;return;}
        J->ToPick=Copy.ToPick;J->ToDrop=Copy.ToDrop;J->ToReturn=Copy.ToReturn;
    }
    for(auto& C:Traffic.Claims)if(C.Owner==V->VehicleId){C.Quarantined=false;C.Heartbeat=Age;}
    Phase(*J,*State,Age);J->LeaseUntil=V->TimeS+.15;Notice=TEXT("Explicit resume accepted");
}
void ACampusOperations::UpdateVehicle(AStabilityLab& V,double Dt)
{
    auto* J=Job(V.VehicleId);if(!J||!J->Engaged)return;
    if(FleetTest&&TestCase==TEXT("watchdog")&&V.VehicleId==TEXT("AGV-01")&&TestStage==1&&Age-StageStarted<1)return;
    J->ManualRecovery=false;J->Velocity=FVector2D::ZeroVector;J->YawRate=0;J->Brake=true;J->LeaseUntil=V.TimeS+.15;
    // Inputs have already been read by the player controller; never erase takeover.
    const bool Manual=V.bCampusManualInput;
    const FVector ManualDrive(V.Throttle,V.Strafe,V.Steering);const double ManualLift=V.Lift,ManualBrake=V.Brake;
    V.Throttle=V.Strafe=V.Steering=V.Lift=0;V.Brake=0;
    if(Manual&&!J->Held()){J->Hold(TEXT("manual_takeover"));if(V.HandlingControl->Model.Active)V.HandlingControl->Model.Stop(TEXT("manual_takeover"));}
    if(J->Held()){RefreshCarrier(V,*J);J->ManualRecovery=Manual;if(Manual){V.Throttle=ManualDrive.X;V.Strafe=ManualDrive.Y;V.Steering=ManualDrive.Z;V.Lift=ManualLift;V.Brake=ManualBrake;}else V.Brake=0;return;}
    if(const auto* B=Batteries.Find(V.VehicleId))if(BatterySpec.Enabled&&B->CriticalLatched){J->Hold(TEXT("critical_soc_manual_recovery_required"),true);return;}
    if(!Ready(V)){J->Hold(TEXT("sensor_or_localization_unavailable"),true);if(V.HandlingControl->Model.Active)V.HandlingControl->Model.Stop(TEXT("fleet_sensor_stop"));return;}
    Traffic.Heartbeat(V.VehicleId,Age);RefreshCarrier(V,*J);
    if(V.VehicleGuard->Decision.DriveHold&&!V.VehicleGuard->Decision.Obstacle&&!V.VehicleGuard->EstopRequested&&V.DriveSensors->Stationary(V.TimeS))V.VehicleGuard->AckRequested=true;
    if(J->State.StartsWith(TEXT("align_"))&&Age-J->StageS>60){J->Hold(TEXT("alignment_no_progress"));return;}
    if(Age-J->Started>1800){J->Hold(TEXT("job_timeout"),true);return;}
    auto& H=*V.HandlingControl;const auto Pose=V.DriveSensors->Model.MapPose(V.TimeS);const FVector2D Here(Pose.X,Pose.Y);
    if(J->State==TEXT("queued"))
    {
        // Also exclude measured parked/manual peers, even when they own no job.
        bool PeerClear=true;for(AStabilityLab* Peer:Vehicles)if(Peer!=&V){if(!Ready(*Peer)){PeerClear=false;break;}const auto Points=TrafficRoute(V,*J);const auto PeerPosition=Position(*Peer);for(int K=1;K<Points.Num();++K){const auto D=Points[K]-Points[K-1];const double T=FMath::Clamp(FVector2D::DotProduct(PeerPosition-Points[K-1],D)/FMath::Max(1e-9,D.SizeSquared()),0.,1.);if((PeerPosition-Points[K-1]-D*T).Size()<2.5){PeerClear=false;break;}}}
        if(!PeerClear){J->Reason=TEXT("peer_footprint_or_pose_unavailable");return;}
        if(!ClaimRoute(V,*J)){J->Reason=TEXT("traffic_wait:")+Traffic.Blocker;return;}
        Phase(*J,J->Kind==TEXT("charge")?TEXT("to_charger"):TEXT("to_pickup"),Age);
    }
    if(J->Kind==TEXT("charge")&&UpdateCharge(V,*J,Dt))return;
    if(J->Travelling())
    {
        J->TrafficWaiting=false;
        if(Traffic.Sections&&!ClaimRoute(V,*J)){J->TrafficWaiting=true;J->Reason=TEXT("traffic_wait:")+Traffic.Blocker;J->LastProgressS=Age;return;}J->Reason.Empty();
        auto& Route=Path(*J);if(!Route.Valid||!Route.Points.IsValidIndex(J->Waypoint)){J->Hold(TEXT("invalid_route"),true);return;}
        if(V.TimeS>=J->NextObserve)
        {
            J->NextObserve=V.TimeS+1;auto Observed=ObservedGraph(*this,V);const auto Goal=Route.Points[J->Waypoint];
            if(!Observed.Clearance.Clear(Here,Goal))
            {
                // A bounded local grid handles sensed obstructions before the
                // protective field. A protective stop always remains latched.
                const auto LocalGoal=Here+(Goal-Here).GetClampedToMaxSize(14.);auto& P=Observed.Clearance;
                const FVector2D Low(FMath::Max(P.Spec.Bounds.Min.X,FMath::Min(Here.X,LocalGoal.X)-4),FMath::Max(P.Spec.Bounds.Min.Y,FMath::Min(Here.Y,LocalGoal.Y)-4)),High(FMath::Min(P.Spec.Bounds.Max.X,FMath::Max(Here.X,LocalGoal.X)+4),FMath::Min(P.Spec.Bounds.Max.Y,FMath::Max(Here.Y,LocalGoal.Y)+4));P.Spec.Bounds=FBox2D(Low,High);P.Spec.Cell=.25;
                auto Local=P.Plan(Here,LocalGoal);if(!Local.Valid){J->Hold(TEXT("measured_route_blocked:")+Local.Reason);return;}
                if((LocalGoal-Goal).Size()>.01)Local.Points.Add(Goal);for(int K=J->Waypoint+1;K<Route.Points.Num();++K)Local.Points.Add(Route.Points[K]);Local.Length=0;for(int K=1;K<Local.Points.Num();++K)Local.Length+=(Local.Points[K]-Local.Points[K-1]).Size();
                auto Copy=*J;Path(Copy)=Local;Copy.Waypoint=1;if(!ClaimRoute(V,Copy)){J->Hold(TEXT("detour_traffic_conflict:" )+Traffic.Blocker);return;}
                Route=Local;J->Waypoint=1;J->BestDistance=1e9;J->LastProgressS=Age;++J->Replans;
            }
        }
        const auto Goal=Route.Points[J->Waypoint];const auto D=Goal-Here;const bool Last=J->Waypoint==Route.Points.Num()-1;const double Dist=D.Size();
        if(Dist<J->BestDistance-.03){J->BestDistance=Dist;J->LastProgressS=Age;}if(Age-J->LastProgressS>30){J->Hold(TEXT("no_progress_or_bottleneck"));return;}
        const bool Near=Dist<(Last?.035:.12)&&FMath::Abs(FMath::UnwindRadians(Pose.Yaw))<.025;
        if(Near&&Last&&J->Kind!=TEXT("charge")&&J->State!=TEXT("returning")&&Traffic.Sections){auto Dock=*J;Dock.State=J->State==TEXT("to_pickup")?TEXT("align_pickup"):TEXT("align_dropoff");if(!ClaimRoute(V,Dock)){J->TrafficWaiting=true;J->Reason=TEXT("traffic_wait_docking:")+Traffic.Blocker;J->LastProgressS=Age;return;}}
        if(Near){J->StableS+=Dt;if(J->StableS>.25&&V.DriveSensors->Stationary(V.TimeS,.03)){if(!Last){++J->Waypoint;J->BestDistance=1e9;J->LastProgressS=Age;}else if(J->State==TEXT("returning")){Phase(*J,TEXT("complete"),Age);J->Engaged=false;Traffic.Release(V.VehicleId,true);Inventory.Release(J->Load,V.VehicleId);}else Phase(*J,J->State==TEXT("to_charger")?TEXT("align_charge"):J->State==TEXT("to_pickup")?TEXT("align_pickup"):TEXT("align_dropoff"),Age);}return;}
        J->StableS=0;
        const double CruiseMps=J->Carrying?CampusFleet::LoadedCruiseMps:CampusFleet::EmptyCruiseMps;
        // Slow proportionally before each waypoint, including the docking stop.
        const FVector2D WorldVelocity=(D*.65).GetClampedToMaxSize(CruiseMps);
        const double C=FMath::Cos(Pose.Yaw),S=FMath::Sin(Pose.Yaw);
        J->Velocity=FVector2D(C*WorldVelocity.X+S*WorldVelocity.Y,-S*WorldVelocity.X+C*WorldVelocity.Y);
        J->YawRate=FMath::Clamp(-Pose.Yaw,-.15,.15);J->Brake=false;
        if(J->Carrying&&Load(V)<J->ReferenceLoad*.55){J->Hold(TEXT("carried_load_lost"),true);return;}
        if(V.JointSensors->Model.Channels[4].Last.Value>.36){J->Hold(TEXT("travel_height_not_low"),true);return;}
        if(V.VehicleGuard->Decision.DriveHold&&V.DriveSensors->Stationary(V.TimeS))V.VehicleGuard->AckRequested=true;
    }
    else if(J->State==TEXT("align_pickup")||J->State==TEXT("align_dropoff"))
    {
        const bool Pick=J->State==TEXT("align_pickup");const double Target=Pick?J->PickYaw:J->DropYaw;const double Y=FMath::UnwindRadians(Target-Pose.Yaw);J->YawRate=FMath::Clamp(Y,-.15,.15);J->Brake=FMath::Abs(Y)<.02;
        if(J->Brake&&V.DriveSensors->Stationary(V.TimeS)){J->StableS+=Dt;if(J->StableS>.35){H.SurveyFrame=true;const FVector Center=Pick?J->PickCenter:J->DropCenter;H.Station={Center.X,Center.Y,Target};H.SurveyUnloadOffsetM=J->UnloadOffset;H.SurveyCarryCenterM=Pick?0:J->CarrierFront+.12+.55;const FString Slot=Pick?Inventory.Loads[J->Load].Slot:J->Destination;H.SurveyBeamSides=SlotBeamSides.FindRef(Slot);H.Model=Handling::FModel();H.Model.Spec.Load(Error);Phase(*J,Pick?TEXT("pickup"):TEXT("dropoff"),Age);H.Request=Pick?(J->PickTier>1?Handling::ERequest::PickHigh:J->PickTier>.1?Handling::ERequest::PickLow:Handling::ERequest::PickGround):(J->DropTier>1?Handling::ERequest::PlaceHigh:Handling::ERequest::PlaceLow);}}else J->StableS=0;
        if(V.DriveSensors->Stationary(V.TimeS)&&!V.VehicleGuard->EstopRequested)V.VehicleGuard->AckRequested=true;
    }
    else if(J->State==TEXT("pickup")||J->State==TEXT("dropoff"))
    {
        if(H.Model.Faulted){J->Hold(TEXT("handling:")+H.Model.Reason,true);return;}J->HandlingStarted|=H.Model.Active;
        if(J->HandlingStarted&&!H.Model.Active&&H.Model.State==TEXT("complete"))
        {
            if(J->State==TEXT("pickup")){if(!H.Model.FullSeen||!H.Model.Receipt||!H.Model.TransferSeen){J->Hold(TEXT("missing_pickup_evidence"),true);return;}J->Carrying=J->PickupReceipt=true;J->ReferenceLoad=Load(V);J->UnloadOffset=H.Model.InsertionHeight-H.Model.Tier;RefreshCarrier(V,*J);Phase(*J,TEXT("align_travel"),Age);}
            else {if(!H.Model.TransferSeen||H.Model.Receipt||Load(V)>100||Inventory.Loads[J->Load].Slot!=J->Destination){J->Reason=TEXT("waiting_rack_support_and_inventory_confirmation");if(Age-J->StageS>180)J->Hold(TEXT("placement_not_confirmed"),true);return;}J->Carrying=false;J->DropoffReceipt=true;Phase(*J,TEXT("align_return"),Age);}
        }
        else if(!J->HandlingStarted&&Age-J->StageS>2){J->Hold(TEXT("handling_start_rejected:")+H.Model.Reason,true);return;}
    }
    else if(J->State==TEXT("align_travel")||J->State==TEXT("align_return"))
    {
        const double Y=FMath::UnwindRadians(-Pose.Yaw);J->YawRate=FMath::Clamp(Y,-.15,.15);J->Brake=FMath::Abs(Y)<.02;
        if(J->Brake&&V.DriveSensors->Stationary(V.TimeS)){J->StableS+=Dt;if(J->StableS>.35){const bool Return=J->State==TEXT("align_return");auto& R=Return?J->ToReturn:J->ToDrop;R=Graph.Plan(Here,Return?FVector2D(ParkingStartM[Vehicles.IndexOfByKey(&V)]):J->Dropoff);if(!R.Valid){J->Hold(TEXT("post_handling_route_blocked"),true);return;}Phase(*J,Return?TEXT("returning"):TEXT("to_dropoff"),Age);}}else J->StableS=0;
    }
}
void ACampusOperations::ApplyVehicle(AStabilityLab& V)
{
    if(const auto* B=Batteries.Find(V.VehicleId))if(BatterySpec.Enabled&&B->CriticalLatched){V.Throttle=V.Strafe=V.Steering=V.Lift=0;V.Brake=1;return;}
    auto* J=Job(V.VehicleId);if(!J||!J->Engaged)return;
    if(V.TimeS>J->LeaseUntil)J->Hold(TEXT("fleet_command_expired"),true);
    if(!J->Held()&&(J->Travelling()||J->State.StartsWith(TEXT("align_")))&&V.VehicleGuard&&V.VehicleGuard->Decision.Obstacle)J->Hold(TEXT("protective_obstacle_stop"));
    if(J->Held()){if(J->ManualRecovery)return;V.Throttle=V.Strafe=V.Steering=V.Lift=0;V.Brake=1;if(V.JointSensors)V.LiftTargetM=V.JointSensors->Model.Channels[4].Last.Value;return;}
    if(J->State==TEXT("pickup")||J->State==TEXT("dropoff"))return;
    V.Throttle=J->Velocity.X;V.Strafe=J->Velocity.Y;V.Steering=J->YawRate/.6;V.Brake=J->Brake?1:0;V.Lift=0;
    if(J->Travelling())V.LiftTargetM=J->Carrying?.25:0;
}
bool ACampusOperations::FleetPermit(const AStabilityLab& V,FVector2D P) const
{
    const auto* J=Jobs.Find(V.VehicleId);if(!J||!J->Carrying||!J->PickupReceipt||!Ready(V)||!V.HandlingControl)return false;
    const auto& H=V.HandlingControl->Model.Last;if(V.TimeS>J->CarrierUntil||H.Height<.12||H.Height>.36||H.Load[0]<80||H.Load[1]<80||H.Load[0]+H.Load[1]<J->ReferenceLoad*.55)return false;
    return P.X>=J->CarrierFront&&P.X<=J->CarrierBack&&FMath::Abs(P.Y)<.62;
}
bool ACampusOperations::FleetField(const AStabilityLab& V,FVector2D P) const
{
    const auto* J=Jobs.Find(V.VehicleId);if(!J)return true;
    if(V.HandlingControl&&V.HandlingControl->Model.Active&&V.HandlingControl->Model.Permit&&V.HandlingControl->Model.Last.Healthy)
    {
        auto Motion=V.HandlingControl->Model.Velocity;const auto Measured=V.DriveSensors->MeasuredVelocity(V.TimeS);if(FVector2D(Measured).Size()>Motion.Size())Motion=FVector2D(Measured);const auto D=CampusFleet::DockingStopOffset(Motion);
        return P.X>=-V.Settings.BodyLengthM*.5-.12+FMath::Min(0.,D.X)&&P.X<=V.Settings.ForkFaceXM()+V.Settings.Structure.LengthM+.12+FMath::Max(0.,D.X)&&FMath::Abs(P.Y)<=FMath::Max(.55,V.Settings.BodyWidthM*.5)+.07+FMath::Abs(D.Y);
    }
    if(J->State.StartsWith(TEXT("align_"))){const double Front=V.Settings.ForkFaceXM()+V.Settings.Structure.LengthM,Side=V.PayloadSettings.ForkSpacingM*.5+V.Settings.Structure.WidthM*.5;double Radius=FMath::Sqrt(Front*Front+Side*Side)+.15;if(J->Carrying){const double Edge=J->CarrierBack-.12;Radius=FMath::Max(Radius,FMath::Sqrt(Edge*Edge+.55*.55)+.15);}return P.SizeSquared()<=Radius*Radius;}if(!J->Travelling())return true;
    auto Motion=J->Velocity;const auto Measured=V.DriveSensors->MeasuredVelocity(V.TimeS);if(FVector2D(Measured).Size()>Motion.Size())Motion=FVector2D(Measured);const auto D=Motion.GetSafeNormal()*(.18+Motion.Size()*.35+Motion.SizeSquared());
    return P.X>=-.95+FMath::Min(0.,D.X)&&P.X<=2.2+FMath::Max(0.,D.X)&&FMath::Abs(P.Y)<=.8+FMath::Abs(D.Y);
}
