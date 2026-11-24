#include "CampusOperations.h"
#include "StabilityLab.h"
#include "SensorDrive.h"
TArray<FVector2D> ACampusOperations::TrafficRoute(const AStabilityLab& V,const CampusFleet::FJob& J) const
{
 if(!Traffic.Sections){TArray<FVector2D> R=J.ToPick.Points;R.Append(J.ToDrop.Points);R.Append(J.ToReturn.Points);return R;}
 const auto Pose=V.DriveSensors->Model.MapPose(V.TimeS);const FVector2D Here(Pose.X,Pose.Y);
 const bool Pick=J.State==TEXT("queued")||J.State==TEXT("to_pickup")||J.State==TEXT("to_charger");
 const auto& P=Pick?J.ToPick:J.State==TEXT("returning")?J.ToReturn:J.ToDrop;
 if(J.State==TEXT("queued")||J.Travelling())
 {
  TArray<FVector2D> R={Here};const int K=J.State==TEXT("queued")?1:J.Waypoint;
  // Acquire current graph leg plus its exit leg atomically. Never move into an unowned leg.
  for(int I=K;I<FMath::Min(P.Points.Num(),K+2);++I)R.Add(P.Points[I]);if(R.Num()<2)R.Add(Here);return R;
 }
 if(J.Kind==TEXT("charge"))return {Here,Here};
 const bool AtPick=J.State==TEXT("align_pickup")||J.State==TEXT("pickup");
 const FVector2D Approach=AtPick?J.Pickup:J.Dropoff,Center=FVector2D(AtPick?J.PickCenter:J.DropCenter);
 // Docking/rotation envelope stays reserved throughout local handling.
 return {Here,Approach,Center};
}
bool ACampusOperations::ClaimRoute(const AStabilityLab& V,const CampusFleet::FJob& J){return Traffic.Acquire(V.VehicleId,TrafficRoute(V,J),Age);}
void ACampusOperations::ObserveTraffic()
{
 int Moving=0;for(AStabilityLab* V:Vehicles)
 {
  const auto& D=V->DriveSensors->Model;const bool Fresh=D.Ready&&D.Localized&&V->TimeS-D.LastMatchS<1.5;
  const auto P=D.MapPose(V->TimeS);if(Traffic.Sections)Traffic.Observe(V->VehicleId,FVector2D(P.X,P.Y),Fresh,Age);
  if(Fresh&&V->DriveSensors->GroundFresh(V->TimeS)&&FVector2D(V->DriveSensors->MeasuredVelocity(V->TimeS)).Size()>.05)++Moving;
 }
 PeakMoving=FMath::Max(PeakMoving,Moving);
 const auto Cycle=Traffic.Deadlock();for(const auto& Id:Cycle)if(auto* J=Jobs.Find(Id))if(J->Engaged&&!J->Held())J->Hold(TEXT("traffic_deadlock_manual_recovery"),true);
}

Mission::FPlan ACampusOperations::AssignmentPlan(FVector2D Start,FVector2D Goal) const
{
 if(!BoardEnabled())return Graph.Plan(Start,Goal);
 const FString Key=FString::Printf(TEXT("%.17g,%.17g:%.17g,%.17g"),Start.X,Start.Y,Goal.X,Goal.Y);
 if(const auto* Cached=AssignmentPlans.Find(Key))return *Cached;
 if(PlanningBudget<=0){Mission::FPlan P;P.Reason=TEXT("assignment_planning_budget_wait");return P;}
 --PlanningBudget;auto Plan=Graph.Plan(Start,Goal);if(AssignmentPlans.Num()>=4096)AssignmentPlans.Reset();AssignmentPlans.Add(Key,Plan);return Plan;
}
