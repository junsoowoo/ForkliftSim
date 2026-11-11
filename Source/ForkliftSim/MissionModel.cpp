#include "MissionNavigation.h"
void Mission::FModel::Phase(const TCHAR* Next,double Now)
{State=Next;StageS=Now;StableS=0;BestDistance=1e9;ProgressS=Now;NextPlanS=Now;Route=FPlan();Waypoint=0;Reason.Empty();}
void Mission::FModel::Pause(const TCHAR* Why,bool Fault)
{
    if(!Holding()){Checkpoint=State;PausedS=Last.Now;}
    State=Fault?TEXT("fault_latched"):TEXT("paused");Reason=Why;Engaged=true;Velocity=FVector2D::ZeroVector;YawRate=0;Brake=HoldLift=true;HeightTarget=Last.Height;CancelHandling=Last.HandlingActive;
}
bool Mission::FModel::Resume(const FInput& I)
{
    if(!I.Healthy||!I.Localized||I.VehicleFault||I.Manual||I.Velocity.Size2D()>.025){Reason=TEXT("resume_requires_healthy_localized_stationary_neutral");return false;}
    if(Carrying&&I.Load[0]+I.Load[1]<ReferenceLoad*.55){Reason=TEXT("resume_carried_load_missing");return false;}
    if(Checkpoint==TEXT("to_pickup")||Checkpoint==TEXT("to_dropoff"))
    {
        if(I.HandlingActive||I.HandlingFault){Reason=TEXT("acknowledge_local_handling_before_navigation");return false;}
        const FString Next=Checkpoint;StartS+=I.Now-PausedS;Phase(*Next,I.Now);HeightTarget=Carrying?.25:0;HoldLift=false;return true;
    }
    if(Checkpoint==TEXT("pickup")&&I.Camera&&(I.HandlingFailedStage==TEXT("pick_height")||I.HandlingFailedStage==TEXT("acquire"))&&I.Load[0]<80&&I.Load[1]<80)
    {StartS+=I.Now-PausedS;Phase(TEXT("request_pickup"),I.Now);HoldLift=false;return true;}
    Reason=TEXT("partial_insertion_or_transfer_requires_manual_recovery");return false;
}
void Mission::FModel::Step(const FInput& I)
{
    Last=I;Velocity=FVector2D::ZeroVector;YawRate=0;Brake=true;HandlingRequest=Handling::ERequest::None;CancelHandling=false;HoldLift=false;
    if(I.Request==ERequest::Cancel)
    {if(Engaged){Pause(TEXT("operator_cancel"));State=TEXT("cancelled");}return;}
    if(I.Request==ERequest::Pause&&Engaged){Pause(TEXT("operator_pause"));return;}
    if(I.Request==ERequest::Resume&&Holding()){Resume(I);return;}
    if(I.Request==ERequest::StartLow||I.Request==ERequest::StartHigh)
    {
        if(Engaged){Reason=TEXT("mission_already_engaged");return;}
        if(!I.Healthy||!I.Localized||I.VehicleFault||I.Manual||I.HandlingActive||I.HandlingFault||I.Velocity.Size2D()>.025||I.Load[0]>80||I.Load[1]>80){Reason=TEXT("start_requires_empty_healthy_localized_stationary_neutral");return;}
        Engaged=true;High=I.Request==ERequest::StartHigh;Carrying=SawPickup=SawDropoff=false;Retries=Replans=0;StartS=I.Now;HeightTarget=I.Height;HavePose=false;Phase(FVector2D(I.Pose.X-Spec.Pickup.X,I.Pose.Y-Spec.Pickup.Y).Size()<.065?TEXT("request_pickup"):TEXT("to_pickup"),I.Now);return;
    }
    if(!Engaged)return;
    if(Holding()){HoldLift=true;HeightTarget=I.Height;return;}
    if(I.Manual){Pause(TEXT("manual_takeover"));return;}
    if(!I.Healthy||I.VehicleFault){Pause(TEXT("required_sensor_or_vehicle_fault"),true);return;}
    if(I.Now-StartS>Spec.Timeout){Pause(TEXT("mission_timeout"),true);return;}
    if(Navigating())
    {
        if(!I.Localized){Pause(TEXT("localization_stale"),true);return;}
        if(FMath::Abs(I.Pose.Yaw)>.06){Pause(TEXT("fixed_heading_envelope_exceeded"),true);return;}
        if(HavePose&&FVector2D(I.Pose.X-PreviousPose.X,I.Pose.Y-PreviousPose.Y).Size()>.35){Pause(TEXT("localization_jump"),true);return;}PreviousPose=I.Pose;HavePose=true;
        if(Carrying&&(I.Load[0]+I.Load[1]<ReferenceLoad*.55||FMath::Min(I.Load[0],I.Load[1])<20)){Pause(TEXT("carried_load_lost_or_uneven"),true);return;}
        HeightTarget=Carrying?.25:0;
        if(FMath::Abs(I.Height-HeightTarget)>.025){if(I.Now-StageS>20)Pause(TEXT("travel_height_not_reached"),true);return;}
        if(I.Now-StageS>Spec.NavTimeout){Pause(TEXT("navigation_timeout"),true);return;}
        const FVector2D Here(I.Pose.X,I.Pose.Y),Goal=State==TEXT("to_pickup")?Spec.Pickup:Spec.Dropoff;const double Distance=(Goal-Here).Size();
        if(Distance<.065&&I.Velocity.Size2D()<.025&&FMath::Abs(I.Velocity.Z)<.025)
        {StableS+=I.Dt;if(StableS>.4)Phase(Carrying?TEXT("request_dropoff"):TEXT("request_pickup"),I.Now);return;}StableS=0;
        if(I.Now>=NextPlanS||!Route.Valid)
        {
            NextPlanS=I.Now+Spec.ReplanS;Planner.Spec=Spec;
            bool Keep=Route.Valid&&Waypoint<Route.Points.Num()&&Planner.Clear(Here,Route.Points[Waypoint]);
            if(Keep)for(int K=Waypoint;K+1<Route.Points.Num();++K)if(!Planner.Clear(Route.Points[K],Route.Points[K+1])){Keep=false;break;}
            if(!Keep)
            {
                const auto New=Planner.Plan(Here,Goal);++Replans;
                if(!New.Valid){Pause(*New.Reason);State=TEXT("blocked");return;}
                Route=New;Waypoint=1;BestDistance=1e9;ProgressS=I.Now;
            }
        }
        while(Waypoint+1<Route.Points.Num()&&(Route.Points[Waypoint]-Here).Size()<.08&&Planner.Clear(Here,Route.Points[Waypoint+1])){++Waypoint;BestDistance=1e9;ProgressS=I.Now;}
        if(Waypoint>=Route.Points.Num()){Route.Valid=false;return;}
        const auto Delta=Route.Points[Waypoint]-Here;const double Cap=Carrying?Spec.LoadedSpeed:Spec.EmptySpeed;
        const double V=FMath::Min(Cap,.8*Delta.Size());const auto World=Delta.GetSafeNormal()*V;const double C=FMath::Cos(I.Pose.Yaw),S=FMath::Sin(I.Pose.Yaw);
        Velocity=FVector2D(C*World.X+S*World.Y,-S*World.X+C*World.Y);YawRate=FMath::Clamp(-1.2*I.Pose.Yaw,-.08,.08);Brake=Distance<.045;if(Brake)Velocity=FVector2D::ZeroVector;
        if(Delta.Size()<BestDistance-.04){BestDistance=Delta.Size();ProgressS=I.Now;}else if(I.Now-ProgressS>12){Pause(TEXT("navigation_progress_timeout"),true);}
        return;
    }
    if(State==TEXT("request_pickup")||State==TEXT("request_dropoff"))
    {
        const bool Place=State==TEXT("request_dropoff");HandlingBaseline=I.HandlingSequence;
        HandlingRequest=Place?(High?Handling::ERequest::PlaceHigh:Handling::ERequest::PlaceLow):Handling::ERequest::PickGround;
        Phase(Place?TEXT("dropoff"):TEXT("pickup"),I.Now);AwaitingHandling=true;return;
    }
    if(State==TEXT("pickup")||State==TEXT("dropoff"))
    {
        if(I.HandlingSequence<=HandlingBaseline)
        {if(I.Now-StageS>2)Pause(TEXT("handling_request_rejected"),true);return;}
        AwaitingHandling=false;
        if(I.HandlingFault)
        {
            const bool SafeRetry=State==TEXT("pickup")&&!I.HandlingFull&&(I.HandlingFailedStage==TEXT("acquire")||I.HandlingFailedStage==TEXT("pick_height"))&&I.Load[0]<80&&I.Load[1]<80;
            if(SafeRetry&&Retries<Spec.MaxRetries&&I.Camera){++Retries;Phase(TEXT("retry_pickup"),I.Now);return;}
            Pause(*(TEXT("handling_")+I.HandlingReason),true);return;
        }
        if(I.HandlingState==TEXT("complete")&&!I.HandlingActive)
        {
            if(!I.HandlingTransfer){Pause(TEXT("handling_missing_transfer_receipt"),true);return;}
            if(State==TEXT("pickup"))
            {
                if(!I.HandlingFull||I.Load[0]<80||I.Load[1]<80){Pause(TEXT("pickup_missing_full_or_load"),true);return;}
                SawPickup=Carrying=true;ReferenceLoad=I.Load[0]+I.Load[1];HavePose=false;Phase(TEXT("to_dropoff"),I.Now);
            }
            else
            {
                if(I.Load[0]+I.Load[1]>FMath::Max(100.,ReferenceLoad*.1)){Pause(TEXT("dropoff_load_not_released"),true);return;}
                SawDropoff=true;Carrying=Engaged=false;HeightTarget=.25;State=TEXT("complete");StageS=I.Now;
            }
        }return;
    }
    if(State==TEXT("retry_pickup")&&I.Now-StageS>1)Phase(TEXT("request_pickup"),I.Now);
}
bool Mission::FModel::GuardPoint(FVector2D P) const
{
    // Current full footprint plus a bounded command-direction stopping sweep.
    const double V=Velocity.Size(),Extra=.12+.35*V+V*V/(2*.5);const FVector2D D=Velocity.GetSafeNormal()*Extra;
    const double MinX=-Spec.Rear-Spec.Margin+FMath::Min(0.,D.X),MaxX=Spec.Front+Spec.Margin+FMath::Max(0.,D.X);
    const double MinY=-Spec.HalfWidth-Spec.Margin+FMath::Min(0.,D.Y),MaxY=Spec.HalfWidth+Spec.Margin+FMath::Max(0.,D.Y);
    return P.X>=MinX&&P.X<=MaxX&&P.Y>=MinY&&P.Y<=MaxY;
}
