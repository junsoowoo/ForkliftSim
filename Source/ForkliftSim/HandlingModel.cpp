#include "HandlingControl.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonSerializer.h"
bool Handling::FSpec::Load(FString& Error)
{
    FString Text;TSharedPtr<FJsonObject> J;if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("HandlingControl.json")))||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J){Error=TEXT("Cannot read HandlingControl.json");return false;}
    struct F{const TCHAR* K;double* V;};F Fields[]={{TEXT("max_drive_mps"),&Speed},{TEXT("max_strafe_mps"),&Strafe},{TEXT("max_yaw_rad_s"),&Yaw},{TEXT("insert_depth_m"),&Depth},{TEXT("withdraw_m"),&Withdraw},{TEXT("carry_height_m"),&Carry},{TEXT("lift_clearance_m"),&Clear},{TEXT("stage_timeout_s"),&StageTimeout},{TEXT("job_timeout_s"),&JobTimeout},{TEXT("settle_dwell_s"),&Dwell},{TEXT("sensor_stale_s"),&Stale},{TEXT("max_station_approach_m"),&MaxApproach}};
    for(auto& V:Fields)if(!J->TryGetNumberField(V.K,*V.V)||!FMath::IsFinite(*V.V)||*V.V<=0){Error=TEXT("Invalid handling coefficient");return false;}
    if(Speed>.15||Strafe>.08||Yaw>.1||Depth<1.06||Depth>1.1||Withdraw<1.3||Withdraw>1.6||Carry>.4||Clear<.1||Clear>.2||Dwell<.3||Stale>.5||StageTimeout>60||JobTimeout>180||MaxApproach>3){Error=TEXT("Handling bounds unsupported");return false;}return true;
}
void Handling::FModel::Transition(const TCHAR* Next,double Now){State=Next;StageS=Now;StableS=0;VisionLostS=-1;}
bool Handling::FModel::Stable(bool V,double Dt){StableS=V?StableS+Dt:0;return StableS>=Spec.Dwell;}
void Handling::FModel::Stop(const TCHAR* Why){HeightTarget=Last.Height;StageS=Last.Now;FailedStage=State;Active=false;Faulted=true;Reason=Why;State=TEXT("fault_latched");Velocity=FVector2D::ZeroVector;YawRate=0;Brake=true;Permit=false;Receipt=false;}
bool Handling::FModel::Start(const FInput& I,ERequest R)
{
    Reason.Empty();
    if(!I.Healthy||I.Fault||I.Manual||I.Velocity.Size2D()>.025||FMath::Abs(I.Velocity.Z)>.025){Reason=TEXT("start_requires_healthy_stationary_neutral");return false;}
    Place=R==ERequest::PlaceLow||R==ERequest::PlaceHigh;Tier=R==ERequest::PickGround?0:(R==ERequest::PickHigh||R==ERequest::PlaceHigh?Spec.High:Spec.Low);
    if(Place)
    {
        bool Presence=true;for(bool V:I.Presence)Presence&=V;
        if(!I.HeelValid||!Presence||FMath::Min(I.HeelDepth[0],I.HeelDepth[1])-.03<1.02||FMath::Min(I.Load[0],I.Load[1])<80||I.Height<.12){Reason=TEXT("place_requires_measured_carried_full_depth");return false;}
        const double DX=Spec.RackX-Spec.ForkCenter-I.Pose.X;
        if(DX<-.1||DX>Spec.MaxApproach||FMath::Abs(I.Pose.Y-Spec.RackY)>.25||FMath::Abs(I.Pose.Yaw)>.12){Reason=TEXT("outside_surveyed_station_approach");return false;}
    }
    else if(I.Load[0]>80||I.Load[1]>80){Reason=TEXT("pickup_requires_empty_forks");return false;}
    Faulted=false;FailedStage.Empty();Active=true;FullSeen=false;TransferSeen=false;StartS=I.Now;StartPose=I.Pose;HeightTarget=I.Height;ReferenceLoad=I.Load[0]+I.Load[1];Receipt=Place;
    Transition(Place?TEXT("place_raise"):TEXT("pick_height"),I.Now);return true;
}
void Handling::FModel::Step(const FInput& I)
{
    Last=I;Velocity=FVector2D::ZeroVector;YawRate=0;Brake=true;Permit=false;
    if(I.Cancel||I.Request==ERequest::Cancel){if(Working())Stop(TEXT("operator_cancel"));return;}
    if(I.Request!=ERequest::None&&!Active){Start(I,I.Request);return;}
    if(!Active)return;
    if(!I.Healthy||I.Fault){HeightTarget=I.Height;Stop(TEXT("required_sensor_or_controller_fault"));return;}
    if(I.Manual){HeightTarget=I.Height;Stop(TEXT("manual_takeover_requires_new_request"));return;}
    if(I.Now-StartS>Spec.JobTimeout||I.Now-StageS>Spec.StageTimeout){Stop(TEXT("stage_or_job_timeout"));return;}
    if(State!=TEXT("pick_lift")&&State!=TEXT("place_lower")&&((I.Load[0]>80&&I.Load[1]<20)||(I.Load[1]>80&&I.Load[0]<20))){Stop(TEXT("uneven_load"));return;}
    const bool Still=I.Velocity.Size2D()<.025&&FMath::Abs(I.Velocity.Z)<.025;
    auto HeightAt=[&](double H){HeightTarget=H;return FMath::Abs(I.Height-H)<.012;};
    auto Move=[&](double X,double Y,double Yaw,double Cap)
    {
        const double C=FMath::Cos(I.Pose.Yaw),S=FMath::Sin(I.Pose.Yaw),DX=X-I.Pose.X,DY=Y-I.Pose.Y;
        Velocity=FVector2D(FMath::Clamp(.7*(C*DX+S*DY),-Cap,Cap),FMath::Clamp(.7*(-S*DX+C*DY),-Spec.Strafe,Spec.Strafe));YawRate=FMath::Clamp(1.2*FMath::UnwindRadians(Yaw-I.Pose.Yaw),-Spec.Yaw,Spec.Yaw);
        const bool Near=FVector2D(DX,DY).Size()<.018&&FMath::Abs(FMath::UnwindRadians(Yaw-I.Pose.Yaw))<.018;
        if(Near){Velocity=FVector2D::ZeroVector;YawRate=0;Brake=true;}else Brake=false;
        return Stable(Near&&Still,I.Dt);
    };
    const bool PickupGeometry=I.Camera&&I.Geometry&&I.Now-I.PerceptionStamp<=Spec.Stale;
    if(State==TEXT("pick_height"))
    {if(Stable(HeightAt(Tier+.035)&&Still,I.Dt))Transition(TEXT("acquire"),I.Now);}
    else if(State==TEXT("acquire"))
    {if(Stable(PickupGeometry&&I.Vision&&Still,I.Dt))Transition(TEXT("align"),I.Now);}
    else if(State==TEXT("align")||State==TEXT("insert"))
    {
        if(!PickupGeometry){Stop(TEXT("pallet_track_or_camera_lost"));return;}
        Permit=true;
        if(!I.Measurements)
        {
            StableS=0;Reason=TEXT("transient_measurement_gap_hold");if(VisionLostS<0)VisionLostS=I.Now;
            if(I.Now-VisionLostS>.25)Stop(TEXT("insertion_measurement_gap"));return;
        }
        VisionLostS=-1;Reason.Empty();
        if(FMath::Max(I.Depth[0],I.Depth[1])>1.15){Stop(TEXT("insertion_depth_limit"));return;}
        Permit=true;
        const double Depth=FMath::Min(I.Depth[0],I.Depth[1]);
        if(State==TEXT("align"))
        {
            HeightTarget=FMath::Clamp(I.Height+I.Z,0.,2.1);
            if(Depth>.08&&(FMath::Abs(I.Y)>.04||FMath::Abs(I.Yaw)>.04)){Stop(TEXT("alignment_requires_withdrawal"));return;}
            Velocity.Y=FMath::Clamp(I.Y*.8,-Spec.Strafe,Spec.Strafe);YawRate=FMath::Clamp(I.Yaw*.8,-Spec.Yaw,Spec.Yaw);Brake=false;
            const bool Ready=FMath::Abs(I.Y)<.018&&FMath::Abs(I.Yaw)<.018&&FMath::Abs(I.Z)<.008;
            if(Ready){Velocity=FVector2D::ZeroVector;YawRate=0;Brake=true;}
            if(Stable(Ready&&Still,I.Dt))Transition(TEXT("insert"),I.Now);
        }
        else
        {
            if(!I.Aligned||FMath::Abs(I.Y)>.045||FMath::Abs(I.Yaw)>.04){StableS=0;Reason=TEXT("alignment_recheck_hold");if(AlignmentSince<0)AlignmentSince=I.Now;if(I.Now-AlignmentSince>1)Stop(TEXT("insertion_alignment_lost"));return;}
            AlignmentSince=-1;Reason.Empty();
            // No yaw/side correction inside occupied channels.
            Velocity.X=FMath::Clamp((Spec.Depth-Depth)*.8,0.,Spec.Speed);Brake=Depth>=Spec.Depth;
            if(Brake)Velocity.X=0;
            if(!I.Full)StableS=0;
            if(I.Full){FullSeen=true;if(Stable(Still&&Depth>=Spec.Depth-.006,I.Dt)){HeightTarget=Tier+Spec.Clear;Transition(TEXT("pick_lift"),I.Now);}}
        }
    }
    else if(State==TEXT("pick_lift"))
    {
        Permit=true;HeightTarget=Tier+Spec.Clear;
        if(!FullSeen){Stop(TEXT("lift_without_full_insertion"));return;}
        if(Stable(HeightAt(HeightTarget)&&I.Load[0]>80&&I.Load[1]>80&&Still,I.Dt))
        {ReferenceLoad=I.Load[0]+I.Load[1];Receipt=TransferSeen=true;WithdrawPose=I.Pose;Transition(TEXT("pick_withdraw"),I.Now);}
    }
    else if(State==TEXT("pick_withdraw")||State==TEXT("place_withdraw"))
    {
        Permit=true;const bool Pick=State==TEXT("pick_withdraw");
        if(Pick&&(I.Load[0]+I.Load[1]<ReferenceLoad*.55)){Stop(TEXT("carried_load_lost"));return;}
        if(!Pick&&I.Load[0]+I.Load[1]>FMath::Max(100.,ReferenceLoad*.15)){Stop(TEXT("load_recontact_during_withdrawal"));return;}
        if(Move(WithdrawPose.X-Spec.Withdraw*FMath::Cos(WithdrawPose.Yaw),WithdrawPose.Y-Spec.Withdraw*FMath::Sin(WithdrawPose.Yaw),WithdrawPose.Yaw,Spec.Speed))Transition(TEXT("lower_to_carry"),I.Now);
    }
    else if(State==TEXT("lower_to_carry"))
    {if(Receipt&&I.Load[0]+I.Load[1]<ReferenceLoad*.55){Stop(TEXT("carried_load_lost"));return;}if(Stable(HeightAt(Spec.Carry)&&Still,I.Dt)){Active=false;Transition(TEXT("complete"),I.Now);}}
    else if(State==TEXT("place_raise"))
    {
        if(I.Load[0]+I.Load[1]<ReferenceLoad*.55){Stop(TEXT("carried_load_lost"));return;}
        if(Stable(HeightAt(Tier+Spec.Clear)&&Still,I.Dt))Transition(TEXT("place_approach"),I.Now);
    }
    else if(State==TEXT("place_approach"))
    {
        HeightTarget=Tier+Spec.Clear;Permit=true;
        if(I.Load[0]+I.Load[1]<ReferenceLoad*.55){Stop(TEXT("carried_load_lost"));return;}
        if(Move(Spec.RackX-Spec.ForkCenter,Spec.RackY,0,Spec.Speed))Transition(TEXT("place_lower"),I.Now);
    }
    else if(State==TEXT("place_lower"))
    {
        Permit=true;HeightTarget=Tier+.035;
        const bool Unloaded=I.Load[0]+I.Load[1]<FMath::Max(50.,ReferenceLoad*.03);
        if(Stable(HeightAt(HeightTarget)&&Unloaded&&Still,I.Dt)){Receipt=false;TransferSeen=true;WithdrawPose=I.Pose;Transition(TEXT("place_withdraw"),I.Now);}
    }
    if(I.Obstacle&&State!=TEXT("pick_height")&&State!=TEXT("acquire")&&State!=TEXT("place_raise")){HeightTarget=I.Height;Stop(TEXT("non_target_obstacle"));}
}
bool Handling::FModel::TargetPoint(FVector2D P) const
{
    if(!Active||!Permit||!Last.Healthy||Last.Fault)return false;
    // Spatial association only, never actor tags. Points outside the target
    // pallet envelope / surveyed rack posts remain protected. Not safety-rated.
    if(!Place&&Last.Geometry&&Last.Camera)
    {
        const double X=Spec.ForkCenter+1.1-.6079257-FMath::Min(Last.Depth[0],Last.Depth[1]);
        if(P.X>=X-.12&&P.X<=X+1.34&&FMath::Abs(P.Y-Last.Y)<.62)return true;
    }
    if(Receipt||(State==TEXT("pick_lift")&&FullSeen)||State==TEXT("place_withdraw"))
    {
        if(State!=TEXT("place_withdraw")&&Last.HeelValid&&FMath::Min(Last.HeelDepth[0],Last.HeelDepth[1])>.99&&FMath::Abs(Last.HeelDepth[0]-Last.HeelDepth[1])<.08)
        {
            const double Front=Spec.ForkCenter+.6-(Last.HeelDepth[0]+Last.HeelDepth[1])*.5;
            if(P.X>=Front-.12&&P.X<=Front+1.2158514+.12&&FMath::Abs(P.Y)<.62)return true;
        }
        if(P.X>=Spec.ForkCenter-.64&&P.X<=Spec.ForkCenter+.64&&FMath::Abs(P.Y)<.62)return true;
    }
    if(Place||Tier>0)
    {
        const double C=FMath::Cos(Last.Pose.Yaw),S=FMath::Sin(Last.Pose.Yaw);
        const FVector2D W(Last.Pose.X+C*P.X-S*P.Y,Last.Pose.Y+S*P.X+C*P.Y);
        for(double DX:{-Spec.RackDepth*.5,Spec.RackDepth*.5})for(double DY:{-Spec.RackWidth*.5-.05,Spec.RackWidth*.5+.05})
            if(FMath::Abs(W.X-Spec.RackX-DX)<.12&&FMath::Abs(W.Y-Spec.RackY-DY)<.12)return true;
    }
    return false;
}
