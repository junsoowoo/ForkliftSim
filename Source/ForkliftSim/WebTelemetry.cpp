#include "WebTelemetry.h"
#include "CameraSensors.h"
#include "JointSensors.h"
#include "DynamicsSensors.h"
#include "ProximitySensors.h"
#include "SensorLiftControl.h"
#include "LidarSafety.h"
#include "SensorDrive.h"
#include "PalletPerception.h"
#include "HandlingControl.h"
#include "MissionNavigation.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "WheelActor.h"
#include "ForkActor.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"
#include "HttpModule.h"
#include "Interfaces/IHttpResponse.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace
{
    int64 NowMs() { const FDateTime T=FDateTime::UtcNow(); return T.ToUnixTimestamp()*1000+T.GetMillisecond(); }
    void Number(const TSharedRef<FJsonObject>& J,const TCHAR* K,double V)
    { if(FMath::IsFinite(V)) J->SetNumberField(K,V); else J->SetField(K,MakeShared<FJsonValueNull>()); }
    TArray<TSharedPtr<FJsonValue>> Vector(const FVector& V)
    { return {MakeShared<FJsonValueNumber>(V.X),MakeShared<FJsonValueNumber>(V.Y),MakeShared<FJsonValueNumber>(V.Z)}; }
    TSharedRef<FJsonObject> Body(UStaticMeshComponent* B,UStaticMeshComponent* Base)
    {
        auto J=MakeShared<FJsonObject>();
        Number(J,TEXT("mass_kg"),B->GetMass());
        J->SetArrayField(TEXT("inertia_mass_frame_kg_m2"),Vector(B->GetInertiaTensor()/10000.));
        const FQuat MassQ=B->BodyInstance.GetMassSpaceLocal().GetRotation();
        J->SetArrayField(TEXT("mass_frame_rotation_xyzw"),{MakeShared<FJsonValueNumber>(MassQ.X),MakeShared<FJsonValueNumber>(MassQ.Y),MakeShared<FJsonValueNumber>(MassQ.Z),MakeShared<FJsonValueNumber>(MassQ.W)});
        J->SetArrayField(TEXT("world_position_m"),Vector(B->GetComponentLocation()/100.));
        J->SetArrayField(TEXT("local_position_m"),Vector(Base->GetComponentQuat().UnrotateVector(B->GetComponentLocation()-Base->GetComponentLocation())/100.));
        J->SetArrayField(TEXT("center_of_mass_world_m"),Vector(B->GetCenterOfMass()/100.));
        const FRotator A=B->GetComponentRotation(); J->SetArrayField(TEXT("rotation_deg"),Vector(FVector(A.Roll,A.Pitch,A.Yaw)));
        return J;
    }
}
TSharedPtr<FWebTelemetryPublisher,ESPMode::ThreadSafe> FWebTelemetryPublisher::Create()
{
    FString Url;
    if(!FParse::Value(FCommandLine::Get(),TEXT("WebTelemetry="),Url))
    { if(!FParse::Param(FCommandLine::Get(),TEXT("WebTelemetry"))) return nullptr; Url=TEXT("http://127.0.0.1:4173/api/telemetry"); }
    FString Port;
    const FString Prefix=TEXT("http://127.0.0.1:");
    if(Url.StartsWith(Prefix) && Url.EndsWith(TEXT("/api/telemetry"))) Port=Url.Mid(Prefix.Len(),Url.Len()-Prefix.Len()-14);
    if(Port.IsEmpty() || !Port.IsNumeric() || FCString::Atoi(*Port)<1024 || FCString::Atoi(*Port)>65535)
    { UE_LOG(LogTemp,Error,TEXT("WEB_TELEMETRY requires http://127.0.0.1:<port>/api/telemetry")); return nullptr; }
    auto P=MakeShared<FWebTelemetryPublisher,ESPMode::ThreadSafe>();
    P->Url=Url; P->WorldId=FGuid::NewGuid().ToString(EGuidFormats::Digits); P->StartedMs=NowMs();
    UE_LOG(LogTemp,Display,TEXT("WEB_TELEMETRY enabled %s (20 Hz wall-clock, asynchronous)"),*Url);
    return P;
}
void FWebTelemetryPublisher::NewRun() { RunId=FGuid::NewGuid().ToString(EGuidFormats::Digits); NextWallS=0; }
bool FWebTelemetryPublisher::IsDue() const { return !bStopped && !Pending.IsValid() && FPlatformTime::Seconds()>=NextWallS; }
void FWebTelemetryPublisher::Send(TSharedRef<FJsonObject> Frame) { if(IsDue()) Dispatch(Frame,false); }
void FWebTelemetryPublisher::Dispatch(TSharedRef<FJsonObject> Frame,bool Final)
{
    Frame->SetNumberField(TEXT("schema"),1);
    auto Source=MakeShared<FJsonObject>();
    Source->SetStringField(TEXT("kind"),TEXT("unreal")); Source->SetStringField(TEXT("world_id"),WorldId); Source->SetStringField(TEXT("run_id"),RunId);
    Source->SetNumberField(TEXT("started_at_ms"),double(StartedMs)); Source->SetNumberField(TEXT("sent_at_ms"),double(NowMs()));
    Source->SetNumberField(TEXT("sequence"),double(++Sequence)); Source->SetStringField(TEXT("state"),Final?TEXT("stopped"):TEXT("running"));
    Frame->SetObjectField(TEXT("source"),Source); LastFrame=Frame;
    FString Text; FJsonSerializer::Serialize(Frame,TJsonWriterFactory<TCHAR,TCondensedJsonPrintPolicy<TCHAR>>::Create(&Text));
    auto Request=FHttpModule::Get().CreateRequest(); Request->SetURL(Url); Request->SetVerb(TEXT("POST"));
    Request->SetHeader(TEXT("Content-Type"),TEXT("application/json")); Request->SetContentAsString(Text); Request->SetTimeout(1.5f);
    if(!Final)
    {
        Pending=Request; const TWeakPtr<FWebTelemetryPublisher,ESPMode::ThreadSafe> Weak=AsShared();
        Request->OnProcessRequestComplete().BindLambda([Weak](FHttpRequestPtr,FHttpResponsePtr Response,bool Success)
        {
            if(auto P=Weak.Pin())
            {
                P->Pending.Reset(); const bool Ok=Success && Response.IsValid() && Response->GetResponseCode()==202;
                if(!Ok) { P->NextWallS=FPlatformTime::Seconds()+.5; if(!P->bReportedFailure) UE_LOG(LogTemp,Warning,TEXT("WEB_TELEMETRY server unavailable; physics continues, retrying")); }
                else if(P->bReportedFailure) UE_LOG(LogTemp,Display,TEXT("WEB_TELEMETRY connection restored"));
                P->bReportedFailure=!Ok;
            }
        });
    }
    NextWallS=FPlatformTime::Seconds()+.05;
    if(!Request->ProcessRequest()) { Pending.Reset(); NextWallS=FPlatformTime::Seconds()+.5; }
}
void FWebTelemetryPublisher::Stop()
{
    bStopped=true;
    if(Pending) { Pending->OnProcessRequestComplete().Unbind(); Pending->CancelRequest(); Pending.Reset(); }
    if(LastFrame) Dispatch(LastFrame.ToSharedRef(),true);
}
void AStabilityLab::PublishWebTelemetry()
{
    if(!WebTelemetry || !WebTelemetry->IsDue() || !Chassis || !Fork || Wheels.Num()!=4) return;
    auto Frame=MakeShared<FJsonObject>();
    if(CameraSensors)Frame->SetObjectField(TEXT("camera_sensors"),CameraSensors->Telemetry(TimeS,bCameraViews));
    if(JointSensors){auto J=JointSensors->Telemetry(TimeS);J->SetBoolField(TEXT("controller_connected"),SensorLift.IsValid());J->SetBoolField(TEXT("lift_controller_connected"),SensorLift.IsValid());Frame->SetObjectField(TEXT("joint_sensors"),J);}
    if(DynamicsSensors){auto J=DynamicsSensors->Telemetry(TimeS);J->SetBoolField(TEXT("controller_connected"),SensorLift.IsValid());J->SetBoolField(TEXT("lift_controller_connected"),SensorLift.IsValid());Frame->SetObjectField(TEXT("dynamics_sensors"),J);}
    if(ProximitySensors){auto P=ProximitySensors->Telemetry(TimeS);P->SetBoolField(TEXT("controller_connected"),HandlingControl.IsValid());Frame->SetObjectField(TEXT("proximity_sensors"),P);}
    if(SensorLift)Frame->SetObjectField(TEXT("lift_control"),SensorLift->Telemetry());
    else {auto Control=MakeShared<FJsonObject>();Control->SetStringField(TEXT("mode"),bSensorLiftRequested?TEXT("sensor_control_unavailable_hold"):TEXT("legacy_physics_test_or_fixture"));Frame->SetObjectField(TEXT("lift_control"),Control);}
    if(VehicleGuard)
    {
        auto J=VehicleGuard->Telemetry(TimeS);const bool Nav=MissionControl&&MissionControl->NavField(TimeS);J->SetStringField(TEXT("protection_mode"),Nav?TEXT("mission_fixed_heading_sweep"):TEXT("full_padding"));
        if(Nav){const auto& M=MissionControl->Model;const double V=M.Velocity.Size(),Extra=.12+.35*V+V*V;const auto D=M.Velocity.GetSafeNormal()*Extra;J->SetNumberField(TEXT("field_front_m"),M.Spec.Front+M.Spec.Margin+FMath::Max(0.,D.X));J->SetNumberField(TEXT("field_rear_m"),M.Spec.Rear+M.Spec.Margin-FMath::Min(0.,D.X));J->SetNumberField(TEXT("field_half_width_m"),M.Spec.HalfWidth+M.Spec.Margin+FMath::Abs(D.Y));}
        Frame->SetObjectField(TEXT("lidar_safety"),J);
    }
    if(DriveSensors)Frame->SetObjectField(TEXT("drive_estimator"),DriveSensors->Telemetry(TimeS));
    if(MissionControl)Frame->SetObjectField(TEXT("mission"),MissionControl->Telemetry());
    if(HandlingControl)Frame->SetObjectField(TEXT("handling_control"),HandlingControl->Telemetry());
    if(PalletRecognition)Frame->SetObjectField(TEXT("pallet_perception"),PalletRecognition->Telemetry(TimeS));
    Number(Frame,TEXT("sim_time_s"),TimeS); Number(Frame,TEXT("measurement_interval_s"),PreviousPhysicsDt);
    Number(Frame,TEXT("physics_elapsed_s"),ValidationState.PhysicsSampleS);
    Number(Frame,TEXT("actual_realtime_ratio"),RealtimeRatio);Number(Frame,TEXT("game_fps"),FrameWallSmoothedS>0?1./FrameWallSmoothedS:0);
    Number(Frame,TEXT("physics_clock_lag_s"),TimeS-ValidationState.PhysicsSampleS);
    Number(Frame,TEXT("physics_max_inner_s"),ValidationState.MaxInnerStepS);
    Number(Frame,TEXT("physics_selected_step_s"),SelectedPhysicsStepS);Number(Frame,TEXT("physics_smallest_observed_s"),SmallestObservedPhysicsStepS);
    Number(Frame,TEXT("physics_inner_clock_error_s"),MaxInnerClockErrorS);
    Frame->SetStringField(TEXT("bearing_integrator"),Settings.Mecanum.bReduced?TEXT("roller_coordinates_eliminated"):Settings.bMecanum && Settings.Mecanum.bBearingIntegrator && !FParse::Param(FCommandLine::Get(),TEXT("MecanumLegacyIntegration"))?TEXT("symmetric_spin_verlet"):TEXT("legacy"));
    Number(Frame,TEXT("game_max_frame_s"),ValidationState.MaxGameStepS);
    Number(Frame,TEXT("playback_rate"),GetWorld()->GetWorldSettings()->TimeDilation);
    Frame->SetStringField(TEXT("drive_update"),Settings.bMecanum?TEXT("physics_inner_step"):TEXT("game_frame"));
    Frame->SetBoolField(TEXT("contact_sample_valid"),PreviousPhysicsDt>0);
    auto Energy=MakeShared<FJsonObject>(); const auto& A=EnergyAudit; const auto& EH=Fork->HydraulicState;
    Energy->SetBoolField(TEXT("reference_valid"),A.bReferenced); Energy->SetBoolField(TEXT("full_closure"),false);
    Energy->SetStringField(TEXT("scope"),TEXT("Stored energy + observed drive/steering work + hydraulic DC/heat + chain damping + tyre/rolling loss; residual includes unmeasured contact, structure/body loss and numerical error."));
    Number(Energy,TEXT("shaft_loss_j"),A.ShaftLossJ);Number(Energy,TEXT("shaft_joint_work_j"),A.ShaftJointJ-A.ReferenceShaftJointJ);
    Number(Energy,TEXT("bearing_transport_work_j"),A.BearingTransportJ-A.ReferenceBearingTransportJ);
    Number(Energy,TEXT("chain_damping_j"),EH.ChainLossJ-A.ReferenceChainLossJ);
    Number(Energy,TEXT("reduced_contact_loss_j"),A.ReducedLossJ-A.RefReducedLossJ);
    Number(Energy,TEXT("plastic_relaxation_j"),A.PlasticRelaxationJ-A.RefPlasticRelaxationJ);Number(Energy,TEXT("fracture_release_j"),A.FractureReleaseJ-A.RefFractureReleaseJ);
    Number(Energy,TEXT("contact_normal_work_estimate_j"),A.ContactNormalJ-A.RefContactNormalJ);Number(Energy,TEXT("contact_tangent_work_estimate_j"),A.ContactTangentJ-A.RefContactTangentJ);
    Number(Energy,TEXT("contact_correction_pseudo_j"),A.CorrectionPseudoJ-A.RefCorrectionPseudoJ);Number(Energy,TEXT("fork_damping_estimate_j"),A.ForkDampingEstimateJ-A.RefForkDampingEstimateJ);
    Energy->SetStringField(TEXT("contact_work_scope"),TEXT("Signed whole-solve mean-velocity diagnostic; joint/contact cross terms unresolved. Correction is pseudo-work. Estimates excluded from known_loss_j."));
    Number(Energy,TEXT("stored_delta_j"),A.DeltaStoredJ); Number(Energy,TEXT("drive_work_j"),A.DriveJ-A.ReferenceDriveJ);
    Number(Energy,TEXT("steering_joint_work_j"),A.SteeringJ-A.ReferenceSteeringJ); Number(Energy,TEXT("rolling_work_j"),A.RollingJ-A.ReferenceRollingJ);
    Number(Energy,TEXT("hydraulic_dc_j"),EH.DcJ-A.ReferenceDcJ); Number(Energy,TEXT("hydraulic_heat_j"),EH.HeatJ-A.ReferenceHeatJ);
    Number(Energy,TEXT("tyre_damping_j"),TyreState.DampingJ-A.ReferenceTyreJ); Number(Energy,TEXT("known_input_j"),A.InputJ);
    Number(Energy,TEXT("roller_bearing_loss_j"),-(A.RollerBearingJ-A.ReferenceRollerBearingJ));
    Number(Energy,TEXT("known_loss_j"),A.KnownLossJ); Number(Energy,TEXT("unresolved_j"),A.UnresolvedJ); Number(Energy,TEXT("positive_peak_j"),A.PeakPositiveJ);
    Frame->SetObjectField(TEXT("energy_audit"),Energy);
    auto Rack=MakeShared<FJsonObject>();Rack->SetBoolField(TEXT("present"),HandlingRack!=nullptr);
    Number(Rack,TEXT("support_n"),RackSupportN);Number(Rack,TEXT("lower_top_m"),RackSettings.Low);Number(Rack,TEXT("upper_top_m"),RackSettings.High);
    Number(Rack,TEXT("trial_stage"),RackTrial.Stage);Frame->SetObjectField(TEXT("handling_rack"),Rack);
    Frame->SetStringField(TEXT("world_name"),GetWorld()->GetName()); Frame->SetStringField(TEXT("scenario"),TestMode.IsEmpty()?TEXT("manual"):TestMode);
    Frame->SetStringField(TEXT("terrain"),TerrainPreset); Frame->SetBoolField(TEXT("has_payload"),bPayload && Pallet);
    auto Controls=MakeShared<FJsonObject>();
    Number(Controls,TEXT("brake"),Brake); Number(Controls,TEXT("throttle"),Throttle);
    Number(Controls,TEXT("steering"),Steering); Number(Controls,TEXT("brake_cap_per_wheel_nm"),Settings.BrakeNm);
    Controls->SetBoolField(TEXT("auto_hold"),bMecanumAutoHold);Frame->SetObjectField(TEXT("controls"),Controls);
    Number(Controls,TEXT("strafe"),Strafe);
    Frame->SetStringField(TEXT("drive_type"),Settings.bMecanum?TEXT("mecanum"):Settings.bSwerve?TEXT("swerve"):TEXT("rear_steer"));
    auto Vehicle=Body(Chassis->Body,Chassis->Body);
    Number(Vehicle,TEXT("dry_mass_kg"),MassKg); Number(Vehicle,TEXT("speed_mps"),SpeedMps); Number(Vehicle,TEXT("lateral_speed_mps"),LateralMps);
    Number(Vehicle,TEXT("acceleration_mps2"),AccelerationMps2); Number(Vehicle,TEXT("yaw_rate_deg_s"),YawDegS);
    Number(Vehicle,TEXT("roll_deg"),RollDeg); Number(Vehicle,TEXT("pitch_deg"),PitchDeg); Number(Vehicle,TEXT("normal_sum_n"),TotalNormalLoadN);
    Vehicle->SetNumberField(TEXT("near_floor_wheels"),Grounded); Vehicle->SetBoolField(TEXT("neutral"),bNeutral); Frame->SetObjectField(TEXT("vehicle"),Vehicle);
    auto SettingsJson=MakeShared<FJsonObject>();
    Number(SettingsJson,TEXT("tyre_mu"),Settings.TyreMu); Number(SettingsJson,TEXT("road_mu"),Settings.RoadMu);
    Number(SettingsJson,TEXT("crr"),Settings.RollingResistance); Number(SettingsJson,TEXT("wheelbase_m"),Settings.WheelbaseM); Number(SettingsJson,TEXT("track_m"),Settings.TrackM);
    Number(SettingsJson,TEXT("wheel_radius_m"),Settings.RadiusM); SettingsJson->SetBoolField(TEXT("resistance_enabled"),bResistanceEnabled);
    Frame->SetObjectField(TEXT("settings"),SettingsJson);
    TArray<TSharedPtr<FJsonValue>> WheelArray;
    const TCHAR* Names[]={TEXT("FL"),TEXT("FR"),TEXT("RL"),TEXT("RR")};
    for(int I=0;I<4;++I)
    {
        const AWheelActor* W=Wheels[I]; auto J=Body(W->Tyre,Chassis->Body); J->SetStringField(TEXT("id"),Names[I]);
        Number(J,TEXT("normal_n"),W->NormalLoadN); Number(J,TEXT("normal_filtered_n"),W->NormalLoadFilteredN);
        Number(J,TEXT("friction_n"),W->ContactFrictionForceN.Size()); J->SetArrayField(TEXT("friction_world_n"),Vector(W->ContactFrictionForceN));
        if(W->FloorContacts>0) Number(J,TEXT("contact_mu"),W->ContactMu); else J->SetField(TEXT("contact_mu"),MakeShared<FJsonValueNull>());
        J->SetNumberField(TEXT("contact_events"),W->FloorContacts); J->SetBoolField(TEXT("near_floor"),W->bGrounded);
        Number(J,TEXT("compression_m"),W->CompressionM); Number(J,TEXT("omega_rad_s"),W->OmegaRadS);
        Number(J,TEXT("shear_x_m"),W->ShearXM); Number(J,TEXT("shear_y_m"),W->ShearYM);
        Number(J,TEXT("estimated_patch_length_m"),W->EstimatedPatchLengthM); Number(J,TEXT("estimated_peak_pressure_pa"),W->EstimatedPeakPressurePa);
        const auto& Contact=W->ContactData; auto ContactJson=MakeShared<FJsonObject>();
        const double ContactDt=PreviousPhysicsDt>0?PreviousPhysicsDt:1;
        ContactJson->SetBoolField(TEXT("valid"),PreviousPhysicsDt>0 && Contact.NormalNs>1e-8);
        Number(ContactJson,TEXT("point_normal_sum_n"),Contact.NormalNs/ContactDt);
        Number(ContactJson,TEXT("peak_active_points"),Contact.PeakActivePoints);
        ContactJson->SetArrayField(TEXT("band_normal_n"),{MakeShared<FJsonValueNumber>(Contact.BandNs[0]/ContactDt),MakeShared<FJsonValueNumber>(Contact.BandNs[1]/ContactDt),MakeShared<FJsonValueNumber>(Contact.BandNs[2]/ContactDt)});
        if(Contact.NormalNs>1e-8) ContactJson->SetArrayField(TEXT("load_center_world_m"),Vector(Contact.PointWeightedMNs/Contact.NormalNs));
        else ContactJson->SetField(TEXT("load_center_world_m"),MakeShared<FJsonValueNull>());
        ContactJson->SetArrayField(TEXT("normal_world_n"),Vector(Contact.NormalImpulseNs/ContactDt));
        ContactJson->SetArrayField(TEXT("normal_moment_world_nm"),Vector(Contact.NormalMomentNms/ContactDt));
        J->SetObjectField(TEXT("solver_contact"),ContactJson);
        auto Tread=MakeShared<FJsonObject>(); const auto& Spec=W->Compliance.Tread;
        Tread->SetBoolField(TEXT("enabled"),!W->TreadPads.IsEmpty());
        Tread->SetBoolField(TEXT("coupled"),Spec.bCoupled && !W->TreadPads.IsEmpty());
        Tread->SetBoolField(TEXT("aggregate_shear_spring"),W->Compliance.bEnabled && W->Compliance.bDetailed && !Spec.bCoupled);
        J->SetBoolField(TEXT("estimated_patch_valid"),!Settings.bMecanum && !Spec.bCoupled);
        Tread->SetBoolField(TEXT("aggregate_radial_spring"),W->Compliance.bEnabled && W->TreadPads.IsEmpty());
        Number(Tread,TEXT("segments"),W->TreadPads.Num()); Number(Tread,TEXT("depth_m"),Spec.DepthM);
        Tread->SetBoolField(TEXT("resolution_qualified"),false); // design status, not a sensor reading
        Number(Tread,TEXT("phase_rad"),Spec.PhaseRad);
        Number(Tread,TEXT("travel_m"),Spec.TravelM); Number(Tread,TEXT("mass_kg"),W->TreadPads.Num()>0?Spec.MassKg:0);
        Number(Tread,TEXT("pad_k_npm"),Spec.PadK(Settings.RadiusM,Settings.WidthM)); Number(Tread,TEXT("pad_c_nspm"),Spec.PadC(Settings.RadiusM,Settings.WidthM));
        Number(Tread,TEXT("max_compression_m"),W->TreadMaxCompressionM); Number(Tread,TEXT("min_compression_m"),W->TreadMinCompressionM);
        Number(Tread,TEXT("stored_j"),W->TreadStoredJ); Number(Tread,TEXT("radial_vertical_estimate_n"),W->TreadSupportN);
        Number(Tread,TEXT("shear_stored_j"),W->TreadShearJ); Number(Tread,TEXT("rotation_stored_j"),W->TreadRotationJ); Number(Tread,TEXT("link_stored_j"),W->TreadLinkJ);
        Number(Tread,TEXT("max_shear_m"),W->TreadMaxShearM); Number(Tread,TEXT("max_rotation_rad"),W->TreadMaxRotationRad); Number(Tread,TEXT("links"),W->TreadLinks.Num());
        Tread->SetBoolField(TEXT("bottomed"),W->bTreadBottomed);
        TArray<TSharedPtr<FJsonValue>> Pads;
        for(int P=0;P<W->TreadPads.Num();++P)
        {
            auto Pad=MakeShared<FJsonObject>(); Number(Pad,TEXT("index"),P); Number(Pad,TEXT("compression_m"),W->TreadCompressionM[P]);
            Number(Pad,TEXT("rate_mps"),W->TreadRateMps[P]); Number(Pad,TEXT("normal_n"),W->TreadNormalN[P]);
            Number(Pad,TEXT("shear_m"),W->TreadShearM[P]); Number(Pad,TEXT("rotation_rad"),W->TreadRotationRad[P]);
            Pad->SetArrayField(TEXT("world_position_m"),Vector(W->TreadPads[P]->GetComponentLocation()/100.));
            Pads.Add(MakeShared<FJsonValueObject>(Pad));
        }
        Tread->SetArrayField(TEXT("pads"),Pads); J->SetObjectField(TEXT("tread"),Tread);
        J->SetBoolField(TEXT("detailed_compliance"),W->Compliance.bEnabled && W->Compliance.bDetailed);
        J->SetBoolField(TEXT("steerable"),W->bSteerable);
        J->SetArrayField(TEXT("motor_reaction_world_nm"),Vector(-W->Knuckle->GetRightVector()*W->DriveTorqueNm));
        Number(J,TEXT("target_surface_speed_mps"),W->SpeedTargetMps);
        if(W->bMecanum)
        {
            auto M=MakeShared<FJsonObject>();M->SetStringField(TEXT("model"),W->bReducedMecanum?TEXT("equivalent-directional-contact-v1"):TEXT("physical-passive-rollers-v1"));M->SetStringField(TEXT("slip_measurement"),W->bReducedMecanum?TEXT("equivalent-traction-axis-slip-rms-passive-axis-omitted"):TEXT("physics-contact-normal-impulse-weighted-rms"));
            M->SetStringField(TEXT("contact_geometry"),W->bReducedMecanum?(FParse::Param(FCommandLine::Get(),TEXT("NativeReducedSupport"))?TEXT("legacy-convex-cylinder"):TEXT("analytic-finite-cylinder-box-v2-directional-friction")):W->bSphereContact?TEXT("inscribed-9-spheres"):TEXT("barrel-convex"));M->SetBoolField(TEXT("rubber_deformation_modelled"),false);M->SetBoolField(TEXT("rated_load_qualified"),false);
            M->SetBoolField(TEXT("independent_roller_states"),!W->bReducedMecanum);Number(M,TEXT("contact_loss_estimate_j"),W->ReducedContactLossJ);Number(M,TEXT("contact_elastic_j"),W->ReducedContactElasticJ);
            Number(M,TEXT("hand"),W->RollerHand);Number(M,TEXT("roller_angle_deg"),45);Number(M,TEXT("hub_mass_kg"),W->Tyre->GetMass());
            if(W->bReducedMecanum)M->SetField(TEXT("bearing_loss_w"),MakeShared<FJsonValueNull>());else Number(M,TEXT("bearing_loss_w"),W->RollerBearingPowerW);
            double WheelMass=W->Tyre->GetMass(); for(const auto& Roller:W->Rollers) WheelMass+=Roller->GetMass();
            Number(M,TEXT("physical_wheel_mass_kg"),WheelMass);
            Number(M,TEXT("contact_inner_peak_n"),W->ContactInnerPeakN);
            M->SetArrayField(TEXT("hub_inertia_mass_frame_kg_m2"),Vector(W->Tyre->GetInertiaTensor()/10000.));
            M->SetStringField(TEXT("catalogue_comparison_model"),Settings.Mecanum.ComparisonModel);
            M->SetStringField(TEXT("catalogue_comparison_url"),Settings.Mecanum.ComparisonUrl);
            Number(M,TEXT("catalogue_listed_load_kg"),Settings.Mecanum.ComparisonLoadKg);
            Number(M,TEXT("catalogue_listed_mass_kg"),Settings.Mecanum.ComparisonMassKg);
            M->SetField(TEXT("catalogue_load_comparison_ratio"),MakeShared<FJsonValueNull>());
            M->SetStringField(TEXT("catalogue_load_status"),TEXT("Manufacturer webpage 3000 kg conflicts with catalogue single-wheel 750 kg; ratio withheld."));
            M->SetStringField(TEXT("property_status"),TEXT("custom geometry/mass/inertia/friction; catalogue comparison only; not calibrated or rated"));
            TArray<TSharedPtr<FJsonValue>> Rs;
            for(int R=0;R<W->Rollers.Num();++R)
            {
                auto B=Body(W->Rollers[R],Chassis->Body);Number(B,TEXT("index"),R);Number(B,TEXT("normal_n"),W->RollerNormalN[R]);
                Number(B,TEXT("relative_omega_rad_s"),W->RollerOmega[R]);Number(B,TEXT("bearing_torque_nm"),W->RollerBearingNm[R]);
                Number(B,TEXT("anchor_error_m"),W->RollerAnchorErrorM[R]);
                B->SetArrayField(TEXT("axis_world"),Vector(W->Rollers[R]->GetForwardVector()));Rs.Add(MakeShared<FJsonValueObject>(B));
            }
            M->SetArrayField(TEXT("rollers"),Rs);J->SetObjectField(TEXT("mecanum"),M);
        }
        Number(J,TEXT("steer_deg"),W->SteerActualDeg); Number(J,TEXT("long_slip_mps"),W->LongSlipMps); if(W->bReducedMecanum)J->SetField(TEXT("lateral_slip_mps"),MakeShared<FJsonValueNull>());else Number(J,TEXT("lateral_slip_mps"),W->LateralSlipMps);
        Number(J,TEXT("drive_torque_nm"),W->DriveTorqueNm); Number(J,TEXT("brake_limit_nm"),W->BrakeLimitNm);
        Number(J,TEXT("brake_elastic_j"),W->BrakeElasticJ); Number(J,TEXT("brake_plastic_work_j"),W->BrakePlasticWorkJ);
        Number(J,TEXT("rolling_torque_nm"),W->RollingTorqueNm); Number(J,TEXT("bearing_torque_nm"),W->BearingTorqueNm); Number(J,TEXT("loss_power_w"),W->ResistancePowerW);
        WheelArray.Add(MakeShared<FJsonValueObject>(J));
    }
    Frame->SetArrayField(TEXT("wheels"),WheelArray);
    auto ForkJson=Body(Fork->Carriage,Chassis->Body);
    Number(ForkJson,TEXT("height_m"),ForkHeightM); Number(ForkJson,TEXT("target_m"),LiftTargetM); Number(ForkJson,TEXT("velocity_mps"),Fork->HydraulicVelocityMps);
    TArray<TSharedPtr<FJsonValue>> Tines;
    for(int I=0;I<Fork->Tines.Num();++I)
    {
        auto T=Body(Fork->Tines[I]->Body,Chassis->Body); T->SetStringField(TEXT("id"),I==0?TEXT("L"):TEXT("R"));
        Number(T,TEXT("vertical_n"),PayloadState.VerticalN[I]); Number(T,TEXT("normal_n"),PayloadState.NormalN[I]); Number(T,TEXT("friction_n"),PayloadState.FrictionN[I]);
        Number(T,TEXT("depth_m"),PayloadState.DepthM[I]); Number(T,TEXT("cop_x_m"),PayloadState.CopXM[I]);
        Number(T,TEXT("contact_mu_config"),FMath::Min(PayloadSettings.ForkMu,I==0?PayloadSettings.LeftHoleMu:PayloadSettings.RightHoleMu));
        const AForkTineActor* Blade=Fork->Tines[I];
        Number(T,TEXT("tip_deflection_m"),Blade->TipDeflectionM); Number(T,TEXT("max_stress_pa"),Blade->MaxStressPa);
        Number(T,TEXT("elastic_j"),Blade->ElasticEnergyJ());
        Number(T,TEXT("plastic_deg"),Blade->PlasticDeg); T->SetBoolField(TEXT("failed"),Blade->bFailed);
        T->SetBoolField(TEXT("flexible"),Blade->Structure.bEnabled); T->SetNumberField(TEXT("section_count"),Blade->Sections.Num());
        T->SetArrayField(TEXT("tip_world_m"),Vector(Blade->Tip()/100.));
        Tines.Add(MakeShared<FJsonValueObject>(T));
    }
    ForkJson->SetArrayField(TEXT("tines"),Tines); ForkJson->SetArrayField(TEXT("load_center_from_heel_m"),Vector(PayloadState.LoadCenterM));
    Frame->SetObjectField(TEXT("fork"),ForkJson);
    const auto& H=Fork->HydraulicState; auto Hydraulic=MakeShared<FJsonObject>();
    Hydraulic->SetBoolField(TEXT("enabled"),Fork->HydraulicSettings.bEnabled); Hydraulic->SetBoolField(TEXT("holding"),H.bHolding);
    Number(Hydraulic,TEXT("pressure_pa"),H.PressurePa); Number(Hydraulic,TEXT("force_n"),H.ForceN); Number(Hydraulic,TEXT("valve"),H.Valve);
    Number(Hydraulic,TEXT("pump_m3s"),H.PumpM3s); Number(Hydraulic,TEXT("return_m3s"),H.ReturnM3s); Number(Hydraulic,TEXT("leak_m3s"),H.LeakM3s);
    Number(Hydraulic,TEXT("relief_m3s"),H.ReliefM3s); Number(Hydraulic,TEXT("dc_w"),H.DcW); Number(Hydraulic,TEXT("mechanical_w"),H.MechanicalW);
    Number(Hydraulic,TEXT("dc_j"),H.DcJ); Number(Hydraulic,TEXT("heat_j"),H.HeatJ); Number(Hydraulic,TEXT("relief_limit_pa"),Fork->HydraulicSettings.ReliefPa);
    Hydraulic->SetBoolField(TEXT("detailed"),Fork->HydraulicSettings.bDetailed);
    Hydraulic->SetBoolField(TEXT("chain_mast"),Fork->HydraulicSettings.bChainMast);
    Number(Hydraulic,TEXT("oil_temp_c"),H.OilTempC); Number(Hydraulic,TEXT("viscosity_cst"),H.ViscosityCst);
    Number(Hydraulic,TEXT("pump_pressure_pa"),H.PumpPressurePa); Number(Hydraulic,TEXT("peak_line_pressure_pa"),H.PeakLinePressurePa);
    Number(Hydraulic,TEXT("line_heat_w"),H.LineHeatW); Number(Hydraulic,TEXT("chain_tension_n"),H.ChainTensionN);
    Number(Hydraulic,TEXT("chain_stretch_m"),H.ChainStretchM); Number(Hydraulic,TEXT("mast_height_m"),H.MastHeightM);
    Number(Hydraulic,TEXT("chain_damping_w"),H.ChainLossW); Number(Hydraulic,TEXT("chain_damping_total_j"),H.ChainLossJ);
    TArray<TSharedPtr<FJsonValue>> Pressures; for(double P:H.LinePressurePa) Pressures.Add(MakeShared<FJsonValueNumber>(P));
    Hydraulic->SetArrayField(TEXT("line_pressures_pa"),Pressures); Hydraulic->SetBoolField(TEXT("pressure_floor_hit"),H.bPressureFloorHit);
    Frame->SetObjectField(TEXT("hydraulic"),Hydraulic);
    if(bPayload && Pallet && Pallet->Cargo)
    {
        auto Payload=Body(Pallet->Deck,Chassis->Body); Number(Payload,TEXT("height_m"),PayloadState.HeightM); Number(Payload,TEXT("ground_n"),PayloadState.GroundN);
        Number(Payload,TEXT("total_mass_kg"),Pallet->LoadMass()); Number(Payload,TEXT("hole_shift_y_m"),PayloadSettings.HoleShiftYM);
        Number(Payload,TEXT("fork_spacing_m"),PayloadSettings.ForkSpacingM); Frame->SetObjectField(TEXT("pallet"),Payload);
        auto Cargo=Body(Pallet->Cargo->GetStaticMeshComponent(),Chassis->Body);
        Number(Cargo,TEXT("normal_n"),CargoState.NormalN); Number(Cargo,TEXT("friction_n"),CargoState.FrictionN);
        Number(Cargo,TEXT("slip_mps"),CargoState.SlipMps); Number(Cargo,TEXT("tilt_deg"),CargoState.TiltDeg); Number(Cargo,TEXT("ground_n"),CargoState.GroundN);
        Cargo->SetArrayField(TEXT("shift_m"),Vector(CargoState.ShiftM)); Cargo->SetArrayField(TEXT("contact_point_world_m"),Vector(CargoState.ContactPointM));
        Number(Cargo,TEXT("static_mu_config"),FMath::Min(PayloadSettings.PalletTopStaticMu,PayloadSettings.CargoStaticMu));
        Number(Cargo,TEXT("dynamic_mu_config"),FMath::Min(PayloadSettings.PalletTopDynamicMu,PayloadSettings.CargoDynamicMu));
        Cargo->SetBoolField(TEXT("fell"),CargoState.bFell); Cargo->SetBoolField(TEXT("tipped"),CargoState.bTipped); Cargo->SetBoolField(TEXT("lost_support"),CargoState.bLostSupport);
        Frame->SetObjectField(TEXT("cargo"),Cargo);
    }
    WebTelemetry->Send(Frame);
}
