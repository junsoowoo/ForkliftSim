#include "StabilityLab.h"
#include "ChassisActor.h"
#include "WheelActor.h"
#include "ForkActor.h"
#include "Vehicle3DHelpers.h"
#include "SwerveKinematics.h"
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"
#include "Engine/StaticMeshActor.h"
#include "EngineUtils.h"
#include "HighResScreenshot.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/Parse.h"
#include "HAL/FileManager.h"
#include "DrawDebugHelpers.h"
#include "ContactLoadSampler.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "PBDRigidsSolver.h"
#include "TerrainCourse.h"
#include "WebTelemetry.h"
#include "Modeling.h"
#include "JointSensors.h"
#include "DynamicsSensors.h"
#include "ProximitySensors.h"
#include "SensorLiftControl.h"
#include "LidarSafety.h"
#include "SensorDrive.h"
#include "PalletPerception.h"
#include "HandlingControl.h"
#include "MissionNavigation.h"

namespace { AStabilityLab* Lab(UWorld* W) { for(TActorIterator<AStabilityLab> I(W);I;++I) return *I; return nullptr; } }
AStabilityLab::AStabilityLab() { PrimaryActorTick.bCanEverTick=true; PrimaryActorTick.TickGroup=TG_PrePhysics; }
void AStabilityLab::BeginPlay()
{
    Super::BeginPlay();
    bCameraViews=FParse::Param(FCommandLine::Get(),TEXT("CameraViews"));
    FParse::Value(FCommandLine::Get(),TEXT("StabilityTest="),TestMode);
    if (!Settings.Load(Error))
    {
        UE_LOG(LogTemp,Error,TEXT("STABILITY_CONFIG %s"),*Error);
        if(!TestMode.IsEmpty()) FGenericPlatformMisc::RequestExitWithStatus(false,1);
        return;
    }
    if(FParse::Param(FCommandLine::Get(),TEXT("ReferencePhysics"))) Settings=FVehicle3DSettings();
    // Preserve the documented 0.3 regression cases; manual and swerve_* use 0.4.
    Settings.bSwerve=TestMode.IsEmpty() || TestMode.StartsWith(TEXT("swerve_")) || TestMode.StartsWith(TEXT("mecanum_"));
    Settings.bMecanum=TestMode.IsEmpty() || TestMode.StartsWith(TEXT("mecanum_")) || FParse::Param(FCommandLine::Get(),TEXT("Mecanum"));
    if(FParse::Param(FCommandLine::Get(),TEXT("LegacySwerve")) || FParse::Param(FCommandLine::Get(),TEXT("CoupledTread")) || FParse::Param(FCommandLine::Get(),TEXT("DeformableTread"))) Settings.bMecanum=false;
    if(Settings.bMecanum)
    {
        if(!Settings.Mecanum.Load(Error) || Settings.Mecanum.Count*Settings.Mecanum.RollerMassKg>=Settings.WheelMassKg-5 || Settings.Mecanum.RollerRadiusM>=Settings.RadiusM*.5 || Settings.Mecanum.HubRadiusM+2*Settings.Mecanum.RollerRadiusM>=Settings.RadiusM)
        {UE_LOG(LogTemp,Error,TEXT("MECANUM_CONFIG %s"),*Error);FGenericPlatformMisc::RequestExitWithStatus(false,1);return;}
        Settings.WidthM=Settings.Mecanum.WidthM;
        if(TestMode==TEXT("mecanum_grip_low")) Settings.RoadMu=.05;
    }
    if(FParse::Param(FCommandLine::Get(),TEXT("LegacyRearSteer"))) Settings.bSwerve=false;
    NormalComOffset=Settings.ChassisComOffsetM; NormalMaxSpeed=Settings.MaxSpeedMps;
    NormalWheelSurfaceLimit=Settings.WheelSurfaceSpeedLimitMps;
    NormalSteerRate=Settings.SteerRateDegS; NormalMaxSteer=Settings.MaxSteerDeg;
    InitialRoadMu=Settings.RoadMu;
    bCapture=FParse::Param(FCommandLine::Get(),TEXT("StabilityCapture"));
    FParse::Value(FCommandLine::Get(),TEXT("StabilityCaptureTime="),CaptureTimeS);
    CaptureTimeS=FMath::Clamp(CaptureTimeS,.5,60.);
    if (TestMode==TEXT("low_grip") || TestMode==TEXT("powered_turn_low")) Settings.RoadMu=.08;
    if (TestMode==TEXT("low_tyre")) Settings.TyreMu=.08;
    if (TestMode==TEXT("zero_grip")) Settings.RoadMu=0;
    if (TestMode==TEXT("swerve_zero_grip")) Settings.RoadMu=0;
    if (TestMode==TEXT("mecanum_zero_grip")) Settings.RoadMu=0;
    if (TestMode==TEXT("swerve_low_grip")) Settings.RoadMu=.08;
    if(TestMode.StartsWith(TEXT("swerve_coast_")))
    {
        // Reproducible coefficient experiments, independent of user's tuning.
        Settings.RollingResistance=.015; Settings.BearingCoulombNm=1; Settings.BearingViscousNms=.05;
        if(TestMode==TEXT("swerve_coast_none") || TestMode==TEXT("swerve_coast_bearing")) Settings.RollingResistance=0;
        if(TestMode==TEXT("swerve_coast_none") || TestMode==TEXT("swerve_coast_rolling")) Settings.BearingCoulombNm=Settings.BearingViscousNms=0;
        // Amplified bearing-only diagnostic: the nominal 1 Nm loss is below
        // the whole-rig contact solver's coast drift. Do not call this calibration.
        if(TestMode==TEXT("swerve_coast_bearing")) { Settings.BearingCoulombNm=8; Settings.BearingViscousNms=.2; }
        if(TestMode==TEXT("swerve_coast_high")) Settings.RollingResistance=.03;
    }
    if (TestMode==TEXT("swerve_rollover"))
    {
        Settings.Actuators.bEnabled=false; // historical ideal-actuator stress comparison
        Settings.Tyres.bEnabled=false; // historical rigid-wheel stress comparison
        Settings.TyreMu=Settings.RoadMu=1.5; Settings.ChassisComOffsetM=1.2;
        Settings.MaxSpeedMps=8; Settings.YawRateDegS=90; Settings.SteerRateDegS=120;
        Settings.WheelSurfaceSpeedLimitMps=8;
    }
    if (TestMode==TEXT("rollover"))
    {
        Settings.TyreMu=1.5; Settings.RoadMu=1.5; Settings.ChassisComOffsetM=1.2;
        Settings.SteerRateDegS=120; Settings.MaxSteerDeg=35;
    }
    if(!ForkliftModel::Configure(Settings,Error,GetWorld()->GetMapName().Contains(TEXT("WarehouseLab"))))
    {UE_LOG(LogTemp,Error,TEXT("MODELING_CONFIG %s"),*Error);FGenericPlatformMisc::RequestExitWithStatus(false,1);return;}
    for (TActorIterator<AStaticMeshActor> I(GetWorld());I;++I)
        if (I->ActorHasTag(TEXT("StabilityRoad"))) Road=I->GetStaticMeshComponent();
    if(Settings.bReferenceGeometry && !RackSettings.Load(Error)){UE_LOG(LogTemp,Error,TEXT("RACK_CONFIG %s"),*Error);return;}
    if(IsHandlingTest()&&(TestMode.Contains(TEXT("place"))||TestMode.EndsWith(TEXT("low"))||TestMode.EndsWith(TEXT("high")))){RackSettings.X=3.2;RackSettings.Y=0;}
    if(!ConfigurePayload()) { UE_LOG(LogTemp,Error,TEXT("PAYLOAD_CONFIG %s"),*Error); if(!TestMode.IsEmpty()) FGenericPlatformMisc::RequestExitWithStatus(false,1); return; }
    if(!ConfigurePowerLab()) { UE_LOG(LogTemp,Error,TEXT("POWER_CONFIG %s"),*Error); if(!TestMode.IsEmpty()) FGenericPlatformMisc::RequestExitWithStatus(false,1); return; }
    if(!ConfigureTyres()) { UE_LOG(LogTemp,Error,TEXT("TYRE_CONFIG %s"),*Error); if(!TestMode.IsEmpty()) FGenericPlatformMisc::RequestExitWithStatus(false,1); return; }
    if(!ConfigureHydraulics()) { UE_LOG(LogTemp,Error,TEXT("HYDRAULIC_CONFIG %s"),*Error); if(!TestMode.IsEmpty()) FGenericPlatformMisc::RequestExitWithStatus(false,1); return; }
    if(!ConfigureValidation()) { UE_LOG(LogTemp,Error,TEXT("VALIDATION_CONFIG %s"),*Error); FGenericPlatformMisc::RequestExitWithStatus(false,1); return; }
    double ReferenceCrr=-1;
    if(FParse::Value(FCommandLine::Get(),TEXT("ReferenceRollingCoefficient="),ReferenceCrr))
    {
        if(TestMode!=TEXT("mecanum_coast") || !FMath::IsFinite(ReferenceCrr) || ReferenceCrr<.001 || ReferenceCrr>.05)
        {UE_LOG(LogTemp,Error,TEXT("REFERENCE_COAST invalid test/coefficient"));FGenericPlatformMisc::RequestExitWithStatus(false,1);return;}
        Settings.RollingResistance=ReferenceCrr;Settings.BearingCoulombNm=Settings.BearingViscousNms=0;
        Settings.Mecanum.RollerCoulombNm=Settings.Mecanum.RollerViscousNms=0;
        UE_LOG(LogTemp,Display,TEXT("REFERENCE_COAST crr=%.6f bearing=0 scope=material_proxy_not_mecanum_measurement"),ReferenceCrr);
    }
    if(Settings.bMecanum) Settings.MaxSpeedMps=FMath::Min(Settings.MaxSpeedMps,Settings.Mecanum.MaxTranslationMps);
    double Rate=Settings.Tyres.Tread.bEnabled && TestMode.IsEmpty()?(Settings.Tyres.Tread.Count>=32?.25:.5):1.;
    FParse::Value(FCommandLine::Get(),TEXT("PlaybackRate="),Rate);
    if(!FMath::IsFinite(Rate) || Rate<.1 || Rate>1)
    { Error=TEXT("PlaybackRate must be between 0.1 and 1.0"); UE_LOG(LogTemp,Error,TEXT("PLAYBACK_CONFIG %s"),*Error); return; }
    GetWorld()->GetWorldSettings()->SetTimeDilation(Rate);
    if(Settings.bMecanum)
    {
        // Limit the WORLD delta, before both physics and controllers consume it.
        // When rendering cannot keep up this is honest slow playback, never a
        // large controller step paired with a truncated physics step.
        GetWorld()->GetWorldSettings()->MaxUndilatedFrameTime=Settings.Mecanum.MaxControlStepS;
        BasePhysicsStepS=SelectedPhysicsStepS=SmallestPhysicsStepS=UPhysicsSettings::Get()->MaxSubstepDeltaTime;
        SmallestObservedPhysicsStepS=BasePhysicsStepS;
        UE_LOG(LogTemp,Display,TEXT("MECANUM_CLOCK max_world_step_s=%.9f slow_frames=bounded_simulation_time"),Settings.Mecanum.MaxControlStepS);
    }
    if(Settings.bSwerve)
    {
        // The light non-spinning carrier adds a stiff, high mass-ratio joint
        // chain. Increase numerical resolution, NOT mass conditioning or force.
        // Reduced iteration counts are retained ONLY for reproducing the
        // rejected numerical candidate. Production keeps the validated counts.
        const bool Balanced=Settings.Tyres.Tread.bEnabled && FParse::Param(FCommandLine::Get(),TEXT("ReducedTreadIterations"));
        int Pos=Settings.bMecanum?Settings.Mecanum.PositionIterations:Balanced?96:127,Vel=Settings.bMecanum?Settings.Mecanum.VelocityIterations:Balanced?16:32;
        if(Settings.bMecanum) {FParse::Value(FCommandLine::Get(),TEXT("MecanumPositionIterations="),Pos);FParse::Value(FCommandLine::Get(),TEXT("MecanumVelocityIterations="),Vel);}
        Pos=FMath::Clamp(Pos,8,127);Vel=FMath::Clamp(Vel,8,127);
        GetWorld()->GetPhysicsScene()->GetSolver()->SetPositionIterations(Pos);
        GetWorld()->GetPhysicsScene()->GetSolver()->SetVelocityIterations(Vel);
        UE_LOG(LogTemp,Display,TEXT("TREAD_SOLVER quality=%s position_iterations=%d velocity_iterations=%d"),Balanced?TEXT("rejected-candidate"):TEXT("production/reference"),Pos,Vel);
    }
    NormalMaxSpeed=Settings.MaxSpeedMps;
    WebTelemetry=FWebTelemetryPublisher::Create();
    ResetRig();
}
void AStabilityLab::ApplyMaterials()
{
    TyreMaterial=Vehicle3D::Material(this,Settings.TyreMu); RoadMaterial=Vehicle3D::Material(this,Settings.RoadMu);
    if(Settings.bSwerve)
    {
        // Merely waking once per game tick still allows an island to sleep
        // between inner substeps, dropping static support from instrumentation.
        TyreMaterial->SleepLinearVelocityThreshold=0;
        TyreMaterial->SleepAngularVelocityThreshold=0;
        TyreMaterial->SleepCounterThreshold=MAX_int32;
    }
    if (Road) Road->SetPhysMaterialOverride(RoadMaterial);
    if(Course) Course->SetMu(Settings.RoadMu);
    for (AWheelActor* W : Wheels) W->SetMu(TyreMaterial);
}
void AStabilityLab::ResetRig()
{
    JointSensors.Reset();
    DynamicsSensors.Reset();
    ProximitySensors.Reset();
    SensorLift.Reset();bSensorLiftRequested=false;
    MissionControl.Reset();MissionTestStarted=MissionTestInjected=MissionTestAck=MissionTestResumed=MissionTestPaused=false;MissionTestEventS=MissionTestPauseS=0;MissionTestMinClearance=100;
    HandlingControl.Reset();HandlingTestRequested=HandlingTestFaultInjected=false;
    PalletRecognition.Reset();PalletTestSawDetection=PalletTestSawPartial=PalletTestSawFull=PalletTestConfigured=false;PalletTestFrames=0;PalletTestMaxDistanceError=0;if(PalletVisionDistractor){PalletVisionDistractor->Destroy();PalletVisionDistractor=nullptr;}ClearNavigationWalls();DriveSensors.Reset();bSensorDriveRequested=false;VehicleGuard.Reset();bVehicleSafetyRequested=false;if(GuardObstacle){GuardObstacle->Destroy();GuardObstacle=nullptr;}
    if(WebTelemetry) WebTelemetry->NewRun();
    Flush();
    ReleaseLoadSampler();
    if(Pallet) { Pallet->Destroy(); Pallet=nullptr; }
    PayloadState=FPayloadState();
    CargoState=FCargoState();
    PowerState=FPowerState();
    TyreState=FTyreState();
    HydraulicLabState=FHydraulicLabState();
    EnergyAudit=FEnergyAudit();
    ValidationState=FValidationState();
    SetupTerrain();
    if(!SetupNavigationLandmarks())return;
    if (Fork) Fork->Destroy();
    for(AWheelActor* W:Wheels) W->Destroy();
    Wheels.Empty();
    if (Chassis) Chassis->Destroy();
    if(ReactionWheel) {ReactionWheel->Destroy();ReactionWheel=nullptr;}
    if(ReactionChassis) {ReactionChassis->Destroy();ReactionChassis=nullptr;}
    TimeS=NextRecord=0; NextFlush=5; MaxTilt=MaxLateralSlip=MaxYawRate=MaxSpeed=MaxForkHeight=MaxWheelSurfaceSpeed=0; MinGrounded=4;
    PreviousVelocity=FVector::ZeroVector;
    PreviousPhysicsDt=0;
    bMecanumAutoHold=false;bNeutral=false; bResistanceEnabled=true; bReactionIsolated=false;ReactionPtError=0;bReactionCoastSample=false;ReactionCoastStartJ=ReactionCoastLastJ=ReactionCoastPeakJ=ReactionPredictorWorkJ=0;ClockWallStartS=0;MecanumVelocityIntegral=FVector2D::ZeroVector;ReactionError=ReactionImpulse=ReactionPitch=0;ReactionInitialL=FVector::ZeroVector;
    LoadSumIntegral=FrontLoadIntegral=LoadSampleTime=TotalNormalLoadN=CoastStartSpeed=0;
    FrictionNotice.Empty(); FrictionNoticeUntil=0;
    bFrictionSwitched=bFrictionRestored=bFrictionKeysPassed=false; MaxLiveSlip=0;
    MinPoweredTurnSpeed=TNumericLimits<double>::Max();
    Throttle=Brake=Steering=Lift=SteerDeg=LiftTargetM=0; bSeeded=bFinished=bCaptured=false;
    Strafe=TargetYawRadS=TargetHeadingDeg=MaxSideSpeed=MaxModuleError=0; TargetVelocity=FVector2D::ZeroVector;
    bAligning=bLateralReorienting=false; AlignmentErrorDeg=0;
    WheelSpeedScale=1; AllocatedYawRadS=CoordinatedYawRadS=0;
    const double Z=Settings.RadiusM*100+4+(IsTyreTest() && TestMode.Contains(TEXT("drop"))?2:0);
    // Deck underside sits 8 cm above the wheel top at any steering angle.
    Chassis=GetWorld()->SpawnActor<AChassisActor>(FVector(0,0,Z+(Settings.bSwerve?Settings.RadiusM*100+19:36)),FRotator::ZeroRotator);
    Chassis->Configure(Settings);
    ApplyMaterials();
    for(int I=0;I<4;++I)
    {
        const FVector P((I<2?1:-1)*Settings.WheelbaseM*50,(I%2==0?-1:1)*Settings.TrackM*50,Z);
        auto* W=GetWorld()->SpawnActor<AWheelActor>(P,FRotator::ZeroRotator);
        W->Configure(Chassis->Body,Settings,I>=2,TyreMaterial,I); Wheels.Add(W);
    }
    const bool ForkBench=IsComponentTest() && (TestMode.EndsWith(TEXT("plastic")) || TestMode.EndsWith(TEXT("fracture")));
    const double ForkZ=ForkBench?224.5:Settings.bReferenceGeometry?22.5:bPayload && Settings.Structure.bEnabled?24.5:Z+(bPayload?-9.5:16);
    const double ForkX=Settings.bReferenceGeometry?(Settings.ForkFaceXM()-.03)*100:Settings.bSwerve?(Settings.MastXM()+.28)*100:118;
    Fork=GetWorld()->SpawnActor<AForkActor>(FVector(ForkX,0,ForkZ),FRotator::ZeroRotator);
    Fork->Configure(Chassis->Body,Settings,bPayload?&PayloadSettings:nullptr);
    if(bPayload) SpawnPayload();
    SetupHandlingRack();
    ForkliftModel::Present(this);
    ConfigureCameraViews();
    if(bCameraViews)bHydraulicView=false;
    if(Settings.bMecanum)
    {
        SetMecanumIterations(Chassis);SetMecanumIterations(Fork);
        for(AWheelActor* W:Wheels) SetMecanumIterations(W);
        for(AForkTineActor* T:Fork->Tines) SetMecanumIterations(T);
        if(Pallet) {SetMecanumIterations(Pallet);SetMecanumIterations(Pallet->Cargo);}
    }
    if(Settings.bSwerve)
    {
        // Explicit losses replace arbitrary body damping for the new rig.
        // Legacy rigs retain their historical damping and test behaviour.
        auto ClearDamping=[this](UStaticMeshComponent* Body) { Body->SetLinearDamping(0); Body->SetAngularDamping(0); if(Settings.bMecanum) {Body->BodyInstance.SetGyroscopicTorqueEnabled(!FParse::Param(FCommandLine::Get(),TEXT("MecanumNoGyro")));Body->BodyInstance.SetMaxAngularVelocityInRadians(2000,false);} };
        ClearDamping(Chassis->Body); ClearDamping(Fork->Carriage);
        for(AWheelActor* W:Wheels) for(UStaticMeshComponent* B:W->PhysicalBodies()) ClearDamping(B);
        LoadSampler=GetWorld()->GetPhysicsScene()->GetSolver()->CreateAndRegisterSimCallbackObject_External<FContactLoadSampler>();
    }
    ForkBaseZ=Fork->GetActorLocation().Z-Chassis->GetActorLocation().Z;
    if(Settings.bMecanum)
    {
        FString JointError;JointSensors=FJointSensorRig::Create(Settings.ForkTravelM,Settings.RadiusM,JointError);
        if(!JointSensors){UE_LOG(LogTemp,Warning,TEXT("JOINT_SENSOR_CONFIG %s"),*JointError);if(!TestMode.IsEmpty())FGenericPlatformMisc::RequestExitWithStatus(false,1);}
        DynamicsSensors=FDynamicsSensorRig::Create(JointError);
        if(!DynamicsSensors){UE_LOG(LogTemp,Warning,TEXT("DYNAMICS_SENSOR_CONFIG %s"),*JointError);if(!TestMode.IsEmpty())FGenericPlatformMisc::RequestExitWithStatus(false,1);}
        Fork->bPressureSensor=DynamicsSensors.IsValid();Fork->PressureSamplePeriodS=DynamicsSensors?1./DynamicsSensors->Model.Spec.Hz:.01;
        ProximitySensors=FProximitySensorRig::Create(JointError);
        if(!ProximitySensors){UE_LOG(LogTemp,Warning,TEXT("PROXIMITY_SENSOR_CONFIG %s"),*JointError);if(!TestMode.IsEmpty())FGenericPlatformMisc::RequestExitWithStatus(false,1);}
        bSensorLiftRequested=Settings.Hydraulics.bEnabled&&(TestMode.IsEmpty()||IsSensorLiftTest()||IsVehicleGuardTest()||IsSensorDriveTest()||IsPalletPerceptionTest()||IsHandlingTest()||IsMissionTest())&&!FParse::Param(FCommandLine::Get(),TEXT("LegacyLiftControl"));
        if(bSensorLiftRequested)
        {
            FHydraulicCommand Stop;Stop.Direct=true;Stop.PumpEnabled=false;Fork->SetValveCommand(Stop,.1);
            const auto& Hyd=Settings.Hydraulics;
            const double ReturnGain=(Hyd.bChainMast?Hyd.ChainRatio:1)*Hyd.ReturnM3sSqrtPa/Hyd.AreaM2;
            SensorLift=FSensorLiftRig::Create(Settings.ForkTravelM,ReturnGain,JointError);
            if(!SensorLift){UE_LOG(LogTemp,Warning,TEXT("SENSOR_LIFT_CONFIG %s"),*JointError);if(!TestMode.IsEmpty())FGenericPlatformMisc::RequestExitWithStatus(false,1);}
            if(IsSensorLiftTest()&&JointSensors&&TestMode.Contains(TEXT("fault")))
            {JointSensors->Model.TimedFaultMask=1u<<4;JointSensors->Model.FaultStartS=TestMode.EndsWith(TEXT("down"))?22:6;JointSensors->Model.FaultEndS=JointSensors->Model.FaultStartS+2;}
        }
        bSensorDriveRequested=(TestMode.IsEmpty()||IsSensorDriveTest()||IsPalletPerceptionTest()||IsHandlingTest()||IsMissionTest())&&!FParse::Param(FCommandLine::Get(),TEXT("LegacyDriveControl"));
        if(bSensorDriveRequested){DriveSensors=FSensorDriveRig::Create(JointError);if(!DriveSensors){UE_LOG(LogTemp,Error,TEXT("SENSOR_DRIVE_CONFIG %s"),*JointError);if(!TestMode.IsEmpty())FGenericPlatformMisc::RequestExitWithStatus(false,1);}}
        if(TestMode.IsEmpty()||IsPalletPerceptionTest()||IsHandlingTest()||IsMissionTest()){PalletRecognition=FPalletPerceptionRig::Create(JointError);if(!PalletRecognition){UE_LOG(LogTemp,Error,TEXT("PALLET_PERCEPTION_CONFIG %s"),*JointError);if(!TestMode.IsEmpty())FGenericPlatformMisc::RequestExitWithStatus(false,1);}}
        bVehicleSafetyRequested=TestMode.IsEmpty()||IsVehicleGuardTest()||IsSensorDriveTest()||IsPalletPerceptionTest()||IsHandlingTest()||IsMissionTest();
        if(bVehicleSafetyRequested){VehicleGuard=FLidarSafetyRig::Create(JointError);if(!VehicleGuard){UE_LOG(LogTemp,Warning,TEXT("LIDAR_SAFETY_CONFIG %s"),*JointError);if(!TestMode.IsEmpty())FGenericPlatformMisc::RequestExitWithStatus(false,1);}}
    }
    if(TestMode.IsEmpty()||IsHandlingTest()||IsMissionTest()){FString E;HandlingControl=FHandlingRig::Create(E);if(!HandlingControl){Error=E;UE_LOG(LogTemp,Error,TEXT("HANDLING_CONFIG %s"),*E);}}
    if(TestMode.IsEmpty()||IsMissionTest()){FString E;MissionControl=FMissionRig::Create(E);if(!MissionControl){Error=E;UE_LOG(LogTemp,Error,TEXT("MISSION_CONFIG %s"),*E);}}
    MassKg=Chassis->Body->GetMass()+Fork->Carriage->GetMass();
    for(AForkTineActor* T:Fork->Tines) for(UStaticMeshComponent* B:T->Sections) MassKg+=B->GetMass();
    if(Fork->MovingMast) MassKg+=Fork->MovingMast->GetMass();
    for(AWheelActor* W:Wheels) for(UStaticMeshComponent* B:W->PhysicalBodies()) MassKg+=B->GetMass();
    ForkHeightM=0;
    TyreState.EnergyReferenceJ=TyreMechanicalEnergyJ();
    Rows={TEXT("time_s,tyre_mu,road_mu,effective_mu,chassis_com_offset_m,speed_mps,lateral_mps,yaw_deg_s,roll_deg,pitch_deg,tilt_deg,near_floor_wheels,steer_command_deg,fork_target_m,fork_actual_m,FL_lateral_slip_mps,FR_lateral_slip_mps,RL_lateral_slip_mps,RR_lateral_slip_mps,FL_clearance_m,FR_clearance_m,RL_clearance_m,RR_clearance_m")};
    const FString Dir=FPaths::ProjectSavedDir()/TEXT("Telemetry"); IFileManager::Get().MakeDirectory(*Dir,true);
    Rows[0]+=TEXT(",acceleration_mps2,world_vx_mps,world_vy_mps,chassis_com_height_m");
    for(const TCHAR* Name:{TEXT("FL"),TEXT("FR"),TEXT("RL"),TEXT("RR")})
        Rows[0]+=FString::Printf(TEXT(",%s_omega_rad_s,%s_long_slip_mps,%s_drive_torque_nm,%s_brake_limit_nm,%s_steer_actual_deg"),Name,Name,Name,Name,Name);
    Rows[0]+=TEXT(",throttle,brake,RL_twist_deg,RR_twist_deg,RL_axle_alignment,RR_axle_alignment,RL_joint_error_m,RR_joint_error_m,FL_awake,RL_awake,RL_chassis_contacts,RR_chassis_contacts");
    Rows[0]+=TEXT(",strafe,target_vx_mps,target_vy_mps,target_yaw_rad_s,chassis_x_m,chassis_y_m");
    for(const TCHAR* Name:{TEXT("FL"),TEXT("FR"),TEXT("RL"),TEXT("RR")})
        Rows[0]+=FString::Printf(TEXT(",%s_target_angle_deg,%s_target_speed_mps,%s_self_contacts"),Name,Name,Name);
    for(const TCHAR* Name:{TEXT("FL"),TEXT("FR"),TEXT("RL"),TEXT("RR")})
        Rows[0]+=FString::Printf(TEXT(",%s_normal_n,%s_normal_filtered_n,%s_floor_events,%s_rolling_nm,%s_bearing_nm,%s_loss_w"),Name,Name,Name,Name,Name,Name);
    Rows[0]+=TEXT(",normal_sum_n,neutral,resistance_enabled,crr,bearing_coulomb_nm,bearing_viscous_nms,load_interval_s");
    Rows[0]+=TEXT(",wheel_surface_limit_mps,wheel_speed_scale,allocated_yaw_rad_s");
    if(bPayload) Rows[0]+=PayloadCsv(true);
    if(bPayload) Rows[0]+=CargoCsv(true);
    if(Settings.bSwerve) Rows[0]+=PowerCsv(true);
    if(Settings.bSwerve) Rows[0]+=TyreCsv(true);
    if(Settings.bSwerve) Rows[0]+=HydraulicCsv(true);
    if(Settings.bSwerve) Rows[0]+=EnergyCsv(true);
    if(Settings.bSwerve) Rows[0]+=ValidationCsv(true);
    Rows[0]+=MecanumCsv(true);
    Rows[0]+=TEXT(",rack_support_n,rack_trial_stage,pallet_physics_mass_kg,cargo_physics_mass_kg");
    CsvPath=Dir/FString::Printf(TEXT("Stability-%s-%s-%s.csv"),TestMode.IsEmpty()?TEXT("manual"):*TestMode,*FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")),*FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(8));
    UE_LOG(LogTemp,Display,TEXT("STABILITY_RIG mass=%.2f chassis=%.2f fork=%.2f com_z=%.3f mu=%.2f swerve=%d"),MassKg,Chassis->Body->GetMass(),Fork->Carriage->GetMass(),Chassis->Body->GetCenterOfMass().Z/100,Settings.EffectiveMu(),Settings.bSwerve?1:0);
}
void AStabilityLab::DriveSwerve(double Dt)
{
    if(bNeutral || Brake>0)
    {
        // Brake overrides W/S, strafe and yaw before their module targets are
        // allocated. Keep steering directions fixed while braking on a slope.
        // Neutral also disables propulsion; passive wheel losses remain active.
        TargetVelocity=FVector2D::ZeroVector; TargetYawRadS=0;
        WheelSpeedScale=1; AllocatedYawRadS=CoordinatedYawRadS=0;
        TargetHeadingDeg=Chassis->GetActorRotation().Yaw; bAligning=bLateralReorienting=false;
        for(AWheelActor* W:Wheels) { W->SpeedTargetMps=0; W->Actuate(Settings,Dt,0,Brake*Settings.BrakeNm,W->SteerTargetDeg,false); }
        return;
    }
    FVector2D Desired(Throttle,Strafe);
    if(Desired.Size()>1) Desired.Normalize();
    Desired*=Settings.MaxSpeedMps;
    const bool Stopping=Desired.IsNearlyZero() && FMath::Abs(Steering)<.01;
    const double PreviousTargetSpeed=TargetVelocity.Size();
    const FVector2D Delta=Desired-TargetVelocity;
    const double Step=Settings.AccelerationLimitMps2*Dt;
    TargetVelocity+=Delta.GetSafeNormal()*FMath::Min(Delta.Size(),Step);
    TargetYawRadS=Stopping?Swerve::DecelerationYaw(TargetYawRadS,PreviousTargetSpeed,TargetVelocity.Size()):
        FMath::FInterpConstantTo(TargetYawRadS,FMath::DegreesToRadians(Steering*Settings.YawRateDegS),Dt,FMath::DegreesToRadians(90.));
    // Hold heading only after a turn has ended. Integrating an unreachable yaw
    // target while steering is rate-limited builds a catch-up turn at reversal.
    if(FMath::Abs(TargetYawRadS)>.001) TargetHeadingDeg=Chassis->GetActorRotation().Yaw;
    // Chassis yaw feedback is allocated to wheel targets, never a body torque
    // or transform override. It counters unequal contact loads and steering reaction.
    const double HeadingError=FMath::FindDeltaAngleDegrees(Chassis->GetActorRotation().Yaw,TargetHeadingDeg);
    double ControlledYaw=TargetYawRadS+FMath::Clamp(FMath::DegreesToRadians(.8*HeadingError)+.5*(TargetYawRadS-FMath::DegreesToRadians(YawDegS)),-.5,.5);
    if(FMath::Abs(TargetYawRadS)<1e-5 && FMath::Abs(HeadingError)<.1 && FMath::Abs(YawDegS)<.1) ControlledYaw=0;
    FVector2D Positions[4]; double PreviousAngles[4],ActualAngles[4];
    for(int I=0;I<4;++I)
    { Positions[I]=FVector2D((I<2?1:-1)*Settings.WheelbaseM*.5,(I%2==0?-1:1)*Settings.TrackM*.5); PreviousAngles[I]=Wheels[I]->SteerTargetDeg; ActualAngles[I]=Wheels[I]->SteerActualDeg; }
    // Keep module directions fixed during a full input release. The motors still
    // brake through the existing torque limits; this adds no force or transform.
    if(Stopping)
    {
        CoordinatedYawRadS=Swerve::DecelerationYaw(CoordinatedYawRadS,PreviousTargetSpeed,TargetVelocity.Size());
        TargetHeadingDeg=Chassis->GetActorRotation().Yaw;
    }
    else if(TargetVelocity.Size()<.01 && FMath::Abs(TargetYawRadS)<.001) CoordinatedYawRadS=0;
    else if(TargetVelocity.Size()<.01) CoordinatedYawRadS=ControlledYaw; // pure spin has a fixed tangent direction, independent of yaw magnitude
    // Heavy front tyres can steer slower than the nominal command rate. Limit
    // COMMON yaw look-ahead to 5 deg of measured module lag, not all drive speed.
    else CoordinatedYawRadS=Swerve::LimitYawForSteering(Positions,PreviousAngles,4,TargetVelocity,CoordinatedYawRadS,ControlledYaw,Settings.SteerRateDegS*Dt,ActualAngles,5);
    Swerve::FTarget Targets[4]; double WorstAlignment=0;
    for(int I=0;I<4;++I)
    {
        const FVector2D P((I<2?1:-1)*Settings.WheelbaseM*.5,(I%2==0?-1:1)*Settings.TrackM*.5);
        // Optimize against the previous command to avoid random +/-90 choices
        // from tiny contact jitter when beginning a sideways move.
        Targets[I]=Swerve::Solve(P,TargetVelocity,CoordinatedYawRadS,Wheels[I]->SteerTargetDeg);
        WorstAlignment=FMath::Max(WorstAlignment,FMath::Abs(FMath::FindDeltaAngleDegrees(Wheels[I]->SteerActualDeg,Targets[I].AngleDeg)));
    }
    // Do not accumulate an unreachable heading while the modules are still
    // aligning. Otherwise the first motion tries to catch up with that backlog.
    const double ModuleLimit=Settings.WheelSurfaceSpeedLimitMps;
    // Outer wheels on a curve must roll faster than the chassis cruise speed.
    WheelSpeedScale=Swerve::DesaturationScale(Targets,4,ModuleLimit);
    AllocatedYawRadS=CoordinatedYawRadS*WheelSpeedScale;
    AlignmentErrorDeg=WorstAlignment; bAligning=WorstAlignment>10;
    // A pure sideways request needs its 90-degree module reorientation before
    // rolling, otherwise the alignment transient first drives diagonally.
    // This latch is NOT used for forward + yaw, and does not chatter at 10 deg.
    const bool PureLateral=FMath::Abs(Throttle)<.01 && FMath::Abs(Strafe)>.01 && FMath::Abs(Steering)<.01;
    if(!PureLateral || WorstAlignment<8) bLateralReorienting=false;
    else if(WorstAlignment>60) bLateralReorienting=true;
    if(bAligning || TiltDeg>70) TargetHeadingDeg=Chassis->GetActorRotation().Yaw;
    for(int I=0;I<4;++I)
    {
        AWheelActor* W=Wheels[I]; const auto& T=Targets[I];
        W->SteerTargetDeg=Swerve::Slew(W->SteerTargetDeg,T.AngleDeg,Settings.SteerRateDegS*Dt);
        const double ErrorDeg=FMath::FindDeltaAngleDegrees(W->SteerActualDeg,T.AngleDeg);
        // Reduce rolling demand while a module is pointing across its desired path.
        W->SpeedTargetMps=T.SpeedMps*WheelSpeedScale*FMath::Max(0.,FMath::Cos(FMath::DegreesToRadians(ErrorDeg)));
        // Each wheel follows the component along its current rolling direction.
        // A global alignment gate brakes ALL wheels whenever one module lags,
        // causing a loaded W+D turn to stall and restart the heading controller.
        if(bLateralReorienting) W->SpeedTargetMps=0;
        double Torque=FMath::Clamp((W->SpeedTargetMps-W->OmegaRadS*Settings.RadiusM)*Settings.WheelSpeedGain,-Settings.DriveNm,Settings.DriveNm);
        if(Brake>0 || TiltDeg>70) Torque=0;
        W->Actuate(Settings,Dt,Torque,Brake*Settings.BrakeNm,W->SteerTargetDeg,Brake<=0 && TiltDeg<=70);
        // Above-deck indicators follow measured module orientation, not its command.
        const FVector Marker=W->Knuckle->GetComponentLocation()+Chassis->Body->GetUpVector()*(Settings.RadiusM*100+45);
        DrawDebugDirectionalArrow(GetWorld(),Marker,Marker+W->Knuckle->GetForwardVector()*35,10,FColor::Cyan,false,0,0,2);
        if(TimeS>10) MaxModuleError=FMath::Max(MaxModuleError,FMath::Abs(FMath::FindDeltaAngleDegrees(W->SteerActualDeg,W->SteerTargetDeg)));
    }
    if(Brake>0) { TargetVelocity=FVector2D::ZeroVector; TargetYawRadS=0; TargetHeadingDeg=Chassis->GetActorRotation().Yaw; }
}
void AStabilityLab::ChangeFriction(bool Tyre,double Delta)
{
    double& Mu=Tyre?Settings.TyreMu:Settings.RoadMu;
    const double Old=Mu; Mu=FMath::Clamp(Mu+Delta,0.,2.); ApplyMaterials();
    FrictionNotice=FString::Printf(TEXT("%s mu %.2f -> %.2f | contact %.2f"),Tyre?TEXT("Tyre"):TEXT("Road"),Old,Mu,Settings.EffectiveMu());
    FrictionNoticeUntil=TimeS+4;
    UE_LOG(LogTemp,Display,TEXT("FRICTION_CHANGE %s tyre=%.3f road=%.3f contact=%.3f"),Tyre?TEXT("tyre"):TEXT("road"),Settings.TyreMu,Settings.RoadMu,Settings.EffectiveMu());
}
void AStabilityLab::ToggleRoadGrip() { ChangeFriction(false,(Settings.RoadMu>.2?.08:.8)-Settings.RoadMu); }
bool AStabilityLab::FrictionMaterialsMatch() const
{
    if(!Road || Wheels.Num()!=4) return false;
    auto Matches=[](UStaticMeshComponent* Body,double Mu)
    {
        const UPhysicalMaterial* M=Body->BodyInstance.GetSimplePhysicalMaterial();
        return M && FMath::IsNearlyEqual(double(M->Friction),Mu,1e-5)
            && M->bOverrideFrictionCombineMode && M->FrictionCombineMode==EFrictionCombineMode::Min;
    };
    if(!Matches(Road,Settings.RoadMu)) return false;
    for(AWheelActor* W:Wheels) { if(!Matches(W->Tyre,Settings.TyreMu)) return false; for(UWheelTyreComponent* P:W->TreadPads) if(!Matches(P,Settings.TyreMu)) return false;for(UWheelTyreComponent* P:W->Rollers) if(!Matches(P,Settings.TyreMu)) return false; }
    return true;
}
void AStabilityLab::CycleStress()
{
    if(bPayload) { FrictionNotice=TEXT("Payload lab: high-CoM stress disabled; use separate lab"); FrictionNoticeUntil=TimeS+4; return; }
    bStress=!bStress;
    Settings.MaxSpeedMps=bStress?8.:NormalMaxSpeed; Settings.ChassisComOffsetM=bStress?1.2:NormalComOffset;
    Settings.WheelSurfaceSpeedLimitMps=bStress?8.:NormalWheelSurfaceLimit;
    Settings.SteerRateDegS=bStress?120.:NormalSteerRate; Settings.MaxSteerDeg=bStress?35.:NormalMaxSteer;
    ResetRig();
}
void AStabilityLab::SeedSpeed(double Speed)
{
    // A reproducible initial condition, applied ONCE after settling; never steering by teleportation.
    const FVector V(Speed*100,0,0);
    Chassis->Body->SetPhysicsLinearVelocity(V); Fork->Carriage->SetPhysicsLinearVelocity(V);
    for(AWheelActor* W:Wheels)
    {
        W->Knuckle->SetPhysicsLinearVelocity(V); W->Tyre->SetPhysicsLinearVelocity(V); W->BeltCarrier->SetPhysicsLinearVelocity(V);
        W->Tyre->SetPhysicsAngularVelocityInRadians(W->Knuckle->GetRightVector()*Speed/Settings.RadiusM);
        const FVector Angular=W->Knuckle->GetRightVector()*Speed/Settings.RadiusM;
        for(UWheelTyreComponent* P:W->TreadPads) { P->SetPhysicsAngularVelocityInRadians(Angular); P->SetPhysicsLinearVelocity(V+FVector::CrossProduct(Angular,P->GetCenterOfMass()-W->Tyre->GetCenterOfMass())); }
    }
}
void AStabilityLab::Tick(float Dt)
{
    Super::Tick(Dt); if (!Chassis || !Error.IsEmpty() || bFinished) return;
    if(Settings.bMecanum) SelectMecanumStep();
    const double FrameNow=FPlatformTime::Seconds();
    if(LastFrameWallS>0)
    {
        const double WallDt=FrameNow-LastFrameWallS;
        if(WallDt>0 && WallDt<1){FrameWallSmoothedS=FrameWallSmoothedS>0?FMath::Lerp(FrameWallSmoothedS,WallDt,1-FMath::Exp(-WallDt/.5)):WallDt;if((bCapture||IsMissionTest()) && TimeS>1)PerformanceFrameMs.Add(WallDt*1000);}
    }
    LastFrameWallS=FrameNow;
    TimeS+=Dt;
    if(ClockWallStartS==0) {ClockWallStartS=FPlatformTime::Seconds();ClockSimStartS=TimeS;}
    const double WallElapsed=FPlatformTime::Seconds()-ClockWallStartS;
    if(WallElapsed>.1) RealtimeRatio=(TimeS-ClockSimStartS)/WallElapsed;
    ValidationState.MaxGameStepS=FMath::Max(ValidationState.MaxGameStepS,double(Dt));
    const FVector V=Chassis->Body->GetPhysicsLinearVelocity()/100;
    AccelerationMps2=Dt>0?(V-PreviousVelocity).Size()/Dt:0; PreviousVelocity=V;
    SpeedMps=FVector::DotProduct(V,Chassis->Body->GetForwardVector());
    LateralMps=FVector::DotProduct(V,Chassis->Body->GetRightVector());
    YawDegS=FMath::RadiansToDegrees(Chassis->Body->GetPhysicsAngularVelocityInRadians().Z);
    const FRotator Rot=Chassis->Body->GetComponentRotation(); RollDeg=Rot.Roll; PitchDeg=Rot.Pitch;
    TiltDeg=FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Chassis->Body->GetUpVector().Z,-1.,1.)));
    ForkHeightM=(Chassis->Body->GetComponentQuat().UnrotateVector(Fork->GetActorLocation()-Chassis->GetActorLocation()).Z-ForkBaseZ)/100;
    double Impulses[4]={0,0,0,0}, MuImpulses[4]={0,0,0,0}; FVector Moments[4]={}, SteerImpulses[4]={}, AxleImpulses[4]={}, RoadFrictionImpulses[4]={}; int32 ContactCounts[4]={0,0,0,0}; PreviousPhysicsDt=0;
    RackSupportN=0;
    if(bPayload) { PayloadState.GroundN=0; for(int I=0;I<2;++I) { PayloadState.VerticalN[I]=PayloadState.NormalN[I]=PayloadState.FrictionN[I]=0; PayloadState.PointWeighted[I]=FVector::ZeroVector; } }
    if(bPayload) CargoState.Frame=FCargoContactData();
    for(AWheelActor* W:Wheels) { W->SolverSlipAlongSum=W->SolverSlipAcrossSum=W->SolverSlipWeight=W->FrictionUsedNs=0; W->ContactInnerPeakN=0; W->ContactData=FWheelContactData(); for(double& N:W->TreadNormalN) N=0; for(FVector& P:W->TreadContactPointM) P=FVector::ZeroVector; W->RollerBearingPowerW=0;for(double& T:W->RollerBearingNm) T=0;for(double& N:W->RollerNormalN) N=0;for(FVector& P:W->RollerContactPointM) P=FVector::ZeroVector; }
    if(LoadSampler)
    {
        while(auto Output=LoadSampler->PopOutputData_External())
        {
            if(JointSensors)for(const auto& Sample:Output->JointSamples)JointSensors->Model.Capture(Sample);
            if(DynamicsSensors)for(const auto& Sample:Output->DynamicsSamples)DynamicsSensors->Model.Capture(Sample);
            PreviousPhysicsDt+=Output->IntervalS;
            if(Output->InnerSteps>0) SmallestObservedPhysicsStepS=FMath::Min(SmallestObservedPhysicsStepS,Output->IntervalS/Output->InnerSteps);
            MaxInnerClockErrorS=FMath::Max(MaxInnerClockErrorS,FMath::Abs(Output->IntervalS-Output->CompletedInnerS));
            if(DriveSensors)DriveSensors->ActuatorExpired|=Output->bDriveCommandExpired;
            if(Output->bMecanumMotorValid) for(int I=0;I<4;++I)
            {
                Wheels[I]->DriveMotor=Output->MecanumMotor[I];Wheels[I]->DriveTorqueNm=Output->MecanumMotor[I].OutputNm;
                Wheels[I]->DriveRequestNm=Output->MecanumRequestNm[I];Wheels[I]->RollingTorqueNm=Output->MecanumRollingNm[I];
            }
            if(Output->bBenchValid)
            {
                ReactionPtError=FMath::Max(ReactionPtError,Output->BenchL.Size());
                ReactionPredictorWorkJ+=Output->BenchPredictorWorkJ;
                if(TimeS>3.05) {if(!bReactionCoastSample) {bReactionCoastSample=true;ReactionCoastStartJ=Output->BenchEnergyJ;}ReactionCoastLastJ=Output->BenchEnergyJ;ReactionCoastPeakJ=FMath::Max(ReactionCoastPeakJ,Output->BenchEnergyJ-ReactionCoastStartJ);}
            }
            for(int I=0;I<4;++I)
            {
                Wheels[I]->SolverSlipAlongSum+=Output->SlipAlongM2Ns[I];Wheels[I]->SolverSlipAcrossSum+=Output->SlipAcrossM2Ns[I];
                Wheels[I]->SolverSlipWeight+=Output->SlipWeightNs[I];Wheels[I]->FrictionUsedNs+=Output->FrictionUsedNs[I];
            }
            for(int I=0;I<4;++I) Wheels[I]->ContactInnerPeakN=FMath::Max(Wheels[I]->ContactInnerPeakN,Output->PeakNormalN[I]);
            ValidationState.PhysicsSampleS+=Output->IntervalS;
            ValidationState.MaxInnerStepS=FMath::Max(ValidationState.MaxInnerStepS,Output->MaxInnerDt);
            ValidationState.RefinedContactCount+=Output->RefinedContacts;
            ValidationState.RefinedEdgeCount+=Output->RefinedEdgeContacts;
            for(int I=0;I<4;++I) {Wheels[I]->ContactData.Add(Output->WheelContact[I]);Wheels[I]->ReducedContactLossJ+=Output->ReducedLossJ[I];Wheels[I]->ReducedContactElasticJ=Output->ReducedElasticJ[I];}
            for(int I=0;I<4;++I) for(int J=0;J<Wheels[I]->TreadNormalN.Num();++J) {Wheels[I]->TreadNormalN[J]+=Output->TreadNormalNs[I][J];Wheels[I]->TreadContactPointM[J]+=Output->TreadPointMNs[I][J];}
            for(int I=0;I<4;++I) for(int J=0;J<Wheels[I]->RollerNormalN.Num();++J) {Wheels[I]->RollerNormalN[J]+=Output->TreadNormalNs[I][J];Wheels[I]->RollerContactPointM[J]+=Output->TreadPointMNs[I][J];}
            for(int I=0;I<4;++I) {EnergyAudit.RollerBearingJ+=Output->RollerBearingWorkJ[I];Wheels[I]->RollerBearingPowerW-=Output->RollerBearingWorkJ[I];for(int J=0;J<Wheels[I]->RollerBearingNm.Num();++J) Wheels[I]->RollerBearingNm[J]+=Output->RollerBearingImpulseNms[I][J];}
            if(IsIntegrationTest() && TimeS>10) for(int I=0;I<4;++I)
            {
                ValidationState.PeakInnerLoadN=FMath::Max(ValidationState.PeakInnerLoadN,Output->PeakNormalN[I]);
                ValidationState.MinInnerWheelN=FMath::Min(ValidationState.MinInnerWheelN,Output->MinNormalN[I]);
            }
            if(bPayload) AccumulatePayload(*Output);
            RackSupportN+=Output->PalletRackNs;
            EnergyAudit.ContactNormalJ+=Output->ContactNormalWorkJ;EnergyAudit.ContactTangentJ+=Output->ContactTangentWorkJ;EnergyAudit.CorrectionPseudoJ+=Output->ContactCorrectionPseudoJ;
            EnergyAudit.ForkDampingEstimateJ+=Output->ForkDampingEstimateJ;EnergyAudit.ContactWorkPoints+=Output->ContactWorkPoints;
            EnergyAudit.PalletPhysicsMassKg=Output->PalletPhysicsMassKg;EnergyAudit.CargoPhysicsMassKg=Output->CargoPhysicsMassKg;
            if(bPayload) CargoState.Frame.Add(Output->Cargo);
            TyreState.DampingJ+=Output->TyreDissipatedJ;
            EnergyAudit.BearingTransportJ+=Output->BearingPredictorWorkJ;
            EnergyAudit.ShaftJointJ+=Output->ShaftJointWorkJ;EnergyAudit.DriveJ+=Output->DriveWorkJ; EnergyAudit.RollingJ+=Output->RollingWorkJ; EnergyAudit.SteeringJ+=Output->SteeringJointWorkJ;
            TyreState.PeakCompressionM=FMath::Max(TyreState.PeakCompressionM,Output->TyrePeakCompressionM);
            for(int I=0;I<4;++I) RoadFrictionImpulses[I]+=Output->WheelFrictionImpulseNs[I];
            for(int32 I=0;I<4;++I) { Impulses[I]+=Output->ImpulseNs[I]; ContactCounts[I]+=Output->Contacts[I]; MuImpulses[I]+=Output->WheelMuNs[I]; Moments[I]+=Output->WheelMomentNs[I]; SteerImpulses[I]+=Output->SteerImpulseNms[I]; AxleImpulses[I]+=Output->AxleImpulseNs[I]; }
        }
        auto* Input=LoadSampler->GetProducerInputData_External();
        if(PreviousPhysicsDt>0)RackSupportN/=PreviousPhysicsDt;
        for(int I=0;I<2;++I)Input->RackIds[I]=HandlingRack && HandlingRack->Supports.IsValidIndex(I)?HandlingRack->Supports[I]->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx:INDEX_NONE;
        // Split lanes replace the old floor; NoCollision releases its physics
        // actor. Publish an active lane as primary instead of dereferencing it.
        UStaticMeshComponent* PrimaryRoad=TerrainPreset==TEXT("split") && Course && Course->Surfaces.Num()>0?Course->Surfaces[0].Get():Road.Get();
        Input->RoadId=PrimaryRoad->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
        Input->ChassisId=Chassis->Body->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
        Input->TyreDampingNsPm=Settings.Tyres.bEnabled && !Settings.Tyres.Tread.bEnabled?Settings.Tyres.DampingNsPm:0;
        Input->TreadPadC=Settings.Tyres.Tread.PadC(Settings.RadiusM,Settings.WidthM);
        Input->TreadPhaseRad=Settings.Tyres.Tread.PhaseRad;
        Input->bTreadCoupled=Settings.Tyres.Tread.bCoupled;
        Input->bMecanum=Settings.bMecanum;
        Input->bJointSensors=JointSensors.IsValid();Input->JointSamplePeriodS=JointSensors?1./JointSensors->Model.Spec.MaxHz():.01;Input->JointForkBaseM=ForkBaseZ/100.;
        Input->bDynamicsSensors=DynamicsSensors.IsValid();Input->DynamicsPeriodS=DynamicsSensors?1./DynamicsSensors->Model.Spec.Hz:.01;
        Input->ImuOffsetM=DynamicsSensors?DynamicsSensors->Model.Spec.MountFromComM:FVector::ZeroVector;
        Input->SensorGravityMps2=FVector(0,0,GetWorld()->GetGravityZ()/100.);
        Input->bReducedMecanum=Settings.Mecanum.bReduced;
        Input->ReducedRollerCoulombNm=Settings.Mecanum.RollerCoulombNm;Input->ReducedRollerViscousNms=Settings.Mecanum.RollerViscousNms;Input->ReducedRollerRadiusM=Settings.Mecanum.RollerRadiusM;
        Input->ReducedTyreMu=Settings.TyreMu;Input->ReducedRoadMu=Settings.RoadMu;
        const bool BearingIntegration=Settings.bMecanum && Settings.Mecanum.bBearingIntegrator && !FParse::Param(FCommandLine::Get(),TEXT("MecanumLegacyIntegration"));
        Input->bBearingPredictor=BearingIntegration || FParse::Param(FCommandLine::Get(),TEXT("MecanumBearingPredictor"));
        Input->bExactRollerSpin=BearingIntegration || FParse::Param(FCommandLine::Get(),TEXT("MecanumExactRollerSpin"));
        Input->BenchCount=0;Input->BenchHubId=INDEX_NONE;Input->BenchRollerCount=0;
        if(ReactionChassis && ReactionWheel)
        {
            TArray<UStaticMeshComponent*> Bench={ReactionChassis->Body.Get()};Bench.Append(ReactionWheel->PhysicalBodies());
            for(auto* B:Bench) if(Input->BenchCount<32) Input->BenchIds[Input->BenchCount++]=B->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
            Input->BenchOriginM=ReactionOriginM;
            if(!FParse::Param(FCommandLine::Get(),TEXT("MecanumDetachedRollers")))
            {
                Input->BenchHubId=ReactionWheel->Tyre->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
                for(UWheelTyreComponent* B:ReactionWheel->Rollers) Input->BenchRollerIds[Input->BenchRollerCount++]=B->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
            }
        }
        Input->TreadShearC=Settings.Tyres.Tread.PadShearC(Settings.RadiusM,Settings.WidthM);
        Input->TreadRotC=Settings.Tyres.Tread.PadRotC(Settings.RadiusM,Settings.WidthM);
        Input->TreadLinkC=Settings.Tyres.Tread.LinkC(Settings.RadiusM,Settings.WidthM);
        Input->TreadBendC=Settings.Tyres.Tread.BendC(Settings.RadiusM,Settings.WidthM);
        Input->TreadRestRadiusM=Settings.RadiusM-Settings.Tyres.Tread.DepthM*.5;
        Input->bTyreEnabled=Settings.Tyres.bEnabled;
        Input->bSmoothTyreContacts=Settings.bSwerve && !Settings.bMecanum && !FParse::Param(FCommandLine::Get(),TEXT("LegacyTyreContacts"));
        Input->bSmoothTyreEdges=Settings.Tyres.bDetailed && !Settings.bMecanum && !FParse::Param(FCommandLine::Get(),TEXT("NativeTyreEdges"));
        if(Settings.Mecanum.bReduced && !FParse::Param(FCommandLine::Get(),TEXT("NativeReducedSupport")))Input->bSmoothTyreContacts=Input->bSmoothTyreEdges=true;
        for(int I=0;I<4;++I) Input->bBrakeHolding[I]=Brake>0 && FMath::Abs(Wheels[I]->OmegaRadS)*Settings.RadiusM<.05
            && Wheels[I]->Tyre->GetPhysicsLinearVelocity().Size()<5;
        Input->TyreRadiusCm=Settings.RadiusM*100; Input->TyreHalfWidthCm=Settings.WidthM*50;
        if(Settings.Mecanum.bReduced)Input->TyreHalfWidthCm=Settings.Mecanum.HubWidthM*50;
        for(int32& Surface:Input->SurfaceIds) Surface=INDEX_NONE;
        if(Course) for(int I=0;I<Course->Surfaces.Num();++I)
        { Input->SurfaceIds[I]=Course->Surfaces[I]->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
          Input->ReducedSurfaceMu[I]=Course->Surfaces[I]->BodyInstance.GetSimplePhysicalMaterial()->Friction; }
        if(Pallet)
        {
            Input->PalletId=Pallet->Deck->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
            Input->CargoId=Pallet->Cargo->GetStaticMeshComponent()->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
            for(int32& Id:Input->TineIds) Id=INDEX_NONE;
            for(int I=0;I<2;++I) for(int J=0;J<Fork->Tines[I]->Sections.Num();++J)
                Input->TineIds[I*8+J]=Fork->Tines[I]->Sections[J]->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
        }
        for(int32 I=0;I<4;++I) Input->WheelIds[I]=Wheels[I]->Tyre->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
        for(int I=0;I<4;++I)
        {
            const auto& Contacts=Settings.bMecanum?Wheels[I]->Rollers:Wheels[I]->TreadPads;
            Input->TreadCounts[I]=Contacts.Num();
            for(int J=0;J<128;++J) Input->TreadIds[I][J]=J<Contacts.Num()?Contacts[J]->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx:INDEX_NONE;
        }
        for(int32 I=0;I<4;++I) Input->KnuckleIds[I]=Wheels[I]->Knuckle->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
        for(int32 I=0;I<4;++I) Input->CarrierIds[I]=Wheels[I]->BeltCarrier->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
        for(int32 I=0;I<4;++I)
        {
            Wheels[I]->UpdateLoad(Impulses[I],ContactCounts[I],PreviousPhysicsDt);
            for(int J=0;J<Wheels[I]->TreadNormalN.Num();++J)
            {
                double& N=Wheels[I]->TreadNormalN[J];
                if(N>1e-10) Wheels[I]->TreadContactPointM[J]/=N;
                N=PreviousPhysicsDt>0?N/PreviousPhysicsDt:0;
            }
            for(int J=0;J<Wheels[I]->RollerNormalN.Num();++J)
            {
                double& N=Wheels[I]->RollerNormalN[J];if(N>1e-10) Wheels[I]->RollerContactPointM[J]/=N;
                N=PreviousPhysicsDt>0?N/PreviousPhysicsDt:0;
                Wheels[I]->RollerBearingNm[J]=PreviousPhysicsDt>0?Wheels[I]->RollerBearingNm[J]/PreviousPhysicsDt:0;
            }
            Wheels[I]->RollerBearingPowerW=PreviousPhysicsDt>0?Wheels[I]->RollerBearingPowerW/PreviousPhysicsDt:0;
            Wheels[I]->ContactFrictionForceN=PreviousPhysicsDt>0?RoadFrictionImpulses[I]/PreviousPhysicsDt:FVector::ZeroVector;
            Wheels[I]->ContactMu=Impulses[I]>1e-8?MuImpulses[I]/Impulses[I]:0;
            Wheels[I]->ContactSteerNm=PreviousPhysicsDt>0?FVector::DotProduct(Moments[I],Chassis->Body->GetUpVector())/PreviousPhysicsDt:0;
            Wheels[I]->SteerMeasuredNm=PreviousPhysicsDt>0?FMath::Abs(FVector::DotProduct(SteerImpulses[I],Chassis->Body->GetUpVector()))/PreviousPhysicsDt:0;
            Wheels[I]->AxleSupportN=PreviousPhysicsDt>0?FMath::Abs(FVector::DotProduct(AxleImpulses[I],Wheels[I]->Knuckle->GetUpVector()))/PreviousPhysicsDt:0;
        }
    }
    Grounded=0; TotalNormalLoadN=0;
    for(AWheelActor* W:Wheels)
    {
        W->Measure(Settings.RadiusM,Settings.WidthM); Grounded+=W->bGrounded?1:0;
        TotalNormalLoadN+=W->NormalLoadN;
        MaxWheelSurfaceSpeed=FMath::Max(MaxWheelSurfaceSpeed,FMath::Abs(W->OmegaRadS)*Settings.RadiusM);
        W->SteerActualDeg=FMath::RadiansToDegrees(FMath::Atan2(
            FVector::DotProduct(W->Knuckle->GetForwardVector(),Chassis->Body->GetRightVector()),
            FVector::DotProduct(W->Knuckle->GetForwardVector(),Chassis->Body->GetForwardVector())));
        if(TimeS>2 && W->bGrounded) MaxLateralSlip=FMath::Max(MaxLateralSlip,FMath::Abs(W->LateralSlipMps));
    }
    for(AForkTineActor* T:Fork->Tines) T->UpdateStructure();
    if(bPayload) UpdatePayload(Dt,PreviousPhysicsDt);
    if(bPayload) UpdateCargo(Dt,PreviousPhysicsDt);
    if(TimeS>2)
    { MaxSideSpeed=FMath::Max(MaxSideSpeed,FMath::Abs(LateralMps)); MaxTilt=FMath::Max(MaxTilt,TiltDeg); MaxYawRate=FMath::Max(MaxYawRate,FMath::Abs(YawDegS)); MaxSpeed=FMath::Max(MaxSpeed,FMath::Abs(SpeedMps)); MinGrounded=FMath::Min(MinGrounded,Grounded); MaxForkHeight=FMath::Max(MaxForkHeight,ForkHeightM); }
    if(TestMode==TEXT("swerve_load") && TimeS>3)
    {
        LoadSumIntegral+=TotalNormalLoadN*PreviousPhysicsDt;
        FrontLoadIntegral+=(Wheels[0]->NormalLoadN+Wheels[1]->NormalLoadN)*PreviousPhysicsDt;
        LoadSampleTime+=PreviousPhysicsDt;
    }
    if(!TestMode.IsEmpty())
    {
        if(TimeS>=(IsTyreTest() && TestMode.Contains(TEXT("drop"))?8:2) && !bSeeded)
        {
            SettledTilt=TiltDeg; SettledSpeed=V.Size(); bSeeded=true;
            SwerveStart=Chassis->GetActorLocation();
            FVector WeightedCom=Chassis->Body->GetCenterOfMass()*Chassis->Body->GetMass()+Fork->Carriage->GetCenterOfMass()*Fork->Carriage->GetMass();
            for(AWheelActor* W:Wheels) for(UStaticMeshComponent* B:W->PhysicalBodies()) WeightedCom+=B->GetCenterOfMass()*B->GetMass();
            const double ComX=Chassis->Body->GetComponentQuat().UnrotateVector(WeightedCom/MassKg-Chassis->GetActorLocation()).X/100;
            ExpectedFrontFraction=.5+ComX/Settings.WheelbaseM;
            if(TestMode.StartsWith(TEXT("swerve_coast_")))
            {
                CoastStartSpeed=TestMode==TEXT("swerve_coast_reverse")?-2:2;
                SeedSpeed(CoastStartSpeed); bNeutral=true;
            }
            if(TestMode==TEXT("steering") || TestMode==TEXT("low_grip") || TestMode==TEXT("low_tyre")) SeedSpeed(6);
            if(TestMode==TEXT("rollover")) SeedSpeed(8);
            if(TestMode==TEXT("brake")) SeedSpeed(3);
            if(TestMode==TEXT("friction_keys"))
                if(auto* PC=Cast<AStabilityController>(GetWorld()->GetFirstPlayerController())) bFrictionKeysPassed=PC->TestFrictionBindings();
        }
        Throttle=((TestMode==TEXT("straight") || TestMode==TEXT("zero_grip")) && TimeS>2 && TimeS<7)?1:0;
        Steering=(TestMode==TEXT("steering") || TestMode==TEXT("low_grip") || TestMode==TEXT("low_tyre") || TestMode==TEXT("rollover")) && TimeS>2.5?1:0;
        if(TestMode==TEXT("powered_turn") || TestMode==TEXT("powered_turn_low") || TestMode==TEXT("powered_turn_left") || TestMode==TEXT("powered_turn_start"))
        {
            Throttle=TimeS>2?1:0;
            const double TurnAt=TestMode==TEXT("powered_turn_start")?2:4;
            Steering=TimeS>TurnAt?(TestMode==TEXT("powered_turn_left")?-1:1):0;
        }
        if(TestMode==TEXT("friction_live"))
        {
            Throttle=TimeS>2?1:0;
            if(TimeS>6 && !bFrictionSwitched) { bFrictionSwitched=true; ChangeFriction(false,-Settings.RoadMu); }
            if(bFrictionSwitched && !bFrictionRestored) MaxLiveSlip=FMath::Max(MaxLiveSlip,FMath::Abs(Wheels[0]->LongSlipMps));
            if(TimeS>9 && !bFrictionRestored) { bFrictionRestored=true; ChangeFriction(false,InitialRoadMu-Settings.RoadMu); }
        }
        Brake=(TestMode==TEXT("brake") && TimeS>2.5)?1:0;
        const double Target=((TestMode==TEXT("fork") || TestMode==TEXT("swerve_fork")) && TimeS>2 && TimeS<7)?1:0;
        if(!bPayload) LiftTargetM=FMath::FInterpConstantTo(LiftTargetM,Target,Dt,Settings.ForkSpeedMps);
        if(Settings.bSwerve)
        {
            Throttle=Strafe=Steering=Brake=0;
            if(TimeS>2)
            {
                if(TestMode==TEXT("swerve_straight") || TestMode==TEXT("swerve_zero_grip") || TestMode==TEXT("swerve_low_grip") || TestMode==TEXT("swerve_brake")) Throttle=1;
                if(TestMode==TEXT("swerve_reverse")) Throttle=-1;
                if(TestMode==TEXT("swerve_friction_live"))
                {
                    Throttle=1;
                    if(TimeS>6 && !bFrictionSwitched) { bFrictionSwitched=true; ChangeFriction(false,-Settings.RoadMu); }
                    if(TimeS>9 && !bFrictionRestored) { bFrictionRestored=true; ChangeFriction(false,InitialRoadMu-Settings.RoadMu); }
                }
                if(TestMode==TEXT("swerve_strafe")) Strafe=1;
                if(TestMode==TEXT("swerve_strafe_left")) Strafe=-1;
                if(TestMode==TEXT("swerve_spin")) Steering=1;
                if(TestMode==TEXT("swerve_spin_left")) Steering=-1;
                if(TestMode==TEXT("swerve_combined")) { Throttle=.5; Strafe=.3; Steering=.5; }
                if(TestMode==TEXT("swerve_rollover")) { Throttle=1; Steering=TimeS>6?1:0; }
                if(TestMode==TEXT("swerve_transition"))
                { if(TimeS<5) Throttle=1; else if(TimeS<10) Strafe=1; else Steering=1; }
                if(TestMode==TEXT("swerve_brake") && TimeS>7) { Throttle=0; Brake=1; }
                if(TestMode==TEXT("swerve_friction_keys") && !bFrictionKeysPassed)
                    if(auto* PC=Cast<AStabilityController>(GetWorld()->GetFirstPlayerController())) bFrictionKeysPassed=PC->TestFrictionBindings();
            }
        }
    }
    if(bPayload && !TestMode.IsEmpty() && !IsTyreTest() && !IsHydraulicTest() && !IsIntegrationTest()) ControlPayloadTest(Dt);
    if(IsPowerTest()) ControlPowerTest();
    if(IsTyreTest()) ControlTyreTest();
    if(IsHydraulicTest()) ControlHydraulicTest();
    if(IsIntegrationTest()) ControlIntegrationTest();
    if(IsComponentTest()) ControlComponentTest();
    if(TestMode.StartsWith(TEXT("mecanum_"))) ControlMecanumTest();
    SteerDeg=FMath::FInterpConstantTo(SteerDeg,Steering*Settings.MaxSteerDeg,Dt,Settings.SteerRateDegS);
    if(TestMode.StartsWith(TEXT("powered_turn")) && TimeS>6 && Throttle>.9)
        MinPoweredTurnSpeed=FMath::Min(MinPoweredTurnSpeed,SpeedMps);
    LiftTargetM=FMath::Clamp(LiftTargetM+Lift*Settings.ForkSpeedMps*Dt,0.,Settings.ForkTravelM);
    Fork->SetLiftTarget(LiftTargetM);
    Fork->UpdateHydraulics();
    if(DynamicsSensors)for(const auto& Sample:Fork->PressureSamples)DynamicsSensors->Model.Capture(Sample);
    if(JointSensors)JointSensors->Model.Deliver(TimeS);
    if(DynamicsSensors)DynamicsSensors->Model.Deliver(TimeS);
    if(DriveSensors)DriveSensors->Observe(*this,Dt);
    if(MissionControl)MissionControl->Update(*this,Dt);
    if(HandlingControl)HandlingControl->Update(*this,Dt);
    if(VehicleGuard)VehicleGuard->Update(*this);
    if(HandlingControl)HandlingControl->Apply(*this);
    if(MissionControl)MissionControl->Apply(*this);
    if(DriveSensors)DriveSensors->Correct(*this);
    UpdateSensorLiftControl(Dt);
    if(Settings.bSwerve) UpdateHydraulicLab(Dt);
    if(Settings.bMecanum) DriveMecanum(Dt);
    else if(Settings.bSwerve) DriveSwerve(Dt);
    else for(int I=0;I<4;++I)
    {
        AWheelActor* W=Wheels[I]; double RearAngle=0;
        if(I>=2 && FMath::Abs(SteerDeg)>.001)
        {
            const double Radius=Settings.WheelbaseM/FMath::Tan(FMath::DegreesToRadians(FMath::Abs(SteerDeg)));
            const double Y=(I%2==0?-1:1)*Settings.TrackM*.5;
            RearAngle=-FMath::Sign(SteerDeg)*FMath::RadiansToDegrees(FMath::Atan(Settings.WheelbaseM/(Radius-FMath::Sign(SteerDeg)*Y)));
        }
        double Torque=I<2?Throttle*Settings.DriveNm*FMath::Clamp(1-FMath::Max(0.,W->OmegaRadS*FMath::Sign(Throttle))*Settings.RadiusM/Settings.MaxSpeedMps,0.,1.):0;
        if(Brake>0 || TiltDeg>70) Torque=0;
        W->Drive(Torque,Brake*Settings.BrakeNm,RearAngle);
    }
    if(Settings.bSwerve) for(AWheelActor* W:Wheels) W->ApplyResistance(Settings,bResistanceEnabled && !(Settings.bMecanum && FParse::Param(FCommandLine::Get(),TEXT("MecanumNoBearing"))));
    if(Settings.bSwerve) PrepareEnergyWork();
    if(Settings.bSwerve) UpdatePower(Dt);
    if(Settings.bSwerve) UpdateTyres(Dt);
    if(Settings.bSwerve) UpdateValidation(Dt);
    if(Settings.bSwerve) UpdateEnergyAudit();
    if(JointSensors)JointSensors->Update(TimeS);
    if(DynamicsSensors)DynamicsSensors->Update(TimeS);
    if(ProximitySensors)ProximitySensors->Update(*this);
    PublishWebTelemetry();
    if(TimeS>=NextRecord)
    {
        NextRecord=TimeS+.05;
        FString Row=FString::Printf(TEXT("%.4f,%.2f,%.2f,%.2f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.3f,%.3f,%.4f"),TimeS,Settings.TyreMu,Settings.RoadMu,Settings.EffectiveMu(),Settings.ChassisComOffsetM,SpeedMps,LateralMps,YawDegS,RollDeg,PitchDeg,TiltDeg,Grounded,SteerDeg,LiftTargetM,ForkHeightM);
        for(AWheelActor* W:Wheels) Row+=FString::Printf(TEXT(",%.5f"),W->LateralSlipMps);
        for(AWheelActor* W:Wheels) Row+=FString::Printf(TEXT(",%.5f"),W->ClearanceM);
        Row+=FString::Printf(TEXT(",%.4f,%.5f,%.5f,%.4f"),AccelerationMps2,V.X,V.Y,Chassis->Body->GetCenterOfMass().Z/100);
        for(AWheelActor* W:Wheels) Row+=FString::Printf(TEXT(",%.5f,%.5f,%.3f,%.3f,%.3f"),W->OmegaRadS,W->LongSlipMps,W->DriveTorqueNm,W->BrakeLimitNm,W->SteerActualDeg);
        Row+=FString::Printf(TEXT(",%.1f,%.1f,%.3f,%.3f,%.5f,%.5f,%.6f,%.6f"),Throttle,Brake,
            Wheels[2]->SteeringJoint->GetCurrentTwist(),Wheels[3]->SteeringJoint->GetCurrentTwist(),
            FMath::Abs(FVector::DotProduct(Wheels[2]->Tyre->GetUpVector(),Wheels[2]->Knuckle->GetRightVector())),
            FMath::Abs(FVector::DotProduct(Wheels[3]->Tyre->GetUpVector(),Wheels[3]->Knuckle->GetRightVector())),
            FVector::Distance(Wheels[2]->Tyre->GetComponentLocation(),Wheels[2]->Knuckle->GetComponentLocation())/100,
            FVector::Distance(Wheels[3]->Tyre->GetComponentLocation(),Wheels[3]->Knuckle->GetComponentLocation())/100);
        Row+=FString::Printf(TEXT(",%d,%d,%d,%d"),Wheels[0]->Tyre->RigidBodyIsAwake()?1:0,Wheels[2]->Tyre->RigidBodyIsAwake()?1:0,Wheels[2]->ChassisContactCount,Wheels[3]->ChassisContactCount);
        Row+=FString::Printf(TEXT(",%.2f,%.4f,%.4f,%.4f,%.4f,%.4f"),Strafe,TargetVelocity.X,TargetVelocity.Y,TargetYawRadS,Chassis->GetActorLocation().X/100,Chassis->GetActorLocation().Y/100);
        for(AWheelActor* W:Wheels) Row+=FString::Printf(TEXT(",%.3f,%.4f,%d"),W->SteerTargetDeg,W->SpeedTargetMps,W->ChassisContactCount);
        Rows.Add(Row);
        for(AWheelActor* W:Wheels) Rows.Last()+=FString::Printf(TEXT(",%.3f,%.3f,%d,%.4f,%.4f,%.4f"),W->NormalLoadN,W->NormalLoadFilteredN,W->FloorContacts,W->RollingTorqueNm,W->BearingTorqueNm,W->ResistancePowerW);
        Rows.Last()+=FString::Printf(TEXT(",%.3f,%d,%d,%.5f,%.4f,%.4f,%.6f"),TotalNormalLoadN,bNeutral?1:0,bResistanceEnabled?1:0,Settings.RollingResistance,Settings.BearingCoulombNm,Settings.BearingViscousNms,PreviousPhysicsDt);
        Rows.Last()+=FString::Printf(TEXT(",%.3f,%.6f,%.5f"),Settings.WheelSurfaceSpeedLimitMps,WheelSpeedScale,AllocatedYawRadS);
        if(bPayload) Rows.Last()+=PayloadCsv(false);
        if(bPayload) Rows.Last()+=CargoCsv(false);
        if(Settings.bSwerve) Rows.Last()+=PowerCsv(false);
        if(Settings.bSwerve) Rows.Last()+=TyreCsv(false);
        if(Settings.bSwerve) Rows.Last()+=HydraulicCsv(false);
        if(Settings.bSwerve) Rows.Last()+=EnergyCsv(false);
        if(Settings.bSwerve) Rows.Last()+=ValidationCsv(false);
        Rows.Last()+=MecanumCsv(false);
        Rows.Last()+=FString::Printf(TEXT(",%.8f,%d,%.8f,%.8f"),RackSupportN,RackTrial.Stage,EnergyAudit.PalletPhysicsMassKg,EnergyAudit.CargoPhysicsMassKg);
    }
    if(TimeS>=NextFlush) { NextFlush+=5; Flush(); }
    if(bCapture && TimeS>CaptureTimeS && !bCaptured)
    {
        bCaptured=true;
        const FString Name=TestMode.IsEmpty()?TEXT("StabilityLab.png"):TEXT("Stability-")+FPaths::MakeValidFileName(TestMode)+TEXT(".png");
        FScreenshotRequest::RequestScreenshot(FPaths::ProjectSavedDir()/TEXT("Screenshots")/Name,true,false);
    }
    if(bCapture && TimeS>CaptureTimeS+1) { WritePerformanceReport();Flush(); FGenericPlatformMisc::RequestExit(false); }
    double TestDuration=12;
    if(TestMode==TEXT("mecanum_joint_sensors"))TestDuration=70;
    if(TestMode==TEXT("swerve_transition")) TestDuration=18;
    if(bPayload || IsPowerTest() || IsTyreTest()) TestDuration=16;
    if(TestMode==TEXT("swerve_payload_unload") || TestMode==TEXT("swerve_payload_cargo_unload") || IsHydraulicTest()) TestDuration=20;
    if(TestMode==TEXT("swerve_payload_s_curve")) TestDuration=32;
    // Gravity return is pressure dependent: the unloaded fork lowers near
    // 0.10 m/s, unlike the former 0.25 m/s bidirectional position servo.
    if(TestMode==TEXT("swerve_fork") && Settings.Hydraulics.bEnabled) TestDuration=24;
    // Allow a full climb + descent without artificial contact propulsion.
    // Distance, speed and stopping checks remain unchanged.
    if(IsIntegrationTest()) TestDuration=TerrainPreset==TEXT("ramp")?80:32;
    if(TestMode.Contains(TEXT("ramp_brake"))) TestDuration=40;
    if(IsComponentTest()) TestDuration=TestMode.Contains(TEXT("ramp_"))?60:32;
    if(TestMode.StartsWith(TEXT("mecanum_")) && bPayload) TestDuration=TestMode.Contains(TEXT("ramp"))?60:32;
    if(IsSensorLiftTest())TestDuration=60;
    if(IsSensorDriveTest())TestDuration=30;
    if(IsPalletPerceptionTest())TestDuration=TestMode.EndsWith(TEXT("high"))?24:16;
    if(IsVehicleGuardTest())TestDuration=TestMode.EndsWith(TEXT("carry"))?40:30;
    double ReferenceCrr=-1;
    if(TestMode==TEXT("mecanum_coast") && FParse::Value(FCommandLine::Get(),TEXT("ReferenceRollingCoefficient="),ReferenceCrr))TestDuration=FMath::Max(TestDuration,5+2./(9.81*ReferenceCrr));
    if(IsRackTest())
    {
        TestDuration=180;
        if(RackTrial.Failed || (RackTrial.Stage>=9 && TimeS-RackTrial.StageStart>2)){FinishTest();return;}
    }
    if(IsHandlingTest())TestDuration=140;
    if(IsHandlingTest()&&HandlingControl&&(HandlingControl->Model.State==TEXT("complete")||HandlingControl->Model.Faulted)&&TimeS-HandlingControl->Model.StageS>2){FinishTest();return;}
    if(IsMissionTest()){TestDuration=600;if(MissionTestCanFinish()){FinishTest();return;}}
    if(!TestMode.IsEmpty() && TimeS>=TestDuration) FinishTest();
}
void AStabilityLab::FinishTest()
{
    bFinished=true; Flush();if(IsMissionTest())WritePerformanceReport();
    UE_LOG(LogTemp,Display,TEXT("STABILITY_CSV %s"),*FPaths::ConvertRelativePathToFull(CsvPath));
    UE_LOG(LogTemp,Display,TEXT("NUMERICAL_STEP max_game_s=%.9f max_inner_s=%.9f"),ValidationState.MaxGameStepS,ValidationState.MaxInnerStepS);
    UE_LOG(LogTemp,Display,TEXT("NUMERICAL_CONTACT refined=%lld legacy=%d"),ValidationState.RefinedContactCount,FParse::Param(FCommandLine::Get(),TEXT("LegacyTyreContacts"))?1:0);
    const double ExpectedMass=Settings.ChassisMassKg+Settings.ForkMassKg+4*(Settings.WheelMassKg+Settings.KnuckleMassKg);
    bool Pass=SettledTilt<3 && SettledSpeed<.2 && FMath::Abs(MassKg-ExpectedMass)<1
        && FMath::IsFinite(SpeedMps) && FMath::IsFinite(TiltDeg) && MaxSpeed<8.5 && MaxYawRate<400;
    if(Settings.bSwerve)
    {
        const FVector Displacement=(Chassis->GetActorLocation()-SwerveStart)/100;
        Pass &= FrictionMaterialsMatch();
        if(TestMode!=TEXT("swerve_rollover")) Pass &= MaxTilt<10 && MaxModuleError<8;
        for(AWheelActor* W:Wheels) Pass &= W->ChassisContactCount==0 && FMath::IsFinite(W->OmegaRadS) && FMath::Abs(W->DriveTorqueNm)<=Settings.DriveNm+.01;
        if(TestMode.StartsWith(TEXT("mecanum_"))) Pass &= CheckMecanumTest();
        else if(IsComponentTest()) Pass &= CheckComponentTest();
        else if(IsIntegrationTest()) Pass &= CheckIntegrationTest();
        else if(IsHydraulicTest()) Pass &= CheckHydraulicTest();
        else if(IsTyreTest()) Pass &= CheckTyreTest();
        else if(IsPowerTest()) Pass &= CheckPowerTest();
        else if(bPayload) Pass &= CheckPayloadTest();
        else if(TestMode==TEXT("swerve_settle")) Pass &= MaxSpeed<.1 && MaxSideSpeed<.1 && Grounded==4;
        else if(TestMode==TEXT("swerve_straight")) Pass &= SpeedMps>2 && MaxYawRate<5 && FMath::Abs(Displacement.Y)<.5;
        else if(TestMode==TEXT("swerve_reverse")) Pass &= SpeedMps<-2 && MaxYawRate<5;
        else if(TestMode==TEXT("swerve_friction_live")) Pass &= bFrictionSwitched && bFrictionRestored && FrictionMaterialsMatch() && SpeedMps>1;
        else if(TestMode==TEXT("swerve_strafe") || TestMode==TEXT("swerve_strafe_left")) Pass &= LateralMps*(TestMode==TEXT("swerve_strafe")?1:-1)>2 && MaxYawRate<5 && FMath::Abs(Displacement.X)<.5;
        else if(TestMode==TEXT("swerve_spin") || TestMode==TEXT("swerve_spin_left")) Pass &= YawDegS*(TestMode==TEXT("swerve_spin")?1:-1)>25 && Displacement.Size2D()<.5 && FMath::Abs(SpeedMps)<.2 && FMath::Abs(LateralMps)<.2;
        else if(TestMode==TEXT("swerve_combined")) Pass &= SpeedMps>.5 && LateralMps>.2 && YawDegS>10;
        else if(TestMode==TEXT("swerve_transition")) Pass &= MaxSpeed>1 && MaxSideSpeed>1 && YawDegS>20;
        else if(TestMode==TEXT("swerve_zero_grip")) Pass &= MaxSpeed<.1 && MaxWheelSurfaceSpeed>2;
        else if(TestMode==TEXT("swerve_low_grip")) Pass &= MaxSpeed>.5 && MaxLateralSlip<2;
        else if(TestMode==TEXT("swerve_brake")) Pass &= MaxSpeed>2 && FMath::Abs(SpeedMps)<.1;
        else if(TestMode==TEXT("swerve_fork")) Pass &= MaxForkHeight>.85 && FMath::Abs(ForkHeightM)<.1;
        else if(TestMode==TEXT("swerve_friction_keys")) Pass &= bFrictionKeysPassed;
        else if(TestMode==TEXT("swerve_rollover")) Pass &= MaxTilt>65 && MinGrounded<=2;
        else if(TestMode==TEXT("swerve_load"))
        {
            const double Mean=LoadSumIntegral/FMath::Max(LoadSampleTime,1e-6);
            const double Fraction=FrontLoadIntegral/FMath::Max(LoadSumIntegral,1e-6);
            Pass &= LoadSampleTime>8 && FMath::Abs(Mean-MassKg*9.81)/(MassKg*9.81)<.05 && FMath::Abs(Fraction-ExpectedFrontFraction)<.05;
            for(AWheelActor* W:Wheels) Pass &= W->NormalLoadFilteredN>1000;
            UE_LOG(LogTemp,Display,TEXT("LOAD_CHECK mean_n=%.3f weight_n=%.3f front_fraction=%.4f expected_front=%.4f"),Mean,MassKg*9.81,Fraction,ExpectedFrontFraction);
        }
        else if(TestMode.StartsWith(TEXT("swerve_coast_")))
        {
            const double Final=SpeedMps*FMath::Sign(CoastStartSpeed);
            Pass &= bNeutral && Final>-.05 && Final<2.1 && FMath::Abs(LateralMps)<.15;
            if(TestMode==TEXT("swerve_coast_none")) Pass &= Final>1.8;
            if(TestMode==TEXT("swerve_coast_bearing")) Pass &= Final>.8 && Final<1.9;
            if(TestMode==TEXT("swerve_coast_both") || TestMode==TEXT("swerve_coast_rolling") || TestMode==TEXT("swerve_coast_reverse")) Pass &= Final<1.1 && Final>.1;
            if(TestMode==TEXT("swerve_coast_high")) Pass &= Final<.1;
            for(AWheelActor* W:Wheels) Pass &= W->DriveTorqueNm==0 && W->BrakeLimitNm==0 && W->ResistancePowerW>=0;
            UE_LOG(LogTemp,Display,TEXT("COAST_CHECK initial=%.3f final=%.4f crr=%.4f bearing=%.3f/%.3f"),CoastStartSpeed,SpeedMps,Settings.RollingResistance,Settings.BearingCoulombNm,Settings.BearingViscousNms);
            UE_LOG(LogTemp,Display,TEXT("NUMERICAL_COAST start_s=%.4f duration_s=%.4f start_mps=%.6f final_mps=%.6f delta_j=%.4f peak_gain_j=%.4f passive_steer=%d"),ValidationState.CoastStartS,TimeS-ValidationState.CoastStartS,ValidationState.CoastStartMps,SpeedMps,ValidationState.CoastDeltaJ,ValidationState.CoastPeakGainJ,FParse::Param(FCommandLine::Get(),TEXT("PassiveSteer"))?1:0);
        }
        else Pass=false;
        UE_LOG(LogTemp,Display,TEXT("SWERVE_CHECK displacement=%.3f/%.3f m final_v=%.3f/%.3f yaw=%.3f module_error=%.3f contacts=%d/%d/%d/%d"),Displacement.X,Displacement.Y,SpeedMps,LateralMps,YawDegS,MaxModuleError,Wheels[0]->ChassisContactCount,Wheels[1]->ChassisContactCount,Wheels[2]->ChassisContactCount,Wheels[3]->ChassisContactCount);
    }
    else if(TestMode==TEXT("settle")) Pass &= MaxTilt<3 && MaxSpeed<.2 && Grounded==4;
    else if(TestMode==TEXT("straight")) Pass &= MaxSpeed>1 && MaxSpeed<3.3 && MaxTilt<5 && MaxYawRate<5;
    else if(TestMode==TEXT("brake")) Pass &= FMath::Abs(SpeedMps)<.1 && MaxTilt<10;
    else if(TestMode==TEXT("zero_grip")) Pass &= MaxSpeed<.1 && MaxTilt<3 && MaxWheelSurfaceSpeed>2;
    else if(TestMode==TEXT("steering") || TestMode==TEXT("low_grip") || TestMode==TEXT("low_tyre")) Pass &= MaxYawRate>10 && MaxLateralSlip>.2;
    else if(TestMode==TEXT("rollover")) Pass &= MaxTilt>65 && MinGrounded<=2;
    else if(TestMode==TEXT("fork")) Pass &= MaxForkHeight>.85 && FMath::Abs(ForkHeightM)<.1 && MaxTilt<5;
    else if(TestMode.StartsWith(TEXT("powered_turn")))
        Pass &= SpeedMps>.5 && MinPoweredTurnSpeed>.5 && MaxTilt<10 && MaxYawRate>5 && Wheels[2]->ChassisContactCount==0 && Wheels[3]->ChassisContactCount==0;
    else if(TestMode==TEXT("friction_live")) Pass &= bFrictionSwitched && bFrictionRestored && FrictionMaterialsMatch() && MaxLiveSlip>.2 && FMath::Abs(Wheels[0]->LongSlipMps)<.1 && SpeedMps>1;
    else if(TestMode==TEXT("friction_keys")) Pass &= bFrictionKeysPassed && FrictionMaterialsMatch();
    else Pass=false;
    UE_LOG(LogTemp,Display,TEXT("STABILITY_TEST %s %s settled_tilt=%.3f settled_speed=%.4f mass=%.1f max_speed=%.3f max_tilt=%.3f max_lateral_slip=%.3f max_yaw=%.3f min_wheels=%d final_speed=%.4f fork_max=%.4f fork_final=%.4f"),*TestMode,Pass?TEXT("PASS"):TEXT("FAIL"),SettledTilt,SettledSpeed,MassKg,MaxSpeed,MaxTilt,MaxLateralSlip,MaxYawRate,MinGrounded,SpeedMps,MaxForkHeight,ForkHeightM);
    UE_LOG(LogTemp,Display,TEXT("DRIVE_CHECK %s input=%.1f brake=%.1f rear_steer=%.3f/%.3f self_contacts=%d/%d front_slip=%.4f live_peak_slip=%.4f keys=%d material_match=%d"),
        *TestMode,Throttle,Brake,Wheels[2]->SteerActualDeg,Wheels[3]->SteerActualDeg,Wheels[2]->ChassisContactCount,Wheels[3]->ChassisContactCount,Wheels[0]->LongSlipMps,MaxLiveSlip,bFrictionKeysPassed?1:0,FrictionMaterialsMatch()?1:0);
    if(TestMode.StartsWith(TEXT("powered_turn"))) UE_LOG(LogTemp,Display,TEXT("POWERED_MIN_SPEED %s %.4f m/s"),*TestMode,MinPoweredTurnSpeed);
    FGenericPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
}
void AStabilityLab::Flush() { if(!CsvPath.IsEmpty() && Rows.Num()>1) FFileHelper::SaveStringArrayToFile(Rows,*CsvPath,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM); }
void AStabilityLab::ReleaseLoadSampler()
{
    if(LoadSampler)
    {
        if(GetWorld() && GetWorld()->GetPhysicsScene()) GetWorld()->GetPhysicsScene()->GetSolver()->UnregisterAndFreeSimCallbackObject_External(LoadSampler);
        LoadSampler=nullptr;
    }
}
void AStabilityLab::EndPlay(const EEndPlayReason::Type R) { ShutdownCameraViews(); if(WebTelemetry) { WebTelemetry->Stop(); WebTelemetry.Reset(); } ReleaseLoadSampler(); JointSensors.Reset(); DynamicsSensors.Reset(); ProximitySensors.Reset(); SensorLift.Reset(); MissionControl.Reset(); HandlingControl.Reset(); PalletRecognition.Reset();if(PalletVisionDistractor){PalletVisionDistractor->Destroy();PalletVisionDistractor=nullptr;}ClearNavigationWalls();DriveSensors.Reset(); VehicleGuard.Reset(); if(GuardObstacle){GuardObstacle->Destroy();GuardObstacle=nullptr;} if(HandlingRack){HandlingRack->Destroy();HandlingRack=nullptr;} Flush(); Super::EndPlay(R); }

void AStabilityController::BeginPlay()
{
    Super::BeginPlay(); LabCamera=GetWorld()->SpawnActor<ACameraActor>(); LabCamera->GetCameraComponent()->SetFieldOfView(50);
    bTopView=FParse::Param(FCommandLine::Get(),TEXT("SwerveTopView"));
    if(FParse::Param(FCommandLine::Get(),TEXT("WarehouseLab")) || GetWorld()->GetMapName().Contains(TEXT("WarehouseLab"))) {CameraDistanceCm=760;CameraPitchDeg=-19;CameraYawDeg=-110;}
    SetViewTarget(LabCamera);
    FInputModeGameAndUI Mode; Mode.SetHideCursorDuringCapture(false); Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
    SetInputMode(Mode); bShowMouseCursor=true;
}
void AStabilityController::PlayerTick(float Dt)
{
    Super::PlayerTick(Dt); auto* B=Lab(GetWorld()); if(!B || !B->Chassis) return;
    // Inspection view hides only the deck render mesh. Welded parts, collision,
    // mass and all constraints remain active, including across a rig reset.
    if(!B->Settings.bReferenceGeometry) B->Chassis->Body->SetVisibility(!bTopView || !B->Settings.bSwerve,false);
    else
    {
        TInlineComponentArray<UStaticMeshComponent*> Models(B->Chassis);
        for(auto* Model:Models) if(Model->ComponentHasTag(TEXT("ReferenceBodyVisual"))) Model->SetVisibility(!bTopView,false);
    }
    if(B->TestMode.IsEmpty())
    {
        B->Throttle=(IsInputKeyDown(EKeys::W)?1:0)-(IsInputKeyDown(EKeys::S)?1:0);
        B->Steering=(IsInputKeyDown(EKeys::D)?1:0)-(IsInputKeyDown(EKeys::A)?1:0);
        B->Strafe=(IsInputKeyDown(EKeys::C)?1:0)-(IsInputKeyDown(EKeys::Z)?1:0);
        B->Brake=IsInputKeyDown(EKeys::SpaceBar)?1:0;
        B->Lift=(IsInputKeyDown(EKeys::E)?1:0)-(IsInputKeyDown(EKeys::Q)?1:0);
    }
    double MouseX=0,MouseY=0;
    const bool HasMouse=GetMousePosition(MouseX,MouseY);
    if(HasMouse && bHadMouse)
    {
        const FVector2D Delta=FVector2D(MouseX,MouseY)-PreviousMouse;
        if((IsInputKeyDown(EKeys::RightMouseButton) || IsInputKeyDown(EKeys::LeftMouseButton)) && !bTopView)
        {
            CameraYawDeg=FMath::UnwindDegrees(CameraYawDeg-Delta.X*.25);
            CameraPitchDeg=FMath::Clamp(CameraPitchDeg-Delta.Y*.25,-80.,-5.);
        }
        if(IsInputKeyDown(EKeys::MiddleMouseButton))
            CameraPan+=(-LabCamera->GetActorRightVector()*Delta.X+LabCamera->GetActorUpVector()*Delta.Y)*CameraDistanceCm*.001;
    }
    bHadMouse=HasMouse; PreviousMouse=FVector2D(MouseX,MouseY);
    const bool Closeup=FParse::Param(FCommandLine::Get(),TEXT("TreadCloseup")) && B->Wheels.Num()==4;
    const FVector Focus=(Closeup?B->Wheels[0]->Tyre->GetComponentLocation():B->Chassis->GetActorLocation()+(B->Settings.bReferenceGeometry?FVector(140,0,65):FVector(190,140,50)))+CameraPan;
    const double Distance=Closeup?150:CameraDistanceCm;
    const FVector Offset=bTopView?FVector(0,0,Distance):FRotator(-CameraPitchDeg,CameraYawDeg,0).Vector()*Distance;
    const FVector P=Focus+Offset;
    LabCamera->SetActorLocation(P); LabCamera->SetActorRotation(bTopView?FRotator(-90,0,0):(Focus-P).Rotation());
}
void AStabilityController::SetupInputComponent()
{
    Super::SetupInputComponent();
    InputComponent->BindKey(EKeys::R,IE_Pressed,this,&AStabilityController::Reset);
    InputComponent->BindKey(EKeys::One,IE_Pressed,this,&AStabilityController::TyreDown);
    InputComponent->BindKey(EKeys::Two,IE_Pressed,this,&AStabilityController::TyreUp);
    InputComponent->BindKey(EKeys::Three,IE_Pressed,this,&AStabilityController::RoadDown);
    InputComponent->BindKey(EKeys::Four,IE_Pressed,this,&AStabilityController::RoadUp);
    InputComponent->BindKey(EKeys::NumPadOne,IE_Pressed,this,&AStabilityController::TyreDown);
    InputComponent->BindKey(EKeys::NumPadTwo,IE_Pressed,this,&AStabilityController::TyreUp);
    InputComponent->BindKey(EKeys::NumPadThree,IE_Pressed,this,&AStabilityController::RoadDown);
    InputComponent->BindKey(EKeys::NumPadFour,IE_Pressed,this,&AStabilityController::RoadUp);
    for(const FKey& Key:{EKeys::One,EKeys::NumPadOne}) InputComponent->BindKey(Key,IE_Repeat,this,&AStabilityController::RepeatTyreDown);
    for(const FKey& Key:{EKeys::Two,EKeys::NumPadTwo}) InputComponent->BindKey(Key,IE_Repeat,this,&AStabilityController::RepeatTyreUp);
    for(const FKey& Key:{EKeys::Three,EKeys::NumPadThree}) InputComponent->BindKey(Key,IE_Repeat,this,&AStabilityController::RepeatRoadDown);
    for(const FKey& Key:{EKeys::Four,EKeys::NumPadFour}) InputComponent->BindKey(Key,IE_Repeat,this,&AStabilityController::RepeatRoadUp);
    InputComponent->BindKey(EKeys::F,IE_Pressed,this,&AStabilityController::RoadPreset);
    InputComponent->BindKey(EKeys::G,IE_Pressed,this,&AStabilityController::Stress);
    InputComponent->BindKey(EKeys::V,IE_Pressed,this,&AStabilityController::ToggleView);
    InputComponent->BindKey(EKeys::N,IE_Pressed,this,&AStabilityController::ToggleNeutral);
    InputComponent->BindKey(EKeys::B,IE_Pressed,this,&AStabilityController::ToggleResistance);
    InputComponent->BindKey(EKeys::H,IE_Pressed,this,&AStabilityController::ToggleLoadView);
    InputComponent->BindKey(EKeys::M,IE_Pressed,this,&AStabilityController::ToggleMotorView);
    InputComponent->BindKey(EKeys::T,IE_Pressed,this,&AStabilityController::NextTerrain);
    InputComponent->BindKey(EKeys::J,IE_Pressed,this,&AStabilityController::ToggleTyreView);
    InputComponent->BindKey(EKeys::K,IE_Pressed,this,&AStabilityController::ToggleHydraulicView);
    InputComponent->BindKey(EKeys::P,IE_Pressed,this,&AStabilityController::ToggleCameraViews);
    InputComponent->BindKey(EKeys::O,IE_Pressed,this,&AStabilityController::ToggleRearCameraMode);
    InputComponent->BindKey(EKeys::U,IE_Pressed,this,&AStabilityController::ToggleJointSensorOutage);
    InputComponent->BindKey(EKeys::L,IE_Pressed,this,&AStabilityController::ToggleProximityRays);
    InputComponent->BindKey(EKeys::Y,IE_Pressed,this,&AStabilityController::AcknowledgeSensorLift);
    InputComponent->BindKey(EKeys::X,IE_Pressed,this,&AStabilityController::EmergencyVehicleStop);
    InputComponent->BindKey(EKeys::I,IE_Pressed,this,&AStabilityController::ToggleLidarRays);
    InputComponent->BindKey(EKeys::Five,IE_Pressed,this,&AStabilityController::PickGround);
    InputComponent->BindKey(EKeys::Six,IE_Pressed,this,&AStabilityController::PickLow);
    InputComponent->BindKey(EKeys::Seven,IE_Pressed,this,&AStabilityController::PickHigh);
    InputComponent->BindKey(EKeys::Eight,IE_Pressed,this,&AStabilityController::PlaceLow);
    InputComponent->BindKey(EKeys::Nine,IE_Pressed,this,&AStabilityController::PlaceHigh);
    InputComponent->BindKey(EKeys::Zero,IE_Pressed,this,&AStabilityController::CancelHandling);
    InputComponent->BindKey(EKeys::Enter,IE_Pressed,this,&AStabilityController::StartMission);
    InputComponent->BindKey(EKeys::BackSpace,IE_Pressed,this,&AStabilityController::PauseMission);
    InputComponent->BindKey(EKeys::RightBracket,IE_Pressed,this,&AStabilityController::ResumeMission);
    InputComponent->BindKey(EKeys::PageDown,IE_Pressed,this,&AStabilityController::SelectMissionLow);
    InputComponent->BindKey(EKeys::PageUp,IE_Pressed,this,&AStabilityController::SelectMissionHigh);
    InputComponent->BindKey(EKeys::MouseScrollUp,IE_Pressed,this,&AStabilityController::ZoomIn);
    InputComponent->BindKey(EKeys::MouseScrollDown,IE_Pressed,this,&AStabilityController::ZoomOut);
    InputComponent->BindKey(EKeys::Home,IE_Pressed,this,&AStabilityController::ResetCamera);
}
void AStabilityController::ZoomIn() { CameraDistanceCm=FMath::Max(350.,CameraDistanceCm/1.12); }
void AStabilityController::ZoomOut() { CameraDistanceCm=FMath::Min(3500.,CameraDistanceCm*1.12); }
void AStabilityController::ResetCamera() { const auto* B=Lab(GetWorld()); const bool Compact=B && B->Settings.bReferenceGeometry; CameraYawDeg=Compact?-110:-62.86; CameraPitchDeg=Compact?-19:-16.65; CameraDistanceCm=Compact?760:1396; CameraPan=FVector::ZeroVector; bTopView=false; }
void AStabilityController::Reset() { if(auto* B=Lab(GetWorld())) B->ResetRig(); }
void AStabilityController::FrictionInput(bool Tyre,double Delta,bool Repeat)
{
    if(auto* B=Lab(GetWorld()))
    {
        const double Now=GetWorld()->GetTimeSeconds(); // Remains monotonic across rig resets.
        if(Repeat && Now<NextFrictionRepeat) return;
        B->ChangeFriction(Tyre,Delta); NextFrictionRepeat=Now+.15;
    }
}
void AStabilityController::TyreDown() { FrictionInput(true,-.1,false); }
void AStabilityController::TyreUp() { FrictionInput(true,.1,false); }
void AStabilityController::RoadDown() { FrictionInput(false,-.1,false); }
void AStabilityController::RoadUp() { FrictionInput(false,.1,false); }
void AStabilityController::RepeatTyreDown() { FrictionInput(true,-.1,true); }
void AStabilityController::RepeatTyreUp() { FrictionInput(true,.1,true); }
void AStabilityController::RepeatRoadDown() { FrictionInput(false,-.1,true); }
void AStabilityController::RepeatRoadUp() { FrictionInput(false,.1,true); }
void AStabilityController::RoadPreset() { if(auto* B=Lab(GetWorld())) B->ToggleRoadGrip(); }
bool AStabilityController::TestFrictionBindings()
{
    AStabilityLab* B=Lab(GetWorld()); if(!B || !InputComponent) return false;
    const double TyreBefore=B->Settings.TyreMu, RoadBefore=B->Settings.RoadMu;
    TArray<UStaticMeshComponent*> Bodies;for(AWheelActor* W:B->Wheels) Bodies.Append(W->PhysicalBodies());
    TArray<FVector> BeforeI;TArray<double> BeforeMass;for(auto* Body:Bodies) {BeforeI.Add(Body->GetInertiaTensor());BeforeMass.Add(Body->GetMass());}
    bool Pass=true;
    auto Press=[&](const FKey& Key)
    {
        for(const FInputKeyBinding& Binding:InputComponent->KeyBindings)
            if(Binding.Chord.Key==Key && Binding.KeyEvent==IE_Pressed) { Binding.KeyDelegate.Execute(Key); return true; }
        return false;
    };
    const FKey Keys[]={EKeys::One,EKeys::Two,EKeys::Three,EKeys::Four,EKeys::NumPadOne,EKeys::NumPadTwo,EKeys::NumPadThree,EKeys::NumPadFour};
    for(int32 I=0;I<8;++I)
    {
        const bool Tyre=I%4<2; const double Delta=I%2==0?-.1:.1;
        const double Expected=FMath::Clamp((Tyre?B->Settings.TyreMu:B->Settings.RoadMu)+Delta,0.,2.);
        const bool Matched=Press(Keys[I]) && FMath::IsNearlyEqual(Tyre?B->Settings.TyreMu:B->Settings.RoadMu,Expected,1e-5) && B->FrictionMaterialsMatch();
        Pass &= Matched;
        UE_LOG(LogTemp,Display,TEXT("FRICTION_KEY %s %s"),*Keys[I].GetFName().ToString(),Matched?TEXT("PASS"):TEXT("FAIL"));
    }
    Pass &= Press(EKeys::F) && FMath::IsNearlyEqual(B->Settings.RoadMu,.08,1e-5) && B->FrictionMaterialsMatch();
    Pass &= Press(EKeys::F) && FMath::IsNearlyEqual(B->Settings.RoadMu,.8,1e-5) && B->FrictionMaterialsMatch();
    B->ChangeFriction(true,TyreBefore-B->Settings.TyreMu); B->ChangeFriction(false,RoadBefore-B->Settings.RoadMu);
    if(B->Settings.bSwerve)
    {
        const bool NeutralBefore=B->bNeutral, LossBefore=B->bResistanceEnabled, ViewBefore=B->bLoadView;
        Pass &= Press(EKeys::N) && B->bNeutral!=NeutralBefore;
        Pass &= Press(EKeys::N) && B->bNeutral==NeutralBefore;
        Pass &= Press(EKeys::B) && B->bResistanceEnabled!=LossBefore;
        Pass &= Press(EKeys::B) && B->bResistanceEnabled==LossBefore;
        Pass &= Press(EKeys::H) && B->bLoadView!=ViewBefore;
        Pass &= Press(EKeys::H) && B->bLoadView==ViewBefore;
        UE_LOG(LogTemp,Display,TEXT("RESISTANCE_KEYS %s N/B/H"),Pass?TEXT("PASS"):TEXT("FAIL"));
    }
    double InertiaError=0,MassError=0;
    for(int I=0;I<Bodies.Num();++I)
    {
        InertiaError=FMath::Max(InertiaError,(Bodies[I]->GetInertiaTensor()-BeforeI[I]).Size()/FMath::Max(BeforeI[I].Size(),1e-9));
        MassError=FMath::Max(MassError,FMath::Abs(Bodies[I]->GetMass()-BeforeMass[I]));
    }
    Pass &= InertiaError<1e-6 && MassError<1e-5;
    UE_LOG(LogTemp,Display,TEXT("FRICTION_MASS_INERTIA bodies=%d max_relative_i_error=%.12f max_mass_error_kg=%.12f %s"),Bodies.Num(),InertiaError,MassError,Pass?TEXT("PASS"):TEXT("FAIL"));
    return Pass;
}
void AStabilityController::Stress() { if(auto* B=Lab(GetWorld())) B->CycleStress(); }
void AStabilityController::ToggleView() { bTopView=!bTopView; }
void AStabilityController::ToggleNeutral() { if(auto* B=Lab(GetWorld())) if(B->Settings.bSwerve) B->bNeutral=!B->bNeutral; }
void AStabilityController::ToggleResistance() { if(auto* B=Lab(GetWorld())) if(B->Settings.bSwerve) B->bResistanceEnabled=!B->bResistanceEnabled; }
void AStabilityController::ToggleLoadView() { if(auto* B=Lab(GetWorld())) B->bLoadView=!B->bLoadView; }
void AStabilityController::ToggleMotorView() { if(auto* B=Lab(GetWorld())) { B->bMotorView=!B->bMotorView; B->bTyreView=false; } }
void AStabilityController::ToggleTyreView() { if(auto* B=Lab(GetWorld())) B->bTyreView=!B->bTyreView; }
void AStabilityController::ToggleCameraViews() {if(auto* B=Lab(GetWorld()))B->ToggleCameraViews();}
void AStabilityController::ToggleRearCameraMode() {if(auto* B=Lab(GetWorld()))B->ToggleRearCameraMode();}
void AStabilityController::ToggleHydraulicView() { if(auto* B=Lab(GetWorld())) B->bHydraulicView=!B->bHydraulicView; }
void AStabilityController::NextTerrain() { if(auto* B=Lab(GetWorld())) B->CycleTerrain(); }

void AStabilityHUD::DrawHUD()
{
    Super::DrawHUD(); auto* B=Lab(GetWorld()); if(!Canvas || !B) return;
    const float S=FMath::Min(1.f,FMath::Min(Canvas->SizeX/1100.f,Canvas->SizeY/800.f));
    const FLinearColor Ink(.9f,.95f,1), Dim(.55f,.68f,.8f), Accent(.15f,.9f,.75f), Warn(1,.45f,.15f);
    DrawRect(FLinearColor(.02f,.03f,.05f,.94f),16*S,16*S,395*S,680*S);
    float Y=32;
    auto Line=[&](const FString& T,FLinearColor C,float Size=1.f) { DrawText(T,C,32*S,Y*S,GEngine->GetSmallFont(),S*Size,false); Y+=25; };
    Line(FString::Printf(TEXT("FORKLIFT | %.1f FPS | sim x%.2f"),B->GameFPS(),B->ProgressRatio()),Accent,1.35f); Line(B->Settings.bMecanum?(B->Settings.Mecanum.bReduced?TEXT("MECANUM / EQUIVALENT CONTACT"):TEXT("0.18 / MECANUM | PASSIVE ROLLERS")):B->Settings.Tyres.Tread.bEnabled?FString::Printf(TEXT("0.15 / %s | playback x%.2f"),B->Settings.Tyres.Tread.bCoupled?TEXT("COUPLED RING"):TEXT("TREAD LAB"),GetWorld()->GetWorldSettings()->TimeDilation):B->Settings.bSwerve?TEXT("0.15 / STANDARD TYRE CONTACT LAB"):TEXT("0.3.1 / LEGACY REAR STEER"),Dim);
    if(!B->Error.IsEmpty()) { Line(B->Error,Warn); return; }
    Y+=10;
    Line(FString::Printf(TEXT("Tyre / road mu    %.2f / %.2f"),B->Settings.TyreMu,B->Settings.RoadMu),Ink);
    Line(FString::Printf(TEXT("Contact mu = MIN  %.2f (%s limit)"),B->Settings.EffectiveMu(),B->Settings.TyreMu<B->Settings.RoadMu?TEXT("tyre"):TEXT("road")),Accent);
    Line(FString::Printf(TEXT("Speed             %+.2f m/s  (%.1f km/h)"),B->SpeedMps,FMath::Abs(B->SpeedMps)*3.6),Ink);
    Line(FString::Printf(TEXT("SPACE input %s | axle brake %.0f Nm"),B->Brake>0?TEXT("ON"):TEXT("OFF"),B->Wheels.Num()>0?B->Wheels[0]->BrakeLimitNm:0.),B->Brake>0?Accent:Dim);
    Line(FString::Printf(TEXT("Sideways / accel  %+.2f m/s / %.2f m/s2"),B->LateralMps,B->AccelerationMps2),Ink);
    Line(FString::Printf(TEXT("Yaw rate          %+.1f deg/s"),B->YawDegS),Ink);
    Line(FString::Printf(TEXT("Roll / pitch      %+.1f / %+.1f deg"),B->RollDeg,B->PitchDeg),Ink);
    Line(FString::Printf(TEXT("Tilt / peak       %.1f / %.1f deg"),B->TiltDeg,B->MaxTilt),B->TiltDeg>20?Warn:Ink);
    Line(B->Settings.bMecanum?FString::Printf(TEXT("Contact-loaded wheels %d / 4"),B->Grounded):FString::Printf(TEXT("Near-floor wheels %d / 4  (geometry)"),B->Grounded),B->Grounded<4?Warn:Ink);
    if(!B->Settings.bMecanum) Line(FString::Printf(TEXT("Yaw target / steer slew %+.1f / %.0f deg/s"),FMath::RadiansToDegrees(B->TargetYawRadS),B->Settings.SteerRateDegS),Ink);
    Line(FString::Printf(TEXT("Mass / motor cap  %.0f kg / %.0f Nm"),B->MassKg,B->Settings.DriveNm),Ink);
    Line(FString::Printf(TEXT("Chassis CoM offset +%.2f m"),B->Settings.ChassisComOffsetM),B->Settings.ChassisComOffsetM>0?Warn:Dim);
    Line(FString::Printf(TEXT("Fork target / pos %.2f / %.2f m"),B->LiftTargetM,B->ForkHeightM),Ink);
    const bool LoadView=B->bLoadView && B->Settings.bSwerve;
    Y+=10; Line(LoadView?TEXT("Wheel load kN | roll Nm | bearing Nm (H)"):B->Settings.bMecanum?TEXT("Wheel target m/s | Nm | contact slip (H)"):TEXT("Wheel target/actual deg | Nm | slip m/s (H)"),Dim);
    const TCHAR* Names[]={TEXT("FL"),TEXT("FR"),TEXT("RL"),TEXT("RR")};
    for(int I=0;I<B->Wheels.Num();++I)
    { AWheelActor* W=B->Wheels[I]; Line(LoadView?FString::Printf(TEXT("%s    %.2f     %+.1f     %+.2f"),Names[I],W->NormalLoadFilteredN/1000,W->RollingTorqueNm,W->BearingTorqueNm):B->Settings.bMecanum?FString::Printf(TEXT("%s  %+.2f   %+.0f   %.3f"),Names[I],W->SpeedTargetMps,W->DriveTorqueNm,W->LongSlipMps):FString::Printf(TEXT("%s  %+.1f / %+.1f   %+.0f   %.2f"),Names[I],W->SteerTargetDeg,W->SteerActualDeg,W->DriveTorqueNm,FMath::Sqrt(FMath::Square(W->LongSlipMps)+FMath::Square(W->LateralSlipMps))),W->bGrounded?Ink:Warn); }
    if(B->Wheels.Num()==4)
    {
        double FilteredSum=0; for(AWheelActor* W:B->Wheels) FilteredSum+=W->NormalLoadFilteredN;
        Line(LoadView?FString::Printf(TEXT("Support / dry wt  %.2f / %.2f kN"),FilteredSum/1000,B->MassKg*9.81/1000):FString::Printf(TEXT("Front RPM         %+.1f / %+.1f"),B->Wheels[0]->OmegaRadS*60/(2*PI),B->Wheels[1]->OmegaRadS*60/(2*PI)),Ink);
        Line(LoadView?FString::Printf(TEXT("Crr %.3f | bearing %.2f + %.2f*w | %s"),B->Settings.RollingResistance,B->Settings.BearingCoulombNm,B->Settings.BearingViscousNms,B->bResistanceEnabled?TEXT("ON"):TEXT("OFF")):FString::Printf(TEXT("Rear RPM          %+.1f / %+.1f"),B->Wheels[2]->OmegaRadS*60/(2*PI),B->Wheels[3]->OmegaRadS*60/(2*PI)),B->bResistanceEnabled?Ink:Warn);
    }
    Line(B->Settings.bMecanum?(B->Settings.Mecanum.bReduced?FString::Printf(TEXT("%d visual rollers / wheel | equivalent contact"),B->Settings.Mecanum.Count):FString::Printf(TEXT("%d passive rollers / wheel | lift cap %.0f N"),B->Settings.Mecanum.Count,B->Settings.ForkForceN)):FString::Printf(TEXT("Steer / lift caps %.0f Nm / %.0f N"),B->Settings.SteeringTorqueNm,B->Settings.ForkForceN),Dim);
    Y+=10; Line(B->TimeS<B->FrictionNoticeUntil?B->FrictionNotice:B->Settings.bMecanum?(B->bMecanumAutoHold?TEXT("AUTO HOLD ON | N: neutral / coast"):TEXT("AUTO HOLD OFF | N: neutral / coast")):B->bPayload?TEXT("Contact payload / no sensors / no autonomy"):TEXT("No payload / no sensors / no autonomy"),B->TimeS<B->FrictionNoticeUntil?Accent:Dim);
    Line(B->bNeutral?TEXT("NEUTRAL: drive motors OFF"):B->Settings.bMecanum?FString::Printf(TEXT("MECANUM / %s | %.0f Hz; full energy closure incomplete"),B->Settings.Mecanum.bReduced?TEXT("REDUCED"):TEXT("DETAILED"),1./B->SelectedPhysicsStepS):B->Settings.bSwerve?FString::Printf(TEXT("%s | module direction error %.1f deg"),B->bAligning?TEXT("STEERING / wheel-wise drive"):TEXT("READY / speed control"),B->AlignmentErrorDeg):TEXT("Uncalibrated rear-steer prototype"),B->bNeutral || B->bAligning?Warn:Accent);
    Line(TEXT("N Neutral  B Losses  H Loads  K Hydraulics  P Cameras  O RGB/D"),Accent);
    const float H=Canvas->SizeY/S;
    DrawRect(FLinearColor(.02f,.03f,.05f,.94f),16*S,(H-90)*S,(Canvas->SizeX-32*S),74*S);
    Y=H-80; Line(B->Settings.bSwerve?TEXT("W/S Forward/back  A/D Rotate  Z/C Strafe  SPACE Brake  R Reset  Q/E Lift"):TEXT("W/S Drive  A/D Rear steer  SPACE Brake  R Reset  Q/E Lift"),Ink);
    Line(B->bPayload?TEXT("1/2 Tyre -/+  3/4 Road -/+  F Dry/Ice  V View | Pallet settings: Config/Payload.json"):TEXT("1/2 Tyre -/+  3/4 Road -/+ (numpad OK)  F Dry/Ice  G High-CoM  V View"),Ink);
    Line(TEXT("LMB/RMB drag Orbit | MMB drag Pan | Wheel Zoom | Home Camera reset | V Top view | U Joint-sensor outage"),Dim);
    if(!B->bCameraViews)
    {
        if(B->bHydraulicView) DrawHydraulics(B,S);
        else if(B->bTyreView) DrawTyres(B,S);
        else if(B->bMotorView) DrawMotors(B,S);
        else if(B->bPayload && B->Pallet) DrawPayload(B,S);
    }
    B->DrawCameraViews(this,Canvas);
    if(B->MissionControl){Y=H-177;Line(FString::Printf(TEXT("MISSION: %s | %s"),*B->MissionControl->Model.State,*B->MissionControl->Model.Reason),B->MissionControl->Model.Holding()?Warn:Accent,.8);Line(TEXT("ENTER Start | PgDn/PgUp Tier 1/2 | BACKSPACE Pause | ] Resume | 0 Cancel | Y Ack"),Accent,.8);}
    if(B->HandlingControl){Y=H-132;Line(FString::Printf(TEXT("HANDLING: %s | %s"),*B->HandlingControl->Model.State,*B->HandlingControl->Model.Reason),B->HandlingControl->Model.Faulted?Warn:Accent,.85);Line(TEXT("5 Ground pick | 6/7 Rack low/high pick | 8/9 Rack low/high place | 0 Cancel"),Accent,.8);}
}
