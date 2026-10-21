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
#include "Engine/StaticMeshActor.h"
#include "EngineUtils.h"
#include "HighResScreenshot.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/Parse.h"
#include "HAL/FileManager.h"
#include "DrawDebugHelpers.h"

namespace { AStabilityLab* Lab(UWorld* W) { for(TActorIterator<AStabilityLab> I(W);I;++I) return *I; return nullptr; } }
AStabilityLab::AStabilityLab() { PrimaryActorTick.bCanEverTick=true; PrimaryActorTick.TickGroup=TG_PrePhysics; }
void AStabilityLab::BeginPlay()
{
    Super::BeginPlay();
    FParse::Value(FCommandLine::Get(),TEXT("StabilityTest="),TestMode);
    if (!Settings.Load(Error))
    {
        UE_LOG(LogTemp,Error,TEXT("STABILITY_CONFIG %s"),*Error);
        if(!TestMode.IsEmpty()) FGenericPlatformMisc::RequestExitWithStatus(false,1);
        return;
    }
    // Preserve the documented 0.3 regression cases; manual and swerve_* use 0.4.
    Settings.bSwerve=TestMode.IsEmpty() || TestMode.StartsWith(TEXT("swerve_"));
    if(FParse::Param(FCommandLine::Get(),TEXT("LegacyRearSteer"))) Settings.bSwerve=false;
    NormalComOffset=Settings.ChassisComOffsetM; NormalMaxSpeed=Settings.MaxSpeedMps;
    NormalSteerRate=Settings.SteerRateDegS; NormalMaxSteer=Settings.MaxSteerDeg;
    InitialRoadMu=Settings.RoadMu;
    bCapture=FParse::Param(FCommandLine::Get(),TEXT("StabilityCapture"));
    FParse::Value(FCommandLine::Get(),TEXT("StabilityCaptureTime="),CaptureTimeS);
    CaptureTimeS=FMath::Clamp(CaptureTimeS,.5,10.);
    if (TestMode==TEXT("low_grip") || TestMode==TEXT("powered_turn_low")) Settings.RoadMu=.08;
    if (TestMode==TEXT("low_tyre")) Settings.TyreMu=.08;
    if (TestMode==TEXT("zero_grip")) Settings.RoadMu=0;
    if (TestMode==TEXT("swerve_zero_grip")) Settings.RoadMu=0;
    if (TestMode==TEXT("swerve_low_grip")) Settings.RoadMu=.08;
    if (TestMode==TEXT("swerve_rollover"))
    {
        Settings.TyreMu=Settings.RoadMu=1.5; Settings.ChassisComOffsetM=1.2;
        Settings.MaxSpeedMps=8; Settings.YawRateDegS=90; Settings.SteerRateDegS=120;
    }
    if (TestMode==TEXT("rollover"))
    {
        Settings.TyreMu=1.5; Settings.RoadMu=1.5; Settings.ChassisComOffsetM=1.2;
        Settings.SteerRateDegS=120; Settings.MaxSteerDeg=35;
    }
    for (TActorIterator<AStaticMeshActor> I(GetWorld());I;++I)
        if (I->ActorHasTag(TEXT("StabilityRoad"))) Road=I->GetStaticMeshComponent();
    ResetRig();
}
void AStabilityLab::ApplyMaterials()
{
    TyreMaterial=Vehicle3D::Material(this,Settings.TyreMu); RoadMaterial=Vehicle3D::Material(this,Settings.RoadMu);
    if (Road) Road->SetPhysMaterialOverride(RoadMaterial);
    for (AWheelActor* W : Wheels) W->SetMu(TyreMaterial);
}
void AStabilityLab::ResetRig()
{
    Flush();
    if (Fork) Fork->Destroy();
    for(AWheelActor* W:Wheels) W->Destroy();
    Wheels.Empty();
    if (Chassis) Chassis->Destroy();
    TimeS=NextRecord=0; NextFlush=5; MaxTilt=MaxLateralSlip=MaxYawRate=MaxSpeed=MaxForkHeight=MaxWheelSurfaceSpeed=0; MinGrounded=4;
    PreviousVelocity=FVector::ZeroVector;
    FrictionNotice.Empty(); FrictionNoticeUntil=0;
    bFrictionSwitched=bFrictionRestored=bFrictionKeysPassed=false; MaxLiveSlip=0;
    MinPoweredTurnSpeed=TNumericLimits<double>::Max();
    Throttle=Brake=Steering=Lift=SteerDeg=LiftTargetM=0; bSeeded=bFinished=bCaptured=false;
    Strafe=TargetYawRadS=TargetHeadingDeg=MaxSideSpeed=MaxModuleError=0; TargetVelocity=FVector2D::ZeroVector;
    bAligning=false; AlignmentErrorDeg=0;
    const double Z=Settings.RadiusM*100+4;
    // Deck underside sits 8 cm above the wheel top at any steering angle.
    Chassis=GetWorld()->SpawnActor<AChassisActor>(FVector(0,0,Z+(Settings.bSwerve?Settings.RadiusM*100+19:36)),FRotator::ZeroRotator);
    Chassis->Configure(Settings);
    ApplyMaterials();
    for(int I=0;I<4;++I)
    {
        const FVector P((I<2?1:-1)*Settings.WheelbaseM*50,(I%2==0?-1:1)*Settings.TrackM*50,Z);
        auto* W=GetWorld()->SpawnActor<AWheelActor>(P,FRotator::ZeroRotator);
        W->Configure(Chassis->Body,Settings,I>=2,TyreMaterial); Wheels.Add(W);
    }
    Fork=GetWorld()->SpawnActor<AForkActor>(FVector(Settings.bSwerve?(Settings.MastXM()+.28)*100:118,0,Z+16),FRotator::ZeroRotator);
    Fork->Configure(Chassis->Body,Settings);
    ForkBaseZ=Fork->GetActorLocation().Z-Chassis->GetActorLocation().Z;
    MassKg=Chassis->Body->GetMass()+Fork->Carriage->GetMass();
    for(AWheelActor* W:Wheels) MassKg+=W->Knuckle->GetMass()+W->Tyre->GetMass();
    Rows={TEXT("time_s,tyre_mu,road_mu,effective_mu,chassis_com_offset_m,speed_mps,lateral_mps,yaw_deg_s,roll_deg,pitch_deg,tilt_deg,near_floor_wheels,steer_command_deg,fork_target_m,fork_actual_m,FL_lateral_slip_mps,FR_lateral_slip_mps,RL_lateral_slip_mps,RR_lateral_slip_mps,FL_clearance_m,FR_clearance_m,RL_clearance_m,RR_clearance_m")};
    const FString Dir=FPaths::ProjectSavedDir()/TEXT("Telemetry"); IFileManager::Get().MakeDirectory(*Dir,true);
    Rows[0]+=TEXT(",acceleration_mps2,world_vx_mps,world_vy_mps,chassis_com_height_m");
    for(const TCHAR* Name:{TEXT("FL"),TEXT("FR"),TEXT("RL"),TEXT("RR")})
        Rows[0]+=FString::Printf(TEXT(",%s_omega_rad_s,%s_long_slip_mps,%s_drive_torque_nm,%s_brake_limit_nm,%s_steer_actual_deg"),Name,Name,Name,Name,Name);
    Rows[0]+=TEXT(",throttle,brake,RL_twist_deg,RR_twist_deg,RL_axle_alignment,RR_axle_alignment,RL_joint_error_m,RR_joint_error_m,FL_awake,RL_awake,RL_chassis_contacts,RR_chassis_contacts");
    Rows[0]+=TEXT(",strafe,target_vx_mps,target_vy_mps,target_yaw_rad_s,chassis_x_m,chassis_y_m");
    for(const TCHAR* Name:{TEXT("FL"),TEXT("FR"),TEXT("RL"),TEXT("RR")})
        Rows[0]+=FString::Printf(TEXT(",%s_target_angle_deg,%s_target_speed_mps,%s_self_contacts"),Name,Name,Name);
    CsvPath=Dir/FString::Printf(TEXT("Stability-%s-%s-%s.csv"),TestMode.IsEmpty()?TEXT("manual"):*TestMode,*FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")),*FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(8));
    UE_LOG(LogTemp,Display,TEXT("STABILITY_RIG mass=%.2f chassis=%.2f fork=%.2f com_z=%.3f mu=%.2f swerve=%d"),MassKg,Chassis->Body->GetMass(),Fork->Carriage->GetMass(),Chassis->Body->GetCenterOfMass().Z/100,Settings.EffectiveMu(),Settings.bSwerve?1:0);
}
void AStabilityLab::DriveSwerve(double Dt)
{
    FVector2D Desired(Throttle,Strafe);
    if(Desired.Size()>1) Desired.Normalize();
    Desired*=Settings.MaxSpeedMps;
    const FVector2D Delta=Desired-TargetVelocity;
    const double Step=Settings.AccelerationLimitMps2*Dt;
    TargetVelocity+=Delta.GetSafeNormal()*FMath::Min(Delta.Size(),Step);
    TargetYawRadS=FMath::FInterpConstantTo(TargetYawRadS,FMath::DegreesToRadians(Steering*Settings.YawRateDegS),Dt,FMath::DegreesToRadians(90.));
    TargetHeadingDeg=FMath::UnwindDegrees(TargetHeadingDeg+FMath::RadiansToDegrees(TargetYawRadS)*Dt);
    // Chassis yaw feedback is allocated to wheel targets, never a body torque
    // or transform override. It counters unequal contact loads and steering reaction.
    const double HeadingError=FMath::FindDeltaAngleDegrees(Chassis->GetActorRotation().Yaw,TargetHeadingDeg);
    double ControlledYaw=TargetYawRadS+FMath::Clamp(FMath::DegreesToRadians(.8*HeadingError)+.5*(TargetYawRadS-FMath::DegreesToRadians(YawDegS)),-.5,.5);
    if(FMath::Abs(TargetYawRadS)<1e-5 && FMath::Abs(HeadingError)<.1 && FMath::Abs(YawDegS)<.1) ControlledYaw=0;
    Swerve::FTarget Targets[4]; double Peak=Settings.MaxSpeedMps, WorstAlignment=0;
    for(int I=0;I<4;++I)
    {
        const FVector2D P((I<2?1:-1)*Settings.WheelbaseM*.5,(I%2==0?-1:1)*Settings.TrackM*.5);
        // Optimize against the previous command to avoid random +/-90 choices
        // from tiny contact jitter when beginning a sideways move.
        Targets[I]=Swerve::Solve(P,TargetVelocity,ControlledYaw,Wheels[I]->SteerTargetDeg);
        Peak=FMath::Max(Peak,FMath::Abs(Targets[I].SpeedMps));
        WorstAlignment=FMath::Max(WorstAlignment,FMath::Abs(FMath::FindDeltaAngleDegrees(Wheels[I]->SteerActualDeg,Targets[I].AngleDeg)));
    }
    // Do not accumulate an unreachable heading while the modules are still
    // aligning. Otherwise the first motion tries to catch up with that backlog.
    AlignmentErrorDeg=WorstAlignment; bAligning=WorstAlignment>10;
    if(bAligning || TiltDeg>70) TargetHeadingDeg=Chassis->GetActorRotation().Yaw;
    for(int I=0;I<4;++I)
    {
        AWheelActor* W=Wheels[I]; const auto& T=Targets[I];
        W->SteerTargetDeg=Swerve::Slew(W->SteerTargetDeg,T.AngleDeg,Settings.SteerRateDegS*Dt);
        const double ErrorDeg=FMath::FindDeltaAngleDegrees(W->SteerActualDeg,T.AngleDeg);
        // Reduce rolling demand while a module is pointing across its desired path.
        W->SpeedTargetMps=T.SpeedMps*Settings.MaxSpeedMps/Peak*FMath::Max(0.,FMath::Cos(FMath::DegreesToRadians(ErrorDeg)));
        if(bAligning) W->SpeedTargetMps=0; // Align all modules before applying translation.
        double Torque=FMath::Clamp((W->SpeedTargetMps-W->OmegaRadS*Settings.RadiusM)*Settings.WheelSpeedGain,-Settings.DriveNm,Settings.DriveNm);
        if(Brake>0 || TiltDeg>70) Torque=0;
        W->Drive(Torque,Brake*Settings.BrakeNm,W->SteerTargetDeg);
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
    for(AWheelActor* W:Wheels) if(!Matches(W->Tyre,Settings.TyreMu)) return false;
    return true;
}
void AStabilityLab::CycleStress()
{
    bStress=!bStress;
    Settings.MaxSpeedMps=bStress?8.:NormalMaxSpeed; Settings.ChassisComOffsetM=bStress?1.2:NormalComOffset;
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
        W->Knuckle->SetPhysicsLinearVelocity(V); W->Tyre->SetPhysicsLinearVelocity(V);
        W->Tyre->SetPhysicsAngularVelocityInRadians(W->Knuckle->GetRightVector()*Speed/Settings.RadiusM);
    }
}
void AStabilityLab::Tick(float Dt)
{
    Super::Tick(Dt); if (!Chassis || !Error.IsEmpty() || bFinished) return;
    TimeS+=Dt;
    const FVector V=Chassis->Body->GetPhysicsLinearVelocity()/100;
    AccelerationMps2=Dt>0?(V-PreviousVelocity).Size()/Dt:0; PreviousVelocity=V;
    SpeedMps=FVector::DotProduct(V,Chassis->Body->GetForwardVector());
    LateralMps=FVector::DotProduct(V,Chassis->Body->GetRightVector());
    YawDegS=FMath::RadiansToDegrees(Chassis->Body->GetPhysicsAngularVelocityInRadians().Z);
    const FRotator Rot=Chassis->Body->GetComponentRotation(); RollDeg=Rot.Roll; PitchDeg=Rot.Pitch;
    TiltDeg=FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Chassis->Body->GetUpVector().Z,-1.,1.)));
    ForkHeightM=(Chassis->Body->GetComponentQuat().UnrotateVector(Fork->GetActorLocation()-Chassis->GetActorLocation()).Z-ForkBaseZ)/100;
    Grounded=0;
    for(AWheelActor* W:Wheels)
    {
        W->Measure(Settings.RadiusM,Settings.WidthM); Grounded+=W->bGrounded?1:0;
        MaxWheelSurfaceSpeed=FMath::Max(MaxWheelSurfaceSpeed,FMath::Abs(W->OmegaRadS)*Settings.RadiusM);
        W->SteerActualDeg=FMath::RadiansToDegrees(FMath::Atan2(
            FVector::DotProduct(W->Knuckle->GetForwardVector(),Chassis->Body->GetRightVector()),
            FVector::DotProduct(W->Knuckle->GetForwardVector(),Chassis->Body->GetForwardVector())));
        if(TimeS>2 && W->bGrounded) MaxLateralSlip=FMath::Max(MaxLateralSlip,FMath::Abs(W->LateralSlipMps));
    }
    if(TimeS>2)
    { MaxSideSpeed=FMath::Max(MaxSideSpeed,FMath::Abs(LateralMps)); MaxTilt=FMath::Max(MaxTilt,TiltDeg); MaxYawRate=FMath::Max(MaxYawRate,FMath::Abs(YawDegS)); MaxSpeed=FMath::Max(MaxSpeed,FMath::Abs(SpeedMps)); MinGrounded=FMath::Min(MinGrounded,Grounded); MaxForkHeight=FMath::Max(MaxForkHeight,ForkHeightM); }
    if(!TestMode.IsEmpty())
    {
        if(TimeS>=2 && !bSeeded)
        {
            SettledTilt=TiltDeg; SettledSpeed=V.Size(); bSeeded=true;
            SwerveStart=Chassis->GetActorLocation();
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
        LiftTargetM=FMath::FInterpConstantTo(LiftTargetM,Target,Dt,Settings.ForkSpeedMps);
        if(Settings.bSwerve)
        {
            Throttle=Strafe=Steering=Brake=0;
            if(TimeS>2)
            {
                if(TestMode==TEXT("swerve_straight") || TestMode==TEXT("swerve_zero_grip") || TestMode==TEXT("swerve_low_grip") || TestMode==TEXT("swerve_brake")) Throttle=1;
                if(TestMode==TEXT("swerve_reverse")) Throttle=-1;
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
    SteerDeg=FMath::FInterpConstantTo(SteerDeg,Steering*Settings.MaxSteerDeg,Dt,Settings.SteerRateDegS);
    if(TestMode.StartsWith(TEXT("powered_turn")) && TimeS>6 && Throttle>.9)
        MinPoweredTurnSpeed=FMath::Min(MinPoweredTurnSpeed,SpeedMps);
    LiftTargetM=FMath::Clamp(LiftTargetM+Lift*Settings.ForkSpeedMps*Dt,0.,Settings.ForkTravelM);
    Fork->SetLiftTarget(LiftTargetM);
    if(Settings.bSwerve) DriveSwerve(Dt);
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
    }
    if(TimeS>=NextFlush) { NextFlush+=5; Flush(); }
    if(bCapture && TimeS>CaptureTimeS && !bCaptured)
    {
        bCaptured=true;
        const FString Name=TestMode.IsEmpty()?TEXT("StabilityLab.png"):TEXT("Stability-")+FPaths::MakeValidFileName(TestMode)+TEXT(".png");
        FScreenshotRequest::RequestScreenshot(FPaths::ProjectSavedDir()/TEXT("Screenshots")/Name,true,false);
    }
    if(bCapture && TimeS>CaptureTimeS+1) { Flush(); FGenericPlatformMisc::RequestExit(false); }
    if(!TestMode.IsEmpty() && TimeS>=(TestMode==TEXT("swerve_transition")?18:12)) FinishTest();
}
void AStabilityLab::FinishTest()
{
    bFinished=true; Flush();
    const double ExpectedMass=Settings.ChassisMassKg+Settings.ForkMassKg+4*(Settings.WheelMassKg+Settings.KnuckleMassKg);
    bool Pass=SettledTilt<3 && SettledSpeed<.2 && FMath::Abs(MassKg-ExpectedMass)<1
        && FMath::IsFinite(SpeedMps) && FMath::IsFinite(TiltDeg) && MaxSpeed<8.5 && MaxYawRate<400;
    if(Settings.bSwerve)
    {
        const FVector Displacement=(Chassis->GetActorLocation()-SwerveStart)/100;
        Pass &= FrictionMaterialsMatch();
        if(TestMode!=TEXT("swerve_rollover")) Pass &= MaxTilt<10 && MaxModuleError<8;
        for(AWheelActor* W:Wheels) Pass &= W->ChassisContactCount==0 && FMath::IsFinite(W->OmegaRadS) && FMath::Abs(W->DriveTorqueNm)<=Settings.DriveNm+.01;
        if(TestMode==TEXT("swerve_settle")) Pass &= MaxSpeed<.1 && MaxSideSpeed<.1 && Grounded==4;
        else if(TestMode==TEXT("swerve_straight")) Pass &= SpeedMps>2 && MaxYawRate<5 && FMath::Abs(Displacement.Y)<.5;
        else if(TestMode==TEXT("swerve_reverse")) Pass &= SpeedMps<-2 && MaxYawRate<5;
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
void AStabilityLab::EndPlay(const EEndPlayReason::Type R) { Flush(); Super::EndPlay(R); }

void AStabilityController::BeginPlay()
{
    Super::BeginPlay(); LabCamera=GetWorld()->SpawnActor<ACameraActor>(); LabCamera->GetCameraComponent()->SetFieldOfView(50);
    bTopView=FParse::Param(FCommandLine::Get(),TEXT("SwerveTopView"));
    SetViewTarget(LabCamera); SetInputMode(FInputModeGameOnly()); bShowMouseCursor=false;
}
void AStabilityController::PlayerTick(float Dt)
{
    Super::PlayerTick(Dt); auto* B=Lab(GetWorld()); if(!B || !B->Chassis) return;
    // Inspection view hides only the deck render mesh. Welded parts, collision,
    // mass and all constraints remain active, including across a rig reset.
    B->Chassis->Body->SetVisibility(!bTopView || !B->Settings.bSwerve,false);
    if(B->TestMode.IsEmpty())
    {
        B->Throttle=(IsInputKeyDown(EKeys::W)?1:0)-(IsInputKeyDown(EKeys::S)?1:0);
        B->Steering=(IsInputKeyDown(EKeys::D)?1:0)-(IsInputKeyDown(EKeys::A)?1:0);
        B->Strafe=(IsInputKeyDown(EKeys::C)?1:0)-(IsInputKeyDown(EKeys::Z)?1:0);
        B->Brake=IsInputKeyDown(EKeys::SpaceBar)?1:0;
        B->Lift=(IsInputKeyDown(EKeys::E)?1:0)-(IsInputKeyDown(EKeys::Q)?1:0);
    }
    const FVector Center=B->Chassis->GetActorLocation();
    const FVector P=Center+(bTopView?FVector(0,0,1300):FVector(800,-1050,450));
    LabCamera->SetActorLocation(P); LabCamera->SetActorRotation(bTopView?FRotator(-90,0,0):(Center+FVector(190,140,50)-P).Rotation());
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
}
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
    return Pass;
}
void AStabilityController::Stress() { if(auto* B=Lab(GetWorld())) B->CycleStress(); }
void AStabilityController::ToggleView() { bTopView=!bTopView; }

void AStabilityHUD::DrawHUD()
{
    Super::DrawHUD(); auto* B=Lab(GetWorld()); if(!Canvas || !B) return;
    const float S=FMath::Min(1.f,FMath::Min(Canvas->SizeX/1100.f,Canvas->SizeY/800.f));
    const FLinearColor Ink(.9f,.95f,1), Dim(.55f,.68f,.8f), Accent(.15f,.9f,.75f), Warn(1,.45f,.15f);
    DrawRect(FLinearColor(.02f,.03f,.05f,.94f),16*S,16*S,395*S,680*S);
    float Y=32;
    auto Line=[&](const FString& T,FLinearColor C,float Size=1.f) { DrawText(T,C,32*S,Y*S,GEngine->GetSmallFont(),S*Size,false); Y+=25; };
    Line(TEXT("FORKLIFT / 3D CONTACT LAB"),Accent,1.35f); Line(B->Settings.bSwerve?TEXT("0.4 / FOUR INDEPENDENT STEER + DRIVE"):TEXT("0.3.1 / LEGACY REAR STEER"),Dim);
    if(!B->Error.IsEmpty()) { Line(B->Error,Warn); return; }
    Y+=10;
    Line(FString::Printf(TEXT("Tyre / road mu    %.2f / %.2f"),B->Settings.TyreMu,B->Settings.RoadMu),Ink);
    Line(FString::Printf(TEXT("Contact mu = MIN  %.2f (%s limit)"),B->Settings.EffectiveMu(),B->Settings.TyreMu<B->Settings.RoadMu?TEXT("tyre"):TEXT("road")),Accent);
    Line(FString::Printf(TEXT("Speed             %+.2f m/s  (%.1f km/h)"),B->SpeedMps,FMath::Abs(B->SpeedMps)*3.6),Ink);
    Line(FString::Printf(TEXT("Sideways / accel  %+.2f m/s / %.2f m/s2"),B->LateralMps,B->AccelerationMps2),Ink);
    Line(FString::Printf(TEXT("Yaw rate          %+.1f deg/s"),B->YawDegS),Ink);
    Line(FString::Printf(TEXT("Roll / pitch      %+.1f / %+.1f deg"),B->RollDeg,B->PitchDeg),Ink);
    Line(FString::Printf(TEXT("Tilt / peak       %.1f / %.1f deg"),B->TiltDeg,B->MaxTilt),B->TiltDeg>20?Warn:Ink);
    Line(FString::Printf(TEXT("Near-floor wheels %d / 4  (geometry)"),B->Grounded),B->Grounded<4?Warn:Ink);
    Line(FString::Printf(TEXT("Yaw target / steer slew %+.1f / %.0f deg/s"),FMath::RadiansToDegrees(B->TargetYawRadS),B->Settings.SteerRateDegS),Ink);
    Line(FString::Printf(TEXT("Mass / motor cap  %.0f kg / %.0f Nm"),B->MassKg,B->Settings.DriveNm),Ink);
    Line(FString::Printf(TEXT("Chassis CoM offset +%.2f m"),B->Settings.ChassisComOffsetM),B->Settings.ChassisComOffsetM>0?Warn:Dim);
    Line(FString::Printf(TEXT("Fork target / pos %.2f / %.2f m"),B->LiftTargetM,B->ForkHeightM),Ink);
    Y+=10; Line(TEXT("Wheel target/actual deg | Nm | slip m/s"),Dim);
    const TCHAR* Names[]={TEXT("FL"),TEXT("FR"),TEXT("RL"),TEXT("RR")};
    for(int I=0;I<B->Wheels.Num();++I)
    { AWheelActor* W=B->Wheels[I]; Line(FString::Printf(TEXT("%s  %+.1f / %+.1f   %+.0f   %.2f"),Names[I],W->SteerTargetDeg,W->SteerActualDeg,W->DriveTorqueNm,FMath::Sqrt(FMath::Square(W->LongSlipMps)+FMath::Square(W->LateralSlipMps))),W->bGrounded?Ink:Warn); }
    if(B->Wheels.Num()==4)
    {
        Line(FString::Printf(TEXT("Front RPM         %+.1f / %+.1f"),B->Wheels[0]->OmegaRadS*60/(2*PI),B->Wheels[1]->OmegaRadS*60/(2*PI)),Ink);
        Line(FString::Printf(TEXT("Rear RPM          %+.1f / %+.1f"),B->Wheels[2]->OmegaRadS*60/(2*PI),B->Wheels[3]->OmegaRadS*60/(2*PI)),Ink);
    }
    Line(FString::Printf(TEXT("Steer / lift caps %.0f Nm / %.0f N"),B->Settings.SteeringTorqueNm,B->Settings.ForkForceN),Dim);
    Y+=10; Line(B->TimeS<B->FrictionNoticeUntil?B->FrictionNotice:TEXT("No payload / no sensors / no autonomy"),B->TimeS<B->FrictionNoticeUntil?Accent:Dim);
    Line(B->Settings.bSwerve?FString::Printf(TEXT("%s | module direction error %.1f deg"),B->bAligning?TEXT("ALIGNING / drive paused"):TEXT("READY / speed control"),B->AlignmentErrorDeg):TEXT("Uncalibrated rear-steer prototype"),B->bAligning?Warn:Accent);
    Line(TEXT("G: CoM +1.2 m / fast-steer stress (reset)"),Warn);
    const float H=Canvas->SizeY/S;
    DrawRect(FLinearColor(.02f,.03f,.05f,.94f),16*S,(H-90)*S,(Canvas->SizeX-32*S),74*S);
    Y=H-80; Line(B->Settings.bSwerve?TEXT("W/S Forward/back  A/D Rotate  Z/C Strafe  SPACE Brake  R Reset  Q/E Lift"):TEXT("W/S Drive  A/D Rear steer  SPACE Brake  R Reset  Q/E Lift"),Ink);
    Line(TEXT("1/2 Tyre -/+  3/4 Road -/+ (numpad OK)  F Dry/Ice  G High-CoM  V View"),Ink);
    Line(B->Settings.bSwerve && B->Chassis && !B->Chassis->Body->IsVisible()?TEXT("TOP INSPECTION: deck mesh hidden; all collision and mass still active"):TEXT("Physics ground truth, not sensors | Telemetry: Saved/Telemetry/Stability-*.csv"),Dim);
}
