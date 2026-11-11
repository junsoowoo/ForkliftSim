#include "StabilityLab.h"
#include "ChassisActor.h"
#include "WheelActor.h"
#include "ForkActor.h"
#include "Engine/World.h"
#include "ContactLoadSampler.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PBDRigidsSolver.h"
#include "LidarSafety.h"
#include "SensorDrive.h"

void AStabilityLab::SelectMecanumStep()
{
    if(FParse::Param(FCommandLine::Get(),TEXT("MecanumFixedStep")) || FParse::Param(FCommandLine::Get(),TEXT("MecanumLegacyIntegration"))) return;
    double Omega=0;
    auto Sample=[&](AWheelActor* W) {if(W) for(UStaticMeshComponent* B:W->PhysicalBodies()) Omega=FMath::Max(Omega,B->GetPhysicsAngularVelocityInRadians().Size());};
    for(AWheelActor* W:Wheels) Sample(W);Sample(ReactionWheel);
    const int Hz=FMath::Max(Settings.Mecanum.MinimumPhysicsHz,Settings.Mecanum.bAdaptiveStep?(Omega>35?1920:Omega>15?960:480):0);
    SelectedPhysicsStepS=FMath::Min(BasePhysicsStepS,1./Hz);
    SmallestPhysicsStepS=FMath::Min(SmallestPhysicsStepS,SelectedPhysicsStepS);
    // PrePhysics runs after World SetUpForFrame and before StartFrame. Keep
    // the full bounded world interval when selecting a smaller inner step.
    auto* RigidSolver=GetWorld()->GetPhysicsScene()->GetSolver();
    RigidSolver->SetMaxDeltaTime_External(SelectedPhysicsStepS);
    RigidSolver->SetMaxSubSteps_External(FMath::Max(128,FMath::CeilToInt(Settings.Mecanum.MaxControlStepS/SelectedPhysicsStepS)+1));
}

void AStabilityLab::SetMecanumIterations(AActor* Actor)
{
    if(!Settings.bMecanum || !Actor) return;
    int Pos=Settings.Mecanum.PositionIterations,Vel=Settings.Mecanum.VelocityIterations;
    FParse::Value(FCommandLine::Get(),TEXT("MecanumPositionIterations="),Pos);FParse::Value(FCommandLine::Get(),TEXT("MecanumVelocityIterations="),Vel);
    TArray<UPrimitiveComponent*> Components;Actor->GetComponents(Components);
    for(auto* B:Components) if(B->IsSimulatingPhysics()) {B->BodyInstance.SetPositionSolverIterationCount(FMath::Clamp(Pos,8,127));B->BodyInstance.SetVelocitySolverIterationCount(FMath::Clamp(Vel,8,127));}
}

void AStabilityLab::DriveMecanum(double Dt)
{
    if(bSensorDriveRequested)
    {
        if(DriveSensors){DriveSensors->Drive(*this,Dt,LoadSampler);return;}
        // Configuration failure cannot fall through into legacy GT feedback.
        if(LoadSampler){auto* In=LoadSampler->GetProducerInputData_External();In->bMecanumPowered=false;In->bMecanumSensorTorque=true;for(int I=0;I<4;++I){In->MecanumSensorTorqueNm[I]=0;In->ReducedServiceBrake[I]=true;}}
        for(AWheelActor* W:Wheels)W->SetBrakeLimit(Settings.BrakeNm);return;
    }
    const bool StopCommand=FMath::Abs(Throttle)<.01 && FMath::Abs(Strafe)<.01 && FMath::Abs(Steering)<.01;
    if(!StopCommand || bNeutral) bMecanumAutoHold=false;
    else if(TargetVelocity.Size()<.005 && FMath::Abs(TargetYawRadS)<.005 && FVector2D(SpeedMps,LateralMps).Size()<.03 && FMath::Abs(YawDegS)<1) bMecanumAutoHold=true;
    const bool GuardHold=bVehicleSafetyRequested&&(!VehicleGuard||VehicleGuard->Decision.DriveHold);
    const double EffectiveBrake=FMath::Max(Brake,(bMecanumAutoHold||GuardHold)?1.:0.);
    auto* Input=LoadSampler?LoadSampler->GetProducerInputData_External():nullptr;
    if(Input)
    {
        Input->bMecanumControl=true;Input->bMecanumPowered=!bNeutral && EffectiveBrake<=0 && TiltDeg<70;
        Input->bMecanumDetailedMotor=Settings.Actuators.bEnabled;Input->bMecanumResistance=bResistanceEnabled && !FParse::Param(FCommandLine::Get(),TEXT("MecanumNoRolling"));Input->MecanumMotor=Settings.Actuators.Drive;
        for(int I=0;I<4;++I)Input->ReducedServiceBrake[I]=EffectiveBrake>0;
        Input->MecanumRadiusM=Settings.RadiusM;Input->MecanumGain=Settings.WheelSpeedGain;Input->MecanumTorqueLimitNm=Settings.DriveNm;
        Input->MecanumCrr=Settings.RollingResistance;
        for(int I=0;I<4;++I) Input->MecanumLoadN[I]=Wheels[I]->bGrounded?Wheels[I]->NormalLoadFilteredN:0;
    }
    if(TestMode==TEXT("mecanum_reaction") && TimeS>2)
    {
        // Dedicated free-floating motor bench. The forklift's live hydraulic
        // mast is outside this isolated angular-momentum boundary.
        if(!bReactionIsolated)
        {
            ReactionChassis=GetWorld()->SpawnActor<AChassisActor>(FVector(5000,0,1000),FRotator::ZeroRotator);
            ReactionChassis->Configure(Settings);ReactionChassis->SetActorHiddenInGame(true);
            ReactionOriginM=ReactionChassis->Body->GetCenterOfMass()/100.;
            ReactionWheel=GetWorld()->SpawnActor<AWheelActor>(ReactionOriginM*100.,FRotator::ZeroRotator);
            ReactionWheel->Configure(ReactionChassis->Body,Settings,false,TyreMaterial,0);ReactionWheel->SetActorHiddenInGame(true);
            SetMecanumIterations(ReactionChassis);SetMecanumIterations(ReactionWheel);
            if(FParse::Param(FCommandLine::Get(),TEXT("MecanumDetachedRollers"))) for(UPhysicsConstraintComponent* J:ReactionWheel->RollerJoints) J->BreakConstraint();
        }
        TArray<UStaticMeshComponent*> Bodies={ReactionChassis->Body.Get()};Bodies.Append(ReactionWheel->PhysicalBodies());
        FVector L=FVector::ZeroVector;
        for(auto* B:Bodies)
        {
            if(!bReactionIsolated) {B->SetEnableGravity(false);B->SetCollisionResponseToAllChannels(ECR_Ignore);B->SetLinearDamping(0);B->SetAngularDamping(0);B->BodyInstance.SetGyroscopicTorqueEnabled(!FParse::Param(FCommandLine::Get(),TEXT("MecanumNoGyro")));B->BodyInstance.SetMaxAngularVelocityInRadians(2000,false);}
            const FQuat Q=B->GetComponentQuat()*B->BodyInstance.GetMassSpaceLocal().GetRotation();
            const FVector Angular=Q.UnrotateVector(B->GetPhysicsAngularVelocityInRadians()),I=B->GetInertiaTensor()/10000.;
            const FVector Spin=Q.RotateVector(I*Angular),Orbit=FVector::CrossProduct(B->GetCenterOfMass()/100.-ReactionOriginM,B->GetPhysicsLinearVelocity()/100.*B->GetMass());
            L+=Spin+Orbit;
            if((TimeS>2.98 && TimeS<3) || (TimeS>11.98 && TimeS<12))
                UE_LOG(LogTemp,Display,TEXT("REACTION_BODY t=%.4f name=%s mass=%.4f I=%s w=%s spin_y=%.8f orbit_y=%.8f cap=%.4f"),TimeS,*B->GetName(),B->GetMass(),*I.ToString(),*B->GetPhysicsAngularVelocityInRadians().ToString(),Spin.Y,Orbit.Y,B->BodyInstance.GetMaxAngularVelocityInRadians());
        }
        if(!bReactionIsolated) {bReactionIsolated=true;ReactionInitialL=L;}
        ReactionError=FMath::Max(ReactionError,(L-ReactionInitialL).Size());ReactionPitch=FMath::Max(ReactionPitch,FMath::Abs(ReactionChassis->GetActorRotation().Pitch));
        const double Torque=TimeS<3?50:0;ReactionImpulse+=Torque*Dt;
        ReactionWheel->Drive(Torque,0,0);
        if(Input) {Input->bMecanumPowered=false;for(double& V:Input->MecanumTargetMps) V=0;}
        for(AWheelActor* W:Wheels) {W->SetBrakeLimit(Settings.BrakeNm);}
        return;
    }
    bAligning=bLateralReorienting=false;AlignmentErrorDeg=0;
    if(bNeutral || EffectiveBrake>0)
    {
        TargetVelocity=FVector2D::ZeroVector;TargetYawRadS=AllocatedYawRadS=CoordinatedYawRadS=0;
        MecanumVelocityIntegral=FVector2D::ZeroVector;
        TargetHeadingDeg=Chassis->GetActorRotation().Yaw;WheelSpeedScale=1;
        if(Input) for(double& V:Input->MecanumTargetMps) V=0;
        for(AWheelActor* W:Wheels) {W->SpeedTargetMps=W->SteerTargetDeg=0;W->SetBrakeLimit(EffectiveBrake*Settings.BrakeNm);}
        return;
    }
    const bool Creep=VehicleGuard&&VehicleGuard->Decision.State==TEXT("manual_creep");
    FVector2D Desired(Throttle,Strafe);if(Desired.Size()>1) Desired.Normalize();Desired*=VehicleGuard?FMath::Min(Settings.MaxSpeedMps,Creep?.15:VehicleGuard->Sensor.Spec.MaxSpeed):Settings.MaxSpeedMps;
    FVector2D Delta=Desired-TargetVelocity;const double Step=Settings.AccelerationLimitMps2*Dt;
    if(Delta.Size()>Step) Delta=Delta.GetSafeNormal()*Step;TargetVelocity+=Delta;
    const double YawLimit=FMath::DegreesToRadians(Settings.YawRateDegS);
    const double YawDesired=Steering*(VehicleGuard?FMath::Min(YawLimit,Creep?.15:VehicleGuard->Sensor.Spec.MaxYaw):YawLimit);
    const double YawStep=Step/((Settings.WheelbaseM+Settings.TrackM)*.5);
    TargetYawRadS+=FMath::Clamp(YawDesired-TargetYawRadS,-YawStep,YawStep);
    TargetHeadingDeg=FMath::UnwindDegrees(TargetHeadingDeg+FMath::RadiansToDegrees(TargetYawRadS)*Dt);
    const double HeadingError=FMath::DegreesToRadians(FMath::FindDeltaAngleDegrees(Chassis->GetActorRotation().Yaw,TargetHeadingDeg));
    CoordinatedYawRadS=TargetYawRadS+FMath::Clamp(HeadingError*(Settings.bReferenceGeometry?2.:.8),-.3,.3);
    const FVector2D VelocityError=TargetVelocity-FVector2D(SpeedMps,LateralMps);
    const auto& C=Settings.Mecanum;
    if(WheelSpeedScale>.999 && TiltDeg<20) MecanumVelocityIntegral+=VelocityError*(C.VelocityKi*Dt);
    if(MecanumVelocityIntegral.Size()>C.MaxCorrectionMps) MecanumVelocityIntegral=MecanumVelocityIntegral.GetSafeNormal()*C.MaxCorrectionMps;
    FVector2D Correction=VelocityError*C.VelocityKp+MecanumVelocityIntegral;
    if(Correction.Size()>C.MaxCorrectionMps) Correction=Correction.GetSafeNormal()*C.MaxCorrectionMps;
    double Speeds[4];Mecanum::Solve(TargetVelocity+Correction,CoordinatedYawRadS,Settings.WheelbaseM,Settings.TrackM,Speeds);
    const double MotorSurfaceLimit=.98*Settings.Actuators.Drive.MaxRpm*PI/30/Settings.Actuators.Drive.Ratio*Settings.RadiusM;
    WheelSpeedScale=Mecanum::Desaturate(Speeds,FMath::Min(Settings.WheelSurfaceSpeedLimitMps,MotorSurfaceLimit));AllocatedYawRadS=CoordinatedYawRadS*WheelSpeedScale;
    // Prevent the position target from accumulating unreachable yaw during saturation.
    if(WheelSpeedScale<1) TargetHeadingDeg=FMath::UnwindDegrees(TargetHeadingDeg-FMath::RadiansToDegrees(TargetYawRadS*(1-WheelSpeedScale))*Dt);
    for(int I=0;I<4;++I)
    {
        auto* W=Wheels[I].Get();W->SpeedTargetMps=Speeds[I];W->SteerTargetDeg=0;
        W->SetBrakeLimit(0);
        if(Input) Input->MecanumTargetMps[I]=Speeds[I];
    }
}

void AStabilityLab::ControlMecanumTest()
{
    if(IsMissionTest()){ControlMissionTest();return;}
    if(IsHandlingTest()){ControlHandlingTest();return;}
    if(IsPalletPerceptionTest()){ControlPalletPerceptionTest();return;}
    if(IsSensorDriveTest()){ControlSensorDriveTest();return;}
    if(IsVehicleGuardTest()){ControlVehicleGuardTest();return;}
    if(IsSensorLiftTest()){ControlSensorLiftTest();return;}
    if(TestMode==TEXT("mecanum_joint_sensors"))
    {
        Throttle=Strafe=Steering=Lift=0;Brake=1;LiftTargetM=TimeS>=2 && TimeS<28?Settings.ForkTravelM:0;
        if(TimeS>=54 && TimeS<58){Throttle=1;Brake=0;}
        if(TimeS>=58 && TimeS<62){Throttle=-1;Brake=0;}
        if(TimeS>=62 && TimeS<64){Strafe=1;Brake=0;}
        if(TimeS>=64 && TimeS<66){Steering=.4;Brake=0;}
        return;
    }
    if(IsRackTest()){ControlRackTest();return;}
    Throttle=Strafe=Steering=Lift=Brake=0;bNeutral=false;
    const bool Loaded=bPayload;const double Start=Loaded?14:2;
    if(Loaded) LiftTargetM=TimeS>2?(TestMode==TEXT("mecanum_high_carry")?.9:.45):0;
    if(TestMode==TEXT("mecanum_lower") && TimeS>18) LiftTargetM=0;
    if(TimeS<Start) {Brake=1;return;}
    if(TestMode==TEXT("mecanum_settle") || TestMode.EndsWith(TEXT("lift")) || TestMode==TEXT("mecanum_lower")) {Brake=1;return;}
    if(TestMode==TEXT("mecanum_straight") || TestMode==TEXT("mecanum_zero_grip")) Throttle=1;
    if(TestMode==TEXT("mecanum_reverse")) Throttle=-1;
    if(TestMode==TEXT("mecanum_friction_keys") && !bFrictionKeysPassed && TimeS>3)
        if(auto* PC=Cast<AStabilityController>(GetWorld()->GetFirstPlayerController())) bFrictionKeysPassed=PC->TestFrictionBindings();
    if(TestMode==TEXT("mecanum_grip_low") || TestMode==TEXT("mecanum_grip_normal")) {Throttle=TimeS<6?1:TimeS<9?-1:0;Brake=TimeS>=9?1:0;}
    if(TestMode==TEXT("mecanum_strafe")) Strafe=1;
    if(TestMode==TEXT("mecanum_strafe_left")) Strafe=-1;
    if(TestMode==TEXT("mecanum_diagonal")) Throttle=Strafe=.7;
    if(TestMode==TEXT("mecanum_spin")) Steering=.6;
    if(TestMode==TEXT("mecanum_curve")) {Throttle=.6;Steering=TimeS>5?.4:0;}
    if(TestMode==TEXT("mecanum_brake")) {Throttle=TimeS<7?1:0;Brake=TimeS>=7?1:0;}
    if(TestMode==TEXT("mecanum_coast"))
    {
        Throttle=TimeS<5?1:0;bNeutral=TimeS>=5;
        auto& T=ValidationState;
        if(bNeutral)
        {
            if(T.CoastStartS==0) {T.CoastStartS=TimeS;T.CoastStartJ=TyreMechanicalEnergyJ();T.CoastStartMps=SpeedMps;}
            T.CoastDeltaJ=TyreMechanicalEnergyJ()-T.CoastStartJ;
            T.CoastPeakGainJ=FMath::Max(T.CoastPeakGainJ,T.CoastDeltaJ);
        }
    }
    if(TestMode.EndsWith(TEXT("carry"))) {if(TimeS<26){Throttle=.6;Strafe=TimeS>19?.3:0;Steering=TimeS>22?.2:0;}else Brake=1;}
    if(TestMode==TEXT("mecanum_step") || TestMode==TEXT("mecanum_halfstep")) {if(Chassis->GetActorLocation().X<900) Throttle=.7;else Brake=1;}
    if(TestMode.StartsWith(TEXT("mecanum_cargo_")))
    {
        // Same commanded acceleration; only cargo/deck friction differs.
        if(TimeS<24) {Throttle=1;Strafe=TimeS>18?1:0;Steering=TimeS>21?.5:0;}else Brake=1;
    }
    if(TestMode==TEXT("mecanum_ramp_brake"))
    {
        auto& T=ValidationState;const double X=Chassis->GetActorLocation().X/100.;
        if(T.Stage==0 && X>8) {T.Stage=1;T.PlateauAtS=TimeS;T.BrakeStartX=X;}
        if(T.Stage==0) Throttle=.9;else Brake=1;
        if(T.Stage==1) T.StopDistanceM=FMath::Max(T.StopDistanceM,FMath::Abs(X-T.BrakeStartX));
        if(T.Stage==1 && TimeS>T.PlateauAtS+3){if(T.HoldSampleS==0) T.HoldStartX=X;T.HoldSampleS+=GetWorld()->GetDeltaSeconds();T.HoldDriftM=FMath::Max(T.HoldDriftM,FMath::Abs(X-T.HoldStartX));}
    }
}

bool AStabilityLab::CheckMecanumTest() const
{
    if(IsMissionTest())return CheckMissionTest();
    if(IsHandlingTest())return CheckHandlingTest();
    if(IsPalletPerceptionTest())return CheckPalletPerceptionTest();
    if(IsSensorDriveTest())return CheckSensorDriveTest();
    if(IsVehicleGuardTest())return CheckVehicleGuardTest();
    if(IsSensorLiftTest())return CheckSensorLiftTest();
    if(IsRackTest())return CheckRackTest();
    const FVector D=(Chassis->GetActorLocation()-SwerveStart)/100.;bool Pass=Settings.bMecanum;
    double Closure=0;int ContactRollers=0;double RollerPeak=0;
    for(AWheelActor* W:Wheels)
    {
        double Sum=W->bReducedMecanum?W->NormalLoadN:0;for(int I=0;I<W->Rollers.Num();++I){Sum+=W->RollerNormalN[I];ContactRollers+=W->RollerNormalN[I]>1?1:0;RollerPeak=FMath::Max(RollerPeak,FMath::Abs(W->RollerOmega[I]));}
        const double PointN=PreviousPhysicsDt>0?W->ContactData.NormalNs/PreviousPhysicsDt:0;
        Closure=FMath::Max(Closure,FMath::Abs(Sum-PointN));
        Pass &= !W->bSteerable && W->Rollers.Num()==(W->bReducedMecanum?0:Settings.Mecanum.Count) && W->TreadPads.IsEmpty() && FMath::Abs(W->SteerActualDeg)<.5;
        for(double E:W->RollerAnchorErrorM) Pass &= FMath::IsFinite(E) && E<.003;
    }
    Pass &= Closure<1e-5;
    bool Known=true;
    if(TestMode==TEXT("mecanum_joint_sensors"))return Pass && MaxInnerClockErrorS<1e-7 && MaxTilt<10 && ForkHeightM<.05 && MaxForkHeight>Settings.ForkTravelM-.02 && MaxSpeed>.5;
    if(TestMode==TEXT("mecanum_settle")) Pass &= Grounded==4 && MaxSpeed<.1 && MaxSideSpeed<.1 && FMath::Abs(TotalNormalLoadN/MassKg/9.81-1)<.05;
    else if(TestMode==TEXT("mecanum_idle"))
    {
        Pass &= Grounded==4 && MaxSpeed<.05 && MaxSideSpeed<.05 && D.Size2D()<.01;
        for(AWheelActor* W:Wheels) Pass &= FMath::Abs(W->OmegaRadS)*Settings.RadiusM<.005 && W->LongSlipMps<.01 && W->LateralSlipMps<.01;
    }
    else if(TestMode==TEXT("mecanum_friction_keys")) Pass &= bFrictionKeysPassed && FrictionMaterialsMatch();
    else if(TestMode==TEXT("mecanum_straight")) Pass &= SpeedMps>.85 && SpeedMps<1.15 && FMath::Abs(D.Y)<.3 && MaxYawRate<5;
    else if(TestMode==TEXT("mecanum_reverse")) Pass &= SpeedMps<-.85 && SpeedMps>-1.15 && FMath::Abs(D.Y)<.3 && MaxYawRate<5;
    else if(TestMode==TEXT("mecanum_strafe") || TestMode==TEXT("mecanum_strafe_left")) Pass &= LateralMps*(TestMode.EndsWith(TEXT("left"))?-1:1)>.85 && FMath::Abs(LateralMps)<1.15 && FMath::Abs(D.X)<.3 && MaxYawRate<5 && (Settings.Mecanum.bReduced || RollerPeak>1);
    else if(TestMode==TEXT("mecanum_diagonal")) Pass &= SpeedMps>.6 && LateralMps>.6 && SpeedMps<.85 && LateralMps<.85 && FMath::Abs(D.X-D.Y)<.4 && MaxYawRate<5;
    else if(TestMode==TEXT("mecanum_spin")) Pass &= YawDegS>15 && D.Size2D()<.5;
    else if(TestMode==TEXT("mecanum_curve")) Pass &= SpeedMps>.45 && YawDegS>8;
    else if(TestMode==TEXT("mecanum_brake")) Pass &= MaxSpeed>.85 && FMath::Abs(SpeedMps)<.1 && FMath::Abs(LateralMps)<.1;
    else if(TestMode==TEXT("mecanum_coast"))
    {
        const auto& T=ValidationState;const double Kinetic=.5*MassKg*T.CoastStartMps*T.CoastStartMps;
        Pass &= T.CoastStartMps>.8 && TimeS-T.CoastStartS>5 && FMath::Abs(SpeedMps)<.1 && T.CoastPeakGainJ<Kinetic*.02 && T.CoastDeltaJ<-.7*Kinetic;
        UE_LOG(LogTemp,Display,TEXT("MECANUM_COAST start_mps=%.6f end_mps=%.6f delta_energy_j=%.6f peak_gain_j=%.6f initial_translational_j=%.6f"),T.CoastStartMps,SpeedMps,T.CoastDeltaJ,T.CoastPeakGainJ,Kinetic);
    }
    else if(TestMode==TEXT("mecanum_zero_grip")) Pass &= MaxSpeed<.1 && MaxSideSpeed<.1 && MaxWheelSurfaceSpeed>.8;
    else if(TestMode==TEXT("mecanum_grip_low") || TestMode==TEXT("mecanum_grip_normal")) Pass &= MaxSpeed>.5 && FMath::Abs(SpeedMps)<.1;
    else if(TestMode==TEXT("mecanum_reaction")) Pass &= bReactionIsolated && ReactionPitch>.05 && ReactionImpulse>40 && ReactionError<ReactionImpulse*.02 && bReactionCoastSample && ReactionCoastPeakJ<ReactionCoastStartJ*.02 && FMath::Abs(ReactionCoastLastJ-ReactionCoastStartJ)<ReactionCoastStartJ*.02;
    else if(bPayload)
    {
        Known=TestMode.EndsWith(TEXT("lift")) || TestMode.EndsWith(TEXT("carry")) || TestMode==TEXT("mecanum_step") || TestMode==TEXT("mecanum_halfstep") || TestMode==TEXT("mecanum_ramp_brake") || TestMode==TEXT("mecanum_lower") || TestMode.StartsWith(TEXT("mecanum_cargo_"));
        if(TestMode==TEXT("mecanum_lower")) Pass &= ForkHeightM<.05 && PayloadState.GroundN>100 && !CargoState.bFell;
        else if(TestMode==TEXT("mecanum_cargo_drop")) Pass &= CargoState.bFell && CargoState.MaxShiftM>.2;
        else if(TestMode==TEXT("mecanum_cargo_slide")) Pass &= ForkHeightM>.35 && CargoState.MaxShiftM>.1;
        else Pass &= ForkHeightM>.35 && PayloadState.HeightM>.15 && PayloadState.GroundN<10 && !CargoState.bFell && !CargoState.bTipped && !CargoState.bLostSupport;
        if(TestMode==TEXT("mecanum_high_carry")) Pass &= ForkHeightM>.8;
        for(AForkTineActor* T:Fork->Tines) Pass &= !T->bFailed && FMath::Abs(T->TipDeflectionM)<.03;
        if(TestMode.EndsWith(TEXT("carry"))) Pass &= D.Size2D()>3 && MaxSideSpeed>.1 && FMath::Abs(SpeedMps)<.1;
        if(TestMode==TEXT("mecanum_step") || TestMode==TEXT("mecanum_halfstep")) Pass &= Chassis->GetActorLocation().X>880 && MaxTilt<10;
        if(TestMode==TEXT("mecanum_ramp_brake")) Pass &= ValidationState.HoldSampleS>10 && ValidationState.HoldDriftM<.03 && ValidationState.StopDistanceM<.5 && FMath::Abs(PitchDeg)>4;
    }
    else Known=false;
    if(TestMode==TEXT("mecanum_reaction") && ReactionChassis && ReactionWheel)
    {
        const FVector Axis=ReactionWheel->Knuckle->GetRightVector();
        const double RootRate=FVector::DotProduct(ReactionChassis->Body->GetPhysicsAngularVelocityInRadians(),Axis);
        const double RimRate=FVector::DotProduct(ReactionWheel->Tyre->GetPhysicsAngularVelocityInRadians(),Axis);
        Pass &= RootRate<-.0001 && RimRate>.001;
        UE_LOG(LogTemp,Display,TEXT("MECANUM_REACTION root_rad_s=%.8f rim_rad_s=%.8f delta_l_nms=%.8f input_impulse_nms=%.8f physics_thread_delta_l_nms=%.8f coast_start_j=%.8f coast_delta_j=%.8f coast_peak_gain_j=%.8f predictor_work_j=%.8f"),RootRate,RimRate,ReactionError,ReactionImpulse,ReactionPtError,ReactionCoastStartJ,ReactionCoastLastJ-ReactionCoastStartJ,ReactionCoastPeakJ,ReactionPredictorWorkJ);
    }
    Pass &= Known && MaxInnerClockErrorS<1e-7;
    if(TestMode==TEXT("mecanum_brake") || TestMode==TEXT("mecanum_ramp_brake"))
    {
        const double Work=EnergyAudit.ShaftJointJ-EnergyAudit.ReferenceShaftJointJ;
        Pass &= Work<-10 && EnergyAudit.ShaftLossJ>-1;
        UE_LOG(LogTemp,Display,TEXT("MECANUM_BRAKE_ENERGY joint_work_j=%.9f shaft_loss_j=%.9f passive=%d"),Work,EnergyAudit.ShaftLossJ,Work<-10 && EnergyAudit.ShaftLossJ>-1?1:0);
    }
    UE_LOG(LogTemp,Display,TEXT("MECANUM_CHECK %s delta_m=%.4f/%.4f speed_mps=%.4f/%.4f yaw_deg_s=%.4f roller_peak_rad_s=%.4f contacts=%d closure_n=%.9f normal_n=%.3f fork_m=%.5f cargo_fell=%d reaction_pitch_deg=%.6f reaction_delta_l_nms=%.6f reaction_impulse_nms=%.6f stop_distance_m=%.6f hold_drift_m=%.6f %s"),*TestMode,D.X,D.Y,SpeedMps,LateralMps,YawDegS,RollerPeak,ContactRollers,Closure,TotalNormalLoadN,ForkHeightM,CargoState.bFell?1:0,ReactionPitch,ReactionError,ReactionImpulse,ValidationState.StopDistanceM,ValidationState.HoldDriftM,Pass?TEXT("PASS"):TEXT("FAIL"));
    UE_LOG(LogTemp,Display,TEXT("MECANUM_NUMERICS base_step_s=%.9f smallest_selected_s=%.9f smallest_observed_s=%.9f inner_clock_error_s=%.12f"),BasePhysicsStepS,SmallestPhysicsStepS,SmallestObservedPhysicsStepS,MaxInnerClockErrorS);
    return Pass;
}

FString AStabilityLab::MecanumCsv(bool Header) const
{
    FString S=Header?TEXT(",mecanum,actual_realtime_ratio,physics_clock_lag_s,auto_hold"):FString::Printf(TEXT(",%d,%.6f,%.9f,%d"),Settings.bMecanum?1:0,RealtimeRatio,TimeS-ValidationState.PhysicsSampleS,bMecanumAutoHold?1:0);
    if(Settings.bMecanum) for(int I=0;I<Wheels.Num();++I)
    {
        const TCHAR* N=I==0?TEXT("FL"):I==1?TEXT("FR"):I==2?TEXT("RL"):TEXT("RR");
        S+=Header?FString::Printf(TEXT(",%s_friction_used_n,%s_solver_slip_weight_n"),N,N):FString::Printf(TEXT(",%.6f,%.6f"),PreviousPhysicsDt>0?Wheels[I]->FrictionUsedNs/PreviousPhysicsDt:0,PreviousPhysicsDt>0?Wheels[I]->SolverSlipWeight/PreviousPhysicsDt:0);
    }
    if(!Settings.bMecanum) return S;
    for(int I=0;I<Wheels.Num();++I)
    {
        const auto* W=Wheels[I].Get();const FString N=I==0?TEXT("FL"):I==1?TEXT("FR"):I==2?TEXT("RL"):TEXT("RR");
        if(Header) S+=TEXT(",")+N+TEXT("_roller_bearing_w");else S+=FString::Printf(TEXT(",%.6f"),W->RollerBearingPowerW);
        for(int J=0;J<W->Rollers.Num();++J)
        {
            if(Header) S+=FString::Printf(TEXT(",%s_roller_%02d_normal_n,%s_roller_%02d_omega_rad_s,%s_roller_%02d_bearing_nm"),*N,J,*N,J,*N,J);
            else S+=FString::Printf(TEXT(",%.6f,%.6f,%.6f"),W->RollerNormalN[J],W->RollerOmega[J],W->RollerBearingNm[J]);
        }
    }
    return S;
}
