#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "WheelActor.h"
#include "Engine/StaticMeshActor.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

bool AStabilityLab::IsIntegrationTest() const { return TestMode.StartsWith(TEXT("swerve_integrated_")); }
bool AStabilityLab::ConfigureValidation()
{
    if(!IsIntegrationTest()) return true;
    bool Known=false;
    for(const TCHAR* N:{TEXT("flat"),TEXT("ramp"),TEXT("step"),TEXT("halfstep"),TEXT("split"),TEXT("ramp_eccentric"),TEXT("step_eccentric"),TEXT("ramp_brake"),TEXT("ramp_brake_input"),TEXT("ramp_brake_ice")})
        Known |= TestMode==FString(TEXT("swerve_integrated_"))+N;
    if(!Known) { Error=TEXT("Unknown integrated fixture"); return false; }
    // Frozen virtual reference V1. These are design assumptions, NOT measured
    // specifications of a commercial forklift. Manual JSON tuning is untouched.
    Settings=FVehicle3DSettings(); Settings.MaxSpeedMps=1;
    PayloadSettings=FPayloadSettings(); PayloadSettings.InitialDepthM=1.1;
    if(TestMode.EndsWith(TEXT("eccentric"))) PayloadSettings.CargoYM=.12;
    TerrainPreset=TestMode.Contains(TEXT("ramp"))?TEXT("ramp"):TestMode.EndsWith(TEXT("halfstep"))?TEXT("halfstep"):TestMode.Contains(TEXT("step"))?TEXT("step"):TestMode.EndsWith(TEXT("split"))?TEXT("split"):TEXT("flat");
    bMotorView=true;
    return PayloadSettings.Validate(Error);
}
void AStabilityLab::ControlIntegrationTest()
{
    auto& T=ValidationState;
    Throttle=Strafe=Steering=Lift=Brake=0;
    LiftTargetM=TimeS>2?.45:0; // real fork/hole contact, no weld or teleport
    const double X=Chassis->GetActorLocation().X/100.;
    if(TimeS<10) { Brake=1; bNeutral=true; return; }
    bNeutral=false;
    if(TerrainPreset==TEXT("ramp"))
    {
        if(TestMode.Contains(TEXT("ramp_brake")))
        {
            // Stop with all four wheels ON the 6-degree slope, not on the plateau.
            if(T.Stage==0 && X>8) { T.Stage=3; T.BrakeStartX=X; T.PlateauAtS=TimeS; }
            Throttle=T.Stage==0?.9:0;
            if(T.Stage==3)
            {
                Brake=1;
                // A held accelerator/turn must not defeat the service brake.
                if(TestMode.EndsWith(TEXT("_input"))) Throttle=Steering=1;
                if(TestMode.EndsWith(TEXT("_ice")) && TimeS>T.PlateauAtS+6 && Settings.RoadMu>.02) ChangeFriction(false,.02-Settings.RoadMu);
            }
            return;
        }
        // Full ascent, stop on plateau, reverse descent, stop on staging floor.
        if(T.Stage==0 && X>15.2) { T.Stage=1; T.PlateauAtS=TimeS; }
        if(T.Stage==1 && TimeS-T.PlateauAtS>2) T.Stage=2;
        if(T.Stage==2 && X<3) { T.Stage=3; T.BrakeStartX=X; }
        Throttle=T.Stage==0?.9:T.Stage==2?-.65:0;
    }
    else
    {
        if(T.Stage==0 && X>11) { T.Stage=3; T.BrakeStartX=X; }
        Throttle=T.Stage==0?.8:0;
    }
    if(Throttle==0) { Brake=1; bNeutral=true; }
}
void AStabilityLab::UpdateValidation(double Dt)
{
    auto& T=ValidationState;
    if(TestMode.StartsWith(TEXT("swerve_coast_")))
    {
        if(FParse::Param(FCommandLine::Get(),TEXT("PassiveSteer"))) for(AWheelActor* W:Wheels)
        { W->SteeringJoint->SetOrientationDriveTwistAndSwing(false,false); W->SteeringJoint->SetAngularVelocityDriveTwistAndSwing(false,false); }
        // Exclude the artificial initial velocity injection and its first step.
        if(TimeS>=2.1 && T.CoastStartS==0)
        { T.CoastStartS=TimeS; T.CoastStartJ=TyreMechanicalEnergyJ(); T.CoastStartMps=SpeedMps; }
        if(T.CoastStartS>0)
        { T.CoastDeltaJ=TyreMechanicalEnergyJ()-T.CoastStartJ; T.CoastPeakGainJ=FMath::Max(T.CoastPeakGainJ,T.CoastDeltaJ); }
    }
    if(!IsIntegrationTest()) return;
    const double X=Chassis->GetActorLocation().X/100.;
    if(TestMode.Contains(TEXT("ramp_brake")) && T.Stage==3 && TimeS>T.PlateauAtS+3)
    {
        if(T.HoldSampleS==0) T.HoldStartX=X;
        T.HoldSampleS+=Dt;
        T.HoldDriftM=FMath::Max(T.HoldDriftM,FMath::Abs(X-T.HoldStartX));
        T.HoldMaxSpeedMps=FMath::Max(T.HoldMaxSpeedMps,Chassis->Body->GetPhysicsLinearVelocity().Size()/100.);
    }
    T.PeakX=FMath::Max(T.PeakX,X);
    T.PeakZ=FMath::Max(T.PeakZ,Chassis->GetActorLocation().Z/100.);
    T.bLifted |= TimeS<10 && PayloadState.HeightM>.25 && PayloadState.GroundN<10;
    if(TimeS>7 && TimeS<10 && PreviousPhysicsDt>0)
    {
        const double H=PreviousPhysicsDt;
        T.StaticS+=H; T.NormalIntegral+=TotalNormalLoadN*H;
        T.FrontIntegral+=(Wheels[0]->NormalLoadN+Wheels[1]->NormalLoadN)*H;
        T.LeftIntegral+=(Wheels[0]->NormalLoadN+Wheels[2]->NormalLoadN)*H;
        FVector Sum=FVector::ZeroVector; double Mass=0;
        auto Add=[&](UStaticMeshComponent* B) { Sum+=B->GetCenterOfMass()*B->GetMass(); Mass+=B->GetMass(); };
        Add(Chassis->Body); Add(Fork->Carriage); for(AForkTineActor* Blade:Fork->Tines) for(UStaticMeshComponent* B:Blade->Sections) Add(B);
        if(Fork->MovingMast) Add(Fork->MovingMast);
        for(AWheelActor* W:Wheels) { for(UStaticMeshComponent* B:W->PhysicalBodies()) Add(B); }
        Add(Pallet->Deck); Add(Pallet->Cargo->GetStaticMeshComponent());
        const FVector C=Chassis->Body->GetComponentQuat().UnrotateVector(Sum/Mass-Chassis->GetActorLocation())/100.;
        T.ExpectedFrontIntegral+=(.5+C.X/Settings.WheelbaseM)*H;
        T.ExpectedLeftIntegral+=(.5-C.Y/Settings.TrackM)*H;
        T.PressureIntegral+=Fork->HydraulicState.PressurePa*H;
    }
    if(TimeS>10)
    {
        T.PeakCrossHeightM=FMath::Max(T.PeakCrossHeightM,FMath::Abs(Wheels[0]->Tyre->GetComponentLocation().Z-Wheels[1]->Tyre->GetComponentLocation().Z)/100.);
        T.TravelS+=Dt;
        T.LeftMuIntegral+=(Wheels[0]->ContactMu+Wheels[2]->ContactMu)*.5*Dt;
        T.RightMuIntegral+=(Wheels[1]->ContactMu+Wheels[3]->ContactMu)*.5*Dt;
        for(AWheelActor* W:Wheels)
        {
            T.PeakLoadN=FMath::Max(T.PeakLoadN,W->NormalLoadN);
            T.MinWheelN=FMath::Min(T.MinWheelN,W->NormalLoadN);
            T.PeakSlipMps=FMath::Max(T.PeakSlipMps,FMath::Abs(W->LongSlipMps));
            T.bCourseContact |= W->FloorContacts>0 && W->Tyre->GetComponentLocation().Z/100>Settings.RadiusM+.015;
        }
        if(X>7.5 && X<12.5 && FMath::Abs(PitchDeg)>3)
        { if(SpeedMps>.1) T.UphillS+=Dt; if(SpeedMps<-.1) T.DownhillS+=Dt; }
        if(T.Stage==3 && !T.bStopped && FMath::Abs(SpeedMps)<.03)
        { T.bStopped=true; T.StopDistanceM=FMath::Abs(X-T.BrakeStartX); }
    }
}
bool AStabilityLab::CheckIntegrationTest() const
{
    const auto& T=ValidationState;
    if(TestMode.Contains(TEXT("ramp_brake")))
    {
        bool Pass=T.Stage==3 && T.bLifted && T.HoldSampleS>10 && T.HoldDriftM<.02 && T.HoldMaxSpeedMps<.01;
        Pass &= FMath::Abs(PitchDeg)>4 && FMath::Abs(PitchDeg)<8 && Grounded==4;
        if(TestMode.EndsWith(TEXT("_ice"))) Pass=T.Stage==3 && T.bLifted && T.HoldSampleS>10 && T.HoldDriftM>.2 && T.HoldMaxSpeedMps>.1 && FMath::IsNearlyEqual(Settings.RoadMu,.02);
        Pass &= !CargoState.bFell && !CargoState.bLostSupport && !CargoState.bTipped;
        for(AWheelActor* W:Wheels) Pass &= W->BrakeLimitNm==Settings.BrakeNm && W->DriveTorqueNm==0;
        UE_LOG(LogTemp,Display,TEXT("RAMP_BRAKE_CHECK %s hold_s=%.3f drift_m=%.6f peak_speed_mps=%.6f pitch_deg=%.3f brake=%.1f hold_x=%.4f final_x=%.4f %s"),
            *TestMode,T.HoldSampleS,T.HoldDriftM,T.HoldMaxSpeedMps,PitchDeg,Brake,T.HoldStartX,Chassis->GetActorLocation().X/100.,Pass?TEXT("PASS"):TEXT("FAIL"));
        return Pass;
    }
    const double S=FMath::Max(T.StaticS,1e-8), Weight=(MassKg+Pallet->LoadMass())*9.81;
    const double LoadError=FMath::Abs(T.NormalIntegral/S-Weight)/Weight;
    const double Front=T.FrontIntegral/FMath::Max(T.NormalIntegral,1e-8), Expected=T.ExpectedFrontIntegral/S;
    const double Left=T.LeftIntegral/FMath::Max(T.NormalIntegral,1e-8), ExpectedLeft=T.ExpectedLeftIntegral/S;
    const double Pressure=T.PressureIntegral/S, ExpectedPressure=(Settings.ForkMassKg+Pallet->LoadMass())*9.81/Settings.Hydraulics.AreaM2;
    UE_LOG(LogTemp,Display,TEXT("INTEGRATED_INNER_LOAD peak_n=%.3f min_n=%.3f mean_sample_peak_n=%.3f mean_sample_min_n=%.3f"),T.PeakInnerLoadN,T.MinInnerWheelN,T.PeakLoadN,T.MinWheelN);
    bool Pass=T.StaticS>2.5 && T.bLifted && T.bStopped && LoadError<.03 && FMath::Abs(Front-Expected)<.03 && FMath::Abs(Left-ExpectedLeft)<.03;
    Pass &= FMath::Abs(Pressure-ExpectedPressure)/ExpectedPressure<.08 && PowerState.bLimitsValid && HydraulicLabState.bLimitsValid;
    Pass &= !CargoState.bFell && !CargoState.bLostSupport && !CargoState.bTipped && CargoState.MaxShiftM<.05 && PayloadState.MaxSlip<.05;
    Pass &= !TyreState.bBottomed && Grounded==4 && FMath::Abs(SpeedMps)<.03 && T.StopDistanceM<.6;
    if(TerrainPreset==TEXT("ramp")) Pass &= T.bCourseContact && T.PeakX>15.2 && T.UphillS>3 && T.DownhillS>3 && Chassis->GetActorLocation().X<310;
    else Pass &= T.PeakX>11;
    if(TerrainPreset==TEXT("step")) Pass &= T.bCourseContact;
    if(TerrainPreset==TEXT("halfstep")) Pass &= T.bCourseContact && T.PeakCrossHeightM>.015;
    const double MuL=T.LeftMuIntegral/FMath::Max(T.TravelS,1e-8), MuR=T.RightMuIntegral/FMath::Max(T.TravelS,1e-8);
    if(TerrainPreset==TEXT("split")) Pass &= FMath::Abs(MuL-.08)<.01 && FMath::Abs(MuR-.8)<.01;
    UE_LOG(LogTemp,Display,TEXT("INTEGRATED_CHECK %s load_error=%.6f front_fraction=%.6f expected_front=%.6f left_fraction=%.6f expected_left=%.6f pressure_pa=%.2f expected_pressure_pa=%.2f peak_x_m=%.4f final_x_m=%.4f up_s=%.3f down_s=%.3f stop_m=%.5f cargo_shift_m=%.6f pallet_slip_m=%.6f peak_wheel_n=%.2f min_wheel_n=%.2f peak_slip_mps=%.5f left_mu=%.4f right_mu=%.4f lifted=%d stopped=%d %s"),
        *TestMode,LoadError,Front,Expected,Left,ExpectedLeft,Pressure,ExpectedPressure,T.PeakX,Chassis->GetActorLocation().X/100.,T.UphillS,T.DownhillS,T.StopDistanceM,CargoState.MaxShiftM,PayloadState.MaxSlip,T.PeakLoadN,T.MinWheelN,T.PeakSlipMps,MuL,MuR,T.bLifted?1:0,T.bStopped?1:0,Pass?TEXT("PASS"):TEXT("FAIL"));
    return Pass;
}
FString AStabilityLab::ValidationCsv(bool Header) const
{
    if(Header) return TEXT(",validation_stage,coast_energy_delta_j,coast_peak_gain_j,integrated_peak_wheel_n,integrated_stop_m,integrated_peak_cross_height_m,integrated_inner_peak_wheel_n,integrated_inner_min_wheel_n");
    const auto& T=ValidationState;
    return FString::Printf(TEXT(",%d,%.6f,%.6f,%.3f,%.6f,%.6f,%.3f,%.3f"),T.Stage,T.CoastDeltaJ,T.CoastPeakGainJ,T.PeakLoadN,T.StopDistanceM,T.PeakCrossHeightM,T.PeakInnerLoadN,T.MinInnerWheelN<1e29?T.MinInnerWheelN:0);
}
