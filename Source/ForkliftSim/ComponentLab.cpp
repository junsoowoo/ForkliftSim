#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "WheelActor.h"
#include "PalletLoadActor.h"
#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

void AStabilityLab::ControlComponentTest()
{
    Throttle=Strafe=Steering=Lift=0; Brake=1; bNeutral=false;
    LiftTargetM=TimeS>2?.45:0;
    for(AWheelActor* W:Wheels)
    {
        const auto& C=W->ContactData;
        ValidationState.ContactPeakPoints=FMath::Max(ValidationState.ContactPeakPoints,C.PeakActivePoints);
        ValidationState.ContactBandError=FMath::Max(ValidationState.ContactBandError,FMath::Abs(C.BandNs[0]+C.BandNs[1]+C.BandNs[2]-C.NormalNs));
        if(PreviousPhysicsDt>0) ValidationState.ContactNormalError=FMath::Max(ValidationState.ContactNormalError,FMath::Abs(C.NormalNs/PreviousPhysicsDt-W->NormalLoadN));
        if(TimeS>14)
        {
            ValidationState.ContactPeakLoadN=FMath::Max(ValidationState.ContactPeakLoadN,W->NormalLoadN);
            ValidationState.ContactPeakCompressionM=FMath::Max(ValidationState.ContactPeakCompressionM,W->CompressionM);
        }
        if(!W->TreadPads.IsEmpty())
        {
            double N=0; for(double F:W->TreadNormalN) N+=F;
            const double PointN=PreviousPhysicsDt>0?C.NormalNs/PreviousPhysicsDt:0;
            ValidationState.TreadClosureN=FMath::Max(ValidationState.TreadClosureN,FMath::Abs(PointN-N));
            ValidationState.TreadSpreadM=FMath::Max(ValidationState.TreadSpreadM,W->TreadMaxCompressionM-W->TreadMinCompressionM);
            if(TimeS>20) ValidationState.TreadCompressionIntegral+=W->TreadMaxCompressionM*PreviousPhysicsDt*.25;
        }
    }
    if(TimeS>20) { ValidationState.TreadStaticS+=PreviousPhysicsDt; ValidationState.TreadLoadIntegral+=TotalNormalLoadN*PreviousPhysicsDt; }
    if(TestMode.EndsWith(TEXT("fracture")) || TestMode.EndsWith(TEXT("plastic")))
    {
        // Explicit fixed-root destructive bench, not a free driving condition.
        Chassis->Body->SetSimulatePhysics(false); Fork->Carriage->SetSimulatePhysics(false);
        // Bench mounting height is selected BEFORE joint creation in ResetRig.
        // Moving an articulated assembly after binding its frames would inject
        // an artificial constraint impulse and invalidate the material test.
        const bool Plastic=TestMode.EndsWith(TEXT("plastic"));
        const double Load=Plastic?50000*FMath::Min(FMath::Clamp((TimeS-5)/3,0.,1.),FMath::Clamp((12-TimeS)/2,0.,1.)):
            150000*FMath::Min(FMath::Clamp(TimeS-5,0.,1.),FMath::Clamp(8-TimeS,0.,1.));
        if(Load>0) for(AForkTineActor* T:Fork->Tines) if(!T->bFailed)
            T->Sections.Last()->AddForceAtLocation(FVector(0,0,-Load*100),T->Tip());
        return;
    }
    if(TestMode.EndsWith(TEXT("lower")) && TimeS>18) LiftTargetM=0;
    const double X=Chassis->GetActorLocation().X/100.;
    if(TestMode.EndsWith(TEXT("step")))
    {
        if(TimeS>14 && X>9) ValidationState.Stage=3;
        if(TimeS>14 && ValidationState.Stage==0) { Brake=0; Throttle=.7; }
    }
    else if(TestMode.EndsWith(TEXT("ramp_brake")) || TestMode.EndsWith(TEXT("ramp_ice")))
    {
        auto& T=ValidationState;
        if(TimeS>14 && T.Stage==0 && X>8) { T.Stage=3; T.BrakeStartX=X; T.PlateauAtS=TimeS; }
        if(TimeS>14 && T.Stage==0) { Brake=0; Throttle=.9; }
        if(T.Stage==3 && TimeS>T.PlateauAtS+3 && !bFrictionSwitched)
        {
            if(T.HoldSampleS==0) T.HoldStartX=X;
            T.HoldSampleS+=GetWorld()->GetDeltaSeconds();
            T.HoldDriftM=FMath::Max(T.HoldDriftM,FMath::Abs(X-T.HoldStartX));
            if(TestMode.EndsWith(TEXT("ramp_ice")) && T.HoldSampleS>10)
            { Settings.RoadMu=.02; ApplyMaterials(); bFrictionSwitched=true; }
        }
    }
    else if(TestMode.EndsWith(TEXT("tread_coast")))
    {
        if(TimeS>14 && TimeS<20) { Brake=0; Throttle=.8; }
        if(TimeS>=20)
        {
            Brake=0; bNeutral=true;
            for(AWheelActor* W:Wheels) { W->SteeringJoint->SetOrientationDriveTwistAndSwing(false,false); W->SteeringJoint->SetAngularVelocityDriveTwistAndSwing(false,false); }
            auto& T=ValidationState;
            if(TimeS>22 && T.CoastStartS==0) { T.CoastStartS=TimeS; T.CoastStartJ=TyreMechanicalEnergyJ(); T.CoastStartMps=SpeedMps; T.TreadPumpStartJ=Fork->HydraulicState.DcJ; }
            if(T.CoastStartS>0)
            {
                T.CoastDeltaJ=TyreMechanicalEnergyJ()-T.CoastStartJ-(Fork->HydraulicState.DcJ-T.TreadPumpStartJ);
                T.CoastPeakGainJ=FMath::Max(T.CoastPeakGainJ,T.CoastDeltaJ);
            }
        }
    }
    else if(TestMode.EndsWith(TEXT("carry")) && TimeS>14 && TimeS<26)
    { Brake=0; Throttle=.6; Steering=TimeS>19?.35:0; }
}
bool AStabilityLab::CheckComponentTest() const
{
    UE_LOG(LogTemp,Display,TEXT("ENERGY_AUDIT reference=%d stored_delta_j=%.6f drive_j=%.6f steering_joint_j=%.6f input_j=%.6f known_loss_j=%.6f unresolved_j=%.6f positive_peak_j=%.6f full_closure=0"),
        EnergyAudit.bReferenced?1:0,EnergyAudit.DeltaStoredJ,EnergyAudit.DriveJ-EnergyAudit.ReferenceDriveJ,EnergyAudit.SteeringJ-EnergyAudit.ReferenceSteeringJ,
        EnergyAudit.InputJ,EnergyAudit.KnownLossJ,EnergyAudit.UnresolvedJ,EnergyAudit.PeakPositiveJ);
    const auto& H=Fork->HydraulicState;
    UE_LOG(LogTemp,Display,TEXT("TREAD_TIMESTEP world_s=%.6f physics_s=%.6f max_inner_s=%.9f max_game_s=%.6f"),TimeS,ValidationState.PhysicsSampleS,ValidationState.MaxInnerStepS,ValidationState.MaxGameStepS);
    if(TestMode.EndsWith(TEXT("fracture")) || TestMode.EndsWith(TEXT("plastic")))
    {
        const bool Broken=Fork->Tines[0]->bFailed && Fork->Tines[1]->bFailed;
        const bool Pass=TestMode.EndsWith(TEXT("plastic"))?(!Fork->Tines[0]->bFailed && !Fork->Tines[1]->bFailed && Fork->Tines[0]->PlasticDeg>.01 && Fork->Tines[1]->PlasticDeg>.01):Broken;
        UE_LOG(LogTemp,Display,TEXT("COMPONENT_CHECK %s fixed_root_bench=1 fractured=%d/%d max_stress_mpa=%.3f/%.3f plastic_deg=%.3f/%.3f %s"),*TestMode,Fork->Tines[0]->bFailed?1:0,Fork->Tines[1]->bFailed?1:0,Fork->Tines[0]->MaxStressPa/1e6,Fork->Tines[1]->MaxStressPa/1e6,Fork->Tines[0]->PlasticDeg,Fork->Tines[1]->PlasticDeg,Pass?TEXT("PASS"):TEXT("FAIL"));
        return Pass;
    }
    bool Pass=Settings.Structure.bEnabled && Settings.Tyres.bDetailed && H.bDetailedInitialized && Fork->MovingMast;
    Pass &= FMath::IsFinite(H.PressurePa) && FMath::IsFinite(H.OilTempC) && H.DcJ>0 && !CargoState.bFell && !CargoState.bTipped && !CargoState.bLostSupport;
    Pass &= FMath::Abs(MassKg-Settings.TotalMassKg())<1 && Grounded==4;
    for(AWheelActor* W:Wheels) Pass &= W->CompressionM>.001 && W->EstimatedPatchLengthM>.01 && !W->bTyreTravelLimit;
    for(AForkTineActor* T:Fork->Tines) Pass &= T->Sections.Num()==Settings.Structure.Segments && !T->bFailed && FMath::Abs(T->TipDeflectionM)<.03;
    const double ChainError=FMath::Abs(H.MastHeightM*Settings.Hydraulics.ChainRatio-ForkHeightM);
    Pass &= ChainError<.03 && H.ChainTensionN>=0;
    if(TestMode.EndsWith(TEXT("lower"))) Pass &= ForkHeightM<.03 && PayloadState.HeightM<.03 && MaxForkHeight>.35;
    else Pass &= ForkHeightM>.35 && PayloadState.HeightM>.15 && PayloadState.GroundN<10;
    if(TestMode.EndsWith(TEXT("ramp_brake"))) Pass &= ValidationState.HoldSampleS>10 && ValidationState.HoldDriftM<.03 && FMath::Abs(PitchDeg)>4;
    if(TestMode.EndsWith(TEXT("ramp_ice"))) Pass &= bFrictionSwitched && ValidationState.HoldSampleS>10 && ValidationState.HoldDriftM<.03
        && Chassis->GetActorLocation().X/100.<ValidationState.BrakeStartX-1 && Settings.RoadMu==.02;
    const bool Step=TestMode.EndsWith(TEXT("step"));
    if(Step) Pass &= Chassis->GetActorLocation().X>880 && MaxTilt<10 && !TyreState.bBottomed
        && (Settings.Tyres.Tread.bEnabled || ValidationState.RefinedEdgeCount>0 || FParse::Param(FCommandLine::Get(),TEXT("NativeTyreEdges")));
    Pass &= ValidationState.ContactPeakPoints>=2 && ValidationState.ContactBandError<1e-7;
    if(Settings.Tyres.Tread.bEnabled)
    {
        Pass &= ValidationState.TreadSpreadM>.001 && ValidationState.TreadClosureN<1e-5;
        for(AWheelActor* W:Wheels) Pass &= W->TreadPads.Num()==Settings.Tyres.Tread.Count && !W->bTreadBottomed;
        if(TestMode.EndsWith(TEXT("tread_soft")) || TestMode.EndsWith(TEXT("tread_stiff")))
            Pass &= ValidationState.TreadStaticS>5 && FMath::Abs(ValidationState.TreadLoadIntegral/ValidationState.TreadStaticS/((MassKg+Pallet->LoadMass())*9.81)-1)<.03;
        if(TestMode.EndsWith(TEXT("tread_coast"))) Pass &= ValidationState.CoastStartMps>.25 && ValidationState.CoastPeakGainJ<.01*.5*(MassKg+Pallet->LoadMass())*FMath::Square(ValidationState.CoastStartMps);
        UE_LOG(LogTemp,Display,TEXT("TREAD_CHECK %s count=%d static_s=%.4f mean_compression_m=%.7f spread_m=%.7f closure_n=%.8f coast_start_mps=%.6f coast_delta_j=%.6f coast_peak_j=%.6f"),*TestMode,Settings.Tyres.Tread.Count,ValidationState.TreadStaticS,ValidationState.TreadCompressionIntegral/FMath::Max(1e-8,ValidationState.TreadStaticS),ValidationState.TreadSpreadM,ValidationState.TreadClosureN,ValidationState.CoastStartMps,ValidationState.CoastDeltaJ,ValidationState.CoastPeakGainJ);
    }
    UE_LOG(LogTemp,Display,TEXT("CONTACT_CHECK %s edge_refinements=%lld peak_points=%d band_closure_ns=%.9f point_vs_pair_peak_n=%.5f peak_normal_n=%.3f peak_compression_mm=%.3f x_m=%.4f max_tilt_deg=%.4f"),
        *TestMode,ValidationState.RefinedEdgeCount,ValidationState.ContactPeakPoints,ValidationState.ContactBandError,ValidationState.ContactNormalError,
        ValidationState.ContactPeakLoadN,ValidationState.ContactPeakCompressionM*1000,Chassis->GetActorLocation().X/100.,MaxTilt);
    UE_LOG(LogTemp,Display,TEXT("COMPONENT_CHECK %s mass_kg=%.3f fork_m=%.5f pallet_m=%.5f mast_m=%.5f chain_error_m=%.6f tension_n=%.3f oil_c=%.4f pump_pa=%.2f cylinder_pa=%.2f line_peak_pa=%.2f deflection_mm=%.5f/%.5f hold_s=%.3f drift_m=%.6f %s"),
        *TestMode,MassKg,ForkHeightM,PayloadState.HeightM,H.MastHeightM,ChainError,H.ChainTensionN,H.OilTempC,H.PumpPressurePa,H.PressurePa,H.PeakLinePressurePa,
        Fork->Tines[0]->TipDeflectionM*1000,Fork->Tines[1]->TipDeflectionM*1000,ValidationState.HoldSampleS,ValidationState.HoldDriftM,Pass?TEXT("PASS"):TEXT("FAIL"));
    return Pass;
}
