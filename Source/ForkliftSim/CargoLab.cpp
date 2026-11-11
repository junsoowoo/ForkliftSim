#include "StabilityLab.h"
#include "CargoGeometry.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "WheelActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"

bool AStabilityLab::IsCargoTest() const { return TestMode.StartsWith(TEXT("swerve_payload_cargo_")); }
void AStabilityLab::ConfigureCargoTest()
{
    auto& S=PayloadSettings; S.bCargoSecured=false;
    if(TestMode==TEXT("swerve_payload_cargo_entry")) S.InitialDepthM=-.25;
    if(TestMode==TEXT("swerve_payload_cargo_accel_low") || TestMode==TEXT("swerve_payload_cargo_corner_low"))
    { S.CargoStaticMu=.025; S.CargoDynamicMu=.02; }
    if(TestMode==TEXT("swerve_payload_cargo_brake_low")) { S.CargoStaticMu=.08; S.CargoDynamicMu=.06; }
    if(TestMode==TEXT("swerve_payload_cargo_drop")) { S.CargoStaticMu=.015; S.CargoDynamicMu=.01; }
    if(TestMode==TEXT("swerve_payload_cargo_tip"))
    {
        S.CargoLengthM=.20; S.CargoHeightM=1.2; Settings.MaxSpeedMps=2;
        S.PalletTopStaticMu=S.CargoStaticMu=1; S.PalletTopDynamicMu=S.CargoDynamicMu=.9;
        S.ForkMu=S.LeftHoleMu=S.RightHoleMu=1;
    }
}
void AStabilityLab::UpdateCargo(double Dt,double PhysicsDt)
{
    auto& C=CargoState; const auto& S=PayloadSettings;
    auto* Body=Pallet->Cargo->GetStaticMeshComponent(); auto* Deck=Pallet->Deck.Get();
    const FQuat Q=Deck->GetComponentQuat();
    const FVector Relative=Q.UnrotateVector(Body->GetCenterOfMass()-Deck->GetComponentLocation())/100.;
    C.ShiftM=Relative-C.InitialRelativeM;
    C.TiltDeg=FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(FVector::DotProduct(Body->GetUpVector(),Deck->GetUpVector()),-1.,1.)));
    C.WorldTiltDeg=FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Body->GetUpVector().Z,-1.,1.)));
    C.NormalN=C.FrictionN=C.GroundN=C.OtherN=0; C.ForceN=C.FrictionForceN=FVector::ZeroVector;
    if(PhysicsDt>0)
    {
        C.NormalN=C.Frame.NormalNs/PhysicsDt; C.FrictionN=C.Frame.FrictionMagnitudeNs/PhysicsDt;
        C.GroundN=C.Frame.GroundNormalNs/PhysicsDt; C.OtherN=C.Frame.OtherNormalNs/PhysicsDt;
        C.ForceN=C.Frame.ImpulseNs/PhysicsDt; C.FrictionForceN=C.Frame.FrictionImpulseNs/PhysicsDt;
    }
    C.ContactPointM=C.Frame.NormalNs>1e-8?C.Frame.PointWeightedMNs/C.Frame.NormalNs:Body->GetCenterOfMass()/100.;
    const FVector N=C.Frame.NormalImpulseNs.GetSafeNormal();
    const FVector V=(Body->GetPhysicsLinearVelocityAtPoint(C.ContactPointM*100)-Deck->GetPhysicsLinearVelocityAtPoint(C.ContactPointM*100))/100.;
    C.SlipMps=C.NormalN>1?(V-N*FVector::DotProduct(V,N)).Size():0;
    const double Alpha=1-FMath::Exp(-Dt/.12);
    C.FilteredNormalN+=(C.NormalN-C.FilteredNormalN)*Alpha; C.FilteredFrictionN+=(C.FrictionN-C.FilteredFrictionN)*Alpha;
    TArray<FVector2D> Footprint;
    const FVector Corners[]={FVector(-50,-50,-50),FVector(50,-50,-50),FVector(50,50,-50),FVector(-50,50,-50)};
    for(const FVector& Corner:Corners)
    {
        const FVector Local=Q.UnrotateVector(Body->GetComponentTransform().TransformPosition(Corner)-Deck->GetComponentLocation())/100.;
        Footprint.Add(FVector2D(Local.X,Local.Y));
    }
    C.Overlap=CargoGeometry::OverlapFraction(MoveTemp(Footprint),FVector2D(S.LengthM,S.WidthM)*.5);
    const double Weight=Body->GetMass()*9.81;
    if(PayloadState.HeightM>.15) C.bArmed=true;
    if(C.bArmed && PayloadState.HeightM>.15)
    {
        C.MaxShiftM=FMath::Max(C.MaxShiftM,FVector2D(C.ShiftM.X,C.ShiftM.Y).Size());
        C.MaxTiltDeg=FMath::Max(C.MaxTiltDeg,C.TiltDeg);
        C.MaxFrictionN=FMath::Max(C.MaxFrictionN,C.FrictionN);
        C.SeparationS=C.NormalN<Weight*.01?C.SeparationS+Dt:0;
        C.bLostSupport |= !S.bCargoSecured && C.SeparationS>.15;
        C.bTipped |= C.TiltDeg>45;
        C.bFell |= C.GroundN>Weight*.05;
        const double Mu=FMath::Min(S.PalletTopStaticMu,S.CargoStaticMu);
        if(C.NormalN>Weight*.1 && Mu>0)
        {
            const double Ratio=C.FrictionN/(Mu*C.NormalN);
            C.MaxUtilization=FMath::Max(C.MaxUtilization,Ratio);
            // NetPushOut/Dt plus velocity impulses is the measured engine reaction.
            // During edge impacts its ratio need not satisfy a steady planar
            // Coulomb bound. Keep that raw maximum; validate the bound separately
            // on near-horizontal support, without clipping reported forces.
            if(C.TiltDeg<5 && C.WorldTiltDeg<5 && C.GroundN<1 && C.OtherN<1)
            { C.MaxPlanarUtilization=FMath::Max(C.MaxPlanarUtilization,Ratio); C.PlanarSampleS+=PhysicsDt; }
        }
    }
    if(TimeS>(TestMode==TEXT("swerve_payload_cargo_unload")?16:12) && PhysicsDt>0)
    {
        C.SampleS+=PhysicsDt; C.NormalIntegral+=C.NormalN*PhysicsDt; C.VerticalIntegral+=C.ForceN.Z*PhysicsDt;
        C.GroundIntegral+=C.GroundN*PhysicsDt; C.WheelIntegral+=TotalNormalLoadN*PhysicsDt;
        C.ForkIntegral+=(PayloadState.VerticalN[0]+PayloadState.VerticalN[1])*PhysicsDt;
    }
    if(TimeS>5 && TimeS<6 && PhysicsDt>0)
    {
        C.BaselineFrontIntegral+=(Wheels[0]->NormalLoadN+Wheels[1]->NormalLoadN)*PhysicsDt;
        C.BaselineWheelIntegral+=TotalNormalLoadN*PhysicsDt;
    }
}
void AStabilityLab::ControlCargoTest(double Dt)
{
    auto& C=CargoState;
    Throttle=Steering=Strafe=Brake=Lift=0;
    if(TestMode==TEXT("swerve_payload_cargo_entry") || TestMode==TEXT("swerve_payload_cargo_unload"))
    {
        const bool Entry=TestMode==TEXT("swerve_payload_cargo_entry");
        const bool Unloading=!Entry && TimeS>=9;
        LiftTargetM=FMath::FInterpConstantTo(LiftTargetM,TimeS>(Entry?6:2) && !Unloading?.6:0,Dt,Settings.ForkSpeedMps);
        if(Entry && TimeS>2 && TimeS<5.5) Throttle=.4;
        if(!Entry && TimeS>6 && TimeS<9) Throttle=1;
        if(!Entry && TimeS>13 && TimeS<16) Throttle=-1;
        return;
    }
    LiftTargetM=FMath::FInterpConstantTo(LiftTargetM,TimeS>2?.6:0,Dt,Settings.ForkSpeedMps);
    const bool Accel=TestMode.Contains(TEXT("cargo_accel"));
    const bool Corner=TestMode.Contains(TEXT("cargo_corner"));
    const bool Hold=TestMode==TEXT("swerve_payload_cargo_hold");
    if(Hold) return;
    if(Accel) { if(TimeS>6 && TimeS<10) Throttle=1; return; }
    // A matched initial condition isolates braking/cornering from the ramp-up
    // acceleration. Applied ONCE after lifting, to every loaded rig body.
    if(TimeS>6 && !C.bSeeded)
    {
        const double Speed=TestMode==TEXT("swerve_payload_cargo_tip")?2:1;
        C.bSeeded=true; SeedSpeed(Speed);
        const FVector V(Speed*100,0,0);
        for(AForkTineActor* T:Fork->Tines) for(UStaticMeshComponent* B:T->Sections) B->SetPhysicsLinearVelocity(V);
        if(Fork->MovingMast) Fork->MovingMast->SetPhysicsLinearVelocity(V);
        Pallet->Deck->SetPhysicsLinearVelocity(V); Pallet->Cargo->GetStaticMeshComponent()->SetPhysicsLinearVelocity(V);
        TargetVelocity=FVector2D(Speed,0); TargetHeadingDeg=Chassis->GetActorRotation().Yaw;
    }
    if(Corner) { if(TimeS>6 && TimeS<12) Throttle=1; if(TimeS>8 && TimeS<12) Steering=1; }
    else
    {
        if(TimeS>6 && TimeS<7) Throttle=1;
        // Matched moderate braking isolates cargo/deck friction while keeping
        // the UNSECURED pallet on its forks. Full brakes remain in tip/drop
        // stress cases; do not weaken real brakes to preserve an old fixture.
        if(TimeS>=7 && TimeS<10) Brake=TestMode.Contains(TEXT("cargo_brake"))?.12:1;
    }
}
bool AStabilityLab::CheckCargoTest() const
{
    const auto& C=CargoState; const auto& S=PayloadSettings; const auto& P=PayloadState;
    const double T=FMath::Max(C.SampleS,1e-6), Weight=S.CargoMassKg*9.81;
    const double Normal=C.NormalIntegral/T, Vertical=C.VerticalIntegral/T, Ground=C.GroundIntegral/T, WheelsN=C.WheelIntegral/T;
    bool Pass=!S.bCargoSecured && C.bArmed && C.SampleS>3.5 && FMath::IsFinite(C.MaxShiftM) && FMath::IsFinite(C.MaxTiltDeg)
        && FMath::IsFinite(C.MaxUtilization) && C.PlanarSampleS>1 && C.MaxPlanarUtilization<1.05;
    if(TestMode==TEXT("swerve_payload_cargo_unload"))
        Pass &= !C.bFell && !C.bTipped && C.MaxShiftM<.04 && P.HeightM<.05 && P.DepthM[0]<.1 && P.DepthM[1]<.1
            && P.GroundN>Pallet->LoadMass()*9.81*.9 && FMath::Abs(C.ForkIntegral/T)<100
            && FMath::Abs(Normal-Weight)/Weight<.05 && FMath::Abs(WheelsN-MassKg*9.81)/(MassKg*9.81)<.05;
    else if(TestMode==TEXT("swerve_payload_cargo_tip")) Pass &= C.bTipped && C.MaxTiltDeg>45;
    else if(TestMode==TEXT("swerve_payload_cargo_drop"))
        Pass &= C.bFell && C.bLostSupport && C.MaxShiftM>.6 && Ground>Weight*.8 && FMath::Abs(WheelsN-(MassKg+S.PalletMassKg)*9.81)/((MassKg+S.PalletMassKg)*9.81)<.05;
    else if(TestMode.EndsWith(TEXT("_low"))) Pass &= C.MaxShiftM>.08 && C.MaxFrictionN>5;
    else
    {
        Pass &= !C.bFell && !C.bTipped && C.MaxShiftM<.04 && C.MaxTiltDeg<5 && P.HeightM>.25;
        Pass &= FMath::Abs(Normal-Weight)/Weight<.05 && FMath::Abs(Vertical-Weight)/Weight<.05;
        Pass &= FMath::Abs(WheelsN-(MassKg+Pallet->LoadMass())*9.81)/((MassKg+Pallet->LoadMass())*9.81)<.05;
        if(TestMode!=TEXT("swerve_payload_cargo_hold")) Pass &= C.MaxFrictionN>30;
    }
    const double Front=P.FrontIntegral/FMath::Max(P.WheelIntegral,1e-6);
    const double BaselineFront=C.BaselineFrontIntegral/FMath::Max(C.BaselineWheelIntegral,1e-6);
    if(TestMode==TEXT("swerve_payload_cargo_brake_low"))
        Pass &= !C.bFell && C.ShiftM.X>.08 && Front>BaselineFront+.005 && Normal>Weight*.9;
    UE_LOG(LogTemp,Display,TEXT("CARGO_CHECK %s normal_n=%.2f vertical_n=%.2f weight_n=%.2f ground_n=%.2f wheel_n=%.2f max_shift=%.4f max_tilt=%.2f max_friction_n=%.2f max_mu_ratio=%.3f max_planar_mu_ratio=%.3f planar_sample_s=%.2f front_fraction=%.4f baseline_front=%.4f lost=%d tipped=%d fell=%d %s"),*TestMode,Normal,Vertical,Weight,Ground,WheelsN,C.MaxShiftM,C.MaxTiltDeg,C.MaxFrictionN,C.MaxUtilization,C.MaxPlanarUtilization,C.PlanarSampleS,Front,BaselineFront,C.bLostSupport?1:0,C.bTipped?1:0,C.bFell?1:0,Pass?TEXT("PASS"):TEXT("FAIL"));
    return Pass;
}
FString AStabilityLab::CargoCsv(bool Header) const
{
    if(Header) return TEXT(",cargo_shift_x_m,cargo_shift_y_m,cargo_shift_z_m,cargo_max_shift_m,cargo_tilt_rel_deg,cargo_tilt_world_deg,cargo_max_tilt_deg,cargo_base_overlap,cargo_contact_normal_n,cargo_contact_friction_n,cargo_force_x_n,cargo_force_y_n,cargo_force_z_n,cargo_friction_x_n,cargo_friction_y_n,cargo_friction_z_n,cargo_cop_x_m,cargo_cop_y_m,cargo_cop_z_m,cargo_slip_mps,cargo_floor_normal_n,cargo_other_normal_n,cargo_contact_points,cargo_lost_support,cargo_tipped,cargo_fell,pallet_top_static_mu,pallet_top_dynamic_mu,cargo_static_mu,cargo_dynamic_mu,cargo_length_m,cargo_width_m,cargo_height_m");
    const auto& C=CargoState; const auto& S=PayloadSettings;
    return FString::Printf(TEXT(",%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f"),C.ShiftM.X,C.ShiftM.Y,C.ShiftM.Z,C.MaxShiftM,C.TiltDeg,C.WorldTiltDeg,C.MaxTiltDeg,C.Overlap,C.NormalN,C.FrictionN,C.ForceN.X,C.ForceN.Y,C.ForceN.Z,C.FrictionForceN.X,C.FrictionForceN.Y,C.FrictionForceN.Z,C.ContactPointM.X,C.ContactPointM.Y,C.ContactPointM.Z,C.SlipMps,C.GroundN,C.OtherN,C.Frame.Points,C.bLostSupport?1:0,C.bTipped?1:0,C.bFell?1:0,S.PalletTopStaticMu,S.PalletTopDynamicMu,S.CargoStaticMu,S.CargoDynamicMu,S.CargoLengthM,S.CargoWidthM,S.CargoHeightM);
}
void AStabilityHUD::DrawCargo(AStabilityLab* B,float Scale,float X,float Y)
{
    const auto& C=B->CargoState; const auto& S=B->PayloadSettings;
    const FLinearColor Ink(.9f,.95f,1), Accent(.15f,.9f,.75f), Warn(1,.45f,.15f);
    auto Line=[&](FString T,FLinearColor Color) { DrawText(T,Color,X*Scale,Y*Scale,GEngine->GetSmallFont(),Scale,false); Y+=25; };
    Line(S.bCargoSecured?TEXT("CARGO vs PALLET (SECURED comparison)"):TEXT("CARGO vs PALLET (independent contact)"),Accent);
    Line(FString::Printf(TEXT("Cargo shift X/Y  %+.3f / %+.3f m"),C.ShiftM.X,C.ShiftM.Y),Ink);
    Line(FString::Printf(TEXT("Normal / friction  %.2f / %.2f kN"),C.FilteredNormalN/1000,C.FilteredFrictionN/1000),Ink);
    Line(FString::Printf(TEXT("Slip / tilt  %.3f m/s / %.1f deg"),C.SlipMps,C.TiltDeg),Ink);
    Line(FString::Printf(TEXT("Base overlap %.0f%% | contact points %d"),100*C.Overlap,C.Frame.Points),Ink);
    Line(FString::Printf(TEXT("Contact mu static/dynamic %.2f / %.2f"),FMath::Min(S.PalletTopStaticMu,S.CargoStaticMu),FMath::Min(S.PalletTopDynamicMu,S.CargoDynamicMu)),Ink);
    Line(S.bCargoSecured?TEXT("Fixed joint: deck/cargo contact DISABLED"):C.bFell?TEXT("CARGO FELL: floor contact after lifting"):C.bTipped?TEXT("CARGO TIPPED: relative tilt exceeded 45 deg"):C.bLostSupport?TEXT("CARGO LOST DECK SUPPORT"):TEXT("Cargo tracked separately from pallet"),C.bFell || C.bTipped || C.bLostSupport?Warn:Accent);
}
