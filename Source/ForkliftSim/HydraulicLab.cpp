#include "StabilityLab.h"
#include "ForkActor.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "ValidationMath.h"

bool AStabilityLab::IsHydraulicTest() const { return TestMode.StartsWith(TEXT("swerve_hydraulic_")); }
bool AStabilityLab::ConfigureHydraulics()
{
    bHydraulicView=IsHydraulicTest() || FParse::Param(FCommandLine::Get(),TEXT("HydraulicLab"));
    if(IsHydraulicTest())
    {
        bool Known=false;
        for(const TCHAR* N:{TEXT("empty"),TEXT("loaded"),TEXT("low_power"),TEXT("overload"),TEXT("hold"),TEXT("leak"),TEXT("lower"),TEXT("legacy")})
            Known |= TestMode==FString(TEXT("swerve_hydraulic_"))+N;
        if(!Known) { Error=TEXT("Unknown hydraulic test"); return false; }
        Settings.Hydraulics=FHydraulicSettings(); TerrainPreset=TEXT("flat");
        if(TestMode.EndsWith(TEXT("low_power"))) Settings.Hydraulics.MotorW=600;
        if(TestMode.EndsWith(TEXT("overload"))) Settings.Hydraulics.ReliefPa=2000000;
        if(TestMode.EndsWith(TEXT("leak"))) Settings.Hydraulics.LeakM3sPa=2e-12;
        if(TestMode.EndsWith(TEXT("legacy"))) Settings.Hydraulics.bEnabled=false;
    }
    if(FParse::Param(FCommandLine::Get(),TEXT("LegacyLift"))) Settings.Hydraulics.bEnabled=false;
    return Settings.Hydraulics.Valid();
}
void AStabilityLab::ControlHydraulicTest()
{
    Throttle=Strafe=Steering=Lift=0; Brake=1; bNeutral=true;
    // Step target exposes the actuator's real flow/power limits, not the old
    // target ramp. Manual Q/E still uses the common target-height ramp.
    LiftTargetM=TimeS>2?.9:0;
    if(TestMode.EndsWith(TEXT("lower")) && TimeS>12) LiftTargetM=0;
}
void AStabilityLab::UpdateHydraulicLab(double Dt)
{
    const auto& S=Fork->HydraulicState; const auto& C=Fork->HydraulicSettings;
    auto& T=HydraulicLabState;
    T.PeakPressure=FMath::Max(T.PeakPressure,Fork->HydraulicFramePeakPressurePa);
    T.PeakDcW=FMath::Max(T.PeakDcW,Fork->HydraulicFramePeakDcW);
    T.ReliefM3+=Fork->HydraulicFrameReliefM3;
    T.bLimitsValid &= FMath::IsFinite(S.PressurePa) && FMath::IsFinite(S.ForceN) && FMath::IsFinite(S.DcJ)
        && S.PressurePa>=0 && S.PressurePa<=FMath::Min(C.ReliefPa,Settings.ForkForceN/C.AreaM2)+.01
        && S.DcW<=C.MotorW+.01 && S.HeatW>=-.01;
    // Mid-stroke speed isolates acceleration, stopping and initial pallet take-up.
    if(IsHydraulicTest() && TimeS<12)
        ValidationMath::RisingWindow(T.PreviousHeightM,ForkHeightM,Dt,.25,.55,T.LiftDistance,T.LiftS);
    T.PreviousHeightM=ForkHeightM;
    if(TimeS<12) T.HoldStartM=ForkHeightM;
    T.HoldEndM=ForkHeightM;
    if(TimeS>18) T.LowerMinM=FMath::Min(T.LowerMinM,ForkHeightM);
}
bool AStabilityLab::CheckHydraulicTest() const
{
    const auto& T=HydraulicLabState; const auto& S=Fork->HydraulicState;
    const bool Over=TestMode.EndsWith(TEXT("overload")), Legacy=TestMode.EndsWith(TEXT("legacy"));
    const double Sag=T.HoldStartM-T.HoldEndM, Speed=T.LiftDistance/FMath::Max(T.LiftS,1e-8);
    bool Pass=T.bLimitsValid && !CargoState.bFell;
    // The blades first traverse ~11 cm of real clearance inside the holes.
    // Overload means the PALLET stays grounded, not zero free carriage travel.
    if(Over) Pass &= ForkHeightM<.15 && T.PeakPressure>Settings.Hydraulics.ReliefPa*.98 && T.ReliefM3>1e-5 && PayloadState.HeightM<.02;
    else if(TestMode.EndsWith(TEXT("lower"))) Pass &= T.HoldStartM>.8 && FMath::Abs(ForkHeightM)<.03 && PayloadState.HeightM<.03;
    else if(Legacy) Pass &= ForkHeightM>.8 && S.DcJ==0;
    else
    {
        Pass &= ForkHeightM>.7 && T.LiftS>.5 && Speed>.05 && Speed<.3 && S.DcJ>1000;
        if(TestMode.EndsWith(TEXT("leak"))) Pass &= Sag>.01 && Sag<.08;
        else if(!TestMode.EndsWith(TEXT("low_power"))) Pass &= FMath::Abs(Sag)<.005;
        if(!TestMode.EndsWith(TEXT("leak"))) Pass &= ForkHeightM>.88 && S.bHolding;
        if(bPayload) Pass &= PayloadState.HeightM>.65;
    }
    UE_LOG(LogTemp,Display,TEXT("HYDRAULIC_CHECK %s mean_lift_mps=%.6f pressure_peak_pa=%.1f dc_peak_w=%.3f dc_j=%.3f mechanical_j=%.3f heat_j=%.3f relief_l=%.6f hold_sag_m=%.6f fork_m=%.5f payload_m=%.5f limits=%d %s"),
        *TestMode,Speed,T.PeakPressure,T.PeakDcW,S.DcJ,S.MechanicalJ,S.HeatJ,T.ReliefM3*1000,Sag,ForkHeightM,bPayload?PayloadState.HeightM:0,T.bLimitsValid?1:0,Pass?TEXT("PASS"):TEXT("FAIL"));
    return Pass;
}
FString AStabilityLab::HydraulicCsv(bool Header) const
{
    if(Header) return TEXT(",hydraulic_enabled,lift_pressure_pa,lift_valve,lift_holding,lift_velocity_mps,lift_pump_m3s,lift_return_m3s,lift_leak_m3s,lift_relief_m3s,lift_force_n,lift_dc_w,lift_hydraulic_w,lift_mechanical_w,lift_heat_w,lift_dc_j,lift_mechanical_j,lift_heat_j,lift_fluid_stored_j");
    const auto& H=Fork->HydraulicState;
    return FString::Printf(TEXT(",%d,%.3f,%.6f,%d,%.6f,%.9f,%.9f,%.9f,%.9f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f"),
        Fork->HydraulicSettings.bEnabled?1:0,H.PressurePa,H.Valve,H.bHolding?1:0,Fork->HydraulicVelocityMps,H.PumpM3s,H.ReturnM3s,H.LeakM3s,H.ReliefM3s,H.ForceN,H.DcW,H.HydraulicW,H.MechanicalW,H.HeatW,H.DcJ,H.MechanicalJ,H.HeatJ,H.StoredJ(Fork->HydraulicSettings,ForkHeightM));
}
void AStabilityHUD::DrawHydraulics(AStabilityLab* B,float S)
{
    const float X=Canvas->SizeX/S-376; float Y=32;
    DrawRect(FLinearColor(.02,.03,.05,.94),X*S,16*S,360*S,410*S);
    const FLinearColor Ink(.9,.95,1);
    auto Line=[&](FString Text) { DrawText(Text,Ink,(X+12)*S,Y*S,GEngine->GetSmallFont(),S,false); Y+=25; };
    const auto& H=B->Fork->HydraulicState; const auto& C=B->Fork->HydraulicSettings;
    Line(TEXT("0.10 / HYDRAULIC LIFT | K HUD"));
    Line(C.bEnabled?TEXT("Single acting / gravity return / 1:1"):TEXT("LEGACY joint position servo"));
    Line(FString::Printf(TEXT("Target / actual %.3f / %.3f m"),B->LiftTargetM,B->ForkHeightM));
    Line(FString::Printf(TEXT("Pressure %.2f / %.2f MPa"),H.PressurePa/1e6,FMath::Min(C.ReliefPa,B->Settings.ForkForceN/C.AreaM2)/1e6));
    Line(FString::Printf(TEXT("Cylinder force %.2f kN | v %+.3f m/s"),H.ForceN/1000,B->Fork->HydraulicVelocityMps));
    Line(FString::Printf(TEXT("Valve %+.2f | %s"),H.Valve,H.bHolding?TEXT("HOLD (no auto re-level)"):TEXT("TRACK")));
    Line(FString::Printf(TEXT("Pump / return %.2f / %.2f L/min"),H.PumpM3s*60000,H.ReturnM3s*60000));
    Line(FString::Printf(TEXT("Leak / relief %.3f / %.2f L/min"),H.LeakM3s*60000,H.ReliefM3s*60000));
    Line(FString::Printf(TEXT("DC %.0f / %.0f W | hydraulic %.0f W"),H.DcW,C.MotorW,H.HydraulicW));
    Line(FString::Printf(TEXT("DC %.1f J | work %.1f J"),H.DcJ,H.MechanicalJ));
    Line(FString::Printf(TEXT("Heat %.1f J | fluid U %.1f J"),H.HeatJ,H.StoredJ(C,B->ForkHeightM)));
    Line(TEXT("Q/E lower/lift | R resets pressure + energy"));
    Line(TEXT("Config/Hydraulics.json | SI units"));
    Line(TEXT("Ground truth, no lift sensors/autonomy"));
    Line(TEXT("Assumed parameters, NOT real-machine rating"));
}
