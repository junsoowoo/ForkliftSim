#include "HydraulicLift.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

bool FHydraulicSettings::Valid() const
{
    const double V[]={AreaM2,DeadVolumeM3,BulkPa,ReliefPa,PumpFlowM3s,MotorW,Efficiency,ReturnM3sSqrtPa,LeakM3sPa,SealN,ViscousNsPm,ValveResponseS,ValveGainPerM,DeadbandM};
    for(double X:V) if(!FMath::IsFinite(X)) return false;
    for(double X:{ChainRatio,MastMassKg,ChainK,ChainD,HoseLengthM,HoseBoreM,PumpVolumeM3,OilDensityKgM3,Nu40Cst,Nu100Cst,InitialTempC,AmbientC,ThermalMassJpK,CoolingWpK}) if(!FMath::IsFinite(X)) return false;
    return ChainRatio>=1 && ChainRatio<=3 && MastMassKg>=1 && MastMassKg<=100 && ChainK>=1e4 && ChainK<=2e6 && ChainD>=0 && ChainD<=10000
        && HoseLengthM>=.2 && HoseLengthM<=5 && HoseBoreM>=.004 && HoseBoreM<=.02 && PumpVolumeM3>=.0001 && PumpVolumeM3<=.01
        && OilDensityKgM3>=700 && OilDensityKgM3<=1100 && Nu40Cst>=10 && Nu40Cst<=150 && Nu100Cst>1 && Nu100Cst<Nu40Cst
        && InitialTempC>=0 && InitialTempC<=100 && AmbientC>=0 && AmbientC<=60 && ThermalMassJpK>=1000 && CoolingWpK>=0
        && AreaM2>=.001 && AreaM2<=.005 && DeadVolumeM3>=.001 && DeadVolumeM3<=.01
        && BulkPa>=10000000 && BulkPa<=300000000 && ReliefPa>=1000000 && ReliefPa<=20000000
        && PumpFlowM3s>0 && PumpFlowM3s<=.001 && MotorW>=100 && MotorW<=10000 && Efficiency>.1 && Efficiency<=1
        && ReturnM3sSqrtPa>0 && ReturnM3sSqrtPa<=.000001 && LeakM3sPa>=0 && LeakM3sPa<=1e-10
        && SealN>=0 && SealN<=1000 && ViscousNsPm>=0 && ViscousNsPm<=5000
        && ValveResponseS>=.01 && ValveResponseS<=1 && ValveGainPerM>0 && ValveGainPerM<=20 && DeadbandM>=.001 && DeadbandM<=.02;
}
bool FHydraulicSettings::Load(FString& Error)
{
    FString Text; TSharedPtr<FJsonObject> J;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("Hydraulics.json"))) ||
        !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J) || !J.IsValid())
    { Error=TEXT("Cannot read Config/Hydraulics.json"); return false; }
    auto Read=[&](const TCHAR* Key,double& V) { return J->TryGetNumberField(Key,V); };
    if(!J->TryGetBoolField(TEXT("enabled"),bEnabled)
        || !Read(TEXT("cylinder_area_m2"),AreaM2) || !Read(TEXT("dead_volume_m3"),DeadVolumeM3)
        || !Read(TEXT("effective_bulk_modulus_pa"),BulkPa) || !Read(TEXT("relief_pressure_pa"),ReliefPa)
        || !Read(TEXT("pump_max_flow_m3_s"),PumpFlowM3s) || !Read(TEXT("motor_input_limit_w"),MotorW)
        || !Read(TEXT("motor_pump_efficiency"),Efficiency) || !Read(TEXT("return_orifice_m3_s_sqrt_pa"),ReturnM3sSqrtPa)
        || !Read(TEXT("closed_valve_leak_m3_s_pa"),LeakM3sPa) || !Read(TEXT("seal_friction_n"),SealN)
        || !Read(TEXT("viscous_friction_ns_m"),ViscousNsPm) || !Read(TEXT("valve_response_s"),ValveResponseS)
        || !Read(TEXT("valve_gain_per_m"),ValveGainPerM) || !Read(TEXT("position_deadband_m"),DeadbandM))
    { Error=TEXT("Invalid hydraulic settings"); return false; }
    if(J->HasField(TEXT("detailed_enabled")))
    {
        if(!J->TryGetBoolField(TEXT("detailed_enabled"),bDetailed) || !J->TryGetBoolField(TEXT("chain_mast_enabled"),bChainMast)
            || !Read(TEXT("chain_ratio"),ChainRatio) || !Read(TEXT("mast_stage_mass_kg"),MastMassKg)
            || !Read(TEXT("chain_stiffness_npm"),ChainK) || !Read(TEXT("chain_damping_nspm"),ChainD)
            || !Read(TEXT("hose_length_m"),HoseLengthM) || !Read(TEXT("hose_bore_m"),HoseBoreM) || !Read(TEXT("pump_volume_m3"),PumpVolumeM3)
            || !Read(TEXT("oil_density_kg_m3"),OilDensityKgM3) || !Read(TEXT("oil_nu40_cst"),Nu40Cst) || !Read(TEXT("oil_nu100_cst"),Nu100Cst)
            || !Read(TEXT("initial_oil_temp_c"),InitialTempC) || !Read(TEXT("ambient_temp_c"),AmbientC)
            || !Read(TEXT("thermal_mass_jpk"),ThermalMassJpK) || !Read(TEXT("cooling_wpk"),CoolingWpK))
        { Error=TEXT("Invalid detailed hydraulic settings"); return false; }
    }
    if(!Valid()) { Error=TEXT("Hydraulic settings outside supported bounds"); return false; }
    return true;
}
double FHydraulicState::ValveRequest(const FHydraulicSettings& C,double Target,double H,double Travel,const FHydraulicCommand& Control)
{
    if(Control.Direct){const double Command=FMath::IsFinite(Control.Spool)?FMath::Clamp(Control.Spool,-1.,1.):0.;bHolding=FMath::Abs(Command)<1e-6;return Command;}
    Target=FMath::Clamp(Target,0.,Travel);
    if(FMath::Abs(Target-PreviousTarget)>1e-8) bHolding=false;
    PreviousTarget=Target;
    const double Error=Target-H;
    if(!bHolding && FMath::Abs(Error)<C.DeadbandM) bHolding=true;
    // Once closed, do not silently re-level a leaking cylinder. A new operator
    // target reopens the valve. Position is physics ground truth, NOT a sensor.
    double Command=bHolding?0:FMath::Clamp(Error*C.ValveGainPerM,-1.,1.);
    if((H<=0 && Command<0) || (H>=Travel && Command>0)) Command=0;
    return Command;
}
void FHydraulicState::Step(const FHydraulicSettings& C,double Dt,double Target,double H,double V,double Travel,double ForceCapN,const FHydraulicCommand& Control)
{
    if(Dt<=0) return;
    if(C.bDetailed) { StepDetailed(C,Dt,Target,H,V,Travel,ForceCapN,Control); return; }
    const double MaxP=FMath::Min(C.ReliefPa,ForceCapN/C.AreaM2);
    const double Command=ValveRequest(C,Target,H,Travel,Control);
    Valve+=(Command-Valve)*(1-FMath::Exp(-Dt/C.ValveResponseS));
    if(FMath::Abs(Valve)<1e-6) Valve=0;
    const double P=FMath::Clamp(PressurePa,0.,MaxP);
    const double Qmax=FMath::Min(C.PumpFlowM3s,C.MotorW*C.Efficiency/FMath::Max(P,1000.));
    PumpM3s=(!Control.Direct||Control.PumpEnabled)?FMath::Max(Valve,0.)*Qmax:0;
    ReturnM3s=FMath::Max(-Valve,0.)*C.ReturnM3sSqrtPa*FMath::Sqrt(P);
    LeakM3s=C.LeakM3sPa*P;
    const double Volume=C.DeadVolumeM3+C.AreaM2*FMath::Clamp(H,0.,Travel);
    const double Raw=P+C.BulkPa/Volume*(PumpM3s-ReturnM3s-LeakM3s-C.AreaM2*V)*Dt;
    PressurePa=FMath::Clamp(Raw,0.,MaxP);
    // Ideal relief pressure cap; flow includes externally back-driven flow.
    ReliefM3s=FMath::Max(0.,Raw-MaxP)*Volume/C.BulkPa/Dt;
    const double Friction=C.SealN*FMath::Tanh(V/.01)+C.ViscousNsPm*V;
    ForceN=C.AreaM2*PressurePa-Friction;
    // Power reporting uses inlet pressure at evaluation, not post-update pressure.
    HydraulicW=P*PumpM3s; DcW=HydraulicW/C.Efficiency;
    MechanicalW=ForceN*V;
    HeatW=DcW-HydraulicW+P*(ReturnM3s+LeakM3s)+MaxP*ReliefM3s+Friction*V;
    DcJ+=DcW*Dt; MechanicalJ+=MechanicalW*Dt; HeatJ+=HeatW*Dt;
}
