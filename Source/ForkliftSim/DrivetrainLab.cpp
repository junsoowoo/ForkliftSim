#include "StabilityLab.h"
#include "TerrainCourse.h"
#include "ChassisActor.h"
#include "WheelActor.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

bool AStabilityLab::IsPowerTest() const { return TestMode.StartsWith(TEXT("swerve_power_")); }
bool AStabilityLab::ConfigurePowerLab()
{
    FParse::Value(FCommandLine::Get(),TEXT("Terrain="),TerrainPreset);
    if(IsPowerTest())
    {
        bool Known=false;
        for(const TCHAR* Name:{TEXT("flat"),TEXT("flat_weak"),TEXT("ramp"),TEXT("ramp_weak"),TEXT("step"),TEXT("split"),TEXT("uniform"),TEXT("steer"),TEXT("reset")})
            Known |= TestMode==FString(TEXT("swerve_power_"))+Name;
        if(!Known) { Error=TEXT("Unknown drivetrain test"); return false; }
        Settings.Actuators=FDrivetrainSettings(); Settings.MaxSpeedMps=3; Settings.TyreMu=.9; Settings.RoadMu=.8;
        TerrainPreset=TestMode.Contains(TEXT("ramp"))?TEXT("ramp"):TestMode.EndsWith(TEXT("step"))?TEXT("step"):TestMode.EndsWith(TEXT("split"))?TEXT("split"):TEXT("flat");
        if(TestMode.EndsWith(TEXT("reset"))) TerrainPreset=TEXT("ramp");
        if(TestMode.EndsWith(TEXT("_weak"))) Settings.Actuators.Drive.PowerW=400;
    }
    if(TerrainPreset!=TEXT("flat") && TerrainPreset!=TEXT("ramp") && TerrainPreset!=TEXT("step") && TerrainPreset!=TEXT("halfstep") && TerrainPreset!=TEXT("split"))
    { Error=TEXT("Terrain must be flat/ramp/step/halfstep/split"); return false; }
    if(FParse::Param(FCommandLine::Get(),TEXT("IdealActuators"))) Settings.Actuators.bEnabled=false;
    bMotorView=IsPowerTest() || TerrainPreset!=TEXT("flat");
    return true;
}
void AStabilityLab::SetupTerrain()
{
    if(Course) { Course->Destroy(); Course=nullptr; }
    if(Road)
    {
        Road->SetCollisionEnabled(TerrainPreset==TEXT("split")?ECollisionEnabled::NoCollision:ECollisionEnabled::QueryAndPhysics);
        Road->SetVisibility(TerrainPreset!=TEXT("split"));
    }
    if(TerrainPreset!=TEXT("flat"))
    { Course=GetWorld()->SpawnActor<ATerrainCourse>(); Course->Configure(TerrainPreset,Settings.RoadMu,bPayload?3.:0.); }
}
void AStabilityLab::CycleTerrain()
{
    if(!TestMode.IsEmpty()) return; // scripted fixtures cannot change under a test
    TerrainPreset=TerrainPreset==TEXT("flat")?TEXT("ramp"):TerrainPreset==TEXT("ramp")?TEXT("step"):TerrainPreset==TEXT("step")?TEXT("halfstep"):TerrainPreset==TEXT("halfstep")?TEXT("split"):TEXT("flat");
    bMotorView=true; ResetRig();
}
void AStabilityLab::ControlPowerTest()
{
    if(TestMode==TEXT("swerve_power_reset") && TimeS>6 && PowerResetStage<2)
    {
        ++PowerResetStage; TerrainPreset=PowerResetStage==1?TEXT("split"):TEXT("flat"); ResetRig();
        UE_LOG(LogTemp,Display,TEXT("POWER_RESET stage=%d terrain=%s recreated surfaces, actors and sampler"),PowerResetStage,*TerrainPreset);
        return;
    }
    Throttle=Strafe=Steering=Brake=0;
    if(TimeS>2)
    {
        if(TestMode==TEXT("swerve_power_steer")) Steering=TimeS<8?1:-1;
        else Throttle=1;
    }
}
void AStabilityLab::UpdatePower(double Dt)
{
    auto& P=PowerState;
    for(AWheelActor* W:Wheels)
    {
        const auto& M=W->DriveMotor;
        P.bLimitsValid &= FMath::IsFinite(M.OutputNm) && FMath::IsFinite(M.ShaftW) && FMath::IsFinite(W->SteerMeasuredNm);
        if(Settings.Actuators.bEnabled)
            P.bLimitsValid &= FMath::Abs(M.OutputNm)<=Settings.DriveNm+.01 && FMath::Abs(M.ShaftW)<=Settings.Actuators.Drive.PowerW+.01
                && FMath::Abs(M.MotorNm)<=Settings.Actuators.Drive.PeakNm+.01 && M.LossW>=0;
        P.MaxMotorW=FMath::Max(P.MaxMotorW,FMath::Abs(M.ShaftW));
        P.MaxSteerNm=FMath::Max(P.MaxSteerNm,W->SteerMeasuredNm);
        P.MaxRoadSteerNm=FMath::Max(P.MaxRoadSteerNm,FMath::Abs(W->ContactSteerNm));
        P.MaxSteerDelay=FMath::Max(P.MaxSteerDelay,FMath::Abs(FMath::FindDeltaAngleDegrees(W->SteerGearDeg,W->SteerTargetDeg)));
        P.DcEnergyJ+=M.DcW*Dt; P.WheelEnergyJ+=(M.DcW-M.LossW)*Dt; P.LossEnergyJ+=M.LossW*Dt;
    }
    if(TimeS<=4) return;
    P.SampleS+=Dt; P.DistanceM=Chassis->GetActorLocation().X/100.;
    P.LeftMuIntegral+=(Wheels[0]->ContactMu+Wheels[2]->ContactMu)*.5*Dt;
    P.RightMuIntegral+=(Wheels[1]->ContactMu+Wheels[3]->ContactMu)*.5*Dt;
    if(TimeS<8)
    {
        P.LeftSlipIntegral+=(FMath::Abs(Wheels[0]->LongSlipMps)+FMath::Abs(Wheels[2]->LongSlipMps))*.5*Dt;
        P.SlipSampleS+=Dt;
    }
    for(int I=0;I<4;++I)
    {
        double& Max=I%2==0?P.LeftSlipMax:P.RightSlipMax;
        Max=FMath::Max(Max,FMath::Abs(Wheels[I]->LongSlipMps));
        if(Course && Wheels[I]->FloorContacts>0 && Wheels[I]->Tyre->GetComponentLocation().Z/100>Settings.RadiusM+.01) P.bCourseContact=true;
    }
    if(TerrainPreset==TEXT("ramp") && FMath::Abs(PitchDeg)>2 && Chassis->GetActorLocation().X<1000)
    { P.SlopeSampleS+=Dt; P.SlopeSpeedIntegral+=SpeedMps*Dt; }
}
bool AStabilityLab::CheckPowerTest() const
{
    const auto& P=PowerState; const double T=FMath::Max(P.SampleS,1e-6);
    const double Left=P.LeftMuIntegral/T, Right=P.RightMuIntegral/T, SlopeSpeed=P.SlopeSpeedIntegral/FMath::Max(P.SlopeSampleS,1e-6);
    bool Pass=P.bLimitsValid && Settings.Actuators.bEnabled && P.SampleS>10 && P.MaxMotorW>10;
    if(TerrainPreset==TEXT("ramp")) Pass &= P.bCourseContact && P.SlopeSampleS>1 && SlopeSpeed>.15 && P.DistanceM>4 && MaxTilt>3 && MaxTilt<10;
    else if(TerrainPreset==TEXT("step")) Pass &= P.bCourseContact && P.DistanceM>8 && Grounded==4;
    else if(TerrainPreset==TEXT("split")) Pass &= FMath::Abs(Left-.08)<.01 && FMath::Abs(Right-.8)<.01 && P.LeftSlipMax>.05 && P.DistanceM>5;
    else if(TestMode==TEXT("swerve_power_steer")) Pass &= P.MaxSteerDelay>.1 && P.MaxSteerNm>5 && P.MaxRoadSteerNm>1 && MaxYawRate>20;
    else Pass &= SpeedMps>1 && P.DistanceM>10 && FMath::Abs(Left-.8)<.01 && FMath::Abs(Right-.8)<.01;
    if(TestMode==TEXT("swerve_power_reset")) Pass &= PowerResetStage==2;
    UE_LOG(LogTemp,Display,TEXT("POWER_CHECK %s distance_m=%.4f final_mps=%.4f slope_mps=%.4f slope_s=%.3f left_mu=%.3f right_mu=%.3f left_slip=%.3f right_slip=%.3f left_slip_mean=%.5f slip_sample_s=%.3f motor_peak_w=%.3f steer_peak_nm=%.3f road_steer_peak_nm=%.3f steer_delay_deg=%.3f dc_j=%.2f wheel_j=%.2f loss_j=%.2f %s"),*TestMode,P.DistanceM,SpeedMps,SlopeSpeed,P.SlopeSampleS,Left,Right,P.LeftSlipMax,P.RightSlipMax,P.LeftSlipIntegral/FMath::Max(P.SlipSampleS,1e-6),P.SlipSampleS,P.MaxMotorW,P.MaxSteerNm,P.MaxRoadSteerNm,P.MaxSteerDelay,P.DcEnergyJ,P.WheelEnergyJ,P.LossEnergyJ,Pass?TEXT("PASS"):TEXT("FAIL"));
    return Pass;
}
FString AStabilityLab::PowerCsv(bool Header) const
{
    FString Out;
    if(Header)
    {
        for(const TCHAR* N:{TEXT("FL"),TEXT("FR"),TEXT("RL"),TEXT("RR")})
            Out+=FString::Printf(TEXT(",%s_drive_request_nm,%s_drive_available_nm,%s_motor_rpm,%s_motor_nm,%s_motor_shaft_w,%s_dc_estimate_w,%s_motor_gear_loss_w,%s_steer_available_nm,%s_steer_measured_nm,%s_steer_motor_rpm,%s_steer_gear_deg,%s_contact_mu,%s_contact_steer_nm"),N,N,N,N,N,N,N,N,N,N,N,N,N);
        return Out+TEXT(",terrain_preset,detailed_actuators,drive_motor_power_limit_w,drive_ratio,drive_gear_efficiency,drive_response_s,steer_backlash_deg,dc_energy_estimate_j,wheel_work_estimate_j,drive_loss_estimate_j");
    }
    for(AWheelActor* W:Wheels)
    {
        const auto& M=W->DriveMotor;
        Out+=FString::Printf(TEXT(",%.3f,%.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.3f"),W->DriveRequestNm,M.LimitNm,M.MotorRpm,M.MotorNm,M.ShaftW,M.DcW,M.LossW,W->SteerAvailableNm,W->SteerMeasuredNm,W->SteerMotorRpm,W->SteerGearDeg,W->ContactMu,W->ContactSteerNm);
    }
    return Out+FString::Printf(TEXT(",%s,%d,%.2f,%.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f"),*TerrainPreset,Settings.Actuators.bEnabled?1:0,Settings.Actuators.Drive.PowerW,Settings.Actuators.Drive.Ratio,Settings.Actuators.Drive.GearEfficiency,Settings.Actuators.Drive.ResponseS,Settings.Actuators.BacklashDeg,PowerState.DcEnergyJ,PowerState.WheelEnergyJ,PowerState.LossEnergyJ);
}
void AStabilityHUD::DrawMotors(AStabilityLab* B,float S)
{
    const float X=Canvas->SizeX/S-376; float Y=32;
    DrawRect(FLinearColor(.02,.03,.05,.94),X*S,16*S,360*S,594*S);
    const FLinearColor Ink(.9,.95,1), Accent(.15,.9,.75);
    auto Line=[&](FString Text,FLinearColor Color) { DrawText(Text,Color,(X+12)*S,Y*S,GEngine->GetSmallFont(),S,false); Y+=25; };
    Line(B->Settings.bMecanum?TEXT("0.16 / MECANUM ELECTRIC DRIVE"):TEXT("0.8 / ELECTRIC DRIVE + STATIC TERRAIN"),Accent);
    Line(FString::Printf(TEXT("Course %s | T cycle + RESET"),*B->TerrainPreset),Ink);
    Line(B->Settings.Actuators.bEnabled?TEXT("Detailed motors ON | M cargo/motor HUD"):TEXT("IDEAL actuator comparison (no envelope)"),Accent);
    const TCHAR* Names[]={TEXT("FL"),TEXT("FR"),TEXT("RL"),TEXT("RR")};
    for(int I=0;I<B->Wheels.Num();++I)
    {
        const AWheelActor* W=B->Wheels[I]; const auto& M=W->DriveMotor;
        Line(FString::Printf(TEXT("%s drive %+.0f / %.0f Nm | motor %+.0f rpm"),Names[I],M.OutputNm,M.LimitNm,M.MotorRpm),Accent);
        Line(FString::Printf(TEXT("shaft %.2f kW | loss %.2f kW | mu %.2f"),M.ShaftW/1000,M.LossW/1000,W->ContactMu),Ink);
        if(!B->Settings.bMecanum) Line(FString::Printf(TEXT("Steer %.0f / %.0f Nm | gear %+.1f deg"),W->SteerMeasuredNm,W->SteerAvailableNm,W->SteerGearDeg),Ink);
        if(B->Settings.bMecanum) Line(FString::Printf(TEXT("%d passive rollers | bearing estimate %.2f W"),W->Rollers.Num(),W->RollerBearingPowerW),Ink);
        else Line(FString::Printf(TEXT("Road twist reaction %+.1f Nm"),W->ContactSteerNm),Ink);
    }
    Line(TEXT("Power is per motor; DC/energy are estimates"),Ink);
    Line(TEXT("6 deg ramp | 25 mm full/left step | split mu"),Ink);
    Line(TEXT("Config/Drivetrain.json | uncalibrated"),Accent);
}
