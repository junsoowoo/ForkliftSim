#include "StabilityLab.h"
#include "ChassisActor.h"
#include "WheelActor.h"
#include "ForkActor.h"
#include "ContactLoadSampler.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

bool AStabilityLab::ConfigurePayload()
{
    bPayload=IsMissionTest() || IsHandlingTest() || IsPalletPerceptionTest() || (IsSensorLiftTest()&&!TestMode.EndsWith(TEXT("empty"))) || IsRackTest() || (Settings.bReferenceGeometry && TestMode.IsEmpty()) || FParse::Param(FCommandLine::Get(),TEXT("PayloadLab")) || TestMode.StartsWith(TEXT("swerve_payload_")) || TestMode==TEXT("swerve_tyre_payload")
        || (IsHydraulicTest() && !TestMode.EndsWith(TEXT("empty"))) || IsIntegrationTest() || IsComponentTest()
        || (TestMode.StartsWith(TEXT("mecanum_")) && (TestMode.EndsWith(TEXT("lift")) || TestMode.EndsWith(TEXT("carry")) || TestMode.EndsWith(TEXT("step")) || TestMode.EndsWith(TEXT("ramp_brake")) || TestMode==TEXT("mecanum_halfstep") || TestMode==TEXT("mecanum_lower") || TestMode.StartsWith(TEXT("mecanum_cargo_"))));
    if(IsPalletPerceptionTest()&&TestMode.EndsWith(TEXT("box")))bPayload=false;
    if(!bPayload) return true;
    if(!TestMode.IsEmpty())
    {
        bool Known=TestMode==TEXT("swerve_tyre_payload") || IsHydraulicTest() || IsIntegrationTest() || IsComponentTest() || TestMode.StartsWith(TEXT("mecanum_"));
        for(const TCHAR* Case:{TEXT("lift"),TEXT("heavy"),TEXT("eccentric"),TEXT("holes"),TEXT("entry"),TEXT("miss"),TEXT("shallow"),TEXT("carry"),TEXT("slip"),TEXT("unload"),TEXT("reset"),TEXT("loose"),TEXT("turn"),TEXT("turn_left"),TEXT("turn_start"),TEXT("turn_high"),TEXT("s_curve"),TEXT("cargo_hold"),TEXT("cargo_entry"),TEXT("cargo_unload"),TEXT("cargo_accel"),TEXT("cargo_accel_low"),TEXT("cargo_brake"),TEXT("cargo_brake_low"),TEXT("cargo_corner"),TEXT("cargo_corner_low"),TEXT("cargo_tip"),TEXT("cargo_drop")})
            Known |= TestMode==FString(TEXT("swerve_payload_"))+Case;
        if(!Known) { Error=TEXT("Unknown or incompatible payload test"); return false; }
    }
    if(!Settings.bSwerve) { Error=TEXT("Payload requires swerve rig"); return false; }
    if(!PayloadSettings.Load(Error)) return false;
    Settings.MaxSpeedMps=1;
    if(IsComponentTest()) PayloadSettings.InitialDepthM=.9;
    if(TestMode.StartsWith(TEXT("mecanum_")))
    {
        PayloadSettings.InitialDepthM=.9;PayloadSettings.bCargoSecured=false;
        if(IsRackTest())PayloadSettings.InitialDepthM=1.1;
        if(TestMode==TEXT("mecanum_eccentric_lift")) PayloadSettings.CargoYM=.12;
        if(TestMode==TEXT("mecanum_shallow_lift")) PayloadSettings.InitialDepthM=.65;
        if(TestMode==TEXT("mecanum_cargo_slide")) {PayloadSettings.CargoStaticMu=.025;PayloadSettings.CargoDynamicMu=.02;}
        if(TestMode==TEXT("mecanum_cargo_drop")) {PayloadSettings.CargoStaticMu=.015;PayloadSettings.CargoDynamicMu=.01;}
    }
    if(TestMode.StartsWith(TEXT("swerve_payload_")))
    {
        // Test fixtures do not depend on the user's experimental JSON values.
        PayloadSettings=FPayloadSettings(); PayloadSettings.InitialDepthM=1.1;
        // Historical fixtures explicitly represent secured cargo. Manual mode
        // and the new cargo_* suite default to physical, unrestrained contact.
        PayloadSettings.bCargoSecured=!IsCargoTest();
        if(TestMode==TEXT("swerve_payload_heavy")) PayloadSettings.CargoMassKg=600;
        if(TestMode==TEXT("swerve_payload_eccentric")) PayloadSettings.CargoYM=.12;
        if(TestMode==TEXT("swerve_payload_holes")) PayloadSettings.HoleShiftYM=PayloadSettings.ForkShiftYM=.08;
        if(TestMode==TEXT("swerve_payload_shallow")) PayloadSettings.InitialDepthM=.3;
        if(TestMode==TEXT("swerve_payload_entry") || TestMode==TEXT("swerve_payload_miss")) PayloadSettings.InitialDepthM=-.25;
        if(TestMode==TEXT("swerve_payload_miss")) { PayloadSettings.HoleWidthM=.18; PayloadSettings.HoleShiftYM=.12; }
        if(TestMode==TEXT("swerve_payload_slip")) PayloadSettings.LeftHoleMu=PayloadSettings.RightHoleMu=.01;
        if(TestMode==TEXT("swerve_payload_loose")) PayloadSettings.bCargoSecured=false;
        if(IsCargoTest()) ConfigureCargoTest();
    }
    if(TestMode==TEXT("swerve_tyre_payload")) { PayloadSettings=FPayloadSettings(); PayloadSettings.InitialDepthM=1.1; }
    if(IsHydraulicTest())
    {
        PayloadSettings=FPayloadSettings(); PayloadSettings.InitialDepthM=1.1;
        // Overload uses a lower relief setting, not a dangerously tipping mass.
    }
    if(Settings.bReferenceGeometry && !PayloadSettings.UseWarehouseGeometry(Error))return false;
    if(IsPalletPerceptionTest()||IsHandlingTest()||IsMissionTest())PayloadSettings.InitialDepthM=-.25;
    if(IsHandlingTest()&&TestMode.Contains(TEXT("place")))PayloadSettings.InitialDepthM=1.1;
    return PayloadSettings.Validate(Error);
}
void AStabilityLab::SpawnPayload()
{
    const auto& S=PayloadSettings;
    const double TipX=Fork->Tines.Num()>0?Fork->Tines[0]->Tip().X:Fork->GetActorLocation().X+135;
    FVector Spawn(TipX-S.InitialDepthM*100+S.LengthM*50,S.PalletYM*100,(S.bWarehouseAssets?S.PalletDatumM.Z+.002:S.HoleHeightM+.03)*100);
    if(IsRackTest() && IsRackPickup())Spawn=FVector(RackSettings.X*100,RackSettings.Y*100,(RackTierM()+S.PalletDatumM.Z+.002)*100);
    if(IsHandlingTest()&&!TestMode.Contains(TEXT("place"))&&(TestMode.EndsWith(TEXT("low"))||TestMode.EndsWith(TEXT("high"))))Spawn=FVector(RackSettings.X*100,RackSettings.Y*100,((TestMode.EndsWith(TEXT("high"))?RackSettings.High:RackSettings.Low)+S.PalletDatumM.Z+.002)*100);
    if(IsHandlingTest()&&TestMode.EndsWith(TEXT("align")))Spawn.Y+=8;
    if(IsMissionTest()&&FParse::Param(FCommandLine::Get(),TEXT("MissionFixture")))Spawn.X+=300;
    FRotator InitialRotation=FRotator::ZeroRotator;
    if(IsPalletPerceptionTest()){if(TestMode.EndsWith(TEXT("shift")))Spawn.Y+=15;if(TestMode.EndsWith(TEXT("high")))Spawn.Z+=180;if(TestMode.EndsWith(TEXT("yaw")))InitialRotation.Yaw=10;}
    Pallet=GetWorld()->SpawnActor<APalletLoadActor>(Spawn,InitialRotation);
    Pallet->Configure(S);
    CargoState.InitialRelativeM=Pallet->Deck->GetComponentQuat().UnrotateVector(Pallet->Cargo->GetActorLocation()-Pallet->GetActorLocation())/100.;
    PayloadState.Start=Pallet->GetActorLocation();
}
void AStabilityLab::AccumulatePayload(const FContactLoadOutput& O)
{
    auto& P=PayloadState; P.GroundN+=O.PalletGroundNs;
    for(int I=0;I<2;++I)
    {
        P.VerticalN[I]+=O.ForkVerticalNs[I]; P.NormalN[I]+=O.ForkNormalNs[I];
        P.FrictionN[I]+=O.ForkFrictionNs[I]; P.PointWeighted[I]+=O.ForkPointWeighted[I];
    }
}
void AStabilityLab::UpdatePayload(double Dt,double PhysicsDt)
{
    auto& P=PayloadState;
    const auto RelativeToFork=[&](FVector World) { return Fork->Carriage->GetComponentQuat().UnrotateVector(World-Fork->GetActorLocation())/100.; };
    P.LoadCenterM=RelativeToFork(Pallet->LoadCOM())+FVector(Settings.bReferenceGeometry?-.03:.05,0,0);
    P.HeightM=(Pallet->GetActorLocation().Z-P.Start.Z)/100;
    P.TiltDeg=FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Pallet->Deck->GetUpVector().Z,-1.,1.)));
    const FVector Relative=RelativeToFork(Pallet->GetActorLocation());
    if(P.HeightM>.15 && !P.bSeeded) { P.bSeeded=true; P.InitialRelative=Relative; }
    if(P.bSeeded && P.HeightM>.15) P.MaxSlip=FMath::Max(P.MaxSlip,FVector2D(Relative-P.InitialRelative).Size());
    if(TimeS>2) P.MaxTilt=FMath::Max(P.MaxTilt,P.TiltDeg);
    const bool SCurve=TestMode==TEXT("swerve_payload_s_curve");
    if((TestMode.StartsWith(TEXT("swerve_payload_turn")) && TimeS>10 && TimeS<14) || (SCurve && TimeS>11 && TimeS<27))
    {
        P.TurnTimeS+=Dt; P.TurnDistanceM+=FMath::Abs(SpeedMps)*Dt;
        P.TurnYawIntegral+=YawDegS*Dt; P.MinTurnSpeedMps=FMath::Min(P.MinTurnSpeedMps,FMath::Abs(SpeedMps));
    }
    if(SCurve) for(int I=0;I<3;++I) if(TimeS>12+6*I && TimeS<14+6*I)
    { P.SCurveSampleS[I]+=Dt; P.SCurveYawIntegral[I]+=YawDegS*Dt; }
    for(int I=0;I<2;++I)
    {
        // Geometric overlap is NOT proof of contact or successful insertion.
        const auto Local=[&](FVector W) { return Pallet->Deck->GetComponentQuat().UnrotateVector(W-Pallet->GetActorLocation())/100.; };
        const double A=Local(Fork->Tines[I]->Heel()).X;
        const double B=Local(Fork->Tines[I]->Tip()).X;
        P.DepthM[I]=FMath::Max(0.,FMath::Min(FMath::Max(A,B),PayloadSettings.LengthM*.5)-FMath::Max(FMath::Min(A,B),-PayloadSettings.LengthM*.5));
        P.CopXM[I]=P.NormalN[I]>1e-8?RelativeToFork(P.PointWeighted[I]/P.NormalN[I]*100).X+(Settings.bReferenceGeometry?-.03:.05):0;
        if(PhysicsDt>0) { P.VerticalN[I]/=PhysicsDt; P.NormalN[I]/=PhysicsDt; P.FrictionN[I]/=PhysicsDt; }
        P.FilteredN[I]+=(P.VerticalN[I]-P.FilteredN[I])*(1-FMath::Exp(-Dt/.12));
    }
    if(PhysicsDt>0) P.GroundN/=PhysicsDt;
    if(TimeS>(TestMode==TEXT("swerve_payload_unload")?16:12) && PhysicsDt>0)
    {
        P.SampleS+=PhysicsDt; P.WheelIntegral+=TotalNormalLoadN*PhysicsDt;
        P.FrontIntegral+=(Wheels[0]->NormalLoadN+Wheels[1]->NormalLoadN)*PhysicsDt;
        for(int I=0;I<2;++I) P.ForkIntegral[I]+=P.VerticalN[I]*PhysicsDt;
        FVector Weighted=FVector::ZeroVector; double Mass=0;
        auto Add=[&](UStaticMeshComponent* C) { Weighted+=C->GetCenterOfMass()*C->GetMass(); Mass+=C->GetMass(); };
        Add(Chassis->Body); Add(Fork->Carriage); for(AForkTineActor* T:Fork->Tines) for(UStaticMeshComponent* B:T->Sections) Add(B);
        if(Fork->MovingMast) Add(Fork->MovingMast);
        for(AWheelActor* W:Wheels) { for(UStaticMeshComponent* B:W->PhysicalBodies()) Add(B); } Add(Pallet->Deck); Add(Pallet->Cargo->GetStaticMeshComponent());
        // Reference static front/rear moment balance, not a force applied to wheels.
        const double X=Chassis->Body->GetComponentQuat().UnrotateVector(Weighted/Mass-Chassis->GetActorLocation()).X/100;
        P.ExpectedFrontIntegral+=(.5+X/Settings.WheelbaseM)*PhysicsDt;
    }
}
void AStabilityLab::ControlPayloadTest(double Dt)
{
    if(IsCargoTest()) { ControlCargoTest(Dt); return; }
    if(TestMode==TEXT("swerve_payload_reset") && TimeS>6 && !bPayloadResetTestDone)
    { bPayloadResetTestDone=true; ResetRig(); UE_LOG(LogTemp,Display,TEXT("PAYLOAD_RESET recreated all bodies and sampler")); return; }
    Throttle=Steering=Strafe=Brake=Lift=0;
    const bool Entry=TestMode==TEXT("swerve_payload_entry"), Miss=TestMode==TEXT("swerve_payload_miss");
    const double LiftAt=Entry?6:2;
    if((Entry || Miss) && TimeS>2 && TimeS<5.5) Throttle=.4;
    if(Miss) return;
    const bool Unloading=TestMode==TEXT("swerve_payload_unload") && TimeS>=9;
    const double LiftHeight=TestMode==TEXT("swerve_payload_turn_high")?1.2:.4;
    LiftTargetM=FMath::FInterpConstantTo(LiftTargetM,TimeS>LiftAt && !Unloading?LiftHeight:0,Dt,Settings.ForkSpeedMps);
    if(TestMode==TEXT("swerve_payload_unload"))
    {
        if(TimeS>6 && TimeS<9) Throttle=1;
        if(TimeS>13 && TimeS<16) Throttle=-1;
    }
    if(TestMode==TEXT("swerve_payload_carry") || TestMode==TEXT("swerve_payload_slip"))
    { if(TimeS>6 && TimeS<10) Throttle=1; if(TimeS>=10 && TimeS<12) Brake=1; }
    if(TestMode.StartsWith(TEXT("swerve_payload_turn")))
    {
        if(TimeS>7 && TimeS<14) Throttle=1;
        if(TimeS>(TestMode==TEXT("swerve_payload_turn_start")?7:9) && TimeS<14)
            Steering=TestMode==TEXT("swerve_payload_turn_left")?-1:1;
    }
    if(TestMode==TEXT("swerve_payload_s_curve"))
    {
        if(TimeS>7 && TimeS<27) Throttle=1;
        if(TimeS>9 && TimeS<27) Steering=TimeS<15?1:TimeS<21?-1:1;
    }
}
bool AStabilityLab::CheckPayloadTest()
{
    if(IsCargoTest()) return CheckCargoTest();
    const auto& P=PayloadState;
    const double T=FMath::Max(P.SampleS,1e-6), Weight=Pallet->LoadMass()*9.81;
    const double L=P.ForkIntegral[0]/T,R=P.ForkIntegral[1]/T, Wheel=P.WheelIntegral/T;
    const double Front=P.FrontIntegral/FMath::Max(P.WheelIntegral,1e-6), ExpectedFront=P.ExpectedFrontIntegral/T;
    bool Pass=FMath::Abs(Pallet->LoadMass()-PayloadSettings.PalletMassKg-PayloadSettings.CargoMassKg)<.1;
    const double Moved=(Pallet->GetActorLocation()-P.Start).Size2D()/100;
    if(TestMode.StartsWith(TEXT("swerve_payload_turn")))
    {
        const double Direction=TestMode==TEXT("swerve_payload_turn_left")?-1:1;
        Pass &= P.TurnTimeS>3.8 && P.MinTurnSpeedMps>.65 && P.TurnDistanceM/P.TurnTimeS>.75 && Direction*P.TurnYawIntegral/P.TurnTimeS>25 && P.HeightM>.15 && P.MaxSlip<.15;
        UE_LOG(LogTemp,Display,TEXT("PAYLOAD_TURN min_speed=%.4f mean_speed=%.4f mean_yaw=%.3f duration=%.3f"),P.MinTurnSpeedMps,P.TurnDistanceM/FMath::Max(P.TurnTimeS,1e-6),P.TurnYawIntegral/FMath::Max(P.TurnTimeS,1e-6),P.TurnTimeS);
    }
    else if(TestMode==TEXT("swerve_payload_s_curve"))
    {
        Pass &= P.TurnTimeS>15.8 && P.MinTurnSpeedMps>.5 && P.TurnDistanceM/P.TurnTimeS>.75 && P.HeightM>.15 && P.MaxSlip<.15;
        for(int I=0;I<3;++I) Pass &= P.SCurveSampleS[I]>1.8 && (I==1?-1:1)*P.SCurveYawIntegral[I]/FMath::Max(P.SCurveSampleS[I],1e-6)>15;
        UE_LOG(LogTemp,Display,TEXT("PAYLOAD_S_CURVE min_speed=%.4f mean_speed=%.4f yaw_phases=%.3f/%.3f/%.3f duration=%.3f"),P.MinTurnSpeedMps,P.TurnDistanceM/FMath::Max(P.TurnTimeS,1e-6),P.SCurveYawIntegral[0]/FMath::Max(P.SCurveSampleS[0],1e-6),P.SCurveYawIntegral[1]/FMath::Max(P.SCurveSampleS[1],1e-6),P.SCurveYawIntegral[2]/FMath::Max(P.SCurveSampleS[2],1e-6),P.TurnTimeS);
    }
    else if(TestMode==TEXT("swerve_payload_shallow")) Pass &= P.MaxTilt>10;
    else if(TestMode==TEXT("swerve_payload_miss")) Pass &= FMath::Max(P.DepthM[0],P.DepthM[1])<.25 && P.HeightM<.05;
    else if(TestMode==TEXT("swerve_payload_slip")) Pass &= P.MaxSlip>.08;
    else if(TestMode==TEXT("swerve_payload_unload"))
        Pass &= P.SampleS>3 && FMath::Abs(Wheel-MassKg*9.81)/(MassKg*9.81)<.02 && FMath::Abs(L+R)<20 && FMath::Abs(P.HeightM)<.01 && FMath::Max(P.DepthM[0],P.DepthM[1])<.1 && Moved>1;
    else
    {
        Pass &= P.SampleS>3 && FMath::Abs(L+R-Weight)/Weight<.05 && FMath::Abs(Wheel-(MassKg+Pallet->LoadMass())*9.81)/((MassKg+Pallet->LoadMass())*9.81)<.05;
        Pass &= P.HeightM>.2 && P.TiltDeg<5 && FMath::Abs(Front-ExpectedFront)<.05;
        if(TestMode==TEXT("swerve_payload_eccentric")) Pass &= R>L*1.3;
        if(TestMode==TEXT("swerve_payload_holes")) Pass &= L>R*1.3;
        if(TestMode==TEXT("swerve_payload_carry")) Pass &= Moved>1 && P.MaxSlip<.1;
        if(TestMode==TEXT("swerve_payload_entry")) Pass &= P.DepthM[0]>.8 && P.DepthM[1]>.8;
        if(TestMode==TEXT("swerve_payload_reset")) Pass &= bPayloadResetTestDone;
        // Static contact-point moment balance: aggregate CoP must follow load COM.
        const double Cop=(P.CopXM[0]*P.NormalN[0]+P.CopXM[1]*P.NormalN[1])/FMath::Max(P.NormalN[0]+P.NormalN[1],1e-6);
        Pass &= FMath::Abs(Cop-P.LoadCenterM.X)<.05;
    }
    UE_LOG(LogTemp,Display,TEXT("PAYLOAD_CHECK %s mass=%.2f fork_n=%.2f/%.2f expected_n=%.2f wheels_n=%.2f expected_wheels=%.2f front=%.4f/%.4f lift=%.3f depth=%.3f/%.3f tilt=%.2f max_tilt=%.2f slip=%.4f moved=%.3f %s"),*TestMode,Pallet->LoadMass(),L,R,Weight,Wheel,(MassKg+Pallet->LoadMass())*9.81,Front,ExpectedFront,P.HeightM,P.DepthM[0],P.DepthM[1],P.TiltDeg,P.MaxTilt,P.MaxSlip,Moved,Pass?TEXT("PASS"):TEXT("FAIL"));
    return Pass;
}
FString AStabilityLab::PayloadCsv(bool Header) const
{
    if(Header) return TEXT(",payload_mass_kg,payload_com_heel_x_m,payload_com_y_m,payload_com_z_m,pallet_lift_m,pallet_tilt_deg,pallet_ground_n,left_depth_m,right_depth_m,left_fork_vertical_n,right_fork_vertical_n,left_contact_normal_n,right_contact_normal_n,left_friction_n,right_friction_n,left_cop_heel_x_m,right_cop_heel_x_m,max_relative_slip_m,hole_shift_y_m,hole_spacing_m,fork_shift_y_m,fork_spacing_m,fork_mu,left_hole_mu,right_hole_mu,cargo_secured");
    const auto& P=PayloadState; const auto& S=PayloadSettings;
    return FString::Printf(TEXT(",%.3f,%.5f,%.5f,%.5f,%.5f,%.4f,%.3f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d"),Pallet->LoadMass(),P.LoadCenterM.X,P.LoadCenterM.Y,P.LoadCenterM.Z,P.HeightM,P.TiltDeg,P.GroundN,P.DepthM[0],P.DepthM[1],P.VerticalN[0],P.VerticalN[1],P.NormalN[0],P.NormalN[1],P.FrictionN[0],P.FrictionN[1],P.CopXM[0],P.CopXM[1],P.MaxSlip,S.HoleShiftYM,S.HoleSpacingM,S.ForkShiftYM,S.ForkSpacingM,S.ForkMu,S.LeftHoleMu,S.RightHoleMu,S.bCargoSecured?1:0);
}
void AStabilityHUD::DrawPayload(AStabilityLab* B,float S)
{
    const auto& P=B->PayloadState; const auto& C=B->PayloadSettings;
    const float X=Canvas->SizeX/S-376;
    DrawRect(FLinearColor(.02f,.03f,.05f,.94f),X*S,16*S,360*S,517*S);
    float Y=32;
    const FLinearColor Ink(.9f,.95f,1), Accent(.15f,.9f,.75f);
    auto Line=[&](FString T,FLinearColor Color) { DrawText(T,Color,(X+12)*S,Y*S,GEngine->GetSmallFont(),S,false); Y+=25; };
    Line(TEXT("PALLET / REAL CONTACT (not a sensor)"),Accent);
    Line(FString::Printf(TEXT("Pallet + cargo  %.0f + %.0f kg"),C.PalletMassKg,C.CargoMassKg),Ink);
    Line(FString::Printf(TEXT("Load center heel X/Y  %.2f / %+.2f m"),P.LoadCenterM.X,P.LoadCenterM.Y),Ink);
    Line(FString::Printf(TEXT("Overlap L/R  %.2f / %.2f m (geometry)"),P.DepthM[0],P.DepthM[1]),Ink);
    Line(FString::Printf(TEXT("Fork support L/R  %.2f / %.2f kN"),P.FilteredN[0]/1000,P.FilteredN[1]/1000),Accent);
    Line(FString::Printf(TEXT("Contact CoP heel X  %.2f / %.2f m"),P.CopXM[0],P.CopXM[1]),Ink);
    Line(FString::Printf(TEXT("Pallet lift / tilt  %.2f m / %.1f deg"),P.HeightM,P.TiltDeg),Ink);
    Line(FString::Printf(TEXT("Contact mu L/R  %.2f / %.2f (MIN)"),FMath::Min(C.ForkMu,C.LeftHoleMu),FMath::Min(C.ForkMu,C.RightHoleMu)),Ink);
    Line(FString::Printf(TEXT("Total weight incl. cargo  %.2f kN"),(B->MassKg+B->Pallet->LoadMass())*9.81/1000),Ink);
    Line(C.bCargoSecured?TEXT("Cargo strapped to PALLET, never to forks"):TEXT("LOOSE cargo: deck friction only"),Accent);
    Line(TEXT("W insert -> release W -> E lift -> W carry"),Ink);
    Line(TEXT("Q lower | R reset | tune Config/Payload.json"),Ink);
    DrawCargo(B,S,X+12,Y+10);
}
