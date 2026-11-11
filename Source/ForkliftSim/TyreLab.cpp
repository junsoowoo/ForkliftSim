#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "WheelActor.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Engine/StaticMeshActor.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

bool AStabilityLab::IsTyreTest() const { return TestMode.StartsWith(TEXT("swerve_tyre_")); }
bool AStabilityLab::ConfigureTyres()
{
    if(IsTyreTest())
    {
        bool Known=false;
        for(const TCHAR* N:{TEXT("settle"),TEXT("soft"),TEXT("stiff"),TEXT("drop"),TEXT("drop_low"),TEXT("step"),TEXT("step_rigid"),TEXT("roll"),TEXT("payload")})
            Known |= TestMode==FString(TEXT("swerve_tyre_"))+N;
        if(!Known) { Error=TEXT("Unknown tyre test"); return false; }
        Settings.Tyres=FTyreComplianceSettings();
        if(TestMode.EndsWith(TEXT("soft"))) Settings.Tyres.StiffnessNpm=300000;
        if(TestMode.EndsWith(TEXT("stiff"))) Settings.Tyres.StiffnessNpm=1200000;
        if(TestMode.EndsWith(TEXT("drop_low"))) Settings.Tyres.DampingNsPm=1500;
        if(TestMode.EndsWith(TEXT("step_rigid"))) Settings.Tyres.bEnabled=false;
        TerrainPreset=TestMode.Contains(TEXT("step"))?TEXT("step"):TEXT("flat");
    }
    if(FParse::Param(FCommandLine::Get(),TEXT("RigidTyres"))) Settings.Tyres.bEnabled=false;
    if(FParse::Param(FCommandLine::Get(),TEXT("DeformableTread"))) Settings.Tyres.Tread.bEnabled=true;
    if(FParse::Param(FCommandLine::Get(),TEXT("CoupledTread"))) {Settings.Tyres.Tread.bEnabled=true;Settings.Tyres.Tread.bCoupled=true;}
    if(FParse::Param(FCommandLine::Get(),TEXT("CoupledTread")) && FParse::Param(FCommandLine::Get(),TEXT("RigidTread"))) {Error=TEXT("CoupledTread and RigidTread are mutually exclusive");return false;}
    if(FParse::Param(FCommandLine::Get(),TEXT("RigidTread"))) Settings.Tyres.Tread.bEnabled=false;
    if(TestMode.StartsWith(TEXT("swerve_components_tread_"))) Settings.Tyres.Tread.bEnabled=true;
    int Segments=0;
    if(FParse::Value(FCommandLine::Get(),TEXT("TreadSegments="),Segments))
    {
        if(!Settings.Tyres.Tread.bEnabled || (Segments!=16 && Segments!=32 && !(Settings.Tyres.Tread.bCoupled && (Segments==48 || Segments==64))))
        { Error=TEXT("TreadSegments requires deformable tread; coupled profile supports 16/32/48/64"); return false; }
        Settings.Tyres.Tread.Count=Segments;
    }
    if(TestMode==TEXT("swerve_components_tread_soft")) Settings.Tyres.Tread.FoundationNpm3*=.5;
    if(TestMode==TEXT("swerve_components_tread_stiff")) Settings.Tyres.Tread.FoundationNpm3*=2;
    Settings.Tyres.bEnabled &= Settings.bSwerve;
    if(!Settings.Tyres.bEnabled || !Settings.Tyres.bDetailed) Settings.Tyres.Tread.bEnabled=false;
    Settings.Tyres.Tread.bCoupled &= Settings.Tyres.Tread.bEnabled;
    if(Settings.bMecanum) {Settings.Tyres.bEnabled=false;Settings.Tyres.Tread.bEnabled=false;Settings.Tyres.Tread.bCoupled=false;}
    if(Settings.Tyres.Tread.bEnabled && (Settings.Tyres.Tread.DepthM>=Settings.RadiusM*.8 || Settings.Tyres.Tread.MassKg>=Settings.WheelMassKg-1)) return false;
    bTyreView=IsTyreTest() || FParse::Param(FCommandLine::Get(),TEXT("TyreLab"));
    return Settings.Tyres.Valid();
}
void AStabilityLab::ControlTyreTest()
{
    Throttle=Strafe=Steering=Brake=Lift=0;
    const bool Moving=TestMode.Contains(TEXT("step")) || TestMode.EndsWith(TEXT("roll"));
    bNeutral=!Moving; bResistanceEnabled=Moving;
    if(!Moving) Brake=1; // passive mechanical brake isolates vertical bench motion
    if(Moving && TimeS>2) Throttle=TestMode.EndsWith(TEXT("roll"))?.4:.35;
    if(!Moving) for(AWheelActor* W:Wheels)
    {
        // Passive tyre fixtures do not inject steering-servo work into the rig.
        W->SteeringJoint->SetOrientationDriveTwistAndSwing(false,false);
        W->SteeringJoint->SetAngularVelocityDriveTwistAndSwing(false,false);
    }
    LiftTargetM=FMath::FInterpConstantTo(LiftTargetM,bPayload && TimeS>2?.6:0,GetWorld()->GetDeltaSeconds(),Settings.ForkSpeedMps);
}
double AStabilityLab::TyreMechanicalEnergyJ() const
{
    double E=0;
    auto Body=[&](UStaticMeshComponent* B)
    {
        const FVector V=B->GetPhysicsLinearVelocity()/100.;
        const FQuat MassQ=B->GetComponentQuat()*B->BodyInstance.GetMassSpaceLocal().GetRotation();
        const FVector W=MassQ.UnrotateVector(B->GetPhysicsAngularVelocityInRadians()), I=B->GetInertiaTensor()/10000.;
        E+=.5*B->GetMass()*V.SizeSquared()+.5*FVector::DotProduct(I*W,W)+B->GetMass()*9.81*B->GetCenterOfMass().Z/100.;
    };
    if(!Chassis || !Fork) return 0;
    Body(Chassis->Body); Body(Fork->Carriage);
    for(AForkTineActor* T:Fork->Tines) { for(UStaticMeshComponent* B:T->Sections) Body(B); E+=T->ElasticEnergyJ(); }
    if(Fork->MovingMast) Body(Fork->MovingMast);
    for(AWheelActor* W:Wheels) { for(UStaticMeshComponent* B:W->PhysicalBodies()) Body(B); E+=W->SpringEnergyJ+W->BrakeElasticJ+W->ReducedContactElasticJ; }
    if(Pallet) { Body(Pallet->Deck); Body(Pallet->Cargo->GetStaticMeshComponent()); }
    if(Fork->HydraulicSettings.bEnabled) E+=Fork->HydraulicState.StoredJ(Fork->HydraulicSettings,ForkHeightM);
    else E+=.5*100000*ForkHeightM*ForkHeightM;
    return E;
}
void AStabilityLab::UpdateTyres(double Dt)
{
    auto& T=TyreState;
    const FVector V=Chassis->Body->GetPhysicsLinearVelocity()/100.;
    const double Az=Dt>0?(V.Z-T.LastVelocity.Z)/Dt:0; T.LastVelocity=V;
    T.EnergyJ=TyreMechanicalEnergyJ()-T.EnergyReferenceJ;
    T.MaxEnergyGainJ=FMath::Max(T.MaxEnergyGainJ,T.EnergyJ);
    T.MaxAccountedGainJ=FMath::Max(T.MaxAccountedGainJ,T.EnergyJ+T.DampingJ);
    for(AWheelActor* W:Wheels)
    { T.PeakCompressionM=FMath::Max(T.PeakCompressionM,W->CompressionM); T.bBottomed |= W->bTyreTravelLimit; }
    if(TimeS>.2 && TimeS<1.2) { T.VelocitySquareIntegral+=V.Z*V.Z*Dt; T.VibrationS+=Dt; }
    if(TimeS>1.8 && TimeS<2) T.InitialFrontCompressionM=(Wheels[0]->CompressionM+Wheels[1]->CompressionM)*.5;
    const double X=Chassis->GetActorLocation().X/100.;
    if(TerrainPreset==TEXT("step") && X>2.5 && X<6.5) T.PeakVerticalAccel=FMath::Max(T.PeakVerticalAccel,FMath::Abs(Az));
    if(TimeS>8)
    {
        T.SampleS+=Dt;
        for(int I=0;I<4;++I)
        { T.CompressionIntegral[I]+=Wheels[I]->CompressionM*Dt; T.NormalIntegral[I]+=Wheels[I]->NormalLoadN*Dt; T.AxleIntegral[I]+=Wheels[I]->AxleSupportN*Dt; }
    }
    if(TimeS>4 && TestMode.EndsWith(TEXT("roll"))) for(AWheelActor* W:Wheels) if(W->bRollingRadiusValid)
    { T.RollingIntegral+=W->RollingRadiusM*Dt; T.RollingS+=Dt; }
}
bool AStabilityLab::CheckTyreTest() const
{
    const auto& T=TyreState; const auto& S=Settings.Tyres;
    const bool Step=TestMode.Contains(TEXT("step")), Roll=TestMode.EndsWith(TEXT("roll"));
    const double Den=FMath::Max(T.SampleS,1e-8);
    double Comp=0, Support=0, Normal=0, MaxForceError=0;
    for(int I=0;I<4;++I)
    {
        const double D=T.CompressionIntegral[I]/Den, F=T.AxleIntegral[I]/Den, N=T.NormalIntegral[I]/Den;
        Comp+=D*.25; Support+=F; Normal+=N;
        if(S.bEnabled) MaxForceError=FMath::Max(MaxForceError,FMath::Abs(F-S.StiffnessNpm*D)/FMath::Max(F,100.));
    }
    const double Expected=(MassKg+(bPayload?Pallet->LoadMass():0))*9.81;
    const double Vibration=FMath::Sqrt(T.VelocitySquareIntegral/FMath::Max(T.VibrationS,1e-8));
    const double R=T.RollingIntegral/FMath::Max(T.RollingS,1e-8);
    bool Pass=T.SampleS>7.5 && FMath::IsFinite(T.EnergyJ) && !T.bBottomed && Grounded==4;
    if(Step) Pass &= Chassis->GetActorLocation().X>800 && T.PeakVerticalAccel>1;
    else
    {
        Pass &= FMath::Abs(Normal-Expected)/Expected<.05 && Comp>.0005 && MaxForceError<.06;
        Pass &= FMath::Abs((Normal-4*(Settings.WheelMassKg+Settings.KnuckleMassKg*.2)*9.81)-Support)/Expected<.05;
        if(Roll) Pass &= T.RollingS>10 && FMath::Abs(R-Settings.RadiusM)<.015;
        else if(bPayload) Pass &= PayloadState.HeightM>.25 && !CargoState.bFell && (Wheels[0]->CompressionM+Wheels[1]->CompressionM)*.5>T.InitialFrontCompressionM+.002;
        else Pass &= FMath::Abs(Chassis->Body->GetPhysicsLinearVelocity().Z)<2 && T.MaxEnergyGainJ<25 && T.MaxAccountedGainJ<40;
    }
    UE_LOG(LogTemp,Display,TEXT("TYRE_CHECK %s mean_compression_mm=%.4f peak_compression_mm=%.4f force_error=%.5f normal_n=%.2f axle_n=%.2f expected_n=%.2f vibration_mps=%.6f step_accel_mps2=%.3f rolling_radius_m=%.5f damping_j=%.3f max_energy_gain_j=%.3f max_accounted_gain_j=%.3f bottomed=%d %s"),*TestMode,Comp*1000,T.PeakCompressionM*1000,MaxForceError,Normal,Support,Expected,Vibration,T.PeakVerticalAccel,R,T.DampingJ,T.MaxEnergyGainJ,T.MaxAccountedGainJ,T.bBottomed?1:0,Pass?TEXT("PASS"):TEXT("FAIL"));
    return Pass;
}
FString AStabilityLab::TyreCsv(bool Header) const
{
    FString R;
    for(int I=0;I<4;++I)
    {
        if(Header)
        {
            const TCHAR* N=I==0?TEXT("FL"):I==1?TEXT("FR"):I==2?TEXT("RL"):TEXT("RR");
            R+=FString::Printf(TEXT(",%s_tyre_compression_m,%s_tyre_rate_mps,%s_axle_support_n,%s_tyre_force_estimate_n,%s_tyre_spring_j,%s_loaded_radius_m,%s_observed_roll_radius_m,%s_roll_radius_valid,%s_tyre_travel_limit"),N,N,N,N,N,N,N,N,N);
        }
        else
        {
            const AWheelActor* W=Wheels[I];
            R+=FString::Printf(TEXT(",%.6f,%.6f,%.3f,%.3f,%.5f,%.6f,%.6f,%d,%d"),W->CompressionM,W->CompressionRateMps,W->AxleSupportN,W->SpringEstimateN,W->SpringEnergyJ,W->LoadedRadiusM,W->RollingRadiusM,W->bRollingRadiusValid?1:0,W->bTyreTravelLimit?1:0);
        }
        const FString N=I==0?TEXT("FL"):I==1?TEXT("FR"):I==2?TEXT("RL"):TEXT("RR");
        if(Header) for(const TCHAR* Field:{TEXT("point_normal_n"),TEXT("band_negative_n"),TEXT("band_center_n"),TEXT("band_positive_n"),TEXT("contact_center_x_m"),TEXT("contact_center_y_m"),TEXT("contact_center_z_m"),TEXT("normal_moment_x_nm"),TEXT("normal_moment_y_nm"),TEXT("normal_moment_z_nm"),TEXT("peak_active_points"),TEXT("contact_center_valid")})
            R+=TEXT(",")+N+TEXT("_")+Field;
        else
        {
            const auto& C=Wheels[I]->ContactData; const double D=PreviousPhysicsDt>0?PreviousPhysicsDt:1;
            const bool Valid=PreviousPhysicsDt>0 && C.NormalNs>1e-8;
            const FVector P=Valid?C.PointWeightedMNs/C.NormalNs:FVector::ZeroVector, M=C.NormalMomentNms/D;
            R+=FString::Printf(TEXT(",%.5f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d"),C.NormalNs/D,C.BandNs[0]/D,C.BandNs[1]/D,C.BandNs[2]/D,P.X,P.Y,P.Z,M.X,M.Y,M.Z,C.PeakActivePoints,Valid?1:0);
        }
    }
    for(int I=0;I<4;++I)
    {
        const AWheelActor* W=Wheels[I]; const FString N=I==0?TEXT("FL"):I==1?TEXT("FR"):I==2?TEXT("RL"):TEXT("RR");
        if(Header) for(const TCHAR* F:{TEXT("tread_enabled"),TEXT("tread_segments"),TEXT("tread_min_compression_m"),TEXT("tread_max_compression_m"),TEXT("tread_stored_j"),TEXT("tread_radial_vertical_n"),TEXT("tread_bottomed"),TEXT("aggregate_radial_spring")}) R+=TEXT(",")+N+TEXT("_")+F;
        else R+=FString::Printf(TEXT(",%d,%d,%.7f,%.7f,%.7f,%.5f,%d,%d"),W->TreadPads.Num()>0?1:0,W->TreadPads.Num(),W->TreadMinCompressionM,W->TreadMaxCompressionM,W->TreadStoredJ,W->TreadSupportN,W->bTreadBottomed?1:0,W->Compliance.bEnabled && W->TreadPads.IsEmpty()?1:0);
        for(int P=0;P<W->TreadPads.Num();++P)
        {
            if(Header) R+=FString::Printf(TEXT(",%s_pad_%02d_compression_m,%s_pad_%02d_rate_mps,%s_pad_%02d_normal_n"),*N,P,*N,P,*N,P);
            else R+=FString::Printf(TEXT(",%.7f,%.7f,%.5f"),W->TreadCompressionM[P],W->TreadRateMps[P],W->TreadNormalN[P]);
        }
        if(W->Compliance.Tread.bCoupled && !W->TreadPads.IsEmpty())
        {
            if(Header) for(const TCHAR* F:{TEXT("tread_coupled"),TEXT("tread_max_shear_m"),TEXT("tread_max_rotation_rad"),TEXT("tread_shear_j"),TEXT("tread_rotation_j"),TEXT("tread_link_j")}) R+=TEXT(",")+N+TEXT("_")+F;
            else R+=FString::Printf(TEXT(",1,%.7f,%.7f,%.7f,%.7f,%.7f"),W->TreadMaxShearM,W->TreadMaxRotationRad,W->TreadShearJ,W->TreadRotationJ,W->TreadLinkJ);
            for(int P=0;P<W->TreadPads.Num();++P)
            {
                if(Header) R+=FString::Printf(TEXT(",%s_pad_%02d_shear_m,%s_pad_%02d_rotation_rad"),*N,P,*N,P);
                else R+=FString::Printf(TEXT(",%.7f,%.7f"),W->TreadShearM[P],W->TreadRotationRad[P]);
            }
        }
    }
    if(Header) return R+TEXT(",tyre_compliance_enabled,tyre_k_npm,tyre_c_nspm,tyre_travel_m,tyre_dissipated_j,rig_mechanical_delta_j,rig_plus_tyre_damping_j,chassis_vz_mps");
    return R+FString::Printf(TEXT(",%d,%.1f,%.1f,%.5f,%.5f,%.5f,%.5f,%.6f"),Settings.Tyres.bEnabled?1:0,Settings.Tyres.StiffnessNpm,Settings.Tyres.DampingNsPm,Settings.Tyres.TravelM,TyreState.DampingJ,TyreState.EnergyJ,TyreState.EnergyJ+TyreState.DampingJ,TyreState.LastVelocity.Z);
}
void AStabilityHUD::DrawTyres(AStabilityLab* B,float S)
{
    const float X=Canvas->SizeX/S-376; float Y=32;
    DrawRect(FLinearColor(.02,.03,.05,.94),X*S,16*S,360*S,568*S);
    const FLinearColor Ink(.9,.95,1), Accent(.15,.9,.75), Warn(1,.45,.15);
    auto Line=[&](FString Text,FLinearColor Color) { DrawText(Text,Color,(X+12)*S,Y*S,GEngine->GetSmallFont(),S,false); Y+=25; };
    Line(B->Settings.Tyres.Tread.bCoupled?TEXT("0.15 / COUPLED ELASTIC RING"):B->Settings.Tyres.Tread.bEnabled?TEXT("0.15 / DISCRETE RADIAL TREAD"):TEXT("0.9 / TYRE VERTICAL COMPLIANCE"),Accent);
    Line(TEXT("J Tyre HUD | M Motor HUD | T Course/reset"),Ink);
    Line(B->Settings.Tyres.Tread.bEnabled?FString::Printf(TEXT("%d pads | %.0f kN/m | C %.0f Ns/m"),B->Settings.Tyres.Tread.Count,B->Settings.Tyres.Tread.PadK(B->Settings.RadiusM,B->Settings.WidthM)/1000,B->Settings.Tyres.Tread.PadC(B->Settings.RadiusM,B->Settings.WidthM)):FString::Printf(TEXT("K %.0f kN/m | C %.0f Ns/m | %s"),B->Settings.Tyres.StiffnessNpm/1000,B->Settings.Tyres.DampingNsPm,B->Settings.Tyres.bEnabled?TEXT("ON"):TEXT("RIGID")),Accent);
    const TCHAR* Names[]={TEXT("FL"),TEXT("FR"),TEXT("RL"),TEXT("RR")};
    for(int I=0;I<4;++I)
    {
        const AWheelActor* W=B->Wheels[I];
        Line(FString::Printf(TEXT("%s compression %.2f mm | axle %.2f kN"),Names[I],W->CompressionM*1000,W->AxleSupportN/1000),W->bTyreTravelLimit?Warn:Accent);
        Line(FString::Printf(TEXT("%s %.2f kN | U %.2f J"),B->Settings.Tyres.Tread.bEnabled?TEXT("Radial-only"):TEXT("Spring+damp"),W->SpringEstimateN/1000,W->SpringEnergyJ),Ink);
        Line(FString::Printf(TEXT("Hub-ground radius %.1f mm"),W->LoadedRadiusM*1000),Ink);
        Line(W->bRollingRadiusValid?FString::Printf(TEXT("Observed v/omega radius %.1f mm"),W->RollingRadiusM*1000):TEXT("Observed radius: invalid (slip/low speed)"),Ink);
    }
    Line(B->Settings.Tyres.Tread.bEnabled?TEXT("Physical tread pads, NOT continuum rubber FEA"):B->Settings.Tyres.bDetailed?TEXT("3-axis rim/belt proxy, NOT rubber mesh FEA"):TEXT("1-axis rim/belt proxy, NOT rubber mesh FEA"),Ink);
    Line(TEXT("Road force is not added a second time"),Ink);
    Line(TEXT("Config/Tyres.json | uncalibrated assumptions"),Accent);
}
