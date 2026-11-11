#include "TreadReference.h"
#include "WheelActor.h"
#include "Vehicle3DHelpers.h"
#include "Engine/World.h"
#include "Engine/StaticMeshActor.h"
#include "EngineUtils.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "ContactLoadSampler.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "PBDRigidsSolver.h"
#include "Chaos/PBDRigidsEvolutionGBF.h"
#include "PhysicsEngine/BodySetup.h"
#include "Chaos/Convex.h"

ATreadReference::ATreadReference()
{
    PrimaryActorTick.bCanEverTick=true; PrimaryActorTick.TickGroup=TG_PrePhysics;
    Mount=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("FixedMount")); RootComponent=Mount;
}
void ATreadReference::BeginPlay()
{
    Super::BeginPlay();
    FVehicle3DSettings S; FString Error;
    if(!S.Load(Error)) { UE_LOG(LogTemp,Error,TEXT("TREAD_REFERENCE_CONFIG %s"),*Error); FGenericPlatformMisc::RequestExitWithStatus(false,1); return; }
    S.bSwerve=true; S.Tyres.bEnabled=S.Tyres.bDetailed=S.Tyres.Tread.bEnabled=true;
    const bool CoreLaw=FParse::Param(FCommandLine::Get(),TEXT("TreadCoreLawReference"));
    S.Tyres.Tread.bCoupled=CoreLaw || FParse::Param(FCommandLine::Get(),TEXT("CoupledTread"));
    bProbe=FParse::Param(FCommandLine::Get(),TEXT("TreadProbe"));
    FParse::Value(FCommandLine::Get(),TEXT("TreadReferenceNormal="),ExternalReferenceN);
    const bool ValidSpec=S.Tyres.Valid();
    int Count=S.Tyres.Tread.Count; FParse::Value(FCommandLine::Get(),TEXT("TreadCount="),Count);
    double Phase=0; FParse::Value(FCommandLine::Get(),TEXT("TreadPhaseFraction="),Phase);
    FParse::Value(FCommandLine::Get(),TEXT("TreadHubHeight="),HeightM);
    if(!ValidSpec || (Count!=16 && Count!=32 && Count!=64 && !(S.Tyres.Tread.bCoupled && (Count==48 || Count==96 || Count==128))) || !FMath::IsFinite(Phase) || Phase<0 || Phase>.5
        || (S.Tyres.Tread.bCoupled && !CoreLaw && (!FMath::IsFinite(ExternalReferenceN) || ExternalReferenceN<=0))
        || !FMath::IsFinite(HeightM) || HeightM<S.RadiusM-.02 || HeightM>S.RadiusM-.002)
    { UE_LOG(LogTemp,Error,TEXT("Invalid tread fixture geometry")); FGenericPlatformMisc::RequestExitWithStatus(false,1); return; }
    S.Tyres.Tread.Count=Count; S.Tyres.Tread.PhaseRad=Phase*2*UE_DOUBLE_PI/Count;
    RadiusM=S.RadiusM; WidthM=S.WidthM;
    Mount->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));
    Mount->SetWorldLocation(FVector(0,0,200)); Mount->SetWorldScale3D(FVector(.1));
    Mount->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics); Mount->SetCollisionResponseToAllChannels(ECR_Ignore); Mount->SetVisibility(false);
    auto* Material=Vehicle3D::Material(this,0);
    Material->SleepCounterThreshold=MAX_int32; Material->SleepLinearVelocityThreshold=Material->SleepAngularVelocityThreshold=0;
    for(TActorIterator<AStaticMeshActor> I(GetWorld());I;++I) if(I->ActorHasTag(TEXT("StabilityRoad"))) { Road=I->GetStaticMeshComponent(); Road->SetPhysMaterialOverride(Material); }
    Wheel=GetWorld()->SpawnActor<AWheelActor>(FVector(0,0,HeightM*100),FRotator::ZeroRotator);
    Wheel->Configure(Mount,S,false,Material);
    Wheel->Tyre->SetSimulatePhysics(false); // boundary fixed BEFORE first step
    for(UStaticMeshComponent* B:Wheel->PhysicalBodies()) { B->SetLinearDamping(0); B->SetAngularDamping(0); }
    if(CoreLaw)
    {
        // Isolate the actual native core constitutive law from contact and ring
        // closure. No changes to the vehicle profile or normal reference bench.
        for(UPhysicsConstraintComponent* Link:Wheel->TreadLinks) Link->BreakConstraint();
        for(int I=0;I<Wheel->TreadPads.Num();++I)
        {
            auto* P=Wheel->TreadPads[I].Get(); P->SetEnableGravity(false);
            P->SetCollisionResponseToAllChannels(ECR_Ignore);
            if(I!=0) P->SetSimulatePhysics(false);
        }
    }
    const bool Balanced=FParse::Param(FCommandLine::Get(),TEXT("TreadBalancedReference"));
    int Iterations=Balanced?96:127,Hz=480;
    FParse::Value(FCommandLine::Get(),TEXT("RingReferenceIterations="),Iterations);
    FParse::Value(FCommandLine::Get(),TEXT("RingReferenceHz="),Hz);
    if(Iterations<96 || Iterations>127 || (Hz!=240 && Hz!=480 && Hz!=960 && Hz!=1920)) {FGenericPlatformMisc::RequestExitWithStatus(false,1);return;}
    GetWorld()->GetPhysicsScene()->GetSolver()->SetPositionIterations(Iterations);
    GetWorld()->GetPhysicsScene()->GetSolver()->SetMaxDeltaTime_External(1./Hz);
    GetWorld()->GetPhysicsScene()->GetSolver()->SetVelocityIterations(Balanced?16:32);
    Sampler=GetWorld()->GetPhysicsScene()->GetSolver()->CreateAndRegisterSimCallbackObject_External<FContactLoadSampler>();
}
double ATreadReference::EnergyJ() const
{
    double E=Wheel->TreadStoredJ;
    for(UStaticMeshComponent* B:Wheel->PhysicalBodies()) if(B->IsSimulatingPhysics())
    {
        const FVector V=B->GetPhysicsLinearVelocity()/100.;
        const FQuat Q=B->GetComponentQuat()*B->BodyInstance.GetMassSpaceLocal().GetRotation();
        const FVector W=Q.UnrotateVector(B->GetPhysicsAngularVelocityInRadians()),I=B->GetInertiaTensor()/10000.;
        E+=.5*B->GetMass()*V.SizeSquared()+.5*FVector::DotProduct(I*W,W)+B->GetMass()*9.81*B->GetCenterOfMass().Z/100.;
    }
    return E;
}
void ATreadReference::Tick(float Dt)
{
    Super::Tick(Dt); if(bDone || !Wheel || !Sampler) return; TimeS+=Dt;
    if(FParse::Param(FCommandLine::Get(),TEXT("TreadCoreLawReference")))
    {
        while(auto Out=Sampler->PopOutputData_External()) MaxInnerS=FMath::Max(MaxInnerS,Out->MaxInnerDt);
        auto* In=Sampler->GetProducerInputData_External();
        In->RoadId=Road->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
        const auto& T=Wheel->Compliance.Tread; const FQuat Q=Wheel->Tyre->GetComponentQuat();
        auto* P=Wheel->TreadPads[0].Get(); Wheel->Measure(RadiusM,WidthM);
        const bool TorqueOnly=FParse::Param(FCommandLine::Get(),TEXT("TreadCoreTorqueOnly"));
        if(TimeS>1)
        {
            const FVector Radial=Q.RotateVector(FVector(FMath::Cos(T.PhaseRad),FMath::Sin(T.PhaseRad),0));
            const FVector Tangent=Q.RotateVector(FVector(-FMath::Sin(T.PhaseRad),FMath::Cos(T.PhaseRad),0));
            In->CalibrationBodyId=P->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
            In->CalibrationForceN=TorqueOnly?FVector::ZeroVector:-Radial*100+Tangent*50;
            In->CalibrationTorqueNm=Q.GetAxisZ()*.02;
        }
        if(TimeS>6)
        {
            bDone=true;
            UE_LOG(LogTemp,Display,TEXT("CORE_LAW_NUMERICS max_inner_s=%.12g position_iterations=%d"),MaxInnerS,GetWorld()->GetPhysicsScene()->GetSolver()->GetEvolution()->GetNumPositionIterations());
            const double R=100/T.PadK(RadiusM,WidthM),S=50/T.PadShearK(RadiusM,WidthM),A=.02/T.PadRotK(RadiusM,WidthM);
            const double Error=TorqueOnly?FMath::Abs(Wheel->TreadRotationRad[0]/A-1):FMath::Max3(FMath::Abs(Wheel->TreadCompressionM[0]/R-1),FMath::Abs(Wheel->TreadShearM[0]/S-1),FMath::Abs(Wheel->TreadRotationRad[0]/A-1));
            const bool Pass=Error<.01 && P->GetPhysicsLinearVelocity().Size()<.01;
            UE_LOG(LogTemp,Display,TEXT("CORE_LAW compression_m=%.9f expected_m=%.9f shear_m=%.9f expected_shear_m=%.9f rotation_rad=%.9f expected_rotation_rad=%.9f relative_error=%.8f %s"),
                Wheel->TreadCompressionM[0],R,Wheel->TreadShearM[0],S,Wheel->TreadRotationRad[0],A,Error,Pass?TEXT("PASS"):TEXT("FAIL"));
            auto* Joint=Wheel->TreadJoints[0].Get(); FVector Force,Torque; Joint->GetConstraintForce(Force,Torque);
            const FVector Anchor=(Joint->ConstraintInstance.GetRefFrame(EConstraintFrame::Frame1)*P->BodyInstance.GetUnrealWorldTransform()).GetLocation();
            UE_LOG(LogTemp,Display,TEXT("CORE_LOAD_BALANCE torque_only=%d omega_rad_s=%.9f anchor_error_m=%.12g torque_nm_last_approx=%.9f"),TorqueOnly?1:0,P->GetPhysicsAngularVelocityInRadians().Size(),(Anchor-P->GetComponentLocation()).Size()/100.,FVector::DotProduct(Torque,Q.GetAxisZ())/(10000*FMath::Max(MaxInnerS,1e-9)));
            FGenericPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
        }
        return;
    }
    double J=0,Window=0;
    while(auto Out=Sampler->PopOutputData_External()) { J+=Out->ImpulseNs[0]; Window+=Out->IntervalS; MaxInnerS=FMath::Max(MaxInnerS,Out->MaxInnerDt); }
    auto* In=Sampler->GetProducerInputData_External();
    In->RoadId=Road->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
    In->WheelIds[0]=Wheel->Tyre->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
    In->TreadCounts[0]=Wheel->TreadPads.Num();
    for(int I=0;I<Wheel->TreadPads.Num();++I) In->TreadIds[0][I]=Wheel->TreadPads[I]->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
    In->TreadPadC=Wheel->Compliance.Tread.PadC(RadiusM,WidthM);
    In->TreadPhaseRad=Wheel->Compliance.Tread.PhaseRad;
    In->bTreadCoupled=Wheel->Compliance.Tread.bCoupled;
    In->TreadShearC=Wheel->Compliance.Tread.PadShearC(RadiusM,WidthM);
    In->TreadRotC=Wheel->Compliance.Tread.PadRotC(RadiusM,WidthM);
    In->TreadLinkC=Wheel->Compliance.Tread.LinkC(RadiusM,WidthM);
    In->TreadBendC=Wheel->Compliance.Tread.BendC(RadiusM,WidthM);
    In->TreadRestRadiusM=RadiusM-Wheel->Compliance.Tread.DepthM*.5;
    Wheel->Measure(RadiusM,WidthM);
    if(TimeS>8 && Window>0) { NormalIntegral+=J; SampleS+=Window; }
    if(TimeS>3 && !bPulsed)
    {
        bPulsed=true; ReferenceJ=EnergyJ();
        // Select from actual world poses; Euler roll signs must not decide
        // whether the pulse is on an airborne or floor-supported pad.
        UWheelTyreComponent* Pad=Wheel->TreadPads[0];
        for(UWheelTyreComponent* P:Wheel->TreadPads) if(P->GetComponentLocation().Z>Pad->GetComponentLocation().Z) Pad=P;
        if(bProbe)
        {
            for(int I=0;I<Wheel->TreadPads.Num();++I) if(Wheel->TreadPads[I]==Pad) ProbePad=I;
            const int N=Wheel->TreadPads.Num();
            NeighborReference[0]=Wheel->TreadCompressionM[(ProbePad+N-1)%N];
            NeighborReference[1]=Wheel->TreadCompressionM[(ProbePad+1)%N];
            ProbeStart=Pad->GetComponentLocation()/100.;
            const double A=2*UE_DOUBLE_PI*ProbePad/N+Wheel->Compliance.Tread.PhaseRad;
            ProbeForceN=-Wheel->Tyre->GetComponentQuat().RotateVector(FVector(FMath::Cos(A),FMath::Sin(A),0))*500;
        }
        else
        {
            const FVector Impulse(0,0,.5),V=Pad->GetPhysicsLinearVelocity()/100.;
            ImpulseWorkJ=FVector::DotProduct(Impulse,V)+Impulse.SizeSquared()/(2*Pad->GetMass());
            Pad->AddImpulse(Impulse*100.);
        }
    }
    if(bProbe && bPulsed)
    {
        auto* Pad=Wheel->TreadPads[ProbePad].Get();
        if(TimeS<6+Dt) ImpulseWorkJ=FVector::DotProduct(ProbeForceN,Pad->GetComponentLocation()/100.-ProbeStart);
        if(TimeS<6) Pad->AddForceAtLocation(ProbeForceN*100,Pad->GetComponentLocation());
        if(TimeS>5 && TimeS<6)
        {
            const int N=Wheel->TreadPads.Num();
            NeighborResponseM=FMath::Max(NeighborResponseM,FMath::Abs(Wheel->TreadCompressionM[(ProbePad+N-1)%N]-NeighborReference[0]));
            NeighborResponseM=FMath::Max(NeighborResponseM,FMath::Abs(Wheel->TreadCompressionM[(ProbePad+1)%N]-NeighborReference[1]));
        }
    }
    if(bPulsed) PeakExcessJ=FMath::Max(PeakExcessJ,EnergyJ()-ReferenceJ-ImpulseWorkJ);
    if(TimeS>=12)
    {
        bDone=true;
        double MinGap=1e30;
        for(UWheelTyreComponent* Pad:Wheel->TreadPads) for(const auto& Hull:Pad->GetBodySetup()->AggGeom.ConvexElems)
            if(const auto& Shape=Hull.GetChaosConvexMesh()) for(int V=0;V<Shape->NumVertices();++V)
                MinGap=FMath::Min(MinGap,Pad->GetComponentTransform().TransformPosition(FVector(Shape->GetVertex(V))).Z/100.);
        const auto* Solver=GetWorld()->GetPhysicsScene()->GetSolver();
        UE_LOG(LogTemp,Display,TEXT("RING_NUMERICS max_inner_s=%.12g position_iterations=%d velocity_iterations=%d minimum_vertex_z_m=%.9f"),
            MaxInnerS,Solver->GetEvolution()->GetNumPositionIterations(),Solver->GetEvolution()->GetNumVelocityIterations(),MinGap);
        const double Expected=Wheel->Compliance.Tread.bCoupled?ExternalReferenceN:Wheel->Compliance.Tread.FlatSupportN(RadiusM,WidthM,HeightM),Measured=NormalIntegral/FMath::Max(1e-9,SampleS);
        const double Error=FMath::Abs(Measured/Expected-1),FinalDelta=EnergyJ()-ReferenceJ;
        const bool Pass=SampleS>3 && Error<.03 && PeakExcessJ<.02 && FMath::Abs(FinalDelta)<.02 && !Wheel->bTreadBottomed && (!bProbe || NeighborResponseM>1e-5);
        const auto& Spec=Wheel->Compliance.Tread;
        if(FParse::Param(FCommandLine::Get(),TEXT("TreadPoseDump")))
        {
            FString Csv=TEXT("index,theta,compression,shear,rotation,drive_radial,drive_shear,force_radial_raw,force_shear_raw,torque_raw,speed_mps,omega_rad_s\n");
            const FQuat Q=Wheel->Tyre->GetComponentQuat();
            for(int I=0;I<Spec.Count;++I)
            {
                const double A=2*UE_DOUBLE_PI*I/Spec.Count+Spec.PhaseRad;
                const FVector Dir=Q.RotateVector(FVector(FMath::Cos(A),FMath::Sin(A),0));
                const FVector Tan=Q.RotateVector(FVector(-FMath::Sin(A),FMath::Cos(A),0));
                const double Theta=FMath::Atan2(Dir.X,-Dir.Z);
                const double Sign=FVector::DotProduct(Tan,FVector(FMath::Cos(Theta),0,FMath::Sin(Theta)))>0?1:-1;
                auto* Joint=Wheel->TreadJoints[I].Get(); auto* Pad=Wheel->TreadPads[I].Get();
                const FTransform F0=Joint->ConstraintInstance.GetRefFrame(Spec.bCoupled?EConstraintFrame::Frame2:EConstraintFrame::Frame1)*Wheel->Tyre->BodyInstance.GetUnrealWorldTransform();
                const FTransform F1=Joint->ConstraintInstance.GetRefFrame(Spec.bCoupled?EConstraintFrame::Frame1:EConstraintFrame::Frame2)*Pad->BodyInstance.GetUnrealWorldTransform();
                const FVector Delta=F0.GetRotation().UnrotateVector(F1.GetLocation()-F0.GetLocation())/100.;
                FVector Force,Torque; Joint->GetConstraintForce(Force,Torque); Force=F0.GetRotation().UnrotateVector(Force); Torque=F0.GetRotation().UnrotateVector(Torque);
                Csv+=FString::Printf(TEXT("%d,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g\n"),I,Theta,Wheel->TreadCompressionM[I],Sign*Wheel->TreadShearM[I],Sign*Wheel->TreadRotationRad[I],-Delta.Z,-Sign*Delta.Y,Force.Z,Sign*Force.Y,Sign*Torque.X,Pad->GetPhysicsLinearVelocity().Size()/100.,Pad->GetPhysicsAngularVelocityInRadians().Size());
            }
            FFileHelper::SaveStringToFile(Csv,*(FPaths::ProjectSavedDir()/FString::Printf(TEXT("Validation/TreadPose-%d-%.2f.csv"),Spec.Count,Spec.PhaseRad*Spec.Count/(2*UE_DOUBLE_PI))));
            UE_LOG(LogTemp,Display,TEXT("RING_ACTUAL_MAX_DT %.12g"),GetWorld()->GetPhysicsScene()->GetSolver()->GetMaxDeltaTime_External());
        }
        const double Limit=Spec.bCoupled?0:Spec.ContinuousSupportN(RadiusM,WidthM,HeightM);
        UE_LOG(LogTemp,Display,TEXT("TREAD_REFERENCE count=%d height_m=%.5f phase_fraction=%.5f coupled=%d probe=%d neighbor_response_m=%.8f normal_n=%.5f analytic_n=%.5f continuum_n=%.5f force_error=%.7f continuum_error=%.7f input_work_j=%.7f excess_gain_j=%.7f final_delta_j=%.7f max_compression_m=%.7f shear_m=%.7f rotation_rad=%.7f link_j=%.7f %s"),
            Wheel->TreadPads.Num(),HeightM,Spec.PhaseRad*Spec.Count/(2*UE_DOUBLE_PI),Spec.bCoupled?1:0,bProbe?1:0,NeighborResponseM,Measured,Expected,Limit,Error,Limit>0?FMath::Abs(Measured/Limit-1):0,ImpulseWorkJ,PeakExcessJ,FinalDelta,Wheel->TreadMaxCompressionM,Wheel->TreadMaxShearM,Wheel->TreadMaxRotationRad,Wheel->TreadLinkJ,Pass?TEXT("PASS"):TEXT("FAIL"));
        FGenericPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
    }
}
void ATreadReference::EndPlay(const EEndPlayReason::Type Reason)
{
    if(Sampler) { GetWorld()->GetPhysicsScene()->GetSolver()->UnregisterAndFreeSimCallbackObject_External(Sampler); Sampler=nullptr; }
    Super::EndPlay(Reason);
}
