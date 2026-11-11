#include "ForkActor.h"
#include "Vehicle3DHelpers.h"
#include "UObject/ConstructorHelpers.h"
#include "PalletLoadActor.h"
#include "Engine/World.h"
#include "HydraulicCallback.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "PBDRigidsSolver.h"

AForkTineActor::AForkTineActor()
{
    Body=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Tine")); RootComponent=Body;
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube")); Body->SetStaticMesh(Cube.Object);
    Mount=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Mount"));
}
void AForkTineActor::Configure(UStaticMeshComponent* Carriage,double MassKg,double Mu,const FForkStructureSettings& Spec)
{
    Structure=Spec; Carrier=Carriage;
    const FVector Center=Body->GetComponentLocation(), Start=Center-FVector(Structure.LengthM*50,0,0);
    HeelLocal=Carriage->GetComponentQuat().UnrotateVector(Start-Carriage->GetComponentLocation());
    const int Count=Structure.bEnabled?Structure.Segments:1;
    const double D=Structure.LengthM/Count;
    auto* M=Vehicle3D::Material(this,Mu); M->SleepLinearVelocityThreshold=0; M->SleepAngularVelocityThreshold=0; M->SleepCounterThreshold=MAX_int32;
    for(int I=0;I<Count;++I)
    {
        auto* B=I==0?Body.Get():NewObject<UStaticMeshComponent>(this);
        if(I>0) { B->SetStaticMesh(Body->GetStaticMesh()); B->RegisterComponent(); }
        B->SetWorldLocation(Start+FVector((I+.5)*D*100,0,0));
        B->SetWorldScale3D(FVector(D,Structure.WidthM,Structure.ThicknessM)); Vehicle3D::Dynamic(B,MassKg/Count);
        B->SetLinearDamping(0); B->SetAngularDamping(0); B->SetPhysMaterialOverride(M);
        Vehicle3D::Color(B,FLinearColor(.12f,.18f,.24f)); Sections.Add(B);
        auto* J=I==0?Mount.Get():NewObject<UPhysicsConstraintComponent>(this);
        if(I>0) J->RegisterComponent();
        J->SetWorldLocation(Structure.bEnabled?Start+FVector(I*D*100,0,0):Carriage->GetComponentLocation()); Vehicle3D::Lock(J);
        J->SetConstrainedComponents(I==0?Carriage:Sections[I-1].Get(),NAME_None,B,NAME_None);
        if(Structure.bEnabled)
        {
            J->SetAngularSwing2Limit(ACM_Free,0);
            J->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
            J->SetOrientationDriveTwistAndSwing(false,true); J->SetAngularVelocityDriveTwistAndSwing(false,true);
            const double K=Structure.JointK(I), Inertia=MassKg/Count*D*D/3.;
            const double Damping=2*Structure.DampingRatio*FMath::Sqrt(K*Inertia);
            Vehicle3D::AngularSpring(J,K,Damping,Structure.BreakMomentNm()*2);
            // Use ONE declared material criterion. Chaos' separate impulse
            // break threshold also counts transient constraint reactions and
            // can sever a joint before its elastic/plastic material state fails.
            J->SetAngularBreakable(false,0);
        }
        Bends.Add(J); RestPitchDeg.Add(0); AccumulatedPlasticDeg.Add(0);
    }
}
FVector AForkTineActor::Heel() const { return Body->GetComponentLocation()-Body->GetForwardVector()*(Structure.LengthM/Sections.Num()*50); }
FVector AForkTineActor::Tip() const { auto* B=Sections.Last().Get(); return B->GetComponentLocation()+B->GetForwardVector()*(Structure.LengthM/Sections.Num()*50); }
void AForkTineActor::UpdateStructure()
{
    if(!Structure.bEnabled || !Carrier) return;
    const FVector RestTip=Carrier->GetComponentLocation()+Carrier->GetComponentQuat().RotateVector(HeelLocal+FVector(Structure.LengthM*100,0,0));
    TipDeflectionM=FVector::DotProduct(RestTip-Tip(),Carrier->GetUpVector())/100.;
    if(bFailed) return;
    for(int I=0;I<Bends.Num();++I)
    {
        auto* J=Bends[I].Get(); bFailed |= J->IsBroken(); if(J->IsBroken()) continue;
        const double A=J->GetCurrentSwing2(), Delta=FMath::FindDeltaAngleDegrees(RestPitchDeg[I],A);
        const double K=Structure.JointK(I), Hardening=K*Structure.HardeningRatio;
        const double YieldMoment=Structure.YieldMomentNm()+Hardening*FMath::DegreesToRadians(AccumulatedPlasticDeg[I]);
        const double TrialMoment=FMath::Abs(FMath::DegreesToRadians(Delta))*K;
        const double TrialStress=FMath::Abs(FMath::DegreesToRadians(Delta))*K/Structure.SectionModulusM3();
        MaxStressPa=FMath::Max(MaxStressPa,TrialStress);
        // Drive-only torque is not consistently reported by Chaos' native
        // break threshold. Apply the declared material trial-stress criterion
        // explicitly; breaking the real joint releases its physical bodies.
        if(TrialStress>Structure.BreakPa) { FractureReleaseJ+=.5*K*FMath::Square(FMath::DegreesToRadians(Delta));J->BreakConstraint(); bFailed=true; continue; }
        // 1D return mapping with isotropic hardening. Plastic work changes the
        // rest angle and yield surface; no external support force is invented.
        if(TrialMoment>YieldMoment)
        {
            const double Increment=FMath::RadiansToDegrees((TrialMoment-YieldMoment)/(K+Hardening));
            PlasticRelaxationJ+=.5*K*(FMath::Square(FMath::DegreesToRadians(Delta))-FMath::Square(FMath::DegreesToRadians(FMath::Abs(Delta)-Increment)));
            RestPitchDeg[I]=FMath::UnwindDegrees(RestPitchDeg[I]+FMath::Sign(Delta)*Increment);
            AccumulatedPlasticDeg[I]+=Increment;
            J->SetAngularOrientationTarget(FRotator(RestPitchDeg[I],0,0));
            if(AccumulatedPlasticDeg[I]>Structure.FailurePlasticDeg) { FractureReleaseJ+=.5*K*FMath::Square(FMath::DegreesToRadians(FMath::Abs(Delta)-Increment));J->BreakConstraint(); bFailed=true; }
        }
    }
    PlasticDeg=0; for(double P:AccumulatedPlasticDeg) PlasticDeg+=P;
}
double AForkTineActor::ElasticEnergyJ() const
{
    double E=0;
    // Recoverable bending energy, including intact joints after a local break.
    for(int I=0;I<Bends.Num();++I) if(!Bends[I]->IsBroken() && Structure.bEnabled)
    {
        const double A=FMath::DegreesToRadians(FMath::FindDeltaAngleDegrees(RestPitchDeg[I],Bends[I]->GetCurrentSwing2()));
        E+=.5*Structure.JointK(I)*A*A;
    }
    return E;
}

AForkActor::AForkActor()
{
    Carriage=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ForkCarriage")); RootComponent=Carriage;
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube")); Carriage->SetStaticMesh(Cube.Object);
    Guide=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("LiftGuide"));
}
void AForkActor::Configure(UStaticMeshComponent* Chassis,const FVehicle3DSettings& S,const FPayloadSettings* Payload)
{
    TravelM=S.ForkTravelM; const FVector P=GetActorLocation();
    // Lower blades must enter a ground pallet without dragging the carriage
    // through the floor. Its lowest face retains 3 cm clearance at lift zero.
    const bool Chained=S.Hydraulics.bEnabled && S.bSwerve && S.Hydraulics.bChainMast;
    const double StageMass=Chained?S.Hydraulics.MastMassKg:0;
    const double TineMass=(Payload || S.bReferenceGeometry) && S.Structure.bEnabled?S.Structure.BladeMassKg():S.ForkMassKg*.2;
    const double CarriageMass=S.ForkMassKg-2*TineMass-StageMass;
    Carriage->SetWorldScale3D(FVector(S.bReferenceGeometry?.06:.12,S.bReferenceGeometry?S.CarriageWidthM:.9,(Payload || S.bReferenceGeometry)?.35:.45)); Vehicle3D::Dynamic(Carriage,S.bReferenceGeometry?CarriageMass*.5:CarriageMass);
    if(S.bReferenceGeometry)
    {
        for(double Y:{-.4,.4}) Vehicle3D::Box(this,Carriage,P+FVector(-1,Y*100,45),FVector(.05,.04,.65),CarriageMass*.2,FLinearColor(.12,.16,.19));
        Vehicle3D::Box(this,Carriage,P+FVector(-1,0,76),FVector(.05,.84,.04),CarriageMass*.1,FLinearColor(.12,.16,.19));
    }
    Vehicle3D::Color(Carriage,FLinearColor(.95f,.45f,.035f));
    Carriage->SetPhysMaterialOverride(Vehicle3D::Material(this,.4));
    if(Payload || S.bReferenceGeometry) for(int I=0;I<2;++I)
    {
        const double Y=((Payload?Payload->ForkShiftYM:0)+(I==0?-1:1)*(Payload?Payload->ForkSpacingM:.6)*.5)*100;
        auto* T=GetWorld()->SpawnActor<AForkTineActor>(P+FVector(S.Structure.LengthM*50+(S.bReferenceGeometry?3:-5),Y,-15),FRotator::ZeroRotator);
        const double Mu=Payload && Payload->bWarehouseAssets?FMath::Min(Payload->ForkMu,I==0?Payload->LeftHoleMu:Payload->RightHoleMu):Payload?Payload->ForkMu:.45;
        T->Configure(Carriage,TineMass,Mu,S.Structure); Tines.Add(T);
    }
    else for (double Y : { -30.,30. })
        Vehicle3D::Box(this,Carriage,P+FVector(65,Y,-15),FVector(1.4,.12,.07),S.ForkMassKg*.2,FLinearColor(.12f,.18f,.24f));
    Carriage->BodyInstance.UpdateMassProperties();
    Guide->SetWorldLocation(P+FVector(0,0,TravelM*50));
    Guide->AttachToComponent(Chassis,FAttachmentTransformRules::KeepWorldTransform);
    Vehicle3D::Lock(Guide); Guide->SetLinearZLimit(LCM_Limited,TravelM*50);
    Guide->SetConstrainedComponents(Chassis,NAME_None,Carriage,NAME_None);
    Guide->SetConstraintReferencePosition(EConstraintFrame::Frame2,FVector::ZeroVector);
    HydraulicSettings=S.Hydraulics; HydraulicSettings.bEnabled &= S.bSwerve;
    if(Chained)
    {
        MovingMast=NewObject<UStaticMeshComponent>(this); MovingMast->SetStaticMesh(Carriage->GetStaticMesh());
        MovingMast->RegisterComponent(); MovingMast->SetWorldLocation(P+FVector(-12,0,50)); MovingMast->SetWorldScale3D(S.bReferenceGeometry?FVector(.08,.62,.08):FVector(.1,.6,1.2));
        Vehicle3D::Dynamic(MovingMast,S.bReferenceGeometry?StageMass*.2:StageMass); MovingMast->SetLinearDamping(0); MovingMast->SetAngularDamping(0);
        if(!S.bReferenceGeometry) MovingMast->SetCollisionResponseToAllChannels(ECR_Ignore);
        if(S.bReferenceGeometry)
        {
            for(double Y:{-.27,.27}) Vehicle3D::Box(this,MovingMast,P+FVector(-12,Y*100,110),FVector(.08,.05,1.82),StageMass*.4,FLinearColor(.18,.23,.27));
            MovingMast->BodyInstance.UpdateMassProperties();
        }
        Vehicle3D::Color(MovingMast,FLinearColor(.18,.23,.27));
        MastGuide=NewObject<UPhysicsConstraintComponent>(this); MastGuide->RegisterComponent();
        MastGuide->SetWorldLocation(MovingMast->GetComponentLocation()+FVector(0,0,TravelM/ S.Hydraulics.ChainRatio*50));
        Vehicle3D::Lock(MastGuide); MastGuide->SetLinearZLimit(LCM_Limited,TravelM/S.Hydraulics.ChainRatio*50);
        MastGuide->SetConstrainedComponents(Chassis,NAME_None,MovingMast,NAME_None);
        MastGuide->SetConstraintReferencePosition(EConstraintFrame::Frame2,FVector::ZeroVector);
        if(S.bReferenceGeometry)
        {
            // The guide already constrains these mechanically connected parts.
            // Avoid a second contact constraint between adjacent guide members;
            // retain their collisions with cargo and the warehouse.
            auto* Filter=NewObject<UPhysicsConstraintComponent>(this);AddInstanceComponent(Filter);Filter->RegisterComponent();
            Filter->SetWorldLocation(P);Vehicle3D::Lock(Filter);
            Filter->SetLinearXLimit(LCM_Free,0);Filter->SetLinearYLimit(LCM_Free,0);Filter->SetLinearZLimit(LCM_Free,0);
            Filter->SetAngularTwistLimit(ACM_Free,0);Filter->SetAngularSwing1Limit(ACM_Free,0);Filter->SetAngularSwing2Limit(ACM_Free,0);
            Filter->SetConstrainedComponents(MovingMast,NAME_None,Carriage,NAME_None);
        }
    }
    LiftBase=Chassis; BaseZCm=Chassis->GetComponentQuat().UnrotateVector(P-Chassis->GetComponentLocation()).Z;
    ForceCapN=S.ForkForceN;
    const double EffectiveForkMass=Chained?StageMass+(S.ForkMassKg-StageMass)*HydraulicSettings.ChainRatio:S.ForkMassKg;
    InitialPressurePa=FMath::Min(EffectiveForkMass*9.81/HydraulicSettings.AreaM2,HydraulicSettings.ReliefPa);
    Guide->SetLinearPositionDrive(false,false,!HydraulicSettings.bEnabled);
    Guide->SetLinearVelocityDrive(false,false,!HydraulicSettings.bEnabled);
    Guide->SetLinearDriveParams(100000,15000,S.ForkForceN*100);
    if(HydraulicSettings.bEnabled)
    {
        HydraulicState.PressurePa=InitialPressurePa;
        HydraulicCallback=GetWorld()->GetPhysicsScene()->GetSolver()->CreateAndRegisterSimCallbackObject_External<FHydraulicCallback>();
    }
    SetLiftTarget(0); UpdateHydraulics();
}
void AForkActor::EndPlay(const EEndPlayReason::Type R)
{
    if(HydraulicCallback && GetWorld() && GetWorld()->GetPhysicsScene())
        GetWorld()->GetPhysicsScene()->GetSolver()->UnregisterAndFreeSimCallbackObject_External(HydraulicCallback);
    HydraulicCallback=nullptr;
    for(AForkTineActor* T:Tines) if(T) T->Destroy(); Tines.Empty(); Super::EndPlay(R);
}
void AForkActor::SetLiftTarget(double H)
{
    TargetM=FMath::Clamp(H,0.,TravelM);
    // Chaos drive target is frame 1 relative to frame 2, opposite to carriage height.
    Guide->SetLinearPositionTarget(FVector(0,0,(TravelM*.5-FMath::Clamp(H,0.,TravelM))*100));
}
void AForkActor::UpdateHydraulics()
{
    PressureSamples.Reset();
    HydraulicCommandExpired=false;
    HydraulicFramePeakPressurePa=HydraulicFramePeakDcW=HydraulicFrameReliefM3=0;
    if(!HydraulicCallback) return;
    while(auto Out=HydraulicCallback->PopOutputData_External())
    {
        HydraulicState=Out->State; HydraulicHeightM=Out->HeightM; HydraulicVelocityMps=Out->VelocityMps;
        HydraulicFramePeakPressurePa=FMath::Max(HydraulicFramePeakPressurePa,Out->PeakPressurePa);
        HydraulicFramePeakDcW=FMath::Max(HydraulicFramePeakDcW,Out->PeakDcW);
        HydraulicFrameReliefM3+=Out->ReliefVolumeM3;
        PressureSamples.Append(Out->SensorSamples);
        HydraulicCommandExpired|=Out->CommandExpired;
    }
    auto* In=HydraulicCallback->GetProducerInputData_External();
    In->Settings=HydraulicSettings;
    In->Command=ValveControl;In->CommandValidUntilS=ValveControlValidUntilS;
    In->bPressureSensor=bPressureSensor;In->PressureSamplePeriodS=PressureSamplePeriodS;
    In->ChassisId=LiftBase->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
    In->CarriageId=Carriage->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
    In->MastId=MovingMast?MovingMast->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx:INDEX_NONE;
    In->MastBaseZCm=BaseZCm+50;
    In->TargetM=TargetM; In->BaseZCm=BaseZCm; In->TravelM=TravelM;
    In->ForceCapN=ForceCapN; In->InitialPressurePa=InitialPressurePa;
}
void AForkActor::SetValveCommand(const FHydraulicCommand& Command,double ValidUntilS)
{
    ValveControl=Command;ValveControlValidUntilS=ValidUntilS;
    if(HydraulicCallback){auto* In=HydraulicCallback->GetProducerInputData_External();In->Command=Command;In->CommandValidUntilS=ValidUntilS;}
}
