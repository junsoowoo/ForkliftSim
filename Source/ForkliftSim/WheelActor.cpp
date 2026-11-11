#include "WheelActor.h"
#include "WheelResistance.h"
#include "Vehicle3DHelpers.h"
#include "UObject/ConstructorHelpers.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "PhysicsEngine/BodySetup.h"
#include "Engine/World.h"
#include "Chaos/Convex.h"
#include "Physics/PhysicsInterfaceCore.h"

UBodySetup* UWheelTyreComponent::GetBodySetup()
{
    return RollingCollision?RollingCollision.Get():Super::GetBodySetup();
}
void UWheelTyreComponent::BuildRollingCollision(int32 Segments)
{
    if(FParse::Param(FCommandLine::Get(),TEXT("StockTyreCollision"))) return;
    // The stock render cylinder has a 32-sided collider. A wheel must climb
    // those faces as it rolls; this particularly distorts low-grip behaviour.
    // Give this component a rounder collider without editing the engine asset.
    // 0.5 uses 256 faces with a 240 Hz maximum physics step. The coast tests
    // bound numerical energy drift; more faces alone is not always more stable.
    FParse::Value(FCommandLine::Get(),TEXT("TyreSegments="),Segments);
    Segments=FMath::Clamp(Segments,32,512);
    RollingCollision=NewObject<UBodySetup>(this);
    RollingCollision->CollisionTraceFlag=CTF_UseSimpleAsComplex;
    const FVector Extent=GetStaticMesh()->GetBounds().BoxExtent;
    FKConvexElem Hull;
    for(int32 I=0;I<Segments;++I)
    {
        const double A=2*PI*I/Segments;
        for(double Z:{-Extent.Z,Extent.Z}) Hull.VertexData.Add(FVector(Extent.X*FMath::Cos(A),Extent.Y*FMath::Sin(A),Z));
    }
    Hull.UpdateElemBox(); RollingCollision->AggGeom.ConvexElems.Add(MoveTemp(Hull));
    RollingCollision->InvalidatePhysicsData(); RollingCollision->CreatePhysicsMeshes();
    if(const auto& Convex=RollingCollision->AggGeom.ConvexElems[0].GetChaosConvexMesh())
        UE_LOG(LogTemp,Display,TEXT("TYRE_HULL requested_segments=%d cooked_vertices=%d cooked_planes=%d"),Segments,Convex->NumVertices(),Convex->NumPlanes());
    RecreatePhysicsState();
}

AWheelActor::AWheelActor()
{
    Knuckle=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("SteeringKnuckle")); RootComponent=Knuckle;
    Tyre=CreateDefaultSubobject<UWheelTyreComponent>(TEXT("PhysicalTyre"));
    BeltCarrier=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("NonSpinningBeltCarrier"));
    ComplianceGuide=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("TyreRadialCompliance"));
    ShearGuide=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("TyreShearCompliance"));
    BrakeJoint=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("WheelBrakeMotor"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    Knuckle->SetStaticMesh(Cube.Object); BeltCarrier->SetStaticMesh(Cube.Object); Tyre->SetStaticMesh(Cylinder.Object);
    SteeringJoint=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("SteeringAxis"));
    Axle=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("WheelAxle"));
    ChassisCollisionFilter=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("ChassisTyreSelfCollisionFilter"));
    Tyre->OnComponentHit.AddDynamic(this,&AWheelActor::OnTyreHit);
}
void AWheelActor::Configure(UStaticMeshComponent* Chassis,const FVehicle3DSettings& S,bool Rear,UPhysicalMaterial* Material,int32 WheelSlot)
{
    Compliance=S.Tyres;
    bMecanum=S.bMecanum;
    if(bMecanum) for(UPhysicsConstraintComponent* J:{SteeringJoint.Get(),Axle.Get(),ComplianceGuide.Get(),BrakeJoint.Get()}) J->ConstraintInstance.ProfileInstance.bUseLinearJointSolver=FParse::Param(FCommandLine::Get(),TEXT("MecanumLinearJoints"));
    if(bMecanum) {Compliance.bEnabled=false;Compliance.Tread.bEnabled=false;}
    Compliance.Tread.bEnabled &= S.bSwerve && Compliance.bEnabled && Compliance.bDetailed;
    Compliance.Tread.bCoupled &= Compliance.Tread.bEnabled;
    const bool LocalTread=Compliance.Tread.bEnabled;
    bRear=Rear; bSteerable=!bMecanum && (Rear || S.bSwerve); MotorStator=Chassis; const FVector P=GetActorLocation();
    // Tune the position servo for the loaded swerve tyre's scrub moment. Motor
    // torque/power limits stay unchanged; do not hide lag by changing friction.
    SteeringPositionGain=S.bSwerve?200000000:50000000;
    SteeringVelocityGain=S.bSwerve?4000000:2000000;
    // A 20 cm bearing/knuckle assembly has finite rotational inertia. A 5 cm
    // proxy made the articulated axle needlessly stiff relative to the wheel.
    Knuckle->SetWorldScale3D(FVector(.20)); Vehicle3D::Dynamic(Knuckle,S.KnuckleMassKg*.8);
    Knuckle->SetCollisionResponseToAllChannels(ECR_Ignore); Knuckle->SetVisibility(false);
    BeltCarrier->SetWorldLocation(P); BeltCarrier->SetWorldScale3D(FVector(.20));
    Vehicle3D::Dynamic(BeltCarrier,S.KnuckleMassKg*.2);
    BeltCarrier->SetCollisionResponseToAllChannels(ECR_Ignore); BeltCarrier->SetVisibility(false);
    const double CoreRadius=bMecanum && !S.Mecanum.bReduced?S.Mecanum.HubRadiusM:S.RadiusM-(LocalTread?Compliance.Tread.DepthM:0);
    Tyre->SetWorldLocationAndRotation(P,FRotator(0,0,90)); Tyre->SetWorldScale3D(FVector(CoreRadius*2,CoreRadius*2,bMecanum?S.Mecanum.HubWidthM:S.WidthM));
    CastChecked<UWheelTyreComponent>(Tyre)->BuildRollingCollision(S.bSwerve?256:128);
    Vehicle3D::Dynamic(Tyre,S.WheelMassKg-(bMecanum && !S.Mecanum.bReduced?S.Mecanum.Count*S.Mecanum.RollerMassKg:LocalTread?Compliance.Tread.MassKg:0)); SetMu(Material); Tyre->SetAngularDamping(.02f);
    TreadChannel=static_cast<ECollisionChannel>(ECC_GameTraceChannel1+FMath::Clamp(WheelSlot,0,3));
    if(LocalTread || bMecanum) { Tyre->SetCollisionObjectType(TreadChannel); Tyre->SetCollisionResponseToChannel(TreadChannel,ECR_Ignore); }
    if(bMecanum) {Tyre->SetCollisionEnabled(ECollisionEnabled::PhysicsOnly);}
    ContactMu=S.EffectiveMu();
    Tyre->SetNotifyRigidBodyCollision(true);
    Vehicle3D::Color(Tyre,FLinearColor(.035f,.045f,.06f));
    // Massless visual marks rotate with the real tyre body.
    for (double Side : { -1.,1. })
    {
        auto* Mark=NewObject<UStaticMeshComponent>(this); Mark->SetStaticMesh(Knuckle->GetStaticMesh());
        Mark->SetMobility(EComponentMobility::Movable); Mark->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Mark->RegisterComponent(); Mark->SetWorldLocation(P+FVector(0,Side*((bMecanum?S.Mecanum.HubWidthM:S.WidthM)*50+.5),0));
        Mark->SetWorldScale3D(FVector((bMecanum?CoreRadius:S.RadiusM)*1.5,.015,.035));
        Mark->AttachToComponent(Tyre,FAttachmentTransformRules::KeepWorldTransform); Vehicle3D::Color(Mark,FLinearColor(.75f,.85f,.9f));
    }
    SteeringJoint->SetWorldLocationAndRotation(P,FRotator(90,0,0));
    SteeringJoint->AttachToComponent(Chassis,FAttachmentTransformRules::KeepWorldTransform);
    Vehicle3D::Lock(SteeringJoint);
    if (!bMecanum && S.bSwerve) SteeringJoint->SetAngularTwistLimit(ACM_Free,0);
    else if (!bMecanum && Rear) SteeringJoint->SetAngularTwistLimit(ACM_Limited,55);
    SteeringJoint->SetConstrainedComponents(Chassis,NAME_None,Knuckle,NAME_None);
    SteeringJoint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
    SteeringJoint->SetOrientationDriveTwistAndSwing(bSteerable,false);
    SteeringJoint->SetAngularVelocityDriveTwistAndSwing(bSteerable,false);
    SteeringJoint->SetAngularVelocityTarget(FVector::ZeroVector);
    SteeringJoint->SetAngularDriveParams(SteeringPositionGain,SteeringVelocityGain,S.SteeringTorqueNm*10000);
    Axle->SetWorldLocationAndRotation(P,FRotator(0,90,0)); Axle->AttachToComponent(bMecanum?Chassis:BeltCarrier.Get(),FAttachmentTransformRules::KeepWorldTransform);
    Vehicle3D::Lock(Axle); Axle->SetAngularTwistLimit(ACM_Free,0);
    // Mecanum has a rigid mount: connect the axle to the actual load-bearing
    // chassis rather than routing tyre load through a 1 kg virtual carrier.
    // The separate mount bodies retain their mass and fixed attachments.
    Axle->SetConstrainedComponents(bMecanum?Chassis:BeltCarrier.Get(),NAME_None,Tyre,NAME_None);
    Axle->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
    // A torque-only brake connects tyre directly to its non-spinning hub.
    // Routing its implicit velocity drive through the 1 kg belt carrier loses
    // braking authority in the iterative solver. This joint has NO position or
    // angular limits, so it adds no parallel support/spring and permits travel.
    BrakeJoint->SetWorldLocationAndRotation(P,FRotator(0,90,0));
    BrakeJoint->AttachToComponent(bMecanum?Chassis:Knuckle.Get(),FAttachmentTransformRules::KeepWorldTransform);
    Vehicle3D::Lock(BrakeJoint);
    BrakeJoint->SetLinearXLimit(LCM_Free,0); BrakeJoint->SetLinearYLimit(LCM_Free,0); BrakeJoint->SetLinearZLimit(LCM_Free,0);
    BrakeJoint->SetAngularTwistLimit(ACM_Free,0); BrakeJoint->SetAngularSwing1Limit(ACM_Free,0); BrakeJoint->SetAngularSwing2Limit(ACM_Free,0);
    BrakeJoint->SetConstrainedComponents(bMecanum?Chassis:Knuckle.Get(),NAME_None,Tyre,NAME_None);
    BrakeJoint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
    ComplianceGuide->SetWorldLocation(P); Vehicle3D::Lock(ComplianceGuide);
    ComplianceGuide->SetConstrainedComponents(Knuckle,NAME_None,BeltCarrier,NAME_None);
    Compliance.bEnabled &= S.bSwerve;
    if(S.bSwerve && Compliance.bEnabled)
    {
        // BOTH compliance frames are non-spinning. A free-Z drive in the axle
        // itself would rotate its compliant axis with the tyre (UE child frame).
        ComplianceGuide->SetLinearZLimit(LCM_Limited,Compliance.TravelM*100);
        ComplianceGuide->SetLinearPositionDrive(false,false,true);
        ComplianceGuide->SetLinearVelocityDrive(false,false,true);
        ComplianceGuide->SetLinearPositionTarget(FVector::ZeroVector);
        ComplianceGuide->SetLinearVelocityTarget(FVector::ZeroVector);
        // N/m and N*s/m have the same numeric value in kg-cm-s spring units.
        ComplianceGuide->SetLinearDriveParams(Compliance.StiffnessNpm,Compliance.DampingNsPm,0);
        if(Compliance.bDetailed)
        {
            ComplianceGuide->SetLinearXLimit(LCM_Free,0); ComplianceGuide->SetLinearYLimit(LCM_Free,0);
            // UE has ONE shared linear limit radius. Setting a free axis with
            // radius zero also overwrites the existing limited-axis radius.
            ComplianceGuide->SetLinearZLimit(LCM_Limited,Compliance.TravelM*100);
            ShearGuide->SetWorldLocation(P); Vehicle3D::Lock(ShearGuide);
            ShearGuide->SetLinearXLimit(LCM_Limited,Compliance.ShearTravelM*100);
            ShearGuide->SetLinearYLimit(LCM_Limited,Compliance.ShearTravelM*100); ShearGuide->SetLinearZLimit(LCM_Free,0);
            ShearGuide->SetLinearXLimit(LCM_Limited,Compliance.ShearTravelM*100);
            ShearGuide->SetAngularTwistLimit(ACM_Free,0); ShearGuide->SetAngularSwing1Limit(ACM_Free,0); ShearGuide->SetAngularSwing2Limit(ACM_Free,0);
            ShearGuide->SetConstrainedComponents(Knuckle,NAME_None,BeltCarrier,NAME_None);
            ShearGuide->SetLinearPositionDrive(true,true,false); ShearGuide->SetLinearVelocityDrive(true,true,false);
            ShearGuide->SetLinearDriveParams(Compliance.ShearNpm,Compliance.ShearDampingNsPm,0);
        }
    }

    BuildTread(S,Material);
    BuildMecanum(S,Material,WheelSlot);
    if(S.bSwerve && S.Actuators.bEnabled)
    {
        // Ideal rigid gear coupling: J_motor*G^2 is added at the driven axis.
        // Existing tyre/knuckle geometry inertia is preserved, not replaced.
        auto Rotor=[&](UStaticMeshComponent* Body,const ElectricDrive::FSpec& Motor)
        {
            const double Added=Motor.RotorInertiaKgM2*Motor.Ratio*Motor.Ratio*10000;
            FPhysicsCommand::ExecuteWrite(Body->BodyInstance.GetPhysicsActor(),[Added](const FPhysicsActorHandle& Actor)
            {
                FVector I=FPhysicsInterface::GetLocalInertiaTensor_AssumesLocked(Actor);
                const FQuat MassQ=FPhysicsInterface::GetComTransformLocal_AssumesLocked(Actor).GetRotation();
                const FVector Axis=MassQ.UnrotateVector(FVector::UpVector);
                int K=0;for(int J=1;J<3;++J) if(FMath::Abs(Axis[J])>FMath::Abs(Axis[K])) K=J;
                checkf(FMath::Abs(Axis[K])>.999,TEXT("Equivalent drive rotor requires principal shaft axis"));
                I[K]+=Added;
                FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(Actor,I);
            });
        };
        Rotor(Tyre,S.Actuators.Drive); if(!bMecanum) Rotor(Knuckle,S.Actuators.Steer);
    }
    if(bMecanum) UE_LOG(LogTemp,Display,TEXT("MECANUM_INERTIA scale_z=%.9f shaft_i_kgm2=%.9f rotor_reflected_kgm2=%.9f"),Tyre->BodyInstance.InertiaTensorScale.Z,Tyre->GetInertiaTensor().Z/10000.,S.Actuators.Drive.RotorInertiaKgM2*FMath::Square(S.Actuators.Drive.Ratio));
    // Steering can bring a tyre inside the placeholder solid chassis box. It is
    // part of this assembly, so that pair must not collide (real geometry has a
    // wheel arch). The existing joints only filtered chassis-knuckle and
    // knuckle-tyre; they did not filter chassis-tyre.
    // A fully FREE joint supplies only the per-body collision exclusion: no
    // position/orientation drive, limit, mass, or stabilizing force is added.
    // The new deck has actual swept-volume clearance: leave tyre/chassis
    // collision enabled so incorrect geometry is observable in tests.
    if(!S.bSwerve && !FParse::Param(FCommandLine::Get(),TEXT("EnableChassisTyreCollision")))
    {
        ChassisCollisionFilter->SetWorldLocation(P);
        ChassisCollisionFilter->AttachToComponent(Chassis,FAttachmentTransformRules::KeepWorldTransform);
        Vehicle3D::Lock(ChassisCollisionFilter);
        ChassisCollisionFilter->SetLinearXLimit(LCM_Free,0); ChassisCollisionFilter->SetLinearYLimit(LCM_Free,0); ChassisCollisionFilter->SetLinearZLimit(LCM_Free,0);
        ChassisCollisionFilter->SetAngularTwistLimit(ACM_Free,0); ChassisCollisionFilter->SetAngularSwing1Limit(ACM_Free,0); ChassisCollisionFilter->SetAngularSwing2Limit(ACM_Free,0);
        ChassisCollisionFilter->SetConstrainedComponents(Chassis,NAME_None,Tyre,NAME_None);
    }
}
void AWheelActor::OnTyreHit(UPrimitiveComponent* HitComponent,AActor* OtherActor,UPrimitiveComponent* OtherComponent,FVector NormalImpulse,const FHitResult& Hit)
{
    if(GetWorld()->GetTimeSeconds()<.12 && OtherActor && OtherActor->GetClass()->GetName().Contains(TEXT("Fork")) && FParse::Param(FCommandLine::Get(),TEXT("ModelingContactDiagnostic")))
        UE_LOG(LogTemp,Warning,TEXT("WHEEL_ASSEMBLY_CONTACT rear=%d wheel=%s other=%s part=%s impulse_ns=%.6f"),bRear?1:0,*HitComponent->GetName(),*OtherActor->GetName(),OtherComponent?*OtherComponent->GetName():TEXT("none"),NormalImpulse.Size()/100);
    if(MotorStator && OtherActor==MotorStator->GetOwner())
    {
        ++ChassisContactCount;
        if(ChassisContactCount<=2) UE_LOG(LogTemp,Warning,TEXT("TYRE_CHASSIS_CONTACT rear=%d impulse_Ns=%.4f"),bRear?1:0,NormalImpulse.Size()/100);
    }
}
void AWheelActor::SetMu(UPhysicalMaterial* Material)
{
    auto Apply=[Material](UStaticMeshComponent* B)
    {
        // A friction edit is not a material-density/geometry edit. UE always
        // recalculates mass properties here; retain authored/equivalent inertia.
        const bool Valid=FPhysicsInterface::IsValid(B->BodyInstance.GetPhysicsActor());
        const FVector I=Valid?B->GetInertiaTensor():FVector::ZeroVector;
        const FTransform MassFrame=Valid?B->BodyInstance.GetMassSpaceLocal():FTransform::Identity;
        B->SetPhysMaterialOverride(Material);
        if(Valid) FPhysicsCommand::ExecuteWrite(B->BodyInstance.GetPhysicsActor(),[I,MassFrame](const FPhysicsActorHandle& Actor)
        {
            FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(Actor,I);
            FPhysicsInterface::SetComLocalPose_AssumesLocked(Actor,MassFrame);
        });
        B->WakeAllRigidBodies();
    };
    Apply(Tyre);for(UWheelTyreComponent* P:TreadPads) Apply(P);for(UWheelTyreComponent* P:Rollers) Apply(P);
}
void AWheelActor::UpdateLoad(double ImpulseNs,int32 Contacts,double Dt)
{
    FloorContacts=Contacts;
    NormalLoadN=Dt>0?ImpulseNs/Dt:0;
    const double Alpha=Dt>0?1-FMath::Exp(-Dt/.08):0;
    NormalLoadFilteredN=FloorContacts>0?FMath::Lerp(NormalLoadFilteredN,NormalLoadN,Alpha):0;
    // Instrumented wheels stay awake so resting support does not silently stop
    // producing fresh contact impulses. This adds no force or velocity.
    Tyre->WakeAllRigidBodies();
    for(UWheelTyreComponent* P:TreadPads) P->WakeAllRigidBodies();
    for(UWheelTyreComponent* P:Rollers) P->WakeAllRigidBodies();
}
void AWheelActor::ApplyResistance(const FVehicle3DSettings& S,bool Enabled)
{
    const FVector Axis=Knuckle->GetRightVector();
    const double RelativeOmega=OmegaRadS-FVector::DotProduct(Knuckle->GetPhysicsAngularVelocityInRadians(),Axis);
    // Static course surfaces. Drop delayed load on separation; normals are
    // measured against the local surface, not against an assumed Z=0 floor.
    const double Load=bGrounded && FloorContacts>0?NormalLoadFilteredN:0;
    const auto T=WheelResistance::Evaluate(Load,S.RadiusM,OmegaRadS,RelativeOmega,
        Enabled?S.RollingResistance:0,Enabled?S.BearingCoulombNm:0,Enabled?S.BearingViscousNms:0);
    // Mecanum rolling torque is applied and limited on the physics thread.
    // Keep the returned actual value in telemetry instead of replacing it
    // with the unconstrained game-frame request.
    if(!bMecanum)RollingTorqueNm=T.Rolling; BearingTorqueNm=T.Bearing;
    ResistancePowerW=-(RollingTorqueNm*OmegaRadS+T.Bearing*RelativeOmega);
    // Rolling loss is an EXTERNAL tyre-road moment (static road).
    if(!bMecanum) Tyre->AddTorqueInRadians(Axis*T.Rolling*10000);
    // Bearing friction is INTERNAL to the axle. Solve the dissipative torque
    // in the implicit zero-velocity joint, not as explicit opposite torques on
    // a tiny articulated knuckle. The solver supplies equal/opposite reactions
    // and cannot push wheel speed past zero. Add independent service-brake cap.
    const double Limit=BrakeLimitNm+FMath::Abs(T.Bearing);
    // Force-mode implicit velocity servo: finite damping otherwise needs wheel
    // rotation to generate a holding torque on a slope (T = D * relative omega).
    // Service braking approximates static axle friction with low compliance;
    // its torque cap stays physical. Bearing-only damping remains unchanged.
    UpdateBrakeServo(Limit);
    if(bMecanum) ResistRollers(Enabled);
}
void AWheelActor::UpdateBrakeServo(double Limit)
{
    const bool Elastic=BrakeLimitNm>0 && (!TreadPads.IsEmpty() || bMecanum);
    BrakeJoint->SetAngularVelocityDriveTwistAndSwing(Limit>0,false);
    BrakeJoint->SetAngularVelocityTarget(FVector::ZeroVector);
    BrakeJoint->SetOrientationDriveTwistAndSwing(Elastic,false);
    if(Elastic)
    {
        // Finite-torque elastic/plastic brake clutch. A velocity-only servo
        // cannot restore small angle drift introduced by an articulated solve.
        // Capture engagement angle (no snap), and return-map the rest angle
        // under slip. The REAL constraint still saturates at the brake limit.
        constexpr double K=100000,C=3000; // assumed Nm/rad, Nm*s/rad
        const double Angle=BrakeJoint->GetCurrentTwist();
        if(!bBrakeAngleActive) BrakeHoldAngleDeg=Angle;
        double Delta=FMath::DegreesToRadians(FMath::FindDeltaAngleDegrees(BrakeHoldAngleDeg,Angle));
        const double Yield=Limit/K;
        if(FMath::Abs(Delta)>Yield)
        {
            const double Slip=FMath::Abs(Delta)-Yield;
            BrakeHoldAngleDeg=FMath::UnwindDegrees(BrakeHoldAngleDeg+FMath::RadiansToDegrees(FMath::Sign(Delta)*Slip));
            BrakePlasticWorkJ+=Limit*Slip; Delta=FMath::Sign(Delta)*Yield;
        }
        BrakeElasticJ=.5*K*Delta*Delta;
        BrakeJoint->ConstraintInstance.SetAngularOrientationTarget(WheelResistance::TwistTarget(BrakeHoldAngleDeg));
        Vehicle3D::AngularSpring(BrakeJoint,K,C,Limit);
    }
    else
    {
        BrakeElasticJ=0;
        BrakeJoint->SetAngularDriveParams(0,BrakeLimitNm>0?1.e10:1.e6,Limit*10000);
    }
    bBrakeAngleActive=Elastic;
}
void AWheelActor::Drive(double Torque,double Brake,double Steer)
{
    DriveTorqueNm=Torque; BrakeLimitNm=Brake;
    if(FMath::Abs(Torque)>.001 || (bSteerable && FMath::Abs(FMath::FindDeltaAngleDegrees(SteerActualDeg,Steer))>.1))
    {
        Tyre->WakeAllRigidBodies(); Knuckle->WakeAllRigidBodies();
        if(MotorStator) MotorStator->WakeAllRigidBodies();
    }
    if (bSteerable) SteeringJoint->SetAngularOrientationTarget(FRotator(0,0,Steer));
    const FVector Axis=Knuckle->GetRightVector();
    Tyre->AddTorqueInRadians(Axis*Torque*10000);
    // The front motor is mounted on the chassis. Apply its reaction to that body
    // directly, avoiding an enormous intermediate angular velocity on the tiny knuckle.
    if(MotorStator) MotorStator->AddTorqueInRadians(-Axis*Torque*10000);
    UpdateBrakeServo(Brake);
}
void AWheelActor::Actuate(const FVehicle3DSettings& S,double Dt,double Request,double Brake,double Steer,bool Powered)
{
    DriveRequestNm=Powered?Request:0;
    SteerMotorRpm=SteerOmegaRadS*S.Actuators.Steer.Ratio*30/PI;
    const double RelativeOmega=OmegaRadS-FVector::DotProduct(Knuckle->GetPhysicsAngularVelocityInRadians(),Knuckle->GetRightVector());
    if(bMecanum)
    {
        DriveMotor=S.Actuators.bEnabled?ElectricDrive::Step(Request,RelativeOmega,S.Actuators.Drive,S.DriveNm,Dt,Powered,DriveShaftStateNm):ElectricDrive::Measure(DriveRequestNm,RelativeOmega,S.Actuators.Drive,S.DriveNm);
        SteerAvailableNm=SteerMotorRpm=SteerFilteredDeg=SteerGearDeg=0;
        Drive(DriveMotor.OutputNm,Brake,0);return;
    }
    if(!S.Actuators.bEnabled)
    {
        DriveMotor=ElectricDrive::Measure(DriveRequestNm,RelativeOmega,S.Actuators.Drive,S.DriveNm);
        SteerAvailableNm=S.SteeringTorqueNm;
        SteeringJoint->SetAngularDriveParams(SteeringPositionGain,SteeringVelocityGain,SteerAvailableNm*10000);
        SteerFilteredDeg=SteerGearDeg=Steer;
        Drive(DriveRequestNm,Brake,Steer); return;
    }
    DriveMotor=ElectricDrive::Step(Request,RelativeOmega,S.Actuators.Drive,S.DriveNm,Dt,Powered,DriveShaftStateNm);
    const double Delta=FMath::FindDeltaAngleDegrees(SteerFilteredDeg,Steer);
    SteerFilteredDeg=FMath::UnwindDegrees(SteerFilteredDeg+ElectricDrive::Lag(0,Delta,Dt,S.Actuators.Steer.ResponseS));
    SteerGearDeg=ElectricDrive::Backlash(SteerGearDeg,SteerFilteredDeg,S.Actuators.BacklashDeg);
    // Overspeed must cut motoring, not regenerative braking. Temporarily use
    // only the zero-velocity drive above rated rpm; otherwise a back-driven
    // module loses all control just when it needs braking. Hysteresis avoids
    // position/velocity mode chatter. No angular velocity is clamped.
    const bool WasOverspeed=bSteerOverspeed;
    const double MotorRpm=FMath::Abs(SteerMotorRpm);
    if(MotorRpm>S.Actuators.Steer.MaxRpm) bSteerOverspeed=true;
    else if(MotorRpm<S.Actuators.Steer.MaxRpm*.9) bSteerOverspeed=false;
    if(WasOverspeed!=bSteerOverspeed) SteeringJoint->SetOrientationDriveTwistAndSwing(!bSteerOverspeed && bSteerable,false);
    const double ServoRequest=(SteeringPositionGain*FMath::DegreesToRadians(FMath::FindDeltaAngleDegrees(SteerActualDeg,SteerGearDeg))-SteeringVelocityGain*SteerOmegaRadS)/10000.;
    const bool Motoring=!bSteerOverspeed && ServoRequest*SteerOmegaRadS>=0;
    SteerAvailableNm=ElectricDrive::OutputLimit(S.Actuators.Steer,SteerOmegaRadS,Motoring,S.SteeringTorqueNm);
    // A tiny nonzero engine limit avoids UE's 0=unlimited case.
    SteeringJoint->SetAngularDriveParams(SteeringPositionGain,SteeringVelocityGain,FMath::Max(.001,SteerAvailableNm)*10000);
    Drive(DriveMotor.OutputNm,Brake,SteerGearDeg);
}
void AWheelActor::Measure(double Radius,double Width)
{
    const FVector Axis=Tyre->GetUpVector(); // Cylinder's local Z is its axle.
    FVector Normal=FVector::UpVector, Surface=FVector::ZeroVector;
    FHitResult Hit; const FVector Center=Tyre->GetComponentLocation();
    if(GetWorld()->LineTraceSingleByObjectType(Hit,Center+FVector(0,0,5),Center-FVector(0,0,(Radius+1)*100),FCollisionObjectQueryParams(ECC_WorldStatic)))
    { Normal=Hit.ImpactNormal; Surface=Hit.ImpactPoint; }
    const double Axial=FVector::DotProduct(Axis,Normal);
    const double Extent=Radius*FMath::Sqrt(FMath::Max(0.,1-Axial*Axial)) + Width*.5*FMath::Abs(Axial);
    ClearanceM=FVector::DotProduct(Center-Surface,Normal)/100-Extent;
    bGrounded=ClearanceM<.02 && ClearanceM>-.05;
    const FVector Point=Center-Normal*(Extent*100);
    FVector ContactV=Tyre->GetPhysicsLinearVelocityAtPoint(Point)/100;
    if(TreadPads.Num()>0 && ContactData.NormalNs>1e-8)
    {
        FVector Sum=FVector::ZeroVector; double N=0;
        for(int I=0;I<TreadPads.Num();++I) if(TreadNormalN[I]>0) { Sum+=TreadPads[I]->GetPhysicsLinearVelocityAtPoint(TreadContactPointM[I]*100.)*TreadNormalN[I]/100.; N+=TreadNormalN[I]; }
        if(N>1e-8) ContactV=Sum/N;
    }
    OmegaRadS=FVector::DotProduct(Tyre->GetPhysicsAngularVelocityInRadians(),Knuckle->GetRightVector());
    LongSlipMps=-FVector::DotProduct(ContactV,Knuckle->GetForwardVector());
    LateralSlipMps=FVector::DotProduct(ContactV,Knuckle->GetRightVector());
    const FVector HubUp=Knuckle->GetUpVector(), Offset=BeltCarrier->GetComponentLocation()-Knuckle->GetComponentLocation();
    CompressionM=Compliance.bEnabled?FVector::DotProduct(Offset,HubUp)/100.:0;
    const FVector RelativeVelocity=BeltCarrier->GetPhysicsLinearVelocity()-Knuckle->GetPhysicsLinearVelocity()
        -FVector::CrossProduct(Knuckle->GetPhysicsAngularVelocityInRadians(),Offset);
    CompressionRateMps=Compliance.bEnabled?FVector::DotProduct(RelativeVelocity,HubUp)/100.:0;
    SpringEstimateN=Compliance.bEnabled?Compliance.ForceN(CompressionM,CompressionRateMps):0;
    SpringEnergyJ=Compliance.bEnabled?Compliance.StoredJ(CompressionM):0;
    if(Compliance.bEnabled && Compliance.bDetailed)
    {
        const FVector Local=Knuckle->GetComponentQuat().UnrotateVector(Offset)/100.;
        ShearXM=Local.X; ShearYM=Local.Y;
        if(!Compliance.Tread.bCoupled) SpringEnergyJ+=.5*Compliance.ShearNpm*(ShearXM*ShearXM+ShearYM*ShearYM);
        if(Compliance.Tread.bEnabled)
        {
            MeasureTread(Radius,Width); CompressionM=TreadMaxCompressionM;
            CompressionRateMps=0; SpringEstimateN=TreadSupportN; SpringEnergyJ=TreadStoredJ+(Compliance.Tread.bCoupled?0:.5*Compliance.ShearNpm*(ShearXM*ShearXM+ShearYM*ShearYM));
        }
        else
        {
        const double K=Compliance.TangentK(CompressionM);
        const double Rest=CompressionM-Compliance.ForceN(CompressionM,0)/K;
        ComplianceGuide->SetLinearPositionTarget(FVector(0,0,-Rest*100));
        ComplianceGuide->SetLinearDriveParams(K,Compliance.DampingNsPm,0);
        }
        // Geometric footprint estimate, not a measured pressure map or a
        // second contact force. The real solver still supplies all road forces.
        const double D=FMath::Clamp(CompressionM,0.,Radius);
        EstimatedPatchLengthM=2*FMath::Sqrt(FMath::Max(0.,2*Radius*D-D*D));
        EstimatedPeakPressurePa=EstimatedPatchLengthM>1e-6?4*NormalLoadN/(PI*Width*EstimatedPatchLengthM):0;
    }
    bTyreTravelLimit=Compliance.bEnabled && FMath::Abs(CompressionM)>=Compliance.TravelM*.98;
    if(Compliance.Tread.bEnabled) bTyreTravelLimit=bTreadBottomed;
    LoadedRadiusM=FVector::DotProduct(Knuckle->GetComponentLocation()-Surface,Normal)/100.;
    const double RelativeOmega=OmegaRadS-FVector::DotProduct(Knuckle->GetPhysicsAngularVelocityInRadians(),Knuckle->GetRightVector());
    bRollingRadiusValid=FloorContacts>0 && FMath::Abs(RelativeOmega)>1 && FMath::Abs(LongSlipMps)<.05;
    RollingRadiusM=bRollingRadiusValid?FVector::DotProduct(Tyre->GetPhysicsLinearVelocity()/100.,Knuckle->GetForwardVector())/RelativeOmega:0;
    if(MotorStator)
    {
        const FVector Up=MotorStator->GetUpVector();
        SteerOmegaRadS=FVector::DotProduct(Knuckle->GetPhysicsAngularVelocityInRadians()-MotorStator->GetPhysicsAngularVelocityInRadians(),Up);
    }
    if(bMecanum) MeasureMecanum();
}
