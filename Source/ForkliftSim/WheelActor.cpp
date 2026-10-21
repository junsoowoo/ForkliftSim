#include "WheelActor.h"
#include "Vehicle3DHelpers.h"
#include "UObject/ConstructorHelpers.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "PhysicsEngine/BodySetup.h"

UBodySetup* UWheelTyreComponent::GetBodySetup()
{
    return RollingCollision?RollingCollision.Get():Super::GetBodySetup();
}
void UWheelTyreComponent::BuildRollingCollision()
{
    if(FParse::Param(FCommandLine::Get(),TEXT("StockTyreCollision"))) return;
    // The stock render cylinder has a 32-sided collider. A wheel must climb
    // those faces as it rolls; this particularly distorts low-grip behaviour.
    // Give this component a rounder collider without editing the engine asset.
    constexpr int32 Segments=128;
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
    RecreatePhysicsState();
}

AWheelActor::AWheelActor()
{
    Knuckle=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("SteeringKnuckle")); RootComponent=Knuckle;
    Tyre=CreateDefaultSubobject<UWheelTyreComponent>(TEXT("PhysicalTyre"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cylinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    Knuckle->SetStaticMesh(Cube.Object); Tyre->SetStaticMesh(Cylinder.Object);
    SteeringJoint=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("SteeringAxis"));
    Axle=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("WheelAxle"));
    ChassisCollisionFilter=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("ChassisTyreSelfCollisionFilter"));
    Tyre->OnComponentHit.AddDynamic(this,&AWheelActor::OnTyreHit);
}
void AWheelActor::Configure(UStaticMeshComponent* Chassis,const FVehicle3DSettings& S,bool Rear,UPhysicalMaterial* Material)
{
    bRear=Rear; bSteerable=Rear || S.bSwerve; MotorStator=Chassis; const FVector P=GetActorLocation();
    // A 20 cm bearing/knuckle assembly has finite rotational inertia. A 5 cm
    // proxy made the articulated axle needlessly stiff relative to the wheel.
    Knuckle->SetWorldScale3D(FVector(.20)); Vehicle3D::Dynamic(Knuckle,S.KnuckleMassKg);
    Knuckle->SetCollisionResponseToAllChannels(ECR_Ignore); Knuckle->SetVisibility(false);
    Tyre->SetWorldLocationAndRotation(P,FRotator(0,0,90)); Tyre->SetWorldScale3D(FVector(S.RadiusM*2,S.RadiusM*2,S.WidthM));
    CastChecked<UWheelTyreComponent>(Tyre)->BuildRollingCollision();
    Vehicle3D::Dynamic(Tyre,S.WheelMassKg); SetMu(Material); Tyre->SetAngularDamping(.02f);
    Tyre->SetNotifyRigidBodyCollision(true);
    Vehicle3D::Color(Tyre,FLinearColor(.035f,.045f,.06f));
    // Massless visual marks rotate with the real tyre body.
    for (double Side : { -1.,1. })
    {
        auto* Mark=NewObject<UStaticMeshComponent>(this); Mark->SetStaticMesh(Knuckle->GetStaticMesh());
        Mark->SetMobility(EComponentMobility::Movable); Mark->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Mark->RegisterComponent(); Mark->SetWorldLocation(P+FVector(0,Side*(S.WidthM*50+.5),0));
        Mark->SetWorldScale3D(FVector(S.RadiusM*1.5,.015,.035));
        Mark->AttachToComponent(Tyre,FAttachmentTransformRules::KeepWorldTransform); Vehicle3D::Color(Mark,FLinearColor(.75f,.85f,.9f));
    }
    SteeringJoint->SetWorldLocationAndRotation(P,FRotator(90,0,0));
    SteeringJoint->AttachToComponent(Chassis,FAttachmentTransformRules::KeepWorldTransform);
    Vehicle3D::Lock(SteeringJoint);
    if (S.bSwerve) SteeringJoint->SetAngularTwistLimit(ACM_Free,0);
    else if (Rear) SteeringJoint->SetAngularTwistLimit(ACM_Limited,55);
    SteeringJoint->SetConstrainedComponents(Chassis,NAME_None,Knuckle,NAME_None);
    SteeringJoint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
    SteeringJoint->SetOrientationDriveTwistAndSwing(bSteerable,false);
    SteeringJoint->SetAngularVelocityDriveTwistAndSwing(bSteerable,false);
    SteeringJoint->SetAngularVelocityTarget(FVector::ZeroVector);
    SteeringJoint->SetAngularDriveParams(50000000,2000000,S.SteeringTorqueNm*10000);
    Axle->SetWorldLocationAndRotation(P,FRotator(0,90,0)); Axle->AttachToComponent(Knuckle,FAttachmentTransformRules::KeepWorldTransform);
    Vehicle3D::Lock(Axle); Axle->SetAngularTwistLimit(ACM_Free,0);
    Axle->SetConstrainedComponents(Knuckle,NAME_None,Tyre,NAME_None);
    Axle->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);

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
    if(MotorStator && OtherActor==MotorStator->GetOwner())
    {
        ++ChassisContactCount;
        if(ChassisContactCount<=2) UE_LOG(LogTemp,Warning,TEXT("TYRE_CHASSIS_CONTACT rear=%d impulse_Ns=%.4f"),bRear?1:0,NormalImpulse.Size()/100);
    }
}
void AWheelActor::SetMu(UPhysicalMaterial* Material) { Tyre->SetPhysMaterialOverride(Material); Tyre->WakeAllRigidBodies(); }
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
    Axle->SetAngularVelocityDriveTwistAndSwing(Brake>0,false);
    Axle->SetAngularVelocityTarget(FVector::ZeroVector);
    Axle->SetAngularDriveParams(0,1000000,Brake*10000);
}
void AWheelActor::Measure(double Radius,double Width)
{
    const FVector Axis=Tyre->GetUpVector(); // Cylinder's local Z is its axle.
    const double Extent=Radius*FMath::Sqrt(FMath::Max(0.,1-Axis.Z*Axis.Z)) + Width*.5*FMath::Abs(Axis.Z);
    ClearanceM=Tyre->GetComponentLocation().Z/100-Extent;
    bGrounded=ClearanceM<.02 && ClearanceM>-.05;
    const FVector Point=Tyre->GetComponentLocation()-FVector(0,0,Extent*100);
    const FVector ContactV=Tyre->GetPhysicsLinearVelocityAtPoint(Point)/100;
    OmegaRadS=FVector::DotProduct(Tyre->GetPhysicsAngularVelocityInRadians(),Knuckle->GetRightVector());
    LongSlipMps=-FVector::DotProduct(ContactV,Knuckle->GetForwardVector());
    LateralSlipMps=FVector::DotProduct(ContactV,Knuckle->GetRightVector());
}
