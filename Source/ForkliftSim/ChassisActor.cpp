#include "ChassisActor.h"
#include "Vehicle3DHelpers.h"
#include "UObject/ConstructorHelpers.h"

AChassisActor::AChassisActor()
{
    Body=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ChassisBody")); RootComponent=Body;
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube")); Body->SetStaticMesh(Cube.Object);
}
void AChassisActor::Configure(const FVehicle3DSettings& S)
{
    const FVector P=GetActorLocation();
    Body->SetWorldScale3D(S.bReferenceGeometry?FVector(S.BodyLengthM-.24,S.BodyWidthM,.22):S.bSwerve?FVector(S.WheelbaseM+2*S.SweepRadiusM()+.2,S.TrackM+2*S.SweepRadiusM()+.2,.22):FVector(2.2,.98,.5));
    const double BallastMass=S.ChassisMassKg*.6;
    const double FrameMass=S.ChassisMassKg-80-BallastMass;
    Vehicle3D::Dynamic(Body,S.bReferenceGeometry?FrameMass*.32:FrameMass); Body->SetPhysMaterialOverride(Vehicle3D::Material(this,.4));
    if(S.bReferenceGeometry)
    {
        // Headlength includes the mast and fork face. A full-length solid
        // deck touched the fork heel at lift=0.30 m and jammed the actuator.
        // Leave 120 mm physical clearance while keeping the published rear edge.
        Vehicle3D::Box(this,Body,P+FVector(-(S.BodyLengthM-.12)*50,0,0),FVector(.12,S.BodyWidthM,.22),FrameMass*.03,FLinearColor(.24,.27,.30));
        Vehicle3D::Box(this,Body,P+FVector(-12,0,36),FVector(1.06,.94,.52),FrameMass*.64,FLinearColor(.24,.27,.30));
        Vehicle3D::Box(this,Body,P+FVector(-12,0,62.6),FVector(1,.91,.016),FrameMass*.01,FLinearColor(.24,.27,.30));
    }
    Vehicle3D::Color(Body,FLinearColor(.95f,.45f,.035f));
    for (double Sign : { -1.,1. })
    {
        const double H=S.bReferenceGeometry?S.LoweredMastHeightM-.38:2.;
        const double Z=S.bReferenceGeometry?(.38+H*.5-(2*S.RadiusM+.19))*100:75;
        Vehicle3D::Box(this,Body,P+FVector(S.bSwerve?S.MastXM()*100:90,Sign*(S.bReferenceGeometry?33.5:40),Z),S.bReferenceGeometry?FVector(.08,.07,H):FVector(.12,.12,H),S.bReferenceGeometry?35:40,FLinearColor(.05f,.08f,.12f));
    }
    if(S.bReferenceGeometry) Vehicle3D::Box(this,Body,P+FVector(S.MastXM()*100,0,(S.LoweredMastHeightM-.04-(2*S.RadiusM+.19))*100),FVector(.14,.74,.08),10,FLinearColor(.05f,.08f,.12f));
    // Move an actual welded mass, so both CoM AND rotational inertia follow its
    // geometry. Raising 60% of the chassis mass by h/0.6 raises chassis CoM by h.
    // At the normal setting this ballast sits inside the chassis collision box.
    Vehicle3D::Box(this,Body,P+FVector(-20,0,S.ChassisComOffsetM*100/.6+(S.bSwerve?-25:0)),
        FVector(.9,S.bSwerve?.5:.65,.25),BallastMass,FLinearColor(.15f,.65f,.8f));
    Body->BodyInstance.UpdateMassProperties();
    auto* Marker=NewObject<UStaticMeshComponent>(this);
    Marker->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Sphere.Sphere")));
    Marker->SetMobility(EComponentMobility::Movable); Marker->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Marker->RegisterComponent(); Marker->SetWorldLocation(Body->GetCenterOfMass()); Marker->SetWorldScale3D(FVector(.13));
    Marker->AttachToComponent(Body,FAttachmentTransformRules::KeepWorldTransform);
    Vehicle3D::Color(Marker,FLinearColor(1,.05f,.05f));
}
