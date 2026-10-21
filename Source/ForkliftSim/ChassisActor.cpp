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
    Body->SetWorldScale3D(S.bSwerve?FVector(S.WheelbaseM+2*S.SweepRadiusM()+.2,S.TrackM+2*S.SweepRadiusM()+.2,.22):FVector(2.2,.98,.5));
    const double BallastMass=S.ChassisMassKg*.6;
    Vehicle3D::Dynamic(Body,S.ChassisMassKg-80-BallastMass); Body->SetPhysMaterialOverride(Vehicle3D::Material(this,.4));
    Vehicle3D::Color(Body,FLinearColor(.95f,.45f,.035f));
    for (double Y : { -40.,40. })
        Vehicle3D::Box(this,Body,P+FVector(S.bSwerve?S.MastXM()*100:90,Y,75),FVector(.12,.12,2),40,FLinearColor(.05f,.08f,.12f));
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
