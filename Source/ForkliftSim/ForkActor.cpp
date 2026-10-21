#include "ForkActor.h"
#include "Vehicle3DHelpers.h"
#include "UObject/ConstructorHelpers.h"

AForkActor::AForkActor()
{
    Carriage=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ForkCarriage")); RootComponent=Carriage;
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube")); Carriage->SetStaticMesh(Cube.Object);
    Guide=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("LiftGuide"));
}
void AForkActor::Configure(UStaticMeshComponent* Chassis,const FVehicle3DSettings& S)
{
    TravelM=S.ForkTravelM; const FVector P=GetActorLocation();
    Carriage->SetWorldScale3D(FVector(.12,.9,.45)); Vehicle3D::Dynamic(Carriage,S.ForkMassKg*.6);
    Vehicle3D::Color(Carriage,FLinearColor(.95f,.45f,.035f));
    Carriage->SetPhysMaterialOverride(Vehicle3D::Material(this,.4));
    for (double Y : { -30.,30. })
        Vehicle3D::Box(this,Carriage,P+FVector(65,Y,-15),FVector(1.4,.12,.07),S.ForkMassKg*.2,FLinearColor(.12f,.18f,.24f));
    Carriage->BodyInstance.UpdateMassProperties();
    Guide->SetWorldLocation(P+FVector(0,0,TravelM*50));
    Guide->AttachToComponent(Chassis,FAttachmentTransformRules::KeepWorldTransform);
    Vehicle3D::Lock(Guide); Guide->SetLinearZLimit(LCM_Limited,TravelM*50);
    Guide->SetConstrainedComponents(Chassis,NAME_None,Carriage,NAME_None);
    Guide->SetConstraintReferencePosition(EConstraintFrame::Frame2,FVector::ZeroVector);
    Guide->SetLinearPositionDrive(false,false,true); Guide->SetLinearVelocityDrive(false,false,true);
    Guide->SetLinearDriveParams(100000,15000,S.ForkForceN*100); SetLiftTarget(0);
}
void AForkActor::SetLiftTarget(double H)
{
    // Chaos drive target is frame 1 relative to frame 2, opposite to carriage height.
    Guide->SetLinearPositionTarget(FVector(0,0,(TravelM*.5-FMath::Clamp(H,0.,TravelM))*100));
}
