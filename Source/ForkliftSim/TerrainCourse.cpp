#include "TerrainCourse.h"
#include "Vehicle3DHelpers.h"
#include "Components/SceneComponent.h"
#include "Engine/World.h"

ATerrainCourse::ATerrainCourse() { RootComponent=CreateDefaultSubobject<USceneComponent>(TEXT("CourseRoot")); RootComponent->SetMobility(EComponentMobility::Static); }
void ATerrainCourse::Box(FVector Top,FVector Size,FRotator Rotation,FLinearColor Color)
{
    auto* B=NewObject<UStaticMeshComponent>(this);
    B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));
    B->SetMobility(EComponentMobility::Static); B->SetCollisionProfileName(TEXT("BlockAll"));
    B->SetupAttachment(RootComponent);
    B->SetWorldLocationAndRotation((Top-Rotation.RotateVector(FVector(0,0,Size.Z*.5)))*100,Rotation);
    B->SetWorldScale3D(Size); B->RegisterComponent();
    Vehicle3D::Color(B,Color); Surfaces.Add(B);
}
void ATerrainCourse::Configure(const FString& Preset,double Mu,double ApproachOffsetM)
{
    Mode=Preset;
    if(Mode==TEXT("ramp"))
    {
        const double Angle=FMath::DegreesToRadians(SlopeDeg), Rise=8*FMath::Tan(Angle);
        // Continuous top plane from X=3,Z=0 to X=11,Z=Rise, then a plateau.
        Box(FVector(7+ApproachOffsetM,0,Rise*.5),FVector(8/FMath::Cos(Angle),8,.2),FRotator(SlopeDeg,0,0),FLinearColor(.15,.35,.4));
        Box(FVector(25.5+ApproachOffsetM,0,Rise),FVector(29,8,.2),FRotator::ZeroRotator,FLinearColor(.18,.38,.4));
    }
    if(Mode==TEXT("step")) Box(FVector(22+ApproachOffsetM,0,StepM),FVector(36,8,.2),FRotator::ZeroRotator,FLinearColor(.5,.32,.1));
    if(Mode==TEXT("halfstep")) Box(FVector(22+ApproachOffsetM,-2,StepM),FVector(36,4,.2),FRotator::ZeroRotator,FLinearColor(.5,.32,.1));
    if(Mode==TEXT("split"))
    {
        // Two coplanar half-lanes replace, not overlap, the original floor.
        Box(FVector(15,-10,0),FVector(80,20,.2),FRotator::ZeroRotator,FLinearColor(.12,.35,.55));
        Box(FVector(15,10,0),FVector(80,20,.2),FRotator::ZeroRotator,FLinearColor(.25,.28,.3));
    }
    SetMu(Mu);
}
void ATerrainCourse::SetMu(double Mu)
{
    Materials.Empty();
    for(int I=0;I<Surfaces.Num();++I)
    {
        auto* M=Vehicle3D::Material(this,Mode==TEXT("split") && I==0?Mu*.1:Mu);
        Materials.Add(M); Surfaces[I]->SetPhysMaterialOverride(M);
    }
}
