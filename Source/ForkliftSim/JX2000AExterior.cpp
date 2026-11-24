#include "JX2000AExterior.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"

namespace
{
    const TCHAR* const PartNames[]={TEXT("SM_JX_Body"),TEXT("SM_JX_ForkFrame"),
        TEXT("SM_JX_Tower"),TEXT("SM_JX_DriveWheel"),TEXT("SM_JX_CasterL"),TEXT("SM_JX_CasterR"),
        TEXT("SM_JX_LoadRollerL1"),TEXT("SM_JX_LoadRollerL2"),
        TEXT("SM_JX_LoadRollerR1"),TEXT("SM_JX_LoadRollerR2")};
}

AJX2000AExterior::AJX2000AExterior()
{
    PrimaryActorTick.bCanEverTick=false;
    RootComponent=CreateDefaultSubobject<USceneComponent>(TEXT("ExteriorFrame"));
    RootComponent->SetMobility(EComponentMobility::Static);
    for(const TCHAR* Name:PartNames)
    {
        auto* Part=CreateDefaultSubobject<UStaticMeshComponent>(FName(Name));
        Part->SetupAttachment(RootComponent);
        Part->SetMobility(EComponentMobility::Static);
        Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Part->SetGenerateOverlapEvents(false);
        Part->SetCanEverAffectNavigation(false);
        ExteriorParts.Add(Part);
    }
    Tags.Add(TEXT("JX2000AExteriorOnly"));
}

void AJX2000AExterior::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    RebuildExterior();
}

void AJX2000AExterior::RebuildExterior()
{
    for(int32 I=0;I<ExteriorParts.Num() && I<UE_ARRAY_COUNT(PartNames);++I)
    {
        const FString Path=FString::Printf(TEXT("/Game/JX2000A/%s.%s"),PartNames[I],PartNames[I]);
        if(auto* Mesh=LoadObject<UStaticMesh>(nullptr,*Path)) ExteriorParts[I]->SetStaticMesh(Mesh);
        // Shared chassis coordinate frame, centimetres in native FBX import.
        ExteriorParts[I]->SetRelativeTransform(FTransform::Identity);
    }
}
