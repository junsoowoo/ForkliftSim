#pragma once
#include "CoreMinimal.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"

// The regular sensor path reads Chaos simulation shapes, including PhysicsOnly
// welded bodies. ISM/HISM has one body per instance, not a root BodyInstance.
// Query those actual physics+query shapes through the world's acceleration tree;
// never loop every instance for every laser ray, nor return actor identities.
struct FInstancePhysicsRays
{
    UWorld* World;FCollisionQueryParams Params;bool Any=false;
    explicit FInstancePhysicsRays(UWorld* W):World(W),Params(SCENE_QUERY_STAT(InstancePhysicalSensor),false)
    {
        for(TActorIterator<AActor> I(W);I;++I)
        {
            TInlineComponentArray<UPrimitiveComponent*> Parts(*I);
            for(auto* B:Parts)if(B->IsA<UInstancedStaticMeshComponent>()&&B->GetCollisionEnabled()==ECollisionEnabled::QueryAndPhysics)Any=true;else Params.AddIgnoredComponent(B);
        }
    }
    bool Trace(FVector Start,FVector End,double& DistanceCm) const
    {
        FHitResult Hit;if(!Any||!World->LineTraceSingleByChannel(Hit,Start,End,ECC_Visibility,Params))return false;
        if(!Hit.GetComponent()||!Hit.GetComponent()->IsA<UInstancedStaticMeshComponent>())return false;DistanceCm=Hit.Distance;return true;
    }
};
