#pragma once
#include "Components/StaticMeshComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

namespace Vehicle3D
{
    inline void Color(UStaticMeshComponent* M, FLinearColor Tint)
    {
        if (auto* Base=LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/Materials/M_Lab.M_Lab")))
        { auto* D=UMaterialInstanceDynamic::Create(Base,M); D->SetVectorParameterValue(TEXT("Tint"),Tint); M->SetMaterial(0,D); }
    }
    inline UPhysicalMaterial* Material(UObject* Owner, double Mu)
    {
        auto* M=NewObject<UPhysicalMaterial>(Owner); M->Friction=Mu; M->StaticFriction=Mu; M->Restitution=0;
        M->bOverrideFrictionCombineMode=true; M->FrictionCombineMode=EFrictionCombineMode::Min;
        return M;
    }
    inline void Dynamic(UStaticMeshComponent* M, double Mass)
    {
        M->SetMobility(EComponentMobility::Movable); M->SetCollisionProfileName(TEXT("PhysicsActor"));
        M->SetLinearDamping(.01f); M->SetAngularDamping(.02f);
        M->BodyInstance.bUseCCD=true;
        // Preserve geometry-derived inertia instead of solver-only stabilization
        // that would change the tipping response when joints are added.
        M->BodyInstance.SetInertiaConditioningEnabled(false);
        M->BodyInstance.PositionSolverIterationCount=32; M->BodyInstance.VelocitySolverIterationCount=16;
        M->SetSimulatePhysics(true); M->SetMassOverrideInKg(NAME_None,Mass);
    }
    inline void Lock(UPhysicsConstraintComponent* J)
    {
        J->ConstraintInstance.bScaleLinearLimits=false;
        J->ConstraintInstance.ProfileInstance.bEnableMassConditioning=false;
        J->ConstraintInstance.ProfileInstance.bEnableShockPropagation=false;
        J->SetProjectionEnabled(false);
        J->SetAngularDriveAccelerationMode(false);
        J->SetLinearDriveAccelerationMode(false);
        J->SetLinearXLimit(LCM_Locked,0); J->SetLinearYLimit(LCM_Locked,0); J->SetLinearZLimit(LCM_Locked,0);
        J->SetAngularTwistLimit(ACM_Locked,0); J->SetAngularSwing1Limit(ACM_Locked,0); J->SetAngularSwing2Limit(ACM_Locked,0);
        J->SetDisableCollision(true);
    }
    inline UStaticMeshComponent* Box(AActor* Owner, USceneComponent* Parent, FVector WorldPosCm, FVector SizeM, double MassKg, FLinearColor Tint)
    {
        auto* M=NewObject<UStaticMeshComponent>(Owner);
        M->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));
        M->SetMobility(EComponentMobility::Movable);
        M->SetCollisionProfileName(TEXT("PhysicsActor")); M->SetWorldLocation(WorldPosCm); M->SetWorldScale3D(SizeM);
        M->BodyInstance.SetMassOverride(MassKg,true); M->RegisterComponent();
        if (Parent) M->AttachToComponent(Parent,FAttachmentTransformRules(EAttachmentRule::KeepWorld,true));
        Color(M,Tint); return M;
    }
}
