#pragma once
#include "Components/StaticMeshComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Chaos/ChaosConstraintSettings.h"

namespace Vehicle3D
{
    // UE's constraint adapter applies additional conversion factors (1.5 for
    // angular K/C in UE 5.6). Compensate only physical SI springs; controller
    // gains tuned in engine units are intentionally separate.
    inline void AngularSpring(UPhysicsConstraintComponent* J,double KNmPerRad,double CNmsPerRad,double CapNm=0)
    {
        const double KS=Chaos::ConstraintSettings::AngularDriveStiffnessScale();
        const double CS=Chaos::ConstraintSettings::AngularDriveDampingScale();
        checkf(KS>0 && CS>0,TEXT("Physical angular springs require positive Chaos conversion factors"));
        J->SetAngularDriveParams(KNmPerRad*10000/KS,CNmsPerRad*10000/CS,CapNm*10000);
    }
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
    inline UStaticMeshComponent* Box(AActor* Owner, USceneComponent* Parent, FVector WorldPosCm, FVector SizeM, double MassKg, FLinearColor Tint,FQuat Orientation=FQuat::Identity)
    {
        auto* M=NewObject<UStaticMeshComponent>(Owner);
        M->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));
        M->SetMobility(EComponentMobility::Movable);
        M->SetCollisionProfileName(TEXT("PhysicsActor")); M->SetWorldLocation(WorldPosCm);if(!Orientation.IsIdentity())M->SetWorldRotation(Orientation); M->SetWorldScale3D(SizeM);
        M->BodyInstance.SetMassOverride(MassKg,true); M->RegisterComponent();
        if (Parent) M->AttachToComponent(Parent,FAttachmentTransformRules(EAttachmentRule::KeepWorld,true));
        Color(M,Tint); return M;
    }
}
