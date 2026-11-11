#include "RollingReference.h"
#include "WheelActor.h"
#include "Vehicle3DHelpers.h"
#include "Engine/StaticMeshActor.h"
#include "EngineUtils.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PBDRigidsSolver.h"
#include "ContactLoadSampler.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

ARollingReference::ARollingReference()
{
    PrimaryActorTick.bCanEverTick=true; PrimaryActorTick.TickGroup=TG_PrePhysics;
    Body=CreateDefaultSubobject<UWheelTyreComponent>(TEXT("FreeRollingBody")); RootComponent=Body;
}
void ARollingReference::BeginPlay()
{
    Super::BeginPlay();
    FParse::Value(FCommandLine::Get(),TEXT("ReferenceShape="),Shape);
    FParse::Value(FCommandLine::Get(),TEXT("ReferenceSpeed="),Speed);
    if((Shape!=TEXT("cylinder") && Shape!=TEXT("sphere")) || !FMath::IsFinite(Speed) || FMath::Abs(Speed)>8 || FMath::Abs(Speed)<.1)
    { UE_LOG(LogTemp,Error,TEXT("Invalid rolling reference arguments")); FGenericPlatformMisc::RequestExitWithStatus(false,1); return; }
    const bool Sphere=Shape==TEXT("sphere");
    Body->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,Sphere?TEXT("/Engine/BasicShapes/Sphere.Sphere"):TEXT("/Engine/BasicShapes/Cylinder.Cylinder")));
    Body->SetWorldLocationAndRotation(FVector(0,0,34),FRotator(0,0,90));
    Body->SetWorldScale3D(Sphere?FVector(.6):FVector(.6,.6,.18));
    if(!Sphere) Body->BuildRollingCollision(256);
    Vehicle3D::Dynamic(Body,500);
    Body->SetLinearDamping(0); Body->SetAngularDamping(0);
    auto* Material=Vehicle3D::Material(this,.8);
    Material->SleepLinearVelocityThreshold=0; Material->SleepAngularVelocityThreshold=0; Material->SleepCounterThreshold=MAX_int32;
    Body->SetPhysMaterialOverride(Material);
    for(TActorIterator<AStaticMeshActor> I(GetWorld());I;++I) if(I->ActorHasTag(TEXT("StabilityRoad"))) I->GetStaticMeshComponent()->SetPhysMaterialOverride(Material);
    GetWorld()->GetPhysicsScene()->GetSolver()->SetPositionIterations(128);
    GetWorld()->GetPhysicsScene()->GetSolver()->SetVelocityIterations(32);
    if(!Sphere && !FParse::Param(FCommandLine::Get(),TEXT("LegacyTyreContacts")))
    {
        Sampler=GetWorld()->GetPhysicsScene()->GetSolver()->CreateAndRegisterSimCallbackObject_External<FContactLoadSampler>();
        auto* In=Sampler->GetProducerInputData_External(); In->bSmoothTyreContacts=true;
        In->WheelIds[0]=Body->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
        for(TActorIterator<AStaticMeshActor> I(GetWorld());I;++I) if(I->ActorHasTag(TEXT("StabilityRoad")))
            In->RoadId=I->GetStaticMeshComponent()->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
    }
}
void ARollingReference::EndPlay(const EEndPlayReason::Type Reason)
{
    if(Sampler) { GetWorld()->GetPhysicsScene()->GetSolver()->UnregisterAndFreeSimCallbackObject_External(Sampler); Sampler=nullptr; }
    Super::EndPlay(Reason);
}
double ARollingReference::EnergyJ() const
{
    const FVector V=Body->GetPhysicsLinearVelocity()/100.;
    const FQuat Q=Body->GetComponentQuat()*Body->BodyInstance.GetMassSpaceLocal().GetRotation();
    const FVector W=Q.UnrotateVector(Body->GetPhysicsAngularVelocityInRadians()), I=Body->GetInertiaTensor()/10000.;
    return .5*Body->GetMass()*V.SizeSquared()+.5*FVector::DotProduct(I*W,W)+Body->GetMass()*9.81*Body->GetCenterOfMass().Z/100.;
}
void ARollingReference::Tick(float Dt)
{
    Super::Tick(Dt); if(bDone) return; TimeS+=Dt;
    if(Sampler)
    {
        while(auto Output=Sampler->PopOutputData_External()) RefinedContacts+=Output->RefinedContacts;
        auto* In=Sampler->GetProducerInputData_External(); In->bSmoothTyreContacts=true;
        In->WheelIds[0]=Body->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
        for(TActorIterator<AStaticMeshActor> I(GetWorld());I;++I) if(I->ActorHasTag(TEXT("StabilityRoad")))
            In->RoadId=I->GetStaticMeshComponent()->BodyInstance.GetPhysicsActor()->GetGameThreadAPI().UniqueIdx().Idx;
    }
    if(TimeS>=2 && !bSeeded)
    {
        bSeeded=true; Body->SetPhysicsLinearVelocity(FVector(Speed*100,0,0));
        Body->SetPhysicsAngularVelocityInRadians(FVector(0,Speed/.3,0));
    }
    if(TimeS>=2.1 && SampleAt==0) { SampleAt=TimeS; StartJ=EnergyJ(); StartV=Body->GetPhysicsLinearVelocity().X/100.; }
    if(SampleAt>0) PeakGainJ=FMath::Max(PeakGainJ,EnergyJ()-StartJ);
    if(TimeS>=12)
    {
        bDone=true;
        const double V=Body->GetPhysicsLinearVelocity().X/100., Den=.5*Body->GetMass()*StartV*StartV;
        const bool Pass=FMath::IsFinite(V) && Den>1 && FMath::Abs(V/StartV-1)<.01 && PeakGainJ/Den<.01;
        UE_LOG(LogTemp,Display,TEXT("ROLLING_CONTACT refined=%lld"),RefinedContacts);
        UE_LOG(LogTemp,Display,TEXT("ROLLING_REFERENCE shape=%s start_mps=%.7f final_mps=%.7f delta_j=%.6f peak_gain_j=%.6f gain_fraction=%.7f mass_kg=%.3f %s"),*Shape,StartV,V,EnergyJ()-StartJ,PeakGainJ,PeakGainJ/FMath::Max(Den,1e-9),Body->GetMass(),Pass?TEXT("PASS"):TEXT("FAIL"));
        FGenericPlatformMisc::RequestExitWithStatus(false,Pass?0:1);
    }
}
