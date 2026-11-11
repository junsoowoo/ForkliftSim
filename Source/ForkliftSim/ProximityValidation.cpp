#include "ProximitySensors.h"
#include "StabilityLab.h"
#include "PalletLoadActor.h"
#include "ForkActor.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"

// Isolated scene-query fixture far outside the driving course. It does not
// move the vehicle, change collision profiles of production actors or feed
// test distances into live sensor measurements.
void FProximitySensorRig::ValidateScene(AStabilityLab& Lab)
{
    TArray<AActor*> Fixtures;auto* World=Lab.GetWorld();bool Pass=true;int Checks=0;
    auto Check=[&](const TCHAR* Name,bool OK){++Checks;Pass&=OK;UE_LOG(LogTemp,Display,TEXT("PROXIMITY_CHECK %s %s"),Name,OK?TEXT("PASS"):TEXT("FAIL"));};
    auto Box=[&](FVector PositionM,FVector SizeM,ECollisionEnabled::Type Mode)
    {
        auto* A=World->SpawnActor<AStaticMeshActor>(PositionM*100,FRotator::ZeroRotator);Fixtures.Add(A);auto* B=A->GetStaticMeshComponent();
        B->SetMobility(EComponentMobility::Movable);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));B->SetWorldScale3D(SizeM);B->SetCollisionProfileName(TEXT("BlockAll"));B->SetCollisionEnabled(Mode);return B;
    };
    ProximitySensor::FRay Ray;Ray.OriginM=FVector(300,0,30);
    auto* Solid=Box(FVector(300.5,0,30),FVector(.1,.4,.4),ECollisionEnabled::PhysicsOnly);
    Box(FVector(300.2,0,30),FVector(.1,.4,.4),ECollisionEnabled::QueryOnly);
    auto TestBodies=ProximitySensor::Targets(Lab);auto T=ProximitySensor::Trace(TestBodies,Ray,1.5);
    Check(TEXT("PhysicsOnly_nearest_solid_ignores_QueryOnly_visual"),T.Hit&&FMath::Abs(T.RangeM-.45)<.0001);
    auto* Near=Box(FVector(300.015,0,30),FVector(.01,.4,.4),ECollisionEnabled::QueryAndPhysics);
    TestBodies=ProximitySensor::Targets(Lab);T=ProximitySensor::Trace(TestBodies,Ray,1.5);
    Check(TEXT("Blind_zone_occluder_does_not_reveal_farther_target"),T.Hit&&FMath::Abs(T.RangeM-.01)<.0001);
    Near->SetCollisionEnabled(ECollisionEnabled::NoCollision);Solid->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    TestBodies=ProximitySensor::Targets(Lab);T=ProximitySensor::Trace(TestBodies,Ray,1.5);Check(TEXT("No_return_and_no_collision_visual"),!T.Hit);
    auto* Inside=Box(Ray.OriginM,FVector(.1,.2,.2),ECollisionEnabled::QueryAndPhysics);TestBodies=ProximitySensor::Targets(Lab);T=ProximitySensor::Trace(TestBodies,Ray,1.5);Check(TEXT("Origin_inside_obstacle_is_zero_distance"),T.Hit&&T.RangeM<1e-5);Inside->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    auto Settings=Lab.PayloadSettings;FString Error;
    const bool Loaded=Settings.bWarehouseAssets||Settings.UseWarehouseGeometry(Error);
    Check(TEXT("Warehouse_board_geometry_loaded"),Loaded);
    if(Loaded)
    {
        auto* P=World->SpawnActor<APalletLoadActor>(FVector(300,3,30)*100,FRotator::ZeroRotator);Fixtures.Add(P);P->Configure(Settings);P->Deck->SetSimulatePhysics(false);if(P->Cargo)Fixtures.Add(P->Cargo);
        TestBodies=ProximitySensor::Targets(Lab);Ray.OriginM=FVector(300-Settings.LengthM*.5-.1,3-.225,30+.075-Settings.PalletDatumM.Z);
        T=ProximitySensor::Trace(TestBodies,Ray,Settings.LengthM+.2);Check(TEXT("Actual_welded_pallet_opening_stays_open"),!T.Hit);
        Ray.OriginM.Y=3;T=ProximitySensor::Trace(TestBodies,Ray,Settings.LengthM+.2);Check(TEXT("Actual_welded_pallet_center_block_occludes"),T.Hit&&T.RangeM<.3);
    }
    for(AActor* A:Fixtures)if(IsValid(A))A->Destroy();
    UE_LOG(LogTemp,Display,TEXT("PROXIMITY_SCENE_VALIDATION checks=%d %s"),Checks,Pass?TEXT("PASS"):TEXT("FAIL"));
    if(!Pass)FGenericPlatformMisc::RequestExitWithStatus(false,1);
}
