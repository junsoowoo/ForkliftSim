#include "ProximitySensors.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "WheelActor.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "InstancePhysicsRays.h"
#include "EngineUtils.h"
#include "DrawDebugHelpers.h"
#include "Serialization/Archive.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Physics/Experimental/PhysInterface_Chaos.h"
#include "Physics/PhysicsInterfaceTypes.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

TArray<TWeakObjectPtr<UPrimitiveComponent>> ProximitySensor::Targets(AStabilityLab& Lab)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(Forklift_ProximityTargets);
    TArray<TWeakObjectPtr<UPrimitiveComponent>> Result;
    TSet<AActor*> Self={&Lab,Lab.Chassis.Get(),Lab.Fork.Get()};
    if(Lab.Fork)for(AForkTineActor* T:Lab.Fork->Tines)Self.Add(T);
    for(AWheelActor* Wheel:Lab.Wheels)Self.Add(Wheel);
    for(TActorIterator<AActor> It(Lab.GetWorld());It;++It)
    {
        if(Self.Contains(*It)||!IsValid(*It)||!It->GetActorEnableCollision())continue;
        TInlineComponentArray<UPrimitiveComponent*> Components;It->GetComponents(Components);
        for(UPrimitiveComponent* B:Components)
        {
            const auto Mode=B->GetCollisionEnabled();
            if(!B->IsRegistered()||(Mode!=ECollisionEnabled::QueryAndPhysics&&Mode!=ECollisionEnabled::PhysicsOnly)||(!B->BodyInstance.IsValidBodyInstance()&&!B->IsA<UInstancedStaticMeshComponent>()))continue;
            Result.Add(B);
        }
    }
    return Result;
}
ProximitySensor::FTruth ProximitySensor::Trace(const TArray<TWeakObjectPtr<UPrimitiveComponent>>& Bodies,FRay Ray,double MaxM,const FInstancePhysicsRays* SharedInstances)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(Forklift_ProximityTrace);
    FTruth T;T.Ray=Ray;double Best=MaxM*100;
    const FVector Start=Ray.OriginM*100;
    TUniquePtr<FInstancePhysicsRays> OwnInstances;if(!SharedInstances&&!Bodies.IsEmpty()&&Bodies[0].IsValid()){OwnInstances=MakeUnique<FInstancePhysicsRays>(Bodies[0]->GetWorld());SharedInstances=OwnInstances.Get();}
    if(SharedInstances){double D;if(SharedInstances->Trace(Start,Start+Ray.Direction*Best,D)){Best=D;T.Hit=true;T.RangeM=D/100.;}}
    TSet<const void*> VisitedPhysicsBodies;
    for(const auto& Weak:Bodies)
    {
        auto* B=Weak.Get();if(!IsValid(B)||B->IsA<UInstancedStaticMeshComponent>()||!B->BodyInstance.IsValidBodyInstance())continue;
        const auto Mode=B->GetCollisionEnabled();if(Mode!=ECollisionEnabled::PhysicsOnly&&Mode!=ECollisionEnabled::QueryAndPhysics)continue;
        FPhysicsCommand::ExecuteRead(B->BodyInstance.GetPhysicsActor(),[&](const FPhysicsActorHandle& Actor)
        {
            if(!Actor||Best<=UE_SMALL_NUMBER||!Ray.Direction.IsNormalized()||VisitedPhysicsBodies.Contains(Actor))return;VisitedPhysicsBodies.Add(Actor);
            const auto& API=Actor->GetGameThreadAPI();const FTransform Pose(API.R(),API.X());
            const FVector LocalStart=Pose.InverseTransformPositionNoScale(Start),LocalDirection=Pose.InverseTransformVectorNoScale(Ray.Direction);
            PhysicsInterfaceTypes::FInlineShapeArray Shapes;FPhysicsInterface::GetAllShapes_AssumedLocked(Actor,Shapes);
            for(const auto& ShapeRef:Shapes)
            {
                if(Best<=UE_SMALL_NUMBER)break; // Origin already overlaps a shape; zero range is the closest possible hit.
                const auto* Shape=ShapeRef.Shape;if(!Shape->GetSimEnabled()||Shape->GetIsProbe())continue;
                const auto* Geometry=Shape->GetGeometry();if(!Geometry)continue;
                if(Geometry->HasBoundingBox())
                {
                    const auto Bounds=Geometry->BoundingBox();const FBox Box(FVector(Bounds.Min()),FVector(Bounds.Max()));
                    if(!FMath::LineBoxIntersection(Box.ExpandBy(.01),LocalStart,LocalStart+LocalDirection*Best,LocalDirection*Best))continue;
                }
                Chaos::FReal Distance;Chaos::FVec3 Position,Normal;int32 Face;
                if(Geometry->Raycast(LocalStart,LocalDirection,Best,0,Distance,Position,Normal,Face)&&Distance<=Best)
                {Best=FMath::Max(0.,double(Distance));T.Hit=true;T.RangeM=Best/100.;}
            }
        });
    }
    return T;
}
namespace
{
    // One immutable snapshot per acquisition, not per ray. Keep the actual
    // Chaos geometry alive while all aperture rays use the SAME measured pose.
    // No component bounds, names, pallet identities or query-only substitutes.
    class FProximityBatch
    {
        struct FShape {Chaos::FImplicitObjectPtr Geometry;FBox Bounds;};
        struct FBody {FTransform Pose;TArray<FShape> Shapes;FBox Bounds=FBox(ForceInit);};
        TArray<FBody> Snapshot;const FInstancePhysicsRays& Instances;
    public:
        FProximityBatch(const TArray<TWeakObjectPtr<UPrimitiveComponent>>& Bodies,const FInstancePhysicsRays& InInstances):Instances(InInstances)
        {
            TRACE_CPUPROFILER_EVENT_SCOPE(Forklift_ProximitySnapshot);
            TSet<const void*> Visited;
            for(const auto& Weak:Bodies)
            {
                auto* B=Weak.Get();if(!IsValid(B)||B->IsA<UInstancedStaticMeshComponent>()||!B->BodyInstance.IsValidBodyInstance())continue;
                const auto Mode=B->GetCollisionEnabled();if(Mode!=ECollisionEnabled::PhysicsOnly&&Mode!=ECollisionEnabled::QueryAndPhysics)continue;
                FPhysicsCommand::ExecuteRead(B->BodyInstance.GetPhysicsActor(),[&](const FPhysicsActorHandle& Actor)
                {
                    if(!Actor||Visited.Contains(Actor))return;Visited.Add(Actor);
                    const auto& API=Actor->GetGameThreadAPI();FBody Body;Body.Pose=FTransform(API.R(),API.X());
                    PhysicsInterfaceTypes::FInlineShapeArray Shapes;FPhysicsInterface::GetAllShapes_AssumedLocked(Actor,Shapes);
                    bool Bounded=true;
                    for(const auto& Ref:Shapes)
                    {
                        const auto* Shape=Ref.Shape;if(!Shape->GetSimEnabled()||Shape->GetIsProbe())continue;
                        auto* G=Shape->GetGeometry();if(!G)continue;
                        FShape Copy;Copy.Geometry=G;Copy.Bounds=FBox(ForceInit);
                        if(G->HasBoundingBox()){const auto Box=G->BoundingBox();Copy.Bounds=FBox(FVector(Box.Min()),FVector(Box.Max())).ExpandBy(.01);Body.Bounds+=Copy.Bounds;}
                        else Bounded=false;
                        Body.Shapes.Add(MoveTemp(Copy));
                    }
                    if(!Bounded)Body.Bounds=FBox(ForceInit);
                    if(!Body.Shapes.IsEmpty())Snapshot.Add(MoveTemp(Body));
                });
            }
        }
        ProximitySensor::FTruth Trace(ProximitySensor::FRay Ray,double MaxM) const
        {
            TRACE_CPUPROFILER_EVENT_SCOPE(Forklift_ProximityBatchTrace);
            ProximitySensor::FTruth T;T.Ray=Ray;double Best=MaxM*100;const FVector Start=Ray.OriginM*100;
            double D;if(Instances.Trace(Start,Start+Ray.Direction*Best,D)){Best=D;T.Hit=true;T.RangeM=D/100.;}
            if(!Ray.Direction.IsNormalized())return T;
            for(const auto& Body:Snapshot)
            {
                if(Best<=UE_SMALL_NUMBER)break;
                const FVector LocalStart=Body.Pose.InverseTransformPositionNoScale(Start),LocalDirection=Body.Pose.InverseTransformVectorNoScale(Ray.Direction);
                if(Body.Bounds.IsValid&&!FMath::LineBoxIntersection(Body.Bounds,LocalStart,LocalStart+LocalDirection*Best,LocalDirection*Best))continue;
                for(const auto& Shape:Body.Shapes)
                {
                    if(Best<=UE_SMALL_NUMBER)break;
                    if(Shape.Bounds.IsValid&&!FMath::LineBoxIntersection(Shape.Bounds,LocalStart,LocalStart+LocalDirection*Best,LocalDirection*Best))continue;
                    Chaos::FReal Distance;Chaos::FVec3 P,N;int32 Face;
                    if(Shape.Geometry->Raycast(LocalStart,LocalDirection,Best,0,Distance,P,N,Face)&&Distance<=Best)
                    {Best=FMath::Max(0.,double(Distance));T.Hit=true;T.RangeM=Best/100.;}
                }
            }
            return T;
        }
    };
}
ProximitySensor::FRay ProximitySensor::Mount(const AForkTineActor& Tine,int Kind,const FSpec& S)
{
    const double Length=Tine.Structure.LengthM,HalfThickness=Tine.Structure.ThicknessM*.5;
    int Index=Kind==0?0:Tine.Sections.Num()-1;
    double Along=Kind==0?-Length/(2*Tine.Sections.Num())+S.ForwardOffsetM:Length/(2*Tine.Sections.Num())+S.ForwardOffsetM;
    if(Kind>=2)
    {
        const double Fraction=Kind==2?S.NearFraction:S.FarFraction;
        Index=FMath::Min(Tine.Sections.Num()-1,int(FMath::FloorToDouble(Fraction*Tine.Sections.Num())));
        Along=(Fraction-(Index+.5)/Tine.Sections.Num())*Length;
    }
    const auto* B=Tine.Sections[Index].Get();const FVector Up=B->GetUpVector(),Forward=B->GetForwardVector();FRay Ray;
    Ray.OriginM=B->GetComponentLocation()/100.+Forward*Along+Up*(HalfThickness+(Kind<2?S.RangeAboveTopM:S.PresenceAboveTopM));
    Ray.Direction=Kind<2?Forward:Up;return Ray;
}
void FProximitySensorRig::Update(AStabilityLab& Lab)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(Forklift_ProximityUpdate);
    const double NowS=Lab.TimeS;
    if(FParse::Param(FCommandLine::Get(),TEXT("ProximityValidate"))&&!Validated&&NowS>.2){Validated=true;ValidateScene(Lab);}
    if(Lab.Fork&&Lab.Fork->Tines.Num()==2&&NowS+1e-9>=NextCaptureS)
    {
        Bodies=ProximitySensor::Targets(Lab);FInstancePhysicsRays SharedInstances(Lab.GetWorld());
        static const bool Compare=FParse::Param(FCommandLine::Get(),TEXT("ProximityCompareTracing"));
        static const bool UseReference=FParse::Param(FCommandLine::Get(),TEXT("ProximityReferenceTracing"));
        TUniquePtr<FProximityBatch> Scan;if(!UseReference||Compare)Scan=MakeUnique<FProximityBatch>(Bodies,SharedInstances);int Checks=0;
        auto TraceRay=[&](ProximitySensor::FRay Ray,double MaxM)
        {
            if(!Scan)return ProximitySensor::Trace(Bodies,Ray,MaxM,&SharedInstances);
            const auto T=Scan->Trace(Ray,MaxM);
            if(Compare)
            {
                const auto Reference=ProximitySensor::Trace(Bodies,Ray,MaxM,&SharedInstances);
                checkf(T.Hit==Reference.Hit&&(!T.Hit||FMath::Abs(T.RangeM-Reference.RangeM)<1e-9),TEXT("Proximity batch/reference ray mismatch"));++Checks;
            }
            return T;
        };
        NextCaptureS=(FMath::FloorToDouble((NowS+1e-9)*Model.Spec.Hz)+1)/Model.Spec.Hz;
        for(int I=0;I<ProximitySensor::Count;++I)
        {
            const auto& Tine=*Lab.Fork->Tines[I%2];if(Tine.Sections.IsEmpty())continue;
            const auto Ray=ProximitySensor::Mount(Tine,I/2,Model.Spec);const double MaxM=I<4?Model.Spec.MaxM:Model.Spec.PresenceMaxM;
            auto T=TraceRay(Ray,MaxM);
            if(I>=4&&Model.Spec.PresenceApertureM>0)
            {
                const auto Forward=Tine.Sections[0]->GetForwardVector(),Side=Tine.Sections[0]->GetRightVector();
                for(int Axis=0;Axis<2&&!T.Hit;++Axis)for(int K=-4;K<=4&&!T.Hit;++K)if(K){auto Spot=Ray;Spot.OriginM+=(Axis?Side:Forward)*(K*Model.Spec.PresenceApertureM/4);const auto Return=TraceRay(Spot,MaxM);if(Return.Hit)T=Return;if(ShowRays)DrawDebugLine(Lab.GetWorld(),Spot.OriginM*100,(Spot.OriginM+Spot.Direction*(Return.Hit?Return.RangeM:MaxM))*100,Return.Hit?FColor::Orange:FColor::Cyan,false,.12,0,.4);}
            }
            T.Channel=I;T.TimeS=NowS;Model.Capture(T);
            if(ShowRays)DrawDebugLine(Lab.GetWorld(),Ray.OriginM*100,(Ray.OriginM+Ray.Direction*(T.Hit?T.RangeM:MaxM))*100,T.Hit?FColor::Orange:FColor::Cyan,false,.12,0,1.5);
        }
        if(Compare)UE_LOG(LogTemp,Display,TEXT("PROXIMITY_BATCH_CHECK sim_s=%.6f checks=%d PASS"),NowS,Checks);
    }
    Model.Deliver(NowS);if(!Csv||NowS+1e-9<NextCsvS)return;NextCsvS=NowS+.05;
    auto N=[](double V){return FString::Printf(TEXT("%.9f"),V);};
    for(int I=0;I<ProximitySensor::Count;++I)
    {
        const auto& C=Model.Channels[I];const auto& P=C.Last;const auto& T=P.Truth;const bool Transport=Model.TransportValid(I,NowS),Valid=Model.MeasurementValid(I,NowS);
        TArray<FString> Cells={N(NowS),ProximitySensor::Id(I),Model.Status(I,NowS),Transport?TEXT("1"):TEXT("0"),Valid?TEXT("1"):TEXT("0"),Transport?ProximitySensor::ReadingName(P.Reading):TEXT("unavailable"),FString::Printf(TEXT("%llu"),C.Have?P.Sequence:0),C.Have?N(T.TimeS):TEXT(""),C.Have?N(P.DeliveryS):TEXT(""),C.Have?N(NowS-T.TimeS):TEXT(""),Valid&&I<4?N(P.RangeM):TEXT(""),Valid&&I>=4?(P.Detected?TEXT("1"):TEXT("0")):TEXT(""),C.Have?(T.Hit?TEXT("1"):TEXT("0")):TEXT(""),C.Have&&T.Hit?N(T.RangeM):TEXT("")};
        for(int A=0;A<3;++A)Cells.Add(C.Have?N(T.Ray.OriginM[A]):TEXT(""));for(int A=0;A<3;++A)Cells.Add(C.Have?N(T.Ray.Direction[A]):TEXT(""));
        Cells.Add(FString::Printf(TEXT("%llu"),C.Dropped));Cells.Add(Model.Fault(I,NowS)?TEXT("1"):TEXT("0"));
        FTCHARToUTF8 Utf8(*(FString::Join(Cells,TEXT(","))+TEXT("\n")));Csv->Serialize((void*)Utf8.Get(),Utf8.Length());
    }
    if(NowS>=NextFlushS){Csv->Flush();NextFlushS=NowS+5;}
}
