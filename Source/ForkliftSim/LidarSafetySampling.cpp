#include "LidarSafety.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "JointSensors.h"
#include "DynamicsSensors.h"
#include "SensorLiftControl.h"
#include "SensorDrive.h"
#include "HandlingControl.h"
#include "MissionNavigation.h"
#include "EngineUtils.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Physics/Experimental/PhysInterface_Chaos.h"
#include "Physics/PhysicsInterfaceTypes.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Serialization/Archive.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"

void FLidarSafetyRig::Update(AStabilityLab& Lab)
{
    const double Now=Lab.TimeS;const auto& S=Sensor.Spec;
    if(Now+1e-9>=NextScanS)
    {
        NextScanS=(FMath::FloorToDouble((Now+1e-9)*S.Hz)+1)/S.Hz;
        const FTransform Pose=Lab.Chassis->Body->GetComponentTransform();
        FVector Origins[2];TArray<FVector> Directions;TArray<double> Best;TArray<LidarSafety::EReturn> States;
        Directions.SetNum(S.Rays*2);Best.Init(S.MaxM*100,S.Rays*2);States.Init(LidarSafety::EReturn::NoReturn,S.Rays*2);
        for(int I=0;I<2;++I){Origins[I]=Pose.TransformPositionNoScale(S.Mount(I)*100);for(int K=0;K<S.Rays;++K){const int N=I*S.Rays+K;const double A=S.Angle(I,K);Directions[N]=Pose.TransformVectorNoScale(FVector(FMath::Cos(A),FMath::Sin(A),0));if(S.Masked(I,K))States[N]=LidarSafety::EReturn::SelfMasked;}}
        // One physics read lock per body per scan, rather than per ray. Include
        // welded simulation shapes; no query-profile mutation or object labels.
        for(TActorIterator<AActor> It(Lab.GetWorld());It;++It)
        {
            if(!IsValid(*It)||!It->GetActorEnableCollision())continue;TInlineComponentArray<UPrimitiveComponent*> Components;It->GetComponents(Components);
            for(UPrimitiveComponent* B:Components)
            {
                const auto Mode=B->GetCollisionEnabled();if(!B->IsRegistered()||(Mode!=ECollisionEnabled::PhysicsOnly&&Mode!=ECollisionEnabled::QueryAndPhysics)||!B->BodyInstance.IsValidBodyInstance())continue;
                FPhysicsCommand::ExecuteRead(B->BodyInstance.GetPhysicsActor(),[&](const FPhysicsActorHandle& Actor)
                {
                    if(!Actor)return;const auto& API=Actor->GetGameThreadAPI();const FTransform BodyPose(API.R(),API.X());FVector LocalOrigins[2]={BodyPose.InverseTransformPositionNoScale(Origins[0]),BodyPose.InverseTransformPositionNoScale(Origins[1])};
                    PhysicsInterfaceTypes::FInlineShapeArray Shapes;FPhysicsInterface::GetAllShapes_AssumedLocked(Actor,Shapes);
                    FBox BodyBox(ForceInit);for(const auto& Ref:Shapes)if(Ref.Shape->GetSimEnabled()&&!Ref.Shape->GetIsProbe()&&Ref.Shape->GetGeometry()->HasBoundingBox()){const auto A=Ref.Shape->GetGeometry()->BoundingBox();BodyBox+=FBox(FVector(A.Min()),FVector(A.Max()));}
                    for(int N=0;N<Directions.Num();++N)
                    {
                        if(States[N]==LidarSafety::EReturn::SelfMasked)continue;const FVector Start=LocalOrigins[N/S.Rays],Dir=BodyPose.InverseTransformVectorNoScale(Directions[N]);
                        if(BodyBox.IsValid&&!FMath::LineBoxIntersection(BodyBox.ExpandBy(.01),Start,Start+Dir*Best[N],Dir*Best[N]))continue;
                        for(const auto& Ref:Shapes)
                        {
                            const auto* Shape=Ref.Shape;if(!Shape->GetSimEnabled()||Shape->GetIsProbe())continue;const auto* G=Shape->GetGeometry();if(!G)continue;
                            if(G->HasBoundingBox()){const auto A=G->BoundingBox();if(!FMath::LineBoxIntersection(FBox(FVector(A.Min()),FVector(A.Max())).ExpandBy(.01),Start,Start+Dir*Best[N],Dir*Best[N]))continue;}
                            Chaos::FReal D;Chaos::FVec3 P,Normal;int32 Face;if(G->Raycast(Start,Dir,Best[N],0,D,P,Normal,Face)&&D<=Best[N]){Best[N]=FMath::Max(0.,double(D));States[N]=LidarSafety::EReturn::Valid;}
                        }
                    }
                });
            }
        }
        for(int I=0;I<2;++I)
        {
            TArray<double> Ranges;TArray<LidarSafety::EReturn> Returns;for(int K=0;K<S.Rays;++K){const int N=I*S.Rays+K;Ranges.Add(Best[N]/100);Returns.Add(States[N]);if(ShowRays&&K%3==0)DrawDebugLine(Lab.GetWorld(),Origins[I],Origins[I]+Directions[N]*Best[N],States[N]==LidarSafety::EReturn::Valid?FColor::Orange:FColor::Cyan,false,.12,0,.6);}
            Sensor.Capture(I,Now,Ranges,Returns);
        }
    }
    Sensor.Deliver(Now);
    bool Critical=true;if(Lab.bSensorLiftRequested)Critical=Lab.SensorLift&&Lab.JointSensors&&Lab.DynamicsSensors&&Lab.JointSensors->Model.Usable(4,Now)&&Lab.JointSensors->Model.Usable(5,Now)&&Lab.DynamicsSensors->Model.Usable(1,Now);
    if(Lab.bSensorDriveRequested)Critical&=Lab.DriveSensors&&Lab.DriveSensors->Model.DataHealthy&&Lab.DriveSensors->Model.Calibrated;
    const bool Neutral=FMath::Abs(Lab.Throttle)<.01&&FMath::Abs(Lab.Strafe)<.01&&FMath::Abs(Lab.Steering)<.01&&FMath::Abs(Lab.Lift)<.01;
    const bool WasHold=Decision.DriveHold,WasLift=Decision.LiftHold;
    const auto* Controller=Lab.GetWorld()->GetFirstPlayerController();
    const bool Creep=!(Lab.MissionControl&&Lab.MissionControl->Model.Engaged)&&!(Lab.HandlingControl&&Lab.HandlingControl->Model.Working())&&((Controller&&(Controller->IsInputKeyDown(EKeys::LeftShift)||Controller->IsInputKeyDown(EKeys::RightShift)))||(Lab.IsPalletPerceptionTest()&&Lab.TestMode.EndsWith(TEXT("insert"))&&Now>=2&&Now<12)||(Lab.TestMode.EndsWith(TEXT("vehicle_guard_creep"))&&Now>=10&&Now<11));
    Decision.Step(Sensor,Now,Critical,Lab.SensorLift&&Lab.SensorLift->Model.Faulted,Neutral,EstopRequested,AckRequested,Creep,Lab.DriveSensors&&Lab.DriveSensors->Model.Faulted,[&](FVector2D P){return (Lab.HandlingControl&&Lab.HandlingControl->Model.TargetPoint(P))||(Lab.MissionControl&&Lab.MissionControl->CarriedPoint(P,Now));},Lab.MissionControl&&Lab.MissionControl->NavField(Now)?TFunction<bool(FVector2D)>([&](FVector2D P){return Lab.MissionControl->GuardPoint(P);}):TFunction<bool(FVector2D)>());EstopRequested=AckRequested=false;
    SawObstacle|=Decision.Obstacle;SawFault|=Decision.Faulted;
    if(WasHold&&!Decision.DriveHold)SawRecovered=true;
    if(WasLift&&!Decision.LiftHold&&Lab.JointSensors&&Lab.JointSensors->Model.Usable(4,Now))
    {RecoveryHeightM=Lab.JointSensors->Model.Channels[4].Last.Value;Lab.LiftTargetM=RecoveryHeightM;SawFaultRecovery=true;}
    if(!Csv||Now+1e-9<NextCsvS)return;NextCsvS=Now+.05;
    auto N=[](double V){return FString::Printf(TEXT("%.9f"),V);};auto B=[](bool V){return V?TEXT("1"):TEXT("0");};
    TArray<FString> C={N(Now),Decision.State,Decision.Reason,B(Decision.DriveHold),B(Decision.LiftHold),B(Decision.Faulted),B(Decision.Obstacle),FString::FromInt(Decision.Points),N(Decision.ClearanceM),FString::Printf(TEXT("%llu"),Sensor.Channels[0].Last.Sequence),FString::Printf(TEXT("%llu"),Sensor.Channels[1].Last.Sequence),N(Sensor.Channels[0].Last.CaptureS),N(Sensor.Channels[1].Last.CaptureS),B(Sensor.Fresh(0,Now)),B(Sensor.Fresh(1,Now)),B(Decision.AckRejected),N(Lab.SpeedMps),N(Lab.LateralMps),N(Lab.ForkHeightM)};
    FTCHARToUTF8 U(*(FString::Join(C,TEXT(","))+TEXT("\n")));Csv->Serialize((void*)U.Get(),U.Length());if(Now>=NextFlushS){Csv->Flush();NextFlushS=Now+5;}
}
