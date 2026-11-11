#include "NavigationLandmarks.h"
#include "StabilityLab.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Serialization/JsonSerializer.h"
bool NavigationLandmarks::Load(TArray<FBox>& Boxes,FString& Error)
{
    FString Text;TSharedPtr<FJsonObject> J;const TArray<TSharedPtr<FJsonValue>>* List=nullptr;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("NavigationLandmarks.json")))||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J||!J->TryGetArrayField(TEXT("boxes"),List)||List->Num()!=3){Error=TEXT("Invalid authored navigation pillars");return false;}
    Boxes.Reset();for(const auto& Entry:*List)
    {
        auto O=Entry->AsObject();const TArray<TSharedPtr<FJsonValue>> *C=nullptr,*S=nullptr;if(!O||!O->TryGetArrayField(TEXT("center_m"),C)||!O->TryGetArrayField(TEXT("size_m"),S)||C->Num()!=3||S->Num()!=3){Error=TEXT("Invalid pillar geometry");return false;}
        FBox B;for(int K=0;K<3;++K)if(!(*C)[K]->TryGetNumber(B.Center[K])||!(*S)[K]->TryGetNumber(B.Size[K])||!FMath::IsFinite(B.Center[K])||FMath::Abs(B.Center[K])>100||!FMath::IsFinite(B.Size[K])||B.Size[K]<.2||B.Size[K]>2){Error=TEXT("Pillar dimensions out of range");return false;}Boxes.Add(B);
    }return true;
}
void NavigationLandmarks::AppendSurvey(const TArray<FBox>& Boxes,TArray<SensorDrive::FSegment>& Map)
{
    for(const auto& B:Boxes){const FVector2D A(B.Center.X-B.Size.X*.5,B.Center.Y-B.Size.Y*.5),C(B.Center.X+B.Size.X*.5,B.Center.Y+B.Size.Y*.5);const FVector2D P[]={A,FVector2D(C.X,A.Y),C,FVector2D(A.X,C.Y)};for(int K=0;K<4;++K)Map.Add({P[K],P[(K+1)%4]});}
}
bool AStabilityLab::SetupNavigationLandmarks()
{
    if(!GetWorld()->GetMapName().Contains(TEXT("WarehouseLab"))||FParse::Param(FCommandLine::Get(),TEXT("MissionFixture"))||FParse::Param(FCommandLine::Get(),TEXT("SensorDriveFixture")))return true;
    TArray<NavigationLandmarks::FBox> Survey;if(!NavigationLandmarks::Load(Survey,Error))return false;
    for(const auto& P:Survey){auto* A=GetWorld()->SpawnActor<AStaticMeshActor>(P.Center*100,FRotator::ZeroRotator);auto* B=A->GetStaticMeshComponent();B->SetMobility(EComponentMobility::Movable);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));B->SetWorldScale3D(P.Size);B->SetCollisionProfileName(TEXT("BlockAll"));NavigationWalls.Add(A);}
    UE_LOG(LogTemp,Display,TEXT("NAVIGATION_LANDMARKS count=%d source=authored_static_survey measured_physics_rays=1 live_actor_pose_feedback=0"),Survey.Num());return true;
}
