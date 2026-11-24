#include "CampusOperations.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "Engine/StaticMeshActor.h"
#include "WheelActor.h"
#include "PalletPerception.h"
#include "LidarSafety.h"
#include "ProximitySensors.h"
#include "Components/SceneComponent.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonSerializer.h"

bool FPayloadSettings::UseCampusGeometry(FString& Error)
{
    FString Text;TSharedPtr<FJsonObject> J;const TArray<TSharedPtr<FJsonValue>>* Parts=nullptr;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectContentDir()/TEXT("../ContentSource/Campus/PalletFourWay1100.json")))||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J||!J->TryGetArrayField(TEXT("parts"),Parts)||Parts->Num()<9||Parts->Num()>80){Error=TEXT("Invalid campus pallet parts");return false;}
    BoardCentersM.Reset();BoardSizesM.Reset();
    for(const auto& P:*Parts)
    {
        auto O=P->AsObject();const TArray<TSharedPtr<FJsonValue>> *C=nullptr,*S=nullptr;
        if(!O||!O->TryGetArrayField(TEXT("center_m"),C)||!O->TryGetArrayField(TEXT("size_m"),S)||C->Num()!=3||S->Num()!=3){Error=TEXT("Invalid campus pallet box");return false;}
        FVector Center,Size;for(int I=0;I<3;++I){Center[I]=(*C)[I]->AsNumber();Size[I]=(*S)[I]->AsNumber();}
        if(Center.ContainsNaN()||Size.ContainsNaN()||Size.GetMin()<.005||Size.GetMax()>1.11){Error=TEXT("Invalid campus pallet dimensions");return false;}BoardCentersM.Add(Center);BoardSizesM.Add(Size);
    }
    bWarehouseAssets=bFourWay=true;bCargoSecured=false;LengthM=WidthM=1.1;
    HoleSpacingM=ForkSpacingM=.46;HoleWidthM=.28;HoleHeightM=.09;PalletDatumM=BoardCentersM[0];PalletTopM=.12;
    PalletMeshPath=TEXT("/Game/Campus/SM_PalletFourWay1100");CargoMeshPath=TEXT("/Game/Warehouse/Assets/Props/WM00221-Scatter_Boxes/SM_WM00221-Scatter_Boxes_2");
    CargoLengthM=CargoWidthM=.85;CargoHeightM=.76;CargoBoundsMinM=FVector::ZeroVector;
    return Validate(Error);
}
void AStabilityLab::InitializeCampusRig()
{
    TArray<AActor*> Rig={Chassis.Get(),Fork.Get()};for(AWheelActor* W:Wheels)Rig.Add(W);for(AForkTineActor* T:Fork->Tines)Rig.Add(T);
    // Parts and joints are constructed directly at their placement, so each
    // body and constraint shares the same initialization frame.
    for(auto* A:Rig)A->SetOwner(this);
    for(AWheelActor* W:Wheels){W->Tyre->SetCollisionObjectType(ECC_PhysicsBody);W->Tyre->SetCollisionResponseToAllChannels(ECR_Block);}
    TargetHeadingDeg=CampusSpawn.Rotator().Yaw;
    if(Pallet){PayloadState.Start=Pallet->GetActorLocation();CargoState.InitialRelativeM=Pallet->Deck->GetComponentQuat().UnrotateVector(Pallet->Cargo->GetActorLocation()-Pallet->GetActorLocation())/100.;}
    if(PalletRecognition)
    {
        PalletRecognition->NextDetectionS=FCString::Atoi(*VehicleId.Right(2))*.16;auto& S=PalletRecognition->Spec;S.LayerSearchM=.16;S.Spacing=.46;S.OpeningWidth=.28;S.OpeningHeight=.09;S.SupportHalfWidth=.46;S.Width=1.1;S.FullDepthM=1.0;S.ForkSpacing=.46;
    }
    if(ProximitySensors)ProximitySensors->Model.Spec.PresenceApertureM=.04;
    if(VehicleGuard&&!Campus->Validation)VehicleGuard->Sensor.Spec.MaxM=15;
    if(Campus&&Campus->BoardEnabled()){if(FMath::Abs(Settings.TyreMu-Campus->ScenarioTyre)>.0001)ChangeFriction(true,Campus->ScenarioTyre-Settings.TyreMu);if(FMath::Abs(Settings.RoadMu-Campus->ScenarioRoad)>.0001)ChangeFriction(false,Campus->ScenarioRoad-Settings.RoadMu);}
    if(Campus)Campus->ConfigureVehicle(*this);
    UE_LOG(LogTemp,Display,TEXT("CAMPUS_VEHICLE id=%s origin=%s yaw=%.1f own_wheels=%d external_load=%s"),*VehicleId,*CampusSpawn.GetLocation().ToString(),CampusSpawn.Rotator().Yaw,Wheels.Num(),*CampusLoadId);
}
