#include "Modeling.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "WheelActor.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
    void HideOriginal(AActor* Actor,bool PhysicsSensorProxy)
    {
        TArray<UStaticMeshComponent*> Parts;Actor->GetComponents(Parts);
        for(auto* Part:Parts)
        {
            // Decorative panel edges must not redefine this virtual rig's
            // depth calibration. Render its actual collision geometry to the
            // sensors, including moving/deforming forks; hide debug markers.
            const bool Capture=PhysicsSensorProxy && Part->GetCollisionEnabled()!=ECollisionEnabled::NoCollision;
            Part->SetVisibility(Capture,false);
            Part->SetVisibleInSceneCaptureOnly(Capture);
        }
    }
    bool Visual(AActor* Owner,USceneComponent* Parent,const TCHAR* Path,bool PhysicsSensorProxy)
    {
        auto* Mesh=LoadObject<UStaticMesh>(nullptr,Path);if(!Mesh) return false;
        // A physics box uses component scaling for its collision dimensions.
        // The visual frame inherits pose but keeps its own unit scale.
        auto* Frame=NewObject<USceneComponent>(Owner);Owner->AddInstanceComponent(Frame);
        Frame->SetupAttachment(Parent);Frame->SetAbsolute(false,false,true);Frame->RegisterComponent();Frame->SetWorldScale3D(FVector::OneVector);
        auto* Model=NewObject<UStaticMeshComponent>(Owner);Owner->AddInstanceComponent(Model);
        Model->SetMobility(EComponentMobility::Movable);Model->SetupAttachment(Frame);Model->SetStaticMesh(Mesh);Model->BodyInstance.bAutoWeld=false;Model->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Model->SetGenerateOverlapEvents(false);Model->SetCanEverAffectNavigation(false);Model->RegisterComponent();
        Model->SetHiddenInSceneCapture(PhysicsSensorProxy);
        if(FString(Path).Contains(TEXT("SM_AGVBody"))) Model->ComponentTags.Add(TEXT("ReferenceBodyVisual"));
        return true;
    }
}
bool ForkliftModel::Configure(FVehicle3DSettings& S,FString& Error,bool SelectedByMap)
{
    if(!SelectedByMap && !FParse::Param(FCommandLine::Get(),TEXT("WarehouseLab"))) return true;
    if(!S.bMecanum) {Error=TEXT("Warehouse model requires the four-mecanum rig");return false;}
    FString Text;TSharedPtr<FJsonObject> J;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("Modeling.json"))) ||
       !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J) || !J)
    {Error=TEXT("Cannot read Modeling.json");return false;}
    auto Read=[&](const TCHAR* Key,double& V){return J->TryGetNumberField(Key,V) && FMath::IsFinite(V);};
    if(!Read(TEXT("headlength_m"),S.BodyLengthM) || !Read(TEXT("overall_width_m"),S.BodyWidthM) ||
       !Read(TEXT("lowered_mast_height_m"),S.LoweredMastHeightM) || !Read(TEXT("carriage_width_m"),S.CarriageWidthM) ||
       !Read(TEXT("wheelbase_m"),S.WheelbaseM) || !Read(TEXT("custom_mecanum_track_m"),S.TrackM) ||
       S.BodyLengthM<1.2 || S.BodyLengthM>2 || S.BodyWidthM<.85 || S.BodyWidthM>1.4 ||
       S.LoweredMastHeightM<2 || S.LoweredMastHeightM>3.5 || S.WheelbaseM<.8 || S.WheelbaseM>1.3 ||
       S.TrackM<S.Mecanum.WidthM+.4 || S.TrackM+S.Mecanum.WidthM>S.BodyWidthM ||
       S.ForkFaceXM()-.06-(S.WheelbaseM*.5+S.RadiusM)<.01 ||
       S.CarriageWidthM<.7 || S.CarriageWidthM>S.BodyWidthM || S.WheelbaseM+2*S.RadiusM>S.BodyLengthM)
    {Error=TEXT("Invalid compact model envelope/wheel clearance");return false;}
    if(!Read(TEXT("fork_travel_m"),S.ForkTravelM) || S.ForkTravelM<1.5 || S.ForkTravelM>2.2)
    {Error=TEXT("Invalid compact mast stroke");return false;}
    for(auto Pair:{TPair<const TCHAR*,FString*>(TEXT("body_mesh"),&S.BodyMeshPath),
        {TEXT("fixed_mast_mesh"),&S.FixedMastMeshPath},{TEXT("moving_mast_mesh"),&S.MovingMastMeshPath},
        {TEXT("carriage_mesh"),&S.CarriageMeshPath},{TEXT("fork_segment_mesh"),&S.ForkSegmentMeshPath}})
    {
        if(J->HasField(Pair.Key) && (!J->TryGetStringField(Pair.Key,*Pair.Value) ||
            !Pair.Value->StartsWith(TEXT("/Game/")) || Pair.Value->Contains(TEXT("..")) || Pair.Value->Len()>512))
        {Error=TEXT("Invalid project model asset path");return false;}
    }
    S.bReferenceGeometry=true;
    UE_LOG(LogTemp,Display,TEXT("REFERENCE_GEOMETRY headlength_m=%.4f width_m=%.4f wheelbase_m=%.4f custom_track_m=%.4f mast_m=%.4f"),S.BodyLengthM,S.BodyWidthM,S.WheelbaseM,S.TrackM,S.LoweredMastHeightM);
    return true;
}
void ForkliftModel::Present(AStabilityLab* Lab)
{
    if(!Lab->Settings.bReferenceGeometry || FParse::Param(FCommandLine::Get(),TEXT("ModelingPhysicsOnly"))) return;
    const auto& S=Lab->Settings;
    const bool PhysicsSensorProxy=S.BodyMeshPath.StartsWith(TEXT("/Game/AllturnForkman/"));
    for(const FString& Path:{S.BodyMeshPath,S.FixedMastMeshPath,S.MovingMastMeshPath,S.CarriageMeshPath,S.ForkSegmentMeshPath})
        if(!LoadObject<UStaticMesh>(nullptr,*Path))
        {UE_LOG(LogTemp,Warning,TEXT("MODEL_ASSETS missing; retaining visible physics proxies"));return;}
    HideOriginal(Lab->Chassis,PhysicsSensorProxy);
    Visual(Lab->Chassis,Lab->Chassis->Body,*S.BodyMeshPath,PhysicsSensorProxy);
    Visual(Lab->Chassis,Lab->Chassis->Body,*S.FixedMastMeshPath,PhysicsSensorProxy);
    HideOriginal(Lab->Fork,PhysicsSensorProxy);
    Visual(Lab->Fork,Lab->Fork->Carriage,*S.CarriageMeshPath,PhysicsSensorProxy);
    if(Lab->Fork->MovingMast) Visual(Lab->Fork,Lab->Fork->MovingMast,*S.MovingMastMeshPath,PhysicsSensorProxy);
    for(AForkTineActor* Tine:Lab->Fork->Tines)
    {
        HideOriginal(Tine,PhysicsSensorProxy);
        for(UStaticMeshComponent* Section:Tine->Sections) Visual(Tine,Section,*S.ForkSegmentMeshPath,PhysicsSensorProxy);
    }
    UE_LOG(LogTemp,Display,TEXT("MODEL_PRESENTATION articulated=1 visual_only_components=1 body=%s sensor_self_geometry=%s"),*S.BodyMeshPath,PhysicsSensorProxy?TEXT("physics_proxy"):TEXT("visual_skin"));
}
