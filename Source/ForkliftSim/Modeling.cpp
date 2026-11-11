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
    void HideOriginal(AActor* Actor)
    {
        TArray<UStaticMeshComponent*> Parts;Actor->GetComponents(Parts);
        for(auto* Part:Parts) Part->SetVisibility(false,false);
    }
    bool Visual(AActor* Owner,USceneComponent* Parent,const TCHAR* Path)
    {
        auto* Mesh=LoadObject<UStaticMesh>(nullptr,Path);if(!Mesh) return false;
        // A physics box uses component scaling for its collision dimensions.
        // The visual frame inherits pose but keeps its own unit scale.
        auto* Frame=NewObject<USceneComponent>(Owner);Owner->AddInstanceComponent(Frame);
        Frame->SetupAttachment(Parent);Frame->SetAbsolute(false,false,true);Frame->RegisterComponent();Frame->SetWorldScale3D(FVector::OneVector);
        auto* Model=NewObject<UStaticMeshComponent>(Owner);Owner->AddInstanceComponent(Model);
        Model->SetMobility(EComponentMobility::Movable);Model->SetupAttachment(Frame);Model->SetStaticMesh(Mesh);Model->BodyInstance.bAutoWeld=false;Model->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Model->SetGenerateOverlapEvents(false);Model->SetCanEverAffectNavigation(false);Model->RegisterComponent();
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
    S.bReferenceGeometry=true;
    UE_LOG(LogTemp,Display,TEXT("REFERENCE_GEOMETRY headlength_m=%.4f width_m=%.4f wheelbase_m=%.4f custom_track_m=%.4f mast_m=%.4f"),S.BodyLengthM,S.BodyWidthM,S.WheelbaseM,S.TrackM,S.LoweredMastHeightM);
    return true;
}
void ForkliftModel::Present(AStabilityLab* Lab)
{
    if(!Lab->Settings.bReferenceGeometry || FParse::Param(FCommandLine::Get(),TEXT("ModelingPhysicsOnly"))) return;
    for(const TCHAR* Name:{TEXT("SM_AGVBody"),TEXT("SM_FixedMast"),TEXT("SM_MovingMast"),TEXT("SM_Carriage"),TEXT("SM_ForkSegment")})
        if(!LoadObject<UStaticMesh>(nullptr,*FString::Printf(TEXT("/Game/ForkliftModel/%s.%s"),Name,Name)))
        {UE_LOG(LogTemp,Warning,TEXT("MODEL_ASSETS missing; retaining visible physics proxies"));return;}
    HideOriginal(Lab->Chassis);
    Visual(Lab->Chassis,Lab->Chassis->Body,TEXT("/Game/ForkliftModel/SM_AGVBody.SM_AGVBody"));
    Visual(Lab->Chassis,Lab->Chassis->Body,TEXT("/Game/ForkliftModel/SM_FixedMast.SM_FixedMast"));
    HideOriginal(Lab->Fork);
    Visual(Lab->Fork,Lab->Fork->Carriage,TEXT("/Game/ForkliftModel/SM_Carriage.SM_Carriage"));
    if(Lab->Fork->MovingMast) Visual(Lab->Fork,Lab->Fork->MovingMast,TEXT("/Game/ForkliftModel/SM_MovingMast.SM_MovingMast"));
    for(AForkTineActor* Tine:Lab->Fork->Tines)
    {
        HideOriginal(Tine);
        for(UStaticMeshComponent* Section:Tine->Sections) Visual(Tine,Section,TEXT("/Game/ForkliftModel/SM_ForkSegment.SM_ForkSegment"));
    }
    UE_LOG(LogTemp,Display,TEXT("MODEL_PRESENTATION articulated=1 sensors=0 visual_only_components=1"));
}
