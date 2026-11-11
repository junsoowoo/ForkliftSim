#include "ComponentDetail.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

bool FForkStructureSettings::Valid() const
{
    for(double X:{LengthM,WidthM,ThicknessM,YoungPa,YieldPa,BreakPa,DampingRatio,FailurePlasticDeg,HardeningRatio}) if(!FMath::IsFinite(X)) return false;
    return LengthM>=.8 && LengthM<=1.6 && WidthM>=.08 && WidthM<=.2 && ThicknessM>=.03 && ThicknessM<=.1
        && YoungPa>=50e9 && YoungPa<=300e9 && YieldPa>=100e6 && BreakPa>YieldPa && BreakPa<=2000e6
        && DampingRatio>=0 && DampingRatio<=1 && Segments>=2 && Segments<=8 && FailurePlasticDeg>0 && FailurePlasticDeg<=45 && HardeningRatio>=0 && HardeningRatio<=.5;
}
bool FForkStructureSettings::Load(FString& Error)
{
    FString Text; TSharedPtr<FJsonObject> J;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("ForkStructure.json"))) ||
        !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J) || !J.IsValid())
    { Error=TEXT("Cannot read Config/ForkStructure.json"); return false; }
    double Count=Segments;
    auto Read=[&](const TCHAR* K,double& V) { return J->TryGetNumberField(K,V); };
    if(!J->TryGetBoolField(TEXT("enabled"),bEnabled) || !Read(TEXT("length_m"),LengthM) || !Read(TEXT("width_m"),WidthM)
        || !Read(TEXT("thickness_m"),ThicknessM) || !Read(TEXT("young_modulus_pa"),YoungPa) || !Read(TEXT("yield_strength_pa"),YieldPa)
        || !Read(TEXT("failure_stress_pa"),BreakPa) || !Read(TEXT("damping_ratio"),DampingRatio) || !Read(TEXT("segments"),Count)
        || !FMath::IsFinite(Count) || Count!=FMath::FloorToDouble(Count))
    { Error=TEXT("Invalid fork structure configuration"); return false; }
    Segments=int32(Count); if(!Valid()) { Error=TEXT("Fork structure outside supported bounds"); return false; }
    if(J->HasField(TEXT("failure_plastic_rotation_deg")) && (!Read(TEXT("failure_plastic_rotation_deg"),FailurePlasticDeg) || !Valid()))
    { Error=TEXT("Invalid plastic failure rotation"); return false; }
    if(J->HasField(TEXT("hardening_ratio")) && (!Read(TEXT("hardening_ratio"),HardeningRatio) || !Valid()))
    { Error=TEXT("Invalid plastic hardening ratio"); return false; }
    return true;
}
