#include "TyreCompliance.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

bool FTyreComplianceSettings::Load(FString& Error)
{
    FString Text; TSharedPtr<FJsonObject> J;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("Tyres.json"))) ||
        !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J) || !J.IsValid())
    { Error=TEXT("Cannot read Config/Tyres.json"); return false; }
    if(!J->TryGetBoolField(TEXT("enabled"),bEnabled)
        || !J->TryGetNumberField(TEXT("vertical_stiffness_n_per_m"),StiffnessNpm)
        || !J->TryGetNumberField(TEXT("vertical_damping_ns_per_m"),DampingNsPm)
        || !J->TryGetNumberField(TEXT("max_relative_travel_m"),TravelM))
    { Error=TEXT("Invalid tyre compliance settings"); return false; }
    if(J->HasField(TEXT("detailed_enabled")) && (!J->TryGetBoolField(TEXT("detailed_enabled"),bDetailed)
        || !J->TryGetNumberField(TEXT("cubic_stiffness_n_per_m3"),CubicNpM3)
        || !J->TryGetNumberField(TEXT("shear_stiffness_n_per_m"),ShearNpm)
        || !J->TryGetNumberField(TEXT("shear_damping_ns_per_m"),ShearDampingNsPm)
        || !J->TryGetNumberField(TEXT("shear_travel_m"),ShearTravelM)))
    { Error=TEXT("Invalid detailed tyre settings"); return false; }
    if(J->HasField(TEXT("tread_enabled")))
    {
        double Count=0;
        if(!J->TryGetBoolField(TEXT("tread_enabled"),Tread.bEnabled)
            || !J->TryGetNumberField(TEXT("tread_segments"),Count) || !FMath::IsFinite(Count) || Count<8 || Count>32 || Count!=FMath::Floor(Count)
            || !J->TryGetNumberField(TEXT("tread_depth_m"),Tread.DepthM)
            || !J->TryGetNumberField(TEXT("tread_travel_m"),Tread.TravelM)
            || !J->TryGetNumberField(TEXT("tread_mass_kg"),Tread.MassKg)
            || !J->TryGetNumberField(TEXT("tread_foundation_n_per_m3"),Tread.FoundationNpm3)
            || !J->TryGetNumberField(TEXT("tread_foundation_damping_ns_per_m3"),Tread.FoundationDampingNspm3))
        { Error=TEXT("Invalid discrete tread settings"); return false; }
        Tread.Count=int32(Count);
        if(J->HasField(TEXT("tread_poisson_ratio")) && !J->TryGetNumberField(TEXT("tread_poisson_ratio"),Tread.Poisson)) {Error=TEXT("Invalid tread Poisson ratio");return false;}
    }
    if(!Valid()) { Error=TEXT("Tyre settings outside supported bounds"); return false; }
    return true;
}
