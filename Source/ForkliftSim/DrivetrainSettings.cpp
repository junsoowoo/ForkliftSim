#include "DrivetrainModel.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

bool FDrivetrainSettings::Load(FString& Error)
{
    FString Text; TSharedPtr<FJsonObject> J;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("Drivetrain.json"))) ||
        !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J) || !J.IsValid())
    { Error=TEXT("Cannot read Config/Drivetrain.json"); return false; }
    if(!J->TryGetBoolField(TEXT("enabled"),bEnabled)) { Error=TEXT("Invalid drivetrain.enabled"); return false; }
    auto Number=[&](const FString& Key,double& V)
    { if(J->TryGetNumberField(Key,V) && FMath::IsFinite(V)) return true; Error=TEXT("Invalid drivetrain.")+Key; return false; };
    for(int I=0;I<2;++I)
    {
        auto& S=I==0?Drive:Steer; const FString P=I==0?TEXT("drive_"):TEXT("steer_");
        if(!Number(P+TEXT("motor_peak_nm"),S.PeakNm) || !Number(P+TEXT("motor_power_w"),S.PowerW)
            || !Number(P+TEXT("motor_max_rpm"),S.MaxRpm) || !Number(P+TEXT("gear_ratio"),S.Ratio)
            || !Number(P+TEXT("gear_efficiency"),S.GearEfficiency) || !Number(P+TEXT("motor_efficiency"),S.MotorEfficiency)
            || !Number(P+TEXT("response_s"),S.ResponseS) || !Number(P+TEXT("rotor_inertia_kg_m2"),S.RotorInertiaKgM2)) return false;
    }
    if(!Number(TEXT("steer_backlash_deg"),BacklashDeg)) return false;
    if(!Valid()) { Error=TEXT("Drivetrain values outside supported bounds"); return false; }
    return true;
}
