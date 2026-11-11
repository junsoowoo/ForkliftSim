#include "Vehicle3DSettings.h"
#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

bool FVehicle3DSettings::Load(FString& Error)
{
    FString Text; TSharedPtr<FJsonObject> J;
    if (!FFileHelper::LoadFileToString(Text, *(FPaths::ProjectConfigDir()/TEXT("Vehicle3D.json")))
        || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), J) || !J.IsValid())
    { Error = TEXT("Cannot read Config/Vehicle3D.json"); return false; }
    J->TryGetNumberField(TEXT("swerve_yaw_rate_deg_s"),YawRateDegS);
    J->TryGetNumberField(TEXT("swerve_acceleration_mps2"),AccelerationLimitMps2);
    J->TryGetNumberField(TEXT("swerve_speed_gain_nm_per_mps"),WheelSpeedGain);
    // Missing optional fields preserve compatibility with older config files;
    // present but non-numeric fields are errors, not silently ignored tuning.
    auto OptionalNumber=[&](const TCHAR* Key,double& Value)
    { if(!J->HasField(Key) || (J->TryGetNumberField(Key,Value) && FMath::IsFinite(Value))) return true; Error=FString::Printf(TEXT("Invalid setting %s"),Key); return false; };
    if(!OptionalNumber(TEXT("swerve_wheel_surface_speed_limit_mps"),WheelSurfaceSpeedLimitMps)
        || !OptionalNumber(TEXT("rolling_resistance_coefficient"),RollingResistance)
        || !OptionalNumber(TEXT("bearing_coulomb_nm"),BearingCoulombNm)
        || !OptionalNumber(TEXT("bearing_viscous_nm_per_rad_s"),BearingViscousNms)) return false;
    auto Read = [&](const TCHAR* Key, double& Value)
    { if (J->TryGetNumberField(Key, Value) && FMath::IsFinite(Value)) return true; Error = FString::Printf(TEXT("Invalid setting %s"), Key); return false; };
    if (!(Read(TEXT("chassis_mass_kg"),ChassisMassKg) && Read(TEXT("wheel_mass_kg"),WheelMassKg)
        && Read(TEXT("knuckle_mass_kg"),KnuckleMassKg) && Read(TEXT("fork_mass_kg"),ForkMassKg)
        && Read(TEXT("wheel_radius_m"),RadiusM) && Read(TEXT("wheel_width_m"),WidthM)
        && Read(TEXT("wheelbase_m"),WheelbaseM) && Read(TEXT("track_m"),TrackM)
        && Read(TEXT("tyre_friction"),TyreMu) && Read(TEXT("road_friction"),RoadMu)
        && Read(TEXT("drive_torque_nm"),DriveNm) && Read(TEXT("brake_torque_nm"),BrakeNm)
        && Read(TEXT("max_speed_mps"),MaxSpeedMps) && Read(TEXT("max_steer_deg"),MaxSteerDeg)
        && Read(TEXT("steer_rate_deg_s"),SteerRateDegS) && Read(TEXT("chassis_com_offset_m"),ChassisComOffsetM)
        && Read(TEXT("steering_torque_limit_nm"),SteeringTorqueNm)
        && Read(TEXT("fork_travel_m"),ForkTravelM) && Read(TEXT("fork_speed_mps"),ForkSpeedMps)
        && Read(TEXT("fork_force_limit_n"),ForkForceN))) return false;
    if (!FMath::IsFinite(YawRateDegS) || YawRateDegS<=0 || YawRateDegS>120
        || !FMath::IsFinite(WheelSurfaceSpeedLimitMps) || WheelSurfaceSpeedLimitMps<.5 || WheelSurfaceSpeedLimitMps>12
        || !FMath::IsFinite(RollingResistance) || RollingResistance<0 || RollingResistance>.1
        || !FMath::IsFinite(BearingCoulombNm) || BearingCoulombNm<0 || BearingCoulombNm>20
        || !FMath::IsFinite(BearingViscousNms) || BearingViscousNms<0 || BearingViscousNms>5
        || !FMath::IsFinite(AccelerationLimitMps2) || AccelerationLimitMps2<=0 || AccelerationLimitMps2>5
        || !FMath::IsFinite(WheelSpeedGain) || WheelSpeedGain<=0 || WheelSpeedGain>2000
        || ChassisMassKg < 250 || WheelMassKg < 5 || KnuckleMassKg < 1 || ForkMassKg < 10
        || RadiusM < .15 || RadiusM > .5 || WidthM < .1 || WidthM > .3
        || WheelbaseM < 1.4 || WheelbaseM > 2 || TrackM < 1.25 || TrackM > 1.8
        || TrackM-WidthM < 1.0
        || TrackM*.5-SweepRadiusM()<.28 || WheelbaseM<2*SweepRadiusM()+.1
        || TyreMu < 0 || TyreMu > 2 || RoadMu < 0 || RoadMu > 2
        || DriveNm <= 0 || BrakeNm <= 0 || MaxSpeedMps <= 0 || MaxSpeedMps > 8
        || MaxSteerDeg <= 0 || MaxSteerDeg > 35 || SteerRateDegS <= 0 || SteerRateDegS > 120
        || SteeringTorqueNm <= 0 || SteeringTorqueNm > 10000 || ForkForceN <= 0 || ForkForceN > 30000
        || ChassisComOffsetM < 0 || ChassisComOffsetM > 1.2 || ForkTravelM <= 0 || ForkTravelM > 2 || ForkSpeedMps <= 0 || ForkSpeedMps > .5)
    { Error = TEXT("3D settings outside prototype geometry/actuator bounds"); return false; }
    if(!Actuators.Load(Error)) return false;
    if(!Tyres.Load(Error)) return false;
    if(Tyres.Tread.bEnabled && (Tyres.Tread.DepthM>=RadiusM*.8 || Tyres.Tread.MassKg>=WheelMassKg-1))
    { Error=TEXT("Tread depth/mass must fit inside the wheel radius/mass budget"); return false; }
    if(!Hydraulics.Load(Error)) return false;
    if(!Structure.Load(Error)) return false;
    if(ForkMassKg <= (Structure.bEnabled?2*Structure.BladeMassKg():ForkMassKg*.4)+(Hydraulics.bChainMast?Hydraulics.MastMassKg:0)+1)
    { Error=TEXT("Fork assembly mass must include both blades, carriage and movable mast"); return false; }
    Error.Empty(); return true;
}
