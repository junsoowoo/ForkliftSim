#include "SensorDrive.h"
#include "NavigationLandmarks.h"
#include "StabilityLab.h"
#include "JointSensors.h"
#include "DynamicsSensors.h"
#include "LidarSafety.h"
#include "WheelActor.h"
#include "ChassisActor.h"
#include "ContactLoadSampler.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/Archive.h"
#include "HAL/FileManager.h"
namespace
{
    bool ReadMap(SensorDrive::FModel& M,FString& Error)
    {
        const bool Fixture=FParse::Param(FCommandLine::Get(),TEXT("SensorDriveFixture")),MissionFixture=FParse::Param(FCommandLine::Get(),TEXT("MissionFixture"));FString Text;TSharedPtr<FJsonObject> J;
        if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/(MissionFixture?TEXT("NavigationMissionTestMap.json"):Fixture?TEXT("NavigationTestMap.json"):TEXT("NavigationMap.json"))))||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J){Error=TEXT("Cannot read surveyed navigation map");return false;}
        const TArray<TSharedPtr<FJsonValue>>* Segments=nullptr;const TArray<TSharedPtr<FJsonValue>>* Initial=nullptr;
        if(!J->TryGetStringField(TEXT("id"),M.MapId)||!J->TryGetArrayField(TEXT("initial_pose_m_rad"),Initial)||Initial->Num()!=3||!J->TryGetArrayField(TEXT("segments"),Segments)||Segments->Num()<3||Segments->Num()>5000){Error=TEXT("Invalid navigation map");return false;}
        double P[3];for(int I=0;I<3;++I)if(!(*Initial)[I]->TryGetNumber(P[I])||!FMath::IsFinite(P[I])||FMath::Abs(P[I])>1000){Error=TEXT("Invalid known initial bay pose");return false;}M.MapToOdom={P[0],P[1],P[2]};
        for(const auto& Value:*Segments)
        {
            const auto O=Value->AsObject();const TArray<TSharedPtr<FJsonValue>>* A=nullptr;const TArray<TSharedPtr<FJsonValue>>* B=nullptr;
            if(!O||!O->TryGetArrayField(TEXT("a"),A)||!O->TryGetArrayField(TEXT("b"),B)||A->Num()!=2||B->Num()!=2){Error=TEXT("Invalid map segment");return false;}
            double V[4];for(int I=0;I<4;++I)if(!(I<2?(*A)[I]:(*B)[I-2])->TryGetNumber(V[I])||!FMath::IsFinite(V[I])||FMath::Abs(V[I])>1000){Error=TEXT("Invalid map coordinate");return false;}
            if(FVector2D(V[2]-V[0],V[3]-V[1]).Size()<.001){Error=TEXT("Degenerate map segment");return false;}M.Map.Add({FVector2D(V[0],V[1]),FVector2D(V[2],V[3])});
        }
        if(!Fixture&&!MissionFixture){TArray<NavigationLandmarks::FBox> Survey;if(!NavigationLandmarks::Load(Survey,Error))return false;NavigationLandmarks::AppendSurvey(Survey,M.Map);M.MapId+=TEXT("+pillars-v1");}
        return true;
    }
    void Write(FArchive& A,const FString& S){FTCHARToUTF8 U(*S);A.Serialize((void*)U.Get(),U.Length());}
    void Number(TSharedRef<FJsonObject> J,const TCHAR* K,double V,bool Valid=true){if(Valid&&FMath::IsFinite(V))J->SetNumberField(K,V);else J->SetField(K,MakeShared<FJsonValueNull>());}
}
FSensorDriveRig::~FSensorDriveRig(){if(Csv)Csv->Close();}
TSharedPtr<FSensorDriveRig> FSensorDriveRig::Create(FString& Error)
{
    auto R=MakeShared<FSensorDriveRig>();if(!R->Model.Spec.Load(Error)||!ReadMap(R->Model,Error))return nullptr;
    R->CsvPath=FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir()/TEXT("Telemetry")/FString::Printf(TEXT("SensorDrive-%s-%s.csv"),*FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")),*FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(8)));IFileManager::Get().MakeDirectory(*FPaths::GetPath(R->CsvPath),true);R->Csv.Reset(IFileManager::Get().CreateFileWriter(*R->CsvPath));if(!R->Csv){Error=TEXT("Cannot create sensor drive CSV");return nullptr;}
    Write(*R->Csv,TEXT("sim_s,state,healthy,calibrated,fault,reason,vx_mps,vy_mps,gyro_rad_s,gyro_bias_rad_s,odom_x_m,odom_y_m,odom_yaw_rad,map_x_m,map_y_m,map_yaw_rad,localized,match_accepted,match_points,match_rms_m,unobserved_s,fl_nm,fr_nm,rl_nm,rr_nm,truth_x_m,truth_y_m,truth_yaw_rad,truth_vx_mps,truth_vy_mps,truth_yaw_rad_s\n"));UE_LOG(LogTemp,Display,TEXT("SENSOR_DRIVE_CSV %s"),*R->CsvPath);return R;
}
void FSensorDriveRig::Observe(AStabilityLab& Lab,double Dt)
{
    SensorDrive::FInput In;In.Now=Lab.TimeS;In.Dt=Dt;In.Valid=Lab.JointSensors&&Lab.DynamicsSensors;In.Ack=AckRequested;AckRequested=false;In.Expired=ActuatorExpired;ActuatorExpired=false;
    In.Neutral=FMath::Abs(Lab.Throttle)<.01&&FMath::Abs(Lab.Strafe)<.01&&FMath::Abs(Lab.Steering)<.01;
    if(Lab.JointSensors)for(int I=0;I<4;++I){const auto& C=Lab.JointSensors->Model.Channels[I];In.Valid&=Lab.JointSensors->Model.Usable(I,Lab.TimeS)&&C.Last.RateValid&&!C.Last.Clipped;In.Rate[I]=C.Last.Rate;In.Capture[I]=C.Last.CaptureS;}
    if(Lab.DynamicsSensors){const auto& C=Lab.DynamicsSensors->Model.Channels[0];In.Valid&=Lab.DynamicsSensors->Model.Usable(0,Lab.TimeS)&&!(C.Last.Clipped&(1<<5));In.Gyro=C.Last.Value[5];In.Capture[4]=C.Last.CaptureS;In.ImuSequence=C.Last.Sequence;}
    const bool WasFault=Model.Faulted;Model.Observe(In);SawFault|=Model.Faulted;SawRecovery|=WasFault&&!Model.Faulted;
}
void FSensorDriveRig::Correct(AStabilityLab& Lab)
{
    if(!Lab.VehicleGuard||!Model.Ready)return;const auto& S=Lab.VehicleGuard->Sensor;
    if(!S.Fresh(0,Lab.TimeS)||!S.Fresh(1,Lab.TimeS)||S.Channels[0].Last.Sequence==Model.LastScanSequence[0]||S.Channels[1].Last.Sequence==Model.LastScanSequence[1])return;
    const double Stamp=S.Channels[0].Last.CaptureS;if(FMath::Abs(Stamp-S.Channels[1].Last.CaptureS)>.03)return;
    TArray<FVector2D> Points;
    for(int I=0;I<2;++I){Model.LastScanSequence[I]=S.Channels[I].Last.Sequence;const auto& C=S.Channels[I].Last;const auto Mount=S.Spec.Mount(I);for(int K=0;K<S.Spec.Rays;K+=3)if(C.States[K]==LidarSafety::EReturn::Valid){const double A=S.Spec.Angle(I,K),D=C.Ranges[K];Points.Add(FVector2D(Mount.X+D*FMath::Cos(A),Mount.Y+D*FMath::Sin(A)));}}
    Model.Correct(Points,Stamp,Lab.TimeS);
}
void FSensorDriveRig::Drive(AStabilityLab& Lab,double Dt,FContactLoadSampler* Sampler)
{
    auto& S=Model.Spec;S.Radius=Lab.Settings.RadiusM;S.Wheelbase=Lab.Settings.WheelbaseM;S.Track=Lab.Settings.TrackM;S.Acceleration=Lab.Settings.AccelerationLimitMps2;
    const bool GuardHold=Lab.bVehicleSafetyRequested&&(!Lab.VehicleGuard||Lab.VehicleGuard->Decision.DriveHold);
    const bool Hold=GuardHold||!Model.Ready||Lab.Brake>0||Lab.bNeutral;
    const bool Creep=Lab.VehicleGuard&&Lab.VehicleGuard->Decision.State==TEXT("manual_creep");
    Model.Control(Dt,FVector2D(Lab.Throttle,Lab.Strafe),Lab.Steering,Hold,Creep?.15:1.,Creep?.15:.6);
    Lab.TargetVelocity=Model.Target;Lab.TargetYawRadS=Lab.CoordinatedYawRadS=Model.YawTarget;Lab.WheelSpeedScale=1;
    const bool Withhold=Lab.TestMode==TEXT("mecanum_drive_sensor_watchdog")&&Lab.TimeS>=6&&Lab.TimeS<8;
    if(!Withhold){CommandUntilS=Lab.TimeS+S.LeaseS;CommandPowered=!Hold;for(int I=0;I<4;++I)CommandTorque[I]=Model.Torque[I];}
    if(Sampler)
    {
        auto* In=Sampler->GetProducerInputData_External();In->bMecanumControl=true;In->bMecanumSensorTorque=true;In->MecanumLeaseUntilS=CommandUntilS;
        In->bMecanumPowered=CommandPowered;In->bMecanumDetailedMotor=Lab.Settings.Actuators.bEnabled;In->MecanumMotor=Lab.Settings.Actuators.Drive;
        In->bMecanumResistance=Lab.bResistanceEnabled;In->MecanumRadiusM=Lab.Settings.RadiusM;In->MecanumCrr=Lab.Settings.RollingResistance;In->MecanumTorqueLimitNm=FMath::Min(Lab.Settings.DriveNm,S.MaxTorqueNm);
        for(int I=0;I<4;++I){In->MecanumSensorTorqueNm[I]=CommandTorque[I];In->MecanumTargetMps[I]=Model.Targets[I];In->ReducedServiceBrake[I]=!CommandPowered;In->MecanumLoadN[I]=Lab.Wheels[I]->NormalLoadFilteredN;}
    }
    for(int I=0;I<4;++I){Lab.Wheels[I]->SpeedTargetMps=Model.Targets[I];Lab.Wheels[I]->SetBrakeLimit(Hold?Lab.Settings.BrakeNm:0);}
    Record(Lab);
}
void FSensorDriveRig::Record(AStabilityLab& Lab)
{
    // Ground truth is appended only for validation, after control and localization.
    const auto O=Model.Predicted(Lab.TimeS),P=Model.MapPose(Lab.TimeS);const FVector Truth=Lab.Chassis->Body->GetComponentLocation()/100.;FinalErrorM=FVector2D(P.X-Truth.X,P.Y-Truth.Y).Size();PeakErrorM=FMath::Max(PeakErrorM,FinalErrorM);
    if(Lab.TimeS+1e-9<NextCsv||!Csv)return;NextCsv=Lab.TimeS+.05;
    auto N=[](double V){return FString::Printf(TEXT("%.9f"),V);};auto B=[](bool V){return V?TEXT("1"):TEXT("0");};
    const bool Located=Model.Localized&&Lab.TimeS-Model.LastMatchS<.75;
    TArray<FString> R={N(Lab.TimeS),Model.State,B(Model.DataHealthy),B(Model.Calibrated),B(Model.Faulted),Model.Reason,N(Model.Velocity.X),N(Model.Velocity.Y),N(Model.Velocity.Z),N(Model.GyroBias),N(O.X),N(O.Y),N(O.Yaw),N(P.X),N(P.Y),N(P.Yaw),B(Located),B(Model.Registration.Accepted),FString::FromInt(Model.Registration.Points),N(Model.Registration.Rms),N(Model.UnobservedS)};
    for(double V:Model.Torque)R.Add(N(V));for(double V:{Truth.X,Truth.Y,FMath::DegreesToRadians(Lab.Chassis->Body->GetComponentRotation().Yaw),Lab.SpeedMps,Lab.LateralMps,FMath::DegreesToRadians(Lab.YawDegS)})R.Add(N(V));
    Write(*Csv,FString::Join(R,TEXT(","))+TEXT("\n"));if(Lab.TimeS>=NextFlush){Csv->Flush();NextFlush=Lab.TimeS+5;}
}
TSharedRef<FJsonObject> FSensorDriveRig::Telemetry(double Now) const
{
    const auto& M=Model;auto J=MakeShared<FJsonObject>();const bool Valid=M.Ready,Located=Valid&&M.Localized&&Now-M.LastMatchS<.75;const auto O=M.Predicted(Now),P=M.MapPose(Now);
    J->SetStringField(TEXT("source"),TEXT("encoder_imu_surveyed_map_v1"));J->SetStringField(TEXT("state"),M.State);J->SetStringField(TEXT("reason"),M.Reason);J->SetStringField(TEXT("map_id"),M.MapId);J->SetBoolField(TEXT("fault_latched"),M.Faulted);J->SetBoolField(TEXT("data_healthy"),M.DataHealthy);J->SetBoolField(TEXT("calibrated"),M.Calibrated);J->SetBoolField(TEXT("localized"),Located);J->SetBoolField(TEXT("ground_truth_feedback"),false);
    Number(J,TEXT("vx_mps"),M.Velocity.X,Valid);Number(J,TEXT("vy_mps"),M.Velocity.Y,Valid);Number(J,TEXT("yaw_rad_s"),M.Velocity.Z,Valid);Number(J,TEXT("gyro_bias_rad_s"),M.GyroBias,M.Calibrated);
    Number(J,TEXT("odom_x_m"),O.X,Valid);Number(J,TEXT("odom_y_m"),O.Y,Valid);Number(J,TEXT("odom_yaw_rad"),O.Yaw,Valid);Number(J,TEXT("map_x_m"),P.X,Located);Number(J,TEXT("map_y_m"),P.Y,Located);Number(J,TEXT("map_yaw_rad"),P.Yaw,Located);
    Number(J,TEXT("match_age_s"),Now-M.LastMatchS,M.LastMatchS>=0);Number(J,TEXT("match_rms_m"),M.Registration.Rms,M.Registration.Accepted);Number(J,TEXT("match_points"),M.Registration.Points);Number(J,TEXT("unobserved_motion_s"),M.UnobservedS);Number(J,TEXT("command_lease_s"),M.Spec.LeaseS);
    TArray<TSharedPtr<FJsonValue>> Torque;for(double T:M.Torque)Torque.Add(MakeShared<FJsonValueNumber>(T));J->SetArrayField(TEXT("torque_commands_nm"),Torque);return J;
}
