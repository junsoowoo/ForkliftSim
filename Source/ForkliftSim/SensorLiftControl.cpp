#include "SensorLiftControl.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/FileManager.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/Archive.h"

bool SensorLift::FSpec::Valid(double TravelM) const
{
    const double V[]={Kp,Ki,Kd,IntegralCap,UpCap,DownCap,SlewPerS,DeadbandM,ReopenM,VelocityTauS,MaxMeasuredSpeed,MaxAgeS,HealthyS,LeaseS,UpperMarginM,PressureTripPa,PressureReleasePa,PressureDebounceS,SettleDwellS,MaxLowerMps,LowerApproachPerS,ReturnGainMpsSqrtPa,TravelM};for(double X:V)if(!FMath::IsFinite(X))return false;
    if(MaxLowerMps<.02||MaxLowerMps>.15||LowerApproachPerS<.2||LowerApproachPerS>2||ReturnGainMpsSqrtPa<=1e-6||ReturnGainMpsSqrtPa>.01)return false;
    return Kp>0&&Kp<=20&&Ki>=0&&Ki<=5&&Kd>=0&&Kd<=3&&IntegralCap>=0&&IntegralCap<=.5&&UpCap>0&&UpCap<=1&&DownCap>0&&DownCap<=.5&&SlewPerS>=.5&&SlewPerS<=10&&DeadbandM>=.002&&ReopenM>DeadbandM&&ReopenM<=.03&&VelocityTauS>=.05&&VelocityTauS<=.5&&MaxMeasuredSpeed>=.3&&MaxMeasuredSpeed<=2&&MaxAgeS>=.1&&MaxAgeS<=.5&&HealthyS>=.1&&HealthyS<=1&&LeaseS>=.08&&LeaseS<=.2&&UpperMarginM>=.02&&UpperMarginM<TravelM*.2&&PressureTripPa>1e6&&PressureTripPa<16e6&&PressureReleasePa>0&&PressureReleasePa<PressureTripPa&&PressureDebounceS>=.02&&PressureDebounceS<=.5&&SettleDwellS>=.1&&SettleDwellS<=1;
}
bool SensorLift::FSpec::Load(FString& Error)
{
    FString Text;TSharedPtr<FJsonObject> J;if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("SensorLiftControl.json")))||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J){Error=TEXT("Cannot read SensorLiftControl.json");return false;}
    struct FField {const TCHAR* Key;double* Value;};FField Fields[]={
        {TEXT("kp_per_m"),&Kp},{TEXT("ki_per_m_s"),&Ki},{TEXT("kd_s_per_m"),&Kd},{TEXT("integral_spool_cap"),&IntegralCap},{TEXT("max_up_spool"),&UpCap},{TEXT("max_down_spool"),&DownCap},{TEXT("spool_slew_per_s"),&SlewPerS},
        {TEXT("hold_deadband_m"),&DeadbandM},{TEXT("hold_reopen_m"),&ReopenM},{TEXT("velocity_filter_s"),&VelocityTauS},{TEXT("max_measured_speed_mps"),&MaxMeasuredSpeed},{TEXT("max_sample_age_s"),&MaxAgeS},{TEXT("healthy_before_arm_s"),&HealthyS},{TEXT("command_lease_s"),&LeaseS},{TEXT("upper_target_margin_m"),&UpperMarginM},
        {TEXT("pressure_trip_pa"),&PressureTripPa},{TEXT("pressure_release_pa"),&PressureReleasePa},{TEXT("pressure_debounce_s"),&PressureDebounceS},{TEXT("settle_dwell_s"),&SettleDwellS},{TEXT("max_lower_speed_mps"),&MaxLowerMps},{TEXT("lower_approach_per_s"),&LowerApproachPerS}};
    for(const auto& F:Fields)if(!J->TryGetNumberField(F.Key,*F.Value)){Error=TEXT("Missing sensor lift control field");return false;}return true;
}
void SensorLift::FModel::Reset(const FSpec& S,double Travel){*this=FModel();Spec=S;TravelM=Travel;}
void SensorLift::FModel::Stop(const TCHAR* FaultReason){Faulted=true;Reason=FaultReason;Command=Integral=0;PumpEnabled=false;State=TEXT("fault_latched");Holding=true;HealthySinceS=-1;}
void SensorLift::FModel::Step(const FInput& In)
{
    Rebased=AckRejected=false;PumpEnabled=false;
    auto Fresh=[&](bool Valid,double Capture){return Valid&&FMath::IsFinite(Capture)&&In.NowS>=Capture-1e-9&&In.NowS-Capture<=Spec.MaxAgeS;};
    const bool Clock=FMath::IsFinite(In.NowS)&&FMath::IsFinite(In.Dt)&&In.Dt>0&&In.Dt<=.1;
    const bool FreshInputs=Clock&&Fresh(In.HeightValid,In.HeightCaptureS)&&Fresh(In.LimitsValid,In.LimitCaptureS)&&Fresh(In.PressureValid,In.PressureCaptureS);
    const bool Plausible=FMath::IsFinite(In.HeightM)&&In.HeightM>=0&&In.HeightM<=TravelM&&FMath::IsFinite(In.PressurePa)&&In.PressurePa>=0&&In.PressurePa<=16e6&&!In.PressureClipped&&!(In.Lower&&In.Upper)&&FMath::IsFinite(In.RequestedM);
    const bool Healthy=FreshInputs&&Plausible&&!In.ActuatorExpired;
    if(Healthy){if(HealthySinceS<0)HealthySinceS=In.NowS;}else HealthySinceS=-1;
    if(!Healthy)
    {
        Command=Integral=0;VelocitySeeded=false;
        if(Armed||Faulted)Stop(In.ActuatorExpired?TEXT("actuator_command_expired"):!FreshInputs?TEXT("sensor_timeout"):TEXT("invalid_sensor_or_command"));
        else State=TEXT("initializing");
        AckRejected=In.Acknowledge;return;
    }
    const bool Stable=In.NowS-HealthySinceS+1e-9>=Spec.HealthyS;
    if(Faulted)
    {
        Command=Integral=0;State=TEXT("fault_latched");
        if(!In.Acknowledge||!Stable){AckRejected=In.Acknowledge;return;}
        Faulted=false;Reason.Empty();TargetM=FMath::Clamp(In.HeightM,0.,TravelM-Spec.UpperMarginM);Rebased=true;Holding=true;VelocitySeeded=false;
    }
    if(!Armed){if(!Stable){Command=0;State=TEXT("initializing");return;}Armed=true;}
    if(In.HeightSequence!=PreviousHeightSequence)
    {
        const double Dt=In.HeightCaptureS-PreviousHeightCaptureS;
        if(VelocitySeeded)
        {
            if(Dt<=0||FMath::Abs((In.HeightM-PreviousHeightM)/Dt)>Spec.MaxMeasuredSpeed){Stop(TEXT("height_jump_or_clock"));return;}
            const double Raw=(In.HeightM-PreviousHeightM)/Dt;
            Velocity+=(Raw-Velocity)*(1-FMath::Exp(-Dt/Spec.VelocityTauS));
        }
        else {Velocity=0;VelocitySeeded=true;}
        PreviousHeightM=In.HeightM;PreviousHeightCaptureS=In.HeightCaptureS;PreviousHeightSequence=In.HeightSequence;
    }
    if(In.PressurePa>=Spec.PressureTripPa){if(PressureSinceS<0)PressureSinceS=In.NowS;if(In.NowS-PressureSinceS>=Spec.PressureDebounceS)PressureLimited=true;}
    else {PressureSinceS=-1;if(In.PressurePa<Spec.PressureReleasePa)PressureLimited=false;}
    if(!Rebased)
    {
        const double NewTarget=FMath::Clamp(In.RequestedM,0.,TravelM-Spec.UpperMarginM);
        if(FMath::Abs(NewTarget-TargetM)>.05)Integral=0;
        TargetM=NewTarget;
    }
    const double Error=TargetM-In.HeightM;
    if(Holding&&FMath::Abs(Error)>Spec.ReopenM){Holding=false;SettledSinceS=-1;}
    if(!Holding)
    {
        if(FMath::Abs(Error)<Spec.DeadbandM&&FMath::Abs(Velocity)<.015)
        {if(SettledSinceS<0)SettledSinceS=In.NowS;if(In.NowS-SettledSinceS>=Spec.SettleDwellS)Holding=true;}
        else SettledSinceS=-1;
    }
    const bool PressureBlocked=PressureLimited||In.PressurePa>=Spec.PressureTripPa;
    if(Holding){Command=Integral=0;State=TEXT("holding");PumpEnabled=!(PressureBlocked||In.Upper);return;}
    const double Candidate=FMath::Clamp(Integral+Spec.Ki*Error*In.Dt,-Spec.IntegralCap,Spec.IntegralCap);
    const double Raw=Spec.Kp*Error+Candidate-Spec.Kd*Velocity;
    // Conditional integration prevents windup into a saturated valve.
    const double LowerSpeed=FMath::Min(Spec.MaxLowerMps,Spec.LowerApproachPerS*FMath::Abs(Error));
    const double LowerSpool=FMath::Min(Spec.DownCap,LowerSpeed/(Spec.ReturnGainMpsSqrtPa*FMath::Sqrt(FMath::Max(In.PressurePa,1000.))));
    const double Low=Error<0?-LowerSpool:0,High=Error>0?Spec.UpCap:0;
    if((Raw<=High||Error<0)&&(Raw>=Low||Error>0))Integral=Candidate;
    // Approach by closing the active flow path. Do not counter-stroke the
    // cylinder before crossing the target: that pressurizes the compliant
    // chain/fluid system and can repeatedly trigger the pressure interlock.
    double Wanted=FMath::Clamp(Spec.Kp*Error+Integral-Spec.Kd*Velocity,Low,High);
    State=TEXT("tracking");
    if((Wanted>0&&In.Upper)||(Wanted<0&&In.Lower)||(Wanted>0&&PressureBlocked))
    {
        State=PressureBlocked&&Wanted>0?TEXT("pressure_limited"):Wanted>0?TEXT("upper_limit"):TEXT("lower_limit");Command=Integral=0;PumpEnabled=Wanted<0;return;
    }
    Command=FMath::Abs(Wanted)<1e-6?0:FMath::Clamp(Command+FMath::Clamp(Wanted-Command,-Spec.SlewPerS*In.Dt,Spec.SlewPerS*In.Dt),Low,High);PumpEnabled=!PressureBlocked;
}
namespace
{
    void Write(FArchive& A,const FString& S){FTCHARToUTF8 U(*S);A.Serialize((void*)U.Get(),U.Length());}
    void Number(const TSharedRef<FJsonObject>& J,const TCHAR* K,double V,bool Valid=true){if(Valid&&FMath::IsFinite(V))J->SetNumberField(K,V);else J->SetField(K,MakeShared<FJsonValueNull>());}
}
FSensorLiftRig::~FSensorLiftRig(){if(Csv)Csv->Close();}
TSharedPtr<FSensorLiftRig> FSensorLiftRig::Create(double TravelM,double NominalReturnGain,FString& Error)
{
    SensorLift::FSpec S;if(!S.Load(Error))return nullptr;
    S.ReturnGainMpsSqrtPa=NominalReturnGain;
    double Trip=0;if(FParse::Value(FCommandLine::Get(),TEXT("LiftPressureTripPa="),Trip)){S.PressureTripPa=Trip;S.PressureReleasePa=Trip-.5e6;}
    if(!S.Valid(TravelM)){Error=TEXT("Invalid sensor lift control specification");return nullptr;}
    auto R=MakeShared<FSensorLiftRig>();R->Model.Reset(S,TravelM);
    R->CsvPath=FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir()/TEXT("Telemetry")/FString::Printf(TEXT("SensorLift-%s-%s.csv"),*FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")),*FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(8)));
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(R->CsvPath),true);R->Csv.Reset(IFileManager::Get().CreateFileWriter(*R->CsvPath));if(!R->Csv){Error=TEXT("Cannot open lift controller CSV");return nullptr;}
    Write(*R->Csv,TEXT("sim_s,state,fault_reason,armed,fault_latched,request_m,target_m,height_sensor_m,height_capture_s,height_sequence,pressure_sensor_pa,height_valid,limits_valid,pressure_valid,lower,upper,estimated_velocity_mps,integral_spool,command_spool,pump_enabled,ack_requested,ack_rejected,target_rebased,truth_height_m,truth_valve,actuator_expired\n"));UE_LOG(LogTemp,Display,TEXT("SENSOR_LIFT_CSV %s"),*R->CsvPath);return R;
}
void FSensorLiftRig::Record(double NowS,double HeightM,double Valve,bool Expired)
{
    if(!Csv||NowS+1e-9<NextCsvS)return;NextCsvS=NowS+.05;const auto& I=LastInput;const auto& M=Model;auto N=[](double V){return FString::Printf(TEXT("%.9f"),V);};auto B=[](bool V){return V?TEXT("1"):TEXT("0");};
    TArray<FString> Cells={N(NowS),M.State,M.Reason,B(M.Armed),B(M.Faulted),N(I.RequestedM),N(M.TargetM),I.HeightValid?N(I.HeightM):TEXT(""),I.HeightValid?N(I.HeightCaptureS):TEXT(""),FString::Printf(TEXT("%llu"),I.HeightSequence),I.PressureValid?N(I.PressurePa):TEXT(""),B(I.HeightValid),B(I.LimitsValid),B(I.PressureValid),B(I.Lower),B(I.Upper),N(M.Velocity),N(M.Integral),N(M.Command),B(M.PumpEnabled),B(I.Acknowledge),B(M.AckRejected),B(M.Rebased),N(HeightM),N(Valve),B(Expired)};
    Write(*Csv,FString::Join(Cells,TEXT(","))+TEXT("\n"));if(NowS>=NextFlushS){Csv->Flush();NextFlushS=NowS+5;}
}
TSharedRef<FJsonObject> FSensorLiftRig::Telemetry() const
{
    auto J=MakeShared<FJsonObject>();J->SetStringField(TEXT("mode"),TEXT("sensor_feedback_direct_valve"));J->SetStringField(TEXT("state"),Model.State);J->SetStringField(TEXT("fault_reason"),Model.Reason);J->SetBoolField(TEXT("armed"),Model.Armed);J->SetBoolField(TEXT("fault_latched"),Model.Faulted);J->SetBoolField(TEXT("safety_rated"),false);J->SetBoolField(TEXT("uses_physics_position_feedback"),false);J->SetBoolField(TEXT("pump_enabled"),Model.PumpEnabled);J->SetBoolField(TEXT("ack_rejected"),Model.AckRejected);J->SetBoolField(TEXT("target_rebased"),Model.Rebased);
    Number(J,TEXT("requested_m"),LastInput.RequestedM);Number(J,TEXT("target_m"),Model.TargetM);Number(J,TEXT("measured_height_m"),LastInput.HeightM,LastInput.HeightValid);Number(J,TEXT("measured_pressure_pa"),LastInput.PressurePa,LastInput.PressureValid);Number(J,TEXT("estimated_velocity_mps"),Model.Velocity);Number(J,TEXT("command_spool"),Model.Command);Number(J,TEXT("command_lease_s"),Model.Spec.LeaseS);Number(J,TEXT("pressure_trip_pa"),Model.Spec.PressureTripPa);return J;
}
