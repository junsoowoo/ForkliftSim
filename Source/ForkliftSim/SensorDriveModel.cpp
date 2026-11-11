#include "SensorDrive.h"
#include "MecanumModel.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonSerializer.h"
namespace SensorDrive
{
    FPose Compose(FPose A,FPose B){const double C=FMath::Cos(A.Yaw),S=FMath::Sin(A.Yaw);return {A.X+C*B.X-S*B.Y,A.Y+S*B.X+C*B.Y,FMath::UnwindRadians(A.Yaw+B.Yaw)};}
    FPose Inverse(FPose P){const double C=FMath::Cos(P.Yaw),S=FMath::Sin(P.Yaw);return {-C*P.X-S*P.Y,S*P.X-C*P.Y,-P.Yaw};}
    FPose Advance(FPose P,FVector V,double Dt)
    {
        const double A=V.Z*Dt,H=P.Yaw+A*.5,Sinc=FMath::Abs(A)<1e-8?1:FMath::Sin(A*.5)/(A*.5);
        P.X+=(FMath::Cos(H)*V.X-FMath::Sin(H)*V.Y)*Dt*Sinc;P.Y+=(FMath::Sin(H)*V.X+FMath::Cos(H)*V.Y)*Dt*Sinc;P.Yaw=FMath::UnwindRadians(P.Yaw+A);return P;
    }
    bool FSpec::Load(FString& Error)
    {
        FString Text;TSharedPtr<FJsonObject> J;if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("SensorDrive.json")))||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J){Error=TEXT("Cannot read SensorDrive.json");return false;}
        struct F {const TCHAR* K;double* V;};F Fields[]={{TEXT("filter_s"),&FilterS},{TEXT("calibration_s"),&CalibrateS},{TEXT("healthy_dwell_s"),&HealthyS},{TEXT("command_lease_s"),&LeaseS},{TEXT("wheel_kp_nm_per_mps"),&WheelKp},{TEXT("wheel_ki_nm_per_m"),&WheelKi},{TEXT("integral_limit_nm"),&IntegralNm},{TEXT("torque_limit_nm"),&MaxTorqueNm}};
        for(auto F:Fields)if(!J->TryGetNumberField(F.K,*F.V)||!FMath::IsFinite(*F.V)){Error=TEXT("Invalid sensor drive field");return false;}
        if(FilterS<.02||FilterS>.2||CalibrateS<.3||CalibrateS>2||HealthyS<.2||HealthyS>1||LeaseS<.06||LeaseS>.2||WheelKp<10||WheelKp>500||WheelKi<0||WheelKi>300||IntegralNm<0||IntegralNm>200||MaxTorqueNm<=0||MaxTorqueNm>450){Error=TEXT("Sensor drive values outside supported bounds");return false;}return true;
    }
    void FModel::Observe(const FInput& In)
    {
        NowS=In.Now;
        DataHealthy=In.Valid&&FMath::IsFinite(In.Gyro)&&In.Dt>0&&In.Dt<=.1;
        double Stamp=In.Capture[0],Newest=Stamp;for(int I=0;I<5;++I){DataHealthy&=FMath::IsFinite(In.Capture[I])&&In.Now>=In.Capture[I]&&In.Now-In.Capture[I]<=.25;Stamp=FMath::Min(Stamp,In.Capture[I]);Newest=FMath::Max(Newest,In.Capture[I]);}
        for(double R:In.Rate)DataHealthy&=FMath::IsFinite(R)&&FMath::Abs(R)<50;
        DataHealthy&=Newest-Stamp<=.06;AckRejected=false;
        if(!DataHealthy){HealthySince=-1;if(Calibrated){Faulted=true;Reason=TEXT("drive_sensor_invalid");}CalibrationSince=-1;BiasCount=0;BiasSum=0;}
        else if(HealthySince<0)HealthySince=In.Now;
        if(In.Expired){Faulted=true;Reason=TEXT("drive_command_expired");}
        if(DataHealthy&&!Calibrated&&In.ImuSequence!=LastImuSequence)
        {
            bool Still=In.Neutral&&FMath::Abs(In.Gyro)<.025;for(double R:In.Rate)Still&=FMath::Abs(R*Spec.Radius)<.02;
            if(Still){if(CalibrationSince<0)CalibrationSince=In.Now;BiasSum+=In.Gyro;++BiasCount;if(In.Now-CalibrationSince>=Spec.CalibrateS&&BiasCount>=10){GyroBias=BiasSum/BiasCount;Calibrated=true;}}
            else {CalibrationSince=-1;BiasSum=0;BiasCount=0;}
        }
        if(DataHealthy&&In.ImuSequence!=LastImuSequence)
        {
            LastImuSequence=In.ImuSequence;const double SampleDt=LastImuStamp>=0?FMath::Clamp(In.Capture[4]-LastImuStamp,.001,.1):.01;LastImuStamp=In.Capture[4];const double A=1-FMath::Exp(-SampleDt/Spec.FilterS);
            for(int I=0;I<4;++I)Wheel[I]+=A*(In.Rate[I]*Spec.Radius-Wheel[I]);LastGyro+=A*(In.Gyro-GyroBias-LastGyro);
        }
        if(DataHealthy&&Calibrated)
        {
            const FVector K=Mecanum::Forward(Wheel,Spec.Wheelbase,Spec.Track);Velocity=FVector(K.X,K.Y,LastGyro);
            if(LastStamp>=0&&Stamp>LastStamp&&Stamp-LastStamp<=.1)Odom=Advance(Odom,Velocity,Stamp-LastStamp);
            else if(LastStamp>=0&&Stamp>LastStamp)UnobservedS+=Stamp-LastStamp;
            if(Stamp>LastStamp){LastStamp=Stamp;History.Add({Stamp,Odom});while(History.Num()>1&&Stamp-History[0].Time>3)History.RemoveAt(0,1,EAllowShrinking::No);}
        }
        if(Faulted&&In.Ack){if(DataHealthy&&Calibrated&&In.Neutral&&In.Now-HealthySince>=Spec.HealthyS){Faulted=false;Reason.Empty();}else AckRejected=true;}
        Ready=DataHealthy&&Calibrated&&!Faulted;State=Faulted?TEXT("fault_latched"):!Calibrated?TEXT("calibrating"):Ready?TEXT("ready"):TEXT("waiting");
    }
    FPose FModel::Predicted(double Now) const{return Ready&&LastStamp>=0?Advance(Odom,Velocity,FMath::Clamp(Now-LastStamp,0.,.1)):Odom;}
    bool FModel::At(double T,FPose& P) const
    {
        if(History.IsEmpty()||T<History[0].Time||T>History.Last().Time)return false;
        for(int I=1;I<History.Num();++I)if(History[I].Time>=T){const auto& A=History[I-1];const auto& B=History[I];const double F=(T-A.Time)/(B.Time-A.Time);P={FMath::Lerp(A.Pose.X,B.Pose.X,F),FMath::Lerp(A.Pose.Y,B.Pose.Y,F),FMath::UnwindRadians(A.Pose.Yaw+F*FMath::UnwindRadians(B.Pose.Yaw-A.Pose.Yaw))};return true;}P=History.Last().Pose;return true;
    }
    void FModel::Control(double Dt,FVector2D Command,double Yaw,bool Hold,double SpeedCap,double YawCap)
    {
        if(Hold||!Ready){Target=FVector2D::ZeroVector;YawTarget=0;HeadingTarget=Predicted(NowS).Yaw;for(int I=0;I<4;++I)Torque[I]=Integral[I]=Targets[I]=0;return;}
        if(Command.Size()>1)Command.Normalize();Command*=FMath::Min(Spec.MaxSpeed,SpeedCap);
        const FVector2D D=Command-Target;Target+=D.GetSafeNormal()*FMath::Min(D.Size(),Spec.Acceleration*Dt);
        const double Y=FMath::Clamp(Yaw,-1.,1.)*FMath::Min(Spec.MaxYaw,YawCap);YawTarget+=FMath::Clamp(Y-YawTarget,-Spec.Acceleration*Dt,Spec.Acceleration*Dt);
        HeadingTarget=FMath::UnwindRadians(HeadingTarget+YawTarget*Dt);
        const double HeadingError=FMath::UnwindRadians(HeadingTarget-Predicted(NowS).Yaw);
        const double ControlledYaw=YawTarget+FMath::Clamp(1.5*HeadingError+.4*(YawTarget-Velocity.Z),-.25,.25);
        Mecanum::Solve(Target,ControlledYaw,Spec.Wheelbase,Spec.Track,Targets);Mecanum::Desaturate(Targets,2.);
        for(int I=0;I<4;++I){const double E=Targets[I]-Wheel[I],Candidate=FMath::Clamp(Integral[I]+E*Spec.WheelKi*Dt,-Spec.IntegralNm,Spec.IntegralNm);if(FMath::Abs(E*Spec.WheelKp+Candidate)<Spec.MaxTorqueNm||E*Integral[I]<0)Integral[I]=Candidate;Torque[I]=FMath::Clamp(E*Spec.WheelKp+Integral[I],-Spec.MaxTorqueNm,Spec.MaxTorqueNm);}
    }
    bool FModel::Correct(const TArray<FVector2D>& Points,double Stamp,double Now)
    {
        FPose AtScan;if(!At(Stamp,AtScan))return false;Registration=Match(Points,Map,Compose(MapToOdom,AtScan));
        if(!Registration.Accepted)return false;const FPose Offset=Compose(Registration.Pose,Inverse(AtScan));const double Gain=.7;
        MapToOdom={FMath::Lerp(MapToOdom.X,Offset.X,Gain),FMath::Lerp(MapToOdom.Y,Offset.Y,Gain),FMath::UnwindRadians(MapToOdom.Yaw+Gain*FMath::UnwindRadians(Offset.Yaw-MapToOdom.Yaw))};Localized=true;LastMatchS=Now;if(Velocity.Size2D()<.025&&FMath::Abs(Velocity.Z)<.025)UnobservedS=0;return true;
    }
}
