#include "MissionNavigation.h"
#include "StabilityLab.h"
#include "JointSensors.h"
#include "LidarSafety.h"
#include "PalletPerception.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
bool AStabilityLab::IsMissionTest() const{return TestMode.StartsWith(TEXT("mecanum_mission_"));}
void AStabilityLab::ControlMissionTest()
{
    Throttle=Strafe=Steering=Lift=Brake=0;if(!MissionControl||!HandlingControl)return;auto& R=*MissionControl;auto& M=R.Model;
    LiftTargetM=MissionTestStarted?M.HeightTarget:0;if(HandlingControl->Model.Working())LiftTargetM=HandlingControl->Model.HeightTarget;
    if(NavigationWalls.IsEmpty()&&TimeS<1&&FParse::Param(FCommandLine::Get(),TEXT("MissionFixture")))
    {
        auto Wall=[&](FVector P,FVector S){auto* A=GetWorld()->SpawnActor<AStaticMeshActor>(P*100,FRotator::ZeroRotator);auto* B=A->GetStaticMeshComponent();B->SetMobility(EComponentMobility::Movable);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));B->SetWorldScale3D(S);B->SetCollisionProfileName(TEXT("BlockAll"));NavigationWalls.Add(A);};
        Wall(FVector(-4.1,0,1),FVector(.2,12.4,2));Wall(FVector(14.1,0,1),FVector(.2,12.4,2));Wall(FVector(5,-6.1,1),FVector(18,.2,2));Wall(FVector(5,6.1,1),FVector(18,.2,2));
        if(TestMode.EndsWith(TEXT("blocked")))
        {auto* A=GetWorld()->SpawnActor<AStaticMeshActor>(FVector(350,0,25),FRotator::ZeroRotator);GuardObstacle=A;auto* B=A->GetStaticMeshComponent();B->SetMobility(EComponentMobility::Movable);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));B->SetWorldScale3D(FVector(.5,.5,.5));B->SetCollisionProfileName(TEXT("BlockAll"));}
    }
    if(TimeS>2&&!MissionTestStarted){R.Request=TestMode.EndsWith(TEXT("high"))?Mission::ERequest::StartHigh:Mission::ERequest::StartLow;if(M.Engaged)MissionTestStarted=true;}
    if(TestMode.EndsWith(TEXT("retry"))&&HandlingControl->Model.State==TEXT("acquire")&&!MissionTestInjected){bCameraViews=false;MissionTestInjected=true;}
    if(TestMode.EndsWith(TEXT("retry"))&&HandlingControl->Model.Faulted)bCameraViews=true;
    if(M.State==TEXT("to_dropoff")&&M.Carrying&&TimeS-M.StageS>.5&&!MissionTestInjected)
    {
        MissionTestInjected=true;MissionTestEventS=TimeS;MissionTestStartPlan=M.Replans;
        if(TestMode.EndsWith(TEXT("avoid_pause")))
        {GuardObstacle=GetWorld()->SpawnActor<AStaticMeshActor>(FVector(560,-60,25),FRotator::ZeroRotator);auto* B=GuardObstacle->GetStaticMeshComponent();B->SetMobility(EComponentMobility::Movable);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));B->SetWorldScale3D(FVector(.4,.4,.5));B->SetCollisionProfileName(TEXT("BlockAll"));}
        if(TestMode.EndsWith(TEXT("sensor_recovery"))&&JointSensors){JointSensors->Model.TimedFaultMask=1;JointSensors->Model.FaultStartS=TimeS;JointSensors->Model.FaultEndS=TimeS+2;}
        if(TestMode.EndsWith(TEXT("cancel")))R.Request=Mission::ERequest::Cancel;
    }
    if(TestMode.EndsWith(TEXT("avoid_pause"))&&MissionTestInjected&&!MissionTestPaused&&M.State==TEXT("to_dropoff")&&TimeS-MissionTestEventS>5)
    {R.Request=Mission::ERequest::Pause;MissionTestPaused=true;MissionTestPauseS=TimeS;}
    if(TestMode.EndsWith(TEXT("avoid_pause"))&&MissionTestPaused&&M.Holding()&&!MissionTestResumed&&TimeS-MissionTestPauseS>2){R.Request=Mission::ERequest::Resume;MissionTestResumed=true;}
    if(TestMode.EndsWith(TEXT("sensor_recovery"))&&MissionTestInjected&&TimeS-MissionTestEventS>3&&!MissionTestAck){AcknowledgeSensorLift();MissionTestAck=true;}
    if(TestMode.EndsWith(TEXT("sensor_recovery"))&&MissionTestAck&&M.Holding()&&DriveSensors&&DriveSensors->Model.Ready&&VehicleGuard&&!VehicleGuard->Decision.Faulted&&!HandlingControl->Model.Faulted)
    {R.Request=Mission::ERequest::Resume;MissionTestResumed=true;MissionTestPauseS=TimeS;}
    if(GuardObstacle&&TestMode.EndsWith(TEXT("avoid_pause")))
    {
        // Independent physical diagnostic only, never a planner/control input.
        const auto P=Chassis->GetActorLocation()/100.;const double DX=FMath::Max3(P.X-.8-5.8,5.4-P.X-2.15,0.),DY=FMath::Max(0.,FMath::Abs(P.Y+.6)-.85);MissionTestMinClearance=FMath::Min(MissionTestMinClearance,FMath::Sqrt(DX*DX+DY*DY));
    }
}
bool AStabilityLab::MissionTestCanFinish() const
{
    if(!MissionControl)return false;const auto& M=MissionControl->Model;
    if(M.Holding()&&TimeS-M.PausedS>6&&!TestMode.EndsWith(TEXT("sensor_recovery"))&&!(TestMode.EndsWith(TEXT("avoid_pause"))&&MissionTestPaused&&!MissionTestResumed))return true;
    if(M.State==TEXT("complete"))return TimeS-M.StageS>2;
    if(TestMode.EndsWith(TEXT("realtime")))return TimeS>20;
    if(TestMode.EndsWith(TEXT("retry")))return M.Retries==1&&HandlingControl&&HandlingControl->JobSequence>=2&&HandlingControl->Model.State==TEXT("insert");
    if(TestMode.EndsWith(TEXT("sensor_recovery")))return MissionTestResumed&&M.Navigating()&&TimeS-MissionTestPauseS>3;
    if(TestMode.EndsWith(TEXT("blocked"))||TestMode.EndsWith(TEXT("cancel"))||TestMode.EndsWith(TEXT("watchdog")))return M.Holding()&&TimeS-M.PausedS>3;
    return false;
}
bool AStabilityLab::CheckMissionTest() const
{
    if(!MissionControl||!Pallet)return false;const auto& M=MissionControl->Model;const bool Recovery=TestMode.EndsWith(TEXT("sensor_recovery")),Retry=TestMode.EndsWith(TEXT("retry")),Stop=TestMode.EndsWith(TEXT("blocked"))||TestMode.EndsWith(TEXT("cancel"))||TestMode.EndsWith(TEXT("watchdog")),Realtime=TestMode.EndsWith(TEXT("realtime"));
    bool Pass=M.State==TEXT("complete")&&M.SawPickup&&M.SawDropoff&&!M.Carrying;
    if(Stop)Pass=M.Holding()&&M.Brake&&M.Velocity.IsNearlyZero()&&!M.SawDropoff&&FMath::Abs(SpeedMps)<.04;
    if(Recovery)Pass=MissionTestAck&&MissionTestResumed&&M.Carrying&&M.Navigating()&&M.Last.Localized;
    if(Retry)Pass=M.Retries==1&&HandlingControl->JobSequence>=2&&HandlingControl->Model.Active&&HandlingControl->Model.State==TEXT("insert")&&!M.SawDropoff;
    if(Realtime)Pass=M.Engaged&&!M.Holding()&&M.Last.Healthy;
    if(!Stop&&!Retry&&!Recovery&&!Realtime)Pass&=RackSupportN>Pallet->LoadMass()*9.81*.85&&PayloadState.FilteredN[0]+PayloadState.FilteredN[1]<50&&FMath::Abs(Pallet->GetActorLocation().X/100-RackSettings.X)<.12&&FMath::Abs(Pallet->GetActorLocation().Y/100-RackSettings.Y)<.12;
    if(TestMode.EndsWith(TEXT("avoid_pause")))Pass&=MissionTestPaused&&MissionTestResumed&&M.Replans>MissionTestStartPlan+1&&MissionTestMinClearance>.04;
    Pass&=DriveSensors&&DriveSensors->FinalErrorM<.15;
    Pass&=!CargoState.bFell&&!CargoState.bTipped&&MaxTilt<10&&MaxInnerClockErrorS<1e-7;
    UE_LOG(LogTemp,Display,TEXT("MISSION_CHECK %s state=%s reason=%s pickup=%d dropoff=%d retries=%d replans=%d resumed=%d clearance_m=%.5f map_error_m=%.6f fps=%.3f realtime=%.3f %s"),*TestMode,*M.State,*M.Reason,M.SawPickup,M.SawDropoff,M.Retries,M.Replans,MissionTestResumed,MissionTestMinClearance,DriveSensors?DriveSensors->FinalErrorM:0,GameFPS(),ProgressRatio(),Pass?TEXT("PASS"):TEXT("FAIL"));return Pass;
}
