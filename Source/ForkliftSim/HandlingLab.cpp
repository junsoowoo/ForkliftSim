#include "HandlingControl.h"
#include "StabilityLab.h"
#include "PalletPerception.h"
#include "LidarSafety.h"
#include "JointSensors.h"
#include "ProximitySensors.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "Engine/StaticMeshActor.h"
bool AStabilityLab::IsHandlingTest() const{return TestMode.StartsWith(TEXT("mecanum_handling_"));}
void AStabilityLab::ControlHandlingTest()
{
    Throttle=Strafe=Steering=Lift=Brake=0;
    if(!HandlingControl)return;auto& R=*HandlingControl;const bool Place=TestMode.Contains(TEXT("place"));
    if(HandlingTestRequested)LiftTargetM=R.Model.HeightTarget;
    // Place fixtures are prepared by a stationary manual lift, before the
    // handling job starts. No truth is fed to the job controller.
    if(Place&&!HandlingTestRequested)LiftTargetM=TimeS>2?.25:0;
    if(TimeS>(Place?9:2)&&!HandlingTestRequested)
    {
        R.Request=Place?(TestMode.EndsWith(TEXT("high"))?Handling::ERequest::PlaceHigh:Handling::ERequest::PlaceLow):(TestMode.EndsWith(TEXT("high"))?Handling::ERequest::PickHigh:TestMode.EndsWith(TEXT("low"))?Handling::ERequest::PickLow:Handling::ERequest::PickGround);
        if(R.Model.Active)HandlingTestRequested=true;
    }
    if(!HandlingTestFaultInjected&&R.Model.State==TEXT("insert"))
    {
        if(TestMode.EndsWith(TEXT("camera_loss")))bCameraViews=false;
        if(TestMode.EndsWith(TEXT("sensor_loss"))&&ProximitySensors)ProximitySensors->Model.ManualOutage=true;
        if(TestMode.EndsWith(TEXT("estop"))&&VehicleGuard)VehicleGuard->EstopRequested=true;
        if(TestMode.EndsWith(TEXT("cancel")))R.Cancel=true;
        if(TestMode.EndsWith(TEXT("obstacle")))
        {
            GuardObstacle=GetWorld()->SpawnActor<AStaticMeshActor>(Chassis->GetActorLocation()+FVector(0,90,-25),FRotator::ZeroRotator);auto* B=GuardObstacle->GetStaticMeshComponent();B->SetMobility(EComponentMobility::Movable);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));B->SetWorldScale3D(FVector(.25,.25,.5));B->SetCollisionProfileName(TEXT("BlockAll"));
        }
        HandlingTestFaultInjected=true;
    }
}
bool AStabilityLab::CheckHandlingTest() const
{
    if(!HandlingControl||!Pallet)return false;const auto& M=HandlingControl->Model;
    const bool Fault=TestMode.EndsWith(TEXT("camera_loss"))||TestMode.EndsWith(TEXT("sensor_loss"))||TestMode.EndsWith(TEXT("estop"))||TestMode.EndsWith(TEXT("cancel"))||TestMode.EndsWith(TEXT("obstacle"));
    const double Load=PayloadState.FilteredN[0]+PayloadState.FilteredN[1],Weight=Pallet->LoadMass()*9.81;
    bool Pass=Fault?M.Faulted&&!M.Active&&M.Brake&&FMath::Abs(SpeedMps)<.03:M.State==TEXT("complete")&&M.TransferSeen&&!M.Faulted&&FMath::Abs(ForkHeightM-M.Spec.Carry)<.025;
    if(!Fault)Pass&=M.Place?(RackSupportN>Weight*.85&&Load<50&&FMath::Abs(Pallet->GetActorLocation().X/100-RackSettings.X)<.12):(M.FullSeen&&Load>Weight*.8&&PayloadState.GroundN<20&&RackSupportN<20);
    Pass&=!CargoState.bFell&&!CargoState.bTipped&&MaxTilt<10&&MaxInnerClockErrorS<1e-7;
    UE_LOG(LogTemp,Display,TEXT("HANDLING_CHECK %s state=%s reason=%s full=%d transfer=%d load_n=%.4f rack_n=%.4f fork_m=%.5f pallet_x=%.5f %s"),*TestMode,*M.State,*M.Reason,M.FullSeen,M.TransferSeen,Load,RackSupportN,ForkHeightM,Pallet->GetActorLocation().X/100,Pass?TEXT("PASS"):TEXT("FAIL"));return Pass;
}
