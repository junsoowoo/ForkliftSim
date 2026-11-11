#include "LidarSafety.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "SensorLiftControl.h"
#include "Engine/StaticMeshActor.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"

bool AStabilityLab::IsVehicleGuardTest() const{return TestMode.StartsWith(TEXT("mecanum_vehicle_guard_"));}
void AStabilityLab::EmergencyVehicleStop(){if(VehicleGuard)VehicleGuard->EstopRequested=true;}
void AStabilityLab::ToggleLidarRays(){if(VehicleGuard)VehicleGuard->ShowRays=!VehicleGuard->ShowRays;}
void AStabilityController::EmergencyVehicleStop(){for(TActorIterator<AStabilityLab> I(GetWorld());I;++I){I->EmergencyVehicleStop();break;}}
void AStabilityController::ToggleLidarRays(){for(TActorIterator<AStabilityLab> I(GetWorld());I;++I){I->ToggleLidarRays();break;}}
void AStabilityLab::ControlVehicleGuardTest()
{
    Throttle=Strafe=Steering=Lift=0;Brake=0;if(!VehicleGuard)return;auto& G=*VehicleGuard;
    const bool Fault=TestMode.EndsWith(TEXT("fault")),Estop=TestMode.EndsWith(TEXT("estop")),Loaded=TestMode.EndsWith(TEXT("carry"));
    auto Once=[&](uint32 Bit,double At){if(TimeS<At||(G.TestBits&Bit))return false;G.TestBits|=Bit;return true;};
    if(Fault||Estop)
    {
        if(Fault)G.Sensor.FaultMask=1;
        LiftTargetM=TimeS<2?0:TimeS<5?.45:1.8;
        if(Once(1,4.5))G.AckRequested=true;
        if(Estop&&Once(2,6))G.EstopRequested=true;
        if(TimeS>=5&&TimeS<12)Throttle=1;
        if(Once(4,14))AcknowledgeSensorLift();
        if(G.SawFaultRecovery&&TimeS<18)LiftTargetM=G.RecoveryHeightM;
        if(TimeS>=18)LiftTargetM=.8;
        if(TimeS>=16&&TimeS<24)Throttle=1;if(TimeS>=24)Brake=1;return;
    }
    LiftTargetM=Loaded&&TimeS>=2?.45:0;
    if(!GuardObstacle&&TimeS<1)
    {
        const bool Rear=TestMode.EndsWith(TEXT("rear")),Side=TestMode.EndsWith(TEXT("side")),Diagonal=TestMode.EndsWith(TEXT("diagonal"));
        const FVector P=Diagonal?FVector(8,8,1):Side?FVector(0,5,1):FVector(Rear?-5:8,0,1);
        const FVector Size=Diagonal?FVector(2,2,2):Side?FVector(8,.2,2):FVector(.2,8,2);
        GuardObstacle=GetWorld()->SpawnActor<AStaticMeshActor>(P*100,FRotator::ZeroRotator);auto* B=GuardObstacle->GetStaticMeshComponent();B->SetMobility(EComponentMobility::Movable);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));B->SetWorldScale3D(Size);B->SetCollisionProfileName(TEXT("BlockAll"));
    }
    if(!Loaded&&GuardObstacle&&Once(8,1))
    {
        // Independent analytic ray/box entry check. True pose/bounds here are
        // fixture diagnostics only, never supplied to the protective controller.
        const int Id=(TestMode.EndsWith(TEXT("rear"))||TestMode.EndsWith(TEXT("side")))?1:0;const auto& Spec=G.Sensor.Spec;
        const FTransform Pose=Chassis->Body->GetComponentTransform();const FVector Origin=Pose.TransformPositionNoScale(Spec.Mount(Id)*100),To=Pose.InverseTransformVectorNoScale(GuardObstacle->GetActorLocation()-Origin);
        double Deg=FMath::RadiansToDegrees(FMath::Atan2(To.Y,To.X));if(Id&&Deg<0)Deg+=360;
        const int K=FMath::Clamp(FMath::RoundToInt((Deg-(Id?0:-180))*(Spec.Rays-1)/270.),0,Spec.Rays-1);const double A=Spec.Angle(Id,K);const FVector D=Pose.TransformVectorNoScale(FVector(FMath::Cos(A),FMath::Sin(A),0));const FBox Box=GuardObstacle->GetComponentsBoundingBox(true);
        double Near=0,Far=1.e12;for(int Axis=0;Axis<3;++Axis){if(FMath::Abs(D[Axis])<1e-9)continue;double L=(Box.Min[Axis]-Origin[Axis])/D[Axis],H=(Box.Max[Axis]-Origin[Axis])/D[Axis];if(L>H)Swap(L,H);Near=FMath::Max(Near,L);Far=FMath::Min(Far,H);}
        const auto& Scan=G.Sensor.Channels[Id].Last;const double Got=Scan.Ranges.IsValidIndex(K)?Scan.Ranges[K]:0;
        const bool Beyond=Near/100.>Spec.MaxM;
        G.TestRangePass=G.Sensor.Fresh(Id,TimeS)&&Far>=Near&&Scan.States.IsValidIndex(K)&&(Beyond?Scan.States[K]==LidarSafety::EReturn::NoReturn:Scan.States[K]==LidarSafety::EReturn::Valid&&FMath::Abs(Got-Near/100.)<.025);
        UE_LOG(LogTemp,Display,TEXT("LIDAR_RANGE_CHECK %s measured_m=%.6f analytic_m=%.6f %s"),*TestMode,Got,Near/100.,G.TestRangePass?TEXT("PASS"):TEXT("FAIL"));
    }
    const double Start=Loaded?15:2,Remove=Loaded?25:14,Resume=Loaded?29:18;
    if(Loaded&&Once(1,14))G.AckRequested=true;
    if(TimeS>=Start&&TimeS<Resume-2)
    {
        if(TestMode.EndsWith(TEXT("side")))Strafe=1;
        else if(TestMode.EndsWith(TEXT("diagonal")))Throttle=Strafe=1;
        else Throttle=TestMode.EndsWith(TEXT("rear"))?-1:1;
    }
    if(TimeS>=Remove&&GuardObstacle){GuardObstacle->Destroy();GuardObstacle=nullptr;}
    if(Once(2,Remove+1))G.AckRequested=true; // Deliberately still holding input: reject.
    if(Once(4,Resume-1))G.AckRequested=true;
    if(TimeS>=Resume&&TimeS<Resume+5)Throttle=1;
    if(TimeS>=Resume+5)Brake=1;
}
bool AStabilityLab::CheckVehicleGuardTest() const
{
    if(!VehicleGuard)return false;const auto& G=*VehicleGuard;
    bool Pass=MaxSpeed>.25&&MaxTilt<5&&!CargoState.bFell&&FMath::Abs(SpeedMps)<.1&&FMath::Abs(LateralMps)<.1;
    if(TestMode.EndsWith(TEXT("fault"))||TestMode.EndsWith(TEXT("estop")))Pass&=G.SawFault&&G.SawFaultRecovery;
    else Pass&=G.SawObstacle&&G.SawRecovered&&(TestMode.EndsWith(TEXT("carry"))||G.TestRangePass);
    UE_LOG(LogTemp,Display,TEXT("VEHICLE_GUARD_CHECK %s obstacle=%d fault=%d recovery=%d speed=%.6f side=%.6f fork=%.5f %s"),*TestMode,G.SawObstacle,G.SawFault,G.SawRecovered,SpeedMps,LateralMps,ForkHeightM,Pass?TEXT("PASS"):TEXT("FAIL"));return Pass;
}
