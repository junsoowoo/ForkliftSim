#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/HUD.h"
#include "Vehicle3DSettings.h"
#include "PalletLoadActor.h"
#include "CargoContactData.h"
#include "RackHandling.h"
#include "StabilityLab.generated.h"

UCLASS()
class FORKLIFTSIM_API AStabilityLab : public AActor
{
    GENERATED_BODY()
public:
    AStabilityLab();
    virtual void BeginPlay() override;
    virtual void Tick(float Dt) override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    double GameFPS() const {return FrameWallSmoothedS>0?1./FrameWallSmoothedS:0;}
    double ProgressRatio() const {return RealtimeRatio;}
    void ConfigureCameraViews();
    void UpdateCameraViews();
    void DrawCameraViews(class AHUD* ViewHUD,class UCanvas* ViewCanvas);
    void ToggleCameraViews();
    void ToggleRearCameraMode();
    void ShutdownCameraViews();
    FDelegateHandle CameraPostTickHandle;
    TSharedPtr<struct FCameraSensorRig> CameraSensors;
    TSharedPtr<struct FJointSensorRig> JointSensors;
    TSharedPtr<struct FDynamicsSensorRig> DynamicsSensors;
    TSharedPtr<struct FProximitySensorRig> ProximitySensors;
    TSharedPtr<struct FSensorLiftRig> SensorLift;
    TSharedPtr<struct FLidarSafetyRig> VehicleGuard;
    TSharedPtr<struct FSensorDriveRig> DriveSensors;
    TSharedPtr<struct FPalletPerceptionRig> PalletRecognition;
    TSharedPtr<struct FHandlingRig> HandlingControl;void RequestHandling(int RequestId);
    TSharedPtr<struct FMissionRig> MissionControl;void RequestMission(int RequestId);
    bool IsMissionTest() const;void ControlMissionTest();bool CheckMissionTest() const;bool MissionTestCanFinish() const;
    bool MissionTestStarted=false,MissionTestInjected=false,MissionTestAck=false,MissionTestResumed=false,MissionTestPaused=false;
    double MissionTestEventS=0,MissionTestPauseS=0,MissionTestStartPlan=0,MissionTestMinClearance=100;
    bool IsHandlingTest() const;void ControlHandlingTest();bool CheckHandlingTest() const;
    bool HandlingTestRequested=false,HandlingTestFaultInjected=false;
    bool IsPalletPerceptionTest() const;void ControlPalletPerceptionTest();bool CheckPalletPerceptionTest() const;
    bool PalletTestSawDetection=false,PalletTestSawPartial=false,PalletTestSawFull=false,PalletTestConfigured=false;
    int PalletTestFrames=0;double PalletTestMaxDistanceError=0;
    UPROPERTY() TObjectPtr<class AStaticMeshActor> PalletVisionDistractor;
    bool bSensorDriveRequested=false;
    bool IsSensorDriveTest() const;void ControlSensorDriveTest();bool CheckSensorDriveTest() const;
    UPROPERTY() TArray<TObjectPtr<class AStaticMeshActor>> NavigationWalls;
    void ClearNavigationWalls();bool SetupNavigationLandmarks();
    bool bVehicleSafetyRequested=false;
    bool IsVehicleGuardTest() const;void ControlVehicleGuardTest();bool CheckVehicleGuardTest() const;
    void EmergencyVehicleStop();void ToggleLidarRays();
    UPROPERTY() TObjectPtr<class AStaticMeshActor> GuardObstacle;
    bool bSensorLiftRequested=false;
    bool IsSensorLiftTest() const;
    void UpdateSensorLiftControl(double Dt);
    void ControlSensorLiftTest();bool CheckSensorLiftTest() const;
    void AcknowledgeSensorLift();
    void ToggleJointSensorOutage();
    void ToggleProximityRays();
    bool bRearDepthView=false;
    bool bCameraViews=false;
    UPROPERTY() TObjectPtr<class UTextureRenderTarget2D> FrontViewTexture;
    UPROPERTY() TObjectPtr<class UTextureRenderTarget2D> RearViewTexture;
    UPROPERTY() TObjectPtr<class USceneCaptureComponent2D> FrontViewCapture;
    UPROPERTY() TObjectPtr<class USceneCaptureComponent2D> RearViewCapture;
    UPROPERTY() TObjectPtr<class UTexture2D> FrontDepthImage;
    UPROPERTY() TObjectPtr<class UTexture2D> RearColorImage;
    UPROPERTY() TObjectPtr<class UTexture2D> RearDepthImage;
    double NextFrontCaptureS=0,NextRearCaptureS=.0625;
    void ResetRig();
    void ChangeFriction(bool Tyre, double Delta);
    void ToggleRoadGrip();
    bool FrictionMaterialsMatch() const;
    void CycleStress();
    FVehicle3DSettings Settings;
    FRackSettings RackSettings;
    FRackTrial RackTrial;
    UPROPERTY() TObjectPtr<ARackHandlingCourse> HandlingRack;
    double RackSupportN=0;
    bool IsRackTest() const;
    bool IsRackPickup() const;
    double RackTierM() const;
    void SetupHandlingRack();
    void ControlRackTest();
    bool CheckRackTest() const;
    UPROPERTY() TObjectPtr<class AChassisActor> Chassis;
    UPROPERTY() TObjectPtr<class AForkActor> Fork;
    UPROPERTY() TArray<TObjectPtr<class AWheelActor>> Wheels;
    double Throttle=0, Brake=0, Steering=0, Lift=0, SteerDeg=0, LiftTargetM=0;
    double Strafe=0, TargetYawRadS=0;
    double TargetHeadingDeg=0;
    double WheelSpeedScale=1, AllocatedYawRadS=0, CoordinatedYawRadS=0;
    bool bMecanumAutoHold=false;
    bool bAligning=false, bLateralReorienting=false;
    bool bNeutral=false, bResistanceEnabled=true, bLoadView=true;
    double TotalNormalLoadN=0;
    bool bPayload=false;
    bool bPayloadResetTestDone=false;
    FPayloadSettings PayloadSettings;
    UPROPERTY() TObjectPtr<APalletLoadActor> Pallet;
    struct FPayloadState
    {
        double VerticalN[2]={0,0}, NormalN[2]={0,0}, FrictionN[2]={0,0}, FilteredN[2]={0,0}, DepthM[2]={0,0}, CopXM[2]={0,0};
        FVector PointWeighted[2]={FVector::ZeroVector,FVector::ZeroVector};
        FVector LoadCenterM=FVector::ZeroVector, InitialRelative=FVector::ZeroVector, Start=FVector::ZeroVector;
        double GroundN=0, HeightM=0, TiltDeg=0, MaxTilt=0, MaxSlip=0;
        double SampleS=0, WheelIntegral=0, FrontIntegral=0, ForkIntegral[2]={0,0}, ExpectedFrontIntegral=0;
        bool bSeeded=false;
        double TurnTimeS=0, TurnDistanceM=0, TurnYawIntegral=0, MinTurnSpeedMps=100;
        double SCurveYawIntegral[3]={0,0,0}, SCurveSampleS[3]={0,0,0};
    } PayloadState;
    struct FCargoState
    {
        FCargoContactData Frame;
        FVector InitialRelativeM=FVector::ZeroVector, ShiftM=FVector::ZeroVector;
        FVector ForceN=FVector::ZeroVector, FrictionForceN=FVector::ZeroVector, ContactPointM=FVector::ZeroVector;
        double NormalN=0, FrictionN=0, GroundN=0, OtherN=0, FilteredNormalN=0, FilteredFrictionN=0;
        double SlipMps=0, TiltDeg=0, WorldTiltDeg=0, Overlap=1, MaxShiftM=0, MaxTiltDeg=0, MaxFrictionN=0, MaxUtilization=0;
        double MaxPlanarUtilization=0, PlanarSampleS=0;
        double SeparationS=0, SampleS=0, NormalIntegral=0, VerticalIntegral=0, GroundIntegral=0, WheelIntegral=0, ForkIntegral=0;
        double BaselineFrontIntegral=0, BaselineWheelIntegral=0;
        bool bArmed=false, bLostSupport=false, bTipped=false, bFell=false, bSeeded=false;
    } CargoState;
    bool IsCargoTest() const;
    void ConfigureCargoTest();
    void UpdateCargo(double Dt,double PhysicsDt);
    void ControlCargoTest(double Dt);
    bool CheckCargoTest() const;
    FString CargoCsv(bool Header) const;
    bool ConfigurePayload();
    void SpawnPayload();
    void AccumulatePayload(const struct FContactLoadOutput& Output);
    void UpdatePayload(double Dt,double PhysicsDt);
    void ControlPayloadTest(double Dt);
    bool CheckPayloadTest();
    FString PayloadCsv(bool Header) const;
    double AlignmentErrorDeg=0;
    FVector2D TargetVelocity=FVector2D::ZeroVector;
    double TimeS=0, SpeedMps=0, LateralMps=0, YawDegS=0, TiltDeg=0, RollDeg=0, PitchDeg=0, ForkHeightM=0;
    double MaxTilt=0, MaxLateralSlip=0, MaxYawRate=0, MaxSpeed=0, MassKg=0, AccelerationMps2=0;
    int Grounded=0, MinGrounded=4;
    FString Error, TestMode, FrictionNotice;
    double FrictionNoticeUntil=0;
    FString TerrainPreset=TEXT("flat");
    bool bMotorView=false;
    int32 PowerResetStage=0;
    UPROPERTY() TObjectPtr<class ATerrainCourse> Course;
    struct FPowerState
    {
        double SampleS=0, SlopeSampleS=0, SlopeSpeedIntegral=0, DistanceM=0;
        double LeftMuIntegral=0, RightMuIntegral=0, LeftSlipMax=0, RightSlipMax=0;
        double LeftSlipIntegral=0, SlipSampleS=0;
        double MaxMotorW=0, MaxSteerNm=0, MaxRoadSteerNm=0, MaxSteerDelay=0;
        double DcEnergyJ=0, WheelEnergyJ=0, LossEnergyJ=0;
        bool bLimitsValid=true, bCourseContact=false;
    } PowerState;
    bool IsPowerTest() const;
    bool ConfigurePowerLab();
    void SetupTerrain();
    void CycleTerrain();
    void ControlPowerTest();
    void UpdatePower(double Dt);
    bool CheckPowerTest() const;
    FString PowerCsv(bool Header) const;
    bool bTyreView=false;
    struct FTyreState
    {
        double SampleS=0, CompressionIntegral[4]={0,0,0,0}, NormalIntegral[4]={0,0,0,0}, AxleIntegral[4]={0,0,0,0};
        double PeakCompressionM=0, PeakVerticalAccel=0, VelocitySquareIntegral=0, VibrationS=0;
        double DampingJ=0, EnergyReferenceJ=0, EnergyJ=0, MaxEnergyGainJ=0, MaxAccountedGainJ=0;
        double RollingIntegral=0, RollingS=0, InitialFrontCompressionM=0;
        FVector LastVelocity=FVector::ZeroVector;
        bool bBottomed=false;
    } TyreState;
    bool IsTyreTest() const;
    bool ConfigureTyres();
    void ControlTyreTest();
    void UpdateTyres(double Dt);
    bool CheckTyreTest() const;
    double TyreMechanicalEnergyJ() const;
    FString TyreCsv(bool Header) const;
    struct FEnergyAudit
    {
        double DriveJ=0,RollingJ=0,SteeringJ=0;
        double BearingTransportJ=0,ReferenceBearingTransportJ=0;
        double RollerBearingJ=0,ReferenceRollerBearingJ=0;
        double ShaftJointJ=0,ReferenceShaftJointJ=0,ReferenceBrakeElasticJ=0,ShaftLossJ=0;
        double ReferenceJ=0,ReferenceDriveJ=0,ReferenceRollingJ=0,ReferenceSteeringJ=0;
        double ReferenceDcJ=0,ReferenceHeatJ=0,ReferenceTyreJ=0;
        double ReferenceChainLossJ=0;
        double ContactNormalJ=0,ContactTangentJ=0,CorrectionPseudoJ=0,ForkDampingEstimateJ=0;
        double RefContactNormalJ=0,RefContactTangentJ=0,RefCorrectionPseudoJ=0,RefForkDampingEstimateJ=0;
        double ReducedLossJ=0,RefReducedLossJ=0,PlasticRelaxationJ=0,RefPlasticRelaxationJ=0,FractureReleaseJ=0,RefFractureReleaseJ=0;
        int64 ContactWorkPoints=0;
        double PalletPhysicsMassKg=0,CargoPhysicsMassKg=0;
        double DeltaStoredJ=0,InputJ=0,KnownLossJ=0,UnresolvedJ=0,PeakPositiveJ=0;
        bool bReferenced=false;
    } EnergyAudit;
    double BasePhysicsStepS=1./480.,SelectedPhysicsStepS=1./480.,SmallestPhysicsStepS=1./480.;
    double SmallestObservedPhysicsStepS=1./480.,MaxInnerClockErrorS=0;
    void SelectMecanumStep();
    void SetMecanumIterations(AActor* Actor);
    void PrepareEnergyWork();
    void UpdateEnergyAudit();
    FString EnergyCsv(bool Header) const;
    bool bHydraulicView=false;
    struct FHydraulicLabState
    {
        double PeakPressure=0, PeakDcW=0, ReliefM3=0, LiftS=0, LiftDistance=0;
        double HoldStartM=0, HoldEndM=0, LowerMinM=10;
        double PreviousHeightM=0;
        bool bLimitsValid=true;
    } HydraulicLabState;
    bool IsHydraulicTest() const;
    bool ConfigureHydraulics();
    void ControlHydraulicTest();
    void UpdateHydraulicLab(double Dt);
    bool CheckHydraulicTest() const;
    FString HydraulicCsv(bool Header) const;
    // Ground-truth validation fixtures, not sensors or autonomous navigation.
    struct FValidationState
    {
        int Stage=0;
        double StaticS=0, NormalIntegral=0, FrontIntegral=0, ExpectedFrontIntegral=0;
        double LeftIntegral=0, ExpectedLeftIntegral=0, PressureIntegral=0;
        double UphillS=0, DownhillS=0, PeakX=0, PeakZ=0, BrakeStartX=0, StopDistanceM=0;
        double PeakLoadN=0, MinWheelN=1e9, PeakSlipMps=0, LeftMuIntegral=0, RightMuIntegral=0, TravelS=0;
        double PeakInnerLoadN=0, MinInnerWheelN=1e30, PlateauAtS=0;
        int64 RefinedContactCount=0;
        int64 RefinedEdgeCount=0;
        double ContactBandError=0, ContactNormalError=0, ContactPeakLoadN=0, ContactPeakCompressionM=0;
        int32 ContactPeakPoints=0;
        double TreadStaticS=0,TreadCompressionIntegral=0,TreadLoadIntegral=0,TreadSpreadM=0,TreadClosureN=0;
        double TreadPumpStartJ=0;
        double PhysicsSampleS=0;
        double CoastStartS=0, CoastStartJ=0, CoastDeltaJ=0, CoastPeakGainJ=0, CoastStartMps=0;
        double MaxInnerStepS=0, MaxGameStepS=0;
        double PeakCrossHeightM=0;
        double HoldSampleS=0, HoldStartX=0, HoldDriftM=0, HoldMaxSpeedMps=0;
        bool bLifted=false, bCourseContact=false, bStopped=false;
    } ValidationState;
    bool IsIntegrationTest() const;
    bool ConfigureValidation();
    void ControlIntegrationTest();
    void UpdateValidation(double Dt);
    bool CheckIntegrationTest() const;
    FString ValidationCsv(bool Header) const;
    bool IsComponentTest() const { return TestMode.StartsWith(TEXT("swerve_components_")); }
    void ControlComponentTest();
    bool CheckComponentTest() const;
private:
    UPROPERTY() TObjectPtr<class UPhysicalMaterial> TyreMaterial;
    UPROPERTY() TObjectPtr<class UPhysicalMaterial> RoadMaterial;
    UPROPERTY() TObjectPtr<class UStaticMeshComponent> Road;
    TArray<FString> Rows;
    FString CsvPath;
    double NextRecord=0, NextFlush=5, ForkBaseZ=0, SettledTilt=0, SettledSpeed=0, MaxForkHeight=0;
    double NormalComOffset=0, NormalMaxSpeed=3, NormalWheelSurfaceLimit=4, NormalSteerRate=30, NormalMaxSteer=30, MaxWheelSurfaceSpeed=0;
    double CaptureTimeS=3;
    bool bFrictionSwitched=false, bFrictionRestored=false, bFrictionKeysPassed=false;
    double InitialRoadMu=.8, MaxLiveSlip=0;
    double MinPoweredTurnSpeed=TNumericLimits<double>::Max();
    double MaxSideSpeed=0, MaxModuleError=0;
    FVector SwerveStart=FVector::ZeroVector;
    void DriveSwerve(double Dt);
    void DriveMecanum(double Dt);
    void ControlMecanumTest();
    bool CheckMecanumTest() const;
    FString MecanumCsv(bool Header) const;
    bool bReactionIsolated=false;
    FVector ReactionInitialL=FVector::ZeroVector;
    FVector ReactionOriginM=FVector::ZeroVector;
    UPROPERTY() TObjectPtr<class AChassisActor> ReactionChassis;
    UPROPERTY() TObjectPtr<class AWheelActor> ReactionWheel;
    double ReactionError=0,ReactionImpulse=0,ReactionPitch=0,ReactionPtError=0;
    double ReactionCoastStartJ=0,ReactionCoastLastJ=0,ReactionCoastPeakJ=0,ReactionPredictorWorkJ=0;
    bool bReactionCoastSample=false;
    double ClockWallStartS=0,ClockSimStartS=0,RealtimeRatio=1;
    double LastFrameWallS=0,FrameWallSmoothedS=0;
    TArray<double> PerformanceFrameMs;
    void WritePerformanceReport();
    FVector2D MecanumVelocityIntegral=FVector2D::ZeroVector;
    FVector PreviousVelocity=FVector::ZeroVector;
    double PreviousPhysicsDt=0;
    class FContactLoadSampler* LoadSampler=nullptr;
    TSharedPtr<class FWebTelemetryPublisher,ESPMode::ThreadSafe> WebTelemetry;
    void PublishWebTelemetry();
    void ReleaseLoadSampler();
    double LoadSumIntegral=0, FrontLoadIntegral=0, LoadSampleTime=0, ExpectedFrontFraction=0;
    double CoastStartSpeed=0;
    bool bSeeded=false, bFinished=false, bCapture=false, bCaptured=false, bStress=false;
    void ApplyMaterials();
    void SeedSpeed(double Speed);
    void FinishTest();
    void Flush();
};

UCLASS()
class FORKLIFTSIM_API AStabilityController : public APlayerController
{
    GENERATED_BODY()
public:
    virtual void BeginPlay() override;
    virtual void PlayerTick(float Dt) override;
    virtual void SetupInputComponent() override;
    bool TestFrictionBindings();
private:
    UPROPERTY() TObjectPtr<class ACameraActor> LabCamera;
    void Reset(); void TyreDown(); void TyreUp(); void RoadDown(); void RoadUp(); void Stress();
    void RoadPreset();
    void ToggleView();
    void ToggleNeutral(); void ToggleResistance(); void ToggleLoadView();
    void ToggleMotorView(); void NextTerrain();
    void ToggleTyreView();
    void ToggleHydraulicView();
    void ToggleCameraViews();
    void ToggleRearCameraMode();
    bool bTopView=false;
    void ToggleJointSensorOutage();
    void ToggleProximityRays();
    void AcknowledgeSensorLift();
    void EmergencyVehicleStop();void ToggleLidarRays();
    void StartMission();void PauseMission();void ResumeMission();void SelectMissionLow();void SelectMissionHigh();
    void PickGround();void PickLow();void PickHigh();void PlaceLow();void PlaceHigh();void CancelHandling();
    double CameraYawDeg=-62.86, CameraPitchDeg=-16.65, CameraDistanceCm=1396;
    FVector CameraPan=FVector::ZeroVector;
    FVector2D PreviousMouse=FVector2D::ZeroVector;
    bool bHadMouse=false;
    void ZoomIn(); void ZoomOut(); void ResetCamera();
    void RepeatTyreDown(); void RepeatTyreUp(); void RepeatRoadDown(); void RepeatRoadUp();
    void FrictionInput(bool Tyre,double Delta,bool Repeat);
    double NextFrictionRepeat=0;
};

UCLASS()
class FORKLIFTSIM_API AStabilityHUD : public AHUD
{
    GENERATED_BODY()
public:
    virtual void DrawHUD() override;
    void DrawPayload(AStabilityLab* B,float Scale);
    void DrawCargo(AStabilityLab* B,float Scale,float X,float Y);
    void DrawMotors(AStabilityLab* B,float Scale);
    void DrawTyres(AStabilityLab* B,float Scale);
    void DrawHydraulics(AStabilityLab* B,float Scale);
};
