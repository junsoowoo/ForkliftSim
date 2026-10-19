#include "LiftBench.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/EngineVersion.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "UObject/ConstructorHelpers.h"
#include "UnrealClient.h"

DEFINE_LOG_CATEGORY_STATIC(LogForkliftLab, Log, All);

namespace
{
    constexpr double HomeHeightCm = 30.0;

    void ColorPart(UStaticMeshComponent* Mesh, FLinearColor Color)
    {
        UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Materials/M_Lab.M_Lab"));
        if (!Base) Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
        if (Base)
        {
            UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(Base, Mesh);
            Material->SetVectorParameterValue(TEXT("Tint"), Color);
            Mesh->SetMaterial(0, Material);
        }
    }
}

ALiftBench::ALiftBench()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickGroup = TG_PrePhysics;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("BenchRoot"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
    auto Part = [&](const TCHAR* Name, USceneComponent* Parent, FVector Position, FVector Size)
    {
        auto* Mesh = CreateDefaultSubobject<UStaticMeshComponent>(Name);
        Mesh->SetupAttachment(Parent);
        Mesh->SetStaticMesh(Cube.Object);
        Mesh->SetRelativeLocation(Position);
        Mesh->SetRelativeScale3D(Size / 100.0);
        Mesh->SetCollisionProfileName(TEXT("PhysicsActor"));
        Mesh->SetMobility(EComponentMobility::Movable);
        return Mesh;
    };
    Mast = Part(TEXT("FixedMast"), RootComponent, FVector(-30, 0, 140), FVector(20, 140, 280));
    Carriage = Part(TEXT("Carriage"), RootComponent, FVector(0, 0, HomeHeightCm), FVector(12, 100, 40));
    // Absolute scale avoids inheriting the carriage's non-uniform scale.
    LeftFork = Part(TEXT("LeftFork"), Carriage, FVector::ZeroVector, FVector(140, 12, 8));
    RightFork = Part(TEXT("RightFork"), Carriage, FVector::ZeroVector, FVector(140, 12, 8));
    for (auto* Fork : { LeftFork.Get(), RightFork.Get() }) Fork->SetAbsolute(false, false, true);
    Rail = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("VerticalGuide"));
    Rail->SetupAttachment(RootComponent);
}

void ALiftBench::BeginPlay()
{
    Super::BeginPlay();
    FString Error;
    if (!Settings.Load(Error))
    {
        UE_LOG(LogForkliftLab, Error, TEXT("%s"), *Error);
        Phase = ELiftLabPhase::Fault;
        FaultReason = Error;
        SetActorTickEnabled(false);
        if (FParse::Param(FCommandLine::Get(), TEXT("ForkliftSmokeTest"))) FPlatformMisc::RequestExitWithStatus(false, 2);
        return;
    }
    FParse::Value(FCommandLine::Get(), TEXT("LiftTest="), TestMode);
    bCaptureMode = FParse::Param(FCommandLine::Get(), TEXT("LiftCapture"));
    if (TestMode == TEXT("overload")) Settings.MaxLiftForceN = 1500.0;
    ConfigurePhysics();
    ResetBench();
}

void ALiftBench::ConfigurePhysics()
{
    ContactMaterial = NewObject<UPhysicalMaterial>(this);
    ContactMaterial->Friction = Settings.ContactFriction;
    ContactMaterial->StaticFriction = Settings.ContactFriction;
    ContactMaterial->Restitution = 0;
    ContactMaterial->bOverrideFrictionCombineMode = true;
    ContactMaterial->FrictionCombineMode = EFrictionCombineMode::Min;
    InitialCarriagePosition = GetActorLocation() + FVector(0, 0, HomeHeightCm);
    Carriage->SetWorldLocation(InitialCarriagePosition, false, nullptr, ETeleportType::TeleportPhysics);
    Carriage->SetLinearDamping(0);
    Carriage->SetAngularDamping(0);
    Carriage->SetSimulatePhysics(true);
    int Index = 0;
    for (auto* Fork : { LeftFork.Get(), RightFork.Get() })
    {
        Fork->SetWorldLocation(InitialCarriagePosition + FVector(70, Index++ == 0 ? -30 : 30, -6));
        Fork->SetPhysMaterialOverride(ContactMaterial);
        // Welded body mass is the sum of each constituent's mass, not a root override.
        Fork->BodyInstance.SetMassOverride(Settings.CarriageMassKg * 0.2, true);
        Fork->AttachToComponent(Carriage, FAttachmentTransformRules(EAttachmentRule::KeepWorld, true));
    }
    Carriage->SetPhysMaterialOverride(ContactMaterial);
    Carriage->SetMassOverrideInKg(NAME_None, Settings.CarriageMassKg * 0.6);
    Carriage->BodyInstance.bUseCCD = true;
    // Symmetric travel around mid-stroke; carriage starts at the lower limit.
    Rail->SetWorldLocation(InitialCarriagePosition + FVector(0, 0, Settings.LiftTravelM * 50.0));
    Rail->SetLinearXLimit(LCM_Locked, 0);
    Rail->SetLinearYLimit(LCM_Locked, 0);
    Rail->SetLinearZLimit(LCM_Limited, Settings.LiftTravelM * 50.0);
    Rail->SetAngularSwing1Limit(ACM_Locked, 0);
    Rail->SetAngularSwing2Limit(ACM_Locked, 0);
    Rail->SetAngularTwistLimit(ACM_Locked, 0);
    Rail->SetDisableCollision(true);
    Rail->SetConstrainedComponents(Mast, NAME_None, Carriage, NAME_None);
    // Frame 2 at the carriage origin creates an actual lower-to-upper travel range.
    Rail->SetConstraintReferencePosition(EConstraintFrame::Frame2, FVector::ZeroVector);
    ColorPart(Mast, FLinearColor(0.06f, 0.10f, 0.15f));
    ColorPart(Carriage, FLinearColor(0.95f, 0.45f, 0.04f));
    ColorPart(LeftFork, FLinearColor(0.1f, 0.15f, 0.22f));
    ColorPart(RightFork, FLinearColor(0.1f, 0.15f, 0.22f));
}

void ALiftBench::BuildPallet()
{
    if (IsValid(LoadActor)) LoadActor->Destroy();
    LoadActor = GetWorld()->SpawnActor<AActor>();
    UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    Pallet = NewObject<UStaticMeshComponent>(LoadActor, TEXT("PalletDeck"));
    LoadActor->SetRootComponent(Pallet);
    Pallet->SetStaticMesh(Cube);
    Pallet->SetWorldScale3D(FVector(1.2, 1.2, 0.08));
    Pallet->SetCollisionProfileName(TEXT("PhysicsActor"));
    Pallet->SetPhysMaterialOverride(ContactMaterial);
    Pallet->RegisterComponent();
    InitialPalletPosition = GetActorLocation() + FVector(80, 0, 52);
    Pallet->SetWorldLocation(InitialPalletPosition);
    Pallet->BodyInstance.SetMassOverride(Settings.PalletMassKg * 0.4, true);
    Pallet->SetSimulatePhysics(true);
    Pallet->SetLinearDamping(0.05);
    Pallet->SetAngularDamping(0.1);
    Pallet->BodyInstance.bUseCCD = true;
    ColorPart(Pallet, FLinearColor(0.45f, 0.25f, 0.10f));
    auto AddBlock = [&](const TCHAR* Name, FVector Position, FVector Size, double MassKg, FLinearColor Color)
    {
        UStaticMeshComponent* Block = NewObject<UStaticMeshComponent>(LoadActor, Name);
        Block->SetStaticMesh(Cube);
        Block->SetCollisionProfileName(TEXT("PhysicsActor"));
        Block->SetPhysMaterialOverride(ContactMaterial);
        Block->BodyInstance.SetMassOverride(MassKg, true);
        Block->RegisterComponent();
        Block->SetWorldLocation(GetActorLocation() + Position);
        Block->SetWorldScale3D(Size / 100.0);
        Block->AttachToComponent(Pallet, FAttachmentTransformRules(EAttachmentRule::KeepWorld, true));
        ColorPart(Block, Color);
    };
    for (int32 I = 0; I < 3; ++I)
    {
        const double Y = (I - 1) * 52.0;
        AddBlock(*FString::Printf(TEXT("Runner%d"), I), FVector(80, Y, 28),
                 FVector(120, 12, 40), Settings.PalletMassKg * 0.15, FLinearColor(0.35f, 0.18f, 0.07f));
        AddBlock(*FString::Printf(TEXT("Bottom%d"), I), FVector(80, Y, 4),
                 FVector(120, 14, 8), Settings.PalletMassKg * 0.05, FLinearColor(0.45f, 0.25f, 0.10f));
    }
    AddBlock(TEXT("Payload"), FVector(80, 0, 86), FVector(100, 100, 60), Settings.PayloadMassKg,
             FLinearColor(0.05f, 0.45f, 0.55f));
    // Recompute the sum of deck, runners, bottom boards and payload masses.
    Pallet->BodyInstance.UpdateMassProperties();
}

void ALiftBench::ResetBench()
{
    FlushTelemetry();
    Rail->BreakConstraint();
    Carriage->SetSimulatePhysics(false);
    Carriage->SetWorldLocationAndRotation(InitialCarriagePosition, FRotator::ZeroRotator,
                                        false, nullptr, ETeleportType::TeleportPhysics);
    ContactMaterial->Friction = Settings.ContactFriction;
    ContactMaterial->StaticFriction = Settings.ContactFriction;
    Carriage->SetSimulatePhysics(true);
    Carriage->SetPhysicsLinearVelocity(FVector::ZeroVector);
    Carriage->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector);
    Carriage->SetMassOverrideInKg(NAME_None, Settings.CarriageMassKg * 0.6);
    Rail->SetConstrainedComponents(Mast, NAME_None, Carriage, NAME_None);
    Rail->SetConstraintReferencePosition(EConstraintFrame::Frame2, FVector::ZeroVector);
    BuildPallet();
    Controller.Reset();
    Sensor = FLiftSensorReading();
    SensorQueue.Empty();
    CsvRows.Empty();
    CsvRows.Add(TEXT("time_s,phase,target_height_m,true_height_m,sensed_height_m,sensor_timestamp_s,true_velocity_mps,true_acceleration_mps2,command_velocity_mps,actuator_force_n,guide_resistance_n,payload_kg,contact_friction,pallet_rise_m"));
    const FString Dir = FPaths::ProjectSavedDir() / TEXT("Telemetry");
    IFileManager::Get().MakeDirectory(*Dir, true);
    TelemetryPath = Dir / FString::Printf(TEXT("Lift_%s_%s.csv"),
        *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")), *FGuid::NewGuid().ToString(EGuidFormats::Digits).Left(6));
    Phase = ELiftLabPhase::Ready;
    FaultReason.Empty();
    SimTimeS = ActualHeightM = ActualVelocityMps = ActualAccelerationMps2 = 0;
    TargetHeightM = AppliedLiftForceN = GuideResistanceN = PalletRiseM = MaxPalletRiseM = 0;
    LastSensorSampleS = LastCsvSampleS = -1;
    LastActualVelocityMps = HoldTimeS = StallTimeS = MaxPalletDriftM = 0;
    bSensorFailure = false;
    bTestFinished = false;
    bCaptureRequested = false;
    const bool bMassValid = FMath::Abs(Carriage->GetMass() - Settings.CarriageMassKg) < 0.05
        && FMath::Abs(Pallet->GetMass() - Settings.PalletMassKg - Settings.PayloadMassKg) < 0.05;
    UE_LOG(LogForkliftLab, Display, TEXT("MASS_CHECK %s carriage=%.3f kg load=%.3f kg"),
           bMassValid ? TEXT("PASS") : TEXT("FAIL"), Carriage->GetMass(), Pallet->GetMass());
    if (!bMassValid) EnterFault(TEXT("Welded body mass mismatch"));
    UE_LOG(LogForkliftLab, Display, TEXT("RESET payload=%.1f kg, force_limit=%.1f N, friction=%.2f; CSV=%s"),
           Settings.PayloadMassKg, Settings.MaxLiftForceN, Settings.ContactFriction, *TelemetryPath);
    FString ConfigText;
    FFileHelper::LoadFileToString(ConfigText, *(FPaths::ProjectConfigDir() / TEXT("ForkliftLab.json")));
    const FString Metadata = FString::Printf(
        TEXT("engine=%s\nscenario=%s\nactive_payload_kg=%.3f\nactive_force_limit_n=%.3f\nactive_friction=%.3f\nactual_carriage_mass_kg=%.3f\nactual_load_mass_kg=%.3f\n\nBase configuration (active overrides above take precedence):\n%s\n"),
        *FEngineVersion::Current().ToString(), *TestMode, Settings.PayloadMassKg,
        Settings.MaxLiftForceN, Settings.ContactFriction, Carriage->GetMass(), Pallet->GetMass(), *ConfigText);
    FFileHelper::SaveStringToFile(Metadata, *(TelemetryPath + TEXT(".meta.txt")),
        FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}

void ALiftBench::StartCycle()
{
    if (Phase == ELiftLabPhase::Ready || Phase == ELiftLabPhase::Complete)
    {
        Phase = ELiftLabPhase::Raising;
        TargetHeightM = Settings.LiftTargetM;
        HoldTimeS = StallTimeS = 0;
    }
}

void ALiftBench::CyclePayload()
{
    Settings.PayloadMassKg = Settings.PayloadMassKg < 250 ? 250 : Settings.PayloadMassKg < 500 ? 500 : Settings.PayloadMassKg < 1000 ? 1000 : 100;
    ResetBench();
}
void ALiftBench::CycleFriction()
{
    Settings.ContactFriction = Settings.ContactFriction < 0.5 ? 0.6 : Settings.ContactFriction < 0.8 ? 0.9 : 0.2;
    ResetBench();
}
void ALiftBench::CycleForce()
{
    Settings.MaxLiftForceN = Settings.MaxLiftForceN < 6000 ? 6000 : Settings.MaxLiftForceN < 14000 ? 14000 : 1500;
    ResetBench();
}
void ALiftBench::ToggleSensorFailure() { bSensorFailure = !bSensorFailure; }

const TCHAR* ALiftBench::PhaseLabel() const
{
    switch (Phase)
    {
    case ELiftLabPhase::Ready: return TEXT("READY");
    case ELiftLabPhase::Raising: return TEXT("RAISING");
    case ELiftLabPhase::Holding: return TEXT("HOLDING");
    case ELiftLabPhase::Lowering: return TEXT("LOWERING");
    case ELiftLabPhase::Complete: return TEXT("COMPLETE");
    default: return TEXT("FAULT / LOCKED");
    }
}

void ALiftBench::SampleSensors()
{
    if (!bSensorFailure && SimTimeS - LastSensorSampleS >= 1.0 / Settings.SensorRateHz)
    {
        FLiftSensorReading R;
        R.TimestampS = SimTimeS;
        R.HeightM = FMath::RoundToDouble(ActualHeightM / Settings.SensorResolutionM) * Settings.SensorResolutionM;
        R.VelocityMps = ActualVelocityMps;
        R.bValid = true;
        SensorQueue.Add(R);
        LastSensorSampleS = SimTimeS;
    }
    while (SensorQueue.Num() && SimTimeS - SensorQueue[0].TimestampS >= Settings.SensorDelayS)
    {
        Sensor = SensorQueue[0];
        SensorQueue.RemoveAt(0);
    }
}

void ALiftBench::EnterFault(const FString& Reason)
{
    if (Phase == ELiftLabPhase::Fault) return;
    Phase = ELiftLabPhase::Fault;
    FaultReason = Reason;
    AppliedLiftForceN = 0;
    // Explicit simplification: ideal mechanical holding lock on a stationary bench.
    // This is not a hydraulic brake model or a safety-certified implementation.
    Carriage->SetSimulatePhysics(false);
    UE_LOG(LogForkliftLab, Warning, TEXT("FAULT: %s"), *Reason);
}

void ALiftBench::UpdateMission(double Dt)
{
    if (Phase == ELiftLabPhase::Fault || Phase == ELiftLabPhase::Ready || Phase == ELiftLabPhase::Complete) return;
    if (!Sensor.bValid || SimTimeS - Sensor.TimestampS > Settings.SensorTimeoutS)
    {
        EnterFault(TEXT("Height sensor timeout"));
        return;
    }
    const bool bAtTarget = FMath::Abs(Sensor.HeightM - TargetHeightM) < 0.02 && FMath::Abs(Sensor.VelocityMps) < 0.035;
    HoldTimeS = bAtTarget ? HoldTimeS + Dt : 0;
    if (Phase == ELiftLabPhase::Raising && HoldTimeS > 0.4)
    {
        Phase = ELiftLabPhase::Holding;
        HoldTimeS = 0;
    }
    else if (Phase == ELiftLabPhase::Holding && HoldTimeS > 2.0)
    {
        Phase = ELiftLabPhase::Lowering;
        TargetHeightM = 0;
        HoldTimeS = 0;
    }
    else if (Phase == ELiftLabPhase::Lowering && HoldTimeS > 0.5)
    {
        Phase = ELiftLabPhase::Complete;
        FlushTelemetry();
    }
    const bool bStalled = Phase == ELiftLabPhase::Raising && !bAtTarget
        && AppliedLiftForceN >= Settings.MaxLiftForceN * 0.99 && FMath::Abs(Sensor.VelocityMps) < 0.01;
    StallTimeS = bStalled ? StallTimeS + Dt : 0;
    if (StallTimeS > 2.0) EnterFault(TEXT("Lift stalled at force limit"));
}

void ALiftBench::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    const double Dt = FMath::Max(double(DeltaSeconds), 0.000001);
    SimTimeS += Dt;
    ActualHeightM = (Carriage->GetComponentLocation().Z - InitialCarriagePosition.Z) / 100.0;
    ActualVelocityMps = Carriage->GetPhysicsLinearVelocity().Z / 100.0;
    ActualAccelerationMps2 = (ActualVelocityMps - LastActualVelocityMps) / Dt;
    LastActualVelocityMps = ActualVelocityMps;
    PalletRiseM = (Pallet->GetComponentLocation().Z - InitialPalletPosition.Z) / 100.0;
    MaxPalletRiseM = FMath::Max(MaxPalletRiseM, PalletRiseM);
    MaxPalletDriftM = FMath::Max(MaxPalletDriftM, FVector::DistXY(Pallet->GetComponentLocation(), InitialPalletPosition) / 100.0);
    if (TestMode == TEXT("sensor_timeout") && SimTimeS > 2.0) bSensorFailure = true;
    SampleSensors();
    if ((!TestMode.IsEmpty() || bCaptureMode) && Phase == ELiftLabPhase::Ready && SimTimeS > 1.0) StartCycle();
    UpdateMission(Dt);
    if (Phase != ELiftLabPhase::Fault)
    {
        AppliedLiftForceN = Controller.Step(Settings, TargetHeightM, Sensor, Dt);
        GuideResistanceN = Settings.GuideFrictionN * FMath::Clamp(ActualVelocityMps / 0.01, -1.0, 1.0)
            + Settings.GuideDampingNsPerM * ActualVelocityMps;
        Carriage->AddForce(FVector(0, 0, ForkliftUnits::NewtonToUnreal(AppliedLiftForceN - GuideResistanceN)));
    }
    if (SimTimeS - LastCsvSampleS >= 0.1)
    {
        CsvRows.Add(FString::Printf(TEXT("%.5f,%s,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.1f,%.3f,%.5f"),
            SimTimeS, PhaseLabel(), TargetHeightM, ActualHeightM, Sensor.HeightM, Sensor.TimestampS,
            ActualVelocityMps, ActualAccelerationMps2, Controller.GetCommandVelocityMps(),
            AppliedLiftForceN, GuideResistanceN, Settings.PayloadMassKg, Settings.ContactFriction, PalletRiseM));
        LastCsvSampleS = SimTimeS;
    }
    EvaluateSmokeTest();
    if (bCaptureMode && !bCaptureRequested && SimTimeS > 18.0)
    {
        const FString CaptureDir = FPaths::ProjectSavedDir() / TEXT("Screenshots");
        IFileManager::Get().MakeDirectory(*CaptureDir, true);
        FScreenshotRequest::RequestScreenshot(CaptureDir / TEXT("LiftLab.png"), true, false);
        bCaptureRequested = true;
    }
    if (bCaptureMode && SimTimeS > 20.0) FPlatformMisc::RequestExit(false);
}

void ALiftBench::EvaluateSmokeTest()
{
    if (TestMode.IsEmpty() || bTestFinished) return;
    const bool bTimedOut = SimTimeS > 75;
    if (Phase != ELiftLabPhase::Complete && Phase != ELiftLabPhase::Fault && !bTimedOut) return;
    bool bPassed = false;
    if (TestMode == TEXT("baseline"))
        bPassed = Phase == ELiftLabPhase::Complete && MaxPalletRiseM > 0.35
            && FMath::Abs(PalletRiseM) < 0.03 && MaxPalletDriftM < 0.1;
    else if (TestMode == TEXT("overload"))
        bPassed = Phase == ELiftLabPhase::Fault && FaultReason.Contains(TEXT("force limit")) && MaxPalletRiseM < 0.05;
    else if (TestMode == TEXT("sensor_timeout"))
        bPassed = Phase == ELiftLabPhase::Fault && FaultReason.Contains(TEXT("sensor timeout")) && AppliedLiftForceN == 0;
    bTestFinished = true;
    FlushTelemetry();
    UE_LOG(LogForkliftLab, Display, TEXT("FORKLIFT_TEST %s %s time=%.2f max_pallet_rise=%.4f final_rise=%.4f max_drift=%.4f phase=%s"),
           *TestMode, bPassed ? TEXT("PASS") : TEXT("FAIL"), SimTimeS, MaxPalletRiseM, PalletRiseM, MaxPalletDriftM, PhaseLabel());
    FPlatformMisc::RequestExitWithStatus(false, bPassed ? 0 : 1);
}

void ALiftBench::FlushTelemetry()
{
    if (!TelemetryPath.IsEmpty() && CsvRows.Num() > 1)
        FFileHelper::SaveStringToFile(FString::Join(CsvRows, TEXT("\n")) + TEXT("\n"), *TelemetryPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}

void ALiftBench::EndPlay(const EEndPlayReason::Type Reason)
{
    FlushTelemetry();
    if (IsValid(LoadActor)) LoadActor->Destroy();
    Super::EndPlay(Reason);
}
