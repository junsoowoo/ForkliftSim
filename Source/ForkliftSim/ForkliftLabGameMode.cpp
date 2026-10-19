#include "ForkliftLabGameMode.h"
#include "LiftBench.h"
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/InputComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Canvas.h"
#include "Engine/DirectionalLight.h"
#include "Engine/Engine.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "InputCoreTypes.h"
#include "Materials/MaterialInstanceDynamic.h"

namespace
{
    ALiftBench* FindBench(UWorld* World)
    {
        for (TActorIterator<ALiftBench> It(World); It; ++It) return *It;
        return nullptr;
    }
}

AForkliftLabGameMode::AForkliftLabGameMode()
{
    PlayerControllerClass = AForkliftLabController::StaticClass();
    HUDClass = AForkliftLabHUD::StaticClass();
    DefaultPawnClass = nullptr;
}

void AForkliftLabGameMode::BeginPlay()
{
    Super::BeginPlay();
    auto* Floor = GetWorld()->SpawnActor<AStaticMeshActor>(FVector(0, 0, -10), FRotator::ZeroRotator);
    Floor->GetStaticMeshComponent()->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")));
    Floor->GetStaticMeshComponent()->SetWorldScale3D(FVector(12, 12, 0.2));
    Floor->GetStaticMeshComponent()->SetCollisionProfileName(TEXT("BlockAll"));
    if (auto* Material = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Materials/M_Lab.M_Lab")))
    {
        auto* Instance = UMaterialInstanceDynamic::Create(Material, Floor);
        Instance->SetVectorParameterValue(TEXT("Tint"), FLinearColor(0.14f, 0.18f, 0.24f));
        Floor->GetStaticMeshComponent()->SetMaterial(0, Instance);
    }
    auto* Light = GetWorld()->SpawnActor<ADirectionalLight>(FVector(0, 0, 400), FRotator(-55, -35, 0));
    Light->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    Cast<UDirectionalLightComponent>(Light->GetLightComponent())->SetForwardShadingPriority(1);
    Light->GetLightComponent()->SetIntensity(4.0f);
    auto* Fill = GetWorld()->SpawnActor<ADirectionalLight>(FVector(0, 0, 300), FRotator(-30, 145, 0));
    Fill->GetLightComponent()->SetMobility(EComponentMobility::Movable);
    Fill->GetLightComponent()->SetIntensity(1.5f);
    GetWorld()->SpawnActor<ALiftBench>();
}

void AForkliftLabController::BeginPlay()
{
    Super::BeginPlay();
    LabCamera = GetWorld()->SpawnActor<ACameraActor>();
    LabCamera->GetCameraComponent()->SetFieldOfView(50);
    ChangeCamera();
    SetViewTarget(LabCamera);
    SetInputMode(FInputModeGameOnly());
    bShowMouseCursor = false;
}

void AForkliftLabController::SetupInputComponent()
{
    Super::SetupInputComponent();
    InputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this, &AForkliftLabController::StartCycle);
    InputComponent->BindKey(EKeys::R, IE_Pressed, this, &AForkliftLabController::Reset);
    InputComponent->BindKey(EKeys::M, IE_Pressed, this, &AForkliftLabController::ChangeMass);
    InputComponent->BindKey(EKeys::F, IE_Pressed, this, &AForkliftLabController::ChangeFriction);
    InputComponent->BindKey(EKeys::T, IE_Pressed, this, &AForkliftLabController::ChangeForce);
    InputComponent->BindKey(EKeys::X, IE_Pressed, this, &AForkliftLabController::FailSensor);
    InputComponent->BindKey(EKeys::C, IE_Pressed, this, &AForkliftLabController::ChangeCamera);
}
void AForkliftLabController::StartCycle() { if (auto* B = FindBench(GetWorld())) B->StartCycle(); }
void AForkliftLabController::Reset() { if (auto* B = FindBench(GetWorld())) B->ResetBench(); }
void AForkliftLabController::ChangeMass() { if (auto* B = FindBench(GetWorld())) B->CyclePayload(); }
void AForkliftLabController::ChangeFriction() { if (auto* B = FindBench(GetWorld())) B->CycleFriction(); }
void AForkliftLabController::ChangeForce() { if (auto* B = FindBench(GetWorld())) B->CycleForce(); }
void AForkliftLabController::FailSensor() { if (auto* B = FindBench(GetWorld())) B->ToggleSensorFailure(); }
void AForkliftLabController::ChangeCamera()
{
    if (!LabCamera) return;
    bSideView = !bSideView;
    const FVector Position = bSideView ? FVector(600, -720, 440) : FVector(65, -850, 230);
    LabCamera->SetActorLocation(Position);
    LabCamera->SetActorRotation((FVector(25, 60, 100) - Position).Rotation());
}

void AForkliftLabHUD::DrawHUD()
{
    Super::DrawHUD();
    ALiftBench* B = FindBench(GetWorld());
    if (!Canvas || !B) return;
    const FLinearColor Ink(0.9f, 0.94f, 1.f);
    const FLinearColor Muted(0.55f, 0.66f, 0.76f);
    const FLinearColor Accent(0.15f, 0.85f, 0.8f);
    // Keep the full instrument panel visible on scaled laptop displays.
    const float UiScale = FMath::Min(1.f, FMath::Min(Canvas->SizeX / 1000.f, Canvas->SizeY / 650.f));
    const float LayoutWidth = Canvas->SizeX / UiScale;
    const float LayoutHeight = Canvas->SizeY / UiScale;
    auto Rect = [&](FLinearColor Color, float X, float Y, float W, float H)
    {
        DrawRect(Color, X * UiScale, Y * UiScale, W * UiScale, H * UiScale);
    };
    Rect(FLinearColor(0.02f, 0.035f, 0.06f, 0.94f), 16, 16, 360, 490);
    auto Text = [&](const FString& Value, float X, float Y, FLinearColor Color, float Scale = 1.f)
    {
        DrawText(Value, Color, X * UiScale, Y * UiScale, GEngine->GetSmallFont(), Scale * UiScale, false);
    };
    Text(TEXT("FORKLIFT / PHYSICS LAB"), 32, 30, Accent, 1.5f);
    Text(TEXT("01  STATIONARY LIFT BENCH"), 32, 62, Muted);
    Text(B->PhaseLabel(), 32, 96, B->Phase == ELiftLabPhase::Fault ? FLinearColor(1, .35f, .2f) : Accent, 1.4f);
    Text(FString::Printf(TEXT("Payload          %7.1f kg"), B->Settings.PayloadMassKg), 32, 140, Ink);
    Text(FString::Printf(TEXT("Pallet / carriage  %.0f / %.0f kg"), B->Settings.PalletMassKg, B->Settings.CarriageMassKg), 32, 162, Muted);
    Text(FString::Printf(TEXT("Force limit      %7.0f N"), B->Settings.MaxLiftForceN), 32, 194, Ink);
    Text(FString::Printf(TEXT("Applied force    %7.0f N"), B->AppliedLiftForceN), 32, 216, Ink);
    Rect(FLinearColor(.12f, .18f, .24f), 32, 244, 320, 5);
    Rect(Accent, 32, 244, 320 * FMath::Clamp(B->AppliedLiftForceN / B->Settings.MaxLiftForceN, 0.0, 1.0), 5);
    Text(FString::Printf(TEXT("Target / actual  %.3f / %.3f m"), B->TargetHeightM, B->ActualHeightM), 32, 266, Ink);
    Text(FString::Printf(TEXT("Sensor height    %.3f m"), B->Sensor.HeightM), 32, 288, Ink);
    Text(FString::Printf(TEXT("Actual speed     %+.3f m/s"), B->ActualVelocityMps), 32, 310, Ink);
    Text(FString::Printf(TEXT("Acceleration     %+.3f m/s2"), B->ActualAccelerationMps2), 32, 332, Ink);
    Text(FString::Printf(TEXT("Pallet rise      %.3f m"), B->PalletRiseM), 32, 354, Ink);
    Text(FString::Printf(TEXT("Contact friction %.2f"), B->Settings.ContactFriction), 32, 386, Muted);
    Text(FString::Printf(TEXT("Sensor %.0f Hz / %.0f ms delay"), B->Settings.SensorRateHz, B->Settings.SensorDelayS * 1000), 32, 408, Muted);
    Text(FString::Printf(TEXT("Simulation time  %.1f s"), B->SimTimeS), 32, 430, Muted);
    Text(TEXT("SI units | CSV saved on reset / exit"), 32, 470, Accent);
    Rect(FLinearColor(0.02f, 0.035f, 0.06f, 0.94f), 16, LayoutHeight - 76, LayoutWidth - 32, 60);
    Text(TEXT("SPACE Start cycle    R Reset    M Payload    T Force limit    F Friction    X Sensor fault    C Camera"), 32, LayoutHeight - 58, Ink);
    Text(TEXT("Milestone 1: contact physics + feedback lift. Wheel drive and autonomous pickup are next."), 32, LayoutHeight - 36, Muted);
    if (!B->FaultReason.IsEmpty())
        Text(B->FaultReason + TEXT("  |  R to reset"), 400, 40, FLinearColor(1, .35f, .2f), 1.3f);
}
