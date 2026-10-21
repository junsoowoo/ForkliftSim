#include "WheelLab.h"
#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InputComponent.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "HAL/FileManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/CommandLine.h"
#include "Misc/EngineVersion.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "UnrealClient.h"

DEFINE_LOG_CATEGORY_STATIC(LogWheelLab, Log, All);
namespace
{
    constexpr double FixedStep = 1. / 240.;
    AWheelLab* FindWheels(UWorld* World)
    { for (TActorIterator<AWheelLab> It(World); It; ++It) return *It; return nullptr; }
    UStaticMeshComponent* Part(AActor* Owner, USceneComponent* Parent, FVector PositionM, FVector SizeM,
        FLinearColor Color, bool Cylinder = false)
    {
        auto* Mesh = NewObject<UStaticMeshComponent>(Owner);
        Mesh->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, Cylinder ? TEXT("/Engine/BasicShapes/Cylinder.Cylinder") : TEXT("/Engine/BasicShapes/Cube.Cube")));
        Mesh->SetMobility(EComponentMobility::Movable);
        Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision); // Rendering only: planar solver owns the dynamics.
        Mesh->SetupAttachment(Parent); Mesh->SetRelativeLocation(PositionM * 100); Mesh->SetRelativeScale3D(SizeM);
        Mesh->RegisterComponent();
        if (auto* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Game/Materials/M_Lab.M_Lab")))
        {
            auto* Material = UMaterialInstanceDynamic::Create(Base, Mesh);
            Material->SetVectorParameterValue(TEXT("Tint"), Color); Mesh->SetMaterial(0, Material);
        }
        return Mesh;
    }
}

AWheelLab::AWheelLab()
{
    PrimaryActorTick.bCanEverTick = true;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("VehicleCOM"));
}

void AWheelLab::BeginPlay()
{
    Super::BeginPlay();
    FParse::Value(FCommandLine::Get(), TEXT("WheelTest="), TestMode);
    bCapture = FParse::Param(FCommandLine::Get(), TEXT("WheelCapture"));
    if (!Model.Settings.Load(Error))
    {
        UE_LOG(LogWheelLab, Error, TEXT("%s"), *Error); SetActorTickEnabled(false);
        if (!TestMode.IsEmpty()) FPlatformMisc::RequestExitWithStatus(false, 2);
        return;
    }
    if (TestMode == TEXT("low_grip")) Model.Settings.Friction = .08;
    BuildVisuals(); ResetLab();
}

void AWheelLab::BuildVisuals()
{
    const auto& S = Model.Settings;
    const FLinearColor Yellow(.95f, .45f, .035f), Dark(.045f, .065f, .09f), Metal(.18f, .25f, .3f);
    Part(this, RootComponent, FVector(-S.ComForwardM, 0, .65), FVector(2.2, .94, .45), Yellow);
    Part(this, RootComponent, FVector(-.85-S.ComForwardM, 0, 1), FVector(.55, .95, .4), Yellow);
    Part(this, RootComponent, FVector(.1-S.ComForwardM, 0, 1), FVector(.75, .75, .22), Dark);
    for (double Y : {-.39, .39}) Part(this, RootComponent, FVector(.92-S.ComForwardM, Y, 1.35), FVector(.14, .12, 2.5), Dark);
    Part(this, RootComponent, FVector(.92-S.ComForwardM, 0, 2.55), FVector(.14, .9, .12), Dark);
    Part(this, RootComponent, FVector(1.05-S.ComForwardM, 0, .6), FVector(.15, .9, .5), Yellow);
    for (double Y : {-.3, .3}) Part(this, RootComponent, FVector(1.8-S.ComForwardM, Y, .23), FVector(1.4, .1, .06), Metal);
    Model.Reset();
    for (int I = 0; I < 4; ++I)
    {
        auto* Steer = NewObject<USceneComponent>(this); Steer->SetupAttachment(RootComponent); Steer->RegisterComponent();
        Steer->SetRelativeLocation(FVector(Model.Wheels[I].LocalPosition.X, Model.Wheels[I].LocalPosition.Y, S.WheelRadiusM) * 100);
        auto* Spin = NewObject<USceneComponent>(this); Spin->SetupAttachment(Steer); Spin->RegisterComponent();
        auto* Tyre = Part(this, Spin, FVector::ZeroVector, FVector(2*S.WheelRadiusM, 2*S.WheelRadiusM, S.WheelWidthM), Dark, true);
        Tyre->SetRelativeRotation(FRotator(0, 0, 90));
        const double OutsideY = (I%2 == 0 ? -1 : 1) * (S.WheelWidthM * .5 + .005);
        Part(this, Spin, FVector(0, OutsideY, 0), FVector(S.WheelRadiusM*1.6, .015, .035), FLinearColor(.75f, .85f, .9f));
        SteerPivots.Add(Steer); SpinPivots.Add(Spin);
    }
    // Static visual reference marks every 2 metres; these are not collision objects or sensors.
    AActor* Markers = GetWorld()->SpawnActor<AActor>();
    auto* MarkRoot = NewObject<USceneComponent>(Markers); Markers->SetRootComponent(MarkRoot); MarkRoot->RegisterComponent();
    for (int I = -25; I <= 25; ++I)
    {
        Part(Markers, MarkRoot, FVector(I * 2, 0, .008), FVector(.025, 40, .01), FLinearColor(.25f, .32f, .4f));
        Part(Markers, MarkRoot, FVector(0, I * 2, .009), FVector(100, .025, .01), FLinearColor(.25f, .32f, .4f));
    }
}

void AWheelLab::ResetLab()
{
    FlushTelemetry(); Model.Reset(); Throttle = Brake = Steering = AccelerationMps2 = 0;
    Accumulator = NextRecord = MaximumSpeed = MaxSlip = BrakeStartX = 0;
    StopDistanceM = -1; NextFlush = 5; bBraking = bFinished = bCaptured = false;
    Rows.Empty();
    Rows.Add(TEXT("time_s,x_m,y_m,yaw_rad,speed_mps,accel_mps2,throttle,brake,steer_input,mu,mass_kg,wheel,omega_rad_s,rim_speed_mps,slip_speed_mps,slip_ratio,normal_n,long_force_n,lateral_force_n,drive_nm,steer_rad,kinetic_energy_j"));
    const FString Dir = FPaths::ProjectSavedDir() / TEXT("Telemetry"); IFileManager::Get().MakeDirectory(*Dir, true);
    CsvPath = Dir / FString::Printf(TEXT("Wheels_%s_%s.csv"), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")), *FGuid::NewGuid().ToString().Left(6));
    FString BaseConfig; FFileHelper::LoadFileToString(BaseConfig, *(FPaths::ProjectConfigDir() / TEXT("WheelLab.json")));
    const FString Meta = FString::Printf(TEXT("engine=%s\nmodel=flat-floor planar impulse solver\nstep_s=%.9f\niterations=64\nrelaxation=0.5\nscenario=%s\nactive_mu=%.3f\nactive_mass_kg=%.3f\nactive_yaw_inertia_kgm2=%.3f\nactive_drive_nm=%.3f\nspin_inertia_kgm2=%.6f\nOverrides above take precedence over base JSON:\n%s"),
        *FEngineVersion::Current().ToString(), FixedStep, *TestMode, Model.Settings.Friction, Model.Settings.VehicleMassKg,
        Model.Settings.YawInertiaKgM2, Model.Settings.DriveTorqueNm, Model.Settings.WheelInertia(), *BaseConfig);
    FFileHelper::SaveStringToFile(Meta, *(CsvPath + TEXT(".meta.txt")), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    SetActorLocationAndRotation(FVector::ZeroVector, FRotator::ZeroRotator);
}
void AWheelLab::CycleFriction() { Model.Settings.Friction = Model.Settings.Friction > .5 ? .25 : Model.Settings.Friction > .1 ? .08 : .8; ResetLab(); }
void AWheelLab::CycleMass()
{
    const double Old = Model.Settings.VehicleMassKg;
    Model.Settings.VehicleMassKg = Old < 2200 ? 2200 : Old < 3200 ? 3200 : 1500;
    Model.Settings.YawInertiaKgM2 *= Model.Settings.VehicleMassKg / Old; ResetLab();
}
void AWheelLab::CycleTorque() { Model.Settings.DriveTorqueNm = Model.Settings.DriveTorqueNm < 650 ? 650 : Model.Settings.DriveTorqueNm < 1000 ? 1000 : 300; ResetLab(); }

void AWheelLab::Tick(float Dt)
{
    Super::Tick(Dt);
    // A long render stall slows simulation time instead of using an unstable large physics step.
    Accumulator += FMath::Clamp(double(Dt), 0., .25);
    while (Accumulator + 1e-10 >= FixedStep)
    {
        if (!TestMode.IsEmpty() || bCapture)
        {
            Throttle = Model.TimeS < 3 ? 1 : 0; Brake = Model.TimeS >= 3 ? 1 : 0;
            Steering = TestMode == TEXT("steering") ? .5 : 0;
            if (TestMode == TEXT("steering")) { Throttle = .25; Brake = Model.TimeS >= 5 ? 1 : 0; }
        }
        if (Brake > .5 && !bBraking) { bBraking = true; BrakeStartX = Model.PositionM.X; }
        const double PreviousSpeed = Model.ForwardSpeed();
        Model.Step(Throttle, Brake, Steering, FixedStep);
        AccelerationMps2 = (Model.ForwardSpeed() - PreviousSpeed) / FixedStep;
        MaximumSpeed = FMath::Max(MaximumSpeed, Model.VelocityMps.Size());
        MaxSlip = FMath::Max(MaxSlip, FMath::Abs(Model.Wheels[0].SlipSpeedMps));
        if (bBraking && StopDistanceM < 0 && Model.VelocityMps.Size() < .01) StopDistanceM = Model.PositionM.X - BrakeStartX;
        if (Model.TimeS >= NextRecord) { Record(); NextRecord += .1; }
        if (Model.TimeS >= NextFlush) { FlushTelemetry(); NextFlush += 5; }
        Accumulator -= FixedStep;
    }
    SetActorLocationAndRotation(FVector(Model.PositionM.X * 100, Model.PositionM.Y * 100, 0), FRotator(0, FMath::RadiansToDegrees(Model.YawRad), 0));
    for (int I = 0; I < 4; ++I)
    {
        SteerPivots[I]->SetRelativeRotation(FRotator(0, FMath::RadiansToDegrees(Model.Wheels[I].SteerRad), 0));
        SpinPivots[I]->SetRelativeRotation(FQuat(FVector::RightVector, Model.Wheels[I].AngleRad));
    }
    CheckTest();
    if (bCapture && !bCaptured && Model.TimeS > 2.5)
    {
        const FString Dir = FPaths::ProjectSavedDir() / TEXT("Screenshots"); IFileManager::Get().MakeDirectory(*Dir, true);
        FScreenshotRequest::RequestScreenshot(Dir / TEXT("WheelLab.png"), true, false); bCaptured = true;
    }
    if (bCapture && Model.TimeS > 4) FPlatformMisc::RequestExit(false);
}

void AWheelLab::Record()
{
    for (int I = 0; I < 4; ++I)
    {
        const auto& W = Model.Wheels[I];
        Rows.Add(FString::Printf(TEXT("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.1f,%d,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f"),
            Model.TimeS, Model.PositionM.X, Model.PositionM.Y, Model.YawRad, Model.ForwardSpeed(), AccelerationMps2,
            Throttle, Brake, Steering, Model.Settings.Friction, Model.Settings.VehicleMassKg, I,
            W.OmegaRadS, W.OmegaRadS * Model.Settings.WheelRadiusM, W.SlipSpeedMps, W.SlipRatio,
            W.NormalForceN, W.LongForceN, W.LateralForceN, W.AppliedDriveNm, W.SteerRad, Model.KineticEnergyJ()));
    }
}
void AWheelLab::FlushTelemetry()
{
    if (!CsvPath.IsEmpty() && Rows.Num() > 1)
        FFileHelper::SaveStringToFile(FString::Join(Rows, TEXT("\n")) + TEXT("\n"), *CsvPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}
void AWheelLab::EndPlay(const EEndPlayReason::Type Reason) { FlushTelemetry(); Super::EndPlay(Reason); }

void AWheelLab::CheckTest()
{
    if (TestMode.IsEmpty() || bFinished || Model.TimeS < 10) return;
    bool Passed = Model.VelocityMps.Size() < .02 && FMath::IsFinite(Model.KineticEnergyJ());
    if (TestMode == TEXT("baseline")) Passed &= MaximumSpeed > 3 && StopDistanceM > .1 && StopDistanceM < 5 && MaxSlip < .1;
    else if (TestMode == TEXT("low_grip")) Passed &= MaximumSpeed > .3 && MaximumSpeed < 2 && MaxSlip > 2 && StopDistanceM > 0;
    else if (TestMode == TEXT("steering")) Passed &= FMath::Abs(Model.PositionM.Y) > .5 && FMath::Abs(Model.YawRad) > .2;
    else Passed = false;
    bFinished = true; FlushTelemetry();
    UE_LOG(LogWheelLab, Display, TEXT("WHEEL_TEST %s %s max_speed=%.4f m/s stop_dx=%.4f m max_slip=%.4f m/s x=%.4f y=%.4f yaw=%.4f rad"),
        *TestMode, Passed ? TEXT("PASS") : TEXT("FAIL"), MaximumSpeed, StopDistanceM, MaxSlip, Model.PositionM.X, Model.PositionM.Y, Model.YawRad);
    FPlatformMisc::RequestExitWithStatus(false, Passed ? 0 : 1);
}

void AWheelLabController::BeginPlay()
{
    Super::BeginPlay(); LabCamera = GetWorld()->SpawnActor<ACameraActor>(); LabCamera->GetCameraComponent()->SetFieldOfView(55);
    SetViewTarget(LabCamera); SetInputMode(FInputModeGameOnly()); bShowMouseCursor = false;
}
void AWheelLabController::PlayerTick(float Dt)
{
    Super::PlayerTick(Dt); auto* B = FindWheels(GetWorld()); if (!B || !LabCamera) return;
    // Poll keys every frame: focus loss cannot leave a cached throttle command stuck on.
    B->Throttle = (IsInputKeyDown(EKeys::W) ? 1 : 0) - (IsInputKeyDown(EKeys::S) ? 1 : 0);
    B->Steering = (IsInputKeyDown(EKeys::D) ? 1 : 0) - (IsInputKeyDown(EKeys::A) ? 1 : 0);
    B->Brake = IsInputKeyDown(EKeys::SpaceBar) ? 1 : 0;
    const FVector Center = B->GetActorLocation();
    const FVector Offset = bSideView ? FVector(100, -850, 480) : FVector(650, -800, 550);
    const FVector Pos = Center + Offset;
    LabCamera->SetActorLocation(Pos); LabCamera->SetActorRotation((Center + FVector(140, 100, 95) - Pos).Rotation());
}
void AWheelLabController::SetupInputComponent()
{
    Super::SetupInputComponent();
    InputComponent->BindKey(EKeys::R, IE_Pressed, this, &AWheelLabController::Reset);
    InputComponent->BindKey(EKeys::F, IE_Pressed, this, &AWheelLabController::Friction);
    InputComponent->BindKey(EKeys::M, IE_Pressed, this, &AWheelLabController::Mass);
    InputComponent->BindKey(EKeys::T, IE_Pressed, this, &AWheelLabController::Torque);
    InputComponent->BindKey(EKeys::C, IE_Pressed, this, &AWheelLabController::ChangeCamera);
}
void AWheelLabController::Reset() { if (auto* B = FindWheels(GetWorld())) B->ResetLab(); }
void AWheelLabController::Friction() { if (auto* B = FindWheels(GetWorld())) B->CycleFriction(); }
void AWheelLabController::Mass() { if (auto* B = FindWheels(GetWorld())) B->CycleMass(); }
void AWheelLabController::Torque() { if (auto* B = FindWheels(GetWorld())) B->CycleTorque(); }
void AWheelLabController::ChangeCamera() { bSideView = !bSideView; }

void AWheelLabHUD::DrawHUD()
{
    Super::DrawHUD(); auto* B = FindWheels(GetWorld()); if (!Canvas || !B) return;
    const float Scale = FMath::Min(1.f, FMath::Min(Canvas->SizeX / 1100.f, Canvas->SizeY / 720.f));
    const float W = Canvas->SizeX / Scale, H = Canvas->SizeY / Scale;
    const FLinearColor Ink(.9f,.94f,1), Muted(.6f,.72f,.8f), Accent(.15f,.85f,.8f);
    auto Rect = [&](float X,float Y,float Width,float Height) { DrawRect(FLinearColor(.02f,.035f,.06f,.95f), X*Scale,Y*Scale,Width*Scale,Height*Scale); };
    auto Text = [&](FString T,float X,float Y,FLinearColor Color,float Size=1.f) { DrawText(T,Color,X*Scale,Y*Scale,GEngine->GetSmallFont(),Scale*Size,false); };
    Rect(16,16,380,585); Text(TEXT("FORKLIFT / WHEEL LAB"),32,30,Accent,1.4f);
    Text(TEXT("02  MANUAL DRIVE / FLAT FLOOR"),32,60,Muted);
    if (!B->Error.IsEmpty()) { Text(B->Error,32,100,FLinearColor::Red); return; }
    auto& M = B->Model; const auto& S = M.Settings;
    Text(FString::Printf(TEXT("Speed         %+.3f m/s  |  %.1f km/h"), M.ForwardSpeed(), M.VelocityMps.Size()*3.6),32,100,Ink);
    Text(FString::Printf(TEXT("Acceleration  %+.3f m/s2"),B->AccelerationMps2),32,124,Ink);
    Text(FString::Printf(TEXT("Road friction %.2f     Mass %.0f kg"),S.Friction,S.VehicleMassKg),32,156,Accent);
    Text(FString::Printf(TEXT("Drive limit   %.0f Nm / front wheel"),S.DriveTorqueNm),32,180,Ink);
    Text(FString::Printf(TEXT("Brake limit   %.0f Nm / wheel"),S.BrakeTorqueNm),32,204,Ink);
    Text(FString::Printf(TEXT("Radius %.3f m | Inertia %.3f kg.m2"),S.WheelRadiusM,S.WheelInertia()),32,228,Muted);
    Text(FString::Printf(TEXT("Throttle %+.0f   Brake %.0f   Steer %+.0f"),B->Throttle,B->Brake,B->Steering),32,264,Ink);
    Text(FString::Printf(TEXT("Rear steer %.1f deg | Yaw %.1f deg/s"),FMath::RadiansToDegrees(M.SteeringRad),FMath::RadiansToDegrees(M.YawRateRadS)),32,288,Ink);
    Text(TEXT("Wheel     rad/s    rim m/s    slip m/s"),32,326,Accent);
    const TCHAR* Names[] = { TEXT("FL"), TEXT("FR"), TEXT("RL"), TEXT("RR") };
    for (int I=0;I<4;++I)
        Text(FString::Printf(TEXT("%s       %7.2f    %6.2f    %+7.3f"), Names[I], M.Wheels[I].OmegaRadS,
            M.Wheels[I].OmegaRadS*S.WheelRadiusM,M.Wheels[I].SlipSpeedMps),32,352+I*24,Ink);
    const auto& FL = M.Wheels[0];
    Text(FString::Printf(TEXT("FL normal %.0f N | grip budget %.0f N"),FL.NormalForceN,S.Friction*FL.NormalForceN),32,464,Muted);
    Text(FString::Printf(TEXT("FL Fx %+.0f N | Fy %+.0f N"),FL.LongForceN,FL.LateralForceN),32,488,Muted);
    Text(FString::Printf(TEXT("Time %.1f s | Fixed physics 240 Hz"),M.TimeS),32,528,Accent);
    Text(TEXT("CSV auto-save 5 s | No sensor / autonomy"),32,562,Muted);
    Rect(16,H-76,W-32,60);
    Text(TEXT("W/S Drive/reverse   A/D Steer   SPACE Brake   R Reset   F Friction   M Mass   T Torque   C Camera"),32,H-58,Ink);
    Text(TEXT("Planar dynamics only: fixed forks, no load transfer, suspension, rollover or obstacle collisions."),32,H-36,Muted);
}
