#include "RackHandling.h"
#include "StabilityLab.h"
#include "ChassisActor.h"
#include "ForkActor.h"
#include "WheelActor.h"
#include "ContactLoadSampler.h"
#include "Vehicle3DHelpers.h"
#include "Components/SceneComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/World.h"
#include "Engine/StaticMeshActor.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonSerializer.h"

bool FRackSettings::Load(FString& Error)
{
    FString Text;TSharedPtr<FJsonObject> J;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("RackHandling.json"))) || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J) || !J)
    {Error=TEXT("Cannot read RackHandling.json");return false;}
    auto Read=[&](const TCHAR* K,double& V){return J->TryGetNumberField(K,V) && FMath::IsFinite(V);};
    if(!Read(TEXT("center_x_m"),X)||!Read(TEXT("center_y_m"),Y)||!Read(TEXT("depth_m"),Depth)||!Read(TEXT("width_m"),Width)||!Read(TEXT("lower_deck_top_m"),Low)||!Read(TEXT("upper_deck_top_m"),High)||!Read(TEXT("deck_thickness_m"),Thickness)||!Read(TEXT("post_height_m"),PostHeight)||!Read(TEXT("support_mu"),Mu)||!Read(TEXT("handling_clearance_m"),Clearance)||Depth<1.3||Width<1.2||Low<.2||High<Low+1.4||High>2.2||Thickness<.02||Thickness>.1||PostHeight<High+.4||Mu<0||Mu>1.5||Clearance<.02||Clearance>.2)
    {Error=TEXT("Invalid handling rack dimensions");return false;}
    return true;
}
ARackHandlingCourse::ARackHandlingCourse()
{RootComponent=CreateDefaultSubobject<USceneComponent>(TEXT("RackRoot"));RootComponent->SetMobility(EComponentMobility::Static);}
void ARackHandlingCourse::Configure(const FRackSettings& S)
{
    auto Box=[&](FVector P,FVector Size,FLinearColor Color,bool Collide)
    {
        auto* B=NewObject<UStaticMeshComponent>(this);AddInstanceComponent(B);B->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube")));
        B->SetMobility(EComponentMobility::Static);B->SetupAttachment(RootComponent);B->SetWorldLocation(P*100);B->SetWorldScale3D(Size);
        B->SetCollisionProfileName(Collide?TEXT("BlockAll"):TEXT("NoCollision"));B->RegisterComponent();B->SetPhysMaterialOverride(Vehicle3D::Material(this,S.Mu));Vehicle3D::Color(B,Color);return B;
    };
    for(double Z:{S.Low,S.High})Supports.Add(Box(FVector(S.X,S.Y,Z-S.Thickness*.5),FVector(S.Depth,S.Width,S.Thickness),FLinearColor(.18,.27,.3),true));
    for(double DX:{-S.Depth*.5,S.Depth*.5})for(double DY:{-S.Width*.5-.05,S.Width*.5+.05})
        Box(FVector(S.X+DX,S.Y+DY,S.PostHeight*.5),FVector(.08,.08,S.PostHeight),FLinearColor(.25,.38,.44),true);
    Box(FVector(S.X-2,S.Y,.003),FVector(2,.02,.004),FLinearColor(.2,.8,.7),false);
    for(int I=0;I<2;++I)
    {
        auto* Label=NewObject<UTextRenderComponent>(this);AddInstanceComponent(Label);Label->SetupAttachment(RootComponent);
        Label->SetWorldLocation(FVector((S.X-S.Depth*.5-.05)*100,(S.Y-S.Width*.5-.12)*100,(I?S.High:S.Low)*100+8));
        Label->SetWorldRotation(FRotator(0,180,0));Label->SetWorldSize(10);Label->SetText(FText::FromString(FString::Printf(TEXT("TIER %d / %.2f m"),I+1,I?S.High:S.Low)));Label->RegisterComponent();
    }
}
bool AStabilityLab::IsRackTest() const {return TestMode.StartsWith(TEXT("mecanum_rack_"));}
double AStabilityLab::RackTierM() const {return TestMode.EndsWith(TEXT("high"))?RackSettings.High:RackSettings.Low;}
bool AStabilityLab::IsRackPickup() const {return TestMode.Contains(TEXT("rack_pick_"));}
void AStabilityLab::SetupHandlingRack()
{
    if(HandlingRack){HandlingRack->Destroy();HandlingRack=nullptr;}RackTrial=FRackTrial();RackSupportN=0;
    if(!Settings.bReferenceGeometry || (!TestMode.IsEmpty() && !IsRackTest() && !IsHandlingTest() && !IsMissionTest() && !GetWorld()->GetMapName().Contains(TEXT("WarehouseLab"))))return;
    HandlingRack=GetWorld()->SpawnActor<ARackHandlingCourse>();HandlingRack->Configure(RackSettings);
    double PalletBottom=0;
    if(PayloadSettings.bWarehouseAssets)
    {
        PalletBottom=TNumericLimits<double>::Max();
        for(int I=0;I<PayloadSettings.BoardCentersM.Num();++I)
            PalletBottom=FMath::Min(PalletBottom,PayloadSettings.BoardCentersM[I].Z-PayloadSettings.BoardSizesM[I].Z*.5);
    }
    const double Stack=PayloadSettings.bWarehouseAssets?PayloadSettings.PalletTopM-PalletBottom+PayloadSettings.CargoHeightM:1.4;
    const double Gap=RackSettings.High-RackSettings.Thickness-RackSettings.Low;
    UE_LOG(LogTemp,Display,TEXT("RACK_GEOMETRY gap_m=%.6f stack_m=%.6f clearance_m=%.6f stroke_m=%.3f"),Gap,Stack,Gap-Stack,Settings.ForkTravelM);
}
void AStabilityLab::ControlRackTest()
{
    Throttle=Strafe=Steering=Lift=0;Brake=1;bNeutral=false;
    if(!Pallet || !HandlingRack){RackTrial.Failed=true;RackTrial.Failure=TEXT("missing_fixture");return;}
    auto& T=RackTrial;const bool Pick=IsRackPickup();const double Tier=RackTierM();
    // Blade bottom/top are about lift+0.011/0.061 m. The pallet opening
    // spans deck+0.025..0.119 m: +0.035 m leaves clearance on both faces.
    const FVector Body=Chassis->GetActorLocation()/100.;const double Insert=Tier+.035,Clear=FMath::Min(Settings.ForkTravelM,Tier+.15);
    const double Weight=Pallet->LoadMass()*9.81;
    auto Advance=[&](){UE_LOG(LogTemp,Display,TEXT("RACK_STAGE %s stage=%d time=%.3f fork=%.4f rack_n=%.2f"),*TestMode,T.Stage,TimeS,ForkHeightM,RackSupportN);++T.Stage;T.StageStart=TimeS;T.StableS=0;T.Stopping=false;};
    auto Stable=[&](bool OK){T.StableS=OK?T.StableS+GetWorld()->GetDeltaSeconds():0;return T.StableS>.5;};
    auto Move=[&](double X,double Y,double Limit)
    {
        auto Axis=[&](double E){return FMath::Abs(E)>.01?FMath::Sign(E)*FMath::Clamp(FMath::Abs(E)*.8,.10,Limit):0.;};
        Brake=0;Throttle=Axis(X-Body.X);Strafe=Axis(Y-Body.Y);
        const bool Arrived=FMath::Abs(X-Body.X)<.035 && FMath::Abs(Y-Body.Y)<.035;
        if(FMath::Abs(X-Body.X)<.025 && FMath::Abs(Y-Body.Y)<.025)T.Stopping=true;
        if(T.Stopping){Brake=1;Throttle=Strafe=0;}
        return Stable(Arrived && FVector2D(SpeedMps,LateralMps).Size()<.035);
    };
    if(T.Stage==0)
    {
        LiftTargetM=0;if(TimeS>2){T.InitialPalletX=Pick?1.4104257:Pallet->GetActorLocation().X/100.-Body.X;Advance();}
    }
    else if(T.Stage==1){LiftTargetM=Pick?.4:.5;if(Stable(ForkHeightM>(Pick?.36:.46) && (Pick || PayloadState.GroundN<20)))Advance();}
    else if(T.Stage==2){LiftTargetM=Pick?.4:.5;if(Move(4.,0,.35))Advance();}
    else if(T.Stage==3){LiftTargetM=Pick?.4:.5;if(Move(4.,RackSettings.Y,.3))Advance();}
    else if(T.Stage==4){LiftTargetM=Pick?Insert:Clear;if(Stable(FMath::Abs(ForkHeightM-LiftTargetM)<.035))Advance();}
    else if(T.Stage==5){LiftTargetM=Pick?Insert:Clear;if(Move(RackSettings.X-T.InitialPalletX,RackSettings.Y,.15))Advance();}
    else if(T.Stage==6)
    {
        LiftTargetM=Pick?Clear:Insert;
        const double ForkN=PayloadState.VerticalN[0]+PayloadState.VerticalN[1];
        const bool Transfer=Pick?(RackSupportN<Weight*.05 && ForkN>Weight*.85 && ForkHeightM>Tier+.10):(RackSupportN>Weight*.85 && ForkN<Weight*.1);
        if(Stable(Transfer)){T.Transferred=true;T.TransferS=TimeS;Advance();}
    }
    else if(T.Stage==7){LiftTargetM=Pick?Clear:Insert;if(Move(3.8,RackSettings.Y,.15))Advance();}
    else if(T.Stage==8){LiftTargetM=.45;if(Stable(ForkHeightM<.5 && (Pick?RackSupportN<20:RackSupportN>Weight*.85))){T.FinalRackN=RackSupportN;Advance();}}
    else LiftTargetM=.45;
    if(T.Stage<9 && TimeS-T.StageStart>(T.Stage==0?10:45)) {T.Failed=true;T.Failure=FString::Printf(TEXT("stage_%d_timeout"),T.Stage);}
    if(TiltDeg>20 || CargoState.bFell || CargoState.bTipped){T.Failed=true;T.Failure=TEXT("load_or_chassis_unstable");}
}
bool AStabilityLab::CheckRackTest() const
{
    const double W=Pallet?Pallet->LoadMass()*9.81:1;const FVector P=Pallet?Pallet->GetActorLocation()/100.:FVector::ZeroVector;
    bool OK=RackTrial.Stage>=9 && !RackTrial.Failed && RackTrial.Transferred && !CargoState.bFell && !CargoState.bTipped && CargoState.NormalN>PayloadSettings.CargoMassKg*9.81*.8 && MaxTilt<10 && MaxInnerClockErrorS<1e-7;
    if(IsRackPickup())OK &= RackSupportN<20 && PayloadState.GroundN<20 && PayloadState.VerticalN[0]+PayloadState.VerticalN[1]>W*.85;
    else OK &= FMath::Abs(RackSupportN/W-1)<.1 && FMath::Abs(P.X-RackSettings.X)<.10 && FMath::Abs(P.Y-RackSettings.Y)<.10 && Chassis->GetActorLocation().X<400;
    for(const AForkTineActor* T:Fork->Tines)OK &= !T->bFailed;
    for(const AWheelActor* Wheel:Wheels)
    {
        const double N=PreviousPhysicsDt>0?Wheel->ContactData.NormalNs/PreviousPhysicsDt:0;
        OK &= !Wheel->bSteerable && Wheel->Rollers.IsEmpty() && FMath::Abs(Wheel->NormalLoadN-N)<1e-5;
    }
    UE_LOG(LogTemp,Display,TEXT("RACK_CHECK %s stage=%d transferred=%d rack_n=%.4f weight_n=%.4f pallet_xyz_m=%.5f/%.5f/%.5f fork_m=%.5f max_tilt_deg=%.4f failure=%s %s"),*TestMode,RackTrial.Stage,RackTrial.Transferred?1:0,RackSupportN,W,P.X,P.Y,P.Z,ForkHeightM,MaxTilt,*RackTrial.Failure,OK?TEXT("PASS"):TEXT("FAIL"));return OK;
}
