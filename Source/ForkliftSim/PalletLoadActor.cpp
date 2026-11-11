#include "PalletLoadActor.h"
#include "Vehicle3DHelpers.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "UObject/ConstructorHelpers.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

bool FPayloadSettings::UseWarehouseGeometry(FString& Error)
{
    FString Text;TSharedPtr<FJsonObject> J;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("WarehousePayload.json"))) ||
       !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J) || !J)
    {Error=TEXT("Cannot read WarehousePayload.json");return false;}
    auto Vec=[](const TSharedPtr<FJsonObject>& O,const TCHAR* Key,FVector& V)
    {
        const TArray<TSharedPtr<FJsonValue>>* A=nullptr;
        if(!O->TryGetArrayField(Key,A) || A->Num()!=3)return false;
        for(int I=0;I<3;++I) if(!(*A)[I]->TryGetNumber(V[I]) || !FMath::IsFinite(V[I]))return false;
        return true;
    };
    FVector Size,CargoSize;
    if(!Vec(J,TEXT("pallet_size_m"),Size) || !Vec(J,TEXT("pallet_datum_m"),PalletDatumM) ||
       !Vec(J,TEXT("cargo_size_m"),CargoSize) || !Vec(J,TEXT("cargo_bounds_min_m"),CargoBoundsMinM) ||
       !J->TryGetNumberField(TEXT("pallet_top_m"),PalletTopM) ||
       !J->TryGetNumberField(TEXT("fork_opening_height_m"),HoleHeightM) ||
       !J->TryGetNumberField(TEXT("fork_opening_width_m"),HoleWidthM) ||
       !J->TryGetStringField(TEXT("pallet_mesh"),PalletMeshPath) || !J->TryGetStringField(TEXT("cargo_mesh"),CargoMeshPath))
    {Error=TEXT("Invalid WarehousePayload dimensions");return false;}
    const TArray<TSharedPtr<FJsonValue>>* Boards=nullptr;
    if(!J->TryGetArrayField(TEXT("boards"),Boards) || Boards->Num()<12 || Boards->Num()>40)
    {Error=TEXT("Invalid WarehousePayload boards");return false;}
    BoardCentersM.Reset();BoardSizesM.Reset();
    for(const auto& Value:*Boards)
    {
        const auto O=Value->AsObject();FVector C,D;
        if(!O || !Vec(O,TEXT("center_m"),C) || !Vec(O,TEXT("size_m"),D) || D.GetMin()<.005 || D.GetMax()>1.5)
        {Error=TEXT("Invalid WarehousePayload board box");return false;}
        BoardCentersM.Add(C);BoardSizesM.Add(D);
    }
    LengthM=Size.X;WidthM=Size.Y;HoleSpacingM=.45;
    CargoLengthM=CargoSize.X;CargoWidthM=CargoSize.Y;CargoHeightM=CargoSize.Z;
    bWarehouseAssets=true;bCargoSecured=false;
    return Validate(Error);
}

bool FPayloadSettings::Load(FString& Error)
{
    Error.Reset();
    FString Text; TSharedPtr<FJsonObject> Json;
    if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("Payload.json"))) ||
       !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Json) || !Json.IsValid())
    { Error=TEXT("Cannot read Config/Payload.json"); return false; }
    struct FField { const TCHAR* Name; double* Value; };
    for(const FField& F : {FField{TEXT("pallet_mass_kg"),&PalletMassKg}, {TEXT("cargo_mass_kg"),&CargoMassKg},
        {TEXT("length_m"),&LengthM},{TEXT("width_m"),&WidthM},{TEXT("hole_height_m"),&HoleHeightM},
        {TEXT("hole_spacing_m"),&HoleSpacingM},{TEXT("hole_width_m"),&HoleWidthM},{TEXT("hole_shift_y_m"),&HoleShiftYM},
        {TEXT("fork_spacing_m"),&ForkSpacingM},{TEXT("fork_shift_y_m"),&ForkShiftYM},
        {TEXT("cargo_x_m"),&CargoXM},{TEXT("cargo_y_m"),&CargoYM},{TEXT("cargo_height_m"),&CargoHeightM},
        {TEXT("pallet_y_m"),&PalletYM},{TEXT("initial_depth_m"),&InitialDepthM},
        {TEXT("fork_mu"),&ForkMu},{TEXT("left_hole_mu"),&LeftHoleMu},{TEXT("right_hole_mu"),&RightHoleMu},{TEXT("cargo_mu"),&CargoMu}})
        if(!Json->TryGetNumberField(F.Name,*F.Value) || !FMath::IsFinite(*F.Value))
        { Error=FString::Printf(TEXT("Missing/invalid Payload.%s"),F.Name); return false; }
    if(!Json->TryGetBoolField(TEXT("cargo_secured"),bCargoSecured)) { Error=TEXT("Missing cargo_secured"); return false; }
    // Backward compatibility: legacy cargo_mu applied to both touching faces.
    PalletTopStaticMu=PalletTopDynamicMu=CargoStaticMu=CargoDynamicMu=CargoMu;
    for(const FField& F : {FField{TEXT("cargo_length_m"),&CargoLengthM},{TEXT("cargo_width_m"),&CargoWidthM},
        {TEXT("pallet_top_static_mu"),&PalletTopStaticMu},{TEXT("pallet_top_dynamic_mu"),&PalletTopDynamicMu},
        {TEXT("cargo_static_mu"),&CargoStaticMu},{TEXT("cargo_dynamic_mu"),&CargoDynamicMu}})
        if(Json->HasField(F.Name) && (!Json->TryGetNumberField(F.Name,*F.Value) || !FMath::IsFinite(*F.Value)))
        { Error=FString::Printf(TEXT("Invalid Payload.%s"),F.Name); return false; }
    return Validate(Error);
}
bool FPayloadSettings::Validate(FString& Error) const
{
    Error.Reset();
    const bool OK=PalletMassKg>=10 && PalletMassKg<=100 && CargoMassKg>=1 && CargoMassKg<=1000 &&
        LengthM>=.8 && LengthM<=1.4 && WidthM>=.9 && WidthM<=1.5 && HoleHeightM>=(bWarehouseAssets?.06:.12) && HoleHeightM<=.3 &&
        HoleWidthM>=.16 && HoleWidthM<=.45 && HoleSpacingM-HoleWidthM>=.08 &&
        FMath::Abs(HoleShiftYM)+HoleSpacingM*.5+HoleWidthM*.5<=WidthM*.5-.04 &&
        ForkSpacingM>=.3 && ForkSpacingM<=.85 && FMath::Abs(ForkShiftYM)+ForkSpacingM*.5<=.5 &&
        FMath::Abs(CargoXM)<=.25 && FMath::Abs(CargoYM)<=.25 && CargoHeightM>=.2 && CargoHeightM<=(bWarehouseAssets?1.4:1.2) &&
        FMath::Abs(PalletYM)<=.4 && InitialDepthM>=-.6 && InitialDepthM<=LengthM &&
        ForkMu>=0 && ForkMu<=1.5 && LeftHoleMu>=0 && LeftHoleMu<=1.5 && RightHoleMu>=0 && RightHoleMu<=1.5 && CargoMu>=0 && CargoMu<=1.5 &&
        CargoLengthM>=.2 && CargoLengthM<=(bWarehouseAssets?1.4:1.) && CargoWidthM>=.2 && CargoWidthM<=(bWarehouseAssets?1.4:1.) &&
        PalletTopDynamicMu>=0 && PalletTopStaticMu>=PalletTopDynamicMu && PalletTopStaticMu<=1.5 &&
        CargoDynamicMu>=0 && CargoStaticMu>=CargoDynamicMu && CargoStaticMu<=1.5;
    if(!OK) Error=TEXT("Payload geometry/mass/friction out of supported range; see Docs/PayloadLab.md");
    return OK;
}
APalletLoadActor::APalletLoadActor()
{
    Deck=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Deck")); RootComponent=Deck;
    static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube")); Deck->SetStaticMesh(Cube.Object);
    Strap=CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("CargoStrap"));
}
void APalletLoadActor::Configure(const FPayloadSettings& S)
{
    auto Mat=[&](double Mu) { auto* M=Vehicle3D::Material(this,Mu); M->SleepLinearVelocityThreshold=0; M->SleepAngularVelocityThreshold=0; M->SleepCounterThreshold=MAX_int32; return M; };
    auto ContactMat=[&](double StaticMu,double DynamicMu) { auto* M=Mat(DynamicMu); M->StaticFriction=StaticMu; return M; };
    const FVector P=GetActorLocation();const FQuat Orientation=GetActorQuat();
    if(S.bWarehouseAssets)
    {
        double Volume=0;for(const FVector& D:S.BoardSizesM)Volume+=D.X*D.Y*D.Z;
        const FVector First=S.BoardSizesM[0];
        Deck->SetWorldScale3D(First);Vehicle3D::Dynamic(Deck,S.PalletMassKg*First.X*First.Y*First.Z/Volume);
        Deck->SetLinearDamping(0);Deck->SetAngularDamping(0);
        Deck->SetPhysMaterialOverride(ContactMat(S.PalletTopStaticMu,S.PalletTopDynamicMu));
        for(int I=1;I<S.BoardSizesM.Num();++I)
        {
            const FVector D=S.BoardSizesM[I];
            auto* B=Vehicle3D::Box(this,Deck,P+Orientation.RotateVector((S.BoardCentersM[I]-S.PalletDatumM)*100),D,S.PalletMassKg*D.X*D.Y*D.Z/Volume,FLinearColor(.4,.2,.06),Orientation);
            const double Z=S.BoardCentersM[I].Z;
            B->SetPhysMaterialOverride(Z>.118?ContactMat(S.PalletTopStaticMu,S.PalletTopDynamicMu):Mat(S.CargoMu));
        }
        Deck->BodyInstance.UpdateMassProperties();
    }
    else
    {
    Deck->SetWorldScale3D(FVector(S.LengthM,S.WidthM,.04)); Vehicle3D::Dynamic(Deck,S.PalletMassKg*.4);
    Deck->SetLinearDamping(0); Deck->SetAngularDamping(0); Deck->SetPhysMaterialOverride(ContactMat(S.PalletTopStaticMu,S.PalletTopDynamicMu));
    Vehicle3D::Color(Deck,FLinearColor(.55f,.28f,.08f));
    const double HL=S.HoleShiftYM-S.HoleSpacingM*.5, HR=S.HoleShiftYM+S.HoleSpacingM*.5, HW=S.HoleWidthM*.5;
    const double Edges[]={-S.WidthM*.5,HL-HW,HL+HW,HR-HW,HR+HW,S.WidthM*.5};
    const double SolidWidth=S.WidthM-2*S.HoleWidthM;
    for(int I=0;I<3;++I)
    {
        const double W=Edges[2*I+1]-Edges[2*I], Y=(Edges[2*I]+Edges[2*I+1])*.5;
        auto* B=Vehicle3D::Box(this,Deck,P+Orientation.RotateVector(FVector(0,Y*100,-(S.HoleHeightM*.5+.025)*100)),FVector(S.LengthM,W,S.HoleHeightM+.01),S.PalletMassKg*.5*W/SolidWidth,FLinearColor(.4f,.2f,.06f),Orientation);
        B->SetPhysMaterialOverride(Mat(S.CargoMu));
    }
    for(int I=0;I<2;++I)
    {
        auto* Liner=Vehicle3D::Box(this,Deck,P+Orientation.RotateVector(FVector(0,(I==0?HL:HR)*100,-2.5)),FVector(S.LengthM,S.HoleWidthM,.01),S.PalletMassKg*.05,I==0?FLinearColor(.1f,.5f,.8f):FLinearColor(.6f,.2f,.8f),Orientation);
        Liner->SetPhysMaterialOverride(Mat(I==0?S.LeftHoleMu:S.RightHoleMu));
    }
    Deck->BodyInstance.UpdateMassProperties();
    }
    const double CargoBottom=S.bWarehouseAssets?S.PalletTopM-S.PalletDatumM.Z+.002:.022;
    Cargo=GetWorld()->SpawnActor<AStaticMeshActor>(P+Orientation.RotateVector(FVector(S.CargoXM*100,S.CargoYM*100,(CargoBottom+S.CargoHeightM*.5)*100)),GetActorRotation());
    auto* Body=Cargo->GetStaticMeshComponent(); Body->SetMobility(EComponentMobility::Movable); Body->SetStaticMesh(Deck->GetStaticMesh());
    Body->SetWorldScale3D(FVector(S.CargoLengthM,S.CargoWidthM,S.CargoHeightM)); Vehicle3D::Dynamic(Body,S.CargoMassKg);
    Body->SetLinearDamping(0); Body->SetAngularDamping(0); Body->SetPhysMaterialOverride(ContactMat(S.CargoStaticMu,S.CargoDynamicMu)); Vehicle3D::Color(Body,FLinearColor(.15f,.65f,.7f));
    if(S.bCargoSecured) { Strap->SetWorldLocation(P); Vehicle3D::Lock(Strap); Strap->SetConstrainedComponents(Deck,NAME_None,Body,NAME_None); }
    if(S.bWarehouseAssets)
    {
        auto Present=[&](AActor* VisualOwner,UStaticMeshComponent* Parent,const FString& Path,FVector Offset)
        {
            auto* Mesh=LoadObject<UStaticMesh>(nullptr,*Path);if(!Mesh)return;
            TInlineComponentArray<UStaticMeshComponent*> Parts(VisualOwner);for(auto* Part:Parts)Part->SetVisibility(false,false);
            auto* Frame=NewObject<USceneComponent>(VisualOwner);VisualOwner->AddInstanceComponent(Frame);
            Frame->SetupAttachment(Parent);Frame->SetAbsolute(false,false,true);Frame->RegisterComponent();Frame->SetWorldScale3D(FVector::OneVector);
            auto* Visual=NewObject<UStaticMeshComponent>(VisualOwner);VisualOwner->AddInstanceComponent(Visual);Visual->SetupAttachment(Frame);
            Visual->SetMobility(EComponentMobility::Movable);Visual->SetStaticMesh(Mesh);Visual->SetRelativeLocation(Offset*100);Visual->BodyInstance.bAutoWeld=false;Visual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            Visual->SetGenerateOverlapEvents(false);Visual->RegisterComponent();
        };
        Present(this,Deck,S.PalletMeshPath,-S.PalletDatumM);
        Present(Cargo,Body,S.CargoMeshPath,-S.CargoBoundsMinM-FVector(S.CargoLengthM,S.CargoWidthM,S.CargoHeightM)*.5);
        UE_LOG(LogTemp,Display,TEXT("WAREHOUSE_PAYLOAD boards=%d pallet_kg=%.3f cargo_kg=%.3f secured=0"),S.BoardSizesM.Num(),Deck->GetMass(),Body->GetMass());
    }
}
double APalletLoadActor::LoadMass() const { return Deck->GetMass()+(Cargo?Cargo->GetStaticMeshComponent()->GetMass():0); }
FVector APalletLoadActor::LoadCOM() const
{
    return (Deck->GetCenterOfMass()*Deck->GetMass()+(Cargo?Cargo->GetStaticMeshComponent()->GetCenterOfMass()*Cargo->GetStaticMeshComponent()->GetMass():FVector::ZeroVector))/LoadMass();
}
void APalletLoadActor::EndPlay(const EEndPlayReason::Type R) { if(Cargo) Cargo->Destroy(); Super::EndPlay(R); }
