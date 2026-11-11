#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RackHandling.generated.h"

struct FRackSettings
{
    double X=6.95,Y=-2.5,Depth=1.45,Width=1.8,Low=.35,High=1.95,Thickness=.04,PostHeight=3,Mu=.5,Clearance=.05;
    bool Load(FString& Error);
};
struct FRackTrial
{
    int Stage=0;double StageStart=0,StableS=0,InitialPalletX=0,FinalRackN=0,TransferS=0;
    bool Failed=false,Transferred=false,Stopping=false;FString Failure;
};
UCLASS()
class FORKLIFTSIM_API ARackHandlingCourse : public AActor
{
    GENERATED_BODY()
public:
    ARackHandlingCourse();
    void Configure(const FRackSettings& Settings);
    UPROPERTY() TArray<TObjectPtr<class UStaticMeshComponent>> Supports;
};
