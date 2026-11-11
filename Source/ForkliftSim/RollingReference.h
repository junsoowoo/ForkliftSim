#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RollingReference.generated.h"

// One unconstrained rigid wheel: separates contact/integration errors from
// the forklift's articulation, motors, compliance and hydraulic circuit.
UCLASS()
class FORKLIFTSIM_API ARollingReference : public AActor
{
    GENERATED_BODY()
public:
    ARollingReference();
    virtual void BeginPlay() override;
    virtual void Tick(float Dt) override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    UPROPERTY() TObjectPtr<class UWheelTyreComponent> Body;
private:
    double EnergyJ() const;
    double TimeS=0, StartJ=0, StartV=0, PeakGainJ=0, SampleAt=0, Speed=2;
    bool bSeeded=false, bDone=false;
    FString Shape=TEXT("cylinder");
    class FContactLoadSampler* Sampler=nullptr;
    int64 RefinedContacts=0;
};
