#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TreadReference.generated.h"

// Zero-friction fixed-hub fixture: prescribed indentation, then a known impulse
// on an airborne pad. Not a full-vehicle safety or real-rubber calibration test.
UCLASS()
class FORKLIFTSIM_API ATreadReference : public AActor
{
    GENERATED_BODY()
public:
    ATreadReference();
    virtual void BeginPlay() override;
    virtual void Tick(float Dt) override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
private:
    double EnergyJ() const;
    UPROPERTY() TObjectPtr<class UStaticMeshComponent> Mount;
    UPROPERTY() TObjectPtr<class UStaticMeshComponent> Road;
    UPROPERTY() TObjectPtr<class AWheelActor> Wheel;
    class FContactLoadSampler* Sampler=nullptr;
    double TimeS=0,RadiusM=.15,WidthM=.1,HeightM=.14;
    double NormalIntegral=0,SampleS=0,ReferenceJ=0,ImpulseWorkJ=0,PeakExcessJ=0;
    double MaxInnerS=0;
    bool bPulsed=false,bDone=false;
    double ExternalReferenceN=0;
    bool bProbe=false;
    int ProbePad=0;
    FVector ProbeStart=FVector::ZeroVector,ProbeForceN=FVector::ZeroVector;
    double NeighborReference[2]={0,0},NeighborResponseM=0;
};
