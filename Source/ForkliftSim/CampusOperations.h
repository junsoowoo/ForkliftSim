#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CampusInventory.h"
#include "SensorDrive.h"
#include "PalletLoadActor.h"
#include "CampusFleet.h"
#include "TaskBoard.h"
#include "FleetBattery.h"
#include "CampusOperations.generated.h"
class AStabilityLab;
UCLASS()
class FORKLIFTSIM_API ACampusOperations : public AActor
{
    GENERATED_BODY()
public:
    ACampusOperations();virtual void BeginPlay() override;virtual void Tick(float Dt) override;
    virtual void EndPlay(const EEndPlayReason::Type Reason) override;
    UPROPERTY() TArray<TObjectPtr<AStabilityLab>> Vehicles;
    UPROPERTY() TMap<FString,TObjectPtr<APalletLoadActor>> PhysicalLoads;
    UPROPERTY() TArray<TObjectPtr<UStaticMeshComponent>> Floors;
    UPROPERTY() TArray<TObjectPtr<AActor>> NavigationReferences;
    CampusInventory::FRegistry Inventory;FPayloadSettings PayloadSpec;
    TArray<SensorDrive::FSegment> Survey;
    TArray<FString> LoadOrder;TArray<FVector> ParkingStartM;int InitialVehicles=3;TMap<FString,double> StableSince;
    int Selected=0;FString Notice,Error;double Age=0,NextInventory=0,NextSnapshot=0;
    bool Validation=false;int EntryYaw=0;FString TestCase;
    CampusFleet::FGraph Graph;CampusFleet::FTraffic Traffic;TMap<FString,CampusFleet::FJob> Jobs;
    TMap<FString,FVector> SlotApproaches;TMap<FString,double> SlotYaws;
    TMap<FString,FVector> KnownStations;TMap<FString,FVector2D> SlotBeamSides;
    bool HighTier=false,FleetTest=false;int JobNumber=0;double NextFleetWeb=0;
    TSharedPtr<class FWebTelemetryPublisher,ESPMode::ThreadSafe> FleetWeb;
    CampusFleet::FJob* Job(const FString& Vehicle){return Jobs.Find(Vehicle);}
    WarehouseTasks::FBoard TaskBoard;TArray<TSharedPtr<FJsonValue>> CommandReceipts;TMap<FString,TSharedPtr<FJsonObject>> SeenCommands;
    TSharedPtr<class FWebTaskInbox,ESPMode::ThreadSafe> TaskInbox;
    double NextDispatch=0,NextTaskSave=0,ScenarioTyre=.9,ScenarioRoad=.8;FString TaskFolder;bool BoardAuto=true;
    FleetBattery::FSpec BatterySpec;TMap<FString,FleetBattery::FState> Batteries;
    struct FCharger {FString Id,Owner;FVector2D Position;};TArray<FCharger> Chargers;
    double LastEnergySave=-1;int PeakMoving=0,CompletedCharges=0;
    bool BoardEnabled() const{return !Validation&&(!FleetTest||TestCase.StartsWith(TEXT("board_"))||TestCase.StartsWith(TEXT("ops_")));}
    TArray<FVector2D> TrafficRoute(const AStabilityLab& V,const CampusFleet::FJob& J) const;
    bool ClaimRoute(const AStabilityLab& V,const CampusFleet::FJob& J);
    void ObserveTraffic();void LoadOperationsConfig();void ServiceEnergy(double Dt);void ScheduleCharges();bool UpdateCharge(AStabilityLab& V,CampusFleet::FJob& J,double Dt);
    bool BatteryReady(const AStabilityLab& V) const;
    TSharedRef<FJsonObject> EnergyTelemetry() const;
    void OperationsValidation(float Dt);
    bool BuildJob(AStabilityLab& V,const FString& Load,const FString& Destination,bool High,CampusFleet::FJob& Out,FString& Why) const;
    bool CommitJob(AStabilityLab& V,CampusFleet::FJob Plan,FString& Why);
    bool ReadyForTask(const AStabilityLab& V,bool ForCharge=false) const;
    mutable TMap<FString,Mission::FPlan> AssignmentPlans;mutable int PlanningBudget=8;
    Mission::FPlan AssignmentPlan(FVector2D Start,FVector2D Goal) const;
    bool AddTasks(const TArray<WarehouseTasks::FRequest>& Requests,TArray<FString>& Ids,FString& Why);
    void ServiceTasks(double Dt);void ConsumeTaskCommands();void SaveTasks();void LoadTaskConfig();
    TSharedRef<FJsonObject> TasksTelemetry() const;
    void QueueSelected();void QueueAll();void PauseSelected();void ResumeSelected();void CancelSelected();
    void UpdateVehicle(AStabilityLab& V,double Dt);void ApplyVehicle(AStabilityLab& V);
    bool FleetPermit(const AStabilityLab& V,FVector2D P) const;bool FleetField(const AStabilityLab& V,FVector2D P) const;
    bool VehicleEngaged(const AStabilityLab& V) const;
    void PublishFleet();void FleetValidationStep(float Dt);void TaskValidationStep(float Dt);
    void SetupRouteReferences();
    TSharedRef<FJsonObject> FleetTelemetry() const;
    bool SawVision=false,SawFull=false,SawLift=false,SawPeer=false,SawVehicleSupport=false;
    FVector StartPosition;double IdleDrift=0,MaxForkLoad=0,MaxWheelLoad=0,FinalRise=0;
    AStabilityLab* Active() const;
    void SelectNext();bool SelectLoad(const FString& Id);void CycleLoad();
    APalletLoadActor* ActivateLoad(const FString& Id);
    void RemovePresentation(const FString& Id);
    void ConfigureVehicle(AStabilityLab& Lab);
    void ReconcileInventory();void WriteInventorySnapshot();
    static ACampusOperations* Find(UWorld* World);
private:
    TArray<FVector> IdleStart;double TestStage=0,StageStarted=0,TestDepth=0,LastLoggedStage=-1,TestMinClearance=1e9;
    double TestPeakLoadedMps=0,TestPeakEmptyMps=0,TestObstacleEntryMps=0;
    bool LoadCampus();void ValidateStep(float Dt);void FinishValidation(bool Pass,const FString& Reason);
};
