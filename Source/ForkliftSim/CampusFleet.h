#pragma once
#include "CoreMinimal.h"
#include "SensorDrive.h"
#include "MissionNavigation.h"
// Surveyed routes and measured vehicle poses only. No actor access.
namespace CampusFleet
{
    // Campus transit targets in m/s, within the sensor drive controller's 1 m/s range.
    inline constexpr double EmptyCruiseMps=1.0;
    inline constexpr double LoadedCruiseMps=.85;
    // Research docking sweep: 0.3 s response allowance, 0.5 m/s^2 braking assumption.
    FVector2D DockingStopOffset(FVector2D Motion);
    struct FNode {FString Id;FVector2D P;};
    struct FEdge {int A=0,B=0;double Width=0;};
    struct FGraph
    {
        TArray<FNode> Nodes;TArray<FEdge> Edges;Mission::FPlanner Clearance;
        Mission::FPlan Plan(FVector2D Start,FVector2D Goal) const;
    };
    struct FClaim {FString Owner;TArray<FBox2D> Corridors;double Heartbeat=0;bool Quarantined=false;};
    struct FTraffic
    {
        TArray<FClaim> Claims;FString Blocker;
        struct FOccupant {FString Owner;FBox2D Bounds;bool Fresh=false;double Stamp=0;};
        TArray<FOccupant> Occupants;TMap<FString,FString> WaitingFor;
        bool Sections=false;
        void Observe(const FString& Owner,FVector2D Position,bool Fresh,double Now);
        TSet<FString> Deadlock() const;
        // Atomic whole-job route reservation avoids hold-and-wait deadlocks.
        // Never expire a stopped/unknown vehicle's occupied route into free space.
        bool Acquire(const FString& Owner,const TArray<FVector2D>& Route,double Now);
        void Heartbeat(const FString& Owner,double Now);void Audit(double Now);
        void Release(const FString& Owner,bool ConfirmedClear);
    };
    struct FJob
    {
        FString Id,Vehicle,Load,Destination,State=TEXT("queued"),Reason,Kind=TEXT("transport"),Charger;
        FVector2D Pickup,Dropoff;FVector PickCenter,DropCenter;double PickYaw=0,DropYaw=0,PickTier=0,DropTier=0;
        Mission::FPlan ToPick,ToDrop,ToReturn;int Waypoint=1;bool Carrying=false,Engaged=false,HandlingStarted=false;
        bool ManualRecovery=false,TrafficWaiting=false;bool PickupReceipt=false,DropoffReceipt=false;FString ResumeState;
        double Started=0,StageS=0,StableS=0,LeaseUntil=0,ReferenceLoad=0,LastProgressS=0,BestDistance=1e9;
        double CarrierFront=0,CarrierBack=0,CarrierUntil=-1,UnloadOffset=.015,NextObserve=0;int Replans=0;
        FVector2D Velocity=FVector2D::ZeroVector;double YawRate=0;bool Brake=true;
        bool Travelling() const{return State==TEXT("to_pickup")||State==TEXT("to_dropoff")||State==TEXT("returning")||State==TEXT("to_charger");}
        bool Held() const{return State==TEXT("paused")||State==TEXT("fault")||State==TEXT("cancelled");}
        void Hold(const FString& Why,bool Fault=false);
    };
}
