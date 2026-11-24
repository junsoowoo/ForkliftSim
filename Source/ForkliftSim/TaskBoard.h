#pragma once
#include "CoreMinimal.h"
namespace WarehouseTasks
{
struct FRequest {FString Load,Destination,PreferredVehicle;int Priority=0,Tier=1;};
struct FSample
{
 double Dt=0,Now=0;FString State,Reason;bool Engaged=true,Localized=false,GroundFresh=false;FVector2D Pose;
 double WheelSpeed=0,GroundSpeed=0,VelocityResidual=0,ForkLoad=0,TyreMu=0,RoadMu=0;bool Picked=false,Dropped=false,TrafficWaiting=false;
};
struct FTask
{
 FString Id,Load,Destination,PreferredVehicle,Vehicle,Job,State=TEXT("pending"),Reason,Phase;
 int Priority=0,Tier=1,Stops=0,Recoveries=0,Replans=0;bool RecoveryRequired=false,Picked=false,Dropped=false;
 double Created=0,Assigned=-1,Ended=-1,Updated=0,Elapsed=0,QueueWait=0,TrafficWait=0,Held=0,Travel=0,Handling=0,Distance=0,SlipS=0,PoseGapS=0,PeakForkN=0;
 double TyreMuMin=2,TyreMuMax=0,RoadMuMin=2,RoadMuMax=0;
 bool HavePose=false;FVector2D LastPose;
 bool Pending() const{return State==TEXT("pending");}
 bool Terminal() const{return State==TEXT("completed")||State==TEXT("cancelled")||State==TEXT("interrupted")||State==TEXT("failed");}
 bool OwnsLoad() const{return !Terminal()||RecoveryRequired;}
};
struct FEvent {double Time=0;FString Task,State,Reason;};
struct FBoard
{
 FString Run,Scenario=TEXT("baseline");TArray<FTask> Tasks;TArray<FEvent> Events;int Number=0;double Started=0;
 bool Add(const TArray<FRequest>& Requests,double Now,TArray<FString>& Ids,FString& Error);
 bool CancelPending(const FString& Id,double Now,FString& Error);
 TArray<int> PendingOrder() const;
 void Assign(int Index,const FString& Vehicle,const FString& Job,const FString& Destination,double Now);
 void Observe(FTask& Task,const FSample& Sample);
 void Close(double Now);
 void Event(FTask& Task,const FString& State,const FString& Reason,double Now);
};
}
