#include "TaskBoard.h"
bool WarehouseTasks::FBoard::Add(const TArray<FRequest>& R,double Now,TArray<FString>& Ids,FString& Error)
{
 Ids.Reset();if(R.IsEmpty()||R.Num()>10||Tasks.Num()+R.Num()>200){Error=TEXT("batch_1_to_10_session_limit_200");return false;}
 TSet<FString> Loads;for(const auto& T:Tasks)if(T.OwnsLoad())Loads.Add(T.Load);
 for(const auto& V:R){if(V.Load.IsEmpty()||V.Priority<0||V.Priority>9||(V.Tier!=1&&V.Tier!=2)||Loads.Contains(V.Load)){Error=TEXT("invalid_or_duplicate_open_load:")+V.Load;return false;}Loads.Add(V.Load);}
 for(const auto& V:R){FTask T;T.Id=FString::Printf(TEXT("TASK-%04d"),++Number);T.Load=V.Load;T.Destination=V.Destination;T.PreferredVehicle=V.PreferredVehicle;T.Priority=V.Priority;T.Tier=V.Tier;T.Created=T.Updated=Now;Tasks.Add(T);Ids.Add(T.Id);Event(Tasks.Last(),TEXT("pending"),TEXT("registered"),Now);}return true;
}
void WarehouseTasks::FBoard::Event(FTask& T,const FString& State,const FString& Reason,double Now)
{
 T.State=State;T.Reason=Reason;T.Updated=Now;Events.Add({Now,T.Id,State,Reason});if(Events.Num()>10000)Events.RemoveAt(0,Events.Num()-10000);
}
bool WarehouseTasks::FBoard::CancelPending(const FString& Id,double Now,FString& Error)
{
 for(auto& T:Tasks)if(T.Id==Id){if(!T.Pending()){Error=TEXT("only_pending_tasks_can_be_cancelled_from_web");return false;}T.Ended=Now;T.QueueWait=Now-T.Created;Event(T,TEXT("cancelled"),TEXT("operator_cancel_pending"),Now);return true;}Error=TEXT("unknown_task");return false;
}
TArray<int> WarehouseTasks::FBoard::PendingOrder() const
{
 TArray<int> Order;for(int I=0;I<Tasks.Num();++I)if(Tasks[I].Pending())Order.Add(I);Order.Sort([&](int A,int B){if(Tasks[A].Priority!=Tasks[B].Priority)return Tasks[A].Priority>Tasks[B].Priority;return A<B;});return Order;
}
void WarehouseTasks::FBoard::Assign(int Index,const FString& Vehicle,const FString& Job,const FString& Destination,double Now)
{
 auto& T=Tasks[Index];T.Vehicle=Vehicle;T.Job=Job;T.Destination=Destination;T.Assigned=Now;T.QueueWait=Now-T.Created;Event(T,TEXT("assigned"),TEXT(""),Now);
}
void WarehouseTasks::FBoard::Observe(FTask& T,const FSample& S)
{
 if(T.Assigned<0||T.State==TEXT("completed")||T.State==TEXT("cancelled")||T.State==TEXT("interrupted"))return;
 const double Dt=FMath::Clamp(S.Dt,0.,.1);T.Elapsed=FMath::Max(0.,S.Now-T.Assigned);T.Picked|=S.Picked;T.Dropped|=S.Dropped;
 if(S.Localized&&S.GroundFresh&&S.GroundSpeed<=2){if(S.GroundSpeed>.02&&(S.WheelSpeed>.02||S.GroundSpeed>.08))T.Distance+=S.GroundSpeed*Dt;}else T.PoseGapS+=Dt;
 T.PeakForkN=FMath::Max(T.PeakForkN,S.ForkLoad);T.TyreMuMin=FMath::Min(T.TyreMuMin,S.TyreMu);T.TyreMuMax=FMath::Max(T.TyreMuMax,S.TyreMu);T.RoadMuMin=FMath::Min(T.RoadMuMin,S.RoadMu);T.RoadMuMax=FMath::Max(T.RoadMuMax,S.RoadMu);
 if(S.GroundFresh&&S.VelocityResidual>.05)T.SlipS+=Dt;
 const bool Hold=S.State==TEXT("paused")||S.State==TEXT("fault")||(S.State==TEXT("cancelled")&&S.Engaged);
 const bool WasHold=T.Phase==TEXT("paused")||T.Phase==TEXT("fault")||T.Phase==TEXT("cancelled");
 if(Hold&&!WasHold)++T.Stops;if(!Hold&&WasHold)++T.Recoveries;
 if(Hold)T.Held+=Dt;else if(S.State==TEXT("queued")||S.TrafficWaiting)T.TrafficWait+=Dt;else if(S.State.StartsWith(TEXT("to_"))||S.State==TEXT("returning"))T.Travel+=Dt;else T.Handling+=Dt;
 FString Next=S.State==TEXT("fault")?TEXT("failed"):Hold?TEXT("paused"):TEXT("active");T.RecoveryRequired=Hold&&S.Engaged;
 if(!S.Engaged&&S.State==TEXT("complete")){Next=TEXT("completed");T.Ended=S.Now;T.RecoveryRequired=false;}
 else if(!S.Engaged&&S.State==TEXT("cancelled")){Next=TEXT("cancelled");T.Ended=S.Now;T.RecoveryRequired=false;}
 if(Next!=T.State||S.Reason!=T.Reason)Event(T,Next,S.Reason,S.Now);T.Phase=S.State;
}
void WarehouseTasks::FBoard::Close(double Now)
{
 for(auto& T:Tasks)if(!T.Terminal()||T.RecoveryRequired){T.Ended=Now;T.RecoveryRequired=false;if(T.Assigned<0)T.QueueWait=Now-T.Created;const FString Reason=T.State==TEXT("failed")?T.Reason:TEXT("play_session_ended");Event(T,T.State==TEXT("failed")?TEXT("failed"):TEXT("interrupted"),Reason,Now);}
}
