#include "CampusFleet.h"
FVector2D CampusFleet::DockingStopOffset(FVector2D Motion){return Motion*(.3+Motion.Size());}
Mission::FPlan CampusFleet::FGraph::Plan(FVector2D Start,FVector2D Goal) const
{
    Mission::FPlan R;const int N=Nodes.Num();if(N<2||!Clearance.Free(Start)||!Clearance.Free(Goal)){R.Reason=TEXT("endpoint_footprint_blocked");return R;}
    TArray<FVector2D> P;for(const auto& V:Nodes)P.Add(V.P);P.Add(Start);P.Add(Goal);
    TArray<TArray<int>> Links;Links.SetNum(N+2);
    auto Add=[&](int A,int B){if(Clearance.Clear(P[A],P[B])){Links[A].Add(B);Links[B].Add(A);}};
    for(const auto& E:Edges)if(E.A>=0&&E.A<N&&E.B>=0&&E.B<N&&E.Width>=2.0)Add(E.A,E.B);
    // Endpoint connectors must clear the full fixed-heading vehicle footprint.
    // Keep the nearest four visible authored nodes; do not cut across racks/walls.
    for(int K=N;K<N+2;++K){TArray<int> Order;for(int I=0;I<N;++I)Order.Add(I);Order.Sort([&](int A,int B){return (P[A]-P[K]).SizeSquared()<(P[B]-P[K]).SizeSquared();});int Added=0;for(int I:Order)if(Clearance.Clear(P[I],P[K])){Add(I,K);if(++Added==4)break;}}
    if(Clearance.Clear(Start,Goal))Add(N,N+1);
    TArray<double> D;D.Init(1e30,N+2);D[N]=0;TArray<int> Parent;Parent.Init(-1,N+2);TSet<int> Done;
    for(int Iter=0;Iter<N+2;++Iter){int At=-1;double Best=1e29;for(int I=0;I<N+2;++I)if(!Done.Contains(I)&&D[I]<Best){Best=D[I];At=I;}if(At<0)break;Done.Add(At);++R.Expanded;if(At==N+1)break;for(int Next:Links[At]){double Cost=D[At]+(P[Next]-P[At]).Size();if(Cost<D[Next]){D[Next]=Cost;Parent[Next]=At;}}}
    if(!Done.Contains(N+1)){R.Reason=TEXT("no_surveyed_route");return R;}
    for(int At=N+1;At>=0;At=Parent[At]){R.Points.Insert(P[At],0);if(At==N)break;}R.Valid=true;R.Length=D[N+1];return R;
}
namespace {TArray<FBox2D> Corridors(const TArray<FVector2D>& P,bool Split){TArray<FBox2D> B;for(int I=1;I<P.Num();++I){const int Count=Split?FMath::Max(1,FMath::CeilToInt((P[I]-P[I-1]).Size())):1;for(int K=0;K<Count;++K){FBox2D X(ForceInit);X+=FMath::Lerp(P[I-1],P[I],double(K)/Count);X+=FMath::Lerp(P[I-1],P[I],double(K+1)/Count);X.Min-=FVector2D(2.3,2.3);X.Max+=FVector2D(2.3,2.3);B.Add(X);}}return B;}}
bool CampusFleet::FTraffic::Acquire(const FString& Owner,const TArray<FVector2D>& Route,double Now)
{
    Blocker.Empty();if(Owner.IsEmpty()||Route.Num()<2)return false;const auto Boxes=Corridors(Route,Sections);
    auto Deny=[&](const FString& Other){Blocker=Other;if(Other!=Owner)WaitingFor.Add(Owner,Other);return false;};
    for(auto& C:Claims)if(C.Owner==Owner&&C.Quarantined)return Deny(Owner);
    if(Sections)for(const auto& P:Occupants)if(P.Owner!=Owner)for(const auto& B:Boxes)if(P.Bounds.Intersect(B))return Deny(P.Owner);
    for(const auto& C:Claims)if(C.Owner!=Owner)for(const auto& A:C.Corridors)for(const auto& B:Boxes)if(A.Intersect(B))return Deny(C.Owner);
    WaitingFor.Remove(Owner);for(auto& C:Claims)if(C.Owner==Owner){C.Corridors=Boxes;C.Heartbeat=Now;return true;}Claims.Add({Owner,Boxes,Now,false});return true;
}
void CampusFleet::FTraffic::Observe(const FString& Owner,FVector2D Position,bool Fresh,double Now)
{
 for(auto& P:Occupants)if(P.Owner==Owner){P.Fresh=Fresh;if(Fresh){P.Bounds=FBox2D(Position-FVector2D(2.3,2.3),Position+FVector2D(2.3,2.3));P.Stamp=Now;}return;}
 Occupants.Add({Owner,FBox2D(Position-FVector2D(2.3,2.3),Position+FVector2D(2.3,2.3)),Fresh,Now});
}
TSet<FString> CampusFleet::FTraffic::Deadlock() const
{
 TSet<FString> Result;for(const auto& Start:WaitingFor){TArray<FString> Chain;FString At=Start.Key;while(const FString* Next=WaitingFor.Find(At)){const int Seen=Chain.Find(At);if(Seen!=INDEX_NONE){for(int I=Seen;I<Chain.Num();++I)Result.Add(Chain[I]);break;}Chain.Add(At);At=*Next;}}return Result;
}
void CampusFleet::FTraffic::Heartbeat(const FString& Owner,double Now){for(auto& C:Claims)if(C.Owner==Owner&&!C.Quarantined)C.Heartbeat=Now;}
void CampusFleet::FTraffic::Audit(double Now){for(auto& C:Claims)if(Now-C.Heartbeat>.5)C.Quarantined=true;}
void CampusFleet::FTraffic::Release(const FString& Owner,bool Clear){if(Clear){Claims.RemoveAll([&](const FClaim& C){return C.Owner==Owner;});WaitingFor.Remove(Owner);}}
void CampusFleet::FJob::Hold(const FString& Why,bool Fault){if(!Held())ResumeState=State;State=Fault?TEXT("fault"):TEXT("paused");Reason=Why;Velocity=FVector2D::ZeroVector;YawRate=0;Brake=true;}
