#include "MissionNavigation.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonSerializer.h"
#include <queue>
#include <vector>
namespace
{
    bool SegmentBox(FVector2D A,FVector2D B,FVector2D Min,FVector2D Max)
    {
        double T0=0,T1=1;const FVector2D D=B-A;
        for(int I=0;I<2;++I){if(FMath::Abs(D[I])<1e-10){if(A[I]<Min[I]||A[I]>Max[I])return false;}else{double U=(Min[I]-A[I])/D[I],V=(Max[I]-A[I])/D[I];if(U>V)Swap(U,V);T0=FMath::Max(T0,U);T1=FMath::Min(T1,V);if(T0>T1)return false;}}return true;
    }
}
bool Mission::FSpec::Load(FString& Error)
{
    FString Text;TSharedPtr<FJsonObject> J;if(!FFileHelper::LoadFileToString(Text,*(FPaths::ProjectConfigDir()/TEXT("Mission.json")))||!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),J)||!J){Error=TEXT("Cannot read Mission.json");return false;}
    struct F{const TCHAR* Key;double* Value;};F Fields[]={{TEXT("cell_m"),&Cell},{TEXT("front_m"),&Front},{TEXT("rear_m"),&Rear},{TEXT("half_width_m"),&HalfWidth},{TEXT("clearance_m"),&Margin},{TEXT("empty_speed_mps"),&EmptySpeed},{TEXT("loaded_speed_mps"),&LoadedSpeed},{TEXT("localization_max_age_s"),&FixAge},{TEXT("observation_ttl_s"),&ObservationTtl},{TEXT("replan_s"),&ReplanS},{TEXT("command_lease_s"),&LeaseS},{TEXT("navigation_timeout_s"),&NavTimeout},{TEXT("mission_timeout_s"),&Timeout}};
    for(auto& F:Fields)if(!J->TryGetNumberField(F.Key,*F.Value)||!FMath::IsFinite(*F.Value)||*F.Value<=0){Error=TEXT("Invalid mission coefficient");return false;}
    const TArray<TSharedPtr<FJsonValue>>* A=nullptr;auto Pair=[&](const TCHAR* K,FVector2D& P){if(!J->TryGetArrayField(K,A)||A->Num()!=2)return false;double X,Y;if(!(*A)[0]->TryGetNumber(X)||!(*A)[1]->TryGetNumber(Y)||!FMath::IsFinite(X)||!FMath::IsFinite(Y))return false;P=FVector2D(X,Y);return true;};
    double N;if(!Pair(TEXT("pickup_approach_m"),Pickup)||!Pair(TEXT("dropoff_approach_m"),Dropoff)||!J->TryGetArrayField(TEXT("bounds_m"),A)||A->Num()!=4){Error=TEXT("Invalid mission stations/bounds");return false;}
    double V[4];for(int I=0;I<4;++I)if(!(*A)[I]->TryGetNumber(V[I])||!FMath::IsFinite(V[I])||FMath::Abs(V[I])>100){Error=TEXT("Invalid work area");return false;}Bounds=FBox2D(FVector2D(V[0],V[1]),FVector2D(V[2],V[3]));
    if(!J->TryGetNumberField(TEXT("max_pickup_retries"),N)||N<0||N>2||N!=FMath::FloorToDouble(N)){Error=TEXT("Invalid retry limit");return false;}MaxRetries=int(N);
    const auto Size=Bounds.GetSize();if(Cell<.15||Cell>.3||Size.X<5||Size.Y<5||Size.X*Size.Y/(Cell*Cell)>20000||Front<2.05||Front>2.4||Rear<.78||Rear>1||HalfWidth<.65||HalfWidth>.85||Margin<.18||Margin>.4||EmptySpeed>.3||LoadedSpeed>.2||FixAge>.2+2||ObservationTtl<1||ObservationTtl>3||ReplanS<.5||ReplanS>2||LeaseS>.2||NavTimeout>180||Timeout>900||!Bounds.IsInside(Pickup)||!Bounds.IsInside(Dropoff)){Error=TEXT("Mission specification unsupported");return false;}return true;
}
bool Mission::FPlanner::Free(FVector2D P) const
{
    if(!Spec.Bounds.IsInside(P))return false;
    const FVector2D Min=P-FVector2D(Spec.Rear+Spec.Margin,Spec.HalfWidth+Spec.Margin),Max=P+FVector2D(Spec.Front+Spec.Margin,Spec.HalfWidth+Spec.Margin);
    for(const auto& L:Map)if(SegmentBox(L.A,L.B,Min,Max))return false;
    for(FVector2D Q:Obstacles)if(Q.X>=Min.X-.10&&Q.X<=Max.X+.10&&Q.Y>=Min.Y-.10&&Q.Y<=Max.Y+.10)return false;
    return true;
}
bool Mission::FPlanner::Clear(FVector2D A,FVector2D B) const
{
    const int Steps=FMath::Max(1,FMath::CeilToInt((B-A).Size()/(Spec.Cell*.4)));
    for(int I=0;I<=Steps;++I)if(!Free(FMath::Lerp(A,B,double(I)/Steps)))return false;return true;
}
Mission::FPlan Mission::FPlanner::Plan(FVector2D Start,FVector2D Goal) const
{
    FPlan R;if(!Free(Start)){R.Reason=TEXT("start_footprint_blocked");return R;}if(!Free(Goal)){R.Reason=TEXT("goal_footprint_blocked");return R;}
    if(Clear(Start,Goal)){R.Valid=true;R.Points={Start,Goal};R.Length=(Goal-Start).Size();return R;}
    const auto Size=Spec.Bounds.GetSize();const int NX=FMath::CeilToInt(Size.X/Spec.Cell),NY=FMath::CeilToInt(Size.Y/Spec.Cell),Count=NX*NY;
    if(Count<=0||Count>20000){R.Reason=TEXT("grid_budget");return R;}
    auto Position=[&](int K){return Spec.Bounds.Min+FVector2D((K%NX+.5)*Spec.Cell,(K/NX+.5)*Spec.Cell);};
    auto Index=[&](FVector2D P){const auto D=(P-Spec.Bounds.Min)/Spec.Cell;return FMath::Clamp(FMath::FloorToInt(D.Y),0,NY-1)*NX+FMath::Clamp(FMath::FloorToInt(D.X),0,NX-1);};
    TArray<double> Cost;Cost.Init(1e30,Count);TArray<int> Parent;Parent.Init(-1,Count);TArray<int8> Block;Block.Init(-1,Count);TArray<bool> Closed;Closed.Init(false,Count);
    auto Open=[&](int K){if(Block[K]<0)Block[K]=Free(Position(K))?0:1;return Block[K]==0;};
    auto Endpoint=[&](FVector2D P){const int K=Index(P);int Best=-1;double Distance=1e9;for(int DY=-2;DY<=2;++DY)for(int DX=-2;DX<=2;++DX){const int X=K%NX+DX,Y=K/NX+DY;if(X<0||Y<0||X>=NX||Y>=NY)continue;const int N=Y*NX+X;const double D=(Position(N)-P).SizeSquared();if(D<Distance&&Open(N)&&Clear(P,Position(N))){Best=N;Distance=D;}}return Best;};
    const int First=Endpoint(Start),Last=Endpoint(Goal);if(First<0||Last<0){R.Reason=TEXT("grid_endpoint_blocked");return R;}
    struct FNode{double Score;int Id;bool operator<(const FNode& B) const{return Score>B.Score||(Score==B.Score&&Id>B.Id);}};
    std::priority_queue<FNode> Queue;Cost[First]=0;Queue.push({(Position(First)-Goal).Size(),First});
    while(!Queue.empty())
    {
        const int K=Queue.top().Id;Queue.pop();if(Closed[K])continue;Closed[K]=true;++R.Expanded;if(K==Last)break;
        for(int DY=-1;DY<=1;++DY)for(int DX=-1;DX<=1;++DX)
        {
            if(!DX&&!DY)continue;const int X=K%NX+DX,Y=K/NX+DY;if(X<0||Y<0||X>=NX||Y>=NY)continue;const int N=Y*NX+X;
            if(Closed[N]||!Open(N)||(DX&&DY&&(!Open(K+DX)||!Open(K+DY*NX)))||!Clear(Position(K),Position(N)))continue;
            const double G=Cost[K]+Spec.Cell*(DX&&DY?1.4142135623730951:1.);if(G>=Cost[N])continue;Cost[N]=G;Parent[N]=K;Queue.push({G+(Position(N)-Goal).Size(),N});
        }
    }
    if(!Closed[Last]){R.Reason=TEXT("no_route");return R;}
    TArray<FVector2D> Reverse;for(int K=Last;K>=0;K=Parent[K]){Reverse.Add(Position(K));if(K==First)break;}
    TArray<FVector2D> Raw;Raw.Add(Start);for(int I=Reverse.Num()-1;I>=0;--I)Raw.Add(Reverse[I]);Raw.Add(Goal);
    R.Points.Add(Start);int At=0;while(At<Raw.Num()-1){int Next=Raw.Num()-1;while(Next>At+1&&!Clear(Raw[At],Raw[Next]))--Next;R.Length+=(Raw[Next]-R.Points.Last()).Size();R.Points.Add(Raw[Next]);At=Next;}R.Valid=true;return R;
}
