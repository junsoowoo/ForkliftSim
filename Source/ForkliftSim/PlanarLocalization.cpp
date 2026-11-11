#include "SensorDrive.h"
namespace
{
    bool Solve(double H[3][3],double B[3],double X[3],double& Quality)
    {
        const double Scale=FMath::Max3(H[0][0],H[1][1],H[2][2]);if(Scale<=0)return false;double A[3][4];Quality=1;
        for(int I=0;I<3;++I){for(int J=0;J<3;++J)A[I][J]=H[I][J]/Scale;A[I][3]=B[I]/Scale;}
        for(int I=0;I<3;++I){int P=I;for(int K=I+1;K<3;++K)if(FMath::Abs(A[K][I])>FMath::Abs(A[P][I]))P=K;const double Pivot=FMath::Abs(A[P][I]);Quality=FMath::Min(Quality,Pivot);if(Pivot<1e-5)return false;for(int J=I;J<4;++J)Swap(A[P][J],A[I][J]);const double D=A[I][I];for(int J=I;J<4;++J)A[I][J]/=D;for(int K=0;K<3;++K)if(K!=I){const double M=A[K][I];for(int J=I;J<4;++J)A[K][J]-=M*A[I][J];}}
        for(int I=0;I<3;++I)X[I]=A[I][3];return true;
    }
}
SensorDrive::FMatch SensorDrive::Match(const TArray<FVector2D>& Points,const TArray<FSegment>& Map,FPose Initial)
{
    FMatch R;R.Pose=Initial;if(Points.Num()<24||Map.Num()<3)return R;
    for(int Iter=0;Iter<7;++Iter)
    {
        double H[3][3]={},B[3]={},Error=0;int Count=0;const double C=FMath::Cos(R.Pose.Yaw),S=FMath::Sin(R.Pose.Yaw);
        for(const auto& Local:Points)
        {
            const FVector2D Rot(C*Local.X-S*Local.Y,S*Local.X+C*Local.Y),P=Rot+FVector2D(R.Pose.X,R.Pose.Y);
            double Best=.35*.35;FVector2D N=FVector2D::ZeroVector,Q=FVector2D::ZeroVector;
            for(const auto& Line:Map){const auto V=Line.B-Line.A;const double L=V.SizeSquared();if(L<1e-8)continue;const double T=FVector2D::DotProduct(P-Line.A,V)/L;if(T<0||T>1)continue;const FVector2D Foot=Line.A+V*T;const double D=(P-Foot).SizeSquared();if(D<Best){Best=D;Q=Foot;N=FVector2D(-V.Y,V.X)/FMath::Sqrt(L);}}
            if(N.IsNearlyZero())continue;const double E=FVector2D::DotProduct(P-Q,N),W=FMath::Abs(E)>.05?.05/FMath::Abs(E):1;
            const double J[3]={N.X,N.Y,-N.X*Rot.Y+N.Y*Rot.X};for(int I=0;I<3;++I){B[I]-=W*J[I]*E;for(int K=0;K<3;++K)H[I][K]+=W*J[I]*J[K];}Error+=E*E;++Count;
        }
        R.Points=Count;R.Rms=Count?FMath::Sqrt(Error/Count):0;double D[3]={};if(Count<24||double(Count)/Points.Num()<.18||!Solve(H,B,D,R.Observability))return R;
        if(Iter==6){R.Accepted=R.Rms<.07&&FVector2D(R.Pose.X-Initial.X,R.Pose.Y-Initial.Y).Size()<.4&&FMath::Abs(FMath::UnwindRadians(R.Pose.Yaw-Initial.Yaw))<.12;return R;}
        const double Limit=FMath::Max3(1.,FVector2D(D[0],D[1]).Size()/.1,FMath::Abs(D[2])/.04);R.Pose.X+=D[0]/Limit;R.Pose.Y+=D[1]/Limit;R.Pose.Yaw=FMath::UnwindRadians(R.Pose.Yaw+D[2]/Limit);
    }
    return R;
}
