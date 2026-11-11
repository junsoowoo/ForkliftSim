#pragma once
#include "CoreMinimal.h"

// Centimetres, unit world vectors. Exact lateral surface of a circular cylinder;
// retain the axial coordinate so the two shoulder contacts can carry roll load.
// This does not set velocities, forces, friction, load or stability limits.
namespace WheelContactGeometry
{
    // Exact support point of a finite cylinder against a plane. Unlike the
    // legacy tread-only projection this also covers the end cap and tilted rim.
    inline FVector CylinderSupport(const FVector& Center,const FVector& Axis,const FVector& N,
        const FVector& OldPoint,double Radius,double HalfWidth)
    {
        const double A=FMath::Clamp(FVector::DotProduct(Axis,N),-1.,1.);
        const double U=FMath::Abs(A)<1e-10?FMath::Clamp(FVector::DotProduct(OldPoint-Center,Axis),-HalfWidth,HalfWidth):-FMath::Sign(A)*HalfWidth;
        FVector Radial=-N+Axis*A;
        if(!Radial.Normalize())Radial=FVector::VectorPlaneProject(OldPoint-Center,Axis).GetClampedToMaxSize(Radius);
        else Radial*=Radius;
        return Center+Axis*U+Radial;
    }
    // Finite cylinder versus an axle-skewed box edge. Maximize the supporting
    // separation over the edge's OUTWARD normal cone, never through the solid.
    // Faces/corners outside the finite edge remain in the normal narrow phase.
    inline bool ProjectCylinderEdge(const FVector& Center,const FVector& Axis,const FVector& OldRoad,
        const FVector& Min,const FVector& Max,double Radius,double HalfWidth,
        FVector& WheelPoint,FVector& RoadPoint,FVector& Normal)
    {
        int Faces[2]={};double Signs[2]={};int Count=0,Edge=INDEX_NONE;
        for(int I=0;I<3;++I)
        {
            const bool Lo=FMath::Abs(OldRoad[I]-Min[I])<.25,Hi=FMath::Abs(OldRoad[I]-Max[I])<.25;
            if(Lo||Hi){if(Count==2)return false;Faces[Count]=I;Signs[Count++]=Lo?-1.:1.;}
            else Edge=I;
        }
        if(Count!=2||Edge==INDEX_NONE||Radius<=0||HalfWidth<=0)return false;
        FVector P=Center,F0=FVector::ZeroVector,F1=FVector::ZeroVector;
        P[Faces[0]]=Signs[0]>0?Max[Faces[0]]:Min[Faces[0]];P[Faces[1]]=Signs[1]>0?Max[Faces[1]]:Min[Faces[1]];
        F0[Faces[0]]=Signs[0];F1[Faces[1]]=Signs[1];
        auto Direction=[&](double T){return F0*FMath::Cos(T)+F1*FMath::Sin(T);};
        auto Separation=[&](double T)
        {
            const FVector N=Direction(T);const double A=FMath::Clamp(FVector::DotProduct(Axis,N),-1.,1.);
            return FVector::DotProduct(Center-P,N)-HalfWidth*FMath::Abs(A)-Radius*FMath::Sqrt(FMath::Max(0.,1-A*A));
        };
        constexpr int Samples=32;const double Step=(UE_DOUBLE_PI*.5)/Samples;
        double BestT=0,Best=Separation(0);
        auto Keep=[&](double T){const double S=Separation(T);if(S>Best){Best=S;BestT=T;}};
        for(int I=1;I<=Samples;++I)Keep(I*Step);
        double L=FMath::Max(0.,BestT-Step),H=FMath::Min(UE_DOUBLE_PI*.5,BestT+Step);
        for(int I=0;I<32;++I){const double A=L+(H-L)/3,B=H-(H-L)/3;if(Separation(A)<Separation(B))L=A;else H=B;}
        Keep((L+H)*.5);
        // An exactly axial cap normal is a cusp of the support function.
        const double A0=FVector::DotProduct(Axis,F0),A1=FVector::DotProduct(Axis,F1);
        for(double Sign:{-1.,1.})if(A0*Sign>=0&&A1*Sign>=0)Keep(FMath::Atan2(A1*Sign,A0*Sign));
        for(double Sign:{-1.,1.})if(-A1*Sign>=0&&A0*Sign>=0)Keep(FMath::Atan2(A0*Sign,-A1*Sign));
        Normal=Direction(BestT);WheelPoint=CylinderSupport(Center,Axis,Normal,P,Radius,HalfWidth);RoadPoint=P;RoadPoint[Edge]=WheelPoint[Edge];
        // A native edge normal can be wrong when the nearest feature is the
        // top or side face. Keep that valid finite FACE witness, rather than
        // falling back to the very edge normal that caused the correction.
        if(BestT<1e-7 || UE_DOUBLE_PI*.5-BestT<1e-7)
        {
            const int Face=BestT<1e-7?0:1;Normal=Face==0?F0:F1;
            WheelPoint=CylinderSupport(Center,Axis,Normal,P,Radius,HalfWidth);RoadPoint=WheelPoint;RoadPoint[Faces[Face]]=P[Faces[Face]];
            for(int I=0;I<3;++I)if(I!=Faces[Face] && (RoadPoint[I]<Min[I]-1e-5||RoadPoint[I]>Max[I]+1e-5))return false;
            return true;
        }
        if(FMath::Abs(FVector::DotProduct(Axis,Normal))<1e-8)
        {
            FVector ProjectedAxis=Axis;ProjectedAxis[Edge]=0;
            if(ProjectedAxis.SizeSquared()>1e-10)
            {
                const double Along=FMath::Clamp(FVector::DotProduct(P-Center,ProjectedAxis)/ProjectedAxis.SizeSquared(),-HalfWidth,HalfWidth);
                WheelPoint=Center+Axis*Along-Normal*Radius;RoadPoint[Edge]=WheelPoint[Edge];
            }
        }
        if(RoadPoint[Edge]<Min[Edge]+.1||RoadPoint[Edge]>Max[Edge]-.1)return false;
        const FVector Delta=WheelPoint-RoadPoint;
        // Interior-cone witnesses must agree along the finite edge.
        return (Delta-Normal*FVector::DotProduct(Delta,Normal)).Size()<.025;
    }
    // Circular cross-section against a box edge parallel to the axle (exact
    // when aligned; near-parallel tolerance |axis.edge| >= .99999).
    // Work in box coordinates. Other orientations/caps/corners stay native.
    inline bool ProjectEdge(const FVector& Center,const FVector& Axis,const FVector& OldRoad,
        const FVector& Min,const FVector& Max,double Radius,double HalfWidth,
        FVector& WheelPoint,FVector& RoadPoint,FVector& Normal)
    {
        if(Radius<=0 || HalfWidth<=0) return false;
        int Edge=0; for(int I=1;I<3;++I) if(FMath::Abs(Axis[I])>FMath::Abs(Axis[Edge])) Edge=I;
        if(FMath::Abs(Axis[Edge])<.99999) return false;
        // Exactly two outside coordinates identify an edge rather than a face
        // or vertex. Require the native candidate to identify that SAME edge.
        FVector Nearest=Center; int Outside=0;
        for(int I=0;I<3;++I)
        {
            if(I==Edge) continue;
            if(Center[I]<Min[I]) Nearest[I]=Min[I];
            else if(Center[I]>Max[I]) Nearest[I]=Max[I];
            else return false;
            if(FMath::Abs(OldRoad[I]-Nearest[I])>.25) return false;
            ++Outside;
        }
        if(Outside!=2 || Center[Edge]-HalfWidth<Min[Edge]+.1 || Center[Edge]+HalfWidth>Max[Edge]-.1) return false;
        Normal=Center-Nearest; Normal[Edge]=0;
        if(!Normal.Normalize()) return false;
        const double Along=FMath::Clamp(FVector::DotProduct(OldRoad-Center,Axis),-HalfWidth,HalfWidth);
        WheelPoint=Center-Radius*Normal+Along*Axis;
        RoadPoint=Nearest;
        // Slight axle skew is accepted only within the stated tolerance; keep
        // the road point on the actual edge and normal perpendicular to EDGE.
        RoadPoint[Edge]=WheelPoint[Edge];
        return true;
    }
    inline bool Project(const FVector& Center,const FVector& Axis,const FVector& RoadNormal,
        const FVector& OldPoint,double Radius,double HalfWidth,FVector& Point)
    {
        const double AxialNormal=FVector::DotProduct(Axis,RoadNormal);
        if(FMath::Abs(AxialNormal)>.95 || Radius<=0 || HalfWidth<=0) return false;
        const FVector Radial=(-RoadNormal+Axis*AxialNormal).GetSafeNormal();
        const double Along=FMath::Clamp(FVector::DotProduct(OldPoint-Center,Axis),-HalfWidth,HalfWidth);
        Point=Center+Radius*Radial+Along*Axis;
        return true;
    }
}
