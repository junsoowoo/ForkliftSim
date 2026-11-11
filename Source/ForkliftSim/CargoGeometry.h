#pragma once
#include "CoreMinimal.h"

namespace CargoGeometry
{
    inline double Area(const TArray<FVector2D>& P)
    {
        double A=0;
        for(int I=0;I<P.Num();++I) A+=P[I].X*P[(I+1)%P.Num()].Y-P[(I+1)%P.Num()].X*P[I].Y;
        return FMath::Abs(A)*.5;
    }
    // Projected cargo bottom-face overlap with the rectangular pallet deck.
    // Geometry is a diagnostic; only contact impulses prove actual support.
    inline double OverlapFraction(TArray<FVector2D> Polygon,FVector2D HalfSize)
    {
        const double Original=Area(Polygon);
        if(Original<1e-10) return 0;
        for(int Side=0;Side<4;++Side)
        {
            const int Axis=Side/2; const double Sign=Side%2==0?1:-1;
            const double Limit=HalfSize[Axis]; TArray<FVector2D> Clipped;
            for(int I=0;I<Polygon.Num();++I)
            {
                const FVector2D A=Polygon[I],B=Polygon[(I+1)%Polygon.Num()];
                const double DA=Sign*A[Axis]-Limit, DB=Sign*B[Axis]-Limit;
                if(DA<=0) Clipped.Add(A);
                if((DA<=0)!=(DB<=0)) Clipped.Add(FMath::Lerp(A,B,DA/(DA-DB)));
            }
            Polygon=MoveTemp(Clipped);
        }
        return FMath::Clamp(Area(Polygon)/Original,0.,1.);
    }
}
