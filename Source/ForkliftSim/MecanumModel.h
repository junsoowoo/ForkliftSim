#pragma once
#include "CoreMinimal.h"

// X forward, Y right, positive yaw rotates forward toward right (Unreal).
// Order FL, FR, RL, RR. Positive shaft speed rolls forward.
namespace Mecanum
{
    inline double Hand(int I) { return I==0 || I==3?1.:-1.; }
    inline void Solve(FVector2D V,double Yaw,double Wheelbase,double Track,double* Speed)
    {
        for(int I=0;I<4;++I)
        {
            const double X=(I<2?1:-1)*Wheelbase*.5,Y=(I%2==0?-1:1)*Track*.5;
            Speed[I]=V.X+Hand(I)*V.Y+Yaw*(-Y+Hand(I)*X);
        }
    }
    inline FVector Forward(const double* V,double Wheelbase,double Track)
    {
        const double K=(Wheelbase+Track)*.5;
        return FVector((V[0]+V[1]+V[2]+V[3])*.25,
            (V[0]-V[1]-V[2]+V[3])*.25,(V[0]-V[1]+V[2]-V[3])/(4*K));
    }
    inline double Desaturate(double* V,double Limit)
    {
        double Peak=0; for(int I=0;I<4;++I) Peak=FMath::Max(Peak,FMath::Abs(V[I]));
        const double Scale=Peak>Limit?Limit/Peak:1.; for(int I=0;I<4;++I) V[I]*=Scale; return Scale;
    }
}

struct FMecanumSettings
{
    int Count=12,PositionIterations=32,VelocityIterations=32,MinimumPhysicsHz=1920;
    bool bSphereContact=true,bReduced=false;
    bool bBearingIntegrator=true,bAdaptiveStep=true;
    double MaxControlStepS=1./30.;
    double WidthM=.16,RollerRadiusM=.02,RollerHalfLengthM=.075,RollerMassKg=1;
    double HubRadiusM=.105,HubWidthM=.14;
    double RollerCoulombNm=.03,RollerViscousNms=.002;
    double MaxTranslationMps=1,VelocityKp=.75,VelocityKi=.4,MaxCorrectionMps=.75;
    // Catalogue COMPARISON only; it never grants this custom wheel a rating.
    FString ComparisonModel,ComparisonUrl;
    double ComparisonLoadKg=0,ComparisonMassKg=0;
    bool Load(FString& Error);
};
