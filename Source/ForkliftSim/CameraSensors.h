#pragma once
#include "CoreMinimal.h"
#include "Dom/JsonObject.h"

class AStabilityLab;
class AHUD;
class UCanvas;
struct FCameraSensorState;

namespace DepthCamera
{
    enum class EValidity : uint8 { Valid, NoReturn, TooClose, TooFar, Dropout };
    struct FSpec
    {
        int Width=320,Height=180,Seed=317;
        double Hz=8,FovDeg=85,MinM=.3,MaxM=4,SigmaBaseM=.003,SigmaPerM2=.001,QuantumM=.001,Dropout=.002,LatencyS=.05;
    };
    struct FSample { float RangeM=0,DepthM=0; EValidity Validity=EValidity::NoReturn; };
    // Consumer view contains measured pixels and calibrated intrinsics only.
    // It deliberately excludes renderer/world pose and ideal depth diagnostics.
    struct FFrame {FSpec Spec;uint64 Sequence=0;double CaptureS=0,DeliveryS=0;TArray<float> DepthM;TArray<uint8> Validity;};
    // The renderer supplies optical-axis Z in cm; ToF errors are applied to
    // radial range. The published depth image remains optical Z in metres.
    inline double RayFactor(const FSpec& S,int X,int Y)
    {
        const double T=FMath::Tan(FMath::DegreesToRadians(S.FovDeg)*.5);
        const double U=(2*(X+.5)/S.Width-1)*T,V=(1-2*(Y+.5)/S.Height)*T*S.Height/S.Width;
        return FMath::Sqrt(1+U*U+V*V);
    }
    inline FSample Measure(double OpticalCm,double Factor,const FSpec& S,FRandomStream& Random)
    {
        FSample Out;
        if(!FMath::IsFinite(OpticalCm) || OpticalCm<=0 || OpticalCm>1e8)return Out;
        const double Range=OpticalCm*.01*Factor;
        if(Range<S.MinM){Out.Validity=EValidity::TooClose;return Out;}
        if(Range>S.MaxM){Out.Validity=EValidity::TooFar;return Out;}
        if(Random.FRand()<S.Dropout){Out.Validity=EValidity::Dropout;return Out;}
        const double Gaussian=FMath::Sqrt(-2*FMath::Loge(FMath::Max(1e-7,double(Random.FRand()))))*FMath::Cos(2*PI*Random.FRand());
        const double Noisy=Range+Gaussian*(S.SigmaBaseM+S.SigmaPerM2*Range*Range);
        const double Quantized=FMath::RoundToDouble(Noisy/S.QuantumM)*S.QuantumM;
        if(Quantized<S.MinM){Out.Validity=EValidity::TooClose;return Out;}
        if(Quantized>S.MaxM){Out.Validity=EValidity::TooFar;return Out;}
        Out.Validity=EValidity::Valid;Out.RangeM=Quantized;Out.DepthM=Quantized/Factor;return Out;
    }
    inline const TCHAR* Name(EValidity V)
    {
        switch(V){case EValidity::Valid:return TEXT("valid");case EValidity::TooClose:return TEXT("too_close");case EValidity::TooFar:return TEXT("too_far");case EValidity::Dropout:return TEXT("dropout");default:return TEXT("no_return");}
    }
}

// Non-UObject state owns asynchronous readbacks. Render commands retain their
// channel state across reset; actor UPROPERTYs own the captures and textures.
struct FCameraSensorRig
{
    ~FCameraSensorRig();
    TSharedPtr<FCameraSensorState> State;
    static TSharedPtr<FCameraSensorRig> Create(AStabilityLab& Lab,FString& Error);
    void Update(AStabilityLab& Lab);
    bool ReadFront(DepthCamera::FFrame& Frame,uint64 AfterSequence) const;
    void Draw(AStabilityLab& Lab,AHUD* HUD,UCanvas* Canvas);
    TSharedRef<FJsonObject> Telemetry(double NowS,bool Enabled) const;
    void WriteReport(const FString& Folder,double NowS,bool Enabled) const;
};
