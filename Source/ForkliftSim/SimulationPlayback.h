#pragma once
#include "CoreMinimal.h"
class UWorld;class UInputComponent;
namespace SimulationPlayback
{
 inline bool Valid(double Rate){return FMath::IsFinite(Rate)&&((Rate>=.1&&Rate<=1)||Rate==2||Rate==4);}
 // Unreal multiplies its undilated frame clamp by time dilation. Compensate
 // for fast playback so physics, controllers, sensors and leases share time.
 inline double UndilatedCap(double ControlStep,double Rate){return ControlStep/FMath::Max(1.,Rate);}
 bool Set(UWorld* World,double Rate);
 void Bind(UInputComponent* Input,UWorld* World);
 double StepCap(const UWorld* World);
 bool Limited(const UWorld* World,double FrameWall);
 double RenderLimit();
}
