#pragma once
#include "CoreMinimal.h"
struct FVehicle3DSettings;
class AStabilityLab;
namespace ForkliftModel
{
    bool Configure(FVehicle3DSettings& Settings,FString& Error,bool SelectedByMap=false);
    void Present(AStabilityLab* Lab);
}
