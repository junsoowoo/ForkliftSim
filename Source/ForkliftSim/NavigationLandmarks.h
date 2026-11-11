#pragma once
#include "SensorDrive.h"
namespace NavigationLandmarks
{
    struct FBox {FVector Center,Size;};
    bool Load(TArray<FBox>& Boxes,FString& Error);
    void AppendSurvey(const TArray<FBox>& Boxes,TArray<SensorDrive::FSegment>& Map);
}
