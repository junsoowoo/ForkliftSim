#pragma once
#include "CoreMinimal.h"

// Warehouse bookkeeping. Never supplied as a camera detection or a localization fix.
namespace CampusInventory
{
    struct FSlot {FString Id,Pallet,ReservedBy;FVector CenterM;bool Blocked=false;};
    struct FLoad {FString Id,Slot,ReservedBy;FVector CenterM;bool Physical=false,Moving=false;};
    struct FRegistry
    {
        TMap<FString,FSlot> Slots;TMap<FString,FLoad> Loads;
        bool Reserve(const FString& Load,const FString& Vehicle);
        bool ReserveSlot(const FString& Slot,const FString& Vehicle);
        bool Release(const FString& Load,const FString& Vehicle);
        bool Place(const FString& Load,const FString& Slot,const FString& Vehicle);
        void Depart(const FString& Load);
    };
}
