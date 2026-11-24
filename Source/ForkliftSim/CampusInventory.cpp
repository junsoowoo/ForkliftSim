#include "CampusInventory.h"
bool CampusInventory::FRegistry::Reserve(const FString& Id,const FString& Vehicle)
{
    auto* L=Loads.Find(Id);if(!L||Vehicle.IsEmpty()||(!L->ReservedBy.IsEmpty()&&L->ReservedBy!=Vehicle))return false;
    if(auto* S=Slots.Find(L->Slot))if(S->Blocked||(!S->ReservedBy.IsEmpty()&&S->ReservedBy!=Vehicle))return false;
    L->ReservedBy=Vehicle;return true;
}
bool CampusInventory::FRegistry::ReserveSlot(const FString& Id,const FString& Vehicle)
{
    auto* S=Slots.Find(Id);if(!S||Vehicle.IsEmpty()||S->Blocked||!S->Pallet.IsEmpty()||(!S->ReservedBy.IsEmpty()&&S->ReservedBy!=Vehicle))return false;
    S->ReservedBy=Vehicle;return true;
}
bool CampusInventory::FRegistry::Release(const FString& Id,const FString& Vehicle)
{
    auto* L=Loads.Find(Id);if(!L||L->ReservedBy!=Vehicle||L->Moving)return false;L->ReservedBy.Empty();
    for(auto& Pair:Slots)if(Pair.Value.ReservedBy==Vehicle)Pair.Value.ReservedBy.Empty();return true;
}
void CampusInventory::FRegistry::Depart(const FString& Id)
{
    auto* L=Loads.Find(Id);if(!L)return;if(auto* S=Slots.Find(L->Slot))if(S->Pallet==Id)S->Pallet.Empty();L->Slot.Empty();
}
bool CampusInventory::FRegistry::Place(const FString& Id,const FString& Slot,const FString& Vehicle)
{
    auto* L=Loads.Find(Id);auto* S=Slots.Find(Slot);
    if(!L||!S||S->Blocked||(!S->Pallet.IsEmpty()&&S->Pallet!=Id)||(!S->ReservedBy.IsEmpty()&&S->ReservedBy!=Vehicle)||(!L->ReservedBy.IsEmpty()&&L->ReservedBy!=Vehicle))return false;
    Depart(Id);S->Pallet=Id;S->ReservedBy.Empty();L->Slot=Slot;L->Moving=false;return true;
}
