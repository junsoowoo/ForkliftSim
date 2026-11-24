#pragma once
#include "CoreMinimal.h"
// Energy/SOC model. Constant nominal voltage; no cell chemistry or BMS emulation.
namespace FleetBattery
{
struct FSpec
{
 bool Enabled=true;double Voltage=48,Ah=465,Initial=.8,Low=.25,Resume=.8,Critical=.05,AuxW=180,ChargeW=4000,Efficiency=.9;
 bool Valid() const;
 double CapacityWh() const{return Voltage*Ah;}
};
struct FState
{
 double StoredWh=0,ConsumedWh=0,ChargedWh=0,UnservedWh=0,PowerW=0,ChargeSeconds=0;
 double LastMotorJ=0,LastPumpJ=0;bool Charging=false,CriticalLatched=false;int Sessions=0;FString Mode=TEXT("available"),Reason;
 void Init(const FSpec& S){*this=FState();StoredWh=S.CapacityWh()*S.Initial;}
 double Soc(const FSpec& S) const{return StoredWh/S.CapacityWh();}
 void Step(const FSpec& S,double Dt,double MotorJ,double PumpJ,bool Docked);
 bool Dispatch(const FSpec& S) const{return !S.Enabled||(!CriticalLatched&&!Charging&&Soc(S)>S.Low);}
};
}
