#include "FleetBattery.h"
bool FleetBattery::FSpec::Valid() const
{
 for(double V:{Voltage,Ah,Initial,Low,Resume,Critical,AuxW,ChargeW,Efficiency})if(!FMath::IsFinite(V))return false;
 return Voltage>=12&&Voltage<=100&&Ah>=1&&Ah<=2000&&Initial>=0&&Initial<=1&&Critical>=0&&Critical<Low&&Low<Resume&&Resume<=1&&AuxW>=0&&AuxW<=2000&&ChargeW*Efficiency>AuxW&&ChargeW<=30000&&Efficiency>.1&&Efficiency<=1;
}
void FleetBattery::FState::Step(const FSpec& S,double Dt,double MotorJ,double PumpJ,bool Docked)
{
 const double Motor=FMath::Max(0.,MotorJ-LastMotorJ),Pump=FMath::Max(0.,PumpJ-LastPumpJ);LastMotorJ=MotorJ;LastPumpJ=PumpJ;
 if(!S.Enabled){Charging=false;Mode=TEXT("disabled");return;}if(Dt<=0||!FMath::IsFinite(Dt))return;
 const double Demand=(Motor+Pump+S.AuxW*Dt)/3600.;PowerW=(Motor+Pump)/Dt+S.AuxW;
 Charging=Docked&&!CriticalLatched&&Soc(S)<S.Resume;
 const double Input=Charging?S.ChargeW*S.Efficiency*Dt/3600.:0;
 const double Accepted=FMath::Min(Input,FMath::Max(0.,S.CapacityWh()-StoredWh+Demand));
 ChargedWh+=Accepted;ConsumedWh+=Demand;const double Balance=StoredWh+Accepted-Demand;UnservedWh+=FMath::Max(0.,-Balance);StoredWh=FMath::Clamp(Balance,0.,S.CapacityWh());
 if(Charging)ChargeSeconds+=Dt;
 if(Soc(S)<=S.Critical){CriticalLatched=true;Reason=TEXT("critical_soc_manual_recovery_required");}
 Mode=CriticalLatched?TEXT("critical_hold"):Charging?TEXT("charging"):Soc(S)<=S.Low?TEXT("charge_required"):TEXT("available");
}
