#include "CampusFleet.h"
#include "FleetBattery.h"
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSectionRelease,"Forklift.Operations.SectionReleaseAndRetainedOccupancy",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FSectionRelease::RunTest(const FString&)
{
 CampusFleet::FTraffic T;T.Sections=true;T.Observe(TEXT("A"),FVector2D(0,0),true,0);
 TestTrue(TEXT("Current leg acquired"),T.Acquire(TEXT("A"),{FVector2D(0,0),FVector2D(20,0)},0));
 TestFalse(TEXT("Crossing denied while current route occupies it"),T.Acquire(TEXT("B"),{FVector2D(5,-10),FVector2D(5,10)},0));
 T.Observe(TEXT("A"),FVector2D(12,0),true,.1);TestTrue(TEXT("Passed tail released by atomic replacement"),T.Acquire(TEXT("A"),{FVector2D(12,0),FVector2D(20,0)},.1));
 TestTrue(TEXT("Crossing can now proceed while A continues"),T.Acquire(TEXT("B"),{FVector2D(5,-10),FVector2D(5,10)},.1));
 T.Release(TEXT("A"),true);TestFalse(TEXT("Released route does not erase actual parked footprint"),T.Acquire(TEXT("C"),{FVector2D(12,-5),FVector2D(12,5)},.2));
 T.Observe(TEXT("A"),FVector2D(100,100),false,1);TestFalse(TEXT("Stale pose keeps last known footprint, not passed placeholder"),T.Acquire(TEXT("C"),{FVector2D(12,-5),FVector2D(12,5)},1));return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSectionCycle,"Forklift.Operations.WaitCycleAndAtomicDenial",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FSectionCycle::RunTest(const FString&)
{
 CampusFleet::FTraffic T;T.Sections=true;T.Acquire(TEXT("A"),{FVector2D(0,0),FVector2D(5,0)},0);T.Acquire(TEXT("B"),{FVector2D(20,0),FVector2D(25,0)},0);
 TestFalse(TEXT("A cannot partially take B route"),T.Acquire(TEXT("A"),{FVector2D(0,0),FVector2D(25,0)},.1));
 TestEqual(TEXT("Denied request keeps old claim"),T.Claims[0].Corridors.Last().Max.X,7.3);
 T.Acquire(TEXT("B"),{FVector2D(20,0),FVector2D(0,0)},.1);auto Cycle=T.Deadlock();TestTrue(TEXT("Both participants detected"),Cycle.Num()==2&&Cycle.Contains(TEXT("A"))&&Cycle.Contains(TEXT("B")));
 T.Audit(1);TestFalse(TEXT("Unknown claim never stolen after timeout"),T.Acquire(TEXT("C"),{FVector2D(0,0),FVector2D(5,0)},1));T.Release(TEXT("A"),false);TestEqual(TEXT("Unsafe release forbidden"),T.Claims.Num(),2);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBatteryBalance,"Forklift.Operations.BatteryEnergyBalanceAndNoFreeRegeneration",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBatteryBalance::RunTest(const FString&)
{
 FleetBattery::FSpec S;S.Voltage=48;S.Ah=10;S.AuxW=100;FleetBattery::FState B;B.Init(S);const double Start=B.StoredWh;
 B.Step(S,1,3600,1800,false);TestTrue(TEXT("Motor and pump cumulative joules plus auxiliary consumed once"),FMath::Abs(B.ConsumedWh-(5500./3600.))<1e-9);
 B.Step(S,1,3600,1800,false);TestTrue(TEXT("Repeated cumulative sample only adds auxiliaries"),FMath::Abs(B.ConsumedWh-(5600./3600.))<1e-9);
 B.Step(S,1,3500,1800,false);TestTrue(TEXT("Counter reversal cannot invent free battery energy"),B.StoredWh<Start&&B.ChargedWh==0);
 TestTrue(TEXT("Conservation"),FMath::Abs(Start-B.ConsumedWh-B.StoredWh)<1e-8);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBatteryInterlock,"Forklift.Operations.ChargeHysteresisAndCriticalLatch",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBatteryInterlock::RunTest(const FString&)
{
 FleetBattery::FSpec S;S.Initial=.24;FleetBattery::FState B;B.Init(S);TestFalse(TEXT("Low SOC cannot take new transport"),B.Dispatch(S));
 B.Step(S,1,0,0,false);TestEqual(TEXT("No charge away from dock"),B.ChargedWh,0.);B.Step(S,1,0,0,true);TestTrue(TEXT("Valid dock supplies finite positive energy"),B.ChargedWh>0&&B.Charging);
 const double Charged=B.ChargedWh;B.Step(S,1,0,0,false);TestTrue(TEXT("Dock interlock opens immediately"),!B.Charging&&B.ChargedWh==Charged);
 B.StoredWh=S.CapacityWh()*.049;B.Step(S,1,0,0,false);TestTrue(TEXT("Critical energy latch"),B.CriticalLatched&&!B.Dispatch(S));B.Step(S,1,0,0,true);TestTrue(TEXT("Critical fault does not silently auto-recover"),!B.Charging);
 S.Low=S.Resume;TestFalse(TEXT("Invalid hysteresis rejected"),S.Valid());return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDiagonalWindow,"Forklift.Operations.DiagonalWindowExcludesEmptyBoundingCorner",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FDiagonalWindow::RunTest(const FString&)
{
 CampusFleet::FTraffic T;T.Sections=true;T.Observe(TEXT("parked"),FVector2D(0,6),true,0);
 TestTrue(TEXT("Far side of large diagonal bounding rectangle is not occupied route"),T.Acquire(TEXT("A"),{FVector2D(0,0),FVector2D(30,7)},0));
 T.Observe(TEXT("on_route"),FVector2D(10,2.33),true,0);TestFalse(TEXT("Actual route footprint still blocks"),T.Acquire(TEXT("A"),{FVector2D(0,0),FVector2D(30,7)},.1));return true;
}
