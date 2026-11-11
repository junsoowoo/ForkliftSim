#pragma once
#include "Chaos/SimCallbackObject.h"
#include "HydraulicLift.h"
#include "DynamicsSensors.h"

struct FHydraulicInput : Chaos::FSimCallbackInput
{
    FHydraulicSettings Settings;
    FHydraulicCommand Command;double CommandValidUntilS=0;
    bool bPressureSensor=false;double PressureSamplePeriodS=.01;
    int32 ChassisId=INDEX_NONE, CarriageId=INDEX_NONE, MastId=INDEX_NONE;
    double MastBaseZCm=0;
    double TargetM=0, BaseZCm=0, TravelM=1.5, ForceCapN=15000, InitialPressurePa=0;
    void Reset() { Command=FHydraulicCommand();CommandValidUntilS=0;bPressureSensor=false;PressureSamplePeriodS=.01;ChassisId=CarriageId=MastId=INDEX_NONE; }
};
struct FHydraulicOutput : Chaos::FSimCallbackOutput
{
    FHydraulicState State;
    bool CommandExpired=false;
    TArray<DynamicsSensor::FTruth> SensorSamples;
    double HeightM=0, VelocityMps=0, CompletedS=0;
    double PeakPressurePa=0, PeakDcW=0, ReliefVolumeM3=0;
    void Reset() { CommandExpired=false;SensorSamples.Reset();State=FHydraulicState(); HeightM=VelocityMps=CompletedS=0; PeakPressurePa=PeakDcW=ReliefVolumeM3=0; }
};
// Only POD inputs / particle handles on PT. No actor or component access.
class FHydraulicCallback : public Chaos::TSimCallbackObject<FHydraulicInput,FHydraulicOutput,
    Chaos::ESimCallbackOptions::Presimulate | Chaos::ESimCallbackOptions::PreIntegrate | Chaos::ESimCallbackOptions::PostIntegrate>
{
    FHydraulicState State;
    bool bInitialized=false;
    double SensorClockS=0,NextSensorS=0;
    Chaos::FPBDRigidParticleHandle* Bodies[3]={nullptr,nullptr,nullptr};
    Chaos::FVec3 AddedA[3]={Chaos::FVec3(0),Chaos::FVec3(0),Chaos::FVec3(0)}, AddedW[3]={Chaos::FVec3(0),Chaos::FVec3(0),Chaos::FVec3(0)};
    virtual void OnPreSimulate_Internal() override {}
    virtual void OnPreIntegrate_Internal() override;
    virtual void OnPostIntegrate_Internal() override;
};
