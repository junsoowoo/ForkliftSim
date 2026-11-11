#include "HydraulicLift.h"

double FHydraulicSettings::ViscosityCst(double T) const
{
    // Two-point Walther interpolation. Oil properties are published; plant
    // geometry and extrapolation outside 40..100 C remain design assumptions.
    const double X0=FMath::LogX(10.,313.15), X1=FMath::LogX(10.,373.15);
    const double Y0=FMath::LogX(10.,FMath::LogX(10.,Nu40Cst+.7));
    const double Y1=FMath::LogX(10.,FMath::LogX(10.,Nu100Cst+.7));
    const double B=(Y0-Y1)/(X1-X0), A=Y0+B*X0;
    return FMath::Pow(10.,FMath::Pow(10.,A-B*FMath::LogX(10.,FMath::Clamp(T,0.,120.)+273.15)))-.7;
}
void FHydraulicState::StepDetailed(const FHydraulicSettings& C,double Dt,double Target,double H,double V,double Travel,double Cap,const FHydraulicCommand& Control)
{
    const double MaxP=FMath::Min(C.ReliefPa,Cap/C.AreaM2);
    if(!bDetailedInitialized)
    {
        OilTempC=C.InitialTempC; for(double& P:LinePressurePa) P=FMath::Clamp(PressurePa,0.,MaxP);
        bDetailedInitialized=true;
    }
    const double Command=ValveRequest(C,Target,H,Travel,Control);
    Valve+=(Command-Valve)*(1-FMath::Exp(-Dt/C.ValveResponseS));
    const double A=PI*C.HoseBoreM*C.HoseBoreM*.25, Dx=C.HoseLengthM/4.;
    const double WaveMps=FMath::Sqrt(C.BulkPa/C.OilDensityKgM3);
    const int Steps=FMath::Max(1,FMath::CeilToInt(Dt/(.3*Dx/WaveMps)));
    const double D=Dt/Steps, Inertance=C.OilDensityKgM3*Dx/A;
    const double CylinderVolume=C.DeadVolumeM3+C.AreaM2*FMath::Clamp(H,0.,Travel);
    const double Volumes[5]={C.PumpVolumeM3+A*Dx*.5,A*Dx,A*Dx,A*Dx,CylinderVolume+A*Dx*.5};
    double SumPump=0,SumReturn=0,SumLeak=0,SumRelief=0,SumDc=0,SumHeat=0,SumHyd=0;
    ViscosityCst=C.ViscosityCst(OilTempC);
    const double ViscosityRatio=ViscosityCst/C.Nu40Cst;
    const double Resistance=128*(ViscosityCst*1e-6*C.OilDensityKgM3)*Dx/(PI*FMath::Pow(C.HoseBoreM,4));
    const double Friction=C.SealN*FMath::Tanh(V/.01)+C.ViscousNsPm*ViscosityRatio*V;
    LineHeatW=0;
    for(int S=0;S<Steps;++S)
    {
        const double Pump=(!Control.Direct||Control.PumpEnabled)?FMath::Max(Valve,0.)*FMath::Min(C.PumpFlowM3s,C.MotorW*C.Efficiency/FMath::Max(LinePressurePa[0],1000.)):0;
        const double Return=FMath::Max(-Valve,0.)*C.ReturnM3sSqrtPa*FMath::Sqrt(LinePressurePa[4]);
        const double Leak=C.LeakM3sPa/ViscosityRatio*LinePressurePa[4];
        const double Hyd=LinePressurePa[0]*Pump, Dc=Hyd/C.Efficiency;
        double Heat=Dc-Hyd+LinePressurePa[4]*(Return+Leak)+Friction*V, Relief=0, PipeHeat=0;
        // Staggered pressure/flow finite volumes, implicit viscous loss and
        // CFL substeps. No force is added at the wheel-road contact interface.
        for(int I=0;I<4;++I)
        {
            LineFlowM3s[I]=(LineFlowM3s[I]+D*(LinePressurePa[I]-LinePressurePa[I+1])/Inertance)/(1+D*Resistance/Inertance);
            PipeHeat+=Resistance*LineFlowM3s[I]*LineFlowM3s[I];
        }
        for(int I=0;I<5;++I)
        {
            const double In=I==0?Pump:LineFlowM3s[I-1];
            const double Out=I==4?Return+Leak+C.AreaM2*V:LineFlowM3s[I];
            const double Raw=LinePressurePa[I]+C.BulkPa/Volumes[I]*(In-Out)*D;
            // Pump/cylinder have ideal local relief. A negative gauge-pressure
            // boundary is flagged; this is NOT a two-phase cavitation model.
            const double Limit=(I==0 || I==4)?MaxP:2*MaxP;
            Relief+=FMath::Max(0.,Raw-Limit)*Volumes[I]/C.BulkPa/D;
            if(Raw<0) bPressureFloorHit=true;
            LinePressurePa[I]=FMath::Clamp(Raw,0.,Limit);
            PeakLinePressurePa=FMath::Max(PeakLinePressurePa,LinePressurePa[I]);
        }
        Heat+=PipeHeat+MaxP*Relief;
        SumPump+=Pump; SumReturn+=Return; SumLeak+=Leak; SumRelief+=Relief;
        SumDc+=Dc; SumHyd+=Hyd; SumHeat+=Heat; LineHeatW+=PipeHeat/Steps;
    }
    PumpM3s=SumPump/Steps; ReturnM3s=SumReturn/Steps; LeakM3s=SumLeak/Steps; ReliefM3s=SumRelief/Steps;
    DcW=SumDc/Steps; HydraulicW=SumHyd/Steps; HeatW=SumHeat/Steps;
    PressurePa=LinePressurePa[4]; PumpPressurePa=LinePressurePa[0];
    ForceN=C.AreaM2*PressurePa-Friction; MechanicalW=ForceN*V;
    OilTempC+=(HeatW-C.CoolingWpK*(OilTempC-C.AmbientC))*Dt/C.ThermalMassJpK;
    DcJ+=DcW*Dt; MechanicalJ+=MechanicalW*Dt; HeatJ+=HeatW*Dt;
}
