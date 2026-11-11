#include "DrivetrainModel.h"
#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDrivetrainTest,"ForkliftSim.Drivetrain.EnvelopeEfficiencyResponse",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FDrivetrainTest::RunTest(const FString& Parameters)
{
    FDrivetrainSettings C; TestTrue(TEXT("Default actuator spec valid"),C.Valid());
    C.Drive.GearEfficiency=0; TestFalse(TEXT("Zero gear efficiency rejected"),C.Valid());
    C=FDrivetrainSettings(); auto S=C.Drive; S.ResponseS=0;
    for(double Omega:{-30.,-10.,-1.,0.,1.,10.,30.}) for(double Request:{-450.,450.})
    {
        double State=0; const auto R=ElectricDrive::Step(Request,Omega,S,450,1./60,true,State);
        TestTrue(TEXT("Output safety torque bound"),FMath::Abs(R.OutputNm)<=450.000001);
        TestTrue(TEXT("Shaft torque bound"),FMath::Abs(R.MotorNm)<=S.PeakNm+1e-6);
        TestTrue(TEXT("Mechanical power bound in both directions"),FMath::Abs(R.ShaftW)<=S.PowerW+1e-6);
        TestTrue(TEXT("Passive conversion losses incl regeneration"),R.LossW>=0 && FMath::Abs(R.DcW-R.OutputNm*Omega-R.LossW)<1e-6);
        if(Request*Omega>0 && FMath::Abs(R.MotorRpm)>=S.MaxRpm) TestEqual(TEXT("No motoring above max RPM"),R.OutputNm,0.);
    }
    TestTrue(TEXT("Torque decreases at high speed"),ElectricDrive::OutputLimit(S,10,true,450)<ElectricDrive::OutputLimit(S,1,true,450));
    const double BackDrivenOmega=C.Steer.MaxRpm*PI/30/C.Steer.Ratio*1.5;
    TestEqual(TEXT("Overspeed steering cannot motor faster"),ElectricDrive::OutputLimit(C.Steer,BackDrivenOmega,true,5000),0.);
    TestTrue(TEXT("Overspeed steering retains regenerative brake authority"),ElectricDrive::OutputLimit(C.Steer,BackDrivenOmega,false,5000)>0);
    for(double Dt:{1./30,1./60,1./120})
    {
        double V=0; for(int I=0;I<FMath::RoundToInt(1/Dt);++I) V=ElectricDrive::Lag(V,1,Dt,.1);
        TestTrue(TEXT("Exact first-order response independent of tick interval"),FMath::Abs(V-(1-FMath::Exp(-10.)))<1e-9);
    }
    TestEqual(TEXT("Gear holds inside backlash gap"),ElectricDrive::Backlash(0,.05,.2),0.);
    TestTrue(TEXT("Forward flank"),FMath::Abs(ElectricDrive::Backlash(0,1,.2)-.9)<1e-9);
    TestTrue(TEXT("Reversal traverses gap"),FMath::Abs(ElectricDrive::Backlash(.9,.85,.2)-.9)<1e-9);
    double State=10; const auto Off=ElectricDrive::Step(450,1,S,450,.1,false,State);
    TestEqual(TEXT("Neutral cuts torque immediately"),Off.OutputNm,0.); TestEqual(TEXT("Neutral clears lag state"),State,0.);
    return true;
}
#endif
