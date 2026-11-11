#include "CameraSensors.h"
#include "Misc/AutomationTest.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDepthGeometryTest,"ForkliftSim.Cameras.DepthGeometry",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FDepthGeometryTest::RunTest(const FString&)
{
    DepthCamera::FSpec S;S.SigmaBaseM=S.SigmaPerM2=S.Dropout=0;FRandomStream Random(1);
    TestTrue(TEXT("Optical center ray nearly axial"),FMath::Abs(DepthCamera::RayFactor(S,S.Width/2,S.Height/2)-1)<1e-4);
    TestTrue(TEXT("Image corner ray has longer radial distance"),DepthCamera::RayFactor(S,0,0)>1.3);
    const auto P=DepthCamera::Measure(200,1.5,S,Random);
    TestTrue(TEXT("Range uses cm-to-m and off-axis factor"),FMath::Abs(P.RangeM-3)<1e-6);
    TestTrue(TEXT("Depth preserves optical Z after radial conversion"),FMath::Abs(P.DepthM-2)<1e-6);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDepthValidityTest,"ForkliftSim.Cameras.RangeAndInvalidity",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FDepthValidityTest::RunTest(const FString&)
{
    DepthCamera::FSpec S;S.SigmaBaseM=S.SigmaPerM2=S.Dropout=0;FRandomStream Random(2);
    using V=DepthCamera::EValidity;
    TestTrue(TEXT("Too close retains explicit invalid reason"),DepthCamera::Measure(15,1,S,Random).Validity==V::TooClose);
    TestTrue(TEXT("Too far retains explicit invalid reason"),DepthCamera::Measure(500,1,S,Random).Validity==V::TooFar);
    TestTrue(TEXT("Unrendered/background is no return"),DepthCamera::Measure(0,1,S,Random).Validity==V::NoReturn);
    TestTrue(TEXT("Infinite renderer depth is no return"),DepthCamera::Measure(std::numeric_limits<double>::infinity(),1,S,Random).Validity==V::NoReturn);
    TestTrue(TEXT("Lower range endpoint is valid without noise"),DepthCamera::Measure(30,1,S,Random).Validity==V::Valid);
    TestTrue(TEXT("Upper range endpoint is valid without noise"),DepthCamera::Measure(400,1,S,Random).Validity==V::Valid);
    S.Dropout=1;TestTrue(TEXT("Missing return is distinct from zero distance"),DepthCamera::Measure(200,1,S,Random).Validity==V::Dropout);
    return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDepthNoiseTest,"ForkliftSim.Cameras.SeededNoise",EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FDepthNoiseTest::RunTest(const FString&)
{
    DepthCamera::FSpec S;S.Dropout=.01;FRandomStream A(317),B(317);double Sum=0,Sum2=0;int Count=0,Missing=0;
    for(int I=0;I<10000;++I)
    {
        const auto P=DepthCamera::Measure(200,1,S,A),Q=DepthCamera::Measure(200,1,S,B);
        if(P.Validity!=Q.Validity || P.RangeM!=Q.RangeM){AddError(TEXT("Fixed-seed replay diverged"));return false;}
        if(P.Validity==DepthCamera::EValidity::Valid){Sum+=P.RangeM;Sum2+=double(P.RangeM)*P.RangeM;++Count;}else ++Missing;
    }
    const double Mean=Sum/Count,Sigma=FMath::Sqrt(Sum2/Count-Mean*Mean);
    TestTrue(TEXT("Noise has expected zero-centered mean"),FMath::Abs(Mean-2)<.0005);
    TestTrue(TEXT("Observed sigma follows declared 7 mm model"),Sigma>.006 && Sigma<.0085);
    TestTrue(TEXT("Deterministic sample includes dropouts"),Missing>0);
    return true;
}
