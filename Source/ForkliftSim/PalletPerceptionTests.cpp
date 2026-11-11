#include "PalletPerception.h"
#include "Misc/AutomationTest.h"
namespace
{
    DepthCamera::FFrame Image(bool Pallet,bool Missing=false,double Yaw=0)
    {
        DepthCamera::FFrame F;F.Sequence=1;F.CaptureS=1;F.DeliveryS=1.05;const int W=F.Spec.Width,H=F.Spec.Height;F.DepthM.SetNum(W*H);F.Validity.Init(uint8(DepthCamera::EValidity::Valid),W*H);
        const double Fx=W/(2*FMath::Tan(FMath::DegreesToRadians(F.Spec.FovDeg)*.5));const FQuat Rotation=FRotator(-10,0,0).Quaternion();const FVector N(FMath::Cos(Yaw),FMath::Sin(Yaw),0),T(-N.Y,N.X,0);
        for(int Y=0;Y<H;++Y)for(int X=0;X<W;++X)
        {
            const FVector Ray=Rotation.RotateVector(FVector(1,(X-(W-1)*.5)/Fx,((H-1)*.5-Y)/Fx));const double Scale=1.5/FVector::DotProduct(N,Ray);const FVector P=Ray*Scale;const double U=FVector::DotProduct(T,P)-.1,V=P.Z+.25;
            const bool Opening=Pallet&&FMath::Abs(V)<.0465&&(FMath::Abs(U-.225)<.177||FMath::Abs(U+.225)<.177);
            const bool Structure=FMath::Abs(U)<.5055&&V>-.06&&V<.10;
            const int K=Y*W+X;F.DepthM[K]=Structure&&!Opening?Scale:3.;if(Missing&&Opening){F.DepthM[K]=0;F.Validity[K]=uint8(DepthCamera::EValidity::NoReturn);}
        }return F;
    }
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalletImagePositive,"ForkliftSim.PalletPerception.MeasuredDepthTemplate",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPalletImagePositive::RunTest(const FString&)
{
    PalletPerception::FSpec S;const auto D=PalletPerception::Detect(Image(true),S);TestTrue(TEXT("Known two-entry family detected from measured depth alone"),D.Valid);TestTrue(TEXT("Distance and opening centers recovered"),FMath::Abs(D.DistanceM-1.5)<.03&&FMath::Abs(D.CenterY-.1)<.035);TestTrue(TEXT("Both openings have measured bounded widths"),D.HoleWidth[0]>.25&&D.HoleWidth[1]>.25&&D.GapEvidence>=4);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalletImageNegative,"ForkliftSim.PalletPerception.RejectBoxAndMissingDepth",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPalletImageNegative::RunTest(const FString&)
{
    PalletPerception::FSpec S;TestFalse(TEXT("A solid box is not a two-opening pallet"),PalletPerception::Detect(Image(false),S).Valid);TestFalse(TEXT("No-return pixels cannot fabricate openings"),PalletPerception::Detect(Image(true,true),S).Valid);auto F=Image(true);F.Validity.Init(uint8(DepthCamera::EValidity::Dropout),F.Validity.Num());TestFalse(TEXT("Complete dropout produces no recognition"),PalletPerception::Detect(F,S).Valid);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalletInsertionFusion,"ForkliftSim.PalletPerception.InsertionRequiresIndependentEvidence",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPalletInsertionFusion::RunTest(const FString&)
{
    PalletPerception::FSpec S;PalletPerception::FDecision D;PalletPerception::FSignals I;I.Fresh=true;I.VisionFresh=true;I.Alignment=true;I.Depth[0]=I.Depth[1]=.5;
    for(bool& P:I.Presence)P=true;D.Step(I,S);I.Now=.2;D.Step(I,S);TestTrue(TEXT("Presence alone cannot establish full insertion"),D.State==TEXT("partial")&&!D.ConfirmedFull);
    I.Depth[0]=I.Depth[1]=1.08;I.UncertaintyM=.08;I.Now=.3;D.Step(I,S);TestFalse(TEXT("Uncertainty is subtracted from depth"),D.ConfirmedFull);
    I.Depth[0]=I.Depth[1]=1.16;I.Now=.4;D.Step(I,S);I.Now=.6;D.Step(I,S);TestTrue(TEXT("Bounded geometry + all four presence points + dwell confirms"),D.ConfirmedFull);
    I.Now=.61;I.Fresh=false;D.Step(I,S);TestTrue(TEXT("Stale signals invalidate immediately"),D.State==TEXT("unknown")&&!D.ConfirmedFull);return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalletSupportSemantics,"ForkliftSim.PalletPerception.LoadDoesNotProveInsertion",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPalletSupportSemantics::RunTest(const FString&)
{
    PalletPerception::FSpec S;PalletPerception::FDecision D;PalletPerception::FSignals I;I.Fresh=true;I.Loaded=true;D.Step(I,S);I.Now=.2;D.Step(I,S);TestTrue(TEXT("Two-sided load reports support, not full geometry"),D.State==TEXT("supported")&&!D.ConfirmedFull);I.Now=.3;I.UnilateralLoad=true;D.Step(I,S);TestTrue(TEXT("Unilateral contact has distinct state"),D.State==TEXT("uneven_support"));I.Loaded=I.UnilateralLoad=false;I.Alignment=false;I.VisionFresh=true;I.Now=.4;D.Step(I,S);TestTrue(TEXT("Misalignment is distinct and immediate"),D.State==TEXT("misaligned"));return true;
}
