#include "SimulationPlayback.h"
#include "Misc/AutomationTest.h"
#include <limits>
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlaybackRates,"Forklift.Playback.AllowedRatesAndFiniteInputs",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPlaybackRates::RunTest(const FString&){for(double R:{.1,.25,.5,1.,2.,4.})TestTrue(TEXT("Supported rate"),SimulationPlayback::Valid(R));for(double R:{0.,-.1,1.5,3.,5.,std::numeric_limits<double>::infinity()})TestFalse(TEXT("Unsupported rate"),SimulationPlayback::Valid(R));return true;}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlaybackCap,"Forklift.Playback.SharedWorldClampPreservesControlAndSlowFixtures",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPlaybackCap::RunTest(const FString&){const double Cap=1./30.;for(double R:{1.,2.,4.}){TestTrue(TEXT("Fast playback never increases control delta"),FMath::Abs(SimulationPlayback::UndilatedCap(Cap,R)*R-Cap)<1e-12);TestTrue(TEXT("Short frame can realize requested rate"),FMath::Abs(FMath::Min(.004*R,SimulationPlayback::UndilatedCap(Cap,R)*R)-.004*R)<1e-12);}TestTrue(TEXT("Historical half-speed cap retained"),FMath::Abs(SimulationPlayback::UndilatedCap(Cap,.5)*.5-Cap*.5)<1e-12);return true;}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlaybackPacing,"Forklift.Playback.FrameLimitRestoresFiniteUnlimitedAndWorldCleanup",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPlaybackPacing::RunTest(const FString&)
{
 auto* C=IConsoleManager::Get().FindConsoleVariable(TEXT("t.MaxFPS"));if(!C)return false;const float Original=C->GetFloat();
 auto Set=[&](float V){C->Set(V,EConsoleVariableFlags(C->GetFlags()&ECVF_SetByMask));};
 auto* W=UWorld::CreateWorld(EWorldType::Game,false);if(!W)return false;
 Set(60);TestTrue(TEXT("Apply 4x"),SimulationPlayback::Set(W,4));TestEqual(TEXT("Finite cap opens to 120"),C->GetFloat(),120.f);SimulationPlayback::Set(W,1);TestEqual(TEXT("1x restores original finite cap"),C->GetFloat(),60.f);
 Set(0);SimulationPlayback::Set(W,4);TestEqual(TEXT("Unlimited stays unlimited"),C->GetFloat(),0.f);SimulationPlayback::Set(W,1);TestEqual(TEXT("Unlimited restored"),C->GetFloat(),0.f);
 Set(60);SimulationPlayback::Set(W,4);W->DestroyWorld(false);TestEqual(TEXT("Play-world cleanup restores cap"),C->GetFloat(),60.f);Set(Original);return true;
}
