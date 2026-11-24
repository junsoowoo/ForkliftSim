#include "SimulationPlayback.h"
#include "StabilityLab.h"
#include "Components/InputComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/WorldSettings.h"
#include "HAL/IConsoleManager.h"
namespace
{
 struct FPacing {float Original=60,Applied=60;};
 TMap<TWeakObjectPtr<UWorld>,FPacing> Pacing;
 void SetLimit(IConsoleVariable* C,float Value){C->Set(Value,EConsoleVariableFlags(C->GetFlags()&ECVF_SetByMask));}
 void RestorePacing(UWorld* W,bool,bool)
 {if(auto* S=Pacing.Find(W)){if(auto* C=IConsoleManager::Get().FindConsoleVariable(TEXT("t.MaxFPS")))if(FMath::IsNearlyEqual(C->GetFloat(),S->Applied,.001f))SetLimit(C,S->Original);Pacing.Remove(W);}}
 void UpdatePacing(UWorld* W,double Rate,double Cap)
 {
  static FDelegateHandle Cleanup=FWorldDelegates::OnWorldCleanup.AddStatic(&RestorePacing);
  auto* C=IConsoleManager::Get().FindConsoleVariable(TEXT("t.MaxFPS"));if(!C)return;
  if(Rate<=1){RestorePacing(W,false,false);return;}
  if(!Pacing.Contains(W))Pacing.Add(W,{C->GetFloat(),C->GetFloat()});auto& S=Pacing[W];
  S.Applied=S.Original<=0?0:FMath::Max(S.Original,float(FMath::CeilToInt(Rate/Cap-1e-6)));SetLimit(C,S.Applied);
 }
}
bool SimulationPlayback::Set(UWorld* World,double Rate)
{
 if(!World||!Valid(Rate))return false;double Cap=1./30.;
 for(TActorIterator<AStabilityLab> I(World);I;++I)if(I->Settings.bMecanum)Cap=FMath::Min(Cap,I->Settings.Mecanum.MaxControlStepS);
 auto* Settings=World->GetWorldSettings();Settings->SetTimeDilation(Rate);Settings->MaxUndilatedFrameTime=UndilatedCap(Cap,Rate);
 UpdatePacing(World,Rate,Cap);
 UE_LOG(LogTemp,Display,TEXT("PLAYBACK requested=%.2fx max_world_step=%.9f bounded_physics_and_control=1"),Rate,StepCap(World));return true;
}
void SimulationPlayback::Bind(UInputComponent* Input,UWorld* World)
{
 if(!Input)return;const FKey Keys[]={EKeys::F5,EKeys::F6,EKeys::F7};const double Rates[]={1,2,4};
 for(int I=0;I<3;++I){FInputKeyBinding B(FInputChord(Keys[I]),IE_Pressed);const TWeakObjectPtr<UWorld> Weak=World;const double Rate=Rates[I];B.KeyDelegate.GetDelegateForManualSet().BindLambda([Weak,Rate](){if(Weak.IsValid())Set(Weak.Get(),Rate);});Input->KeyBindings.Add(MoveTemp(B));}
}
double SimulationPlayback::StepCap(const UWorld* World){const auto* S=World->GetWorldSettings();return S->MaxUndilatedFrameTime*S->GetEffectiveTimeDilation();}
bool SimulationPlayback::Limited(const UWorld* World,double FrameWall){return FrameWall>0&&FrameWall*World->GetWorldSettings()->GetEffectiveTimeDilation()>StepCap(World)*1.05;}
double SimulationPlayback::RenderLimit(){const auto* C=IConsoleManager::Get().FindConsoleVariable(TEXT("t.MaxFPS"));return C?C->GetFloat():0;}
