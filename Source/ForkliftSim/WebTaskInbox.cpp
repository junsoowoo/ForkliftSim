#include "WebTaskInbox.h"
#include "HttpModule.h"
#include "Interfaces/IHttpResponse.h"
#include "Serialization/JsonSerializer.h"
#include "Async/Async.h"
void FWebTaskInbox::Poll()
{
 if(Stopped||Pending||FPlatformTime::Seconds()<Next)return;Next=FPlatformTime::Seconds()+.5;
 auto R=FHttpModule::Get().CreateRequest();R->SetURL(Base+TEXT("/api/commands?world_id=")+World+TEXT("&run_id=")+Run);R->SetVerb(TEXT("GET"));R->SetTimeout(1.5f);Pending=R;
 const TWeakPtr<FWebTaskInbox,ESPMode::ThreadSafe> Weak=AsShared();
 R->OnProcessRequestComplete().BindLambda([Weak](FHttpRequestPtr,FHttpResponsePtr Response,bool Success){const FString Body=Success&&Response&&Response->GetResponseCode()==200?Response->GetContentAsString():TEXT("");
  AsyncTask(ENamedThreads::GameThread,[Weak,Body](){if(auto P=Weak.Pin()){P->Pending.Reset();if(P->Stopped||Body.Len()>131072)return;TSharedPtr<FJsonObject> J;const TArray<TSharedPtr<FJsonValue>>* A=nullptr;
   if(FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Body),J)&&J&&J->TryGetArrayField(TEXT("commands"),A)&&A->Num()<=8)for(const auto& V:*A)if(V->Type==EJson::Object)P->Commands.Add(V->AsObject());}});
 });if(!R->ProcessRequest())Pending.Reset();
}
void FWebTaskInbox::Stop(){Stopped=true;if(Pending)Pending->CancelRequest();Pending.Reset();Commands.Reset();}
