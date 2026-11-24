#pragma once
#include "CoreMinimal.h"
#include "Interfaces/IHttpRequest.h"
#include "Dom/JsonObject.h"
class FWebTaskInbox : public TSharedFromThis<FWebTaskInbox,ESPMode::ThreadSafe>
{
public:
 FString Base,World,Run;TArray<TSharedPtr<FJsonObject>> Commands;
 void Poll();void Stop();
private:
 FHttpRequestPtr Pending;double Next=0;bool Stopped=false;
};
