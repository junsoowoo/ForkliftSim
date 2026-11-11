#pragma once
#include "CoreMinimal.h"
#include "Interfaces/IHttpRequest.h"
#include "Dom/JsonObject.h"

// Game-thread, one asynchronous HTTP request in flight. The publisher does not
// write forces, actor state or simulation settings. Loopback destinations only.
class FWebTelemetryPublisher : public TSharedFromThis<FWebTelemetryPublisher,ESPMode::ThreadSafe>
{
public:
    static TSharedPtr<FWebTelemetryPublisher,ESPMode::ThreadSafe> Create();
    void NewRun();
    bool IsDue() const;
    void Send(TSharedRef<FJsonObject> Frame);
    void Stop();
private:
    FString Url, WorldId, RunId;
    int64 Sequence=0, StartedMs=0;
    double NextWallS=0;
    bool bStopped=false, bReportedFailure=false;
    FHttpRequestPtr Pending;
    TSharedPtr<FJsonObject> LastFrame;
    void Dispatch(TSharedRef<FJsonObject> Frame,bool Final);
};
