# Measurement of the default warehouse scene. Start.bat remains the user launcher.
param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[int]$WebPort=4174,[switch]$FullMission,[switch]$Profile)
$ErrorActionPreference='Stop';$root=Split-Path -Parent $PSScriptRoot
$scenario=if($FullMission){'mecanum_mission_low'}else{'mecanum_mission_realtime'}
$prefix=if($FullMission){'MissionDefault-'}else{'MissionRealtime-'}
$run=Join-Path $root ('Saved/Validation/'+$prefix+(Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $run -Force|Out-Null
$log=Join-Path $run 'runtime.log';$started=[DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
$profileArg=if($Profile){'-MissionProfile'}else{'-NoMissionProfile'}
@{scenario=$scenario;full_mission=[bool]$FullMission;dll_sha256=(Get-FileHash "$root/Binaries/Win64/UnrealEditor-ForkliftSim.dll").Hash;benchmark=$false;scene='/Game/Maps/WarehouseLab';camera_views=$true;rendering='balanced-1280x800-dx11';physics_hz=960;web_port=$WebPort;sources=@(Get-ChildItem "$root/Source/ForkliftSim" -File|Get-FileHash|Select-Object Path,Hash);configs=@(Get-ChildItem "$root/Config" -File|Where-Object {$_.Extension -in @('.json','.ini')}|Get-FileHash|Select-Object Path,Hash)}|ConvertTo-Json -Depth 5|Set-Content (Join-Path $run 'manifest.json') -Encoding UTF8
& (Join-Path $EngineRoot 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe') "$root/ForkliftSim.uproject" /Game/Maps/WarehouseLab -game -StabilityLab -Mecanum -ReducedMecanum -WarehouseLab -CameraViews -SingleThreadedPhysics "-StabilityTest=$scenario" $profileArg -RenderOffscreen -windowed -ResX=1280 -ResY=800 -ForceRes -dx11 -unattended -nosound -nosplash -NoAssetRegistryCacheWrite '-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=0.0010416666666666667' '-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubsteps=128' '-ExecCmds=r.AntiAliasingMethod 1,r.ScreenPercentage 75,r.Editor.Viewport.ScreenPercentageMode.RealTime 0,r.Editor.Viewport.ScreenPercentage 75,sg.PostProcessQuality 1,sg.ShadowQuality 1,r.Shadow.MaxResolution 1024' "-WebTelemetry=http://127.0.0.1:$WebPort/api/telemetry" "-abslog=$log"
if($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath $log -Pattern "STABILITY_TEST $scenario PASS" -Quiet)){throw "Default-scene mission sample failed: $log"}
$line=(Select-String -LiteralPath $log -Pattern 'PERFORMANCE_REPORT .* file=(.+)$'|Select-Object -Last 1)
if(-not $line){throw 'Missing measured performance report'}
$path=$line.Matches[0].Groups[1].Value;$path=if([IO.Path]::IsPathRooted($path)){$path}else{[IO.Path]::GetFullPath((Join-Path (Join-Path $EngineRoot 'Engine/Binaries/Win64') $path))}
Copy-Item -LiteralPath $path -Destination (Join-Path $run 'performance.json');$data=Get-Content (Join-Path $run 'performance.json') -Raw|ConvertFrom-Json
if($data.benchmark -or $data.samples -lt 100){throw 'Non-benchmark measured sample required'}
$history=Invoke-RestMethod "http://127.0.0.1:$WebPort/api/history"
$frames=@($history.frames|Where-Object {$_.frame.scenario -eq $scenario -and $_.frame.source.started_at_ms -ge $started})
if($frames.Count -lt 20){throw 'Actual default-scene web frames were not received'}
$frames|ConvertTo-Json -Depth 40|Set-Content (Join-Path $run 'web-frames.json') -Encoding UTF8
$qualified=$data.actual_realtime_ratio -ge .9 -and $data.mean_fps -ge 25 -and $data.p95_frame_ms -le 80
$scope=if($FullMission){'Complete tier-1 mission in actual default warehouse without fixture map, two cameras, reduced 960Hz, balanced offscreen standalone and actual web telemetry. Not Editor PIE or all conditions performance guarantee.'}else{'20 simulation seconds of actual default warehouse, two cameras, reduced 960Hz, balanced offscreen standalone and actual web telemetry. Not full-mission performance or editor PIE guarantee.'}
@{pass=$true;full_mission=[bool]$FullMission;meets_realtime_target=[bool]$qualified;mean_fps=$data.mean_fps;p95_frame_ms=$data.p95_frame_ms;actual_realtime_ratio=$data.actual_realtime_ratio;web_frames=$frames.Count;benchmark=$false;scope=$scope}|ConvertTo-Json|Set-Content (Join-Path $run 'result.json') -Encoding UTF8
Write-Output "MISSION_REALTIME_SAMPLE report=$run qualified=$qualified fps=$($data.mean_fps) realtime=$($data.actual_realtime_ratio)"
