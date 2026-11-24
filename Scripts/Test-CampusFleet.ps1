param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',
 [ValidateSet('performance','dispatch','gate','detour','sensor_stop','watchdog','obstacle','resume','transfer','upper','rack_transfer','full_transfer')][string[]]$Cases=@('performance','dispatch','gate','detour','sensor_stop','watchdog','obstacle','resume','transfer','upper','rack_transfer'),
 [int]$WebPort=4184,[ValidateSet(480,960,1920)][int]$PhysicsHz=960,[ValidateRange(8,127)][int]$SolverIterations=32,[switch]$FullRateCameras,[switch]$ParallelPhysics,[switch]$RealtimeClock)
$ErrorActionPreference='Stop';$projectRoot=Split-Path -Parent $PSScriptRoot
$run=Join-Path $projectRoot ('Saved/Validation/Campus/Fleet-'+(Get-Date -Format yyyyMMdd-HHmmss)+'-'+[guid]::NewGuid().ToString('N').Substring(0,8))
New-Item -ItemType Directory -Path $run -Force | Out-Null
$editor=Join-Path $EngineRoot 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe';$results=@();$failed=@()
foreach($case in $Cases) {
 $extra=@();if(-not $RealtimeClock){$extra+=@('-benchmark','-fps=30')};if(-not $ParallelPhysics){$extra+='-SingleThreadedPhysics'};if($FullRateCameras){$extra+='-CampusFullRateCameras'}
 $log=Join-Path $run ($case+'.log');$report=Join-Path $projectRoot "Saved/Validation/Campus/Fleet-$case.json";$started=Get-Date
 & $editor "$projectRoot/ForkliftSim.uproject" /Game/Maps/WarehouseCampus -game -CampusFleetTest "-CampusFleetCase=$case" -WarehouseLab -ReducedMecanum "-ReducedPhysicsHz=$PhysicsHz" "-MecanumPositionIterations=$SolverIterations" "-MecanumVelocityIterations=$SolverIterations" -Mecanum -CameraViews -RenderOffscreen -dx11 -windowed -ResX=1280 -ResY=800 -ForceRes -unattended -nosound -nosplash -NoAssetRegistryCacheWrite "-WebTelemetry=http://127.0.0.1:$WebPort/api/telemetry" '-ExecCmds=r.AntiAliasingMethod 1,r.ScreenPercentage 75,sg.ShadowQuality 1,sg.PostProcessQuality 1' "-abslog=$log" @extra
 if(!(Test-Path -LiteralPath $report) -or (Get-Item -LiteralPath $report).LastWriteTime -lt $started){throw "No fresh report: $case"}
 $r=Get-Content -LiteralPath $report -Raw | ConvertFrom-Json;$pass=$r.pass -and (Select-String -LiteralPath $log -Pattern "CAMPUS_FLEET PASS case=$case " -Quiet)
 Copy-Item -LiteralPath $report -Destination (Join-Path $run ($case+'.json'))
 $results+=@{case=$case;pass=[bool]$pass;report=$report;log=$log;wall_s=((Get-Date)-$started).TotalSeconds;game_fps=$r.observed_game_fps;sim_wall_ratio=$r.sim_wall_ratio}
 @{physics_hz=$PhysicsHz;solver_iterations=$SolverIterations;full_rate_cameras=[bool]$FullRateCameras;parallel_physics=[bool]$ParallelPhysics;realtime_clock=[bool]$RealtimeClock;dll_sha256=(Get-FileHash (Join-Path $projectRoot 'Binaries/Win64/UnrealEditor-ForkliftSim.dll')).Hash;results=$results}|ConvertTo-Json -Depth 6 | Set-Content -Encoding UTF8 (Join-Path $run 'results.json')
 if(!$pass){$failed+=$case;Write-Output "CAMPUS_FLEET_CASE FAIL $case reason=$($r.reason)"}else{Write-Output "CAMPUS_FLEET_CASE PASS $case fps=$($r.observed_game_fps) ratio=$($r.sim_wall_ratio)"}
}
Write-Output "Campus fleet report: $run"

if($failed.Count){throw "Campus fleet cases failed: $($failed -join ", "). Reports: $run"}
