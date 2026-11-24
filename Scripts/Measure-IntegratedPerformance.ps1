# Headless native diagnostic, not a user launcher. Never changes physics fidelity.
param(
 [ValidatePattern('^[A-Za-z0-9_-]+$')][string]$Label='current',
 [string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',
 [switch]$Trace,[switch]$SerialPhysics,[switch]$CompareProximity,[switch]$ReferenceProximity,
 [ValidateRange(640,3840)][int]$Width=1280,[ValidateRange(480,2160)][int]$Height=800,
 [ValidateSet(0,1,2)][int]$IslandParallelMode=2,[ValidateRange(0,12)][int]$IslandMaxWorkers=0
)
$ErrorActionPreference='Stop';$root=Split-Path -Parent $PSScriptRoot
$folder=Join-Path $root ('Saved/Validation/Optimization/'+$Label)
New-Item -ItemType Directory -Path $folder -Force|Out-Null
$log=Join-Path $folder 'runtime.log';$extra=@()
if($SerialPhysics){$extra+='-SingleThreadedPhysics'}
if($CompareProximity){$extra+='-ProximityCompareTracing'}
if($ReferenceProximity){$extra+='-ProximityReferenceTracing'}
if($Trace){
 # Trace file writing uses the same ASCII junction as the native compiler.
 $sha=[Security.Cryptography.SHA256]::Create()
 try{$hash=[BitConverter]::ToString($sha.ComputeHash([Text.Encoding]::UTF8.GetBytes($root))).Replace('-','').Substring(0,12)}finally{$sha.Dispose()}
 $alias=Join-Path $env:TEMP ('ForkliftSim-'+$hash)
 if(!(Test-Path -LiteralPath $alias) -or (Get-Item -LiteralPath $alias).Target -ne $root){throw 'Build.ps1 must create the verified ASCII project junction before tracing.'}
 $tracePath=Join-Path $alias "Saved/Validation/Optimization/$Label/runtime.utrace"
 $extra+=@('-trace=cpu,frame,bookmark','-statnamedevents',"-tracefile=$tracePath")
}
$started=Get-Date;$watch=[Diagnostics.Stopwatch]::StartNew()
$extra+="-ini:Engine:[SystemSettings]:p.Chaos.Solver.IslandGroups.MaxWorkers=$IslandMaxWorkers"
& (Join-Path $EngineRoot 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe') "$root/ForkliftSim.uproject" /Game/Maps/WarehouseCampus -game -CampusFleetTest -CampusFleetCase=performance -WarehouseLab -ReducedMecanum -ReducedPhysicsHz=480 -MecanumPositionIterations=16 -MecanumVelocityIterations=16 -Mecanum -CameraViews -RenderOffscreen -dx11 -windowed "-ResX=$Width" "-ResY=$Height" -ForceRes -unattended -nosound -nosplash -NoAssetRegistryCacheWrite '-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=0.0020833333333333333' '-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubsteps=128' "-ExecCmds=t.MaxFPS 0,r.AntiAliasingMethod 1,r.ScreenPercentage 75,sg.ShadowQuality 1,sg.PostProcessQuality 1,p.Chaos.Solver.IslandGroups.ParallelMode $IslandParallelMode" "-abslog=$log" @extra
$code=$LASTEXITCODE;$watch.Stop();$report=Join-Path $root 'Saved/Validation/Campus/Fleet-performance.json'
if($code -ne 0 -or !(Test-Path -LiteralPath $report) -or (Get-Item -LiteralPath $report).LastWriteTime -lt $started){throw 'No fresh successful performance report'}
$data=Get-Content -LiteralPath $report -Raw|ConvertFrom-Json
if(!$data.pass -or !(Select-String -LiteralPath $log -Pattern 'CAMPUS_FLEET PASS case=performance ' -Quiet)){throw 'Performance fixture failed'}
if($Trace -and (!(Test-Path -LiteralPath $tracePath) -or (Get-Item -LiteralPath $tracePath).Length -lt 1024)){throw 'CPU trace was not written'}
Copy-Item -LiteralPath $report -Destination (Join-Path $folder 'fleet.json')
$checks=0;if($CompareProximity){$lines=Select-String -LiteralPath $log -Pattern 'PROXIMITY_BATCH_CHECK .*checks=(\d+) PASS';foreach($line in $lines){$checks+=[int]$line.Matches[0].Groups[1].Value};if(!$checks){throw 'No proximity parity checks executed'}}
@{label=$Label;dll_sha256=(Get-FileHash (Join-Path $root 'Binaries/Win64/UnrealEditor-ForkliftSim.dll')).Hash;physics_hz=480;position_iterations=16;velocity_iterations=16;camera_views=$true;trace=[bool]$Trace;reference_proximity=[bool]$ReferenceProximity;proximity_reference_checks=$checks;serial_physics=[bool]$SerialPhysics;island_parallel_mode=$IslandParallelMode;island_max_workers=$IslandMaxWorkers;width=$Width;height=$Height;wall_including_startup_s=$watch.Elapsed.TotalSeconds;sim_s=$data.sim_s;end_game_fps=$data.observed_game_fps;sim_wall_ratio=$data.sim_wall_ratio;pass=$true;scope='12 simulated seconds, idle 3-vehicle offscreen world. End smoothed game FPS and cumulative sim/wall ratio; not Editor PIE or mission throughput.'}|ConvertTo-Json -Depth 4|Set-Content -Encoding UTF8 (Join-Path $folder 'results.json')
Write-Output "INTEGRATED_PERFORMANCE $Label fps=$($data.observed_game_fps) ratio=$($data.sim_wall_ratio) report=$folder"
