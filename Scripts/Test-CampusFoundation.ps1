# Native qualification, separate from the user's Start.bat launcher.
param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',
 [ValidateSet('smoke','inventory','rackvision','0','90','180','270')][string[]]$Cases=@('smoke','inventory','rackvision','0','90','180','270'),[ValidateSet(480,960,1920)][int]$PhysicsHz=960,[ValidateRange(8,127)][int]$SolverIterations=32,[switch]$ParallelPhysics)
$ErrorActionPreference='Stop';$projectRoot=Split-Path -Parent $PSScriptRoot
$run=Join-Path $projectRoot ('Saved/Validation/Campus/Foundation-'+(Get-Date -Format yyyyMMdd-HHmmss)+'-'+[guid]::NewGuid().ToString('N').Substring(0,8))
New-Item -ItemType Directory -Path $run -Force | Out-Null
$editor=Join-Path $EngineRoot 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe'
$results=@()
foreach($case in $Cases) {
 $extra=if($ParallelPhysics){@()}else{@('-SingleThreadedPhysics')}
 $kind=if($case -in @('smoke','inventory','rackvision')){$case}else{'handling'};$yaw=if($case -eq 'rackvision'){90}elseif($kind -eq 'handling'){[int]$case}else{0}
 $log=Join-Path $run ($case+'.log');$report=Join-Path $projectRoot "Saved/Validation/Campus/Foundation-$kind-$yaw.json"
 $started=Get-Date
 & $editor "$projectRoot/ForkliftSim.uproject" /Game/Maps/WarehouseCampus -game -CampusFoundationTest "-CampusCase=$kind" "-CampusEntryYaw=$yaw" -WarehouseLab -ReducedMecanum "-ReducedPhysicsHz=$PhysicsHz" "-MecanumPositionIterations=$SolverIterations" "-MecanumVelocityIterations=$SolverIterations" -Mecanum -CameraViews -RenderOffscreen -dx11 -windowed -ResX=1280 -ResY=800 -ForceRes -unattended -nosound -nosplash -NoAssetRegistryCacheWrite -benchmark -fps=30 '-ExecCmds=r.AntiAliasingMethod 1,r.ScreenPercentage 75,sg.ShadowQuality 1,sg.PostProcessQuality 1' "-abslog=$log" @extra
 if(!(Test-Path -LiteralPath $report) -or (Get-Item -LiteralPath $report).LastWriteTime -lt $started){throw "No fresh report for $case"}
 $r=Get-Content -LiteralPath $report -Raw | ConvertFrom-Json
 $pass=$r.pass -and (Select-String -LiteralPath $log -Pattern "CAMPUS_FOUNDATION PASS case=$kind yaw=$yaw " -Quiet)
 Copy-Item -LiteralPath $report -Destination (Join-Path $run ($case+'.json'))
 $results+=@{case=$case;pass=[bool]$pass;report=$report;log=$log;wall_s=((Get-Date)-$started).TotalSeconds}
 @{parallel_physics=[bool]$ParallelPhysics;physics_hz=$PhysicsHz;solver_iterations=$SolverIterations;scope='Scripted operator inputs; native physical/rendered sensor validation, not autonomous dispatch or real-time qualification';dll_sha256=(Get-FileHash (Join-Path $projectRoot 'Binaries/Win64/UnrealEditor-ForkliftSim.dll')).Hash;results=$results}|ConvertTo-Json -Depth 6 | Set-Content -Encoding UTF8 (Join-Path $run 'results.json')
 if(!$pass){throw "Campus case $case failed. Report: $report"}
 Write-Output "CAMPUS_CASE PASS $case"
}
Write-Output "Campus foundation report: $run"
