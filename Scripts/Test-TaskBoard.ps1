param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',
 [ValidateSet('board_batch','board_cycle','board_fault','board_scenario','board_web')][string[]]$Cases=@('board_batch','board_fault','board_scenario','board_cycle'),[int]$WebPort=4184)
$ErrorActionPreference='Stop';$root=Split-Path -Parent $PSScriptRoot
$run=Join-Path $root ('Saved/Validation/Tasks/Run-'+(Get-Date -Format yyyyMMdd-HHmmss)+'-'+[guid]::NewGuid().ToString('N').Substring(0,8));New-Item -ItemType Directory -Path $run -Force|Out-Null
$results=@();$failed=@();$editor=Join-Path $EngineRoot 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe'
foreach($case in $Cases){
 $log=Join-Path $run ($case+'.log');$report=Join-Path $root "Saved/Validation/Tasks/$case.json";$started=Get-Date;$extra=@()
 if($case -eq 'board_scenario'){$config=Get-Content (Join-Path $root 'Config/CampusOperations.json') -Raw|ConvertFrom-Json;$config.scenario_id='qualification-600kg-mu03';$config.cargo_mass_kg=600;$config.road_mu=.3;$scenario=Join-Path $run 'scenario.json';$config|ConvertTo-Json|Set-Content -Encoding UTF8 $scenario;$extra+="-CampusScenario=$scenario"}
 & $editor "$root/ForkliftSim.uproject" /Game/Maps/WarehouseCampus -game -CampusFleetTest "-CampusFleetCase=$case" -WarehouseLab -ReducedMecanum -ReducedPhysicsHz=480 -MecanumPositionIterations=16 -MecanumVelocityIterations=16 -Mecanum -CameraViews -RenderOffscreen -dx11 -windowed -ResX=1280 -ResY=800 -ForceRes -unattended -nosound -nosplash -NoAssetRegistryCacheWrite -benchmark -fps=30 "-WebTelemetry=http://127.0.0.1:$WebPort/api/telemetry" '-ExecCmds=r.AntiAliasingMethod 1,r.ScreenPercentage 75,sg.ShadowQuality 1,sg.PostProcessQuality 1' "-abslog=$log" @extra
 if(!(Test-Path -LiteralPath $report) -or (Get-Item -LiteralPath $report).LastWriteTime -lt $started){throw "No fresh task report for $case"};$r=Get-Content -LiteralPath $report -Raw|ConvertFrom-Json;$pass=$r.pass -and (Select-String -LiteralPath $log -Pattern "TASK_BOARD PASS case=$case " -Quiet)
 Copy-Item -LiteralPath $report -Destination (Join-Path $run ($case+'.json'));$results+=@{case=$case;pass=[bool]$pass;report=$report;log=$log;wall_s=((Get-Date)-$started).TotalSeconds};@{dll_sha256=(Get-FileHash (Join-Path $root 'Binaries/Win64/UnrealEditor-ForkliftSim.dll')).Hash;results=$results}|ConvertTo-Json -Depth 5|Set-Content -Encoding UTF8 (Join-Path $run 'results.json')
 if(!$pass){$failed+=$case};Write-Output "TASK_BOARD_CASE $case pass=$pass reason=$($r.reason)"
}
Write-Output "Task board report: $run";if($failed.Count){throw "Failed task cases: $($failed -join ', ')"}
