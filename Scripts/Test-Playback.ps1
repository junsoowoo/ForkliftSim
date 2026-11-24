param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',
 [ValidateSet('one','two','four','keys','natural4')][string[]]$Cases=@('one','two','four','keys','natural4'),[int]$WebPort=4184)
$ErrorActionPreference='Stop';$root=Split-Path -Parent $PSScriptRoot
$run=Join-Path $root ('Saved/Validation/Playback/Run-'+(Get-Date -Format yyyyMMdd-HHmmss)+'-'+[guid]::NewGuid().ToString('N').Substring(0,8));New-Item -ItemType Directory -Path $run -Force|Out-Null
$results=@();$failed=@();$editor=Join-Path $EngineRoot 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe'
foreach($case in $Cases){
 $rate=if($case -eq 'two'){2}elseif($case -in @('four','natural4')){4}else{1};$extra=if($case -eq 'natural4'){@()}else{@('-benchmark','-fps=120')};if($case -eq 'keys'){$extra+='-PlaybackKeys'}
 $log=Join-Path $run ($case+'.log');$report=Join-Path $root "Saved/Validation/Playback/$case.json";$started=Get-Date
 & $editor "$root/ForkliftSim.uproject" /Game/Maps/WarehouseCampus -game -CampusFleetTest -CampusFleetCase=board_playback -PlaybackValidation "-PlaybackCase=$case" "-PlaybackRate=$rate" -WarehouseLab -ReducedMecanum -ReducedPhysicsHz=480 -MecanumPositionIterations=16 -MecanumVelocityIterations=16 -Mecanum -CameraViews -RenderOffscreen -dx11 -windowed -ResX=1280 -ResY=800 -ForceRes -unattended -nosound -nosplash -NoAssetRegistryCacheWrite "-WebTelemetry=http://127.0.0.1:$WebPort/api/telemetry" '-ExecCmds=r.AntiAliasingMethod 1,r.ScreenPercentage 75,sg.ShadowQuality 1,sg.PostProcessQuality 1' "-abslog=$log" @extra
 if(!(Test-Path -LiteralPath $report) -or (Get-Item -LiteralPath $report).LastWriteTime -lt $started){throw "No fresh playback report for $case"};$r=Get-Content -LiteralPath $report -Raw|ConvertFrom-Json;$pass=$r.pass -and (Select-String -LiteralPath $log -Pattern "PLAYBACK_TEST PASS case=$case " -Quiet)
 Copy-Item -LiteralPath $report -Destination (Join-Path $run ($case+'.json'));$results+=@{case=$case;rate=$rate;benchmark=($case -ne 'natural4');pass=[bool]$pass;report=$report;log=$log;wall_s=((Get-Date)-$started).TotalSeconds};@{dll_sha256=(Get-FileHash (Join-Path $root 'Binaries/Win64/UnrealEditor-ForkliftSim.dll')).Hash;results=$results}|ConvertTo-Json -Depth 5|Set-Content -Encoding UTF8 (Join-Path $run 'results.json')
 if(!$pass){$failed+=$case};Write-Output "PLAYBACK_CASE $case pass=$pass reason=$($r.reason)"
}
Write-Output "Playback report: $run";if($failed.Count){throw "Failed playback cases: $($failed -join ', ')"}
