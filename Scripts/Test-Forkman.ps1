# Automated rendered regression of the selected articulated appearance; not a launcher.
param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',
 [ValidateSet('handling_pick_high','handling_place_high')][string[]]$Cases=@('handling_pick_high','handling_place_high'),[switch]$CompareProximity)
$ErrorActionPreference='Stop';$root=Split-Path -Parent $PSScriptRoot
$folder=Join-Path $root ('Saved/Validation/Forkman/Run-'+(Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $folder -Force|Out-Null
$expected=(Get-Content (Join-Path $root 'Config/Modeling.json') -Raw|ConvertFrom-Json).body_mesh
$results=@();$editor=Join-Path $EngineRoot 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe'
foreach($case in $Cases){
 $log=Join-Path $folder ($case+'.log')
 $extra=@();if($CompareProximity){$extra+='-ProximityCompareTracing'}
 & $editor "$root/ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -WarehouseLab -Mecanum -ReducedMecanum -ReducedPhysicsHz=960 -SingleThreadedPhysics -CameraViews "-StabilityTest=mecanum_$case" -MecanumPositionIterations=32 -MecanumVelocityIterations=32 '-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=0.0010416666666666667' '-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubsteps=128' -benchmark -fps=30 -RenderOffscreen -windowed -ResX=1280 -ResY=800 -ForceRes -dx11 -unattended -nosound -nosplash -NoAssetRegistryCacheWrite '-ExecCmds=t.MaxFPS 0,r.AntiAliasingMethod 1,r.ScreenPercentage 75,sg.ShadowQuality 1' "-abslog=$log" @extra
 $code=$LASTEXITCODE
 $visible=Select-String -LiteralPath $log -Pattern ('MODEL_PRESENTATION .*body='+[regex]::Escape($expected)) -Quiet
 $complete=Select-String -LiteralPath $log -Pattern "HANDLING_CHECK mecanum_$case .* PASS" -Quiet
 $qualified=Select-String -LiteralPath $log -Pattern "STABILITY_TEST mecanum_$case PASS" -Quiet
 $proxy=Select-String -LiteralPath $log -Pattern 'MODEL_PRESENTATION .*sensor_self_geometry=physics_proxy' -Quiet
 $hidden=Select-String -LiteralPath $log -Pattern '(?:^| )-ModelingPhysicsOnly(?: |$)|MODEL_ASSETS missing' -Quiet
 $pass=$code -eq 0 -and $visible -and $complete -and $qualified -and $proxy -and -not $hidden
 $checks=0;if($CompareProximity){foreach($line in (Select-String -LiteralPath $log -Pattern 'PROXIMITY_BATCH_CHECK .*checks=(\d+) PASS')){$checks+=[int]$line.Matches[0].Groups[1].Value};$pass=$pass -and $checks -gt 0}
 $results+=@{case=$case;pass=[bool]$pass;exit_code=$code;proximity_reference_checks=$checks;selected_model_visible=[bool]$visible;physics_self_geometry=[bool]$proxy;measured_handling_complete=[bool]$complete;physics_only_or_missing=[bool]$hidden;log=$log}
 @{model=$expected;dll_sha256=(Get-FileHash (Join-Path $root 'Binaries/Win64/UnrealEditor-ForkliftSim.dll')).Hash;cases=$results}|ConvertTo-Json -Depth 5|Set-Content -Encoding UTF8 (Join-Path $folder 'results.json')
 Write-Output "FORKMAN_RENDERED $case pass=$pass visible=$visible handling=$complete exit=$code"
}
Write-Output "Forkman validation: $folder"
if(@($results|Where-Object {-not $_.pass}).Count){throw 'Forkman rendered handling regression failed.'}
