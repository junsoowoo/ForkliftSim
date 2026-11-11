param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.6', [ValidateSet(30,60,120)][int]$Fps=60,
    [string[]]$Scenarios=@('lift','heavy','eccentric','holes','entry','miss','shallow','carry','slip','unload','reset','loose','turn','turn_left','turn_start','turn_high','s_curve'))
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
New-Item -ItemType Directory -Path (Join-Path $projectRoot 'Saved\Logs') -Force | Out-Null
foreach ($case in $Scenarios) {
    if ($case -notin @('lift','heavy','eccentric','holes','entry','miss','shallow','carry','slip','unload','reset','loose','turn','turn_left','turn_start','turn_high','s_curve','cargo_hold','cargo_entry','cargo_unload','cargo_accel','cargo_accel_low','cargo_brake','cargo_brake_low','cargo_corner','cargo_corner_low','cargo_tip','cargo_drop')) { throw "Unknown payload test $case" }
    $scenario="swerve_payload_$case"
    $log=Join-Path $projectRoot "Saved\Logs\Payload-$case-$Fps.log"
    & $editor "$projectRoot\ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite -benchmark "-fps=$Fps" "-StabilityTest=$scenario" "-abslog=$log"
    if ($LASTEXITCODE -ne 0) { throw "Payload $case failed ($LASTEXITCODE); see $log" }
    $result=Select-String -LiteralPath $log -Pattern "STABILITY_TEST $scenario PASS"
    if (-not $result) { throw "Missing success marker: $log" }
    (Select-String -LiteralPath $log -Pattern '(PAYLOAD|CARGO)_CHECK').Line
}
