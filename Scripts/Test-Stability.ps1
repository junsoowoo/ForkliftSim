param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.6', [ValidateSet(15,20,30,60,120)][int]$Fps = 60, [string[]]$Scenarios = @('settle','straight','zero_grip','brake','fork','steering','low_grip','low_tyre','rollover','powered_turn','powered_turn_low','powered_turn_left','powered_turn_start','friction_live','friction_keys'))
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$editor = Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$logDir = Join-Path $projectRoot 'Saved\Logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
foreach ($scenario in $Scenarios) {
    if ($scenario -notin @('settle','straight','zero_grip','brake','fork','steering','low_grip','low_tyre','rollover','powered_turn','powered_turn_low','powered_turn_left','powered_turn_start','friction_live','friction_keys','swerve_settle','swerve_straight','swerve_reverse','swerve_strafe','swerve_strafe_left','swerve_spin','swerve_spin_left','swerve_combined','swerve_transition','swerve_zero_grip','swerve_low_grip','swerve_brake','swerve_fork','swerve_friction_keys','swerve_rollover')) { throw "Unknown scenario: $scenario" }
    $log = Join-Path $logDir "Stability-$scenario-$Fps.log"
    & $editor "$projectRoot\ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -nullrhi -unattended -nosound -nosplash -nop4 -benchmark "-fps=$Fps" "-StabilityTest=$scenario" "-abslog=$log"
    if ($LASTEXITCODE -ne 0) { throw "Stability scenario $scenario failed: $LASTEXITCODE" }
    $result = Select-String -LiteralPath $log -Pattern "STABILITY_TEST $scenario PASS"
    if (-not $result) { throw "Missing success marker for $scenario; inspect $log" }
    $result.Line
}
