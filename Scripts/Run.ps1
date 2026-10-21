param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.6', [switch]$Editor, [switch]$Drive, [switch]$Stability,
    [ValidateSet('','settle','straight','zero_grip','brake','fork','steering','low_grip','low_tyre','rollover','powered_turn','powered_turn_low','powered_turn_left','powered_turn_start','friction_live','friction_keys','swerve_settle','swerve_straight','swerve_reverse','swerve_strafe','swerve_strafe_left','swerve_spin','swerve_spin_left','swerve_combined','swerve_transition','swerve_zero_grip','swerve_low_grip','swerve_brake','swerve_fork','swerve_friction_keys','swerve_rollover')][string]$Scenario = '')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$arguments = @("`"$projectRoot\ForkliftSim.uproject`"")
if (-not $Editor) { $arguments += @('-game', '-windowed', '-ResX=1280', '-ResY=800', '-dx11') }
if ($Drive) { $arguments += '-DriveLab' }
if ($Stability) { $arguments += '-StabilityLab' }
if ($Scenario) {
    if (-not $Stability -or $Editor) { throw 'Scenario playback requires -Stability without -Editor.' }
    $arguments += "-StabilityTest=$Scenario"
}
# The visible application is intentional: this script opens the requested interactive simulator.
Start-Process -FilePath (Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor.exe') -ArgumentList $arguments
