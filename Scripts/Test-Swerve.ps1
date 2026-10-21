param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.6', [ValidateSet(15,20,30,60,120)][int]$Fps = 60,
    [string[]]$Scenarios = @('swerve_settle','swerve_straight','swerve_reverse','swerve_strafe','swerve_strafe_left','swerve_spin','swerve_spin_left','swerve_combined','swerve_transition','swerve_zero_grip','swerve_low_grip','swerve_brake','swerve_fork','swerve_friction_keys','swerve_rollover'))
$ErrorActionPreference = 'Stop'
& "$PSScriptRoot\Test-Stability.ps1" -EngineRoot $EngineRoot -Fps $Fps -Scenarios $Scenarios
