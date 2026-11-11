param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.6', [ValidateSet(15,20,30,60,120)][int]$Fps = 60)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$editor = Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$logDir = Join-Path $projectRoot 'Saved\Logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$log = Join-Path $logDir "WheelResistanceModel-$Fps.log"
& $editor "$projectRoot\ForkliftSim.uproject" -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite '-ExecCmds=Automation RunTests ForkliftSim.WheelResistance' '-TestExit=Automation Test Queue Empty' "-abslog=$log"
if ($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath $log -Pattern 'Result=\{Success\}.*ForkliftSim.WheelResistance')) { throw 'Wheel resistance model test failed' }
$scenarios = @('swerve_load','swerve_coast_none','swerve_coast_rolling','swerve_coast_bearing','swerve_coast_both','swerve_coast_high','swerve_coast_reverse')
& "$PSScriptRoot\Test-Stability.ps1" -EngineRoot $EngineRoot -Fps $Fps -Scenarios $scenarios
# Compare the matched 2 m/s initial-condition experiments, not powered speed:
# a closed-loop motor can compensate for losses and obscure their effect.
$final = @{}
foreach ($scenario in $scenarios | Where-Object { $_ -like 'swerve_coast_*' }) {
    $caseLog = Join-Path $projectRoot "Saved\Logs\Stability-$scenario-$Fps.log"
    $line = (Select-String -LiteralPath $caseLog -Pattern 'COAST_CHECK initial=').Line
    if ($line -notmatch 'final=([-0-9.]+)') { throw "Missing coast result: $scenario" }
    $final[$scenario] = [double]::Parse($Matches[1], [Globalization.CultureInfo]::InvariantCulture)
}
if (-not ($final.swerve_coast_none -gt $final.swerve_coast_bearing + 0.15 -and
    $final.swerve_coast_bearing -gt $final.swerve_coast_rolling + 0.3 -and
    $final.swerve_coast_rolling -gt $final.swerve_coast_both + 0.01 -and
    $final.swerve_coast_both -gt $final.swerve_coast_high + 0.1 -and
    [Math]::Abs($final.swerve_coast_both + $final.swerve_coast_reverse) -lt 0.15)) {
    throw "Coast ordering/reverse symmetry failed: $($final | Out-String)"
}
"RESISTANCE_COMPARISON PASS fps=$Fps | $($final | Out-String)"
