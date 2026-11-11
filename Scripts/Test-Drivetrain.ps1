param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[ValidateSet(30,60,120)][int]$Fps=60,
    [string[]]$Cases=@('flat','flat_weak','ramp','ramp_weak','step','split','uniform','steer','reset'))
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
New-Item -ItemType Directory -Path (Join-Path $projectRoot 'Saved\Logs') -Force | Out-Null
$modelLog=Join-Path $projectRoot "Saved\Logs\Drivetrain-Model-$Fps.log"
& $editor "$projectRoot\ForkliftSim.uproject" -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite '-ExecCmds=Automation RunTests ForkliftSim.Drivetrain' '-TestExit=Automation Test Queue Empty' "-abslog=$modelLog"
if ($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath $modelLog -Pattern 'Result=\{Success\}.*ForkliftSim.Drivetrain' -Quiet)) { throw 'Drivetrain model tests failed' }
(Select-String -LiteralPath $modelLog -Pattern 'Result=\{Success\}.*ForkliftSim.Drivetrain').Line
$measurements=@{}
foreach ($case in $Cases) {
    if ($case -notin @('flat','flat_weak','ramp','ramp_weak','step','split','uniform','steer','reset')) { throw "Unknown drivetrain case: $case" }
    $scenario="swerve_power_$case"
    $log=Join-Path $projectRoot "Saved\Logs\Power-$case-$Fps.log"
    & $editor "$projectRoot\ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite -benchmark "-fps=$Fps" "-StabilityTest=$scenario" "-abslog=$log"
    if ($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath $log -Pattern "STABILITY_TEST $scenario PASS" -Quiet)) { throw "Drivetrain $case failed; see $log" }
    $line=(Select-String -LiteralPath $log -Pattern 'POWER_CHECK').Line
    $line
    $values=@{}
    foreach ($field in @('distance_m','slope_mps','motor_peak_w','left_slip','right_slip','left_slip_mean')) {
        if ($line -notmatch "$field=([-0-9.]+)") { throw "Missing $field for $case" }
        $values[$field]=[double]::Parse($Matches[1],[Globalization.CultureInfo]::InvariantCulture)
    }
    $measurements[$case]=$values
}
if ($measurements.ContainsKey('flat') -and $measurements.ContainsKey('flat_weak')) {
    if ($measurements.flat.distance_m -lt $measurements.flat_weak.distance_m+1) { throw 'Flat power A/B failed' }
}
if ($measurements.ContainsKey('ramp') -and $measurements.ContainsKey('ramp_weak')) {
    if ($measurements.ramp.slope_mps -lt $measurements.ramp_weak.slope_mps+.3) { throw 'Ramp power A/B failed' }
}
if ($measurements.ContainsKey('split') -and $measurements.ContainsKey('uniform')) {
    # Keep raw peaks, but compare sustained slip during the same 4..8 s window.
    # A single high-grip constraint impulse can dominate a peak after compliance.
    if ($measurements.split.left_slip_mean -lt $measurements.uniform.left_slip_mean+.02) { throw 'Split grip mean-slip A/B failed' }
}
"DRIVETRAIN_COMPARISON PASS fps=$Fps cases=$($Cases.Count)"
