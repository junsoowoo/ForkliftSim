param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[ValidateSet(30,60,120)][int]$Fps=60,
    [ValidateSet(120,240,480)][int]$PhysicsHz=480,
    [string[]]$Cases=@('settle','soft','stiff','drop','drop_low','step','step_rigid','roll','payload'))
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
New-Item -ItemType Directory -Path (Join-Path $projectRoot 'Saved\Logs') -Force | Out-Null
$tag="$Fps-${PhysicsHz}hz"
$physicsDt=(1.0/$PhysicsHz).ToString('G17',[Globalization.CultureInfo]::InvariantCulture)
$modelLog=Join-Path $projectRoot "Saved\Logs\Tyres-Model-$tag.log"
& $editor "$projectRoot\ForkliftSim.uproject" -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite '-ExecCmds=Automation RunTests ForkliftSim.Tyres' '-TestExit=Automation Test Queue Empty' "-abslog=$modelLog"
if ($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath $modelLog -Pattern 'Result=\{Success\}.*ForkliftSim.Tyres' -Quiet)) { throw 'Tyre model tests failed' }
$values=@{}
foreach ($case in $Cases) {
    if ($case -notin @('settle','soft','stiff','drop','drop_low','step','step_rigid','roll','payload')) { throw "Unknown tyre case $case" }
    $scenario="swerve_tyre_$case"
    $log=Join-Path $projectRoot "Saved\Logs\Tyre-$case-$tag.log"
    & $editor "$projectRoot\ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite -benchmark "-fps=$Fps" "-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=$physicsDt" "-StabilityTest=$scenario" "-abslog=$log"
    if ($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath $log -Pattern "STABILITY_TEST $scenario PASS" -Quiet)) { throw "Tyre $case failed; see $log" }
    $line=(Select-String -LiteralPath $log -Pattern 'TYRE_CHECK').Line
    $line
    $v=@{}
    foreach ($field in @('mean_compression_mm','peak_compression_mm','vibration_mps','step_accel_mps2')) {
        if ($line -notmatch "$field=([-0-9.]+)") { throw "Missing $field for $case" }
        $v[$field]=[double]::Parse($Matches[1],[Globalization.CultureInfo]::InvariantCulture)
    }
    $values[$case]=$v
}
if ($values.ContainsKey('settle') -and $values.ContainsKey('soft') -and $values.ContainsKey('stiff')) {
    $softRatio=$values.soft.mean_compression_mm/$values.settle.mean_compression_mm
    $stiffRatio=$values.stiff.mean_compression_mm/$values.settle.mean_compression_mm
    if ($softRatio -lt 1.7 -or $softRatio -gt 2.3 -or $stiffRatio -lt .4 -or $stiffRatio -gt .6) { throw 'Inverse-stiffness A/B failed' }
}
if ($values.ContainsKey('drop') -and $values.ContainsKey('drop_low')) {
    if ($values.drop_low.vibration_mps -lt $values.drop.vibration_mps+.005 -or $values.drop_low.peak_compression_mm -lt $values.drop.peak_compression_mm+1) { throw 'Damping A/B failed' }
}
if ($values.ContainsKey('step') -and $values.ContainsKey('step_rigid')) {
    if ($values.step.step_accel_mps2 -ge $values.step_rigid.step_accel_mps2) { throw 'Tyre step isolation A/B failed' }
}
"TYRE_COMPARISON PASS fps=$Fps max_physics_hz=$PhysicsHz cases=$($Cases.Count)"
