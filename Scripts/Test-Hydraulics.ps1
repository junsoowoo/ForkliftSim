param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[ValidateSet(30,60,120)][int]$Fps=60,
    [ValidateSet(240,480)][int]$PhysicsHz=480,
    [string[]]$Cases=@('empty','loaded','low_power','overload','hold','leak','lower','legacy'))
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
New-Item -ItemType Directory -Path (Join-Path $projectRoot 'Saved\Logs') -Force | Out-Null
$tag="$Fps-${PhysicsHz}hz"
$physicsDt=(1.0/$PhysicsHz).ToString('G17',[Globalization.CultureInfo]::InvariantCulture)
$modelLog=Join-Path $projectRoot "Saved\Logs\Hydraulics-Model-$tag.log"
& $editor "$projectRoot\ForkliftSim.uproject" -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite '-ExecCmds=Automation RunTests ForkliftSim.Hydraulics' '-TestExit=Automation Test Queue Empty' "-abslog=$modelLog"
if ($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath $modelLog -Pattern 'Result=\{Success\}.*ForkliftSim.Hydraulics' -Quiet)) { throw 'Hydraulic model tests failed' }
$values=@{}
foreach ($case in $Cases) {
    if ($case -notin @('empty','loaded','low_power','overload','hold','leak','lower','legacy')) { throw "Unknown hydraulic case $case" }
    $scenario="swerve_hydraulic_$case"
    $log=Join-Path $projectRoot "Saved\Logs\Hydraulic-$case-$tag.log"
    & $editor "$projectRoot\ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite -benchmark "-fps=$Fps" "-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=$physicsDt" "-StabilityTest=$scenario" "-abslog=$log"
    if ($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath $log -Pattern "STABILITY_TEST $scenario PASS" -Quiet)) { throw "Hydraulic $case failed; see $log" }
    $line=(Select-String -LiteralPath $log -Pattern 'HYDRAULIC_CHECK').Line; $line
    if($line -notmatch 'mean_lift_mps=([-0-9.]+)') { throw 'Missing lift speed' }
    $values[$case]=[double]::Parse($Matches[1],[Globalization.CultureInfo]::InvariantCulture)
}
if($values.ContainsKey('loaded') -and $values.ContainsKey('empty') -and $values.loaded -ge $values.empty*.9) { throw 'Loaded/empty speed A/B failed' }
if($values.ContainsKey('low_power') -and $values.ContainsKey('loaded') -and $values.low_power -ge $values.loaded*.8) { throw 'Motor power/speed A/B failed' }
"HYDRAULIC_COMPARISON PASS fps=$Fps max_physics_hz=$PhysicsHz cases=$($Cases.Count)"
