param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.6', [ValidateSet('All','Model','Runtime')][string]$Mode = 'All', [ValidateSet(30,60,120)][int]$Fps = 60)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$editor = Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$logDir = Join-Path $projectRoot 'Saved\Logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
if ($Mode -in @('All','Model')) {
    $log = Join-Path $logDir 'WheelModelTests.log'
    & $editor "$projectRoot\ForkliftSim.uproject" -nullrhi -unattended -nosound -nosplash -nop4 '-ExecCmds=Automation RunTests ForkliftSim.Wheels' '-TestExit=Automation Test Queue Empty' "-abslog=$log"
    if ($LASTEXITCODE -ne 0) { throw "Wheel model tests failed: $LASTEXITCODE" }
    $results = @(Select-String -LiteralPath $log -Pattern 'Result=\{Success\}.*ForkliftSim.Wheels')
    if ($results.Count -ne 6) { throw 'Expected six successful wheel model tests; inspect WheelModelTests.log' }
    $results.Line
}
if ($Mode -in @('All','Runtime')) {
    foreach ($scenario in @('baseline','low_grip','steering')) {
        $log = Join-Path $logDir "Wheels-$scenario-$Fps.log"
        & $editor "$projectRoot\ForkliftSim.uproject" /Engine/Maps/Entry -game -DriveLab -nullrhi -unattended -nosound -nosplash -nop4 -benchmark "-fps=$Fps" "-WheelTest=$scenario" "-abslog=$log"
        if ($LASTEXITCODE -ne 0) { throw "Wheel scenario $scenario failed: $LASTEXITCODE" }
        $result = Select-String -LiteralPath $log -Pattern "WHEEL_TEST $scenario PASS"
        if (-not $result) { throw "Missing success marker for $scenario; inspect $log" }
        $result.Line
    }
}
