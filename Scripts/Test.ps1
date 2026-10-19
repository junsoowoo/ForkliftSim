param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.6', [ValidateSet('All','Model','Physics')][string]$Mode = 'All')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$editor = Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$logDir = Join-Path $projectRoot 'Saved\Logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
if ($Mode -in @('All','Model')) {
    $log = Join-Path $logDir 'ModelTests.log'
    & $editor "$projectRoot\ForkliftSim.uproject" -nullrhi -unattended -nosound -nosplash -nop4 '-ExecCmds=Automation RunTests ForkliftSim.Model' '-TestExit=Automation Test Queue Empty' "-abslog=$log"
    if ($LASTEXITCODE -ne 0) { throw "Model tests failed: $LASTEXITCODE" }
    $results = Select-String -LiteralPath $log -Pattern 'Result=\{Success\}.*ForkliftSim.Model'
    if ($results.Count -ne 3) { throw 'Expected three successful model tests; inspect ModelTests.log' }
    $results.Line
}
if ($Mode -in @('All','Physics')) {
    foreach ($scenario in @('baseline','overload','sensor_timeout')) {
        $log = Join-Path $logDir "$scenario.log"
        & $editor "$projectRoot\ForkliftSim.uproject" /Engine/Maps/Entry -game -nullrhi -unattended -nosound -nosplash -nop4 -benchmark -fps=60 -ForkliftSmokeTest "-LiftTest=$scenario" "-abslog=$log"
        if ($LASTEXITCODE -ne 0) { throw "Physics scenario $scenario failed: $LASTEXITCODE" }
        $result = Select-String -LiteralPath $log -Pattern "FORKLIFT_TEST $scenario PASS"
        if (-not $result) { throw "Missing success marker for $scenario" }
        $result.Line
    }
}
