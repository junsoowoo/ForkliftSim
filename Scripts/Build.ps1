param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.6')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
# MSVC response-file paths can fail under a non-ASCII parent directory.
# Use an ASCII junction to the same files; never copy or relocate the user's project.
$pathBytes = [System.Text.Encoding]::UTF8.GetBytes($projectRoot)
$sha = [System.Security.Cryptography.SHA256]::Create()
try { $pathHash = [BitConverter]::ToString($sha.ComputeHash($pathBytes)).Replace('-', '').Substring(0, 12) }
finally { $sha.Dispose() }
$buildRoot = Join-Path $env:TEMP "ForkliftSim-$pathHash"
if (Test-Path -LiteralPath $buildRoot) {
    $junction = Get-Item -LiteralPath $buildRoot
    if ($junction.LinkType -ne 'Junction' -or $junction.Target -ne $projectRoot) { throw "Build alias is occupied: $buildRoot" }
} else {
    New-Item -ItemType Junction -Path $buildRoot -Target $projectRoot | Out-Null
}
& (Join-Path $EngineRoot 'Engine\Build\BatchFiles\Build.bat') ForkliftSimEditor Win64 Development "-Project=$buildRoot\ForkliftSim.uproject" -WaitMutex -NoHotReloadFromIDE -NoUBA -NoUBTMakefiles -MaxParallelActions=2
if ($LASTEXITCODE -ne 0) { throw "Unreal build failed: $LASTEXITCODE" }
