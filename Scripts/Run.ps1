param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.6', [switch]$Editor)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$arguments = @("`"$projectRoot\ForkliftSim.uproject`"")
if (-not $Editor) { $arguments += @('-game', '-windowed', '-ResX=1280', '-ResY=800', '-dx11') }
# The visible application is intentional: this script opens the requested interactive simulator.
Start-Process -FilePath (Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor.exe') -ArgumentList $arguments
