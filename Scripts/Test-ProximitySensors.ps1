# Reproducible verification; interactive entry remains Start.bat.
param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[string]$WebTelemetryUrl='',[string]$SceneReport='')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$folder=Join-Path $root ('Saved/Validation/ProximitySensors-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8))
New-Item -ItemType Directory -Path $folder -Force|Out-Null
$common=@{EngineRoot=$EngineRoot;ReducedMecanum=$true;SerialPhysics=$true;ReferenceModel=$true;UncappedBenchmark=$true}
$reports=[System.Collections.Generic.List[string]]::new()
function Trial([string]$Name,[hashtable]$Options){
 $lines=& "$PSScriptRoot/Test-Mecanum.ps1" @common @Options
 $lines|Tee-Object -FilePath (Join-Path $folder ($Name+'.txt'))|Write-Output
 $match=[regex]::Match(($lines -join "`n"),'Mecanum report: ([^\r\n]+)');if(-not $match.Success){throw "Missing native report: $Name"};$reports.Add($match.Groups[1].Value.Trim())
}
if($SceneReport){$reports.Add((Resolve-Path -LiteralPath $SceneReport).Path)}else{Trial 'scene' @{Cases=@('settle');ProximityValidate=$true}}
Trial 'outage' @{Cases=@('carry');ProximitySensorFault='all';WebTelemetryUrl=$WebTelemetryUrl}
Trial 'normal-and-racks' @{Cases=@('carry','rack_pick_low','rack_pick_high')}
Trial 'frame30' @{Cases=@('carry');Fps=30}
$python=Join-Path $env:USERPROFILE 'anaconda3/python.exe';if(-not(Test-Path -LiteralPath $python)){$python=(Get-Command python -ErrorAction Stop).Source}
& $python -X utf8 "$PSScriptRoot/validate_proximity_sensors.py" @reports --output (Join-Path $folder 'sensors.json')
if($LASTEXITCODE -ne 0){throw 'Proximity sensor CSV validation failed'}
Write-Output "PROXIMITY_SENSOR_SUITE PASS: $folder"
