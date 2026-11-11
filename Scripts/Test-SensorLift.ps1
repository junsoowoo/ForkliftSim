# Verification only. Interactive user entry is Start.bat.
param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[string]$WebTelemetryUrl='',[string]$TrackingReport='')
$ErrorActionPreference='Stop';$root=Split-Path -Parent $PSScriptRoot
$folder=Join-Path $root ('Saved/Validation/SensorLift-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8));New-Item -ItemType Directory -Path $folder -Force|Out-Null
$common=@{EngineRoot=$EngineRoot;ReducedMecanum=$true;SerialPhysics=$true;ReferenceModel=$true;UncappedBenchmark=$true};$reports=[System.Collections.Generic.List[string]]::new()
function Trial([string]$Name,[hashtable]$Options){
 $lines=& "$PSScriptRoot/Test-Mecanum.ps1" @common @Options;$lines|Tee-Object -FilePath (Join-Path $folder ($Name+'.txt'))|Write-Output
 $m=[regex]::Match(($lines -join "`n"),'Mecanum report: ([^\r\n]+)');if(-not $m.Success){throw "Missing report: $Name"};$reports.Add($m.Groups[1].Value.Trim())
}
if($TrackingReport){$reports.Add((Resolve-Path -LiteralPath $TrackingReport).Path)}else{Trial 'tracking' @{Cases=@('lift_sensor_empty','lift_sensor_loaded')}}
Trial 'fault-up-web' @{Cases=@('lift_sensor_fault_up');WebTelemetryUrl=$WebTelemetryUrl}
Trial 'fault-down-pressure-watchdog' @{Cases=@('lift_sensor_fault_down','lift_sensor_pressure','lift_sensor_watchdog')}
Trial 'frame30' @{Cases=@('lift_sensor_loaded');Fps=30}
Trial 'legacy-physics-regression' @{Cases=@('carry','ramp_brake')}
$python=Join-Path $env:USERPROFILE 'anaconda3/python.exe';if(-not(Test-Path -LiteralPath $python)){$python=(Get-Command python -ErrorAction Stop).Source}
& $python -X utf8 "$PSScriptRoot/validate_sensor_lift.py" @reports --output (Join-Path $folder 'controller.json');if($LASTEXITCODE -ne 0){throw 'Sensor lift validation failed'}
Write-Output "SENSOR_LIFT_SUITE PASS: $folder"
