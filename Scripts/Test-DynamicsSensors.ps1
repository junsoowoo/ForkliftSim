# Verification only. The single user launcher remains Start.bat.
param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[string]$WebTelemetryUrl='')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$folder=Join-Path $root ('Saved/Validation/DynamicsSensors-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8))
New-Item -ItemType Directory -Path $folder -Force|Out-Null
$common=@{EngineRoot=$EngineRoot;ReducedMecanum=$true;SerialPhysics=$true;ReferenceModel=$true;UncappedBenchmark=$true}
$reports=[System.Collections.Generic.List[string]]::new()
function Trial([string]$Name,[hashtable]$Options){
 $lines=& "$PSScriptRoot/Test-Mecanum.ps1" @common @Options
 $lines|Tee-Object -FilePath (Join-Path $folder ($Name+'.txt'))|Write-Output
 $match=[regex]::Match(($lines -join "`n"),'Mecanum report: ([^\r\n]+)')
 if(-not $match.Success){throw "Missing native result: $Name"};$reports.Add($match.Groups[1].Value.Trim())
}
Trial 'normal' @{Cases=@('settle','carry','eccentric_lift','joint_sensors','halfstep','ramp_brake')}
Trial 'fault' @{Cases=@('carry');DynamicsSensorFault='all';WebTelemetryUrl=$WebTelemetryUrl}
Trial 'frame30' @{Cases=@('carry');Fps=30}
Trial 'physics1920' @{Cases=@('carry');PhysicsHz=1920}
$python=Join-Path $env:USERPROFILE 'anaconda3/python.exe'
if(-not (Test-Path -LiteralPath $python)){$python=(Get-Command python -ErrorAction Stop).Source}
& $python -X utf8 "$PSScriptRoot/validate_dynamics_sensors.py" @reports --output (Join-Path $folder 'sensors.json')
if($LASTEXITCODE -ne 0){throw 'Dynamics sensor data validation failed'}
Write-Output "DYNAMICS_SENSOR_SUITE PASS: $folder"
