# Verification only. Interactive use remains root Start.bat -> Editor -> Play.
param([switch]$Quick,[string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$folder=Join-Path $root ('Saved/Validation/JointSensors-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8))
New-Item -ItemType Directory -Path $folder -Force|Out-Null
$common=@{EngineRoot=$EngineRoot;ReducedMecanum=$true;SerialPhysics=$true;ReferenceModel=$true;UncappedBenchmark=$true}
$reports=[System.Collections.Generic.List[string]]::new()
function Trial([string]$Name,[hashtable]$Options){
 $lines=& "$PSScriptRoot/Test-Mecanum.ps1" @common @Options
 $lines|Tee-Object -FilePath (Join-Path $folder ($Name+'.txt'))|Write-Output
 $match=[regex]::Match(($lines -join "`n"),'Mecanum report: ([^\r\n]+)')
 if(-not $match.Success){throw "Missing native result: $Name"};$reports.Add($match.Groups[1].Value.Trim())
}
Trial 'nominal' @{Cases=@('joint_sensors')}
Trial 'outage' @{Cases=@('joint_sensors');JointSensorFault='all'}
if(-not $Quick){Trial 'frame30' @{Cases=@('joint_sensors');Fps=30};Trial 'physics1920' @{Cases=@('joint_sensors');PhysicsHz=1920}}
Trial 'physics-regression' @{Cases=@('carry','ramp_brake','halfstep','zero_grip')}
$python=Join-Path $env:USERPROFILE 'anaconda3/python.exe'
if(-not (Test-Path -LiteralPath $python)){$python=(Get-Command python -ErrorAction Stop).Source}
& $python -X utf8 "$PSScriptRoot/validate_joint_sensors.py" @reports --output (Join-Path $folder 'sensors.json')
if($LASTEXITCODE -ne 0){throw 'Joint sensor data validation failed'}
Write-Output "JOINT_SENSOR_SUITE PASS: $folder"
