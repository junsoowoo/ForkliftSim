# Verification only. User entry remains Start.bat.
param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[string]$InitialReport='',[string]$WebTelemetryUrl='')
$ErrorActionPreference='Stop';$root=Split-Path -Parent $PSScriptRoot
$folder=Join-Path $root ('Saved/Validation/PalletPerception-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8));New-Item -ItemType Directory -Path $folder -Force|Out-Null
$common=@{EngineRoot=$EngineRoot;ReducedMecanum=$true;SerialPhysics=$true;ReferenceModel=$true;UncappedBenchmark=$true};$reports=[System.Collections.Generic.List[string]]::new()
function Trial([string]$Name,[hashtable]$Options){$lines=& "$PSScriptRoot/Test-Mecanum.ps1" @common @Options;$lines|Tee-Object -FilePath (Join-Path $folder ($Name+'.txt'))|Write-Output;$m=[regex]::Match(($lines -join "`n"),'Mecanum report: ([^\r\n]+)');if(-not $m.Success){throw 'Missing native report'};$reports.Add($m.Groups[1].Value.Trim())}
if($InitialReport){$reports.Add((Resolve-Path -LiteralPath $InitialReport).Path)}else{Trial 'aligned-insert' @{Cases=@('pallet_vision_aligned','pallet_vision_insert');Rendered=$true;Fps=30}}
Trial 'geometry-negative-high' @{Cases=@('pallet_vision_yaw','pallet_vision_shift','pallet_vision_box','pallet_vision_disabled','pallet_vision_high');Rendered=$true;Fps=30}
Trial 'proximity-outage' @{Cases=@('pallet_vision_aligned');Rendered=$true;Fps=30;ProximitySensorFault='all'}
Trial 'live-insert' @{Cases=@('pallet_vision_insert');Rendered=$true;Fps=30;WebTelemetryUrl=$WebTelemetryUrl}
Trial 'legacy-drive-regression' @{Cases=@('drive_sensor_straight','carry','ramp_brake')}
& (Join-Path $env:USERPROFILE 'anaconda3/python.exe') -X utf8 "$PSScriptRoot/validate_pallet_perception.py" @reports --output (Join-Path $folder 'perception.json');if($LASTEXITCODE -ne 0){throw 'Perception CSV validation failed'}
Write-Output "PALLET_PERCEPTION_SUITE PASS: $folder"
