# Automated verification; interactive entry remains root Start.bat.
param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[string]$WebTelemetryUrl='',[string]$InitialReport='')
$ErrorActionPreference='Stop';$root=Split-Path -Parent $PSScriptRoot
$folder=Join-Path $root ('Saved/Validation/SensorDrive-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8));New-Item -ItemType Directory -Path $folder -Force|Out-Null
$common=@{EngineRoot=$EngineRoot;ReducedMecanum=$true;SerialPhysics=$true;ReferenceModel=$true;UncappedBenchmark=$true};$reports=[System.Collections.Generic.List[string]]::new()
function Trial([string]$Name,[hashtable]$Options){$lines=& "$PSScriptRoot/Test-Mecanum.ps1" @common @Options;$lines|Tee-Object -FilePath (Join-Path $folder ($Name+'.txt'))|Write-Output;$m=[regex]::Match(($lines -join "`n"),'Mecanum report: ([^\r\n]+)');if(-not $m.Success){throw 'Native report missing'};$reports.Add($m.Groups[1].Value.Trim())}
if($InitialReport){$reports.Add((Resolve-Path -LiteralPath $InitialReport).Path);Trial 'movement' @{Cases=@('drive_sensor_strafe','drive_sensor_curve')}}else{Trial 'movement-slip' @{Cases=@('drive_sensor_straight','drive_sensor_strafe','drive_sensor_curve','drive_sensor_slip')}}
Trial 'faults' @{Cases=@('drive_sensor_encoder_fault','drive_sensor_imu_fault','drive_sensor_watchdog')}
Trial 'frame30-web' @{Cases=@('drive_sensor_straight');Fps=30;WebTelemetryUrl=$WebTelemetryUrl}
Trial 'warehouse' @{Cases=@('drive_sensor_warehouse');WarehouseMap=$true}
Trial 'legacy' @{Cases=@('carry','ramp_brake')}
& (Join-Path $env:USERPROFILE 'anaconda3/python.exe') -X utf8 "$PSScriptRoot/validate_sensor_drive.py" @reports --output (Join-Path $folder 'drive.json');if($LASTEXITCODE -ne 0){throw 'Sensor drive CSV validation failed'}
Write-Output "SENSOR_DRIVE_SUITE PASS: $folder"
