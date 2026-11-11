# Automated qualification only. User entry is Start.bat.
param([string]$WebTelemetryUrl='http://127.0.0.1:4174/api/telemetry')
$ErrorActionPreference='Stop';$root=Split-Path -Parent $PSScriptRoot
$run=Join-Path $root ('Saved/Validation/Mission-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8));New-Item -ItemType Directory -Path $run -Force|Out-Null
$reports=[System.Collections.Generic.List[string]]::new()
function Trial([string]$Name,[string[]]$Cases,[bool]$Rendered){
 $lines=& "$PSScriptRoot/Test-Mecanum.ps1" -Cases $Cases -ReducedMecanum -SerialPhysics -ReferenceModel -Rendered:$Rendered -UncappedBenchmark -Fps 30 -WebTelemetryUrl $WebTelemetryUrl
 $lines|Tee-Object -FilePath (Join-Path $run ($Name+'.txt'))|Write-Output
 $m=[regex]::Match(($lines -join "`n"),'Mecanum report: ([^\r\n]+)');if(-not $m.Success){throw 'Missing native manifest'};$reports.Add($m.Groups[1].Value.Trim())
}
Trial 'mission' @('mission_blocked','mission_cancel','mission_watchdog','mission_retry','mission_sensor_recovery','mission_low','mission_avoid_pause','mission_high') $true
Trial 'handling-regression' @('handling_pick') $true
Trial 'drive-physics-regression' @('drive_sensor_encoder_fault','ramp_brake') $false
& (Join-Path $env:USERPROFILE 'anaconda3/python.exe') -X utf8 "$PSScriptRoot/validate_mission.py" @reports --output (Join-Path $run 'mission.json');if($LASTEXITCODE -ne 0){throw 'Mission independent validation failed'}
Write-Output "MISSION_SUITE PASS: $run"
