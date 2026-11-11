# Verification only; the user's launcher remains Start.bat.
param([string]$InitialReport='',[string]$WebTelemetryUrl='http://127.0.0.1:4174/api/telemetry')
$ErrorActionPreference='Stop';$root=Split-Path -Parent $PSScriptRoot
$folder=Join-Path $root ('Saved/Validation/Handling-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8));New-Item -ItemType Directory $folder -Force|Out-Null
$reports=[System.Collections.Generic.List[string]]::new()
function Trial([string]$Name,[string[]]$Cases,[bool]$Rendered){
 $lines=& "$PSScriptRoot/Test-Mecanum.ps1" -Cases $Cases -ReducedMecanum -SerialPhysics -ReferenceModel -Rendered:$Rendered -UncappedBenchmark -Fps 30 -WebTelemetryUrl $WebTelemetryUrl
 $lines|Tee-Object -FilePath (Join-Path $folder ($Name+'.txt'))|Write-Output
 $match=[regex]::Match(($lines -join "`n"),'Mecanum report: ([^\r\n]+)');if(-not $match.Success){throw 'Missing report'};$reports.Add($match.Groups[1].Value.Trim())
}
if($InitialReport){$reports.Add((Resolve-Path -LiteralPath $InitialReport).Path);$done=@(Get-Content -LiteralPath (Join-Path $InitialReport 'results.json') -Raw|ConvertFrom-Json|ForEach-Object {$_.case})}else{$done=@()}
$remaining=@('handling_estop','handling_cancel','handling_obstacle','handling_camera_loss','handling_sensor_loss','handling_pick_low','handling_align','handling_pick','handling_place_low','handling_pick_high','handling_place_high')|Where-Object {$_ -notin $done}
Trial 'handling' @($remaining) $true
Trial 'regression' @('drive_sensor_straight','carry','ramp_brake') $false
& (Join-Path $env:USERPROFILE 'anaconda3/python.exe') -X utf8 "$PSScriptRoot/validate_handling.py" @reports --output (Join-Path $folder 'handling.json');if($LASTEXITCODE -ne 0){throw 'Handling validation failed'}
Write-Output "HANDLING_SUITE PASS: $folder"
