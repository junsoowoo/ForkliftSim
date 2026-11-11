param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[string]$InitialReport='',[string]$WebTelemetryUrl='')
$ErrorActionPreference='Stop';$root=Split-Path -Parent $PSScriptRoot
$folder=Join-Path $root ('Saved/Validation/LidarSafety-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8));New-Item -ItemType Directory -Path $folder -Force|Out-Null
$common=@{EngineRoot=$EngineRoot;ReducedMecanum=$true;SerialPhysics=$true;ReferenceModel=$true;UncappedBenchmark=$true};$reports=[System.Collections.Generic.List[string]]::new()
function Trial([string]$Name,[hashtable]$Options){$lines=& "$PSScriptRoot/Test-Mecanum.ps1" @common @Options;$lines|Tee-Object -FilePath (Join-Path $folder ($Name+'.txt'))|Write-Output;$m=[regex]::Match(($lines -join "`n"),'Mecanum report: ([^\r\n]+)');if(-not $m.Success){throw 'Native report missing'};$reports.Add($m.Groups[1].Value.Trim())}
if($InitialReport){$reports.Add((Resolve-Path -LiteralPath $InitialReport).Path)}else{Trial 'initial' @{Cases=@('vehicle_guard_front','vehicle_guard_fault')}}
Trial 'directions-and-estop' @{Cases=@('vehicle_guard_creep','vehicle_guard_rear','vehicle_guard_side','vehicle_guard_diagonal','vehicle_guard_estop','vehicle_guard_carry')}
Trial 'frame30-web' @{Cases=@('vehicle_guard_front');Fps=30;WebTelemetryUrl=$WebTelemetryUrl}
Trial 'legacy-physics' @{Cases=@('carry','ramp_brake')}
$python=Join-Path $env:USERPROFILE 'anaconda3/python.exe';if(-not(Test-Path -LiteralPath $python)){$python=(Get-Command python -ErrorAction Stop).Source}
& $python -X utf8 "$PSScriptRoot/validate_lidar_safety.py" @reports --output (Join-Path $folder 'safety.json');if($LASTEXITCODE -ne 0){throw 'LiDAR safety CSV validation failed'}
Write-Output "LIDAR_SAFETY_SUITE PASS: $folder"
