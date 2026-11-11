param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$folder=Join-Path $root ('Saved/Validation/Cameras-'+(Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $folder -Force|Out-Null
$log=Join-Path $folder 'CameraValidation.log'
& (Join-Path $EngineRoot 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe') "$root\ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -Mecanum -ReducedMecanum -WarehouseLab -CameraViews -CameraValidate -SingleThreadedPhysics -StabilityCapture -StabilityCaptureTime=8 -RenderOffscreen -windowed -ResX=1280 -ResY=800 -ForceRes -dx11 -unattended -nosound -nosplash -NoAssetRegistryCacheWrite '-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=0.0010416666666666667' '-ExecCmds=r.AntiAliasingMethod 1,r.ScreenPercentage 75,sg.PostProcessQuality 1,sg.ShadowQuality 1,r.Shadow.MaxResolution 1024' "-abslog=$log"
if($LASTEXITCODE -ne 0 -or -not (Select-String -LiteralPath $log -Pattern 'CAMERA_VALIDATION PASS' -Quiet)){throw "Camera validation failed: $log"}
$reportLine=(Select-String -LiteralPath $log -Pattern 'CAMERA_REPORT (.+)$'|Select-Object -Last 1).Matches[0].Groups[1].Value
$reportFolder=if([IO.Path]::IsPathRooted($reportLine)){$reportLine}else{[IO.Path]::GetFullPath((Join-Path (Join-Path $EngineRoot 'Engine/Binaries/Win64') $reportLine))}
Copy-Item -LiteralPath (Join-Path $reportFolder 'cameras.json') -Destination (Join-Path $folder 'cameras.json')
$data=Get-Content -LiteralPath (Join-Path $folder 'cameras.json') -Raw|ConvertFrom-Json
foreach($sensor in @($data.front,$data.rear)){
 if($sensor.delivery_sim_s-$sensor.capture_sim_s+1e-6 -lt $sensor.configured_latency_s){throw 'Sensor delivered before configured latency'}
 if($sensor.age_s -lt 0 -or $sensor.valid_fraction -lt 0 -or $sensor.valid_fraction -gt 1){throw 'Invalid sensor timing or validity fraction'}
}
if($data.rear.rgb_sequence -ne $data.rear.depth_sequence -or -not $data.rear.registered_same_exposure){throw 'RGB-D channels are not synchronized'}
Write-Output "CAMERA_SUITE PASS: real GPU depth, metric targets, near/far invalidity, latency and paired RGB-D. Report: $folder"
