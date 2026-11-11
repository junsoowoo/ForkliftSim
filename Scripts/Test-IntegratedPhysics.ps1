param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',
    [ValidateSet(30,60,120)][int]$Fps=60, [ValidateSet(240,480,960)][int]$PhysicsHz=480,
    [string[]]$Cases=@('flat','ramp','step','halfstep','split','ramp_eccentric','step_eccentric'))
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$run=Join-Path $projectRoot ('Saved\Validation\Integrated-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
New-Item -ItemType Directory -Path $run -Force | Out-Null
$reference=Get-Content -LiteralPath "$projectRoot\Config\PhysicsReference.json" -Raw | ConvertFrom-Json
@{reference=$reference.id;fps=$Fps;physics_hz=$PhysicsHz;engine=$EngineRoot;
    binary_sha256=(Get-FileHash -LiteralPath "$projectRoot\Binaries\Win64\UnrealEditor-ForkliftSim.dll" -Algorithm SHA256).Hash;
    source_sha256=@(Get-ChildItem -LiteralPath "$projectRoot\Source\ForkliftSim" -File | Get-FileHash -Algorithm SHA256 | Select-Object Path,Hash)
} | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath "$run\manifest.json" -Encoding UTF8
$dt=(1.0/$PhysicsHz).ToString('G17',[Globalization.CultureInfo]::InvariantCulture)
$results=@()
foreach($case in $Cases) {
    if($case -notin @('flat','ramp','step','halfstep','split','ramp_eccentric','step_eccentric','ramp_brake','ramp_brake_input','ramp_brake_ice')) { throw "Unknown case $case" }
    $scenario="swerve_integrated_$case"
    $log=Join-Path $run "$case-$Fps-${PhysicsHz}hz.log"
    Write-Output "RUN integrated $case fps=$Fps physics_hz=$PhysicsHz"
    & $editor "$projectRoot\ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -ReferencePhysics -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite -benchmark "-fps=$Fps" "-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=$dt" "-StabilityTest=$scenario" "-abslog=$log"
    $exitCode=$LASTEXITCODE
    $text=Get-Content -LiteralPath $log -Raw
    $pass=$exitCode -eq 0 -and $text.Contains("STABILITY_TEST $scenario PASS")
    $line=(Select-String -LiteralPath $log -Pattern 'INTEGRATED_CHECK|RAMP_BRAKE_CHECK').Line
    $line | Write-Output
    $results+= [pscustomobject]@{case=$case;fps=$Fps;physics_hz=$PhysicsHz;pass=$pass;result=$line;log=$log}
    ConvertTo-Json -InputObject $results -Depth 5 | Set-Content -LiteralPath (Join-Path $run 'results.json') -Encoding UTF8
}
ConvertTo-Json -InputObject $results -Depth 5 | Set-Content -LiteralPath (Join-Path $run 'results.json') -Encoding UTF8
Write-Output "Integrated report: $run"
if(@($results | Where-Object {-not $_.pass}).Count) { throw 'Integrated physics failed; all requested cases were collected.' }
Write-Output "INTEGRATED_SUITE PASS $($results.Count) cases"
