param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[ValidateRange(1024,65535)][int]$Port=4183,
    [ValidateSet('loaded','friction','reset')][string[]]$Cases=@('loaded','friction','reset'))
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$webRoot=Join-Path $projectRoot 'Web\PhysicsLab'
$nodeCommand=Get-Command node.exe -ErrorAction SilentlyContinue
$nodePath=if($nodeCommand) {$nodeCommand.Source} else {Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe'}
if(-not (Test-Path -LiteralPath $nodePath -PathType Leaf)) { throw 'Node.js not found.' }
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$url="http://127.0.0.1:$Port"
try { $occupied=Invoke-RestMethod "$url/api/health" -TimeoutSec 1 } catch { $occupied=$null }
if($occupied) { throw "Test port $Port is occupied. Choose another -Port; no existing server will be stopped." }
$run=Join-Path $projectRoot ('Saved\Validation\UnrealWeb-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
New-Item -ItemType Directory -Path $run -Force | Out-Null
$server=$null; $engineProcess=$null
try {
    $server=Start-Process -FilePath $nodePath -ArgumentList @(('"'+(Join-Path $webRoot 'server.mjs')+'"'),'--port',"$Port") -WorkingDirectory $webRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput "$run\server.log" -RedirectStandardError "$run\server.err.log"
    $health=$null
    for($i=0;$i -lt 20;++$i) { try { $health=Invoke-RestMethod "$url/api/health" -TimeoutSec 1 } catch {}; if($health) {break}; if($server.HasExited) {throw 'Test bridge exited'}; Start-Sleep -Milliseconds 100 }
    if(-not $health -or $health.service -ne 'forkliftsim-live-bridge') { throw 'Test bridge not ready' }
    @{engine=$EngineRoot;port=$Port;cases=$Cases;dll_sha256=(Get-FileHash "$projectRoot\Binaries\Win64\UnrealEditor-ForkliftSim.dll").Hash;
        source_sha256=@(Get-Item "$projectRoot\Source\ForkliftSim\WebTelemetry.cpp","$webRoot\bridge.mjs","$webRoot\dist\live-state.mjs" | Get-FileHash | Select-Object Path,Hash)
    } | ConvertTo-Json -Depth 5 | Set-Content "$run\manifest.json" -Encoding UTF8
    foreach($case in $Cases) {
        $scenario=switch($case) {'loaded' {'swerve_hydraulic_loaded'} 'friction' {'swerve_friction_live'} 'reset' {'swerve_payload_reset'}}
        $log=Join-Path $run "$case.log"
        $launch=@(('"'+$projectRoot+'\ForkliftSim.uproject"'),'/Engine/Maps/Entry','-game','-StabilityLab','-ReferencePhysics',"-StabilityTest=$scenario","-WebTelemetry=$url/api/telemetry",'-nullrhi','-unattended','-nosound','-nosplash','-nop4','-NoAssetRegistryCacheWrite','-benchmark','-fps=60',('-abslog="'+$log+'"'))
        Write-Output "RUN actual Unreal -> bridge -> browser state: $case"
        $engineProcess=Start-Process -FilePath $editor -ArgumentList $launch -WorkingDirectory $projectRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput "$run\$case.stdout.log" -RedirectStandardError "$run\$case.stderr.log"
        $finished=$false
        for($wait=0;$wait -lt 120;++$wait) { if($engineProcess.WaitForExit(1000)) {$finished=$true;break} }
        if(-not $finished) { throw "Unreal $case timed out" }
        if(-not (Select-String -LiteralPath $log -Pattern "STABILITY_TEST $scenario PASS" -Quiet)) { throw "Unreal physics failed: $case" }
        & $nodePath (Join-Path $webRoot 'verify-unreal.mjs') $url $case
        if($LASTEXITCODE -ne 0) { throw "Actual telemetry verification failed: $case" }
        Write-Output "UNREAL_PHYSICS_AND_WEB PASS $case"
    }
    Write-Output "UNREAL_WEB_SUITE PASS $($Cases.Count) cases | $run"
} finally {
    if($engineProcess -and -not $engineProcess.HasExited) { Stop-Process -Id $engineProcess.Id }
    if($server -and -not $server.HasExited) { Stop-Process -Id $server.Id }
}
