param([ValidateRange(1024,65535)][int]$Port=4174,[switch]$OpenBrowser,[switch]$DryRun,[switch]$AutoPort,[switch]$PassThru)
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$webRoot=Join-Path $projectRoot 'Web\PhysicsLab'
if($AutoPort) {
    $preferred=$Port
    $listeners=@(Get-NetTCPConnection -State Listen -ErrorAction SilentlyContinue | Select-Object -ExpandProperty LocalPort -Unique)
    for($candidate=$preferred;$candidate -le [math]::Min(65535,$preferred+20);++$candidate) {
        if($candidate -notin $listeners) {$Port=$candidate;break}
        $existing=$null
        try {$existing=Invoke-RestMethod -Uri "http://127.0.0.1:$candidate/api/health" -TimeoutSec 1 -ErrorAction Stop} catch {}
        if($existing -and $existing.service -eq 'forkliftsim-live-bridge' -and $existing.project_root -eq $projectRoot) {$Port=$candidate;break}
        if($candidate -eq [math]::Min(65535,$preferred+20)) {throw 'No local dashboard port available; use Start.bat -WebPort.'}
    }
    if($Port -ne $preferred) {Write-Host "Dashboard port $preferred is occupied; ForkliftSim uses $Port."}
}
$url="http://127.0.0.1:$Port"
if($DryRun) { [pscustomobject]@{Port=$Port;Url=$url;TelemetryUrl="$url/api/telemetry";Server=(Join-Path $webRoot 'server.mjs')}; return }
function Get-BridgeHealth {
    try { return Invoke-RestMethod -Uri "$url/api/health" -TimeoutSec 1 -ErrorAction Stop }
    catch { return $null }
}
$health=Get-BridgeHealth
if($health -and ($health.service -ne 'forkliftsim-live-bridge' -or $health.project_root -ne $projectRoot)) { throw "Port $Port belongs to another server/project. Use -WebPort with a free port." }
if(-not $health) {
    $nodeCommand=Get-Command node.exe -ErrorAction SilentlyContinue
    $nodePath=if($nodeCommand) {$nodeCommand.Source} else {''}
    if(-not $nodePath) {
        foreach($candidate in @((Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe'),(Join-Path $env:ProgramFiles 'nodejs\node.exe'))) {
            if(Test-Path -LiteralPath $candidate -PathType Leaf) { $nodePath=$candidate; break }
        }
    }
    if(-not $nodePath) { throw 'Node.js is required for the local live dashboard (node.exe not found).' }
    $logDir=Join-Path $projectRoot 'Saved\WebDashboard'
    New-Item -ItemType Directory -Path $logDir -Force | Out-Null
    $serverArgs=@(('"'+(Join-Path $webRoot 'server.mjs')+'"'),'--port',"$Port")
    $process=Start-Process -FilePath $nodePath -ArgumentList $serverArgs -WorkingDirectory $webRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logDir "server-$Port.log") -RedirectStandardError (Join-Path $logDir "server-$Port.err.log")
    for($attempt=0;$attempt -lt 40;++$attempt) {
        $health=Get-BridgeHealth
        if($health) { break }
        if($process.HasExited) { throw "Live dashboard server exited. Port $Port may be occupied. See Saved\WebDashboard\server-$Port.err.log or use -WebPort." }
        Start-Sleep -Milliseconds 100
    }
    if(-not $health -or $health.service -ne 'forkliftsim-live-bridge' -or $health.project_root -ne $projectRoot) { throw "Live dashboard did not become ready at $url" }
    $process.Id | Set-Content -LiteralPath (Join-Path $logDir "server-$Port.pid") -Encoding ASCII
}
if($OpenBrowser) { Start-Process -FilePath $url }
Write-Host "Unreal live dashboard: $url"
if($PassThru) {[pscustomobject]@{Port=$Port;Url=$url;TelemetryUrl="$url/api/telemetry"}}
