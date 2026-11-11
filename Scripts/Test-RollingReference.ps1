param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',
    [ValidateSet(30,60,120)][int[]]$Fps=@(30,60),
    [ValidateSet(240,480,960)][int[]]$PhysicsHz=@(240,480),
    [ValidateSet('cylinder','sphere')][string[]]$Shapes=@('cylinder','sphere'),
    [double[]]$Speeds=@(2,-2), [switch]$LegacyTyreContacts)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$culture=[Globalization.CultureInfo]::InvariantCulture
foreach($speed in $Speeds) { if([double]::IsNaN($speed) -or [double]::IsInfinity($speed) -or [Math]::Abs($speed) -lt .1 -or [Math]::Abs($speed) -gt 8) { throw 'Reference speed must have magnitude 0.1..8 m/s' } }
$run=Join-Path $root ('Saved\Validation\Rolling-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
New-Item -ItemType Directory -Path $run -Force | Out-Null
@{fps=$Fps;physics_hz=$PhysicsHz;shapes=$Shapes;speeds=$Speeds;legacy_contacts=[bool]$LegacyTyreContacts;
    source_sha256=@(Get-ChildItem -LiteralPath "$root\Source\ForkliftSim" -File | Get-FileHash -Algorithm SHA256 | Select-Object Path,Hash);
    binary_sha256=(Get-FileHash "$root\Binaries\Win64\UnrealEditor-ForkliftSim.dll" -Algorithm SHA256).Hash
} | ConvertTo-Json -Depth 6 | Set-Content "$run\manifest.json" -Encoding UTF8
$extra=@(); if($LegacyTyreContacts) { $extra=@('-LegacyTyreContacts') }
$rows=@()
foreach($hz in $PhysicsHz) { foreach($frame in $Fps) { foreach($shape in $Shapes) { foreach($speed in $Speeds) {
    $v=$speed.ToString('G17',$culture); $dt=(1.0/$hz).ToString('G17',$culture)
    $log=Join-Path $run "$shape-$v-$frame-${hz}hz.log"
    Write-Output "RUN rolling shape=$shape v=$v fps=$frame physics_hz=$hz"
    & $editor "$root\ForkliftSim.uproject" /Engine/Maps/Entry -game -RollingReference "-ReferenceShape=$shape" "-ReferenceSpeed=$v" @extra -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite -benchmark "-fps=$frame" "-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=$dt" "-abslog=$log"
    $code=$LASTEXITCODE
    $line=(Select-String -LiteralPath $log -Pattern 'ROLLING_REFERENCE shape=').Line
    $pass=$code -eq 0 -and [bool]($line -match ' PASS$')
    $line | Write-Output
    $rows+=[pscustomobject]@{shape=$shape;speed=$speed;fps=$frame;physics_hz=$hz;pass=$pass;result=$line;log=$log}
    ConvertTo-Json -InputObject $rows -Depth 5 | Set-Content "$run\results.json" -Encoding UTF8
} } } }
Write-Output "Rolling report: $run"
if(@($rows | Where-Object {-not $_.pass}).Count) { throw 'Rolling reference failed; all requested cases collected.' }
Write-Output "ROLLING_SUITE PASS $($rows.Count) cases"
