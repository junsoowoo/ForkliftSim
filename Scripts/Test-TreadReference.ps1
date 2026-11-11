param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[ValidateSet(30,60)][int[]]$Fps=@(30,60),[ValidateSet(16,32,64)][int[]]$Counts=@(16,32),[double[]]$Heights=@(.14,.145),[ValidateSet('balanced','high')][string]$Quality='high',[double[]]$PhaseFractions=@(0))
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$run=Join-Path $projectRoot ('Saved\Validation\TreadReference-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
New-Item -ItemType Directory -Path $run -Force | Out-Null
foreach($phase in $PhaseFractions) {if([double]::IsNaN($phase) -or [double]::IsInfinity($phase) -or $phase -lt 0 -or $phase -gt .5){throw 'Phase fractions must be between 0 and 0.5.'}}
@{tread_reference_schema=2;fps=$Fps;quality=$Quality;counts=$Counts;heights_m=$Heights;phase_fractions=$PhaseFractions;dll_sha256=(Get-FileHash "$projectRoot\Binaries\Win64\UnrealEditor-ForkliftSim.dll").Hash;config_sha256=@(Get-ChildItem "$projectRoot\Config\*.json" | Get-FileHash | Select-Object Path,Hash)} | ConvertTo-Json -Depth 5 | Set-Content "$run\manifest.json" -Encoding UTF8
$rows=@(); $culture=[Globalization.CultureInfo]::InvariantCulture
function Read-Scalar([string]$line,[string]$key) {
    $match=[regex]::Match($line,'(?:\s|^)'+[regex]::Escape($key)+'=([-+0-9.eE]+)')
    if($match.Success) {return [double]::Parse($match.Groups[1].Value,$culture)}
    return $null
}
foreach($frameRate in $Fps){foreach($count in $Counts){foreach($height in $Heights){foreach($phase in $PhaseFractions){
    $h=$height.ToString('G17',$culture); $p=$phase.ToString('G17',$culture); $log=Join-Path $run "tread-$count-$frameRate-$h-phase-$p.log"
    Write-Output "RUN fixed-hub radial tread count=$count fps=$frameRate height=$h phase=$p"
    $extra=@(); if($Quality -eq 'balanced') {$extra=@('-TreadBalancedReference')}
    & $editor "$projectRoot\ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -TreadReference -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite -benchmark "-fps=$frameRate" "-TreadCount=$count" "-TreadHubHeight=$h" "-TreadPhaseFraction=$p" "-abslog=$log" @extra
    $code=$LASTEXITCODE; $line=(Select-String -LiteralPath $log -Pattern 'TREAD_REFERENCE count=').Line
    $pass=$code -eq 0 -and $line -match ' PASS$'; $line | Write-Output
    $rows+=[pscustomobject]@{fps=$frameRate;count=$count;height_m=$height;phase_fraction=$phase;pass=[bool]$pass;normal_n=(Read-Scalar $line 'normal_n');analytic_n=(Read-Scalar $line 'analytic_n');continuum_n=(Read-Scalar $line 'continuum_n');solver_relative_error=(Read-Scalar $line 'force_error');continuum_relative_error=(Read-Scalar $line 'continuum_error');result=$line;log=$log}
    ConvertTo-Json -InputObject $rows -Depth 5 | Set-Content "$run\results.json" -Encoding UTF8
}}}}
Write-Output "Tread reference report: $run"
if(@($rows | Where-Object {-not $_.pass}).Count){throw 'Tread fixed-hub force/passivity reference failed'}
Write-Output "TREAD_REFERENCE_SUITE PASS $($rows.Count) cases"
