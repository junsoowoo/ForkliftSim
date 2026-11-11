param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[string]$Python='python',
    [ValidateSet(16,32,48,64,96,128)][int[]]$Counts=@(64),[double[]]$Heights=@(.14,.145),
    [double[]]$PhaseFractions=@(0,.5),[ValidateSet(30,60)][int]$Fps=60,[switch]$Probe,
    [ValidateSet(240,480,960,1920)][int]$PhysicsHz=480,
    [ValidateRange(96,127)][int]$PositionIterations=127,[ValidateRange(16,127)][int]$VelocityIterations=32)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$run=Join-Path $root ('Saved\Validation\CoupledTread-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
New-Item -ItemType Directory -Path $run -Force | Out-Null
$culture=[Globalization.CultureInfo]::InvariantCulture
foreach($p in $PhaseFractions){if([double]::IsNaN($p) -or [double]::IsInfinity($p) -or $p -lt 0 -or $p -gt .5){throw 'Phase fraction outside 0..0.5'}}
@{schema=2;model='cohesive-ring-v1';fps=$Fps;physics_hz=$PhysicsHz;position_iterations=$PositionIterations;velocity_iterations=$VelocityIterations;counts=$Counts;heights_m=$Heights;phase_fractions=$PhaseFractions;probe=[bool]$Probe;
    dll_sha256=(Get-FileHash "$root/Binaries/Win64/UnrealEditor-ForkliftSim.dll").Hash;
    oracle_sha256=(Get-FileHash "$PSScriptRoot/tread_ring_reference.py").Hash;
    configs=@(Get-ChildItem "$root/Config/*.json" | Get-FileHash | Select-Object Path,Hash)} |
    ConvertTo-Json -Depth 5 | Set-Content "$run/manifest.json" -Encoding UTF8
$rows=@()
$env:OPENBLAS_NUM_THREADS='1'
foreach($n in $Counts){foreach($height in $Heights){foreach($phase in $PhaseFractions){
    $h=$height.ToString('G17',$culture); $p=$phase.ToString('G17',$culture)
    Write-Output "RUN cohesive ring n=$n h=$h phase=$p"
    $raw=& $Python "$PSScriptRoot/tread_ring_reference.py" --counts $n --heights $h --phases $p --config-root "$root/Config"
    if($LASTEXITCODE -ne 0){throw 'Independent energy oracle failed'}
    $oracle=$raw | ConvertFrom-Json
    $raw | Set-Content "$run/oracle-$n-$h-$p.json" -Encoding UTF8
    if(-not $oracle.success){throw 'Independent energy/contact solve did not converge'}
    $expected=([double]$oracle.normal_n).ToString('G17',$culture)
    $log=Join-Path $run "ring-$n-$h-$p.log"
    $step=(1.0/$PhysicsHz).ToString('G17',$culture)
    $extra=@("-RingReferenceHz=$PhysicsHz","-RingReferenceIterations=$PositionIterations","-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubsteps=128","-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=$step",
        "-ExecCmds=p.Chaos.Solver.Iterations.Position $PositionIterations,p.Chaos.Solver.Iterations.Velocity $VelocityIterations")
    if($Probe){$extra+='-TreadProbe'}
    & $editor "$root/ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -TreadReference -CoupledTread -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite -benchmark "-fps=$Fps" "-TreadCount=$n" "-TreadHubHeight=$h" "-TreadPhaseFraction=$p" "-TreadReferenceNormal=$expected" "-abslog=$log" @extra
    $code=$LASTEXITCODE
    $line=(Select-String -LiteralPath $log -Pattern 'TREAD_REFERENCE count=').Line
    $line | Write-Output
    $match=[regex]::Match($line,'\snormal_n=([-+0-9.eE]+)')
    $normal=if($match.Success){[double]::Parse($match.Groups[1].Value,$culture)}else{$null}
    $num=(Select-String -LiteralPath $log -Pattern 'RING_NUMERICS max_inner_s=').Line
    $num | Write-Output
    $dtMatch=[regex]::Match($num,'max_inner_s=([-+0-9.eE]+)')
    $actualDt=if($dtMatch.Success){[double]::Parse($dtMatch.Groups[1].Value,$culture)}else{0}
    $observed=$actualDt -gt 0 -and $actualDt -le 1.0/$PhysicsHz+1e-9 -and $num -match "position_iterations=$PositionIterations " -and $num -match "velocity_iterations=$VelocityIterations "
    $rows+=[pscustomobject]@{count=$n;height_m=$height;phase_fraction=$phase;fps=$Fps;physics_hz=$PhysicsHz;max_inner_s=$actualDt;numerics_observed=[bool]$observed;pass=($code -eq 0 -and $line -match ' PASS$' -and $observed);normal_n=$normal;oracle=$oracle;numerics=$num;result=$line;log=$log}
    ConvertTo-Json -InputObject $rows -Depth 7 | Set-Content "$run/results.json" -Encoding UTF8
}}}
Write-Output "Coupled ring report: $run"
if(@($rows | Where-Object {-not $_.pass}).Count){throw 'Coupled ring reference failed; inspect every collected case.'}
Write-Output "COUPLED_RING_SUITE PASS $($rows.Count) cases"
