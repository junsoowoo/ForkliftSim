param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[ValidateSet(480,960,1920)][int]$PhysicsHz=960,
    [ValidateSet(30,60)][int]$Fps=60,[ValidateSet(16,32,64)][int]$Count=64,
    [ValidateSet('Mixed','Torque')][string[]]$Modes=@('Mixed','Torque'))
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$run=Join-Path $root ('Saved\Validation\Constitutive-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
New-Item -ItemType Directory -Path $run -Force|Out-Null
@{physics_hz=$PhysicsHz;fps=$Fps;count=$Count;modes=$Modes;scope='Isolated core law; no floor/gravity/seam contact. Internal-step known wrench. Not real-wheel calibration.';
  dll_sha256=(Get-FileHash "$root\Binaries\Win64\UnrealEditor-ForkliftSim.dll").Hash;
  configs=@(Get-ChildItem "$root\Config\*.json"|Get-FileHash|Select-Object Path,Hash)}|ConvertTo-Json -Depth 5|Set-Content "$run\manifest.json" -Encoding UTF8
$rows=@();$culture=[Globalization.CultureInfo]::InvariantCulture;$step=(1.0/$PhysicsHz).ToString('G17',$culture)
foreach($mode in $Modes){
 $log=Join-Path $run "$mode.log";$extra=@();if($mode -eq 'Torque'){$extra+='-TreadCoreTorqueOnly'}
 & $editor "$root\ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -TreadReference -TreadCoreLawReference -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite -benchmark "-fps=$Fps" "-TreadCount=$Count" "-RingReferenceHz=$PhysicsHz" "-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=$step" '-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubsteps=128' "-abslog=$log" @extra
 $code=$LASTEXITCODE
 $line=(Select-String $log -Pattern 'CORE_LAW compression_m=').Line
 $num=(Select-String $log -Pattern 'CORE_LAW_NUMERICS max_inner_s=').Line
 $dt=[regex]::Match($num,'max_inner_s=([-+0-9.eE]+)')
 $actual=if($dt.Success){[double]::Parse($dt.Groups[1].Value,$culture)}else{0}
 $pass=$code -eq 0 -and $line -match ' PASS$' -and $actual -gt 0 -and $actual -le 1.0/$PhysicsHz+1e-9 -and $num -match 'position_iterations=127$'
 $rows+=[pscustomobject]@{mode=$mode;physics_hz=$PhysicsHz;fps=$Fps;max_inner_s=$actual;pass=[bool]$pass;result=$line;numerics=$num;balance=(Select-String $log -Pattern 'CORE_LOAD_BALANCE').Line;log=$log}
 $line|Write-Output
 ConvertTo-Json -InputObject $rows -Depth 5|Set-Content "$run\results.json" -Encoding UTF8
}
Write-Output "Constitutive report: $run"
if(@($rows|Where-Object {-not $_.pass}).Count){throw 'Native core constitutive criteria failed.'}
Write-Output "CONSTITUTIVE_SUITE PASS $($rows.Count) cases"
