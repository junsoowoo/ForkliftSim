param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',
    [ValidateSet(30,60,120)][int[]]$Fps=@(30,60), [ValidateSet(240,480,960)][int[]]$PhysicsHz=@(240,480),
    [ValidateSet('load','coast_none','tyre_settle','hydraulic_loaded')][string[]]$Cases=@('load','coast_none','tyre_settle','hydraulic_loaded'),
    [switch]$PassiveSteer, [ValidateRange(32,512)][int]$TyreSegments=256, [switch]$LegacyTyreContacts)
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$run=Join-Path $projectRoot ('Saved\Validation\Calibration-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
New-Item -ItemType Directory -Path $run -Force | Out-Null
$culture=[Globalization.CultureInfo]::InvariantCulture
$limits=Get-Content -LiteralPath "$projectRoot\Config\PhysicsReference.json" -Raw | ConvertFrom-Json
Copy-Item -LiteralPath "$projectRoot\Config\PhysicsReference.json" -Destination $run
$files=@(Get-ChildItem -LiteralPath "$projectRoot\Source\ForkliftSim" -File; Get-Item "$projectRoot\Config\DefaultEngine.ini"; Get-Item "$projectRoot\Binaries\Win64\UnrealEditor-ForkliftSim.dll"; Get-Item "$PSScriptRoot\Compare-PhysicsCalibration.ps1"; Get-Item $PSCommandPath)
$hashes=@($files | Get-FileHash -Algorithm SHA256 | Select-Object Path,Hash)
@{reference=$limits.id;utc=[DateTime]::UtcNow.ToString('o');engine=$EngineRoot;fps=$Fps;physics_hz=$PhysicsHz;passive_steer=[bool]$PassiveSteer;tyre_segments=$TyreSegments;legacy_contacts=[bool]$LegacyTyreContacts;source_sha256=$hashes} | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath "$run\manifest.json" -Encoding UTF8
function Number([string]$line,[string]$key) {
    if($line -notmatch "(?:^|\s)$key=([-+0-9.eE]+)") { throw "Missing metric $key" }
    return [double]::Parse($Matches[1],$culture)
}
$rows=@()
foreach($hz in $PhysicsHz) { foreach($frameRate in $Fps) { foreach($case in $Cases) {
    $scenario="swerve_$case"
    $log=Join-Path $run "$case-$frameRate-${hz}hz.log"
    $dt=(1.0/$hz).ToString('G17',$culture)
    $extra=@(); if($PassiveSteer) { $extra+='-PassiveSteer' }
    if($LegacyTyreContacts) { $extra+='-LegacyTyreContacts' }
    Write-Output "RUN calibration $case fps=$frameRate physics_hz=$hz"
    & $editor "$projectRoot\ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -ReferencePhysics -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite -benchmark "-fps=$frameRate" "-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=$dt" "-StabilityTest=$scenario" "-TyreSegments=$TyreSegments" "-abslog=$log" @extra
    $exitCode=$LASTEXITCODE
    $text=Get-Content -LiteralPath $log -Raw
    $legacyPass=$exitCode -eq 0 -and $text.Contains("STABILITY_TEST $scenario PASS")
    $metrics=[ordered]@{}; $checks=[ordered]@{}
    try {
        $step=(Select-String -LiteralPath $log -Pattern 'NUMERICAL_STEP').Line
        $metrics.max_inner_s=Number $step 'max_inner_s'
        $metrics.max_game_s=Number $step 'max_game_s'
        $checks.physics_step=$metrics.max_inner_s -gt 0 -and $metrics.max_inner_s -le 1.0/$hz+1e-7
        $checks.game_step=[Math]::Abs($metrics.max_game_s-1.0/$frameRate) -lt 1e-6
        switch($case) {
            'load' {
                $line=(Select-String -LiteralPath $log -Pattern 'LOAD_CHECK').Line
                $metrics.load_error=[Math]::Abs((Number $line 'mean_n')/(Number $line 'weight_n')-1)
                $metrics.front_error=[Math]::Abs((Number $line 'front_fraction')-(Number $line 'expected_front'))
                $checks.load=$metrics.load_error -lt $limits.tolerances.static_weight_relative
                $checks.moment=$metrics.front_error -lt $limits.tolerances.static_load_fraction_absolute
            }
            'coast_none' {
                $line=(Select-String -LiteralPath $log -Pattern 'NUMERICAL_COAST').Line
                $v=Number $line 'start_mps'; $metrics.speed_error=[Math]::Abs((Number $line 'final_mps')/$v-1)
                $metrics.energy_gain_j=Number $line 'peak_gain_j'
                # Normalize by translational KE only; never by the arbitrary gravitational datum.
                $metrics.energy_gain_fraction=$metrics.energy_gain_j/(.5*2170*$v*$v)
                $checks.speed=$metrics.speed_error -lt $limits.tolerances.coast_speed_relative
                $checks.energy=$metrics.energy_gain_fraction -lt $limits.tolerances.coast_energy_gain_relative
            }
            'tyre_settle' {
                $line=(Select-String -LiteralPath $log -Pattern 'TYRE_CHECK').Line
                $metrics.force_error=Number $line 'force_error'
                $metrics.compression_mm=Number $line 'mean_compression_mm'
                $checks.spring_force=$metrics.force_error -lt $limits.tolerances.tyre_force_relative
            }
            'hydraulic_loaded' {
                $line=(Select-String -LiteralPath $log -Pattern 'HYDRAULIC_CHECK').Line
                $metrics.lift_mps=Number $line 'mean_lift_mps'
                # P*eta=(m*g+seal+C*v)*v, valid in the steady, power-limited mid stroke.
                $force=(150+425)*9.81+80
                $expected=2*1200*.8/($force+[Math]::Sqrt($force*$force+4*400*1200*.8))
                $metrics.expected_lift_mps=$expected
                $metrics.speed_error=[Math]::Abs($metrics.lift_mps/$expected-1)
                $checks.power_balance=$metrics.speed_error -lt $limits.tolerances.hydraulic_speed_relative
            }
        }
    } catch { $checks.metrics=$false; $metrics.error=$_.Exception.Message }
    $pass=$legacyPass -and -not ($checks.Values -contains $false)
    $rows+=[pscustomobject]@{case=$case;fps=$frameRate;physics_hz=$hz;pass=$pass;behaviour_pass=$legacyPass;checks=$checks;metrics=$metrics;log=$log}
    Write-Output "$case $frameRate/$hz pass=$pass $($metrics | ConvertTo-Json -Compress)"
    # Persist even if a later process fails or the user interrupts the run.
    ConvertTo-Json -InputObject $rows -Depth 8 | Set-Content -LiteralPath "$run\results.json" -Encoding UTF8
} } }
& "$PSScriptRoot\Compare-PhysicsCalibration.ps1" -ReportDirectory $run
