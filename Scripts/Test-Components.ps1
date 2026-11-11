param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[ValidateSet(30,60)][int]$Fps=60,[string]$WebTelemetryUrl='',
    [ValidateSet('lift','carry','lower','ramp_brake','ramp_ice','plastic','fracture','step','halfstep','tread_soft','tread_stiff','tread_coast')][string[]]$Cases=@('lift','carry','lower','ramp_brake','plastic','fracture','step','halfstep'),[switch]$NativeTyreEdges,
    [ValidateSet('configured','deformable','rigid')][string]$TreadMode='configured',[ValidateSet('balanced','high')][string]$TreadQuality='high',[ValidateSet(0,16,32,48,64)][int]$TreadSegments=0,[switch]$CoupledTread,[ValidateSet(240,480,960,1920)][int]$PhysicsHz=480)
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$run=Join-Path $projectRoot ('Saved\Validation\Components-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
New-Item -ItemType Directory -Path $run -Force | Out-Null
@{fps=$Fps;physics_hz=$PhysicsHz;tread_mode=$TreadMode;tread_quality=$TreadQuality;tread_segments_override=$TreadSegments;coupled_tread=[bool]$CoupledTread;native_tyre_edges=[bool]$NativeTyreEdges;reference='hybrid4-component-reference-v1';dll_sha256=(Get-FileHash "$projectRoot\Binaries\Win64\UnrealEditor-ForkliftSim.dll").Hash;
  configs=@(Get-ChildItem "$projectRoot\Config\*.json" | Get-FileHash | Select-Object Path,Hash);
  source_sha256=@(Get-ChildItem "$projectRoot\Source\ForkliftSim" -File | Get-FileHash | Select-Object Path,Hash)} | ConvertTo-Json -Depth 5 | Set-Content "$run\manifest.json" -Encoding UTF8
$results=@()
foreach($case in $Cases) {
    $scenario="swerve_components_$case"; $log=Join-Path $run "$case-$Fps.log"
    $step=(1.0/$PhysicsHz).ToString('G17',[Globalization.CultureInfo]::InvariantCulture)
    $extra=@("-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=$step","-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubsteps=128"); if($case -in @('ramp_brake','ramp_ice')) {$extra+='-Terrain=ramp'}
    if($case -in @('step','halfstep')) {$extra+="-Terrain=$case"}
    if($NativeTyreEdges) {$extra+='-NativeTyreEdges'}
    if($TreadMode -eq 'deformable') {$extra+='-DeformableTread'}
    if($CoupledTread) {$extra+='-CoupledTread'; if(-not $TreadSegments){$extra+='-TreadSegments=64'}}
    if($TreadMode -eq 'rigid') {$extra+='-RigidTread'}
    if($TreadSegments) {$extra+="-TreadSegments=$TreadSegments"}
    if($TreadQuality -eq 'high') {$extra+='-HighTreadQuality'}
    else {$extra+='-ReducedTreadIterations'}
    if($WebTelemetryUrl) {
        if($WebTelemetryUrl -notmatch '^http://127\.0\.0\.1:[0-9]{4,5}/api/telemetry$') {throw 'Local telemetry URL required'}
        $extra+="-WebTelemetry=$WebTelemetryUrl"
    }
    Write-Output "RUN actual configured components $case fps=$Fps"
    & $editor "$projectRoot\ForkliftSim.uproject" /Engine/Maps/Entry -game -StabilityLab -nullrhi -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite -benchmark "-fps=$Fps" "-StabilityTest=$scenario" "-abslog=$log" @extra
    $code=$LASTEXITCODE
    $behaviour=$code -eq 0 -and (Select-String -LiteralPath $log -Pattern "STABILITY_TEST $scenario PASS" -Quiet)
    $observed=(Select-String -LiteralPath $log -Pattern 'TREAD_TIMESTEP world_s=').Line
    $inner=[regex]::Match($observed,'max_inner_s=([-+0-9.eE]+)')
    $game=[regex]::Match($observed,'max_game_s=([-+0-9.eE]+)')
    $culture=[Globalization.CultureInfo]::InvariantCulture
    $innerS=if($inner.Success){[double]::Parse($inner.Groups[1].Value,$culture)}else{0}
    $gameS=if($game.Success){[double]::Parse($game.Groups[1].Value,$culture)}else{0}
    $stepPass=$innerS -gt 0 -and $innerS -le 1.0/$PhysicsHz+1e-9 -and [math]::Abs($gameS-1.0/$Fps) -lt 1e-6
    $pass=$behaviour -and $stepPass
    $line=(Select-String -LiteralPath $log -Pattern 'COMPONENT_CHECK|CONTACT_CHECK|TREAD_CHECK|ENERGY_AUDIT|TREAD_TIMESTEP').Line; $line | Write-Output
    $results+=[pscustomobject]@{case=$case;fps=$Fps;physics_hz=$PhysicsHz;max_inner_s=$innerS;max_game_s=$gameS;step_pass=[bool]$stepPass;behaviour_pass=[bool]$behaviour;pass=[bool]$pass;result=$line;log=$log}
    ConvertTo-Json -InputObject $results -Depth 5 | Set-Content "$run\results.json" -Encoding UTF8
}
Write-Output "Components report: $run"
if(@($results | Where-Object {-not $_.pass}).Count) {throw 'Component physics failed; all requested cases were collected.'}
Write-Output "COMPONENT_SUITE PASS $($results.Count) cases"
