param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.6', [switch]$Editor, [switch]$Drive, [switch]$Stability, [switch]$Payload,
    [ValidateSet('flat','ramp','step','halfstep','split')][string]$Terrain='flat', [switch]$IdealActuators, [switch]$Tyres, [switch]$RigidTyres, [switch]$Hydraulics, [switch]$LegacyLift,
    [ValidateSet('','settle','straight','zero_grip','brake','fork','steering','low_grip','low_tyre','rollover','powered_turn','powered_turn_low','powered_turn_left','powered_turn_start','friction_live','friction_keys','swerve_settle','swerve_straight','swerve_reverse','swerve_strafe','swerve_strafe_left','swerve_spin','swerve_spin_left','swerve_combined','swerve_transition','swerve_zero_grip','swerve_low_grip','swerve_brake','swerve_fork','swerve_friction_keys','swerve_rollover','swerve_load','swerve_coast_none','swerve_coast_rolling','swerve_coast_bearing','swerve_coast_both','swerve_coast_high','swerve_coast_reverse')][string]$Scenario = '',
    [ValidateRange(0,6)][int]$CampusVehicles=0,[ValidateSet('','section_windows','whole_job')][string]$CampusTraffic='',[string]$CampusScenario='',[ValidateSet('campus','lab','jx2000a')][string]$Map='campus',[switch]$DryRun,[switch]$BalancedRendering,[switch]$SerialPhysics,[switch]$ReducedMecanum,[switch]$CameraViews,[string]$WebTelemetryUrl='',[switch]$Mecanum,[switch]$LegacySwerve,[switch]$DeformableTread,[switch]$CoupledTread,[ValidateSet(0,16,32,48,64)][int]$TreadSegments=0,[switch]$ExplicitPlayback,[ValidateScript({($_ -ge .1 -and $_ -le 1) -or $_ -eq 2 -or $_ -eq 4})][double]$PlaybackRate=0.5,[ValidateSet(0,480,960,1920)][int]$PhysicsHz=0,[ValidateSet(0,16,32,64)][int]$SolverIterations=0)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
if($CampusScenario) {
    if($Map -ne 'campus' -or -not $Mecanum) {throw '-CampusScenario requires a Mecanum campus run.'}
    $experimentPath=if([IO.Path]::IsPathRooted($CampusScenario)){$CampusScenario}else{Join-Path $projectRoot $CampusScenario}
    $experimentPath=(Resolve-Path -LiteralPath $experimentPath -ErrorAction Stop).Path
    if(-not(Test-Path -LiteralPath $experimentPath -PathType Leaf) -or $experimentPath -match '["\r\n]'){throw 'Invalid campus experiment file.'}
    Get-Content -LiteralPath $experimentPath -Raw | ConvertFrom-Json | Out-Null
}
if($Mecanum -and -not $PhysicsHz) {$PhysicsHz=if($ReducedMecanum){960}else{1920}}
$arguments = @("`"$projectRoot\ForkliftSim.uproject`"")
$campusMap=Join-Path $projectRoot 'Content/Maps/WarehouseCampus.umap'
$warehouseMap=Join-Path $projectRoot 'Content/Maps/WarehouseLab.umap'
$warehousePallet=Join-Path $projectRoot 'Content/Warehouse/Assets/Props/WM00217_Pallet/SM_WM00217-Pallet_Standard.uasset'
if($Map -eq 'jx2000a') {
    if($Stability -or $Drive -or $Mecanum -or $Payload -or $Hydraulics -or $Scenario) {throw 'JX exterior review cannot run the existing physics plant.'}
    if(-not(Test-Path -LiteralPath (Join-Path $projectRoot 'Content/Maps/JX2000AReview.umap'))) {throw 'Import the JX-2000A exterior before opening its review map.'}
    $arguments+='/Game/Maps/JX2000AReview'
} elseif($Mecanum -and $Map -eq 'campus' -and $Terrain -eq 'flat' -and -not $Scenario -and (Test-Path -LiteralPath $campusMap) -and (Test-Path -LiteralPath $warehousePallet)) {
    $arguments+='/Game/Maps/WarehouseCampus'
} elseif($Mecanum -and $Terrain -eq 'flat' -and -not $Scenario -and (Test-Path -LiteralPath $warehouseMap) -and (Test-Path -LiteralPath $warehousePallet)) {
    $arguments+='/Game/Maps/WarehouseLab'
} else {$arguments+='/Engine/Maps/Entry'}
if($CampusScenario){$arguments+='-CampusScenario="'+$experimentPath+'"'}
if($CampusVehicles){if($CampusVehicles -lt 3 -or $Map -ne 'campus' -or -not $Mecanum){throw 'Campus fleet count requires Mecanum campus and 3 to 6 vehicles.'};$arguments+="-CampusVehicles=$CampusVehicles"}
if($CampusTraffic){if($Map -ne 'campus' -or -not $Mecanum){throw 'Campus traffic policy requires Mecanum campus.'};$arguments+="-CampusTraffic=$CampusTraffic"}
if($Mecanum){$arguments+='-WarehouseLab'}
if($ReducedMecanum){$arguments+='-ReducedMecanum';if($PhysicsHz){$arguments+="-ReducedPhysicsHz=$PhysicsHz"}}
if($CameraViews){$arguments+='-CameraViews'}
if($SerialPhysics){$arguments+='-SingleThreadedPhysics'}
if($SolverIterations){$arguments+="-MecanumPositionIterations=$SolverIterations";$arguments+="-MecanumVelocityIterations=$SolverIterations"}
if($BalancedRendering) {
    if($Editor){$arguments+='-dx11'}
    # GPU presentation only. Collision meshes, physics resolution and telemetry
    # remain unchanged. Avoid the expensive default TSR history on a laptop GPU.
    $arguments+='-ExecCmds="r.AntiAliasingMethod 1,r.ScreenPercentage 75,r.Editor.Viewport.ScreenPercentageMode.RealTime 0,r.Editor.Viewport.ScreenPercentage 75,sg.PostProcessQuality 1,sg.ShadowQuality 1,r.Shadow.MaxResolution 1024"'
}

if($WebTelemetryUrl) {
    if($WebTelemetryUrl -notmatch '^http://127\.0\.0\.1:[0-9]{4,5}/api/telemetry$') { throw 'Web telemetry requires a loopback URL.' }
    $arguments+="-WebTelemetry=$WebTelemetryUrl"
}
if (-not $Editor) { $arguments += @('-game', '-windowed', '-ResX=1280', '-ResY=800', '-dx11') }
if ($Drive) { $arguments += '-DriveLab' }
if ($Stability) { $arguments += '-StabilityLab' }
if($Mecanum){if(-not $Stability -or $LegacySwerve -or $CoupledTread -or $DeformableTread){throw 'Mecanum requires 3D mode and is incompatible with legacy tyre fixtures.'};$arguments+='-Mecanum'}
if($LegacySwerve){$arguments+='-LegacySwerve'}
if($Stability -and -not $RigidTyres) {
    $localTread=$DeformableTread -or $CoupledTread -or (Get-Content -LiteralPath (Join-Path $projectRoot 'Config\Tyres.json') -Raw | ConvertFrom-Json).tread_enabled
    if($DeformableTread) {$arguments+='-DeformableTread'}
    if($CoupledTread) {
        $arguments+='-CoupledTread'
        if(-not $PhysicsHz){$PhysicsHz=960}
        if(-not $TreadSegments){$TreadSegments=64}
        if(-not $PSBoundParameters.ContainsKey('PlaybackRate')){$PlaybackRate=.1}
    }
    if($TreadSegments) {
        if(-not $localTread) {throw '-TreadSegments requires -DeformableTread or tread_enabled=true.'}
        if($TreadSegments -gt 32 -and -not $CoupledTread){throw '48/64 tread segments require -CoupledTread.'}
        $arguments+="-TreadSegments=$TreadSegments"
    }
    if($localTread -or $ExplicitPlayback) {$arguments+='-PlaybackRate='+$PlaybackRate.ToString('G17',[Globalization.CultureInfo]::InvariantCulture)}
}
elseif($TreadSegments) {throw '-TreadSegments requires a 3D stability mode with compliant tyres.'}
if($CoupledTread -and (-not $Stability -or $RigidTyres)){throw '-CoupledTread requires a 3D stability mode with compliant tyres.'}
if($PhysicsHz) {
    if(-not $Stability){throw '-PhysicsHz requires a 3D stability mode.'}
    $step=(1.0/$PhysicsHz).ToString('G17',[Globalization.CultureInfo]::InvariantCulture)
    $arguments+="-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=$step"
    $arguments+='-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubsteps=128'
}
if ($Terrain -ne 'flat') { if (-not $Stability) { throw 'Terrain requires -Stability.' }; $arguments += "-Terrain=$Terrain" }
if ($IdealActuators) { $arguments += '-IdealActuators' }
if ($Tyres) { if (-not $Stability) { throw 'Tyres requires -Stability.' }; $arguments += '-TyreLab' }
if ($RigidTyres) { $arguments += '-RigidTyres' }
if ($Hydraulics) { if (-not $Stability) { throw 'Hydraulics requires -Stability.' }; $arguments += '-HydraulicLab' }
if ($LegacyLift) { $arguments += '-LegacyLift' }
if ($Payload) { if (-not $Stability) { throw 'Payload requires -Stability.' }; $arguments += '-PayloadLab' }
if ($Scenario) {
    if (-not $Stability -or $Editor) { throw 'Scenario playback requires -Stability without -Editor.' }
    $arguments += "-StabilityTest=$Scenario"
}
$executable=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor.exe'
if(-not (Test-Path -LiteralPath $executable -PathType Leaf)) { throw "Unreal Editor not found: $executable. Use -EngineRoot to select the engine installation." }
if($DryRun) {
    [pscustomobject]@{Executable=$executable;Arguments=$arguments;WorkingDirectory=$projectRoot}
    return
}
# The visible application is intentional: the user is opening an interactive editor/simulator.
Start-Process -FilePath $executable -ArgumentList $arguments -WorkingDirectory $projectRoot
