# Single user-facing entry point. Double-click Start.bat for the current project.
param(
    [string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',
    [ValidateSet('integrated','payload','hydraulic','tyre','terrain','stability','drive','lift','rollover')]
    [string]$Mode='integrated',
    [ValidateSet('flat','ramp','step','halfstep','split')][string]$Terrain,
    [switch]$Game,
    [switch]$Website,
    [ValidateRange(1024,65535)][int]$WebPort=4174,
    [switch]$NoWebsite,
    [ValidateSet('balanced','original')][string]$Rendering='balanced',
    [switch]$ParallelPhysics,
    [switch]$NoCameraViews,
    [ValidateSet('reduced','detailed')][string]$MecanumModel='reduced',
    [switch]$LegacySwerve,
    [switch]$DeformableTread,
    [switch]$CoupledTread,
    [ValidateSet(0,16,32,48,64)][int]$TreadSegments=0,
    [ValidateRange(0.1,1.0)][double]$PlaybackRate=0.5,
    [ValidateSet(0,480,960,1920)][int]$PhysicsHz=0,
    [switch]$DryRun
)
$ErrorActionPreference='Stop'
if($Website) {
    & (Join-Path $PSScriptRoot 'WebDashboard.ps1') -Port $WebPort -AutoPort:(-not $PSBoundParameters.ContainsKey('WebPort')) -OpenBrowser -DryRun:$DryRun
    return
}
if($CoupledTread -and -not $PSBoundParameters.ContainsKey('PlaybackRate')) {$PlaybackRate=.1}
if($CoupledTread -and -not $TreadSegments) {$TreadSegments=64}
$launch=@{EngineRoot=$EngineRoot;Editor=(-not $Game);DryRun=[bool]$DryRun;DeformableTread=[bool]$DeformableTread;CoupledTread=[bool]$CoupledTread;TreadSegments=$TreadSegments;PlaybackRate=$PlaybackRate;PhysicsHz=$PhysicsHz}
$launch.LegacySwerve=[bool]($LegacySwerve -or $Mode -ne 'integrated' -or $CoupledTread -or $DeformableTread)
$launch.Mecanum=[bool]($Mode -eq 'integrated' -and -not $launch.LegacySwerve)
$launch.BalancedRendering=[bool]($Mode -eq 'integrated' -and $Rendering -eq 'balanced')
$launch.CameraViews=[bool]($launch.Mecanum -and -not $NoCameraViews)
$launch.ReducedMecanum=[bool]($launch.Mecanum -and $MecanumModel -eq 'reduced')
$launch.SerialPhysics=[bool]($launch.Mecanum -and -not $ParallelPhysics)
switch($Mode) {
    'integrated' { $launch.Stability=$true; $launch.Payload=$true; $launch.Hydraulics=$true; $launch.Terrain=if($launch.Mecanum){'flat'}else{'ramp'} }
    'payload'    { $launch.Stability=$true; $launch.Payload=$true }
    'hydraulic'  { $launch.Stability=$true; $launch.Payload=$true; $launch.Hydraulics=$true }
    'tyre'       { $launch.Stability=$true; $launch.Tyres=$true }
    'terrain'    { $launch.Stability=$true; $launch.Terrain='ramp' }
    'stability'  { $launch.Stability=$true }
    'drive'      { $launch.Drive=$true }
    'lift'       { }
    'rollover'   { $launch.Stability=$true; $launch.Editor=$false; $launch.Scenario='rollover' }
}
if($PSBoundParameters.ContainsKey('Terrain')) {
    if(-not $launch.Stability) { throw '-Terrain requires a 3D stability/payload mode.' }
    $launch.Terrain=$Terrain
}
if($launch.Stability -and -not $NoWebsite) {
    $dashboard=& (Join-Path $PSScriptRoot 'WebDashboard.ps1') -Port $WebPort -AutoPort:(-not $PSBoundParameters.ContainsKey('WebPort')) -OpenBrowser -DryRun:$DryRun -PassThru
    $launch.WebTelemetryUrl=$dashboard.TelemetryUrl
}
& (Join-Path $PSScriptRoot 'Run.ps1') @launch
