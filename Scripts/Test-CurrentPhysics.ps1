# Reproduce the current compact/reduced qualification. This is a test harness;
# the user's interactive launcher remains root Start.bat.
param([switch]$Quick,[switch]$UncappedBenchmark,[string]$WebTelemetryUrl='')
$ErrorActionPreference='Stop'
$common=@{ReducedMecanum=$true;SerialPhysics=$true;ReferenceModel=$true;UncappedBenchmark=[bool]$UncappedBenchmark;WebTelemetryUrl=$WebTelemetryUrl}
$cases=if($Quick){@('settle','carry','ramp_brake')}else{@('settle','idle','coast','reaction','brake','friction_keys','carry','eccentric_lift','shallow_lift','high_carry','step','halfstep','split_carry','ramp_brake','cargo_slide','cargo_drop','lower')}
& "$PSScriptRoot/Test-Mecanum.ps1" @common -Cases $cases
if(-not $Quick){
 & "$PSScriptRoot/Test-Mecanum.ps1" @common -Cases rack_place_low,rack_place_high,rack_pick_low,rack_pick_high
 & "$PSScriptRoot/Test-Mecanum.ps1" @common -Cases coast,carry,halfstep,ramp_brake -Fps 30
 & "$PSScriptRoot/Test-Mecanum.ps1" @common -Cases coast,carry,halfstep,ramp_brake -PhysicsHz 1920
 foreach($coefficient in @(.007,.009,.015)){& "$PSScriptRoot/Test-Mecanum.ps1" @common -Cases coast -ReferenceRollingCoefficient $coefficient}
}
