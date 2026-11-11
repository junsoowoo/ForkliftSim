param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',[ValidateSet(30,60,120)][int]$Fps=60)
$ErrorActionPreference='Stop'
$cases=@('cargo_hold','cargo_entry','cargo_unload','cargo_accel','cargo_accel_low','cargo_brake','cargo_brake_low','cargo_corner','cargo_corner_low','cargo_tip','cargo_drop')
& "$PSScriptRoot\Test-Payload.ps1" -EngineRoot $EngineRoot -Fps $Fps -Scenarios $cases
$projectRoot=Split-Path -Parent $PSScriptRoot
$measurements=@{}
foreach ($case in $cases) {
    $log=Join-Path $projectRoot "Saved\Logs\Payload-$case-$Fps.log"
    $line=(Select-String -LiteralPath $log -Pattern 'CARGO_CHECK .* PASS').Line
    if (-not $line) { throw "Missing cargo success marker: $case" }
    $values=@{}
    foreach ($field in @('max_shift','max_friction_n','max_mu_ratio')) {
        if ($line -notmatch "$field=([-0-9.]+)") { throw "Missing $field for $case" }
        $values[$field]=[double]::Parse($Matches[1],[Globalization.CultureInfo]::InvariantCulture)
    }
    $measurements[$case]=$values
    $line
}
foreach ($maneuver in @('accel','brake','corner')) {
    $normal=$measurements["cargo_$maneuver"]
    $low=$measurements["cargo_${maneuver}_low"]
    if ($low.max_shift -lt $normal.max_shift + .05 -or $normal.max_friction_n -lt $low.max_friction_n) {
        throw "Cargo friction A/B comparison failed: $maneuver"
    }
}
"CARGO_FRICTION_COMPARISON PASS fps=$Fps"
