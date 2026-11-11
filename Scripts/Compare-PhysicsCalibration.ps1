# Re-evaluate saved results without starting Unreal again. Windows PowerShell 5.1.
param([Parameter(Mandatory=$true)][string]$ReportDirectory)
$ErrorActionPreference='Stop'
$reportRoot=(Resolve-Path -LiteralPath $ReportDirectory).Path
# PS 5.1 does not enumerate a JSON array emitted by ConvertFrom-Json inside @().
# Decode first, then enumerate explicitly; otherwise a mixed pass/fail array
# becomes one truthy object and can accidentally produce a false PASS.
$decoded=Get-Content -LiteralPath "$reportRoot\results.json" -Raw | ConvertFrom-Json
$rows=@($decoded | ForEach-Object {$_})
$limits=Get-Content -LiteralPath "$reportRoot\PhysicsReference.json" -Raw | ConvertFrom-Json
if(-not $rows.Count) { throw 'Empty calibration results' }
foreach($row in $rows) {
    if($row.pass -isnot [bool] -or $row.behaviour_pass -isnot [bool] -or -not $row.case -or -not $row.metrics -or -not $row.checks) { throw 'Invalid calibration result schema' }
}
$comparisons=@()
foreach($case in @($rows.case | Select-Object -Unique)) {
    $group=@($rows | Where-Object {$_.case -eq $case})
    if($group.Count -lt 2) { continue }
    $key=switch($case) {'load' {'load_error'} 'coast_none' {'speed_error'} 'tyre_settle' {'compression_mm'} 'hydraulic_loaded' {'lift_mps'} default { throw "Unknown case $case" }}
    $valid=$true
    $numbers=@($group | ForEach-Object {
        $metric=$_.metrics.PSObject.Properties[$key]
        if(-not $metric -or $null -eq $metric.Value) { $valid=$false; 0.0 }
        else { $value=[double]$metric.Value; if([double]::IsNaN($value) -or [double]::IsInfinity($value)) { $valid=$false }; $value }
    })
    $range=($numbers | Measure-Object -Maximum).Maximum-($numbers | Measure-Object -Minimum).Minimum
    $den=if($case -in @('load','coast_none')) {1} else {($numbers | Measure-Object -Average).Average}
    $comparisonError=$range/[Math]::Max($den,1e-9)
    $comparisons+=[pscustomobject]@{case=$case;metric=$key;range_error=$comparisonError;pass=$valid -and $comparisonError -lt $limits.tolerances.timestep_relative_or_fraction}
}
ConvertTo-Json -InputObject $comparisons -Depth 5 | Set-Content -LiteralPath "$reportRoot\timestep-comparison.json" -Encoding UTF8
Write-Output "Calibration report: $reportRoot"
if(@($rows | Where-Object {-not $_.pass -or -not $_.behaviour_pass -or @($_.checks.PSObject.Properties | Where-Object {$_.Value -ne $true}).Count}).Count -or @($comparisons | Where-Object {-not $_.pass}).Count) {
    throw 'PHYSICS_CALIBRATION FAIL: numerical/analytic acceptance was NOT achieved. See results; behaviour PASS alone is insufficient.'
}
Write-Output 'PHYSICS_CALIBRATION PASS (virtual reference, NOT real-machine calibration)'
