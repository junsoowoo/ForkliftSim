param()
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$tempRoot=Join-Path $projectRoot ('Saved\Validation\ReportSelfTest-'+[guid]::NewGuid().ToString('N'))
$shellPath=Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe'
$count=0
foreach($case in @('all_pass','mixed_fail','behaviour_fail','comparison_fail','missing_metric','empty')) {
    $dir=Join-Path $tempRoot $case
    New-Item -ItemType Directory -Path $dir -Force | Out-Null
    Copy-Item -LiteralPath "$projectRoot\Config\PhysicsReference.json" -Destination $dir
    $rows=@(
        [pscustomobject]@{case='tyre_settle';fps=30;physics_hz=480;pass=$true;behaviour_pass=$true;checks=@{spring_force=$true};metrics=@{compression_mm=8.0}},
        [pscustomobject]@{case='tyre_settle';fps=60;physics_hz=480;pass=$true;behaviour_pass=$true;checks=@{spring_force=$true};metrics=@{compression_mm=8.01}}
    )
    switch($case) {
        'mixed_fail' {$rows[1].pass=$false; $rows[1].checks.spring_force=$false}
        'behaviour_fail' {$rows[1].behaviour_pass=$false}
        'comparison_fail' {$rows[1].metrics.compression_mm=10}
        'missing_metric' {$rows[1].metrics=@{unrelated=1}}
        'empty' {$rows=@()}
    }
    ConvertTo-Json -InputObject $rows -Depth 8 | Set-Content -LiteralPath "$dir\results.json" -Encoding UTF8
    # Expected failing native processes must not terminate this PS 5.1 harness
    # when redirected stderr becomes NativeCommandError records.
    $ErrorActionPreference='Continue'
    $output=& $shellPath -NoProfile -ExecutionPolicy Bypass -File "$PSScriptRoot\Compare-PhysicsCalibration.ps1" -ReportDirectory $dir 2>&1
    $passed=$LASTEXITCODE -eq 0
    $ErrorActionPreference='Stop'
    if($passed -ne ($case -eq 'all_pass')) { $output; throw "Unexpected report verdict for $case" }
    ++$count; Write-Output "REPORT_SELF_TEST $case PASS (expected process success=$($case -eq 'all_pass'))"
}
Write-Output "REPORT_SELF_TEST PASS $count cases using Windows PowerShell 5.1"
