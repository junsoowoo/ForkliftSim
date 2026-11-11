param([Parameter(Mandatory=$true)][string]$ReferenceReport,[ValidateRange(.001,.2)][double]$Tolerance=.05,[switch]$RequireConvergence)
$ErrorActionPreference='Stop'
$report=Get-Item -LiteralPath $ReferenceReport
$manifest=Get-Content -LiteralPath (Join-Path $report.DirectoryName 'manifest.json') -Raw | ConvertFrom-Json
if($manifest.tread_reference_schema -ne 2){throw 'Use a schema 2 report from the current Test-TreadReference.ps1; old reports have no rotation/continuous-limit metrics.'}
# Windows PowerShell 5.1 emits a JSON array as one pipeline object.
$decoded=Get-Content -LiteralPath $report.FullName -Raw | ConvertFrom-Json
$rows=@($decoded)
if(-not $rows.Count){throw 'Reference report has no measurements.'}
foreach($row in $rows){foreach($key in @('fps','count','height_m','phase_fraction','pass','normal_n','continuum_n')){if($row.PSObject.Properties.Name -notcontains $key){throw "Missing reference field: $key"}}}
$studies=@()
foreach($group in ($rows | Group-Object fps,height_m)) {
    $all=@($group.Group)
    # Array.Count is the row count, not each row's 'count' measurement.
    $counts=@($all | ForEach-Object {$_.count} | Sort-Object -Unique)
    $phases=@($all.phase_fraction | Sort-Object -Unique)
    $complete=$counts.Count -ge 3 -and $counts[0] -eq 16 -and $counts[-1] -ge 64 -and $phases -contains 0 -and $phases -contains .5
    for($i=1;$i -lt $counts.Count;$i++){if($counts[$i] -ne 2*$counts[$i-1]){$complete=$false}}
    $points=@(); $pairs=@(); $referencePass=$true
    foreach($n in $counts) {
        $set=@($all | Where-Object {$_.count -eq $n})
        foreach($x in $set) {
            if(-not $x.pass -or $null -eq $x.normal_n -or $null -eq $x.continuum_n -or $x.normal_n -le 0 -or $x.continuum_n -le 0) {$referencePass=$false}
        }
        if(@($set.phase_fraction | Sort-Object -Unique).Count -ne $phases.Count -or $set.Count -ne $phases.Count){$complete=$false}
        $range=$set | Measure-Object normal_n -Minimum -Maximum
        $bias=($set | ForEach-Object {if($_.continuum_n -gt 0){[math]::Abs($_.normal_n/$_.continuum_n-1)}else{[double]::PositiveInfinity}} | Measure-Object -Maximum).Maximum
        $variation=if($range.Maximum -gt 0){($range.Maximum-$range.Minimum)/$range.Maximum}else{[double]::PositiveInfinity}
        $points+=[pscustomobject]@{count=$n;max_continuum_relative_error=$bias;rotation_relative_spread=$variation;min_normal_n=$range.Minimum;max_normal_n=$range.Maximum}
    }
    for($i=1;$i -lt $counts.Count;$i++) {
        foreach($phase in $phases) {
            $a=@($all | Where-Object {$_.count -eq $counts[$i-1] -and $_.phase_fraction -eq $phase})
            $b=@($all | Where-Object {$_.count -eq $counts[$i] -and $_.phase_fraction -eq $phase})
            if($a.Count -ne 1 -or $b.Count -ne 1){$complete=$false;continue}
            $scale=[math]::Max($a[0].normal_n,$b[0].normal_n)
            $diff=if($scale -gt 0){[math]::Abs($a[0].normal_n-$b[0].normal_n)/$scale}else{[double]::PositiveInfinity}
            $pairs+=[pscustomobject]@{coarse=$counts[$i-1];fine=$counts[$i];phase_fraction=$phase;relative_change=$diff}
        }
    }
    $finest=$counts[-1]; $fine=$points[-1]; $lastPairs=@($pairs | Where-Object {$_.fine -eq $finest})
    $lastError=($lastPairs | Measure-Object relative_change -Maximum).Maximum
    $converged=$complete -and $referencePass -and $lastPairs.Count -eq $phases.Count -and $lastError -le $Tolerance -and $fine.rotation_relative_spread -le $Tolerance -and $fine.max_continuum_relative_error -le $Tolerance
    $studies+=[pscustomobject]@{fps=$all[0].fps;height_m=$all[0].height_m;complete=[bool]$complete;solver_reference_pass=[bool]$referencePass;convergence_pass=[bool]$converged;counts=$points;refinement_pairs=$pairs}
}
$pass=@($studies | Where-Object {-not $_.convergence_pass}).Count -eq 0
$out=Join-Path $report.DirectoryName 'resolution.json'
@{source_report=$report.FullName;source_sha256=(Get-FileHash -LiteralPath $report.FullName).Hash;dll_sha256=$manifest.dll_sha256;tolerance=$Tolerance;convergence_pass=$pass;interpretation='Continuous radial foundation is a mathematical limit, not measured rubber. Individual fixture PASS does not mean resolution convergence.';studies=$studies} | ConvertTo-Json -Depth 9 | Set-Content -LiteralPath $out -Encoding UTF8
Write-Output "Resolution study: $out"
foreach($s in $studies){Write-Output "fps=$($s.fps) height=$($s.height_m) solver_reference_pass=$($s.solver_reference_pass) convergence_pass=$($s.convergence_pass)"; $s.counts | Format-Table | Out-String | Write-Output}
if($RequireConvergence -and -not $pass){throw 'Tread resolution convergence FAILED. Inspect resolution.json; solver reference PASS is insufficient.'}
if(-not $pass){Write-Output 'CONVERGENCE_FAIL: diagnostic report collected; current model is not resolution-qualified.'}
