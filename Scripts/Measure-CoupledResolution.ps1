param([Parameter(Mandatory=$true)][string]$NativeReport,[Parameter(Mandatory=$true)][string]$FineOracle,[switch]$RequirePass)
$ErrorActionPreference='Stop'
$nativeFile=Get-Item -LiteralPath $NativeReport
$manifest=Get-Content -LiteralPath (Join-Path $nativeFile.DirectoryName 'manifest.json') -Raw|ConvertFrom-Json
$fineManifest=Get-Content -LiteralPath ($FineOracle+'.manifest.json') -Raw|ConvertFrom-Json
if($manifest.model -ne 'cohesive-ring-v1' -or $fineManifest.model -ne $manifest.model -or $manifest.probe){throw 'Use unloaded cohesive-ring-v1 static reports.'}
if($manifest.oracle_sha256 -ne $fineManifest.oracle_sha256){throw 'Energy oracle source hashes differ.'}
foreach($config in $manifest.configs){$name=Split-Path $config.Path -Leaf;$other=@($fineManifest.configs|Where-Object {(Split-Path $_.Path -Leaf) -eq $name});if($other.Count -ne 1 -or $other[0].Hash -ne $config.Hash){throw "Reference config differs: $name"}}
$decoded=Get-Content -LiteralPath $nativeFile.FullName -Raw|ConvertFrom-Json
$rows=@($decoded)
$fine=@(Get-Content -LiteralPath $FineOracle|Where-Object {$_}|ForEach-Object {$_|ConvertFrom-Json})
$points=@();$complete=$rows.Count -eq 4
foreach($height in @(.14,.145)){foreach($phase in @(0.,.5)){
    $n=@($rows|Where-Object {$_.count -eq 64 -and $_.height_m -eq $height -and $_.phase_fraction -eq $phase})
    $f=@($fine|Where-Object {$_.count -eq 128 -and $_.height_m -eq $height -and $_.phase_fraction -eq $phase})
    if($n.Count -ne 1 -or $f.Count -ne 1){$complete=$false;continue}
    $native=$n[0];$reference=$f[0];$same=$native.oracle
    # Preserve measured errors for failed native cases instead of replacing them with a sentinel.
    $valid=$same.success -and $reference.success -and $native.normal_n -gt 0 -and $same.normal_n -gt 0 -and $reference.normal_n -gt 0
    $solverError=if($valid){[math]::Abs($native.normal_n/$same.normal_n-1)}else{1e30}
    $spaceError=if($valid){[math]::Abs($same.normal_n-$reference.normal_n)/[math]::Max($same.normal_n,$reference.normal_n)}else{1e30}
    $combinedError=if($valid){[math]::Abs($native.normal_n/$reference.normal_n-1)}else{1e30}
    $points+=[pscustomobject]@{height_m=$height;phase_fraction=$phase;native_pass=[bool]$native.pass;normal_n=$native.normal_n;energy64_n=$same.normal_n;energy128_n=$reference.normal_n;solver_error=$solverError;space_refinement_error=$spaceError;combined_error=$combinedError;pass=[bool]($valid -and $native.pass -and $solverError -le .03 -and $spaceError -le .05 -and $combinedError -le .05)}
}}
$rotations=@()
foreach($height in @(.14,.145)){
    $set=@($points|Where-Object {$_.height_m -eq $height})
    if($set.Count -ne 2){$complete=$false;continue}
    $m=$set|Measure-Object normal_n -Minimum -Maximum
    $spread=if($m.Maximum -gt 0){($m.Maximum-$m.Minimum)/$m.Maximum}else{1e30}
    $rotations+=[pscustomobject]@{height_m=$height;relative_spread=$spread;pass=[bool]($spread -le .05)}
}
$pass=$complete -and @($points|Where-Object {-not $_.pass}).Count -eq 0 -and @($rotations|Where-Object {-not $_.pass}).Count -eq 0
$out=Join-Path $nativeFile.DirectoryName 'resolution.json'
@{model='cohesive-ring-v1';scope='Fixed hub, flat frictionless plane, 5/10 mm indentation, two phases; not real-rubber calibration or full operating-envelope qualification.';native_report=$nativeFile.FullName;native_sha256=(Get-FileHash $nativeFile.FullName).Hash;fine_reference=$FineOracle;fine_sha256=(Get-FileHash $FineOracle).Hash;complete=[bool]$complete;pass=[bool]$pass;points=$points;rotations=$rotations}|ConvertTo-Json -Depth 7|Set-Content -LiteralPath $out -Encoding UTF8
$points|Format-Table|Out-String|Write-Output
$rotations|Format-Table|Out-String|Write-Output
Write-Output "COUPLED_RESOLUTION pass=$pass complete=$complete report=$out"
if($RequirePass -and -not $pass){throw 'Coupled fixed-hub resolution criteria failed.'}
