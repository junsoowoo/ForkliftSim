param([string]$EngineRoot='C:\Program Files\Epic Games\UE_5.6',
 [ValidateSet('mission_low','mission_high','mission_avoid_pause','mission_sensor_recovery','mission_blocked','mission_cancel','mission_watchdog','mission_retry','handling_pick','handling_align','handling_pick_low','handling_pick_high','handling_place_low','handling_place_high','handling_camera_loss','handling_sensor_loss','handling_estop','handling_cancel','handling_obstacle','pallet_vision_aligned','pallet_vision_yaw','pallet_vision_shift','pallet_vision_box','pallet_vision_disabled','pallet_vision_high','pallet_vision_insert','drive_sensor_warehouse','drive_sensor_straight','drive_sensor_strafe','drive_sensor_curve','drive_sensor_slip','drive_sensor_encoder_fault','drive_sensor_imu_fault','drive_sensor_watchdog','vehicle_guard_creep','vehicle_guard_front','vehicle_guard_rear','vehicle_guard_side','vehicle_guard_diagonal','vehicle_guard_fault','vehicle_guard_estop','vehicle_guard_carry','lift_sensor_empty','lift_sensor_loaded','lift_sensor_fault_up','lift_sensor_fault_down','lift_sensor_pressure','lift_sensor_watchdog','joint_sensors','rack_place_low','rack_place_high','rack_pick_low','rack_pick_high','settle','straight','reverse','strafe','strafe_left','diagonal','spin','curve','brake','coast','zero_grip','reaction','lift','carry','step','ramp_brake','eccentric_lift','shallow_lift','high_carry','split_carry','halfstep','cargo_slide','cargo_drop','lower','grip_low','grip_normal','idle','friction_keys')][string[]]$Cases=@('settle','straight','reverse','strafe','strafe_left','diagonal','spin','curve','brake','coast','zero_grip','reaction','lift','carry'),
 [ValidateSet(6,15,30,60)][int]$Fps=60,[ValidateSet(480,960,1920)][int]$PhysicsHz=1920,[string]$WebTelemetryUrl='',[ValidateSet('','all','encoder_fl','encoder_fr','encoder_rl','encoder_rr','fork_height','fork_limits')][string]$JointSensorFault='',[ValidateSet('','all','imu','lift_pressure','fork_load_left','fork_load_right')][string]$DynamicsSensorFault='',[ValidateSet('','all','heel_left','heel_right','tip_left','tip_right','presence_left_near','presence_right_near','presence_left_far','presence_right_far')][string]$ProximitySensorFault='',[switch]$ProximityValidate,[ValidateRange(8,127)][int]$PositionIterations=32,[ValidateRange(8,127)][int]$VelocityIterations=32,[switch]$ReducedMecanum,[switch]$SerialPhysics,[switch]$ReferenceModel,[switch]$WarehouseMap,[switch]$Rendered,[switch]$NoCCD,[switch]$FreshContacts,[switch]$ContactDiagnostics,[switch]$UncappedBenchmark,[switch]$ReducedSmoothContact,[switch]$NativeReducedSupport,[ValidateRange(-1,.05)][double]$ReferenceRollingCoefficient=-1,[ValidateSet('MecanumLinearJoints','MecanumNoGyro','MecanumSphereContact','MecanumDetachedRollers','MecanumBearingPredictor','MecanumExactRollerSpin','MecanumConvexContact','MecanumLegacyIntegration','MecanumFixedStep','MecanumNoRolling','MecanumNoBearing')][string[]]$DiagnosticSwitch=@())
$ErrorActionPreference='Stop'
if($ReducedMecanum -and -not $PSBoundParameters.ContainsKey('PhysicsHz')){$PhysicsHz=960}
$root=Split-Path -Parent $PSScriptRoot
$editor=Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$runId=[guid]::NewGuid().ToString('N')
$run=Join-Path $root ('Saved\Validation\Mecanum-'+(Get-Date -Format 'yyyyMMdd-HHmmss-fff')+'-'+$runId.Substring(0,8))
New-Item -ItemType Directory -Path $run -Force|Out-Null
@{run_id=$runId;test_script_sha256=(Get-FileHash $PSCommandPath).Hash;joint_sensor_fault=$JointSensorFault;dynamics_sensor_fault=$DynamicsSensorFault;proximity_sensor_fault=$ProximitySensorFault;proximity_scene_validation=[bool]$ProximityValidate;model=$(if($ReducedMecanum){'equivalent-directional-contact-v1'}else{'physical-mecanum-rollers-v2-conservative-integrator'});reduced_mecanum=[bool]$ReducedMecanum;single_threaded_physics=[bool]$SerialPhysics;reference_model=[bool]$ReferenceModel;warehouse_map=[bool]$WarehouseMap;rendered=[bool]$Rendered;no_ccd_diagnostic=[bool]$NoCCD;fresh_contacts_diagnostic=[bool]$FreshContacts;contact_diagnostics=[bool]$ContactDiagnostics;uncapped_benchmark=[bool]$UncappedBenchmark;reduced_smooth_contact=[bool]($ReducedMecanum -and -not $NativeReducedSupport);smooth_contact_switch=[bool]$ReducedSmoothContact;native_support_diagnostic=[bool]$NativeReducedSupport;reference_rolling_coefficient=$ReferenceRollingCoefficient;fps=$Fps;physics_hz=$PhysicsHz;diagnostic_switches=$DiagnosticSwitch;position_iterations=$PositionIterations;velocity_iterations=$VelocityIterations;dll_sha256=(Get-FileHash "$root\Binaries\Win64\UnrealEditor-ForkliftSim.dll").Hash;
 configs=@(Get-ChildItem "$root\Config" -File | Where-Object {$_.Extension -in @(".json",".ini")} | Get-FileHash | Select-Object Path,Hash);
 sources=@(Get-ChildItem "$root\Source\ForkliftSim" -File|Get-FileHash|Select-Object Path,Hash)}|ConvertTo-Json -Depth 5|Set-Content "$run\manifest.json" -Encoding UTF8
$maxControlStep=(Get-Content -LiteralPath (Join-Path $root 'Config/Mecanum.json') -Raw|ConvertFrom-Json).max_control_step_s
$results=@()
foreach($case in $Cases){
 $log=Join-Path $run "$case.log";$scenario="mecanum_$case"
 $step=(1.0/$PhysicsHz).ToString('G17',[Globalization.CultureInfo]::InvariantCulture)
 $extra=@("-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubstepDeltaTime=$step",'-ini:Engine:[/Script/Engine.PhysicsSettings]:MaxSubsteps=128',"-MecanumPositionIterations=$PositionIterations","-MecanumVelocityIterations=$VelocityIterations")
 if($ProximitySensorFault){$extra+="-ProximitySensorFault=$ProximitySensorFault"}
 if($ProximityValidate){$extra+="-ProximityValidate"}
 if($DynamicsSensorFault){$extra+="-DynamicsSensorFault=$DynamicsSensorFault"}
 if($JointSensorFault){$extra+="-JointSensorFault=$JointSensorFault"}
 if($ReferenceRollingCoefficient -ge 0){$extra+='-ReferenceRollingCoefficient='+$ReferenceRollingCoefficient.ToString('G17',[Globalization.CultureInfo]::InvariantCulture)}
 if($ReducedMecanum){$extra+='-ReducedMecanum';$extra+="-ReducedPhysicsHz=$PhysicsHz"}
 if($SerialPhysics){$extra+='-SingleThreadedPhysics'}
 $console=@();if($Rendered){$console+='r.AntiAliasingMethod 1';$console+='r.ScreenPercentage 75';$console+='sg.ShadowQuality 1'};if($NoCCD){$console+='p.Chaos.Solver.UseCCD 0'};if($UncappedBenchmark){$console+='t.MaxFPS 0'};if($FreshContacts){$console+='p.Chaos.Solver.Collision.AllowManifoldUpdate 0'}
 if($console.Count){$extra+='-ExecCmds='+($console -join ',')}
 if($ReducedSmoothContact){$extra+='-ReducedSmoothContact'}
 if($NativeReducedSupport){$extra+='-NativeReducedSupport'}
 if($ContactDiagnostics){$extra+='-ReducedContactDiagnostics'}
 foreach($switchName in $DiagnosticSwitch){$extra+="-$switchName"}
 if($ReferenceModel -or $WarehouseMap){$extra+='-WarehouseLab';$extra+='-ModelingPhysicsOnly'}
 if($case.StartsWith('mission_')){$extra+='-MissionFixture'}
 if($case -eq 'lift_sensor_pressure'){$extra+='-LiftPressureTripPa=5500000'}
 if($case.StartsWith('drive_sensor_') -and $case -ne 'drive_sensor_warehouse'){$extra+='-SensorDriveFixture'}
 if($case -eq 'step'){$extra+='-Terrain=step'}
 if($case -eq 'halfstep'){$extra+='-Terrain=halfstep'}
 if($case -eq 'split_carry'){$extra+='-Terrain=split'}
 if($case -eq 'ramp_brake'){$extra+='-Terrain=ramp'}
 if($WebTelemetryUrl){if($WebTelemetryUrl -notmatch '^http://127\.0\.0\.1:[0-9]{4,5}/api/telemetry$'){throw 'Loopback telemetry URL required'};$extra+="-WebTelemetry=$WebTelemetryUrl"}
 Write-Output "RUN $scenario $Fps FPS $PhysicsHz Hz"
 $map=if($WarehouseMap){'/Game/Maps/WarehouseLab'}else{'/Engine/Maps/Entry'}
 $renderArgs=if($Rendered){@('-RenderOffscreen','-windowed','-ResX=1280','-ResY=800','-ForceRes','-dx11','-CameraViews')}else{@('-nullrhi')}
 if(($case.StartsWith('pallet_vision_') -or $case.StartsWith('handling_') -or $case.StartsWith('mission_')) -and -not $Rendered){throw 'Pallet perception validation requires real GPU rendering (-Rendered)'}
 & $editor "$root\ForkliftSim.uproject" $map -game -StabilityLab @renderArgs -unattended -nosound -nosplash -nop4 -NoAssetRegistryCacheWrite -benchmark "-fps=$Fps" "-StabilityTest=$scenario" "-abslog=$log" @extra
 $exit=$LASTEXITCODE
 $lines=(Select-String -LiteralPath $log -Pattern 'Display: (RACK_STAGE|RACK_CHECK|MECANUM_CHECK|MECANUM_NUMERICS|MECANUM_COAST|MECANUM_REACTION root|NUMERICAL_STEP|STABILITY_TEST)').Line
 $command=(Select-String -LiteralPath $log -Pattern 'LogInit: Command Line:').Line
 $metadataPass=$command -match ('-fps='+$Fps+'(?: |$)') -and (($command -match '(?: |^)-ReducedSmoothContact(?: |$)') -eq [bool]$ReducedSmoothContact) -and (($command -match '(?: |^)-ReducedMecanum(?: |$)') -eq [bool]$ReducedMecanum)
 $proximityPass=(-not $ProximityValidate) -or (Select-String -LiteralPath $log -Pattern 'PROXIMITY_SCENE_VALIDATION checks=7 PASS' -Quiet)
 $pass=$proximityPass -and $metadataPass -and $exit -eq 0 -and (Select-String -LiteralPath $log -Pattern "STABILITY_TEST $scenario PASS" -Quiet)
 $inner=[regex]::Match(($lines -join ' '),'max_inner_s=([-+0-9.eE]+)')
 $frame=[regex]::Match(($lines -join ' '),'max_game_s=([-+0-9.eE]+)')
 $dt=if($inner.Success){[double]::Parse($inner.Groups[1].Value,[Globalization.CultureInfo]::InvariantCulture)}else{0}
 $frameDt=if($frame.Success){[double]::Parse($frame.Groups[1].Value,[Globalization.CultureInfo]::InvariantCulture)}else{0}
 $stepPass=$dt -gt 0 -and $dt -le 1.0/$PhysicsHz+1e-9 -and [math]::Abs($frameDt-[math]::Min(1.0/$Fps,$maxControlStep)) -lt 1e-6
 $csvMatch=Select-String -LiteralPath $log -Pattern 'STABILITY_CSV (.+)$'|Select-Object -Last 1
 $csv=if($csvMatch){$csvMatch.Matches[0].Groups[1].Value}else{''}
 $sensorMatch=Select-String -LiteralPath $log -Pattern 'JOINT_SENSOR_CSV (.+)$'|Select-Object -Last 1
 $sensorCsv=if($sensorMatch){$sensorMatch.Matches[0].Groups[1].Value}else{''}
 $dynamicsMatch=Select-String -LiteralPath $log -Pattern 'DYNAMICS_SENSOR_CSV (.+)$'|Select-Object -Last 1
 $dynamicsCsv=if($dynamicsMatch){$dynamicsMatch.Matches[0].Groups[1].Value}else{''}
 $proximityMatch=Select-String -LiteralPath $log -Pattern 'PROXIMITY_SENSOR_CSV (.+)$'|Select-Object -Last 1
 $proximityCsv=if($proximityMatch){$proximityMatch.Matches[0].Groups[1].Value}else{''}
 $liftMatch=Select-String -LiteralPath $log -Pattern 'SENSOR_LIFT_CSV (.+)$'|Select-Object -Last 1
 $liftCsv=if($liftMatch){$liftMatch.Matches[0].Groups[1].Value}else{''}
 $guardMatch=Select-String -LiteralPath $log -Pattern 'LIDAR_SAFETY_CSV (.+)$'|Select-Object -Last 1
 $guardCsv=if($guardMatch){$guardMatch.Matches[0].Groups[1].Value}else{''}
 $driveMatch=Select-String -LiteralPath $log -Pattern 'SENSOR_DRIVE_CSV (.+)$'|Select-Object -Last 1
 $driveCsv=if($driveMatch){$driveMatch.Matches[0].Groups[1].Value}else{''}
 $perceptionMatch=Select-String -LiteralPath $log -Pattern 'PALLET_PERCEPTION_CSV (.+)$'|Select-Object -Last 1
 $perceptionCsv=if($perceptionMatch){$perceptionMatch.Matches[0].Groups[1].Value}else{''}
 $handlingMatch=Select-String -LiteralPath $log -Pattern 'HANDLING_CSV (.+)$'|Select-Object -Last 1
 $handlingCsv=if($handlingMatch){$handlingMatch.Matches[0].Groups[1].Value}else{''}
 $missionMatch=Select-String -LiteralPath $log -Pattern 'MISSION_CSV (.+)$'|Select-Object -Last 1
 $missionCsv=if($missionMatch){$missionMatch.Matches[0].Groups[1].Value}else{''}
 $results+=[pscustomobject]@{mission_csv=$missionCsv;handling_csv=$handlingCsv;pallet_perception_csv=$perceptionCsv;sensor_drive_csv=$driveCsv;lidar_safety_csv=$guardCsv;sensor_lift_csv=$liftCsv;proximity_sensor_csv=$proximityCsv;dynamics_sensor_csv=$dynamicsCsv;joint_sensor_csv=$sensorCsv;csv=$csv;case=$case;metadata_pass=[bool]$metadataPass;pass=[bool]($pass -and $stepPass);behaviour_pass=[bool]$pass;step_pass=[bool]$stepPass;max_inner_s=$dt;fps=$Fps;physics_hz=$PhysicsHz;exit_code=$exit;result=$lines;log=$log}
 $lines|Write-Output
 ConvertTo-Json -InputObject $results -Depth 5|Set-Content "$run\results.json" -Encoding UTF8
}
Write-Output "Mecanum report: $run"
if(@($results|Where-Object {-not $_.pass}).Count){throw 'Mecanum qualification failed; all requested cases were collected.'}
Write-Output "MECANUM_SUITE PASS $($results.Count) cases"

