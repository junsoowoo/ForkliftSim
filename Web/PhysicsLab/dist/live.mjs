import {missionCsv,missionNames,missionRouteSvg} from './mission.mjs?v=3adf283ff3e6';
import {handlingControlCsv,handlingNames} from './handling-control.mjs?v=c76c2f293860';
import {palletPerceptionCsv} from './pallet-perception.mjs?v=d47c68433749';
import {driveEstimatorCsv} from './drive-estimator.mjs?v=19078b425cb7';
import {lidarSafetyCsv,lidarPoints} from './lidar-safety.mjs?v=3bb9c3182b4c';
import {liftControlCsv} from './lift-control.mjs?v=340353cc0e68';
import {proximitySensorCsv} from './proximity-sensors.mjs?v=23124addfe26';
import {dynamicsSensorCsv} from './dynamics-sensors.mjs?v=e867dae7e7c8';
import {features,calibration,integrated} from './data.mjs?v=494e559cfedc';
import {jointSensorCsv} from './joint-sensors.mjs?v=7df102027bf6';
import {LiveState,measurements} from './live-state.mjs?v=26b6f5433508';
const $=s=>document.querySelector(s),$$=s=>[...document.querySelectorAll(s)];
const f=(n,d=0)=>Number.isFinite(n)?n.toLocaleString('ko-KR',{minimumFractionDigits:d,maximumFractionDigits:d}):'—';
const state=new LiveState();let filtered=false,featureId='payload',connection='connecting',renderPending=false,lastRender=0;
const rollerDetails=new Set();
document.addEventListener('toggle',e=>{const d=e.target;if(d.matches?.('details[data-roller]')){if(d.open)rollerDetails.add(d.dataset.roller);else rollerDetails.delete(d.dataset.roller);}},true);
const names={FL:'앞 왼쪽',FR:'앞 오른쪽',RL:'뒤 왼쪽',RR:'뒤 오른쪽'};
const escape=s=>String(s??'—').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const value=(v,unit='',digits=2)=>`${f(v,digits)}${unit?' '+unit:''}`;
function cameraMeasurements(F){
 const labels={running:'측정 중',waiting:'첫 영상 대기',disabled:'꺼짐',stale:'오래된 측정',valid:'유효',too_close:'최소 거리 미만',too_far:'최대 거리 초과',dropout:'표본 누락',no_return:'반환 없음'};
 return group('카메라 센서 측정값',['front','rear'].flatMap((key,i)=>{
  const c=F?.camera_sensors?.[key],name=i===0?'전방 ToF':'후방 RGB-D',active=c?.status==='running';
  return [[name+' 상태',F?.source?.state==='stopped'?'세션 종료 · 마지막 측정':c?labels[c.status]??c.status:'센서 수신 대기'],[name+' 중심 거리',active?value(c.center_range_m,'m',3):'—'],[name+' 중심 반환',c?.center_status?labels[c.center_status]??c.center_status:'—'],[name+' 유효 픽셀',active?value(c.valid_fraction*100,'%',1):'—'],[name+' 측정 나이',c?.age_s===undefined?'—':value(c.age_s*1000,'ms',0)],[name+' 작업 거리',c?`${f(c.min_range_m,1)} ~ ${f(c.max_range_m,1)} m`:'—']];
 }));
}
function jointMeasurements(F,Live){
 const j=F?.joint_sensors,labels={valid:'유효',stale:'오래된 값 · 사용 불가',waiting:'첫 측정 대기',initializing:'스위치 안정화 중'};
 const rows=[];
 for(const s of j?.encoders??[]){const ok=Live&&s.valid;rows.push([s.id,ok?`${f(s.counts,0)} count · ${f(s.angular_velocity_rad_s,3)} rad/s · age ${f(s.age_s*1000,0)} ms`:Live?(labels[s.status]??s.status):'수신 대기 / 중단']);}
 const h=j?.fork_height,l=j?.fork_limits;
 rows.push(['포크 높이 센서',Live&&h?.valid?`${f(h.height_m,3)} m · age ${f(h.age_s*1000,0)} ms`:Live?(labels[h?.status]??'대기'):'수신 대기 / 중단']);
 rows.push(['하단 / 상단 리미트',Live&&l?.valid?`${l.lower_active?'ON':'OFF'} / ${l.upper_active?'ON':'OFF'}`:Live?(labels[l?.status]??'대기'):'수신 대기 / 중단']);
 rows.push(['리미트 작동 높이 하 / 상',l?`${f(l.lower_trip_height_m,3)} / ${f(l.upper_trip_height_m,3)} m`:'—']);
 rows.push(['센서 고장 주입',j?[...(j.encoders??[]),h,l].some(s=>s?.injected_transport_fault_diagnostic)?'전송 중단 주입 중 · U 복구':'없음 · Unreal U로 전송 중단':'—']);
 rows.push(['제어 연결',j?(j.lift_controller_connected?'높이·리미트 → 승강 제어 · 엔코더 미연결':'측정만 제공 · 제어 미연결'):'—']);
 return group('엔코더·높이·리미트 센서 측정값',rows);
}
function dynamicsMeasurements(F,Live){
 const bank=F?.dynamics_sensors,rows=[];
 const fields=[['imu','차체 IMU'],['lift_pressure','실린더 게이지 압력'],['fork_load_left','왼쪽 포크 축방향 접촉 하중'],['fork_load_right','오른쪽 포크 축방향 접촉 하중']];
 for(const [id,label] of fields){const s=bank?.sensors?.find(s=>s.id===id),ok=Live&&s?.valid;
  rows.push([label+' 상태',!Live?'수신 대기 / 중단':s?.status==='valid'?'유효':s?.status==='stale'?'오래된 값 · 사용 불가':'첫 측정 대기']);
  if(id==='imu'){
   rows.push(['비력 X / Y / Z',ok?['x','y','z'].map(a=>f(s['specific_force_'+a+'_mps2'],3)).join(' / ')+' m/s²':'—']);
   rows.push(['각속도 X / Y / Z',ok?['x','y','z'].map(a=>f(s['gyro_'+a+'_rad_s'],4)).join(' / ')+' rad/s':'—']);
  }else rows.push([label,ok?value(id==='lift_pressure'?s.gauge_pressure_pa/1e6:s.fork_axis_force_n,id==='lift_pressure'?'MPa':'N',id==='lift_pressure'?3:1):'—']);
  rows.push([label+' 표본 나이 / 포화',ok?`${f(s.age_s*1000,0)} ms / ${s.clipped_mask?'측정 범위 초과':'정상'}`:'—']);
 }
 rows.push(['측정 기준','IMU: X 전방·Y 우측·Z 위 · 정지 비력 Z ≈ 9.81']);
 rows.push(['포크 하중 해석','가상 계측 패드 · 동적 접촉력 · 화물 질량과 구분']);
 rows.push(['고장 주입 / 제어',bank?`${bank.sensors.some(s=>s.injected_transport_fault_diagnostic)?'전송 중단 주입 중':'정상'} · U 전환 · ${bank.lift_controller_connected?'압력 → 승강 제한 · IMU·하중 미연결':'자동 제어 미연결'}`:'—']);
 return group('IMU·압력·좌우 포크 하중 센서 측정값',rows);
}
function proximityMeasurements(F,Live){
 const bank=F?.proximity_sensors,rows=[],labels={valid:'유효',no_return:'반환 없음',too_close:'최소 거리 미만 · 가림',too_far:'최대 거리 초과',dropout:'광학 측정 누락',initializing:'신호 안정화 중',unavailable:'측정 사용 불가'};
 const names={heel_left:'왼쪽 뿌리 전방',heel_right:'오른쪽 뿌리 전방',tip_left:'왼쪽 끝 전방',tip_right:'오른쪽 끝 전방',presence_left_near:'왼쪽 위·뿌리 쪽',presence_right_near:'오른쪽 위·뿌리 쪽',presence_left_far:'왼쪽 위·끝 쪽',presence_right_far:'오른쪽 위·끝 쪽'};
 for(const [id,name] of Object.entries(names)){const s=bank?.sensors?.find(x=>x.id===id);let text='첫 측정 대기';
  if(!Live)text='수신 대기 / 중단';else if(s?.status==='stale')text='오래된 값 · 사용 불가';else if(s?.measurement_valid)text=(s.kind==='range'?value(s.range_m,'m',3):s.detected?'물체 감지 ON':'물체 없음 OFF')+` · age ${f(s.age_s*1000,0)} ms`;else if(s?.transport_valid)text=labels[s.reading_status]??s.reading_status;
  rows.push([name,text]);
 }
 rows.push(['센서 범위','전방 0.02~1.5m · 위쪽 존재 감지 0~0.15m']);
 rows.push(['감지 해석','광선에 걸린 물체 · 팔레트 종류·완전 삽입 확정 아님']);
 rows.push(['조작 / 제어','Unreal L 광선 표시 · U 통신 중단/복구 · 자동 제어 미연결']);
 return group('포크 근거리·물체 존재 센서 측정값',rows);
}
function liftControlMeasurements(F,Live){
 const c=F?.lift_control,active=c?.mode==='sensor_feedback_direct_valve';
 const names={vehicle_hold:'차량 전체 정지에 따른 승강 중립 유지',initializing:'센서 정상 수신 대기',holding:'목표 높이 유지',tracking:'높이 추종',fault_latched:'고장 정지 · Y 확인 필요',pressure_limited:'압력 제한 · 상승 차단',upper_limit:'상한 스위치 · 상승 차단',lower_limit:'하한 스위치 · 하강 차단'};
 const reasons={sensor_timeout:'센서값 사용 불가/시간 초과',actuator_command_expired:'밸브 명령 유효시간 초과',invalid_sensor_or_command:'센서·명령값 모순/범위 초과',height_jump_or_clock:'높이 급변/취득 시계 이상'};
 const ok=Live&&active;
 return group('센서 기반 포크 높이 제어',[
 ['제어 모드',!Live?'수신 대기 / 중단':active?'높이·리미트·압력 센서 → 유압 밸브':c?.mode==='sensor_control_unavailable_hold'?'제어 설정 오류 · 승강 중립 유지':c?'기존 물리 비교 시험 / 센서 제어 비활성':'첫 수신 대기'],
 ['상태',ok?names[c.state]??c.state:'—'],['정지 사유',ok?(reasons[c.fault_reason]??c.fault_reason)||'없음':'—'],
 ['목표 / 센서 높이',ok?`${f(c.target_m,3)} / ${f(c.measured_height_m,3)} m`:'—'],['측정 압력 / 상승 제한',ok?`${f(Number.isFinite(c.measured_pressure_pa)?c.measured_pressure_pa/1e6:null,3)} / ${f(c.pressure_trip_pa/1e6,3)} MPa`:'—'],
 ['밸브 명령 / 펌프 허용',ok?`${f(c.command_spool,3)} / ${c.pump_enabled?'허용':'차단'}`:'—'],['센서 기반 속도 추정',ok&&!c.fault_latched&&Number.isFinite(c.measured_height_m)?value(c.estimated_velocity_mps,'m/s',3):'—'],
 ['복구 확인',ok?c.ack_rejected?'신호가 아직 정상화되지 않아 거절됨':c.fault_latched?'신호 복구 후 Y · 이전 목표로 자동 재출발 안 함':'Q/E 목표 높이 · U 고장 주입 · Y 복구 확인':'—']
 ]);
}
function missionMeasurements(F,Live){
 const m=F?.mission;if(!m)return group('자율주행·물류 임무',[['상태','Unreal 임무 모듈 수신 대기']]);
 return group('자율주행·물류 임무',[
 ['임무 상태',(Live?'':'마지막 수신 · ')+(missionNames[m.state]??m.state)],['정지/대기 사유',m.reason||'—'],['선택 / 수행 작업단',`${m.selected_high?'2단':'1단'} / ${m.high?'2단':'1단'}`],['위치 추정',m.localized?`${value(m.measured_x_m,'m',3)} / ${value(m.measured_y_m,'m',3)}`:'미확정'],['위치 보정 나이',value(m.localization_age_s,'s',2)],['화물 상태',m.carrying?'운반 중':m.dropoff_receipt?'하역 확인':'인양 전'],['인양 / 하역 확인',`${m.pickup_receipt?'확인':'미확인'} / ${m.dropoff_receipt?'확인':'미확인'}`],['재계획 / 재시도',`${m.replans??0} / ${m.retries??0}`],['경로 길이',value(m.path_length_m,'m',2)],['동적 지도 관측 셀',m.dynamic_cells??'—'],['적재물 자기 반사 구간',m.carrier_self_filter?'현재 하중·삽입 측정으로 유효':'허용하지 않음'],['계획·상태 계산',value(m.planner_ms,'ms',2)],['전진 / 횡이동 명령',`${value(m.vx_command_mps,'m/s',3)} / ${value(m.vy_command_mps,'m/s',3)}`],['조작','Enter 시작 · PgDn/PgUp 1/2단 · Backspace 일시정지 · ] 재개 · 0 취소']])+missionRouteSvg(m);
}
function handlingMeasurements(F,Live){
 const h=F?.handling_control;if(!h)return group('자동 정렬·삽입·하역',[['상태','Unreal 작업 모듈 수신 대기']]);
 return group('자동 정렬·삽입·하역',[
 ['작업 상태',Live?(handlingNames[h.state]??h.state):'마지막 수신 · '+(handlingNames[h.state]??h.state)],['정지 사유',h.reason||'—'],['작업 종류',h.place?'랙 하역':'팔레트 인양'],['작업단 높이',value(h.tier_m,'m',3)],['승강 목표',value(h.target_height_m,'m',3)],['전진 / 횡이동 명령',`${value(h.vx_command_mps,'m/s',3)} / ${value(h.vy_command_mps,'m/s',3)}`],['회전 명령',value(h.yaw_command_rad_s,'rad/s',3)],['완전 삽입 확인 이력',h.full_seen?'확인':'없음'],['하중 전달 확인',h.transfer_seen?'확인':'미확인'],['필수 센서',h.sensors_healthy?'유효':'미확인 / 이상'],['접근 영역 허용',h.target_permission?'등록/인식 대상 영역만':'해제'],['조작','5 바닥 인양 · 6/7 랙 1/2단 인양 · 8/9 랙 1/2단 하역 · 0 취소']]);
}
function palletPerceptionMeasurements(F,Live){
 const p=F?.pallet_perception,names={unknown:'판정 불가 / 측정 대기',observing:'측정 안정 확인 중',aligned:'정렬됨 · 아직 삽입 전',misaligned:'정렬 불량',partial:'부분 삽입',fully_inserted:'완전 삽입 증거 확인',supported:'양쪽 접촉 하중 · 현재 삽입 깊이는 미확정',uneven_support:'한쪽 접촉 하중'};
 const vision=Live&&p?.vision_fresh,geometry=Live&&p?.signals_fresh&&(vision||p?.tracked)&&!['unknown','supported','uneven_support'].includes(p?.state);
 return group('팔레트 인식·삽입 판정',[
 ['상태',Live?names[p?.state]??'첫 수신 대기':'수신 대기 / 중단'],['증거',!Live?'—':vision?'최신 깊이 영상':p?.tracked?(p.heel_range_fused?'뿌리 거리 보강 + 제한된 추적':'가림 구간의 제한된 오도메트리 추적'):'현재 영상 인식 없음'],['기하 일치 점수',vision?value(p.confidence,'',2):'—'],['앞면 거리 / 각도',vision?`${value(p.plane_distance_m,'m',3)} / ${value(p.yaw_rad*180/Math.PI,'deg',1)}`:'—'],['통로 좌우 폭',vision?`${value(p.left_opening_width_m,'m',3)} / ${value(p.right_opening_width_m,'m',3)}`:'—'],['알려진 통로 높이',Live?value(p?.nominal_opening_height_m,'m',3):'—'],['좌우 삽입 깊이 추정',geometry?`${value(p.depth_left_m,'m',3)} / ${value(p.depth_right_m,'m',3)}`:'—'],['깊이 오차 여유',geometry?value(p.depth_uncertainty_m,'m',3):'—'],['좌우 / 높이 정렬 오차',geometry?`${value(p.lateral_error_m,'m',3)} / ${value(p.height_error_m,'m',3)}`:'—'],['지지 전 완전 삽입 이력',Live&&p?p.last_full_before_support?'있음 · 현재 깊이의 보증 아님':'없음':'—'],['기능 범위','알려진 팔레트 계열의 깊이 형상 검사 · 자동 정렬/삽입은 후속']]);
}
function driveEstimatorMeasurements(F,Live){
 const d=F?.drive_estimator,valid=Live&&d?.state==='ready'&&d?.data_healthy,located=valid&&d?.localized;
 const names={calibrating:'정지 상태 바이어스 추정 · 이동 입력을 놓으세요',ready:'센서 주행 허용',fault_latched:'주행 센서/명령 고장 · 중립 후 Y 확인',waiting:'센서 수신 대기'};
 return group('센서 주행·2D 위치 추정',[
 ['상태',!Live?'수신 대기 / 중단':names[d?.state]??'첫 수신 대기'],['전진 / 횡 속도',valid?`${value(d.vx_mps,'m/s',3)} / ${value(d.vy_mps,'m/s',3)}`:'—'],['회전 속도',valid?value(d.yaw_rad_s*180/Math.PI,'deg/s',2):'—'],['추정 gyro 바이어스',Live?value(d?.gyro_bias_rad_s,'rad/s',5):'—'],['오도메트리 X / Y',valid?`${value(d.odom_x_m,'m',3)} / ${value(d.odom_y_m,'m',3)}`:'—'],['지도 보정 X / Y',located?`${value(d.map_x_m,'m',3)} / ${value(d.map_y_m,'m',3)}`:'미확정 / 보정 오래됨'],['지도 보정 방향',located?value(d.map_yaw_rad*180/Math.PI,'deg',2):'—'],['정합 잔차 / 나이',located?`${value(d.match_rms_m,'m',3)} / ${value(d.match_age_s,'s',2)}`:'—'],['관측하지 못한 이동 시간',Live?value(d?.unobserved_motion_s,'s',3):'—'],['고정 지도',Live?d?.map_id??'대기':'—'],['측정 의미','엔코더·IMU 오도메트리 + 고정 지도 LiDAR 정합 · SLAM/자율주행 아님']]);
}
function lidarMeasurements(F,Live){
 const g=F?.lidar_safety,names={manual_creep:'Shift 수동 저속 작업 · 장애물 정지 일시 해제',initializing:'센서 정상 수신 대기',ready:'주행 허용',obstacle_hold:'장애물 · 주행 정지',fault_latched:'전체 정지 · 복구 확인 필요',clear_wait_ack:'장애물 해소 · Y 확인 대기',recovery_hold:'복구 후 정지 유지'};
 const rows=[['상태',!Live?'수신 대기 / 중단':g?names[g.state]??g.state:'첫 수신 대기'],['주행 / 승강',Live&&g?`${g.drive_hold?'주행 차단':'주행 허용'} / ${g.lift_hold?'승강 차단':'승강 조작 가능'}`:'—'],['정지 사유',Live&&g?({drive_controller_fault:'주행 센서/명령 고장',emergency_stop:'X 비상정지',required_sensor_stale:'필수 센서 수신 중단',lift_controller_fault:'승강 제어기 고장',scan_unknown:'스캔 측정 사용 불가'}[g.reason]??g.reason)||'없음':'—'],['보호 방식',Live&&g?(g.protection_mode==='mission_fixed_heading_sweep'?'임무: 전체 외곽 + 진행 방향 정지 여유':'전체 고정 여유'):'—'],['관측된 보호 영역 내 점',Live&&g?f(g.protected_points,0):'—'],['관측 표면과 차체·포크 외곽의 여유',Live&&g?.data_healthy?value(g.clearance_m,'m',3):'—']];
 for(const s of g?.scanners??[])rows.push([s.id,Live&&s.fresh?`${s.ranges_m.length} rays · seq ${s.sequence} · age ${f(s.age_s*1000,0)} ms`:Live?'스캔 대기 / 오래된 값':'수신 중단']);
 rows.push(['조작','X 전체 비상정지 · Y 복구 확인 · I LiDAR 광선']);rows.push(['장애물 정지','주행은 정지 · 팔레트 인양 등 정지 상태의 승강 조작은 허용']);rows.push(['수동 작업','Shift 유지: 장애물 주행 정지만 일시 해제, 목표 속도 0.15 m/s · 고장/비상정지는 유지']);
 let html=group('2D LiDAR·차량 정지 및 복구',rows);
 if(Live&&g){const pts=lidarPoints(g);html+=`<section class="live-value-group"><h3>차체 기준 실제 LiDAR 측정점</h3><svg viewBox="-5 -6 10 10" style="width:100%;height:220px;background:#f1f6f3" role="img" aria-label="전후방 LiDAR 측정점. 지도나 위치 추정 결과가 아닙니다."><rect x="${-g.field_half_width_m}" y="${-g.field_front_m}" width="${2*g.field_half_width_m}" height="${g.field_front_m+g.field_rear_m}" fill="none" stroke="#b34f40" stroke-width=".03" stroke-dasharray=".1 .1"/><rect x="-.65" y="-2.05" width="1.3" height="2.83" fill="#758984" opacity=".3"/>${pts.map(p=>`<circle cx="${p.y}" cy="${-p.x}" r=".035" fill="${p.id==='front_left'?'#007c83':'#bd7620'}"/>`).join('')}</svg><p style="font-size:11px">점선은 보호 영역의 외접 사각형 · 거리 표본만 표시 · 점이 없는 구간은 장애물 부재 보장이 아님</p></section>`;}
 return html;
}
function schedule(){if(renderPending)return;renderPending=true;setTimeout(()=>{renderPending=false;render();},Math.max(0,100-(performance.now()-lastRender)));}
function solverContact(w,valid){
 const c=w.solver_contact;if(!valid||!c?.valid)return '<div class="wheel-sub">실제 접촉점 하중 대기</div>';
 const b=c.band_normal_n,total=c.point_normal_sum_n;
 return `<div class="wheel-sub"><span>실제 접촉점 최대 ${f(c.peak_active_points)}개 / 물리 스텝</span></div><div class="wheel-sub"><span>폭 방향 축 − / 중앙 / + 하중</span></div><div class="wheel-sub"><span>${b.map(n=>f(n/1000,2)).join(' / ')} kN</span></div><div class="contact-bands" aria-label="실제 접촉 하중 분포">${b.map((n,i)=>`<i class="band-${i}" style="width:${total>0?100*n/total:0}%"></i>`).join('')}</div><div class="wheel-sub"><span>접촉 하중 중심 X/Y/Z ${c.load_center_world_m.map(n=>f(n,3)).join(' / ')} m</span></div><div class="wheel-sub"><span>법선력 모멘트 X/Y/Z ${c.normal_moment_world_nm.map(n=>f(n,2)).join(' / ')} Nm</span></div><div class="wheel-sub"><span>솔버 접촉점 집계 · 면압 추정과 별도</span></div>`;
}
function mecanumView(w,valid){
 const m=w.mecanum;if(!m)return '';
 if(m.independent_roller_states===false)return `<section class="tread-readings"><div class="wheel-sub"><strong>경량 메카넘 · 등가 방향성 접촉</strong></div><div class="wheel-sub"><span>바퀴 전체 ${f(m.physical_wheel_mass_kg,1)} kg · 구동 ${f(w.drive_torque_nm,1)} Nm</span></div><div class="wheel-sub"><span>하중 ${valid?f(w.normal_n/1000,2):'—'} kN · 마찰력 ${valid?f(w.friction_n,1):'—'} N</span></div><div class="wheel-sub"><span>구동 방향 슬립 RMS ${valid?f(w.long_slip_mps,3):'—'} m/s</span></div><div class="wheel-sub"><span>접촉 탄성 ${f(m.contact_elastic_j,3)} J · 모델 손실 추정 ${f(m.contact_loss_estimate_j,2)} J</span></div><div class="wheel-sub"><span>개별 롤러 회전·충격·직각 방향 슬립은 계산 범위 밖</span></div></section>`;
 const rs=m.rollers??[],load=rs.reduce((n,r)=>n+r.normal_n,0),peak=Math.max(1,...rs.map(r=>r.normal_n));
 const rpm=rs.map(r=>r.relative_omega_rad_s*60/(2*Math.PI));const err=Math.max(0,...rs.map(r=>r.anchor_error_m));
 return `<section class="tread-readings"><div class="wheel-sub"><strong>메카넘 · ${rs.length}개 자유 롤러 · ${m.hand>0?'＋':'−'}45°</strong></div><div class="wheel-sub"><span>허브 ${f(m.hub_mass_kg,1)} kg / 롤러 합 ${f(rs.reduce((n,r)=>n+r.mass_kg,0),1)} kg</span></div><div class="wheel-sub"><span>접촉 슬립 RMS 축 / 직각 ${valid?f(w.long_slip_mps,3):'—'} / ${valid?f(w.lateral_slip_mps,3):'—'} m/s</span></div><div class="wheel-sub"><span>허브 ${f(w.omega_rad_s*60/(2*Math.PI),1)} RPM · 구동 ${f(w.drive_torque_nm,1)} Nm</span></div><div class="tread-bars" aria-label="실제 롤러별 접촉 하중">${rs.map(r=>`<span title="롤러 ${r.index}: ${valid?f(r.normal_n,1):'—'} N / ${f(r.relative_omega_rad_s,2)} rad/s"><i class="${r.normal_n>1?'loaded':''}" style="height:${Math.max(2,100*r.normal_n/peak)}%"></i></span>`).join('')}</div><div class="wheel-sub"><span>롤러 하중 합 ${valid?f(load/1000,2):'—'} kN · 상대 RPM ${f(Math.min(...rpm),1)} ~ ${f(Math.max(...rpm),1)}</span></div><div class="wheel-sub"><span>관절 위치 오차 최대 ${f(err*1000,3)} mm · 베어링 손실 ${f(m.bearing_loss_w,2)} W</span></div><details data-roller="${w.id}" ${rollerDetails.has(w.id)?'open':''}><summary>롤러별 실제 값</summary><table><thead><tr><th>번호</th><th>N</th><th>상대 RPM</th><th>저항 Nm</th></tr></thead><tbody>${rs.map(r=>`<tr><td>${r.index}</td><td>${valid?f(r.normal_n,1):'—'}</td><td>${f(r.relative_omega_rad_s*60/(2*Math.PI),1)}</td><td>${f(r.bearing_torque_nm,4)}</td></tr>`).join('')}</tbody></table></details><div class="wheel-sub"><span>접촉 모델: ${m.contact_geometry==='inscribed-9-spheres'?'구면 조합':m.contact_geometry==='barrel-convex'?'볼록 다면체':'정보 대기'}</span></div><div class="wheel-sub"><span>강체 롤러 · 고무 변형·면압·정격 하중 미검증</span></div></section>`;
}
function treadView(w,valid){
 const t=w.tread;if(!t?.enabled)return '';
 const pad=t.pads??[];const load=pad.reduce((n,p)=>n+p.normal_n,0);
 return `<section class="tread-readings"><div class="wheel-sub"><strong>${t.coupled?'연결 탄성 링':'국부 탄성'} · 트레드 ${t.segments}개</strong></div><div class="wheel-sub"><span>최소 / 최대 변형 ${f(t.min_compression_m*1000,2)} / ${f(t.max_compression_m*1000,2)} mm</span></div><div class="tread-bars" aria-label="실제 조각별 압축 변형">${pad.map(p=>`<span title="조각 ${p.index}: ${f(p.compression_m*1000,3)} mm · ${valid?f(p.normal_n,1):'—'} N"><i class="${p.normal_n>1?'loaded':''}" style="height:${Math.max(2,Math.min(100,100*Math.max(0,p.compression_m)/t.travel_m))}%"></i></span>`).join('')}</div><div class="wheel-sub"><span>압축 막대 · ${f(t.travel_m*1000,0)} mm 기준 / 음수는 신장</span></div><div class="wheel-sub"><span>조각 하중 합 ${valid?f(load/1000,2):'—'} kN · 탄성 에너지 ${f(t.stored_j,2)} J</span></div>${t.coupled?`<div class="wheel-sub"><span>최대 전단 ${f(t.max_shear_m*1000,2)} mm · 회전 ${f(t.max_rotation_rad*180/Math.PI,2)} °</span></div><div class="wheel-sub"><span>전단 / 회전 / 연결 에너지 ${f(t.shear_stored_j,2)} / ${f(t.rotation_stored_j,2)} / ${f(t.link_stored_j,2)} J</span></div>`:''}<div class="wheel-sub"><span>스트로크 ${t.bottomed?'한계 도달':'정상'} · 연속체 고무 해석과 구분</span></div><div class="wheel-sub"><span>${t.coupled?'연결 링 시험 모델 · 실측 미보정':'해상도 수렴 미확인 · 실측 미보정'}</span></div>${!t.coupled&&t.segments===32?'<div class="wheel-sub"><span>검증 기록 · 32조각 한쪽 턱: 스트로크 기준 미통과</span></div>':''}</section>`;
}
function statusView(){
 const latest=state.latest,age=latest?Date.now()-latest.received_at_ms:null;
 const mode=connection==='disconnected'?'disconnected':!latest?'waiting':latest.frame.source.state==='stopped'?'stopped':age>1500?'stale':'running';
 const labels={waiting:['PLAY 대기','Unreal 데이터 대기','Start.bat로 실행하고 Unreal Editor에서 Play를 누르세요. 수신 전에는 예시 값을 표시하지 않습니다.'],running:['LIVE','Unreal 수신 중','Unreal에서 측정한 실제 상태입니다. 화물 양중·운반·마찰 변경을 Unreal에서 조작하세요.'],stale:['갱신 중단','일시정지 / 수신 중단','최근 1.5초 이상 새 데이터가 없습니다. 아래 값은 마지막 수신값입니다. Unreal의 Pause·Play 상태를 확인하세요.'],stopped:['PLAY 종료','Unreal Play 종료','Play가 종료됐습니다. 아래는 마지막 측정값입니다. 다시 Play하면 새 세션으로 연결됩니다.'],disconnected:['서버 끊김','로컬 서버 연결 끊김','로컬 브리지와 연결되지 않습니다. Start.bat 또는 Start.bat -Website로 서버를 시작하세요.']};
 const [badge,label,detail]=labels[mode];$('#stream-badge').textContent=badge;$('#connection-label').textContent=label;$('#stream-detail').textContent=detail;$('#stream-notice').className=`model-notice ${mode}`;$('#connection').dataset.state=mode;$('#page-lab').classList.toggle('not-live',mode!=='running');
 $('#source-badge').textContent=mode==='running'?'UNREAL LIVE':badge;
 $('#receipt-age').textContent=latest?`마지막 수신: ${f(Math.max(0,age)/1000,1)}초 전`:'마지막 수신: 없음';
 $('#frame-counter').textContent=latest?`sim ${f(latest.frame.sim_time_s,2)} s · 실시간 대비 ${f(latest.frame.actual_realtime_ratio,2)}배 · seq ${latest.frame.source.sequence}`:'Unreal 데이터 대기';
 return mode;
}
function group(title,rows){return `<section class="live-value-group"><h3>${title}</h3><dl>${rows.map(([label,v])=>`<div><dt>${label}</dt><dd>${escape(v)}</dd></div>`).join('')}</dl></section>`;}
// Preserve live controls and focus while actual telemetry updates text.
function patchHtml(target,html){
 const template=document.createElement('template');template.innerHTML=html;
 const sync=(parent,source)=>{
  const desired=[...source.childNodes];
  for(let i=0;i<desired.length;i++){
   const next=desired[i],old=parent.childNodes[i];
   if(!old){parent.append(next.cloneNode(true));continue;}
   if(old.nodeType!==next.nodeType||old.nodeName!==next.nodeName){parent.replaceChild(next.cloneNode(true),old);continue;}
   if(old.nodeType===Node.TEXT_NODE){if(old.nodeValue!==next.nodeValue)old.nodeValue=next.nodeValue;continue;}
   if(old.nodeType===Node.ELEMENT_NODE){
    for(const attr of [...old.attributes])if(!next.hasAttribute(attr.name))old.removeAttribute(attr.name);
    for(const attr of [...next.attributes])if(old.getAttribute(attr.name)!==attr.value)old.setAttribute(attr.name,attr.value);
    sync(old,next);
   }
  }
  while(parent.childNodes.length>desired.length)parent.lastChild.remove();
 };
 sync(target,template.content);
}
function render(){
 lastRender=performance.now();const mode=statusView(),e=state.latest,F=e?.frame,M=measurements(F),valid=F?.contact_sample_valid;
 const E=F?.energy_audit?.reference_valid?F.energy_audit:null;
 const forkTotal=Number.isFinite(M.forkLeft)&&Number.isFinite(M.forkRight)?M.forkLeft+M.forkRight:null;
 patchHtml($('#metrics'),[['실제 차체 속력',F?Math.hypot(F.vehicle.speed_mps,F.vehicle.lateral_speed_mps):null,'m/s',2,`전후 ${f(F?.vehicle.speed_mps,2)} / 좌우 ${f(F?.vehicle.lateral_speed_mps,2)} m/s`],['좌우 포크 지지력',forkTotal===null?null:forkTotal/1000,'kN',2,`L ${f(M.forkLeft===null?null:M.forkLeft/1000,2)} / R ${f(M.forkRight===null?null:M.forkRight/1000,2)} kN`],['실제 실린더 압력',M.pressure===null?null:M.pressure/1e6,'MPa',2,'Unreal 유압 상태 · 요구 압력 추산 아님'],['실제 바퀴 수직하중 합',M.normalTotal===null?null:M.normalTotal/1000,'kN',2,'접촉 충격량에서 측정한 평균 반력']].map(([title,n,unit,digits,note])=>`<article class="metric"><div class="metric-top"><span>${title}</span><span class="metric-icon">↗</span></div><p class="metric-value">${f(n,digits)}<small>${unit}</small></p><p class="metric-foot">${note}</p></article>`).join(''));
 $('#scene-state').textContent=!F?'Unreal 수신 대기':!valid?'첫 물리 출력 대기':F.has_payload?`${F.scenario==='manual'?'수동 조작':F.scenario} · ${F.terrain}`:'무적재 모드';
 $('#scene').innerHTML=drawScene(F,M);
 $('#scene-facts').innerHTML=[['실제 포크 높이',value(M.height,'m',3)],['실제 포크 목표',value(F?.fork?.target_m,'m',3)],['실제 팔레트 상승',value(F?.pallet?.height_m,'m',3)]].map(([label,v])=>`<div class="scene-fact">${label}<strong>${escape(v)}</strong></div>`).join('');
 const wheels=F?.wheels??['FL','FR','RL','RR'].map(id=>({id}));const max=Math.max(1,...wheels.map(w=>valid?(filtered?w.normal_filtered_n:w.normal_n):0));
 patchHtml($('#wheel-loads'),wheels.map(w=>{const n=valid?(filtered?w.normal_filtered_n:w.normal_n):null;return `<div><div class="wheel-top"><code>${w.id}</code><span>${names[w.id]}</span></div><p class="wheel-figure">${f(n===null?null:n/1000,2)}<small>kN</small></p><div class="load-track"><i style="width:${n===null?0:Math.min(100,Math.max(0,n)/max*100)}%"></i></div><div class="wheel-sub"><span>${w.mecanum?'고정 축 · 조향 없음':`눌림 ${f(w.compression_m===undefined?null:w.compression_m*1000,1)} mm`}</span><span>접촉 ${w.contact_events??'—'}</span></div><div class="wheel-sub"><span>구름 ${f(w.rolling_torque_nm,2)} Nm</span><span>베어링 ${f(w.bearing_torque_nm,2)} Nm</span></div>${w.detailed_compliance?`<div class="wheel-sub"><span>전후 / 좌우 전단 ${f(w.shear_x_m*1000,2)} / ${f(w.shear_y_m*1000,2)} mm</span></div><div class="wheel-sub"><span>${w.estimated_patch_valid===false?'패치·면압: 이 모델에서 미산정':`추정 패치 ${f(w.estimated_patch_length_m*1000,1)} mm · 추정 최대 면압 ${f(w.estimated_peak_pressure_pa/1e6,2)} MPa`}</span></div>`:''}${mecanumView(w,valid)}${treadView(w,valid)}${solverContact(w,valid)}</div>`;}).join(''));
 $('#normal-total').textContent=value(M.normalTotal,'N',1);$('#measurement-window').textContent=value(F?.measurement_interval_s===undefined?null:F.measurement_interval_s*1000,'ms',2);
 patchHtml($('#live-values'),group('실행 시간 · 수치 상태',[["실행 FPS",value(F?.game_fps,'FPS',1)],["롤러 적분",F?.bearing_integrator==="roller_coordinates_eliminated"?"경량 · 롤러 좌표 제거":F?.bearing_integrator==="symmetric_spin_verlet"?"대칭 회전 · 베어링 반작용":F?.bearing_integrator??"—"],["선택한 내부 간격",value(F?.physics_selected_step_s===undefined?null:F.physics_selected_step_s*1000,"ms",3)],["관측 최소 평균 내부 간격",value(F?.physics_smallest_observed_s===undefined?null:F.physics_smallest_observed_s*1000,"ms",3)],["내부 시간 합 오차",value(F?.physics_inner_clock_error_s,"s",9)],["실제 시간 대비 진행",value(F?.actual_realtime_ratio,'배',2)],["물리 누적 시간",value(F?.physics_elapsed_s,'s',3)],["제어－물리 시간 차이",value(F?.physics_clock_lag_s,'s',4)],["무입력 정지 유지",F?.controls?.auto_hold===true?"축 브레이크 적용":F?"해제":"—"],["구동 갱신",F?.drive_update==="physics_inner_step"?"내부 물리 스텝":F?"화면 프레임":"—"],["최대 제어 간격",value(F?.game_max_frame_s===undefined?null:F.game_max_frame_s*1000,'ms',3)]])+group('화물·포크',[
  ['장비 + 전체 적재물 질량',value(F?F.vehicle.dry_mass_kg+(F.pallet?.total_mass_kg??0):null,'kg',0)],['화물 질량',value(F?.cargo?.mass_kg,'kg',1)],['팔레트·화물 지면 반력',value(valid&&F.pallet?F.pallet.ground_n+(F.cargo?.ground_n??0):null,'N',1)],['좌 / 우 삽입 겹침',`${f(F?.fork.tines[0]?.depth_m,3)} / ${f(F?.fork.tines[1]?.depth_m,3)} m`],['포크 접촉 압력중심 L / R',`${f(F?.fork.tines[0]?.cop_x_m,3)} / ${f(F?.fork.tines[1]?.cop_x_m,3)} m`],['화물 상대 미끄럼 속도',value(F?.cargo?.slip_mps,'m/s',3)]
 ]) +cameraMeasurements(F)+jointMeasurements(F,mode==='running')+dynamicsMeasurements(F,mode==='running')+proximityMeasurements(F,mode==='running')+liftControlMeasurements(F,mode==='running')+lidarMeasurements(F,mode==='running')+driveEstimatorMeasurements(F,mode==='running')+missionMeasurements(F,mode==='running')+handlingMeasurements(F,mode==='running')+palletPerceptionMeasurements(F,mode==='running')+group('포크 구조 · 계측',[
  ['포크 끝 처짐 L / R',`${f(F?.fork.tines[0]?.tip_deflection_m===undefined?null:F.fork.tines[0].tip_deflection_m*1000,3)} / ${f(F?.fork.tines[1]?.tip_deflection_m===undefined?null:F.fork.tines[1].tip_deflection_m*1000,3)} mm`],
  ['최대 굽힘 응력 추정 L / R',`${f(F?.fork.tines[0]?.max_stress_pa===undefined?null:F.fork.tines[0].max_stress_pa/1e6,2)} / ${f(F?.fork.tines[1]?.max_stress_pa===undefined?null:F.fork.tines[1].max_stress_pa/1e6,2)} MPa`],
  ['누적 소성 회전 L / R',`${f(F?.fork.tines[0]?.plastic_deg,4)} / ${f(F?.fork.tines[1]?.plastic_deg,4)} °`],
  ['포크 연결 파손 L / R',F?`${F.fork.tines[0]?.failed?'파손':'정상'} / ${F.fork.tines[1]?.failed?'파손':'정상'}`:'—']
 ])+group('유압 상태',[
  ['실린더 출력 힘',value(F?.hydraulic?.force_n,'N',1)],['펌프 유량',value(F?F.hydraulic.pump_m3s*60000:null,'L/min',2)],['복귀 / 누설 유량',`${f(F?F.hydraulic.return_m3s*60000:null,3)} / ${f(F?F.hydraulic.leak_m3s*60000:null,3)} L/min`],['입력 전력',value(F?.hydraulic?.dc_w,'W',1)],['밸브 / 상태',F?`${f(F.hydraulic.valve,3)} / ${F.hydraulic.holding?'HOLD':'TRACK'}`:'—'],['누적 입력 에너지',value(F?.hydraulic?.dc_j,'J',1)]
 ])+group('마스트 · 배관 · 오일',[
  ['이동 마스트 높이',!F?'—':F.hydraulic.chain_mast?value(F.hydraulic.mast_height_m,'m',3):'직결 승강'],
  ['체인 장력 / 신장',F?.hydraulic.chain_mast?`${f(F.hydraulic.chain_tension_n,1)} N / ${f(F.hydraulic.chain_stretch_m*1000,3)} mm`:'—'],
  ['오일 온도 / 동점도',F?.hydraulic.detailed?`${f(F.hydraulic.oil_temp_c,2)} °C / ${f(F.hydraulic.viscosity_cst,2)} cSt`:'—'],
  ['펌프측 압력',F?.hydraulic.detailed?value(F.hydraulic.pump_pressure_pa/1e6,'MPa',3):'—'],
  ['배관 관측 최대 압력',F?.hydraulic.detailed?value(F.hydraulic.peak_line_pressure_pa/1e6,'MPa',3):'—'],
  ['배관 점성 손실',F?.hydraulic.detailed?value(F.hydraulic.line_heat_w,'W',2):'—'],
  ['배관 압력 하한 도달',F?.hydraulic.detailed?(F.hydraulic.pressure_floor_hit?'발생 · 공동화 모델 범위 밖':'없음'):'—']
 ])+group('에너지 수지 · 진단',[
   ['저장 에너지 변화',value(E?.stored_delta_j,'J',1)],['구동 / 조향 관절 일',`${f(E?.drive_work_j,1)} / ${f(E?.steering_joint_work_j,1)} J`],['계측 입력 합',value(E?.known_input_j,'J',1)],['모델 손실 합',value(E?.known_loss_j,'J',1)],['롤러 베어링 계측 손실',value(E?.roller_bearing_loss_j,'J',1)],['축 관절 소산 잔차 (부호 포함)',value(E?.shaft_loss_j,'J',1)],['베어링 수송 반작용 일 (부호 포함)',value(E?.bearing_transport_work_j,'J',2)],['체인 감쇠 손실',value(E?.chain_damping_j,'J',2)],['등가 접촉 소산',value(E?.reduced_contact_loss_j,'J',2)],['소성 / 파손 시 제거된 탄성 에너지',`${f(E?.plastic_relaxation_j,2)} / ${f(E?.fracture_release_j,2)} J`],['접촉 법선 / 접선 일 추정',`${f(E?.contact_normal_work_estimate_j,2)} / ${f(E?.contact_tangent_work_estimate_j,2)} J`],['위치 보정 의사 일',value(E?.contact_correction_pseudo_j,'J',2)],['포크 감쇠 추정 (손실 합 제외)',value(E?.fork_damping_estimate_j,'J',2)],['랙 지지 반력',value(valid?F?.handling_rack?.support_n:null,'N',1)],['미계상 손실·수치 잔차',value(E?.unresolved_j,'J',1)],['잔차 해석',E?'접촉·구조·수치 오차 포함 · 전체 에너지 폐합 미완료':'기준 측정 대기']
  ])+group('메카넘 물성 근거 · 비교 전용',[['참고 제품',F?.wheels?.[0]?.mecanum?.catalogue_comparison_model??'—'],['물성 상태',F?.drive_type==='mecanum'?'현재 휠은 가상 설계 · 실측 보정·정격 인증 전':'—'],...['FL','FR','RL','RR'].map(id=>{const m=F?.wheels?.find(w=>w.id===id)?.mecanum;return [id+' 실제 질량 / 카탈로그 하중비',`${f(m?.physical_wheel_mass_kg,1)} kg / ${f(valid&&Number.isFinite(m?.catalogue_load_comparison_ratio)?m.catalogue_load_comparison_ratio*100:null,1)} %`];}),...['FL','FR','RL','RR'].map(id=>{const m=F?.wheels?.find(w=>w.id===id)?.mecanum;return [id+' 내부 스텝 최대 하중',value(valid?m?.contact_inner_peak_n:null,'N',0)];}),...['FL','FR','RL','RR'].map(id=>{const m=F?.wheels?.find(w=>w.id===id)?.mecanum;return [id+' 허브 주관성 (kg·m²)',m?.hub_inertia_mass_frame_kg_m2?.map(x=>f(x,4)).join(' / ')??'—'];}),['하중비 상태','제조사 웹 3,000 kg / 카탈로그 단일 휠 750 kg 상충 · 비교 중단']])+group('주행·재질 설정',[
  ['구동 구조',F?.drive_type==='mecanum'?'메카넘 · 자유 회전 롤러':F?.drive_type??'—'],['스페이스 브레이크 입력',Number.isFinite(F?.controls?.brake)?`${f(F.controls.brake*100,0)} % · ${F.controls.brake>0?'제동 중':'해제'}`:'—'],['바퀴별 브레이크 한도',value(F?.wheels?.[0]?.brake_limit_nm,'Nm',0)],
   ['차체 롤 / 피치',`${f(F?.vehicle.roll_deg,2)} / ${f(F?.vehicle.pitch_deg,2)} °`],['실제 회전 속도',value(F?.vehicle.yaw_rate_deg_s,'°/s',2)],['휠 표면 / 노면 설정 μ',`${f(F?.settings.tyre_mu,2)} / ${f(F?.settings.road_mu,2)}`],['구름저항 Crr / 활성',F?`${f(F.settings.crr,4)} / ${F.settings.resistance_enabled?'ON':'OFF'}`:'—'],['화물 정지 / 운동 설정 μ',`${f(F?.cargo?.static_mu_config,2)} / ${f(F?.cargo?.dynamic_mu_config,2)}`],['차체 위치 X / Y',F?`${f(F.vehicle.world_position_m[0],2)} / ${f(F.vehicle.world_position_m[1],2)} m`:'—'],['재생 배속 설정',value(F?.playback_rate,'×',2)],['실제 누적 물리 시간',value(F?.physics_elapsed_s,'s',3)],['월드 시간 / 물리 시간 차이',F&&Number.isFinite(F.physics_elapsed_s)?`${f(F.sim_time_s,3)} / ${f(F.sim_time_s-F.physics_elapsed_s,3)} s`:'—'],['관측 최대 내부 물리 간격',value(F?.physics_max_inner_s===undefined?null:F.physics_max_inner_s*1000,'ms',3)]
 ]));
 $('#session-info').textContent=F?`${F.world_name} · world ${F.source.world_id.slice(0,8)} / run ${F.source.run_id.slice(0,8)}`:'연결된 Play 세션이 없습니다.';
 const contacts=[];
 for(const w of wheels)contacts.push([`바퀴 ${w.id} ↔ 노면`,valid?w.normal_n:null,valid?w.friction_n:null,w.contact_mu,'실제 접촉 μ']);
 for(const t of F?.fork.tines??[])contacts.push([`포크 ${t.id} ↔ 팔레트`,valid?t.normal_n:null,valid?t.friction_n:null,t.contact_mu_config,'설정 Min μ']);
 if(F?.cargo)contacts.push(['팔레트 ↔ 화물',valid?F.cargo.normal_n:null,valid?F.cargo.friction_n:null,F.cargo.static_mu_config,'설정 정지 μ']);
 patchHtml($('#contact-table'),contacts.map(([name,n,t,mu,kind])=>`<tr><td>${name}</td><td>${f(n,2)}</td><td>${f(t,2)}</td><td>${f(mu,3)} <small>${kind}</small></td><td>${Number.isFinite(mu)&&mu*n>1e-6&&Number.isFinite(t)?f(t/(mu*n)*100,1)+' %':'—'}</td></tr>`).join(''));
 const alerts=[];
 if(F?.drive_type==='mecanum'&&F.wheels.some(w=>w.mecanum?.independent_roller_states===false))alerts.push(['경량 메카넘 모델','바퀴별 실제 접촉 하중과 방향성 마찰을 계산합니다. 개별 롤러 상태는 제거한 모델이며 정밀 모델과 같은 계측값으로 취급하지 않습니다.']);
 else if(F?.drive_type==='mecanum')alerts.push(F.bearing_integrator==='symmetric_spin_verlet'?['관절 보존 회귀 수정','1920Hz에서 자유 부유 반작용 시험을 통과했습니다. 접촉 중 무동력 에너지 증가를 막기 위해 기본 간격도 1920Hz로 맞췄습니다. 전체 장비의 접촉·구조·유압 에너지 수지와 실측 물성·정격은 검증 중입니다.']:['이전 적분 모델 수신값','이 수신 프레임에는 새 롤러 적분 모델이 확인되지 않습니다. 최신 Unreal 빌드의 Play 세션과 구별하세요.']);
 if(F&&!valid)alerts.push(['첫 물리 출력 대기','접촉 평균 시간 구간이 유효해진 뒤 하중을 표시합니다.']);
 if(F?.cargo?.fell)alerts.push(['Unreal에서 화물 낙하 감지','실제 물리 상태의 화물 낙하 표시입니다.']);
 if(F?.fork.tines.some(t=>t.failed))alerts.push(['포크 연결 파손','Unreal의 유연 포크 관절이 파손됐습니다. 파단 계수는 추정값이며 실제 부품의 정격 판정이 아닙니다.']);
 if(F?.cargo?.tipped)alerts.push(['Unreal에서 화물 전도 감지','화물 기울기 판정이 활성화됐습니다.']);
 if(F?.cargo?.lost_support)alerts.push(['Unreal에서 화물 지지 상실 감지','충돌·지지 상태를 Unreal에서 확인하세요.']);
 if(F?.has_payload&&valid&&forkTotal<30&&F.pallet.ground_n>100)alerts.push(['화물은 아직 지면에서 지지됩니다','Unreal에서 W로 포크를 삽입한 다음 E로 들어 올리세요.']);
 $('#warnings').innerHTML=alerts.map(([title,text])=>`<div class="warning"><span class="warning-symbol">!</span><div><h3>${title}</h3><p>${text}</p></div></div>`).join('');
 $('#history-chart').innerHTML=drawHistory();$('#history-state').textContent=`${state.history.length}개 수신 표본 · ${mode==='running'?'기록 중':'마지막 기록'}`;
 $('#export-history').disabled=state.history.length===0;
 $('#export-mission').disabled=!state.history.some(e=>e.frame.mission);
 $('#export-handling-control').disabled=!state.history.some(e=>e.frame.handling_control);
 $('#export-pallet-perception').disabled=!state.history.some(e=>e.frame.pallet_perception);
 $('#export-drive-estimator').disabled=!state.history.some(e=>e.frame.drive_estimator);
 $('#export-lidar-safety').disabled=!state.history.some(e=>e.frame.lidar_safety);
 $('#export-lift-control').disabled=!state.history.some(e=>e.frame.lift_control?.mode==='sensor_feedback_direct_valve');
 $('#export-proximity-sensors').disabled=!state.history.some(e=>e.frame.proximity_sensors);
 $('#export-dynamics-sensors').disabled=!state.history.some(e=>e.frame.dynamics_sensors);
 $('#export-joint-sensors').disabled=!state.history.some(e=>e.frame.joint_sensors);
}
function drawScene(F,M){
 if(!F)return '<text x="410" y="195" text-anchor="middle" class="svg-value">Unreal에서 Play를 누르세요</text><text x="410" y="224" text-anchor="middle" class="svg-label">연결 전에는 장비 하중이나 압력을 생성하지 않습니다.</text>';
 const point=(x,y)=>[410+y*125,300-x*83];let out='';
 for(let x=-1.3;x<=3.2;x+=.25){const a=point(x,-2.2),b=point(x,2.2);out+=`<line x1="${a[0]}" y1="${a[1]}" x2="${b[0]}" y2="${b[1]}" stroke="#dfe6d7"/>`;}
 out+='<rect x="245" y="200" width="330" height="200" rx="18" fill="#e5bd6240" stroke="#c2ae74"/><text x="410" y="335" text-anchor="middle" class="svg-mini">차체 기준 · 전방 ↑</text>';
 const max=Math.max(1,...M.wheels.map(w=>w.displayNormal??0));
 for(const w of M.wheels){const [x,y]=point(...[w.local_position_m[0],w.local_position_m[1]]),n=w.displayNormal;out+=`<g transform="translate(${x} ${y}) rotate(${w.steer_deg??0})"><rect x="-15" y="-27" width="30" height="54" rx="8" fill="${n===null?'#bac5b4':n<=0?'#bca48b':`hsl(${142-(n/max)*35} 22% ${66-(n/max)*22}%)`}" stroke="white" stroke-width="2"/><line x1="0" y1="-20" x2="0" y2="20" stroke="#e9f0df" stroke-width="2"/></g><text x="${x+(w.local_position_m[1]<0?-32:32)}" y="${y-5}" text-anchor="${w.local_position_m[1]<0?'end':'start'}" class="svg-label">${w.id}</text><text x="${x+(w.local_position_m[1]<0?-32:32)}" y="${y+15}" text-anchor="${w.local_position_m[1]<0?'end':'start'}" class="svg-value">${f(n===null?null:n/1000,2)} kN</text>`;}
 for(const t of F.fork.tines){const [x,y]=point(t.local_position_m[0],t.local_position_m[1]);out+=`<rect x="${x-7}" y="${y-58}" width="14" height="116" fill="#a6bdb1" stroke="#829e90"/><text x="${x}" y="${y-67}" text-anchor="middle" class="svg-value">${t.id} ${f(F.contact_sample_valid?t.vertical_n/1000:null,2)} kN</text>`;}
 if(F.pallet){const [x,y]=point(F.pallet.local_position_m[0],F.pallet.local_position_m[1]);out+=`<rect x="${x-75}" y="${y-50}" width="150" height="100" fill="#d8ba8b55" stroke="#b9a27c" stroke-dasharray="5 3"/>`;}
 if(F.cargo){const [x,y]=point(F.cargo.local_position_m[0],F.cargo.local_position_m[1]);out+=`<rect x="${x-40}" y="${y-27}" width="80" height="54" fill="#6ca59b" fill-opacity=".7" stroke="#508e84"/><text x="${x}" y="${y+4}" text-anchor="middle" class="svg-label">${f(F.cargo.mass_kg,0)} kg</text>`;}
 out+='<text x="28" y="425" class="svg-mini">실제 위치·휠 방향·접촉력 기반 계측 도식입니다. Unreal 영상 스트리밍은 아닙니다.</text>';return out;
}
function drawHistory(){
 const data=state.history.filter(e=>e.frame.contact_sample_valid).map(e=>({t:e.frame.sim_time_s,...measurements(e.frame)}));
 if(data.length<2)return '<text x="410" y="115" text-anchor="middle" class="svg-label">실제 수신 표본이 쌓이면 그래프가 표시됩니다.</text>';
 const start=data[0].t,end=Math.max(start+.1,data.at(-1).t),leftMax=Math.max(1,...data.flatMap(d=>[d.forkLeft??0,d.forkRight??0]).map(n=>n/1000))*1.1,leftMin=Math.min(0,...data.flatMap(d=>[d.forkLeft??0,d.forkRight??0]).map(n=>n/1000)),rightMax=Math.max(1,...data.map(d=>(d.pressure??0)/1e6))*1.1;
 const x=t=>55+(t-start)/(end-start)*705,y=(v,min,max)=>190-(v-min)/(max-min)*155;let out='';
 for(let i=0;i<=4;i++){const yy=35+i*38.75;out+=`<line x1="55" y1="${yy}" x2="760" y2="${yy}" stroke="#e3e9df"/><text x="45" y="${yy+3}" text-anchor="end" class="svg-mini">${f(leftMax-(leftMax-leftMin)*i/4,1)}</text><text x="770" y="${yy+3}" class="svg-mini">${f(rightMax*(1-i/4),1)}</text>`;}
 for(const [key,color,divisor,max,min]of [['forkLeft','#438e7a',1000,leftMax,leftMin],['forkRight','#d4a34d',1000,leftMax,leftMin],['pressure','#5b8ca5',1e6,rightMax,0]]){
  const points=data.filter(d=>Number.isFinite(d[key])).map(d=>`${x(d.t).toFixed(2)},${y(d[key]/divisor,min,max).toFixed(2)}`).join(' ');out+=`<polyline points="${points}" fill="none" stroke="${color}" stroke-width="1.7"/>`;
 }
 out+=`<text x="55" y="218" class="svg-mini">${f(start,1)} s</text><text x="760" y="218" text-anchor="end" class="svg-mini">${f(end,1)} s</text><text x="18" y="16" class="svg-mini">kN</text><text x="768" y="16" class="svg-mini">MPa</text>`;return out;
}
function selectFeature(id){const f=features.find(f=>f.id===id)||features[0];featureId=f.id;$$('[data-feature]').forEach(el=>el.classList.toggle('selected',el.dataset.feature===f.id));$('#feature-detail').innerHTML=`<div class="feature-detail-top"><div><span class="tiny-label">SYSTEM ${f.num}</span><h2>${f.title}</h2></div><span class="status">${f.status}</span></div><p>${f.body}</p><div class="formula">${f.formula}</div><dl class="feature-params">${f.params.map(([k,v])=>`<div><dt>${k}</dt><dd>${v}</dd></div>`).join('')}</dl><div class="feature-limit"><strong>구현 범위</strong><p>${f.limit}</p><code>${f.source}</code></div>`;}
function route(){const [name,id]=location.hash.slice(1).split('/'),page=['lab','features','validation'].includes(name)?name:'lab';$$('.page').forEach(el=>{const on=el.id===`page-${page}`;el.classList.toggle('active',on);el.hidden=!on;});$$('.nav-link').forEach(el=>{el.classList.toggle('active',el.dataset.page===page);if(el.dataset.page===page)el.setAttribute('aria-current','page');else el.removeAttribute('aria-current');});$('#breadcrumb-current').textContent={lab:'실시간 계측',features:'구현 기능',validation:'검증 기록'}[page];if(page==='features')selectFeature(id||featureId);}
$('#system-nav').innerHTML=features.map(f=>`<a class="system-link" href="#features/${f.id}" data-feature="${f.id}"><i></i>${f.title}</a>`).join('');
$('#feature-grid').innerHTML=features.map(f=>`<button class="feature-card" data-feature="${f.id}"><span class="feature-num">${f.num}</span><div><h2>${f.title}</h2><p>${f.sub}</p><span class="status">${f.status}</span></div><span class="feature-open">↗</span></button>`).join('');
$$('.feature-card').forEach(el=>el.addEventListener('click',()=>{history.replaceState(null,'',`#features/${el.dataset.feature}`);selectFeature(el.dataset.feature);$('#feature-detail').scrollIntoView({behavior:'smooth'});}));
$('#calibration-table').innerHTML=calibration.map(r=>`<tr><td>${r.fps} FPS</td><td>${r.hz} Hz</td><td>${r.speed.toFixed(5)} %</td><td>${r.energy.toFixed(4)} J</td><td><span class="pass">통과</span></td></tr>`).join('');
$('#integrated-table').innerHTML=integrated.map(([name,stop,mean,inner])=>`<tr><td>${name}</td><td>${stop.toFixed(5)} m</td><td>${mean.toFixed(3)} kN</td><td>${inner.toFixed(3)} kN</td><td><span class="pass">통과</span></td></tr>`).join('');
$('#show-model').addEventListener('click',()=>{$('#model-details').open=true;$('#model-details').scrollIntoView({behavior:'smooth'});});
$('#clear-history').addEventListener('click',()=>{state.clear();render();});
for(const mode of ['raw','filtered'])$(`#load-${mode}`).addEventListener('click',()=>{filtered=mode==='filtered';for(const key of ['raw','filtered']){$(`#load-${key}`).classList.toggle('selected',key===mode);$(`#load-${key}`).setAttribute('aria-pressed',String(key===mode));}render();});
$('#export-history').addEventListener('click',()=>{
 const columns=['received_at_ms','world_id','run_id','sequence','sim_time_s','contact_valid','fork_height_m','fork_left_n','fork_right_n','pressure_pa','speed_mps','FL_normal_n','FR_normal_n','RL_normal_n','RR_normal_n'];
 const rollerCount=Math.max(0,...state.history.flatMap(e=>e.frame.wheels.map(w=>w.mecanum?.rollers?.length??0)));
 const rollerFields=['normal_n','relative_omega_rad_s','bearing_torque_nm'];columns.push('drive_type');
 for(const id of ['FL','FR','RL','RR'])for(let i=0;i<rollerCount;i++)for(const key of rollerFields)columns.push(`${id}_roller_${i}_${key}`);
 columns.push('actual_realtime_ratio','physics_clock_lag_s','auto_hold','drive_update','shaft_joint_work_j','shaft_loss_j');
 for(const id of ['FL','FR','RL','RR'])columns.push(`${id}_contact_axis_slip_rms_mps`,`${id}_contact_cross_slip_rms_mps`);
 columns.push('bearing_integrator','physics_selected_step_s','physics_smallest_observed_s','physics_inner_clock_error_s','bearing_transport_work_j');
 columns.push('chain_damping_j',...['FL','FR','RL','RR'].flatMap(id=>[`${id}_catalogue_load_comparison_ratio`,`${id}_physical_wheel_mass_kg`,`${id}_inner_peak_load_n`]));
 const auditFields=['reduced_contact_loss_j','plastic_relaxation_j','fracture_release_j','contact_normal_work_estimate_j','contact_tangent_work_estimate_j','contact_correction_pseudo_j','fork_damping_estimate_j'];
 columns.push(...auditFields,'rack_support_n','rack_trial_stage');
 const rows=state.history.map(e=>{const F=e.frame,M=measurements(F),rollerValues=[];
 for(const id of ['FL','FR','RL','RR'])for(let i=0;i<rollerCount;i++)for(const key of rollerFields)rollerValues.push(F.wheels.find(w=>w.id===id)?.mecanum?.rollers?.[i]?.[key]??null);
 return [e.received_at_ms,F.source.world_id,F.source.run_id,F.source.sequence,F.sim_time_s,F.contact_sample_valid,M.height,M.forkLeft,M.forkRight,M.pressure,M.speed,...['FL','FR','RL','RR'].map(id=>M.wheels.find(w=>w.id===id)?.displayNormal??null),F.drive_type??'',...rollerValues,F.actual_realtime_ratio,F.physics_clock_lag_s,F.controls?.auto_hold,F.drive_update,F.energy_audit?.reference_valid?F.energy_audit.shaft_joint_work_j:null,F.energy_audit?.reference_valid?F.energy_audit.shaft_loss_j:null,...['FL','FR','RL','RR'].flatMap(id=>{const w=F.wheels.find(w=>w.id===id);return [w?.long_slip_mps,w?.lateral_slip_mps];}),F.bearing_integrator,F.physics_selected_step_s,F.physics_smallest_observed_s,F.physics_inner_clock_error_s,F.energy_audit?.reference_valid?F.energy_audit.bearing_transport_work_j:null,F.energy_audit?.reference_valid?F.energy_audit.chain_damping_j:null,...['FL','FR','RL','RR'].flatMap(id=>{const m=F.wheels.find(w=>w.id===id)?.mecanum;return [F.contact_sample_valid?m?.catalogue_load_comparison_ratio:null,m?.physical_wheel_mass_kg,F.contact_sample_valid?m?.contact_inner_peak_n:null];}),...auditFields.map(k=>F.energy_audit?.reference_valid?F.energy_audit[k]:null),F.contact_sample_valid?F.handling_rack?.support_n:null,F.handling_rack?.trial_stage].map(v=>v??'').join(',');});
 const url=URL.createObjectURL(new Blob(['\ufeff'+[columns.join(','),...rows].join('\r\n')],{type:'text/csv;charset=utf-8'})),a=document.createElement('a');a.href=url;a.download=`ForkliftSim-live-${Date.now()}.csv`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
});
fetch('/api/history').then(r=>r.json()).then(data=>{state.hydrate(data.frames);schedule();}).catch(error=>console.warn('Real history restore unavailable',error));
const stream=new EventSource('/events');stream.onopen=()=>{connection='connected';schedule();};stream.onerror=()=>{connection='disconnected';schedule();};
stream.addEventListener('frame',event=>{try{if(state.accept(JSON.parse(event.data)))schedule();}catch{connection='disconnected';schedule();}});
stream.addEventListener('status',()=>{connection='connected';statusView();});
setInterval(statusView,400);addEventListener('hashchange',route);route();render();

$('#export-joint-sensors').addEventListener('click',()=>{
 const {csv}=jointSensorCsv(state.history);const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download='ForkliftSim-joint-sensors.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
});

$('#export-dynamics-sensors').addEventListener('click',()=>{
 const {csv}=dynamicsSensorCsv(state.history);const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download='ForkliftSim-dynamics-sensors.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
});

$('#export-proximity-sensors').addEventListener('click',()=>{
 const {csv}=proximitySensorCsv(state.history);const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download='ForkliftSim-proximity-sensors.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
});

$('#export-lift-control').addEventListener('click',()=>{
 const {csv}=liftControlCsv(state.history);const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download='ForkliftSim-lift-control.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
});

$('#export-pallet-perception').addEventListener('click',()=>{const {csv}=palletPerceptionCsv(state.history);const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download='ForkliftSim-pallet-perception.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
$('#export-drive-estimator').addEventListener('click',()=>{const {csv}=driveEstimatorCsv(state.history);const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download='ForkliftSim-drive-estimator.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
$('#export-lidar-safety').addEventListener('click',()=>{const {csv}=lidarSafetyCsv(state.history);const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download='ForkliftSim-lidar-safety.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});

$('#export-handling-control').addEventListener('click',()=>{const {csv}=handlingControlCsv(state.history);const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download='ForkliftSim-handling-control.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});

$('#export-mission').addEventListener('click',()=>{const {csv}=missionCsv(state.history);const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download='ForkliftSim-mission.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
