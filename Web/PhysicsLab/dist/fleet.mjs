// Presentation of Unreal fleet frames. No simulator or generated measurements.
export const selectVehicle=(frame,id)=>{
 if(!frame?.fleet)return frame;
 const v=id?frame.fleet.vehicles.find(v=>v.vehicle_id===id):frame.fleet.vehicles[0];
 return v?{...v,source:frame.source,fleet:frame.fleet}:null;
};
export const vehicleHistory=(history,id)=>history.map(e=>({...e,frame:selectVehicle(e.frame,id)})).filter(e=>e.frame);
export function fleetCsv(history){
 const columns=['received_at_ms','world_id','sequence','vehicle_id','sim_s','state','reason','load_id','destination','localized','map_x_m','map_y_m','speed_mps','height_m','left_fork_n','right_fork_n','pressure_pa','waiting_jobs','wheel_vx_mps','wheel_vy_mps','scan_vx_mps','scan_vy_mps','scan_velocity_fresh','battery_soc','battery_stored_wh','battery_consumed_wh','battery_charged_wh','battery_power_w','battery_charging'];
 const energy=(e,v)=>e.frame.fleet?.task_board?.energy?.vehicles?.find(b=>b.vehicle_id===v.vehicle_id);
 const quote=x=>'"'+String(x??'').replaceAll('"','""')+'"';
 const rows=history.flatMap(e=>(e.frame.fleet?.vehicles??[]).map(v=>[e.received_at_ms,e.frame.source.world_id,e.frame.source.sequence,v.vehicle_id,v.sim_time_s,v.campus_job?.state,v.campus_job?.reason,v.target_load_id,v.campus_job?.destination,v.drive_estimator?.localized,v.drive_estimator?.map_x_m,v.drive_estimator?.map_y_m,v.vehicle?.speed_mps,v.fork?.height_m,v.fork?.tines?.[0]?.vertical_n,v.fork?.tines?.[1]?.vertical_n,v.hydraulic?.pressure_pa,e.frame.fleet.waiting_jobs,v.drive_estimator?.vx_mps,v.drive_estimator?.vy_mps,v.drive_estimator?.ground_vx_mps,v.drive_estimator?.ground_vy_mps,v.drive_estimator?.map_velocity_fresh,energy(e,v)?.soc,energy(e,v)?.stored_wh,energy(e,v)?.consumed_wh,energy(e,v)?.charged_wh,energy(e,v)?.power_w,energy(e,v)?.charging]));
 return [columns,...rows].map(r=>r.map(quote).join(',')).join('\n');
}
const esc=x=>String(x??'—').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const n=x=>Number.isFinite(x)?x.toFixed(2):'—';
export function fleetPanel(frame,selected,live=true){
 const f=frame?.fleet;if(!f)return '<p>창고 다중 차량 데이터 수신 대기. 기존 단일 차량 세션은 아래 패널에 표시합니다.</p>';
 const colors=['#087e8b','#c7522a','#8b5fbf','#137547','#ac6500','#4159b5'];
 let svg='<rect x="0" y="0" width="100" height="50" fill="#eef4f5" stroke="#677e89" stroke-width=".2"/><rect x="-40" y="0" width="40" height="50" fill="#dce4e8"/>';
 svg+=f.slots.filter(s=>s.center_m[2]<1).map(s=>`<rect x="${s.center_m[0]-.55}" y="${s.center_m[1]-.55}" width="1.1" height="1.1" fill="${s.blocked?'#ac3540':s.pallet_id?'#a17b51':s.reserved_by?'#dc992b':'#bcc9cb'}"/>`).join('');
 for(const c of f.traffic_claims)for(const b of c.bounds_m??[])svg+=`<rect x="${b[0]}" y="${b[1]}" width="${b[2]-b[0]}" height="${b[3]-b[1]}" fill="none" stroke="${c.quarantined?'#ac3540':'#8aaeb4'}" stroke-dasharray=".5 .5" stroke-width=".12"><title>${esc(c.owner)} · ${c.quarantined?'격리':'예약'}</title></rect>`;
 for(const c of f.task_board?.energy?.chargers??[])svg+=`<rect x="${c.position_m[0]-1}" y="${c.position_m[1]-1}" width="2" height="2" fill="${c.owner?'#dc992b':'#b1d7ba'}"><title>${esc(c.id)} · ${esc(c.owner||'충전 위치')}</title></rect>`;
 const cards=f.vehicles.map((v,i)=>{
  const d=v.drive_estimator,j=v.campus_job,col=colors[i%colors.length],b=f.task_board?.energy?.vehicles?.find(b=>b.vehicle_id===v.vehicle_id);
  if(live&&d?.localized&&Number.isFinite(d.map_x_m)&&Number.isFinite(d.map_y_m)){
   svg+=`<polyline points="${(j?.path_m??[]).map(p=>p.join(',')).join(' ')}" fill="none" stroke="${col}" stroke-width=".25" opacity=".65"/>`;
   svg+=`<g transform="translate(${d.map_x_m} ${d.map_y_m})"><circle r="1" fill="${col}"/><title>${esc(v.vehicle_id)} · ${esc(j?.state)}</title><path d="M0 0L${2*Math.cos(d.map_yaw_rad)} ${2*Math.sin(d.map_yaw_rad)}" stroke="${col}" stroke-width=".35"/></g>`;
  }
  return `<button class="fleet-card" data-fleet-select="${esc(v.vehicle_id)}" aria-pressed="${selected===v.vehicle_id}"><b style="color:${col}">${esc(v.vehicle_id)}</b> · ${esc(j?.state)}<br>${esc(j?.reason||'측정 중')}<br>속도 ${n(v.vehicle?.speed_mps)} m/s · 포크 ${n(v.fork?.height_m)} m · 대기 ${n(j?.waiting_s)} s<br>${b?`배터리 ${n(b.soc*100)}% · ${n(b.power_w)} W · ${esc(b.mode)}${j?.kind==='charge'?' · 충전 작업 '+esc(j.charger_id):''}<br>`:''}${!live?'중단/오래된 마지막 표본 · 지도 위치 표시 중지':d?.localized?'지도 정합 유효':'위치 정합 상실 · 지도 위치 표시 중지'}</button>`;
 }).join('');
 return `<div class="fleet-cards">${cards}</div><svg class="fleet-map" viewBox="-41 -2 143 55" role="img" aria-label="수신한 차량 추정 위치와 실제 재고 상태">${svg}</svg><p>${f.task_board?.traffic_policy==='section_windows'?'구간 예약':'작업 전체 예약'} · 대기 ${f.waiting_jobs}건 · 예약 ${f.traffic_claims.length}건 · 격리 ${f.traffic_claims.filter(c=>c.quarantined).length}건. 경로·위치는 센서 추정값, 슬롯 상태는 시뮬레이터 재고 기록입니다. 웹 차량 선택은 표시만 변경합니다.</p><p>${(f.traffic_waits??[]).map(w=>esc(w.vehicle_id)+' → '+esc(w.blocked_by)).join(' · ')}${f.deadlock_vehicles?.length?' · 교착 보호 정지: '+f.deadlock_vehicles.map(esc).join(', '):''}</p>`;
}
