export const handlingNames={idle:'수동 대기',pick_height:'삽입 높이 맞추기',acquire:'팔레트 인식 대기',align:'자동 정렬',insert:'자동 삽입',pick_lift:'완전 삽입 확인 후 인양',pick_withdraw:'적재 후 후퇴',lower_to_carry:'운반 높이로 이동',place_raise:'랙 하역 높이로 상승',place_approach:'등록된 랙 위치로 접근',place_lower:'하중 전달 확인',place_withdraw:'포크 후퇴',complete:'작업 완료',fault_latched:'작업 정지 · 새 요청 필요'};
export function handlingControlCsv(history){
 const fields=['state','reason','active','fault_latched','place','sensors_healthy','full_seen','load_receipt','transfer_seen','tier_m','target_height_m','vx_command_mps','vy_command_mps','yaw_command_rad_s','brake','target_permission'];
 const columns=['received_at_ms','world_id','run_id','sim_s',...fields];const quote=v=>`"${String(v??'').replaceAll('"','""')}"`;
 const rows=[];for(const e of history){const f=e.frame,h=f?.handling_control;if(!h)continue;rows.push([e.received_at_ms,f.source.world_id,f.source.run_id,f.sim_time_s,...fields.map(k=>h[k])].map(quote).join(','));}
 return {columns,rows,csv:columns.join(',')+'\n'+rows.join('\n')+'\n'};
}
