export const missionNames={idle:'임무 대기',to_pickup:'인양 구역으로 주행',request_pickup:'인양 요청',pickup:'인식·삽입·인양·후퇴',retry_pickup:'안전한 삽입 전 인식 재시도',to_dropoff:'하역 구역으로 운반',request_dropoff:'하역 요청',dropoff:'랙 하역·후퇴',complete:'임무 완료',paused:'일시정지 · 명시적 재개 필요',blocked:'경로 차단 · 재개 전 확인 필요',fault_latched:'고장 정지 · 정상 수신/확인 필요',cancelled:'임무 취소 · Y로 수동 복귀'};
export function missionCsv(history){
 const keys=['state','reason','checkpoint','engaged','high','carrying','pickup_receipt','dropoff_receipt','sensors_healthy','localized','localization_age_s','measured_x_m','measured_y_m','measured_yaw_rad','height_target_m','vx_command_mps','vy_command_mps','yaw_command_rad_s','brake','hold_lift','retries','replans','waypoint','path_length_m','dynamic_cells','planner_ms'];
 const columns=['received_at_ms','world_id','run_id','sim_s',...keys],quote=v=>`"${String(v??'').replaceAll('"','""')}"`,rows=[];
 for(const e of history){const f=e.frame,m=f?.mission;if(!m)continue;rows.push([e.received_at_ms,f.source.world_id,f.source.run_id,f.sim_time_s,...keys.map(k=>!m.localized&&['measured_x_m','measured_y_m','measured_yaw_rad'].includes(k)?null:m[k])].map(quote).join(','));}
 return {columns,rows,csv:columns.join(',')+'\n'+rows.join('\n')+'\n'};
}
export function missionRouteSvg(m){
 if(!m)return '';const pair=p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite),path=(m.path_m??[]).filter(pair),poses=[m.pickup_approach_m,m.dropoff_approach_m,...path].filter(pair);
 if(m.localized&&Number.isFinite(m.measured_x_m)&&Number.isFinite(m.measured_y_m))poses.push([m.measured_x_m,m.measured_y_m]);if(!poses.length)return '';
 const minX=Math.min(...poses.map(p=>p[0]))-1,maxX=Math.max(...poses.map(p=>p[0]))+1,minY=Math.min(...poses.map(p=>p[1]))-1,maxY=Math.max(...poses.map(p=>p[1]))+1;
 const px=p=>40+(p[0]-minX)/(maxX-minX)*440,py=p=>28+(p[1]-minY)/(maxY-minY)*175;let out='<svg viewBox="0 0 520 235" role="img" aria-label="센서 위치 추정과 실제 계획 경로"><rect x="1" y="1" width="518" height="233" rx="8" fill="#f4f7f1"/>';
 if(path.length>1)out+=`<polyline points="${path.map(p=>`${px(p)},${py(p)}`).join(' ')}" fill="none" stroke="#438e7a" stroke-width="3"/>`;
 for(const [p,name,color] of [[m.pickup_approach_m,'PICK','#cb9c45'],[m.dropoff_approach_m,'DROP','#578ba5']])if(pair(p))out+=`<circle cx="${px(p)}" cy="${py(p)}" r="6" fill="${color}"/><text x="${px(p)+9}" y="${py(p)-8}" font-size="11" fill="#394a40">${name}</text>`;
 if(m.localized&&Number.isFinite(m.measured_x_m)&&Number.isFinite(m.measured_y_m)){const p=[m.measured_x_m,m.measured_y_m];out+=`<circle cx="${px(p)}" cy="${py(p)}" r="5" fill="#252e2a"/>`;if(Number.isFinite(m.measured_yaw_rad))out+=`<line x1="${px(p)}" y1="${py(p)}" x2="${px(p)+13*Math.cos(m.measured_yaw_rad)}" y2="${py(p)+13*Math.sin(m.measured_yaw_rad)}" stroke="#252e2a" stroke-width="2"/>`;}
 return out+'<text x="15" y="222" font-size="10" fill="#56645d">센서 좌표·계획 경로 / 지도 전체나 Unreal 영상이 아닙니다.</text></svg>';
}
