export function lidarSafetyCsv(history){
 const columns=['received_at_ms','world_id','run_id','sim_s','state','reason','drive_hold','lift_hold','fault_latched','obstacle','clearance_m','protected_points','front_sequence','rear_sequence','front_fresh','rear_fresh','front_capture_s','rear_capture_s'];
 const rows=[];for(const e of history){const f=e.frame,g=f?.lidar_safety;if(!g)continue;const [a,b]=g.scanners??[];rows.push([e.received_at_ms,f.source.world_id,f.source.run_id,f.sim_time_s,g.state,g.reason,g.drive_hold,g.lift_hold,g.fault_latched,g.obstacle,g.data_healthy?g.clearance_m:null,g.protected_points,a?.sequence,b?.sequence,a?.fresh,b?.fresh,a?.capture_sim_s,b?.capture_sim_s].map(v=>v??'').join(','));}return {columns,rows,csv:columns.join(',')+'\n'+rows.join('\n')+'\n'};
}
export function lidarPoints(bank){
 const points=[];for(const s of bank?.scanners??[]){if(!s.fresh)continue;const m=s.calibrated_mount_chassis_m;if(!Array.isArray(m))continue;for(let k=0;k<s.ranges_m.length;k++){const r=s.ranges_m[k];if(s.return_codes[k]!==0||!Number.isFinite(r))continue;const a=(s.start_deg+s.fov_deg*k/(s.ranges_m.length-1))*Math.PI/180;points.push({x:m[0]+Math.cos(a)*r,y:m[1]+Math.sin(a)*r,id:s.id});}}return points;
}
