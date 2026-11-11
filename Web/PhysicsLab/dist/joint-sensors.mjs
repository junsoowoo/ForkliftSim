// Received sensor outputs only. Never substitute vehicle velocity or true fork
// height when an encoder/transducer sample is absent or stale.
export function jointSensorCsv(history){
 const columns=['received_at_ms','world_id','run_id','sim_time_s','id','status','valid','sequence','capture_sim_s','delivery_sim_s','age_s','counts','position_rad','angular_velocity_rad_s','nominal_surface_mps','height_m','lower_active','upper_active','dropped_packets'];
 const rows=[];
 for(const e of history){const f=e.frame,j=f?.joint_sensors;if(!j)continue;
  for(const s of [...(j.encoders??[]),j.fork_height,j.fork_limits].filter(Boolean)){
   const valid=s.valid===true&&s.status==='valid';const measured=k=>valid?s[k]??'':'';
   const values=[e.received_at_ms,f.source.world_id,f.source.run_id,f.sim_time_s,s.id,s.status,valid,s.sequence,s.capture_sim_s,s.delivery_sim_s,s.age_s,...['counts','position_rad','angular_velocity_rad_s','nominal_surface_mps','height_m','lower_active','upper_active'].map(measured),s.dropped_packets];
   rows.push(values.map(v=>v??'').join(','));
  }
 }
 return {columns,rows,csv:columns.join(',')+'\n'+rows.join('\n')+'\n'};
}
