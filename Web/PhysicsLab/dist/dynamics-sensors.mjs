// Received measurements only, never physics ground-truth fallbacks.
export const dynamicsFields=['specific_force_x_mps2','specific_force_y_mps2','specific_force_z_mps2','gyro_x_rad_s','gyro_y_rad_s','gyro_z_rad_s','gauge_pressure_pa','fork_axis_force_n'];
export function dynamicsSensorCsv(history){
 const columns=['received_at_ms','world_id','run_id','sim_time_s','id','status','valid','sequence','capture_sim_s','delivery_sim_s','age_s',...dynamicsFields,'clipped_mask','dropped_packets'];
 const rows=[];
 for(const e of history){const f=e.frame;if(!f?.dynamics_sensors)continue;
  for(const s of f.dynamics_sensors.sensors??[]){const valid=s.valid===true&&s.status==='valid';
   const cells=[e.received_at_ms,f.source.world_id,f.source.run_id,f.sim_time_s,s.id,s.status,valid,s.sequence,s.capture_sim_s,s.delivery_sim_s,s.age_s,...dynamicsFields.map(k=>valid?s[k]??'':''),valid?s.clipped_mask:'',s.dropped_packets];
   rows.push(cells.map(v=>v??'').join(','));
  }
 }
 return {columns,rows,csv:columns.join(',')+'\n'+rows.join('\n')+'\n'};
}
