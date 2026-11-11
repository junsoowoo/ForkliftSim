export function liftControlCsv(history){
 const fields=['mode','state','fault_reason','armed','fault_latched','requested_m','target_m','measured_height_m','measured_pressure_pa','estimated_velocity_mps','command_spool','pump_enabled','ack_rejected','target_rebased','command_lease_s'];
 const columns=['received_at_ms','world_id','run_id','sim_time_s',...fields],rows=[];
 for(const e of history){const f=e.frame,c=f?.lift_control;if(c?.mode!=='sensor_feedback_direct_valve')continue;rows.push([e.received_at_ms,f.source.world_id,f.source.run_id,f.sim_time_s,...fields.map(k=>c[k])].map(v=>v??'').join(','));}
 return {columns,rows,csv:columns.join(',')+'\n'+rows.join('\n')+'\n'};
}
