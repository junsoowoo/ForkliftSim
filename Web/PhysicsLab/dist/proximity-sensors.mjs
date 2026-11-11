// Real received packets only. Clear presence is valid OFF; no range return is
// an invalid distance in a fresh packet. Never fill either with physics truth.
export function proximitySensorCsv(history){
 const columns=['received_at_ms','world_id','run_id','sim_time_s','id','kind','status','transport_valid','measurement_valid','reading_status','sequence','capture_sim_s','delivery_sim_s','age_s','range_m','detected','dropped_packets'];
 const rows=[];
 for(const e of history){const f=e.frame;if(!f?.proximity_sensors)continue;
  for(const s of f.proximity_sensors.sensors??[]){const valid=s.transport_valid===true&&s.measurement_valid===true&&s.status==='valid'&&s.reading_status==='valid';
   const cells=[e.received_at_ms,f.source.world_id,f.source.run_id,f.sim_time_s,s.id,s.kind,s.status,s.transport_valid,s.measurement_valid,s.reading_status,s.sequence,s.capture_sim_s,s.delivery_sim_s,s.age_s,valid&&s.kind==='range'?s.range_m??'':'',valid&&s.kind==='presence'?s.detected??'':'',s.dropped_packets];rows.push(cells.map(v=>v??'').join(','));
  }
 }
 return {columns,rows,csv:columns.join(',')+'\n'+rows.join('\n')+'\n'};
}
