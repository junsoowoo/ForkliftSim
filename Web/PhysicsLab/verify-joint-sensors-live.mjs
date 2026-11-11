import assert from 'node:assert/strict';
import {writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {jointSensorCsv} from './dist/joint-sensors.mjs';
const base=process.argv[2]??'http://127.0.0.1:4174',earliest=Number(process.argv[3]??Date.now());
const frames=[],seen=new Set();let complete=false;
const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),300000);
function accept(e){const f=e.frame;if(f?.source?.kind!=='unreal'||f.source.started_at_ms<earliest||f.scenario!=='mecanum_joint_sensors')return;
 const key=f.source.world_id+':'+f.source.sequence;if(seen.has(key))return;seen.add(key);frames.push(e);if(f.source.state==='stopped')complete=true;}
try{
 for(const e of (await(await fetch(base+'/api/history')).json()).frames)accept(e);
 const response=await fetch(base+'/events',{signal:abort.signal}),reader=response.body.getReader(),decoder=new TextDecoder();let pending='';
 while(!complete){const {value,done}=await reader.read();if(done)break;pending+=decoder.decode(value,{stream:true});let end;
  while((end=pending.indexOf('\n\n'))>=0){const event=pending.slice(0,end);pending=pending.slice(end+2);if(event.startsWith('event: frame\n'))accept(JSON.parse(event.slice(event.indexOf('data: ')+6)));}}
 await reader.cancel();
}finally{clearTimeout(timer);}
assert(complete&&frames.length>100,'Incomplete native sensor stream');let stale=0,recovery=0,heightDifferent=0;
for(const {frame:f} of frames){const j=f.joint_sensors;assert.equal(j.source,'simulated_physical_joint_sensors_v1');assert.equal(j.controller_connected,false);assert.equal(j.safety_rated,false);assert.equal(j.encoders.length,4);assert(!JSON.stringify(j).includes('truth_'));
 const sensors=[...j.encoders,j.fork_height,j.fork_limits];
 for(const s of sensors){if(s.valid){assert.equal(s.status,'valid');assert(s.delivery_sim_s-s.capture_sim_s+1e-8>=s.latency_s);assert(s.age_s<=s.stale_s+1e-8);}else for(const k of ['counts','position_rad','angular_velocity_rad_s','height_m','lower_active','upper_active'])if(k in s)assert.equal(s[k],null);}
 if(f.sim_time_s>6.4&&f.sim_time_s<8){stale++;assert(sensors.every(s=>s.status==='stale'&&!s.valid));}
 if(f.sim_time_s>8.15&&f.sim_time_s<8.5){recovery++;assert(sensors.every(s=>s.valid));}
 if(j.fork_height.valid&&Math.abs(j.fork_height.height_m-f.fork.height_m)>1e-5)heightDifferent++;
}
assert(stale>3&&recovery>1&&heightDifferent>10,'Missing actual outage/recovery or independently measured height');
const result=jointSensorCsv(frames);assert.equal(result.columns.length,19);for(const row of result.rows)assert.equal(row.split(',').length,19);
await writeFile(new URL('../../Saved/Validation/JointSensors-WebExport.csv',import.meta.url),result.csv);
const raw=frames.map(e=>JSON.stringify(e)).join('\n')+'\n';await writeFile(new URL('../../Saved/Validation/JointSensors-Live.jsonl',import.meta.url),raw);
const report={pass:true,actual_frames:frames.length,stale_frames:stale,recovery_frames:recovery,csv_columns:result.columns.length,csv_rows:result.rows.length,csv_sha256:createHash('sha256').update(result.csv).digest('hex'),stream_sha256:createHash('sha256').update(raw).digest('hex'),scope:'Actual received native sensor stream and shared UI CSV exporter. DOM click not automated. Truth is not used as a fallback. Controller remains disconnected.'};
await writeFile(new URL('../../Docs/References/JointSensorsWeb-20261031.json',import.meta.url),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report));
