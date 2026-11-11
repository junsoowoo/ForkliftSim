import assert from 'node:assert/strict';
import {writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {liftControlCsv} from './dist/lift-control.mjs';
const base=process.argv[2]??'http://127.0.0.1:4174',earliest=Number(process.argv[3]??Date.now()),frames=[],seen=new Set();let complete=false;
const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),300000);
function accept(e){const f=e.frame;if(f?.source?.kind!=='unreal'||f.source.started_at_ms<earliest||f.scenario!=='mecanum_lift_sensor_fault_up')return;const key=f.source.world_id+':'+f.source.sequence;if(seen.has(key))return;seen.add(key);frames.push(e);if(f.source.state==='stopped')complete=true;}
try{for(const e of (await(await fetch(base+'/api/history')).json()).frames)accept(e);const response=await fetch(base+'/events',{signal:abort.signal}),reader=response.body.getReader(),decoder=new TextDecoder();let pending='';
 while(!complete){const {value,done}=await reader.read();if(done)break;pending+=decoder.decode(value,{stream:true});let end;while((end=pending.indexOf('\n\n'))>=0){const event=pending.slice(0,end);pending=pending.slice(end+2);if(event.startsWith('event: frame\n'))accept(JSON.parse(event.slice(event.indexOf('data: ')+6)));}}await reader.cancel();
}finally{clearTimeout(timer);}
assert(complete&&frames.length>100);let fault=0,recoveredButLatched=0,acknowledged=0,invalidHeight=0;
for(const {frame:f} of frames){const c=f.lift_control;assert.equal(c.mode,'sensor_feedback_direct_valve');assert.equal(c.uses_physics_position_feedback,false);assert.equal(c.safety_rated,false);assert(f.joint_sensors.lift_controller_connected&&f.dynamics_sensors.lift_controller_connected);assert.equal(f.proximity_sensors.controller_connected,false);
 if(c.fault_latched){fault++;assert.equal(c.command_spool,0);assert.equal(c.pump_enabled,false);}
 if(f.sim_time_s>8.5&&f.sim_time_s<13.8){recoveredButLatched++;assert(c.fault_latched&&f.joint_sensors.fork_height.valid);}
 if(f.sim_time_s>14.3&&f.sim_time_s<17.7){acknowledged++;assert(!c.fault_latched&&c.target_m<.8);}
 if(!f.joint_sensors.fork_height.valid){invalidHeight++;assert.equal(c.measured_height_m,null);}
}
assert(fault>20&&recoveredButLatched>10&&acknowledged>10&&invalidHeight>5);
const csv=liftControlCsv(frames);assert.equal(csv.columns.length,19);for(const r of csv.rows)assert.equal(r.split(',').length,19);
const raw=frames.map(e=>JSON.stringify(e)).join('\n')+'\n';await writeFile(new URL('../../Saved/Validation/SensorLift-Live.jsonl',import.meta.url),raw);await writeFile(new URL('../../Saved/Validation/SensorLift-WebExport.csv',import.meta.url),csv.csv);
const report={date:'2026-11-05',pass:true,actual_frames:frames.length,fault_frames:fault,fresh_but_latched_frames:recoveredButLatched,acknowledged_hold_frames:acknowledged,invalid_height_frames:invalidHeight,csv_columns:19,csv_rows:csv.rows.length,csv_sha256:createHash('sha256').update(csv.csv).digest('hex'),stream_sha256:createHash('sha256').update(raw).digest('hex'),scope:'Real Unreal sensor lift controller stream and shared CSV exporter; no DOM click automation or full vehicle autonomy.'};await writeFile(new URL('../../Docs/References/SensorLiftWeb-20261105.json',import.meta.url),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report));
