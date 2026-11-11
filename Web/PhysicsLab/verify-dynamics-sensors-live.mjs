import assert from 'node:assert/strict';
import {writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {dynamicsSensorCsv,dynamicsFields} from './dist/dynamics-sensors.mjs';
const base=process.argv[2]??'http://127.0.0.1:4174',earliest=Number(process.argv[3]??Date.now());
const frames=[],seen=new Set();let complete=false;
const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),600000);
function accept(e){const f=e.frame;if(f?.source?.kind!=='unreal'||f.source.started_at_ms<earliest||f.scenario!=='mecanum_carry')return;
 const key=f.source.world_id+':'+f.source.sequence;if(seen.has(key))return;seen.add(key);frames.push(e);if(f.source.state==='stopped')complete=true;}
try{
 for(const e of (await(await fetch(base+'/api/history')).json()).frames)accept(e);
 const response=await fetch(base+'/events',{signal:abort.signal}),reader=response.body.getReader(),decoder=new TextDecoder();let pending='';
 while(!complete){const {value,done}=await reader.read();if(done)break;pending+=decoder.decode(value,{stream:true});let end;
  while((end=pending.indexOf('\n\n'))>=0){const event=pending.slice(0,end);pending=pending.slice(end+2);if(event.startsWith('event: frame\n'))accept(JSON.parse(event.slice(event.indexOf('data: ')+6)));}}
 await reader.cancel();
}finally{clearTimeout(timer);}
assert(complete&&frames.length>100,'Incomplete actual Unreal stream');let stale=0,recovery=0;
for(const {frame:f} of frames){const bank=f.dynamics_sensors;assert.equal(bank.source,'simulated_dynamics_sensors_v1');assert.equal(bank.controller_connected,false);assert.equal(bank.safety_rated,false);assert.equal(bank.sensors.length,4);assert(!JSON.stringify(bank).includes('truth_'));assert.equal(bank.queue_overflows,0);
 for(const s of bank.sensors){if(s.valid){assert.equal(s.status,'valid');assert(s.delivery_sim_s-s.capture_sim_s+1e-8>=s.latency_s);assert(s.age_s>=0&&s.age_s<=s.stale_s+1e-8);}else for(const k of dynamicsFields)if(k in s)assert.equal(s[k],null);}
 if(f.sim_time_s>6.4&&f.sim_time_s<8){stale++;assert(bank.sensors.every(s=>s.status==='stale'&&!s.valid));}
 if(f.sim_time_s>8.2&&f.sim_time_s<8.6){recovery++;assert(bank.sensors.every(s=>s.valid));}
}
assert(stale>3&&recovery>1,'Missing outage/recovery');
const exported=dynamicsSensorCsv(frames);assert.equal(exported.columns.length,21);for(const r of exported.rows)assert.equal(r.split(',').length,21);
const raw=frames.map(e=>JSON.stringify(e)).join('\n')+'\n';
await writeFile(new URL('../../Saved/Validation/DynamicsSensors-Live.jsonl',import.meta.url),raw);
await writeFile(new URL('../../Saved/Validation/DynamicsSensors-WebExport.csv',import.meta.url),exported.csv);
const report={date:'2026-11-05',pass:true,actual_frames:frames.length,stale_frames:stale,recovery_frames:recovery,csv_columns:exported.columns.length,csv_rows:exported.rows.length,csv_sha256:createHash('sha256').update(exported.csv).digest('hex'),stream_sha256:createHash('sha256').update(raw).digest('hex'),scope:'Actual Unreal receive stream and the UI shared CSV exporter; no truth fallback, no DOM click automation.'};
await writeFile(new URL('../../Docs/References/DynamicsSensorsWeb-20261105.json',import.meta.url),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report));
