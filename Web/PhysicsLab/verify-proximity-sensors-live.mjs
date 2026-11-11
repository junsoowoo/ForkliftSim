import assert from 'node:assert/strict';
import {writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {proximitySensorCsv} from './dist/proximity-sensors.mjs';
const base=process.argv[2]??'http://127.0.0.1:4174',earliest=Number(process.argv[3]??Date.now());
const frames=[],seen=new Set();let complete=false;const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),300000);
function accept(e){const f=e.frame;if(f?.source?.kind!=='unreal'||f.source.started_at_ms<earliest||f.scenario!=='mecanum_carry')return;const key=f.source.world_id+':'+f.source.sequence;if(seen.has(key))return;seen.add(key);frames.push(e);if(f.source.state==='stopped')complete=true;}
try{
 for(const e of (await(await fetch(base+'/api/history')).json()).frames)accept(e);
 const response=await fetch(base+'/events',{signal:abort.signal}),reader=response.body.getReader(),decoder=new TextDecoder();let pending='';
 while(!complete){const {value,done}=await reader.read();if(done)break;pending+=decoder.decode(value,{stream:true});let end;while((end=pending.indexOf('\n\n'))>=0){const event=pending.slice(0,end);pending=pending.slice(end+2);if(event.startsWith('event: frame\n'))accept(JSON.parse(event.slice(event.indexOf('data: ')+6)));}}
 await reader.cancel();
}finally{clearTimeout(timer);}
assert(complete&&frames.length>100);let stale=0,recovered=0,noReturn=0,off=0,on=0;
for(const {frame:f} of frames){const bank=f.proximity_sensors;assert.equal(bank.source,'simulated_fork_proximity_sensors_v1');assert.equal(bank.sensors.length,8);assert.equal(bank.controller_connected,false);assert.equal(bank.safety_rated,false);assert.equal(bank.queue_overflows,0);assert(!JSON.stringify(bank).includes('truth_'));
 for(const s of bank.sensors){if(s.transport_valid){assert.equal(s.status,'valid');assert(s.delivery_sim_s-s.capture_sim_s+1e-8>=s.latency_s);assert(s.age_s<=s.stale_s+1e-8);}else assert.equal(s.measurement_valid,false);
  if(!s.measurement_valid){if(s.kind==='range')assert.equal(s.range_m,null);else assert.equal(s.detected,null);}
  else if(s.kind==='range')assert(Number.isFinite(s.range_m)&&s.range_m>=s.min_m&&s.range_m<=s.max_m);else {assert.equal(typeof s.detected,'boolean');if(s.detected)on++;else off++;}
  if(s.reading_status==='no_return'){noReturn++;assert(s.transport_valid&&!s.measurement_valid&&s.range_m===null);}
 }
 if(f.sim_time_s>6.4&&f.sim_time_s<8){stale++;assert(bank.sensors.every(s=>s.status==='stale'&&!s.transport_valid));}
 if(f.sim_time_s>8.3&&f.sim_time_s<8.7){recovered++;assert(bank.sensors.every(s=>s.transport_valid));}
}
assert(stale>3&&recovered>1&&noReturn>10&&off>10&&on>10,{stale,recovered,noReturn,off,on});
const csv=proximitySensorCsv(frames);assert.equal(csv.columns.length,17);for(const r of csv.rows)assert.equal(r.split(',').length,17);
const raw=frames.map(e=>JSON.stringify(e)).join('\n')+'\n';await writeFile(new URL('../../Saved/Validation/ProximitySensors-Live.jsonl',import.meta.url),raw);await writeFile(new URL('../../Saved/Validation/ProximitySensors-WebExport.csv',import.meta.url),csv.csv);
const report={date:'2026-11-05',pass:true,actual_frames:frames.length,stale_frames:stale,recovery_frames:recovered,no_return_observations:noReturn,presence_on:on,presence_off:off,csv_columns:17,csv_rows:csv.rows.length,csv_sha256:createHash('sha256').update(csv.csv).digest('hex'),stream_sha256:createHash('sha256').update(raw).digest('hex'),scope:'Actual received Unreal stream and shared UI exporter; no semantic object truth, no DOM click automation.'};
await writeFile(new URL('../../Docs/References/ProximitySensorsWeb-20261105.json',import.meta.url),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report));
