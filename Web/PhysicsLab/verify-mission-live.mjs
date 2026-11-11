import assert from 'node:assert/strict';
import {writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {missionCsv} from './dist/mission.mjs';
const base=process.argv[2]??'http://127.0.0.1:4174', earliest=Number(process.argv[3]??Date.now());
const names=['blocked','cancel','watchdog','retry','sensor_recovery','low','avoid_pause','high'].map(x=>'mecanum_mission_'+x);
const frames=[],seen=new Set(),finals=new Map(),abort=new AbortController();
const timer=setTimeout(()=>abort.abort(),3600000);
function accept(e){const f=e.frame;if(f?.source?.kind!=='unreal'||f.source.started_at_ms<earliest||!names.includes(f.scenario))return;const key=f.source.world_id+':'+f.source.sequence;if(seen.has(key))return;seen.add(key);frames.push(e);if(f.source.state==='stopped')finals.set(f.scenario,f);}
try {
 for(const e of (await(await fetch(base+'/api/history')).json()).frames)accept(e);
 const reader=(await fetch(base+'/events',{signal:abort.signal})).body.getReader(),decoder=new TextDecoder();let pending='';
 while(finals.size<names.length){const r=await reader.read();if(r.done)break;pending+=decoder.decode(r.value,{stream:true});let end;while((end=pending.indexOf('\n\n'))>=0){const event=pending.slice(0,end);pending=pending.slice(end+2);if(event.startsWith('event: frame\n'))accept(JSON.parse(event.slice(event.indexOf('data: ')+6)));}}
 await reader.cancel();
} finally {clearTimeout(timer);}
assert.equal(finals.size,8);assert(frames.length>500);
let moving=0,self=0,paths=0;const reportCases=[];
for(const {frame:f} of frames){const m=f.mission;assert.equal(m.source,'measured_grid_mission_v1');assert.equal(m.ground_truth_feedback,false);const cap=m.carrying?.18:.25;assert(Math.hypot(m.vx_command_mps,m.vy_command_mps)<=cap+1e-7);if(['paused','fault_latched','blocked','cancelled'].includes(m.state))assert(m.brake&&m.vx_command_mps===0&&m.vy_command_mps===0&&m.yaw_command_rad_s===0);moving+=Math.hypot(m.vx_command_mps,m.vy_command_mps)>0;self+=m.carrier_self_filter;paths+=(m.path_m?.length??0)>1;}
for(const [name,f] of finals){const m=f.mission;const expected=name.endsWith('blocked')?'blocked':name.endsWith('cancel')?'cancelled':name.endsWith('watchdog')?'fault_latched':name.endsWith('retry')?'pickup':name.endsWith('sensor_recovery')?'to_dropoff':'complete';assert.equal(m.state,expected,name);if(expected==='complete')assert(m.pickup_receipt&&m.dropoff_receipt&&!m.carrying);if(name.endsWith('retry'))assert.equal(m.retries,1);if(name.endsWith('watchdog'))assert.equal(m.reason,'mission_command_expired');if(name.endsWith('sensor_recovery'))assert(m.localized&&m.carrying);reportCases.push({case:name,state:m.state,reason:m.reason,pickup_receipt:m.pickup_receipt,dropoff_receipt:m.dropoff_receipt,replans:m.replans,retries:m.retries});}
assert(moving>100&&self>50&&paths>100);const csv=missionCsv(frames);assert.equal(csv.columns.length,30);assert.equal(csv.rows.length,frames.length);assert.equal((await fetch(base+'/mission.mjs')).status,200);
const raw=frames.map(e=>JSON.stringify(e)).join('\n')+'\n';await writeFile(new URL('../../Saved/Validation/Mission-Live.jsonl',import.meta.url),raw);await writeFile(new URL('../../Saved/Validation/Mission-WebExport.csv',import.meta.url),csv.csv);
const hash=s=>createHash('sha256').update(s).digest('hex');
const report={date:'2026-11-07',pass:true,actual_frames:frames.length,moving_frames:moving,carried_self_filter_frames:self,planned_path_frames:paths,csv_columns:30,csv_rows:csv.rows.length,csv_sha256:hash(csv.csv),stream_sha256:hash(raw),cases:reportCases,scope:'Actual Unreal SSE and received-only module export. Complete cycles: low/high/avoid_pause. Retry and sensor recovery end at explicit checkpoints. Browser history remains current-world last 30 seconds/max600; this independent recorder spans all eight cases. Not DOM-click or full-mission realtime qualification.'};
await writeFile(new URL('../../Docs/References/MissionWeb-20261107.json',import.meta.url),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report));
