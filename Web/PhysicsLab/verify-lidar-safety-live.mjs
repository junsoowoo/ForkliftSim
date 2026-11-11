import assert from 'node:assert/strict';
import {writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {lidarSafetyCsv,lidarPoints} from './dist/lidar-safety.mjs';
const base=process.argv[2]??'http://127.0.0.1:4174',earliest=Number(process.argv[3]??Date.now()),frames=[],seen=new Set();let complete=false;
const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),600000);
function accept(e){const f=e.frame;if(f?.source?.kind!=='unreal'||f.source.started_at_ms<earliest||f.scenario!=='mecanum_vehicle_guard_front')return;const k=f.source.world_id+':'+f.source.sequence;if(seen.has(k))return;seen.add(k);frames.push(e);if(f.source.state==='stopped')complete=true;}
try{for(const e of (await(await fetch(base+'/api/history')).json()).frames)accept(e);const response=await fetch(base+'/events',{signal:abort.signal}),reader=response.body.getReader(),decoder=new TextDecoder();let pending='';while(!complete){const {value,done}=await reader.read();if(done)break;pending+=decoder.decode(value,{stream:true});let end;while((end=pending.indexOf('\n\n'))>=0){const event=pending.slice(0,end);pending=pending.slice(end+2);if(event.startsWith('event: frame\n'))accept(JSON.parse(event.slice(event.indexOf('data: ')+6)));}}await reader.cancel();}finally{clearTimeout(timer);}
assert(complete&&frames.length>100);let obstacles=0,heldAfterClear=0,readyAfterAck=0,points=0;
for(const {frame:f} of frames){const g=f.lidar_safety;assert.equal(g.source,'simulated_dual_planar_lidar_v1');assert.equal(g.safety_rated,false);assert.equal(g.scanners.length,2);
 for(const s of g.scanners){assert.equal(s.ranges_m.length,270);assert.equal(s.return_codes.length,270);if(s.fresh)assert(s.delivery_sim_s-s.capture_sim_s+1e-8>=s.latency_s&&s.age_s<=s.stale_s+1e-8);for(let k=0;k<270;k++){if(s.return_codes[k]===0){assert(Number.isFinite(s.ranges_m[k])&&s.ranges_m[k]>=s.min_range_m&&s.ranges_m[k]<=s.max_range_m);}else assert.equal(s.ranges_m[k],null);}}
 if(g.obstacle){obstacles++;assert(g.drive_hold&&!g.lift_hold);}
 if(f.sim_time_s>14.5&&f.sim_time_s<16.7){heldAfterClear++;assert(g.drive_hold&&!g.obstacle);}
 if(f.sim_time_s>17.4&&f.sim_time_s<18){readyAfterAck++;assert(!g.drive_hold);}
 points+=lidarPoints(g).length;
}
assert(obstacles>10&&heldAfterClear>5&&readyAfterAck>1&&points>100);
const csv=lidarSafetyCsv(frames);assert.equal(csv.columns.length,18);for(const r of csv.rows)assert.equal(r.split(',').length,18);
const raw=frames.map(e=>JSON.stringify(e)).join('\n')+'\n';await writeFile(new URL('../../Saved/Validation/LidarSafety-Live.jsonl',import.meta.url),raw);await writeFile(new URL('../../Saved/Validation/LidarSafety-WebExport.csv',import.meta.url),csv.csv);
const report={date:'2026-11-05',pass:true,actual_frames:frames.length,obstacle_frames:obstacles,clear_but_held_frames:heldAfterClear,acknowledged_ready_frames:readyAfterAck,measured_points_projected:points,csv_columns:18,csv_rows:csv.rows.length,csv_sha256:createHash('sha256').update(csv.csv).digest('hex'),stream_sha256:createHash('sha256').update(raw).digest('hex'),scope:'Actual received dual-LiDAR data, measured-only point projection and state CSV. DOM click not automated; no localization or map output.'};await writeFile(new URL('../../Docs/References/LidarSafetyWeb-20261105.json',import.meta.url),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report));
