import assert from 'node:assert/strict';
import {writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {driveEstimatorCsv} from './dist/drive-estimator.mjs';
const base=process.argv[2]??'http://127.0.0.1:4174',earliest=Number(process.argv[3]??Date.now()),frames=[],seen=new Set();let done=false;
const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),600000);
function accept(e){const f=e.frame;if(f?.source?.kind!=='unreal'||f.source.started_at_ms<earliest||f.scenario!=='mecanum_drive_sensor_straight')return;const k=f.source.world_id+':'+f.source.sequence;if(seen.has(k))return;seen.add(k);frames.push(e);if(f.source.state==='stopped')done=true;}
try{for(const e of (await(await fetch(base+'/api/history')).json()).frames)accept(e);const r=await fetch(base+'/events',{signal:abort.signal}),reader=r.body.getReader(),decoder=new TextDecoder();let pending='';while(!done){const {value,done:closed}=await reader.read();if(closed)break;pending+=decoder.decode(value,{stream:true});let end;while((end=pending.indexOf('\n\n'))>=0){const event=pending.slice(0,end);pending=pending.slice(end+2);if(event.startsWith('event: frame\n'))accept(JSON.parse(event.slice(event.indexOf('data: ')+6)));}}await reader.cancel();}finally{clearTimeout(timer);}
assert(done&&frames.length>100);let ready=0,located=0,calibrating=0,moving=0;
for(const {frame:f} of frames){const d=f.drive_estimator;assert.equal(d.source,'encoder_imu_surveyed_map_v1');assert.equal(d.ground_truth_feedback,false);assert.equal(d.torque_commands_nm.length,4);assert(d.torque_commands_nm.every(x=>Number.isFinite(x)&&Math.abs(x)<=450));
 if(d.state==='ready'){ready++;assert(Number.isFinite(d.vx_mps)&&Number.isFinite(d.odom_x_m));if(Math.abs(d.vx_mps)>.1)moving++;}else{calibrating++;assert.equal(d.vx_mps,null);}
 if(d.localized){located++;assert(Number.isFinite(d.map_x_m)&&d.match_age_s<.75);}else assert.equal(d.map_x_m,null);
}
assert(ready>100&&located>100&&calibrating>0&&moving>10);const csv=driveEstimatorCsv(frames);assert.equal(csv.columns.length,26);for(const row of csv.rows)assert.equal(row.split(',').length,26);
const raw=frames.map(e=>JSON.stringify(e)).join('\n')+'\n';await writeFile(new URL('../../Saved/Validation/SensorDrive-Live.jsonl',import.meta.url),raw);await writeFile(new URL('../../Saved/Validation/SensorDrive-WebExport.csv',import.meta.url),csv.csv);
assert.equal((await fetch(base+'/drive-estimator.mjs')).status,200);
const report={date:'2026-11-05',pass:true,actual_frames:frames.length,ready_frames:ready,localized_frames:located,calibrating_frames:calibrating,moving_frames:moving,csv_columns:26,csv_rows:csv.rows.length,csv_sha256:createHash('sha256').update(csv.csv).digest('hex'),stream_sha256:createHash('sha256').update(raw).digest('hex'),scope:'Actual Unreal received sensor-derived velocity/odometry/map localization/control torque and CSV. No DOM clicks, browser physics, GT fallback or FPS qualification.'};await writeFile(new URL('../../Docs/References/SensorDriveWeb-20261105.json',import.meta.url),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report));
