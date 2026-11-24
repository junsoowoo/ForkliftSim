// Sends only task requests. Every displayed value and task acknowledgement comes from Unreal.
import assert from 'node:assert/strict';
import {randomUUID} from 'node:crypto';
import {writeFile,mkdir} from 'node:fs/promises';
import {taskCsv} from './dist/tasks.mjs';
const base=process.argv[2]??'http://127.0.0.1:4184',start=Date.now(),folder=new URL('../../Saved/Validation/Tasks/',import.meta.url);await mkdir(folder,{recursive:true});
const pause=()=>new Promise(r=>setTimeout(r,200));let world=null,run=null,submitted=null,cancel=null,accepted=false,cancelled=false,motion=false;const frames=[],seen=new Set();
const post=(url,body)=>fetch(base+url,{method:'POST',headers:{'Content-Type':'application/json',Origin:base},body:JSON.stringify(body)}).then(async r=>{const j=await r.json();assert.equal(r.status,202,JSON.stringify(j));return j;});
while(Date.now()-start<180000){
 const snapshot=await fetch(base+'/api/snapshot').then(r=>r.json()),f=snapshot.latest?.frame,b=f?.fleet?.task_board;
 if(!b||b.qualification_case!=='board_web'||snapshot.status!=='running'){await pause();continue;}
 if(!world){assert.match(f.source.world_id,/^[A-Fa-f0-9]{32}$/);world=f.source.world_id;run=b.run_id;submitted={request_id:'live_'+randomUUID(),world_id:world,run_id:run,tasks:b.loads.slice(0,10).map((l,i)=>({load_id:l.id,destination:'',priority:i===0?9:0,tier:1}))};await post('/api/tasks',submitted);await post('/api/tasks',submitted);}
 assert.equal(f.source.world_id,world);if(!seen.has(f.source.sequence)){frames.push(snapshot.latest);seen.add(f.source.sequence);}
 const ack=b.command_receipts.find(r=>r.request_id===submitted.request_id);if(ack){assert.equal(ack.status,'accepted',ack.reason);assert.equal(ack.task_ids.length,10);accepted=true;assert.equal(b.tasks.length,10);}
 if(accepted&&!cancel){const t=b.tasks.find(t=>t.state==='pending');if(t){cancel={request_id:'cancel_'+randomUUID(),world_id:world,run_id:run,task_id:t.id};await post('/api/tasks/cancel',cancel);}}
 if(cancel){const ack=b.command_receipts.find(r=>r.request_id===cancel.request_id);if(ack){assert.equal(ack.status,'accepted',ack.reason);cancelled=b.tasks.find(t=>t.id===cancel.task_id)?.state==='cancelled';}}
 motion||=f.fleet.vehicles.some(v=>Math.abs(v.vehicle.speed_mps)>.05)&&b.tasks.some(t=>t.distance_est_m>.15);
 if(accepted&&cancelled&&motion&&frames.length>=20){const board=frames.at(-1).frame.fleet.task_board;const csv=taskCsv(board);assert.equal(csv.split('\n').length,11);assert.equal(csv.split('\n')[0].split(',').length,29);await writeFile(new URL('WebLiveFrames.json',folder),JSON.stringify(frames));await writeFile(new URL('WebLiveTasks.csv',folder),csv);const evidence={pass:true,source:'Actual Unreal HTTP frames, not synthetic telemetry',world_id:world,run_id:run,frames:frames.length,task_count:10,duplicate_request_created_no_extra_tasks:true,pending_cancel_confirmed:true,measured_motion:true,csv_columns:29,request_id:submitted.request_id,cancel_request_id:cancel.request_id};await writeFile(new URL('WebLiveEvidence.json',folder),JSON.stringify(evidence,null,2));console.log(evidence);process.exit(0);}
 await pause();
}
throw new Error('Actual Unreal task registration / cancellation / motion confirmation timed out');
