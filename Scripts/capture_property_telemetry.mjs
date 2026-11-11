// Record actual native substep peaks without the dashboard's 600-frame limit.
import assert from 'node:assert/strict';
import {writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
const base=process.argv[2]??'http://127.0.0.1:4174';
assert.match(base,/^http:\/\/127\.0\.0\.1:\d{4,5}$/);
assert.equal((await(await fetch(base+'/api/health')).json()).service,'forkliftsim-live-bridge');
const samples=[],seen=new Set();let finished=false;
function accept(e){
 const f=e?.frame;if(f?.source?.kind!=='unreal'||f.drive_type!=='mecanum'||!Number.isFinite(f.energy_audit?.chain_damping_j))return;
 const key=f.source.world_id+':'+f.source.sequence;if(seen.has(key))return;seen.add(key);
 samples.push({received_at_ms:e.received_at_ms,source:f.source,scenario:f.scenario,time_s:f.sim_time_s,
  contact_sample_valid:f.contact_sample_valid,energy:f.energy_audit,
  wheels:f.wheels.map(w=>({id:w.id,normal_n:w.normal_n,inner_peak_n:w.mecanum.contact_inner_peak_n,
   catalogue_ratio:w.mecanum.catalogue_load_comparison_ratio,mass_kg:w.mecanum.physical_wheel_mass_kg})),
  native_frame_sha256:createHash('sha256').update(JSON.stringify(f)).digest('hex')});
 if(f.scenario==='mecanum_high_carry'&&f.source.state==='stopped')finished=true;
}
const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),25*60*1000);
const response=await fetch(base+'/events',{signal:controller.signal});assert.equal(response.status,200);
for(const e of (await(await fetch(base+'/api/history')).json()).frames)accept(e);
const reader=response.body.getReader(),decode=new TextDecoder();let pending='';
try{
 while(!finished){const {done,value}=await reader.read();if(done)break;pending+=decode.decode(value,{stream:true});
  let end;while((end=pending.indexOf('\n\n'))>=0){const part=pending.slice(0,end);pending=pending.slice(end+2);
   if(part.startsWith('event: frame\n'))accept(JSON.parse(part.slice(part.indexOf('data: ')+6)));
  }
 }
}finally{clearTimeout(timer);await reader.cancel();}
assert(finished,'Capture ended before actual high-carry stopped');
const groups={};for(const s of samples){if(!s.contact_sample_valid||s.time_s<1)continue;
 const group=groups[s.scenario]??={samples:0,peak_frame_n:0,peak_inner_n:0};group.samples++;
 for(const w of s.wheels){group.peak_frame_n=Math.max(group.peak_frame_n,w.normal_n);group.peak_inner_n=Math.max(group.peak_inner_n,w.inner_peak_n);}
}
const out=new URL('../Saved/Validation/Mecanum-PropertyEnergy-LiveCapture.jsonl',import.meta.url);
const content=samples.map(s=>JSON.stringify(s)).join('\n')+'\n';await writeFile(out,content,'utf8');
const summary={pass:true,samples:samples.length,groups,sha256:createHash('sha256').update(content).digest('hex'),
 scope:'Actually received native frames, with per-frame inner-step peaks; missing frames and pre-subscription history are not reconstructed. Not a global peak qualification.'};
await writeFile(new URL('../Saved/Validation/Mecanum-PropertyEnergy-LiveCapture.json',import.meta.url),JSON.stringify(summary,null,2));
console.log(JSON.stringify(summary));
