import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {LiveState,measurements} from './dist/live-state.mjs';
const base=process.argv[2]||'http://127.0.0.1:4183',mode=process.argv[3]||'loaded';
if(!/^http:\/\/127\.0\.0\.1:\d+$/.test(base))throw new Error('Loopback test server only');
const source=await fetch(`${base}/api/history`).then(r=>r.json());
const world=source.frames.at(-1)?.frame.source.world_id;
const frames=source.frames.filter(e=>e.frame.source.world_id===world),valid=frames.filter(e=>e.frame.contact_sample_valid);
assert.ok(valid.length>=10,'Actual Unreal frames missing');assert.ok(frames.every(e=>e.frame.source.kind==='unreal'));
const model=new LiveState();for(const e of frames)assert.equal(model.accept(e),true,'Unreal source sequence must be ordered');
const last=valid.at(-1),F=last.frame,M=measurements(F);
assert.equal(M.pressure,F.hydraulic.pressure_pa);assert.equal(M.speed,F.vehicle.speed_mps);
for(const e of valid){const f=e.frame;assert.ok(Math.abs(f.wheels.reduce((sum,w)=>sum+w.normal_n,0)-f.vehicle.normal_sum_n)<.001);}
const result={mode,status:source.status,world_id:world,scenario:F.scenario,frames:frames.length,valid_frames:valid.length,run_ids:[...new Set(frames.map(e=>e.frame.source.run_id))],sim_range:[frames[0].frame.sim_time_s,F.sim_time_s],height_range:[Math.min(...valid.map(e=>e.frame.fork.height_m)),Math.max(...valid.map(e=>e.frame.fork.height_m))],fork_n_range:[Math.min(...valid.map(e=>e.frame.fork.tines.reduce((s,t)=>s+t.vertical_n,0))),Math.max(...valid.map(e=>e.frame.fork.tines.reduce((s,t)=>s+t.vertical_n,0)))],pressure_pa_range:[Math.min(...valid.map(e=>e.frame.hydraulic.pressure_pa)),Math.max(...valid.map(e=>e.frame.hydraulic.pressure_pa))],road_mu_values:[...new Set(valid.map(e=>e.frame.settings.road_mu))],final_pressure_pa:M.pressure,final_wheel_sum_n:M.normalTotal};
if(mode==='loaded'){
 assert.equal(F.scenario,'swerve_hydraulic_loaded');assert.ok(result.height_range[0]<.05&&result.height_range[1]>.8,'Actual lift must change');
 assert.ok(result.fork_n_range[0]<100&&result.fork_n_range[1]>3500,'Actual fork contact load must change');
 assert.ok(F.pallet.height_m>.65&&F.pallet.ground_n<10);assert.ok(F.hydraulic.pressure_pa>2.6e6&&F.hydraulic.pressure_pa<3.1e6);
 assert.equal(M.forkLeft,F.fork.tines[0].vertical_n);assert.equal(M.forkRight,F.fork.tines[1].vertical_n);
 for(const w of F.wheels)assert.ok(Math.abs(w.local_position_m[0])>.7&&Math.abs(w.local_position_m[0])<.9,'Local positions must not include chassis mesh scaling');
}
if(mode==='reset')assert.ok(result.run_ids.length>=2,'Actual R/reset fixture must create a new run ID');
if(mode==='friction'){
 assert.ok(result.road_mu_values.length>=2,'Actual road material setting must change during run');
 const lowGrip=valid.filter(e=>e.frame.settings.road_mu===0),normalGrip=valid.filter(e=>e.frame.settings.road_mu>.5);
 assert.ok(lowGrip.length>3&&normalGrip.length>3,'Both road material phases must arrive');
 assert.ok(lowGrip.every(e=>e.frame.wheels.every(w=>w.contact_mu===null||w.contact_mu<.001)),'Actual contact mu must follow the engine material change');
}
const output=new URL(`../../Saved/Validation/WebTelemetry-${mode}-${Date.now()}/`,import.meta.url);await mkdir(output,{recursive:true});
await writeFile(new URL('result.json',output),JSON.stringify({...result,pass:true},null,2));await writeFile(new URL('received-frames.json',output),JSON.stringify(frames));
console.log(`UNREAL_WEB_CHECK PASS ${JSON.stringify(result)}`);console.log(`Actual received frame evidence: ${output.pathname}`);
