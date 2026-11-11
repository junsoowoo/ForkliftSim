import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
const base=process.argv[2]??'http://127.0.0.1:4174';
const response=await fetch(base+'/api/history'); assert.equal(response.status,200);
const {frames}=await response.json(); const world=frames.at(-1)?.frame.source.world_id;
const actual=frames.filter(e=>e.frame.source.world_id===world);
assert(actual.length>20,'Need actual Unreal frames');
assert.equal(actual.at(-1).frame.source.state,'stopped');
const valid=actual.filter(e=>e.frame.energy_audit?.reference_valid);
assert(valid.length>10,'Need an actual referenced energy budget');
for(const {frame:f} of valid){
 assert.equal(f.source.kind,'unreal'); const a=f.energy_audit;
 assert.equal(a.full_closure,false);
 for(const k of ['stored_delta_j','drive_work_j','steering_joint_work_j','rolling_work_j','hydraulic_dc_j','hydraulic_heat_j','tyre_damping_j','known_input_j','known_loss_j','unresolved_j','positive_peak_j']) assert(Number.isFinite(a[k]),k);
 assert(Math.abs(a.unresolved_j-(a.stored_delta_j-a.known_input_j+a.known_loss_j))<1e-6);
 assert(Math.abs(a.known_input_j-a.drive_work_j-a.steering_joint_work_j-a.hydraulic_dc_j)<1e-6);
 assert(Math.abs(a.known_loss_j-a.hydraulic_heat_j-a.tyre_damping_j+a.rolling_work_j-(a.roller_bearing_loss_j??0)-(a.shaft_loss_j??0)-(a.chain_damping_j??0))<1e-6);
 assert(f.fork.tines.every(t=>Number.isFinite(t.elastic_j)&&t.elastic_j>=0));
}
const loaded=valid.filter(e=>e.frame.fork.height_m>.35);
assert(loaded.length>10,'Need actual lifted cargo');
assert(loaded.some(e=>e.frame.fork.tines.every(t=>t.elastic_j>0)),'Loaded flexible forks must store bending energy');
const result={pass:true,scope:'Actual observer provenance, finite values, bending energy and accounting identity; not full energy closure.',full_closure:false,
 world_id:world,frames:actual.length,referenced:valid.length,scenario:actual.at(-1).frame.scenario,final:valid.at(-1).frame.energy_audit};
const out=new URL(`../../Saved/Validation/EnergyWeb-${Date.now()}/`,import.meta.url); await mkdir(out,{recursive:true});
await writeFile(new URL('result.json',out),JSON.stringify(result,null,2));
console.log('ACTUAL_ENERGY_WEB PASS '+JSON.stringify(result));
