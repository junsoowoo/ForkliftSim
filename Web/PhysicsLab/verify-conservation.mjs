import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {measurements} from './dist/live-state.mjs';

const base=process.argv[2]??'http://127.0.0.1:4174';
assert.match(base,/^http:\/\/127\.0\.0\.1:[0-9]{4,5}$/);
const health=await(await fetch(base+'/api/health')).json();
assert.equal(health.service,'forkliftsim-live-bridge');
const {frames}=await(await fetch(base+'/api/history')).json();
const world=frames.at(-1)?.frame.source.world_id;
const actual=frames.filter(e=>e.frame.source.world_id===world);
assert(actual.length>20);assert.equal(actual.at(-1).frame.source.state,'stopped');
for(const {frame:f} of actual){
 assert.equal(f.source.kind,'unreal');assert.equal(f.bearing_integrator,'symmetric_spin_verlet');
 assert(f.physics_selected_step_s<=1/1920+1e-9);
 assert(f.physics_smallest_observed_s<=1/480+1e-9); // initial frame may precede the first inner output
 assert(f.physics_inner_clock_error_s<1e-7);
 assert.equal(f.energy_audit.full_closure,false);
 if(f.energy_audit.reference_valid){assert(Number.isFinite(f.energy_audit.bearing_transport_work_j));assert(Number.isFinite(f.energy_audit.chain_damping_j));assert(f.energy_audit.chain_damping_j>=-1e-7);}
 for(const w of f.wheels){
  const m=w.mecanum;assert(m);assert.equal(m.rated_load_qualified,false);
  assert.equal(m.catalogue_comparison_model,'Hickwall HIT305AHD');
  assert.equal(m.catalogue_listed_load_kg,750);assert.equal(m.catalogue_listed_mass_kg,40);
  assert(Math.abs(m.physical_wheel_mass_kg-50)<.001);
  assert.equal(m.catalogue_load_comparison_ratio,null);assert(m.catalogue_load_status.includes("conflicts"));
  assert(m.hub_inertia_mass_frame_kg_m2.every(x=>Number.isFinite(x)&&x>0));
  for(const r of m.rollers){assert(r.inertia_mass_frame_kg_m2.every(x=>Number.isFinite(x)&&x>0));assert(Math.abs(Math.hypot(...r.mass_frame_rotation_xyzw)-1)<1e-5);}
  if(f.contact_sample_valid)assert(m.contact_inner_peak_n>=w.normal_n*.99);
 }
}
// Execute the actual UI export handler's data construction. It uses recorded
// Unreal frames and the shared measurement selector, with no synthetic plant.
// The DOM download invocation is outside this test's scope.
const source=await readFile(new URL('./dist/live.mjs',import.meta.url),'utf8');
const prefix="$('#export-history').addEventListener('click',()=>{";
const start=source.indexOf(prefix)+prefix.length;
const end=source.indexOf(' const url=URL.createObjectURL',start);
assert(start>=prefix.length&&end>start);
const exportRows=new Function('state','measurements',source.slice(start,end)+'\nreturn {columns,rows};');
const {columns,rows}=exportRows({history:actual},measurements);
assert.equal(columns.length,192);assert.equal(rows.length,actual.length);
for(let i=0;i<rows.length;++i){
 const cells=rows[i].split(',');assert.equal(cells.length,columns.length);
 const values=Object.fromEntries(columns.map((key,k)=>[key,cells[k]]));
 assert.equal(values.world_id,world);assert.equal(values.bearing_integrator,'symmetric_spin_verlet');
 assert.equal(Number(values.physics_selected_step_s),actual[i].frame.physics_selected_step_s);
 assert.equal(Number(values.physics_inner_clock_error_s),actual[i].frame.physics_inner_clock_error_s);
 if(actual[i].frame.energy_audit.reference_valid){assert.equal(Number(values.bearing_transport_work_j),actual[i].frame.energy_audit.bearing_transport_work_j);assert.equal(Number(values.chain_damping_j),actual[i].frame.energy_audit.chain_damping_j);}
 for(const w of actual[i].frame.wheels){assert.equal(Number(values[`${w.id}_physical_wheel_mass_kg`]),w.mecanum.physical_wheel_mass_kg);if(actual[i].frame.contact_sample_valid)assert.equal(Number(values[`${w.id}_inner_peak_load_n`]),w.mecanum.contact_inner_peak_n);}
}
const finalFrame=actual.at(-1).frame;assert(finalFrame.energy_audit.chain_damping_j>0,'Actual moving chain must dissipate energy');
const csv='\ufeff'+[columns.join(','),...rows].join('\r\n');
const path=new URL('../../Saved/Validation/Mecanum-PropertyEnergy-WebExport.csv',import.meta.url);
await writeFile(path,csv,'utf8');
const result={pass:true,scope:'Actual native telemetry and actual UI export data function; not a browser download or complete energy closure.',
 world_id:world,scenario:actual.at(-1).frame.scenario,rows:rows.length,columns:columns.length,
 csv_sha256:createHash('sha256').update(csv).digest('hex'),full_energy_closure:false,final:finalFrame.energy_audit,wheels:finalFrame.wheels.map(w=>({id:w.id,normal_n:w.normal_n,...w.mecanum,rollers:undefined}))};
await writeFile(new URL('../../Saved/Validation/Mecanum-PropertyEnergy-WebExport.json',import.meta.url),JSON.stringify(result,null,2));
console.log('ACTUAL_CONSERVATION_WEB PASS '+JSON.stringify(result));
