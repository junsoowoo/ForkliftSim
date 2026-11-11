import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {measurements} from './dist/live-state.mjs';
const base=process.argv[2]??'http://127.0.0.1:4174';
const earliestStartMs=Number(process.argv[3]??0);
assert.match(base,/^http:\/\/127\.0\.0\.1:\d{4,5}$/);
const response=await(await fetch(base+'/api/history')).json();
const actual=response.frames.filter(e=>e.frame.energy_audit?.contact_work_scope&&e.frame.source.started_at_ms>=earliestStartMs);
assert(actual.length>=20,'Need actual current Unreal frames');
const fields=['reduced_contact_loss_j','plastic_relaxation_j','fracture_release_j','contact_normal_work_estimate_j','contact_tangent_work_estimate_j','contact_correction_pseudo_j','fork_damping_estimate_j'];
for(const {frame:f} of actual){
 assert.equal(f.source.kind,'unreal');assert.equal(f.energy_audit.full_closure,false);
 assert.equal(f.wheels[0].mecanum.model,'equivalent-directional-contact-v1');assert.equal(f.wheels[0].mecanum.contact_geometry,'analytic-finite-cylinder-box-v2-directional-friction');
 for(const w of f.wheels){assert.equal(w.mecanum.catalogue_load_comparison_ratio,null);assert.equal(w.mecanum.catalogue_listed_load_kg,750);assert.equal(w.mecanum.rated_load_qualified,false);}
 assert(Number.isFinite(f.handling_rack.support_n));
 if(f.energy_audit.reference_valid){const e=f.energy_audit;for(const k of fields)assert(Number.isFinite(e[k]));assert(Math.abs(e.stored_delta_j-e.known_input_j+e.known_loss_j-e.unresolved_j)<1e-5);}
}
const source=await readFile(new URL('./dist/live.mjs',import.meta.url),'utf8');
const prefix="$('#export-history').addEventListener('click',()=>{";const start=source.indexOf(prefix)+prefix.length;const end=source.indexOf(' const url=URL.createObjectURL',start);assert(end>start);
const exportRows=new Function('state','measurements',source.slice(start,end)+'\nreturn {columns,rows};');const {columns,rows}=exportRows({history:actual},measurements);
for(let i=0;i<rows.length;++i){const cells=rows[i].split(',');assert.equal(cells.length,columns.length);const v=Object.fromEntries(columns.map((k,n)=>[k,cells[n]]));const f=actual[i].frame;
 if(f.energy_audit.reference_valid)for(const k of fields)assert.equal(Number(v[k]),f.energy_audit[k]);
 if(f.contact_sample_valid)assert.equal(Number(v.rack_support_n),f.handling_rack.support_n);
}
const csv=columns.join(',')+'\n'+rows.join('\n')+'\n';const destination=new URL('../../Saved/Validation/CurrentPhysics-WebExport.csv',import.meta.url);await writeFile(destination,csv);
const report={pass:true,earliest_start_ms:earliestStartMs,actual_frames:actual.length,columns:columns.length,worlds:[...new Set(actual.map(e=>e.frame.source.world_id))],scenarios:[...new Set(actual.map(e=>e.frame.scenario))],csv_sha256:createHash('sha256').update(csv).digest('hex'),full_energy_closure:false,scope:'Actual Unreal frames and UI CSV construction verified. DOM download click not automated.'};
await writeFile(new URL('../../Docs/References/CurrentPhysicsWeb-20261031.json',import.meta.url),JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report));
