import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {LiveState,measurements} from './dist/live-state.mjs';
const url=process.argv[2]??'http://127.0.0.1:4174';
const response=await fetch(url+'/api/history'); assert.equal(response.status,200);
const {frames}=await response.json(),world=frames.at(-1)?.frame.source.world_id;
const actual=frames.filter(e=>e.frame.source.world_id===world&&['swerve_components_lift','swerve_components_carry','swerve_components_lower','swerve_components_step','swerve_components_halfstep','mecanum_lift','mecanum_carry','mecanum_high_carry','mecanum_step','mecanum_ramp_brake'].includes(e.frame.scenario));
assert(actual.length>20,'Need actual component run, not protocol fixture');
const ui=new LiveState();
for(const e of actual){assert.equal(e.frame.source.kind,'unreal');assert(ui.accept(e));const m=measurements(e.frame);assert.equal(m.pressure,e.frame.hydraulic.pressure_pa);}
const valid=actual.filter(e=>e.frame.contact_sample_valid),final=actual.at(-1).frame;
assert.equal(final.source.state,'stopped');
const loaded=valid.filter(e=>e.frame.fork.height_m>.35&&e.frame.hydraulic.chain_tension_n>5000);
assert(loaded.length>10,'Actual lifted load must be supported by the chain');
const last=loaded.at(-1).frame; assert(Math.abs(last.vehicle.dry_mass_kg-3140)<.01);
assert(last.fork.height_m>.35);assert(last.hydraulic.detailed&&last.hydraulic.chain_mast);
assert(last.hydraulic.chain_tension_n>5000);assert(last.hydraulic.chain_stretch_m>0);
assert(last.hydraulic.pressure_pa>5e6&&last.hydraulic.pressure_pa<7e6);
assert(last.hydraulic.oil_temp_c>35&&last.hydraulic.oil_temp_c<45);
assert.equal(last.hydraulic.line_pressures_pa.length,5);
if(last.drive_type==='mecanum'){
 assert(last.wheels.every(w=>!w.detailed_compliance&&w.estimated_patch_valid===false&&!w.tread.enabled&&w.steerable===false&&w.mecanum?.rollers.length===12));
}else assert(last.wheels.every(w=>w.detailed_compliance&&w.estimated_patch_length_m>0&&Number.isFinite(w.shear_y_m)));
assert(last.fork.tines.every(t=>t.flexible&&t.section_count===4&&!t.failed&&t.tip_deflection_m>0&&Number.isFinite(t.max_stress_pa)));
for(const e of valid) assert(Math.abs(e.frame.wheels.reduce((n,w)=>n+w.normal_n,0)-e.frame.vehicle.normal_sum_n)<1e-6);
for(const e of valid)for(const w of e.frame.wheels){
 const c=w.solver_contact;assert(c,'Missing actual solver contact distribution');
 assert.equal(c.band_normal_n.length,3);assert(c.band_normal_n.every(n=>Number.isFinite(n)&&n>=0));
 assert.equal(c.normal_moment_world_nm.length,3);assert(c.normal_moment_world_nm.every(Number.isFinite));
 assert.equal(c.normal_world_n.length,3);assert(c.normal_world_n.every(Number.isFinite));
 if(w.tread?.enabled){
  const t=w.tread;assert.equal(t.aggregate_radial_spring,false);assert.equal(t.pads.length,t.segments);
  assert([16,32,48,64].includes(t.segments));assert.equal(t.resolution_qualified,false);assert.equal(t.phase_rad,0);
  if(t.coupled){assert.equal(t.aggregate_shear_spring,false);assert.equal(t.links,t.segments);assert.equal(w.estimated_patch_valid,false);assert(t.link_stored_j>0);}
  assert(t.pads.every((p,i)=>p.index===i&&Number.isFinite(p.compression_m)&&Number.isFinite(p.rate_mps)&&Number.isFinite(p.normal_n)&&p.normal_n>=0));
  assert(Math.abs(t.pads.reduce((n,p)=>n+p.normal_n,0)-c.point_normal_sum_n)<1e-5);
  for(const p of t.pads){const r=Math.hypot(...p.world_position_m.map((v,i)=>v-w.world_position_m[i]));const radial=e.frame.settings.wheel_radius_m-t.depth_m/2-p.compression_m;const expected=t.coupled?Math.hypot(radial,p.shear_m):radial;assert(Math.abs(r-expected)<.001,'World pose must match actual radial/tangent deformation');if(t.coupled){assert(Number.isFinite(p.shear_m)&&Number.isFinite(p.rotation_rad));}}
 }
 assert(Math.abs(c.band_normal_n.reduce((a,b)=>a+b,0)-c.point_normal_sum_n)<1e-6);
 if(c.valid){assert(c.point_normal_sum_n>0);assert(c.load_center_world_m.length===3&&c.load_center_world_m.every(Number.isFinite));assert(c.peak_active_points>0);}
 else assert.equal(c.load_center_world_m,null);
}
const report={pass:true,frames:actual.length,scenario:final.scenario,world_id:world,height_m:last.fork.height_m,final_height_m:final.fork.height_m,mass_kg:last.vehicle.dry_mass_kg,
 chain_tension_n:last.hydraulic.chain_tension_n,chain_stretch_m:last.hydraulic.chain_stretch_m,oil_temp_c:last.hydraulic.oil_temp_c,
 pressure_pa:last.hydraulic.pressure_pa,deflection_m:last.fork.tines.map(t=>t.tip_deflection_m)};
const out=new URL(`../../Saved/Validation/ComponentWeb-${Date.now()}/`,import.meta.url);await mkdir(out,{recursive:true});
await writeFile(new URL('received-frames.json',out),JSON.stringify(actual));await writeFile(new URL('result.json',out),JSON.stringify(report,null,2));
console.log('ACTUAL_COMPONENT_WEB PASS '+JSON.stringify(report));
