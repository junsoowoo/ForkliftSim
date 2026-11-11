import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
const base=process.argv[2]??'http://127.0.0.1:4174';
const {frames}=await(await fetch(base+'/api/history')).json();
const world=frames.at(-1)?.frame.source.world_id;
const actual=frames.filter(e=>e.frame.source.world_id===world);
assert(actual.length>20,'Need actual Unreal frames');
assert.equal(actual.at(-1).frame.source.state,'stopped');
let maxOmega=0,maxAnchor=0,maxClosure=0,maxBearingLoss=0;
for(const {frame:f} of actual){
 assert.equal(f.source.kind,'unreal');assert.equal(f.drive_type,'mecanum');
 assert.equal(f.drive_update,'physics_inner_step');assert.equal(typeof f.controls.auto_hold,'boolean');
 assert(f.game_max_frame_s<=1/30+1e-6);assert(Math.abs(f.physics_clock_lag_s)<.034);
 assert.equal(f.wheels.length,4);assert.equal(f.energy_audit.full_closure,false);
 for(const w of f.wheels){
  assert.equal(w.steerable,false);assert.equal(w.tread.enabled,false);
  assert.equal(w.estimated_patch_valid,false);assert.equal(w.detailed_compliance,false);
  assert(Math.abs(w.steer_deg)<.5);
  const m=w.mecanum;assert.equal(m.contact_geometry,'inscribed-9-spheres');assert.equal(m.slip_measurement,'physics-contact-normal-impulse-weighted-rms');if(f.controls.auto_hold)assert(w.brake_limit_nm>=899);assert.equal(m.model,'physical-passive-rollers-v1');assert.equal(m.rollers.length,12);
  assert.equal(m.rubber_deformation_modelled,false);assert.equal(m.rated_load_qualified,false);
  assert.equal(m.hub_mass_kg+m.rollers.reduce((n,r)=>n+r.mass_kg,0),50);
  for(const r of m.rollers){
   for(const key of ['normal_n','relative_omega_rad_s','bearing_torque_nm','anchor_error_m'])assert(Number.isFinite(r[key]),key);
   assert.equal(r.world_position_m.length,3);assert.equal(r.axis_world.length,3);
   maxOmega=Math.max(maxOmega,Math.abs(r.relative_omega_rad_s));maxAnchor=Math.max(maxAnchor,r.anchor_error_m);
  }
  if(f.contact_sample_valid&&w.solver_contact.valid){
   const total=m.rollers.reduce((n,r)=>n+r.normal_n,0);const difference=Math.abs(total-w.solver_contact.point_normal_sum_n);
   maxClosure=Math.max(maxClosure,difference);assert(difference<1e-5,'Roller loads must match actual point-normal sum');
  }
 }
 if(f.energy_audit.reference_valid){
  const a=f.energy_audit;assert(Number.isFinite(a.roller_bearing_loss_j));
  assert(Math.abs(a.known_loss_j-a.hydraulic_heat_j-a.tyre_damping_j+a.rolling_work_j-a.roller_bearing_loss_j-(a.shaft_loss_j??0))<1e-6);
  maxBearingLoss=Math.max(maxBearingLoss,a.roller_bearing_loss_j);
 }
}
assert(maxAnchor<.003,'Passive roller pivots must stay attached');
const result={pass:true,scope:'Actual Unreal source, physical rollers, contact-load closure, work accounting, passive pivot error; not real-wheel calibration.',world_id:world,frames:actual.length,
 scenario:actual.at(-1).frame.scenario,max_roller_omega_rad_s:maxOmega,max_anchor_error_m:maxAnchor,max_contact_closure_n:maxClosure,max_bearing_loss_j:maxBearingLoss};
const out=new URL(`../../Saved/Validation/MecanumWeb-${Date.now()}/`,import.meta.url);await mkdir(out,{recursive:true});
await writeFile(new URL('result.json',out),JSON.stringify(result,null,2));console.log('ACTUAL_MECANUM_WEB PASS '+JSON.stringify(result));
