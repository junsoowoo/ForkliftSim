// Actual Unreal-only live qualification. Does not generate/post telemetry.
import assert from 'node:assert/strict';
import {writeFile,mkdir} from 'node:fs/promises';
import {fleetCsv,selectVehicle,vehicleHistory} from './dist/fleet.mjs';
const base=process.argv[2]??'http://127.0.0.1:4184',folder=new URL('../../Saved/Validation/Campus/',import.meta.url);await mkdir(folder,{recursive:true});
const frames=[],seen=new Set();let activeWorld=null;let sawLift=false,sawDifferent=false,sawPhysicalLoad=false,sawMeasuredLoad=false,maxPhysicalLoadN=0,maxMeasuredLoadN=0,stopped=false;const begin=Date.now();
while(Date.now()-begin<120000){
 const s=await fetch(base+'/api/snapshot').then(r=>r.json());const e=s.latest,f=e?.frame;
 if(f?.fleet&&/^[a-f0-9]{32}$/i.test(f.source.world_id)&&!seen.has(f.source.world_id+':'+f.source.sequence)){
  if(activeWorld!==f.source.world_id){activeWorld=f.source.world_id;frames.length=0;sawLift=sawDifferent=sawPhysicalLoad=sawMeasuredLoad=false;maxPhysicalLoadN=maxMeasuredLoadN=0;}
  seen.add(f.source.world_id+':'+f.source.sequence);frames.push(e);assert.equal(f.fleet.vehicles.length,3);assert.equal(new Set(f.fleet.vehicles.map(v=>v.vehicle_id)).size,3);
  for(const v of f.fleet.vehicles){assert.equal(v.wheels.length,4);assert.ok(Number.isFinite(v.fork.height_m));assert.equal(selectVehicle(f,v.vehicle_id).vehicle_id,v.vehicle_id);}
  for(const v of f.fleet.vehicles){const actual=(v.fork.tines??[]).reduce((n,t)=>n+(t.vertical_n??0),0);const measured=(v.dynamics_sensors?.sensors??[]).filter(s=>s.valid&&s.id.startsWith('fork_load_')).reduce((n,s)=>n+(s.fork_axis_force_n??0),0);maxPhysicalLoadN=Math.max(maxPhysicalLoadN,actual);maxMeasuredLoadN=Math.max(maxMeasuredLoadN,measured);sawPhysicalLoad||=actual>1000;sawMeasuredLoad||=measured>1000;}
  const h=f.fleet.vehicles.map(v=>v.fork.height_m);sawLift||=Math.max(...h)>.12;sawDifferent||=Math.max(...h)-Math.min(...h)>.1;stopped=f.source.state==='stopped';
  if(frames.length>=80&&sawLift&&sawDifferent&&sawPhysicalLoad&&sawMeasuredLoad)break;
 }
 await new Promise(r=>setTimeout(r,250));
}
assert.ok(frames.length>=10,'Need actual Unreal fleet frames');const csv=fleetCsv(frames);assert.equal(csv.split('\n').length,1+frames.length*3);assert.equal(csv.split('\n')[0].split(',').length,23);assert.equal(vehicleHistory(frames,'AGV-02').every(e=>e.frame.vehicle_id==='AGV-02'),true);
await writeFile(new URL('FleetLiveFrames.json',folder),JSON.stringify(frames));await writeFile(new URL('FleetLive.csv',folder),csv);
const report={pass:frames.length>=10&&sawLift&&sawDifferent&&sawPhysicalLoad&&sawMeasuredLoad,source:'actual Unreal fleet HTTP frames; no synthetic ingest',frames:frames.length,vehicle_rows:frames.length*3,columns:23,saw_lift:sawLift,saw_independent_heights:sawDifferent,saw_physical_load:sawPhysicalLoad,saw_measured_load:sawMeasuredLoad,max_physical_load_n:maxPhysicalLoadN,max_measured_load_n:maxMeasuredLoadN,stopped,worlds:[...new Set(frames.map(e=>e.frame.source.world_id))]};await writeFile(new URL('FleetLiveEvidence.json',folder),JSON.stringify(report,null,2));console.log(report);assert.ok(report.pass,'Need a lifting vehicle and separate idle vehicle measurements');
