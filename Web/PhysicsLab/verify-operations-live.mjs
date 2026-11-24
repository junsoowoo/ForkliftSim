// Read only: captures actual native suite data; never submits synthetic telemetry.
import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {fleetCsv} from './dist/fleet.mjs';
const base=process.argv[2]??'http://127.0.0.1:4184',folder=new URL('../../Saved/Validation/Operations/',import.meta.url),start=Date.now();await mkdir(folder,{recursive:true});
const minStarted=Number(process.argv[3]??0);
const flags={six:false,charging:false,charge_complete:false,cross_wait:false,cross_overlap:false,whole:false,sections:false,critical:false,charge_fault:false};const samples=[],last=new Map();let six=null,charge=null;
while(Date.now()-start<900000){
 const s=await fetch(base+'/api/snapshot').then(r=>r.json()),f=s.latest?.frame,b=f?.fleet?.task_board,e=b?.energy,c=b?.qualification_case;
 if(c?.startsWith('ops_')&&e&&f.source.started_at_ms>=minStarted){
  assert.equal(e.source,'unreal_motor_hydraulic_energy_v1');
  for(const v of e.vehicles){assert.ok(Number.isFinite(v.soc)&&v.soc>=0&&v.soc<=1);assert.ok(Math.abs(v.soc*e.capacity_wh-v.stored_wh)<.001);}
  if((last.get(c)??-2)<b.sim_s-1){last.set(c,b.sim_s);samples.push({source:f.source,case:c,sim_s:b.sim_s,status:s.status,kpi:b.kpi,energy:e,peak_moving:b.peak_measured_moving,waits:f.fleet.traffic_waits});}
  if(c==='ops_parallel_whole')flags.whole||=b.peak_measured_moving===1;
  if(c==='ops_parallel_sections')flags.sections||=b.peak_measured_moving>=3;
  if(c==='ops_six'&&f.fleet.vehicles.length===6&&e.vehicles.length===6&&b.tasks.filter(t=>t.assigned_s>=0).length===6&&b.kpi.distance_est_m>.15){flags.six=true;six=s.latest;}
  if(c==='ops_cross'){flags.cross_wait||=f.fleet.traffic_waits.length>0;flags.cross_overlap||=b.peak_measured_moving>=2;}
  if(c==='ops_charge'){flags.charging||=e.vehicles.some(v=>v.charging&&v.charged_wh>0);flags.charge_complete||=e.completed_charges===1&&b.tasks.some(t=>t.assigned_s>=0&&t.distance_est_m>.15);if(e.completed_charges===1)charge=s.latest;}
  if(c==='ops_charge_interlock')flags.charge_fault||=e.vehicles.some(v=>v.charged_wh>0&&!v.charging)&&f.fleet.vehicles.some(v=>v.campus_job?.state==='fault');
  if(c==='ops_critical')flags.critical||=e.vehicles.some(v=>v.critical_latched)&&b.tasks.some(t=>t.state==='pending');
  if(Object.values(flags).every(Boolean)){
   const csv=fleetCsv([six]);assert.equal(csv.split('\n').length,7);assert.equal(csv.split('\n')[0].split(',').length,29);
   await writeFile(new URL('WebOperationsSamples.json',folder),JSON.stringify(samples));await writeFile(new URL('WebSixFrame.json',folder),JSON.stringify(six));await writeFile(new URL('WebChargeFrame.json',folder),JSON.stringify(charge));await writeFile(new URL('WebSix.csv',folder),csv);
   const proof={pass:true,minimum_source_started_at_ms:minStarted,world_ids:[...new Set(samples.map(s=>s.source.world_id))],source:'Actual Unreal native operations suite HTTP frames; read-only verifier',flags,samples:samples.length,six_vehicle_rows:6,csv_columns:29};await writeFile(new URL('WebEvidence.json',folder),JSON.stringify(proof,null,2));console.log(proof);process.exit(0);
  }
 }
 await new Promise(r=>setTimeout(r,200));
}
throw new Error('Actual operations HTTP coverage timed out: '+JSON.stringify(flags));
