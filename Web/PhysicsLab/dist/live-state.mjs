// Presentation of received Unreal values only. No load solver or demo source.
export function measurements(frame){
 const valid=frame?.contact_sample_valid===true;
 const tines=frame?.fork?.tines??[];
 return {speed:frame?.vehicle?.speed_mps??null,pressure:frame?.hydraulic?.pressure_pa??null,height:frame?.fork?.height_m??null,
  forkLeft:valid&&tines[0]?tines[0].vertical_n:null,forkRight:valid&&tines[1]?tines[1].vertical_n:null,
  normalTotal:valid?frame.vehicle.normal_sum_n:null,
  wheels:(frame?.wheels??[]).map(w=>({...w,displayNormal:valid?w.normal_n:null,displayFriction:valid?w.friction_n:null}))};
}
export class LiveState{
 constructor(){this.latest=null;this.history=[];this.sequence=-1;this.world=null;this.run=null;this.started=-Infinity;this.historyCleared=false;}
 accept(envelope){
  const f=envelope?.frame,s=f?.source;
  if(s?.kind!=='unreal'||!Number.isSafeInteger(s.sequence))return false;
  if(this.world===s.world_id&&s.sequence<=this.sequence)return false;
  if(this.world&&this.world!==s.world_id&&s.started_at_ms<=this.started)return false;
  if(this.world!==s.world_id||this.run!==s.run_id)this.history=[];
  this.world=s.world_id;this.run=s.run_id;this.started=s.started_at_ms;this.sequence=s.sequence;this.latest=envelope;
  if(s.state==='running')this.history.push(envelope);
  while(this.history.length>600||(this.history.length>1&&this.history[0].frame.sim_time_s<f.sim_time_s-30))this.history.shift();
  return true;
 }
 hydrate(envelopes){
  if(this.historyCleared)return;
  const ordered=(Array.isArray(envelopes)?envelopes:[]).filter(e=>e?.frame?.source?.kind==='unreal'&&Number.isSafeInteger(e.frame.source.sequence)).slice().sort((a,b)=>a.frame.source.started_at_ms-b.frame.source.started_at_ms||a.frame.source.sequence-b.frame.source.sequence);
  for(const e of ordered)this.accept(e);
  if(!this.latest)return;
  const unique=new Map();
  for(const e of [...ordered,...this.history]){
   const f=e.frame,s=f.source;
   if(s.world_id===this.world&&s.run_id===this.run&&s.state==='running'&&f.sim_time_s>=this.latest.frame.sim_time_s-30&&s.sequence<=this.sequence)unique.set(s.sequence,e);
  }
  this.history=[...unique.values()].sort((a,b)=>a.frame.source.sequence-b.frame.source.sequence).slice(-600);
 }
 clear(){this.history=[];this.historyCleared=true;}
}
