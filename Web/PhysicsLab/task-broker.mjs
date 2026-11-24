import {createHash} from 'node:crypto';
export class TaskBroker {
 constructor(){this.requests=new Map();}
 expire(latest){for(const r of this.requests.values())if(r.status==='pending_unreal'&&(Date.now()>r.command.expires_at_ms||latest?.frame.source.world_id!==r.command.world_id||latest?.frame.source.state==='stopped')){r.status='unconfirmed_expired';r.reason='No current Unreal acknowledgement; check actual task list before retrying.';}}
 observe(frame){for(const ack of frame.fleet?.task_board?.command_receipts??[]){const r=this.requests.get(ack.request_id);if(r&&r.command.world_id===frame.source.world_id&&r.command.run_id===frame.fleet.task_board.run_id&&['accepted','rejected'].includes(ack.status)){r.status=ack.status;r.reason=ack.reason;r.task_ids=ack.task_ids;}}}
 submit(body,latest,status,kind='enqueue'){
  const board=latest?.frame.fleet?.task_board;
  if(status!=='running'||!board?.enabled)throw Object.assign(new Error('Current running Unreal task board required'),{status:409});
  if(body.world_id!==latest.frame.source.world_id||body.run_id!==board.run_id)throw Object.assign(new Error('Play session changed; refresh before submitting'),{status:409});
  if(typeof body.request_id!=='string'||!/^[A-Za-z0-9_-]{8,100}$/.test(body.request_id))throw new Error('Invalid request ID');
  const command={request_id:body.request_id,world_id:body.world_id,run_id:body.run_id,kind};
  if(kind==='enqueue'){
   if(!Array.isArray(body.tasks)||!body.tasks.length||body.tasks.length>10)throw new Error('Register 1–10 tasks per batch');
   const seen=new Set(),known=new Set((board.loads??[]).map(l=>l.id)),slots=new Set(latest.frame.fleet.slots.map(s=>s.id));
   command.tasks=body.tasks.map(t=>{if(!t||!known.has(t.load_id)||seen.has(t.load_id)||typeof t.destination!=='string'||(t.destination&&!slots.has(t.destination))||!Number.isInteger(t.priority)||t.priority<0||t.priority>9||![1,2].includes(t.tier))throw new Error('Invalid or duplicate load / destination / priority / tier');seen.add(t.load_id);return {load_id:t.load_id,destination:t.destination,priority:t.priority,tier:t.tier};});
  }else{if(typeof body.task_id!=='string'||!/^TASK-[0-9]{4,}$/.test(body.task_id))throw new Error('Invalid task ID');command.task_id=body.task_id;}
  const fingerprint=createHash('sha256').update(JSON.stringify(command)).digest('hex'),old=this.requests.get(body.request_id);
  if(old){if(old.fingerprint!==fingerprint)throw Object.assign(new Error('Request ID reused with different content'),{status:409});return old;}
  this.expire(latest);if([...this.requests.values()].filter(r=>r.status==='pending_unreal').length>=32)throw Object.assign(new Error('Too many unacknowledged requests'),{status:429});
  if(this.requests.size>=512){const removable=[...this.requests].find(([,r])=>r.status!=='pending_unreal');if(removable)this.requests.delete(removable[0]);else throw new Error('Request history full');}
  command.expires_at_ms=Date.now()+15000;const r={request_id:body.request_id,status:'pending_unreal',reason:'Waiting for actual Unreal registration',task_ids:[],command,fingerprint};this.requests.set(r.request_id,r);return r;
 }
 poll(world,run,latest){this.expire(latest);if(world!==latest?.frame.source.world_id||run!==latest?.frame.fleet?.task_board?.run_id)return [];return [...this.requests.values()].filter(r=>r.status==='pending_unreal'&&r.command.world_id===world&&r.command.run_id===run).slice(0,8).map(r=>r.command);}
 list(latest){this.expire(latest);return [...this.requests.values()].filter(r=>r.command.world_id===latest?.frame.source.world_id).map(({request_id,status,reason,task_ids})=>({request_id,status,reason,task_ids}));}
}
