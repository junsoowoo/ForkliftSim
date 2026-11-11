import test from 'node:test';
import assert from 'node:assert/strict';
import {LiveState} from './dist/live-state.mjs';
// Protocol-only fixtures. The running app hydrates from its real loopback API.
const frame=(n,{world='w',run='r',start=1,time=n,status='running'}={})=>({received_at_ms:n,frame:{sim_time_s:time,source:{kind:'unreal',sequence:n,world_id:world,run_id:run,started_at_ms:start,state:status}}});
test('A stopped SSE snapshot does not prevent restoring real earlier history',()=>{
 const s=new LiveState(),last=frame(4,{status:'stopped'});s.accept(last);s.hydrate([frame(1),frame(2),frame(3),last]);
 assert.equal(s.latest,last);assert.deepEqual(s.history.map(e=>e.frame.source.sequence),[1,2,3]);s.hydrate([frame(2),frame(3),last]);assert.equal(s.history.length,3);
});
test('A late old-world response cannot rewind current live data',()=>{
 const s=new LiveState(),latest=frame(2,{world:'new',start:100});s.accept(latest);s.hydrate([frame(50,{world:'old',start:1}),frame(51,{world:'old',start:1,status:'stopped'})]);
 assert.equal(s.latest,latest);assert.equal(s.history.length,1);assert.equal(s.history[0].frame.source.world_id,'new');
});
test('Hydration respects the current run, retention bound, and explicit clear',()=>{
 const s=new LiveState(),latest=frame(1000,{run:'new',time:100,status:'stopped'});s.accept(latest);
 s.hydrate([frame(10,{run:'old',time:10}),...Array.from({length:900},(_,i)=>frame(i+100,{run:'new',time:60+i/30})),latest]);
 assert(s.history.length<=600);assert(s.history.every(e=>e.frame.source.run_id==='new'&&e.frame.sim_time_s>=70));
 s.clear();s.hydrate([frame(999,{run:'new',time:99}),latest]);assert.equal(s.history.length,0);assert.equal(s.latest,latest);
});
