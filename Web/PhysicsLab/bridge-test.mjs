// Protocol/UI-state tests use an explicit synthetic fixture on an ephemeral
// test server. Actual Unreal integration is checked by verify-unreal.mjs.
import assert from 'node:assert/strict';
import {createTelemetryServer,SERVICE} from './bridge.mjs';
import {LiveState,measurements} from './dist/live-state.mjs';
let checks=0;const check=(label,fn)=>{fn();checks++;console.log(`PASS ${label}`);};
const app=createTelemetryServer({port:0,historyLimit:20}),port=await app.listen(),base=`http://127.0.0.1:${port}`;
const fixture=(sequence=1)=>({schema:1,source:{kind:'unreal',world_id:'protocol_fixture_world',run_id:'run_a',started_at_ms:Date.now()-10000,sent_at_ms:Date.now(),sequence,state:'running'},sim_time_s:sequence,measurement_interval_s:1/60,contact_sample_valid:true,vehicle:{speed_mps:.123,normal_sum_n:11111},settings:{},hydraulic:{pressure_pa:1234567},fork:{height_m:.123,tines:[{vertical_n:111},{vertical_n:333}]},wheels:['FL','FR','RL','RR'].map(id=>({id,normal_n:2222,friction_n:12}))});
const post=(frame,headers={})=>fetch(`${base}/api/telemetry`,{method:'POST',headers:{'Content-Type':'application/json',...headers},body:JSON.stringify(frame)});
let streamAbort=new AbortController();
try{
 const health=await fetch(`${base}/api/health`).then(r=>r.json());check('Loopback health identifies live bridge',()=>assert.equal(health.service,SERVICE));
 const before=await fetch(`${base}/api/snapshot`).then(r=>r.json());check('No generated data before Unreal connects',()=>{assert.equal(before.status,'waiting');assert.equal(before.latest,null);});
 for(const asset of ['/','/live.mjs','/live-state.mjs','/data.mjs','/styles.css','/joint-sensors.mjs','/dynamics-sensors.mjs','/proximity-sensors.mjs','/lift-control.mjs','/lidar-safety.mjs','/drive-estimator.mjs','/pallet-perception.mjs','/handling-control.mjs','/mission.mjs']){const r=await fetch(base+asset);check(`Live asset ${asset}`,()=>assert.equal(r.status,200));}
 const denied=await fetch(`${base}/physics.mjs`);check('Former browser physics calculator is not served',()=>assert.equal(denied.status,404));
 const stream=await fetch(`${base}/events`,{signal:streamAbort.signal}),reader=stream.body.getReader();let eventBuffer='';
 const frame=fixture();assert.equal((await post(frame)).status,202);
 while(!eventBuffer.includes('event: frame')){const {value,done}=await reader.read();if(done)throw new Error('SSE ended before frame');eventBuffer+=new TextDecoder().decode(value);}
 check('SSE transports accepted Unreal frame',()=>assert.ok(eventBuffer.includes('"pressure_pa":1234567')));
 const received=await fetch(`${base}/api/snapshot`).then(r=>r.json()),model=new LiveState();check('Browser state accepts received frame',()=>assert.equal(model.accept(received.latest),true));
 const values=measurements(received.latest.frame);check('UI values are exact received measurements, not recalculated loads',()=>{assert.equal(values.forkLeft,111);assert.equal(values.forkRight,333);assert.equal(values.pressure,1234567);assert.equal(values.normalTotal,11111);});
 check('Contact forces unavailable without a valid physics interval',()=>assert.equal(measurements({...frame,contact_sample_valid:false}).forkLeft,null));
 check('Browser rejects duplicate/out-of-order sequence',()=>assert.equal(model.accept(received.latest),false));
 const repeated=await post(frame);check('Bridge rejects out-of-order sequence',()=>assert.equal(repeated.status,409));
 const reset=structuredClone(frame);reset.source.sequence=2;reset.source.run_id='run_b';reset.sim_time_s=0;
 assert.equal((await post(reset)).status,202);const afterReset=await fetch(`${base}/api/snapshot`).then(r=>r.json());model.accept(afterReset.latest);
 check('Reset run has a separate history, even when simulation time rewinds',()=>{assert.equal(model.history.length,1);assert.equal(model.run,'run_b');});
 const invalid=structuredClone(frame);invalid.source.kind='demo';const bad=await post(invalid);check('No demo/calculator source accepted',()=>assert.equal(bad.status,400));
 const crossOrigin=await post(reset,{Origin:'https://other.example'});check('Cross-origin ingest rejected',()=>assert.equal(crossOrigin.status,403));
 const plain=await post(reset,{'Content-Type':'text/plain'});check('Plain form ingest rejected',()=>assert.equal(plain.status,415));
 const control=await fetch(`${base}/api/control`,{method:'POST'});check('No web control endpoint',()=>assert.equal(control.status,405));
 await new Promise(resolve=>setTimeout(resolve,1600));const stale=await fetch(`${base}/api/snapshot`).then(r=>r.json());check('Paused/disconnected source becomes stale',()=>assert.equal(stale.status,'stale'));
 const stop=structuredClone(reset);stop.source.sequence=3;stop.source.state='stopped';assert.equal((await post(stop)).status,202);const stopped=await fetch(`${base}/api/snapshot`).then(r=>r.json());check('Play stop is explicit',()=>assert.equal(stopped.status,'stopped'));
 const resumed=structuredClone(reset);resumed.source.sequence=4;const noResume=await post(resumed);check('Stopped world cannot resume and masquerade as a new Play',()=>assert.equal(noResume.status,409));
 const newer=fixture(1);newer.source.world_id='newer_world';newer.source.started_at_ms=Date.now();assert.equal((await post(newer)).status,202);
 const old=structuredClone(frame);old.source.sequence=8;const oldResponse=await post(old);check('Older PIE world cannot replace newer world',()=>assert.equal(oldResponse.status,409));
 console.log(`LIVE_BRIDGE_CHECK PASS ${checks} checks (synthetic protocol fixture, not Unreal physics proof)`);
}finally{streamAbort.abort();await app.close();}
