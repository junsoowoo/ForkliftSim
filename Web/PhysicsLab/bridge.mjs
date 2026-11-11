import http from 'node:http';
import {readFile} from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const folder=fileURLToPath(new URL('.',import.meta.url));
const projectRoot=path.resolve(folder,'../..');
const allowed=new Map([['/','live.html'],['/index.html','live.html'],['/styles.css','styles.css'],['/live.mjs','live.mjs'],['/live-state.mjs','live-state.mjs'],['/data.mjs','data.mjs']]);
for(const name of ['joint-sensors.mjs','dynamics-sensors.mjs','proximity-sensors.mjs','lift-control.mjs','lidar-safety.mjs','drive-estimator.mjs','pallet-perception.mjs','handling-control.mjs','mission.mjs'])allowed.set('/'+name,name);
export const SERVICE='forkliftsim-live-bridge';
export function validateFrame(frame){
 if(!frame||frame.schema!==1||frame.source?.kind!=='unreal')throw new Error('Expected Unreal schema 1');
 const s=frame.source,validId=v=>typeof v==='string'&&/^[A-Za-z0-9_-]{1,128}$/.test(v);
 if(!validId(s.world_id)||!validId(s.run_id)||!Number.isSafeInteger(s.sequence)||s.sequence<1||!Number.isFinite(s.started_at_ms)||!Number.isFinite(s.sent_at_ms)||!['running','stopped'].includes(s.state))throw new Error('Invalid source metadata');
 if(!Number.isFinite(frame.sim_time_s)||frame.sim_time_s<0||!Number.isFinite(frame.measurement_interval_s)||frame.measurement_interval_s<0||typeof frame.contact_sample_valid!=='boolean')throw new Error('Invalid sampling metadata');
 if(!frame.vehicle||!frame.fork||!frame.hydraulic||!frame.settings||!Array.isArray(frame.wheels)||frame.wheels.length!==4||!Array.isArray(frame.fork.tines)||frame.fork.tines.length>2)throw new Error('Missing telemetry components');
 if(new Set(frame.wheels.map(w=>w.id)).size!==4||frame.wheels.some(w=>!['FL','FR','RL','RR'].includes(w.id)||!Number.isFinite(w.normal_n)))throw new Error('Invalid wheel measurements');
 let nodes=0;function visit(v,depth){if(++nodes>10000||depth>12)throw new Error('Telemetry object too complex');if(typeof v==='number'&&!Number.isFinite(v))throw new Error('Non-finite value');if(typeof v==='string'&&v.length>4096)throw new Error('Text too long');if(v&&typeof v==='object')for(const value of Object.values(v))visit(value,depth+1);}
 visit(frame,0);return frame;
}
export function createTelemetryServer({port=4174,historyLimit=600}={}){
 let latest=null,nextId=0;const clients=new Set(),history=[],sequences=new Map();
 const state=()=>({status:!latest?'waiting':latest.frame.source.state==='stopped'?'stopped':Date.now()-latest.received_at_ms>1500?'stale':'running',age_ms:latest?Date.now()-latest.received_at_ms:null,world_id:latest?.frame.source.world_id??null,last_sequence:latest?.frame.source.sequence??null});
 const send=(client,event,value)=>{if(!client.destroyed&&!client.writableEnded){if(client.writableLength>512*1024){client.destroy();clients.delete(client);return;}client.write(`event: ${event}\ndata: ${JSON.stringify(value)}\n\n`);}};
 const publish=(event,value)=>{for(const client of clients)send(client,event,value);};
 const server=http.createServer(async(req,res)=>{
  const address=server.address(),origin=`http://127.0.0.1:${address.port}`;
  const host=req.headers.host;if(host!==`127.0.0.1:${address.port}`&&host!==`localhost:${address.port}`){res.writeHead(403);res.end('Loopback host required');return;}
  res.setHeader('X-Content-Type-Options','nosniff');res.setHeader('Cache-Control','no-store');
  res.setHeader('Content-Security-Policy',"default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; connect-src 'self'; img-src 'self' data:; object-src 'none'; base-uri 'none'; frame-ancestors 'self'");
  const url=new URL(req.url,origin),json=(status,data)=>{res.writeHead(status,{'Content-Type':'application/json; charset=utf-8'});res.end(JSON.stringify(data));};
  if(req.method==='GET'&&url.pathname==='/api/health'){json(200,{service:SERVICE,schema:1,project_root:projectRoot,pid:process.pid,...state()});return;}
  if(req.method==='GET'&&url.pathname==='/api/snapshot'){json(200,{...state(),latest});return;}
  if(req.method==='GET'&&url.pathname==='/api/history'){json(200,{...state(),frames:history});return;}
  if(req.method==='GET'&&url.pathname==='/events'){
   if(clients.size>=20){json(503,{error:'Too many dashboard clients'});return;}
   res.writeHead(200,{'Content-Type':'text/event-stream; charset=utf-8','Connection':'keep-alive','X-Accel-Buffering':'no'});res.flushHeaders();res.write('retry: 1000\n\n');clients.add(res);
   send(res,'status',state());if(latest)send(res,'frame',latest);req.on('close',()=>clients.delete(res));return;
  }
  if(req.method==='POST'&&url.pathname==='/api/telemetry'){
   if(req.headers.origin&&req.headers.origin!==origin&&req.headers.origin!==origin.replace('127.0.0.1','localhost')){json(403,{error:'Origin rejected'});return;}
   if(!/^application\/json(?:;|$)/i.test(req.headers['content-type']??'')){json(415,{error:'application/json required'});return;}
   let bytes=0;const chunks=[];try{
    for await(const chunk of req){bytes+=chunk.length;if(bytes>262144){json(413,{error:'Telemetry too large'});req.resume();return;}chunks.push(chunk);}
    const frame=validateFrame(JSON.parse(Buffer.concat(chunks).toString('utf8'))),s=frame.source,previous=sequences.get(s.world_id);
    if(previous&&(s.sequence<=previous.sequence||previous.stopped)){json(409,{error:'Old or stopped world sequence'});return;}
    if(latest&&s.world_id!==latest.frame.source.world_id&&s.started_at_ms<=latest.frame.source.started_at_ms){json(409,{error:'An older world cannot replace the active world'});return;}
    sequences.set(s.world_id,{sequence:s.sequence,stopped:s.state==='stopped'});if(sequences.size>128)sequences.delete(sequences.keys().next().value);
    const envelope={event_id:++nextId,received_at_ms:Date.now(),frame};latest=envelope;history.push(envelope);if(history.length>historyLimit)history.shift();
    publish('frame',envelope);publish('status',state());json(202,{accepted_sequence:s.sequence,event_id:nextId});
   }catch(error){if(!res.writableEnded)json(400,{error:error.message});}return;
  }
  if(req.method!=='GET'&&req.method!=='HEAD'){json(405,{error:'Read-only dashboard; POST is only for Unreal telemetry'});return;}
  const name=allowed.get(url.pathname);if(!name){res.writeHead(404);res.end('Not found');return;}
  try{const data=await readFile(path.join(folder,'dist',name));const mime=name.endsWith('.css')?'text/css':name.endsWith('.mjs')?'text/javascript':'text/html';res.writeHead(200,{'Content-Type':`${mime}; charset=utf-8`});res.end(req.method==='HEAD'?undefined:data);}catch{res.writeHead(404);res.end('Not found');}
 });
 server.requestTimeout=3000;server.headersTimeout=5000;
 const heartbeat=setInterval(()=>{publish('status',state());for(const client of clients)client.write(': heartbeat\n\n');},1000);heartbeat.unref();
 return {server,listen:()=>new Promise((resolve,reject)=>{server.once('error',reject);server.listen(port,'127.0.0.1',()=>resolve(server.address().port));}),close:()=>new Promise(resolve=>{clearInterval(heartbeat);for(const client of clients)client.end();server.close(resolve);server.closeAllConnections();})};
}
