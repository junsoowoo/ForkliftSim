import {createTelemetryServer} from './bridge.mjs';
const index=process.argv.indexOf('--port'),port=Number(index>=0?process.argv[index+1]:process.env.FORKLIFTSIM_WEB_PORT??4174);
if(!Number.isInteger(port)||port<0||port>65535)throw new Error('Invalid port');
const app=createTelemetryServer({port});
try{const actual=await app.listen();console.log(`ForkliftSim live bridge: http://127.0.0.1:${actual}`);}
catch(error){console.error(error.code==='EADDRINUSE'?`Port ${port} is occupied. Use Start.bat -WebPort <another port>.`:error.message);process.exitCode=1;}
for(const signal of ['SIGINT','SIGTERM'])process.on(signal,async()=>{await app.close();process.exit(0);});
