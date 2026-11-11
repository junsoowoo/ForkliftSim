// SI reference calculations. This browser model is deliberately independent
// of Chaos; 4 equal vertical springs close the statically indeterminate system.
export const REFERENCE=Object.freeze({g:9.81,dryMass:2170,palletMass:25,wheelbase:1.6,track:1.3,radius:.3,tyreK:600000,crr:.015,bearingCoulomb:1,bearingViscous:.05,forkMass:150,area:.002,relief:7.5e6,pumpW:1200,efficiency:.8,maxFlow:.0005,seal:80,viscous:400});
export const DEFAULTS=Object.freeze({mass:400,offsetX:0,offsetY:0,depth:1.1,height:.45,forkSpacing:.6,holeShift:0,speed:0,turnRadius:3,brake:0,slope:0,tyreMu:.9,roadMu:.8,cargoMu:.6,cargoDynamicMu:.5,forkMu:.45});
export const WHEELS=Object.freeze([{id:'FL',name:'앞 왼쪽',x:.8,y:-.65},{id:'FR',name:'앞 오른쪽',x:.8,y:.65},{id:'RL',name:'뒤 왼쪽',x:-.8,y:-.65},{id:'RR',name:'뒤 오른쪽',x:-.8,y:.65}]);
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
function solve3(matrix,values){
  const a=matrix.map((row,i)=>[...row,values[i]]);
  for(let c=0;c<3;c++){
    let pivot=c;for(let r=c+1;r<3;r++)if(Math.abs(a[r][c])>Math.abs(a[pivot][c]))pivot=r;
    if(Math.abs(a[pivot][c])<1e-10)return null;
    [a[c],a[pivot]]=[a[pivot],a[c]];
    const div=a[c][c];for(let j=c;j<4;j++)a[c][j]/=div;
    for(let r=0;r<3;r++)if(r!==c){const f=a[r][c];for(let j=c;j<4;j++)a[r][j]-=f*a[c][j];}
  }return a.map(row=>row[3]);
}
export function supportLoads(total,momentX,momentY){
  let active=[0,1,2,3];
  for(let attempt=0;attempt<3;attempt++){
    if(active.length<3)return {valid:false,loads:[null,null,null,null]};
    const basis=active.map(i=>[1,WHEELS[i].x,WHEELS[i].y]);
    const matrix=[0,1,2].map(r=>[0,1,2].map(c=>basis.reduce((n,b)=>n+b[r]*b[c],0)));
    const p=solve3(matrix,[total,momentX,momentY]);
    if(!p)return {valid:false,loads:[null,null,null,null]};
    const loads=WHEELS.map((w,i)=>active.includes(i)?p[0]+p[1]*w.x+p[2]*w.y:0);
    const least=Math.min(...active.map(i=>loads[i]));
    if(least>=-1e-6)return {valid:true,loads:loads.map(n=>Math.max(0,n)),contacts:active.length};
    active=active.filter(i=>loads[i]!==least);
  }return {valid:false,loads:[null,null,null,null]};
}
export function calculate(input={},motion={}){
  const p={...DEFAULTS,...input},R=REFERENCE;
  for(const key of Object.keys(DEFAULTS))if(!Number.isFinite(p[key]))throw new Error(`Invalid number: ${key}`);
  if(p.mass<0||p.mass>1000||p.depth<0||p.depth>1.2||p.height<0||p.height>1.5||p.turnRadius<=0||p.forkSpacing<=0||Math.min(p.tyreMu,p.roadMu,p.cargoMu,p.cargoDynamicMu,p.forkMu)<0)throw new Error('Unsupported input range');
  const aLift=motion.liftAcceleration??0,theta=p.slope*Math.PI/180,gn=R.g*Math.cos(theta);
  const ax=-p.brake,ay=p.speed*p.speed/p.turnRadius;
  const demandX=ax+R.g*Math.sin(theta),demand=Math.hypot(demandX,ay);
  const loadMass=p.mass+R.palletMass;
  const localLoadX=1.4-p.depth+.6+p.offsetX*p.mass/loadMass;
  const localLoadY=p.offsetY*p.mass/loadMass;
  const bladeY=[-p.forkSpacing/2,p.forkSpacing/2];
  const fits=bladeY.map((y,i)=>Math.abs(y-(p.holeShift+(i===0?-.3:.3)))+.06<=.16+1e-9);
  const overlap=Math.min(1.2,p.depth);
  const lengthSupported=overlap>.04&&localLoadX<=1.4+1e-8&&localLoadX>=Math.max(0,1.4-p.depth)-1e-8;
  const leftShare=.5-localLoadY/p.forkSpacing;
  const lateralSupported=leftShare>=0&&leftShare<=1;
  const canLift=fits.every(Boolean)&&lengthSupported&&lateralSupported;
  const carried=canLift?clamp(p.height/.03,0,1):0;
  // Settled geometry approximation derived from the source's welded masses.
  const mast=.8+Math.hypot(.3,.09)+.1,carriageX=mast+.28;
  const rootZ=.79, chassisX=(-1080*.2+80*mast)/1800;
  const chassisZ=rootZ+(-1080*.25+80*.75)/1800;
  const bodies=[{m:1800,x:chassisX,y:0,z:chassisZ},{m:220,x:0,y:0,z:.3},
    {m:90,x:carriageX,y:0,z:.205+p.height,lifting:true},{m:60,x:carriageX+.65,y:0,z:.055+p.height,lifting:true},
    {m:R.palletMass*carried,x:carriageX+1.35-p.depth+.6,y:0,z:.13+p.height,lifting:true},
    {m:p.mass*carried,x:carriageX+1.35-p.depth+.6+p.offsetX,y:p.offsetY,z:.532+p.height,lifting:true}];
  const totalMass=bodies.reduce((s,b)=>s+b.m,0);
  const center={x:0,y:0,z:0};let normal=0,mx=0,my=0;
  for(const b of bodies){for(const d of ['x','y','z'])center[d]+=b.m*b[d]/totalMass;
    const nz=b.m*(gn+(b.lifting?aLift:0));normal+=nz;mx+=nz*b.x-b.m*b.z*demandX;my+=nz*b.y-b.m*b.z*ay;}
  const supports=supportLoads(normal,mx,my),mu=Math.min(p.tyreMu,p.roadMu);
  const tyreLimit=mu*normal,tractionDemand=totalMass*demand;
  const forkNormal=loadMass*carried*Math.max(0,gn+aLift);
  const forkForces=[forkNormal*clamp(leftShare,0,1),forkNormal*(1-clamp(leftShare,0,1))];
  const forkMoment=forkNormal*localLoadX;
  const cylinderForce=(R.forkMass+loadMass*carried)*Math.max(0,gn+aLift);
  const pressure=cylinderForce/R.area;
  const force= (R.forkMass+loadMass*carried)*gn+R.seal;
  const liftSpeed=Math.min(R.maxFlow/R.area,2*R.pumpW*R.efficiency/(force+Math.sqrt(force*force+4*R.viscous*R.pumpW*R.efficiency)));
  const cargoDemand=p.mass*demand,cargoLimit=p.cargoMu*p.mass*Math.max(0,gn+aLift);
  const forkDemand=loadMass*carried*demand,forkLimit=p.forkMu*forkNormal;
  const omega=p.speed/R.radius;
  const wheelData=WHEELS.map((w,i)=>{const n=supports.loads[i];return {...w,n,rolling:n===null?null:R.crr*n,bearing:R.bearingCoulomb*clamp(omega/.5,0,1)+R.bearingViscous*omega,compression:n===null?null:Math.max(0,n-51*gn)/R.tyreK};});
  const warnings=[];
  if(!fits.every(Boolean))warnings.push({level:'danger',title:'포크와 팔레트 구멍이 어긋났습니다',text:'현재 포크 간격·구멍 위치에서는 포크가 삽입될 수 없습니다. 웹에서는 화물을 지면 지지로 처리합니다.'});
  else if(!lengthSupported)warnings.push({level:'danger',title:'화물 중심까지 포크가 들어가지 않았습니다',text:'앞뒤 지지 구간 밖에 무게중심이 있습니다. 수평 양중의 정적 평형이 성립하지 않습니다.'});
  else if(!lateralSupported)warnings.push({level:'danger',title:'무게중심이 좌우 포크 지지 범위를 벗어났습니다',text:'한쪽 포크에 음의 지지력이 필요한 조건입니다. 수평 지지를 가정할 수 없습니다.'});
  if(!supports.valid)warnings.push({level:'danger',title:'바퀴 지지 평형이 성립하지 않습니다',text:'필요한 하중 중심이 지지 영역을 벗어났습니다. 실제 전도 거동은 Unreal에서 확인해야 합니다.'});
  if(tractionDemand>tyreLimit+.01)warnings.push({level:'danger',title:'타이어 접지 한계를 초과했습니다',text:'요구되는 수평 힘이 μN보다 큽니다. 입력한 감속·선회를 그대로 유지할 수 없습니다.'});
  if(carried>0&&cargoDemand>cargoLimit+.01)warnings.push({level:'warning',title:'화물이 미끄러질 수 있습니다',text:'팔레트–화물의 정지 마찰 한계를 초과했습니다. 실제 미끄럼 거리는 이 계산에서 예측하지 않습니다.'});
  if(carried>0&&forkDemand>forkLimit+.01)warnings.push({level:'warning',title:'포크–팔레트 마찰 여유가 부족합니다',text:'단순 접촉 마찰 한계입니다. 구멍 측벽의 지지·충돌 반력은 포함하지 않습니다.'});
  if(pressure>R.relief)warnings.push({level:'danger',title:'유압 릴리프 압력을 초과했습니다',text:'요구 압력이 설정 한계 7.5 MPa를 넘습니다.'});
  return {p,loadMass,totalMass,center,normal,carried,canLift,overlap,fits,localLoadX,localLoadY,leftShare,forkForces,forkNormal,forkMoment,pressure,liftSpeed,mu,tyreLimit,tractionDemand,cargoDemand,cargoLimit,forkDemand,forkLimit,ay,ax,supports,supportPoint:{x:mx/normal,y:my/normal},wheelData,warnings,rollingForce:supports.valid?R.crr*normal:null,cargoDynamicLimit:p.cargoDynamicMu*p.mass*gn};
}
