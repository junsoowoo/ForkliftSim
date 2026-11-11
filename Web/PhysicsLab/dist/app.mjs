import {calculate,DEFAULTS,REFERENCE,WHEELS} from './physics.mjs';
import {features,calibration,integrated} from './data.mjs';
const $=s=>document.querySelector(s),$$=s=>[...document.querySelectorAll(s)];
const f=(n,d=0)=>Number.isFinite(n)?n.toLocaleString('ko-KR',{minimumFractionDigits:d,maximumFractionDigits:d}):'—';
const clamp=(n,a,b)=>Math.max(a,Math.min(b,n));
let params={...DEFAULTS},view='iso',activeFeature='payload',animation=null,animationFrame=0,liftAcceleration=0;
const controls={load:[
 ['mass','화물 질량',0,1000,25,'kg',0,'팔레트 25 kg은 별도로 더합니다.'],
 ['height','포크 상승 높이',0,1.5,.01,'m',2,'현재 위치를 변경하거나 E / Q로 움직입니다.'],
 ['depth','포크 삽입 깊이',0,1.2,.01,'m',2,'포크 끝에서 팔레트 입구까지 들어간 길이'],
 ['offsetY','화물 좌우 편심',-.25,.25,.01,'m',2,'− 왼쪽 · + 오른쪽'],
 ['offsetX','화물 전후 편심',-.25,.25,.01,'m',2,'− 차체 쪽 · + 포크 끝 쪽'],
 ['forkSpacing','좌우 포크 간격',.4,.8,.01,'m',2,'구멍 중심 간격은 0.60 m입니다.'],
 ['holeShift','팔레트 구멍 좌우 이동',-.15,.15,.01,'m',2,'구멍 폭 0.32 m / 포크 폭 0.12 m']
],motion:[
 ['speed','운반 속도',0,3,.05,'m/s',2,'입력한 선회 반경에서 요구 하중을 계산합니다.'],
 ['turnRadius','우선회 반경',1,8,.1,'m',1,'횡가속도 = 속도² / 반경'],
 ['brake','요구 감속도',0,4,.1,'m/s²',1,'감속이 클수록 앞바퀴 하중이 증가합니다.'],
 ['slope','노면 경사',-10,10,.5,'°',1,'+ 오르막 · − 내리막 / 준정적 경사 하중']
],friction:[
 ['tyreMu','타이어 마찰계수',0,1.2,.01,'μ',2,'노면과 Min 규칙으로 결합합니다.'],
 ['roadMu','노면 마찰계수',0,1.2,.01,'μ',2,'높은 접지력과 구름저항은 별개입니다.'],
 ['cargoMu','팔레트–화물 정지 마찰',0,1,.01,'μs',2,'화물이 미끄러지기 전의 한계'],
 ['cargoDynamicMu','팔레트–화물 운동 마찰',0,1,.01,'μk',2,'미끄러짐 중 힘의 크기. 정지 μ 이하로 제한'],
 ['forkMu','포크–팔레트 마찰',0,1,.01,'μ',2,'측벽 충돌을 제외한 접촉 마찰 근사']
]};
const fieldMap=Object.fromEntries(Object.values(controls).flat().map(c=>[c[0],c]));
$('#controls').innerHTML=Object.entries(controls).map(([group,fields])=>`<div class="control-group" role="tabpanel" aria-labelledby="tab-${group}" id="control-${group}" ${group==='load'?'':'hidden'}>${fields.map(([key,label,min,max,step,unit,digits,help],i)=>`${i===5?'<p class="control-section-label">접촉 형상 / GEOMETRY</p>':''}<div class="control"><div class="control-header"><label for="range-${key}">${label}</label><span class="input-value"><input id="number-${key}" type="number" min="${min}" max="${max}" step="${step}" data-field="${key}" aria-label="${label} 숫자 입력"><span>${unit}</span></span></div><input id="range-${key}" type="range" min="${min}" max="${max}" step="${step}" data-field="${key}" aria-label="${label}"><div class="range-hints"><span>${min}</span><span>${max} ${unit}</span></div><p class="control-help">${help}</p></div>`).join('')}</div>`).join('');
function syncInputs(){for(const [key,c] of Object.entries(fieldMap)){const range=$(`#range-${key}`),number=$(`#number-${key}`);range.value=params[key];range.style.setProperty('--fill',`${(params[key]-c[2])/(c[3]-c[2])*100}%`);if(document.activeElement!==number)number.value=params[key].toFixed(c[6]);}}
function toast(message){$('#toast').textContent=message;$('#toast').classList.add('visible');clearTimeout(toast.timer);toast.timer=setTimeout(()=>$('#toast').classList.remove('visible'),2300);}
function stopAnimation(){if(animationFrame)cancelAnimationFrame(animationFrame);animationFrame=0;animation=null;liftAcceleration=0;$('#replay').textContent='▷ 양중 → 운반 → 제동 재생';$('#operation-state').textContent='직접 포크를 조작해 보세요';}
function render(){
 const m=calculate(params,{liftAcceleration});syncInputs();
 $('#metrics').innerHTML=[['장비가 지지하는 질량',f(m.totalMass),'kg',m.carried<1?'화물 일부 / 전부는 지면 지지':'기본 장비 2,170 + 적재 425 kg'.replace('425',f(m.loadMass)),'▱'],['좌우 포크 지지력',f(m.forkNormal/1000,2),'kN',`L ${f(m.forkForces[0]/1000,2)} / R ${f(m.forkForces[1]/1000,2)} kN`,'⇅'],['실린더 요구 압력',f(m.pressure/1e6,2),'MPa',`릴리프 한계 ${f(REFERENCE.relief/1e6,1)} MPa`,'◴'],['타이어–노면 접지',f(m.mu,2),'μ',`min(${f(params.tyreMu,2)}, ${f(params.roadMu,2)})`,'⊞']].map(([label,value,unit,foot,icon])=>`<article class="metric"><div class="metric-top"><span>${label}</span><span class="metric-icon">${icon}</span></div><p class="metric-value">${value}<small>${unit}</small></p><p class="metric-foot"><i>↳</i>${foot}</p></article>`).join('');
 $('#scene-state').textContent=!m.canLift?'삽입 / 지지 조건 확인 필요':m.carried>.99?'팔레트 지지 중':m.carried>0?'지면에서 포크로 하중 인계 중':'화물은 지면 지지 중';
 $('#scene-facts').innerHTML=[['포크 삽입 겹침',`${f(m.overlap,2)} m`],['결합 무게중심 높이',`${f(m.center.z,2)} m`],['유량·출력 기준 상승 속도',`${f(m.liftSpeed,3)} m/s`]].map(([label,value])=>`<div class="scene-fact">${label}<strong>${value}</strong></div>`).join('');
 const peak=Math.max(1,...m.wheelData.map(w=>w.n??0));
 $('#wheel-loads').innerHTML=m.wheelData.map(w=>`<div class="wheel-load"><div class="wheel-top"><code>${w.id}</code><span>${w.name}</span></div><p class="wheel-figure">${f(w.n===null?null:w.n/1000,2)}<small>kN</small></p><div class="load-track"><i style="width:${w.n===null?0:w.n/peak*100}%"></i></div><div class="wheel-sub"><span>눌림 ${f(w.compression===null?null:w.compression*1000,1)} mm</span><span>구름 ${f(w.rolling)} N</span></div></div>`).join('');
 $('#rolling-total').textContent=`${f(m.rollingForce,1)} N`;
 $('#balance-error').textContent=m.supports.valid?`${f(Math.abs(m.wheelData.reduce((n,w)=>n+w.n,0)-m.normal),3)} N`:'평형 불성립';
 $('#load-path').innerHTML=[['01','화물 + 팔레트',`${f(m.loadMass)} <small>kg</small>`],['02','좌 / 우 포크',`${f(m.forkForces[0]/1000,2)} / ${f(m.forkForces[1]/1000,2)} <small>kN</small>`],['03','포크 뿌리의 적재 모멘트',`${f(m.forkMoment/1000,2)} <small>kNm</small>`],['04','바퀴가 바닥에서 받는 수직력',`${f(m.normal/1000,2)} <small>kN</small>`]].map(([n,label,value])=>`<div class="path-step"><span class="path-number">${n}</span><span>${label}</span><strong>${value}</strong></div>`).join('')+`<p class="path-caption">포크 지지력에는 적재물만, 실린더 요구 압력에는 포크 자체 질량 150 kg도 포함합니다.</p>`;
 $('#friction-budgets').innerHTML=[['타이어 ↔ 노면',m.tractionDemand,m.tyreLimit],['팔레트 ↔ 화물',m.carried?m.cargoDemand:0,m.carried?m.cargoLimit:0],['포크 ↔ 팔레트',m.forkDemand,m.forkLimit]].map(([label,need,limit])=>`<div class="budget ${need>limit+.01?'over':''}"><div class="budget-header"><span>${label}</span><strong>${need>limit+.01?'한계 초과':limit?'여유 있음':'지지 없음'}</strong></div><div class="budget-track"><i style="width:${Math.min(100,limit?need/limit*100:need?100:0)}%"></i></div><div class="budget-foot"><span>요구 ${f(need)} N</span><span>한계 ${f(limit)} N</span></div></div>`).join('')+`<p class="path-caption">선회 횡가속도 ${f(m.ay,2)} m/s² · 미끄러짐 중 화물 마찰 크기 ${f(m.cargoDynamicLimit)} N</p>`;
 $('#warnings').innerHTML=m.warnings.length?m.warnings.map(w=>`<div class="warning ${w.level}"><span class="warning-symbol">!</span><div><h3>${w.title}</h3><p>${w.text}</p></div></div>`).join(''):`<div class="warning good"><span class="warning-symbol">✓</span><div><h3>${m.carried?'현재 입력에서 정적 지지가 가능합니다':'현재 화물은 지면에서 지지됩니다'}</h3><p>표시된 힘은 웹 계산 모델의 결과입니다. 충돌·실제 화물 이동은 Unreal에서 확인할 수 있습니다.</p></div></div>`;
 $('#scene').innerHTML=view==='iso'?isoScene(m):planScene(m);
}
const project=(x,y,z=0)=>[260+x*105-y*75,320+x*30+y*41-z*125];
const pts=a=>a.map(p=>p.map(n=>n.toFixed(2)).join(',')).join(' ');
function polygon(a,fill,stroke='#ffffff25'){return `<polygon points="${pts(a)}" fill="${fill}" stroke="${stroke}" stroke-width=".7"/>`;}
function cube(x,y,z,l,w,h,colors){const p=(a,b,c)=>project(a,b,c);const [top,front,side]=colors;return polygon([p(x,y,z+h),p(x+l,y,z+h),p(x+l,y+w,z+h),p(x,y+w,z+h)],top)+polygon([p(x,y+w,z),p(x+l,y+w,z),p(x+l,y+w,z+h),p(x,y+w,z+h)],front)+polygon([p(x+l,y,z),p(x+l,y+w,z),p(x+l,y+w,z+h),p(x+l,y,z+h)],side);}
function line(a,b,color,width=1,dash=''){return `<line x1="${a[0]}" y1="${a[1]}" x2="${b[0]}" y2="${b[1]}" stroke="${color}" stroke-width="${width}" ${dash?`stroke-dasharray="${dash}"`:''}/>`;}
function arrow(a,b,color,id='force'){return `<line x1="${a[0]}" y1="${a[1]}" x2="${b[0]}" y2="${b[1]}" stroke="${color}" stroke-width="1.5" marker-end="url(#${id})"/>`;}
function label(x,y,text,cls='svg-label',anchor='start'){return `<text x="${x}" y="${y}" class="${cls}" text-anchor="${anchor}">${text}</text>`;}
function wheel(x,y,angle=0){
 const ring=(yy,r=.3)=>Array.from({length:40},(_,i)=>project(x+Math.cos(i/40*Math.PI*2)*r,yy,.3+Math.sin(i/40*Math.PI*2)*r));
 const a=ring(y-.09),b=ring(y+.09);let out=polygon([...a.slice(0,21),...b.slice(0,21).reverse()],'#28373a','#28373a');out+=polygon(b,'#35454a','#51605d');out+=polygon(ring(y+.095,.17),'#647573','#8c9b8d');out+=polygon(ring(y+.10,.09),'#8d9e91','#bac5ac');const n=project(x+Math.sin(angle)*.22,y+.11,.3+Math.cos(angle)*.22),c=project(x,y+.11,.3);return out+line(c,n,'#ced4b8',2);
}
const defs=`<defs><marker id="force" markerWidth="7" markerHeight="7" refX="5" refY="3" orient="auto"><path d="M0,0 L6,3 L0,6" fill="none" stroke="#379799" stroke-width="1.3"/></marker><marker id="weight" markerWidth="7" markerHeight="7" refX="5" refY="3" orient="auto"><path d="M0,0 L6,3 L0,6" fill="none" stroke="#c68c30" stroke-width="1.3"/></marker><filter id="shadow"><feGaussianBlur stdDeviation="7"/></filter></defs>`;
function isoScene(m){
 let s=defs;
 for(let x=-3;x<=5;x+=.5)s+=line(project(x,-2),project(x,2.5),'#dde5d5',.65);
 for(let y=-2;y<=2.5;y+=.5)s+=line(project(-3,y),project(5,y),'#dde5d5',.65);
 s+=`<ellipse cx="310" cy="350" rx="208" ry="53" fill="#263c2a15" filter="url(#shadow)"/>`;
 const angle=params.speed*performance.now()/1000/.3;
 s+=wheel(-.8,-.65,angle)+wheel(.8,-.65,angle);
 s+=cube(-1.25,-1.02,.68,2.5,2.04,.23,['#e4b951','#c99a32','#b78724']);
 s+=cube(-.93,-.55,.91,.7,1.1,.24,['#9eb2a5','#6c8d7b','#527563']);
 s+=cube(-1.22,-1.01,.85,.13,2.02,.07,['#f2d48b','#ddb857','#c9a442']);
 s+=cube(1.15,-.51,.79,.12,.12,1.9,['#697c79','#405753','#334b47'])+cube(1.15,.39,.79,.12,.12,1.9,['#697c79','#405753','#334b47']);
 s+=cube(1.14,-.52,2.55,.14,1.04,.1,['#6b8078','#41594e','#344e43']);
 s+=cube(1.39,-.48,.205+params.height,.12,.96,.35,['#cbd5c5','#8c9d8e','#728777']);
 for(const y of [-params.forkSpacing/2,params.forkSpacing/2])s+=cube(1.44,y-.06,.205+params.height,1.4,.12,.07,['#becbc0','#81968e','#637f75']);
 s+=wheel(-.8,.65,angle)+wheel(.8,.65,angle);
 const px=1.493+1.35-params.depth+.6,loadH=m.canLift?params.height*m.carried:0;
 for(const y of [-.51,-.17,.17,.51])s+=cube(px-.6,y-.055,.02+loadH,1.2,.11,.06,['#c2a97c','#a48c63','#8b7755']);
 for(const y of [-.51,0,.51])s+=cube(px-.6,y-.08,.08+loadH,1.2,.16,.14,['#b69e76','#9b825e','#857050']);
 s+=cube(px-.6,-.6,.22+loadH,1.2,1.2,.035,['#cfbc98','#b59d77','#a28963']);
 if(params.mass>0){const cx=px+params.offsetX,cy=params.offsetY,z=.255+loadH;
  s+=cube(cx-.325,cy-.325,z,.65,.65,.6,['#76b6ac','#458b86','#357977']);
  s+=cube(cx-.03,cy-.325,z+.601,.06,.65,.004,['#b4d2b4','#8cb9a0','#8cb9a0']);
  const top=project(cx,cy,z+.61);s+=`<circle cx="${top[0]}" cy="${top[1]}" r="4" fill="#bf6850" stroke="#fff6df" stroke-width="2"/>`;
  const head=[top[0]+54,top[1]-45],tip=[top[0]+54,top[1]-8];s+=arrow(head,tip,'#c68c30','weight')+label(head[0]+11,head[1]+6,`${f(params.mass)} kg`,'svg-value');
 }
 for(const w of m.wheelData){const a=project(w.x,w.y,.01),b=project(w.x,w.y,.01+Math.min(.65,(w.n??0)/15000));s+=arrow(a,b,'#379799');const at=project(w.x,w.y+(w.y<0?-.28:.36),.01);s+=label(at[0],at[1]+13,`${w.id} ${f(w.n===null?null:w.n/1000,2)} kN`,'svg-label','middle');}
 const center=project(m.center.x,m.center.y,m.center.z);s+=`<circle cx="${center[0]}" cy="${center[1]}" r="4" fill="#c36c56" stroke="#f7fbef" stroke-width="2"/>`+line(center,[center[0]-30,center[1]-24],'#be8170',.9,'3 3')+label(center[0]-34,center[1]-26,'장비 결합 COM','svg-mini','end');
 const annotation=project(px+.6,.6,.26+loadH);s+=line(annotation,[annotation[0]+45,annotation[1]-8],'#b8c3ad',.8,'3 3')+label(annotation[0]+50,annotation[1]-8,`적재 모멘트 ${f(m.forkMoment/1000,2)} kNm`,'svg-label');
 s+=label(38,482,'ISOMETRIC / 힘의 화살표는 방향을 설명하는 도식입니다.','svg-mini');return s;
}
function planScene(m){const p=(x,y)=>[405+y*150,350-x*90];let s=defs;
 for(let x=-1.4;x<=3.7;x+=.2)s+=line(p(x,-1.8),p(x,1.8),'#e1e7d8',.6);
 for(let y=-1.8;y<=1.8;y+=.2)s+=line(p(-1.4,y),p(3.7,y),'#e1e7d8',.6);
 const poly=[0,1,3,2].filter(i=>m.wheelData[i].n!==0).map(i=>p(WHEELS[i].x,WHEELS[i].y));s+=polygon(poly,'#88ad8c22','#8dad8b');
 s+=polygon([p(1.26,-1.02),p(1.26,1.02),p(-1.26,1.02),p(-1.26,-1.02)],'#d5b96920','#cbbd8a');
 for(const y of [-params.forkSpacing/2,params.forkSpacing/2])s+=polygon([p(1.44,y-.06),p(2.84,y-.06),p(2.84,y+.06),p(1.44,y+.06)],'#829d93','#6f897e');
 const px=1.493+1.35-params.depth+.6;s+=polygon([p(px-.6,-.6),p(px+.6,-.6),p(px+.6,.6),p(px-.6,.6)],'#ceb88d80','#b69f73');
 if(params.mass>0)s+=polygon([p(px+params.offsetX-.325,params.offsetY-.325),p(px+params.offsetX+.325,params.offsetY-.325),p(px+params.offsetX+.325,params.offsetY+.325),p(px+params.offsetX-.325,params.offsetY+.325)],'#5faaa1','#4d8c84');
 const peak=Math.max(1,...m.wheelData.map(w=>w.n??0));
 for(const w of m.wheelData){const [x,y]=p(w.x,w.y),fraction=(w.n??0)/peak;s+=`<circle cx="${x}" cy="${y}" r="30" fill="${w.n===0?'#c2cab9':`hsl(${140-fraction*42} 22% ${66-fraction*21}%)`}" stroke="white" stroke-width="3"/>`+label(x,y+4,w.id,'svg-label','middle');s+=label(x+(w.y<0?-44:44),y-3,`${f(w.n===null?null:w.n/1000,2)} kN`,'svg-value',w.y<0?'end':'start');}
 const c=p(m.center.x,m.center.y);s+=`<circle cx="${c[0]}" cy="${c[1]}" r="5" fill="#bf6850" stroke="white" stroke-width="2"/>`+label(c[0]+12,c[1]-8,'무게중심 수직 투영','svg-label');
 if(m.supportPoint){const q=p(m.supportPoint.x,m.supportPoint.y);s+=line(c,q,'#4f9388',1,'4 3')+`<circle cx="${q[0]}" cy="${q[1]}" r="9" fill="none" stroke="#4f9388" stroke-width="1.5"/>`;}
 s+=label(405,45,'전방 / 포크 방향 ↑','svg-label','middle')+label(38,482,'TOP VIEW / 원의 색은 현재 네 바퀴 사이의 상대 하중입니다.','svg-mini');return s;
}
function animateLift(target){stopAnimation();const from=params.height,delta=target-from;if(Math.abs(delta)<.005){toast('이미 해당 높이에 있습니다.');return;}const duration=Math.max(1,Math.abs(delta)/calculate(params).liftSpeed*1.5);animation={type:'lift',start:performance.now(),duration,from,delta};$('#operation-state').textContent=delta>0?'포크 상승 중 · 관성 부하 포함':'포크 하강 중 · 관성 부하 포함';tick();}
const smooth=t=>t*t*(3-2*t);
function tick(){if(!animation)return;const t=(performance.now()-animation.start)/1000;
 if(animation.type==='lift'){const u=Math.min(1,t/animation.duration);params.height=clamp(animation.from+animation.delta*smooth(u),0,1.5);liftAcceleration=animation.delta*(6-12*u)/animation.duration**2;if(u===1){stopAnimation();render();return;}}
 else{if(t<5){const u=t/5;params.height=.45*smooth(u);params.speed=0;params.brake=0;liftAcceleration=.45*(6-12*u)/25;$('#operation-state').textContent='01 양중 · 무게중심이 높아집니다';}
  else if(t<10){params.height=.45;params.speed=2*smooth((t-5)/5);params.brake=0;liftAcceleration=0;$('#operation-state').textContent='02 선회 운반 · 좌우 하중이 이동합니다';}
  else if(t<11){params.speed=Math.max(0,2-2*(t-10));params.brake=2;$('#operation-state').textContent='03 제동 · 앞바퀴 하중과 마찰 요구가 증가합니다';}
  else{params.speed=0;params.brake=0;stopAnimation();render();toast('양중·운반·제동 입력 재생을 마쳤습니다.');return;}}
 render();animationFrame=requestAnimationFrame(tick);
}
function selectFeature(id,scroll=false){const feature=features.find(f=>f.id===id)||features[0];activeFeature=feature.id;$$('[data-feature]').forEach(el=>el.classList.toggle('selected',el.dataset.feature===feature.id));$('#feature-detail').innerHTML=`<div class="feature-detail-top"><div><span class="tiny-label">SYSTEM ${feature.num} / IMPLEMENTATION DETAIL</span><h2>${feature.title}</h2></div><span class="status">${feature.status}</span></div><p>${feature.body}</p><div class="formula">${feature.formula}</div><dl class="feature-params">${feature.params.map(([k,v])=>`<div><dt>${k}</dt><dd>${v}</dd></div>`).join('')}</dl><div class="feature-limit"><strong>모델의 범위</strong><p>${feature.limit}</p><code>Source / ${feature.source}</code></div>`;if(scroll)$('#feature-detail').scrollIntoView({behavior:'smooth',block:'start'});}
function route(){const [name,feature]=location.hash.slice(1).split('/'),page=['lab','features','validation'].includes(name)?name:'lab';if(page!=='lab')stopAnimation();$$('.page').forEach(el=>{const active=el.id===`page-${page}`;el.classList.toggle('active',active);el.hidden=!active;});$$('.nav-link').forEach(el=>{const active=el.dataset.page===page;el.classList.toggle('active',active);if(active)el.setAttribute('aria-current','page');else el.removeAttribute('aria-current');});$('#breadcrumb-current').textContent={lab:'부하 실험실',features:'구현 기능',validation:'검증 기록'}[page];if(page==='features')selectFeature(feature||activeFeature);}
$('#system-nav').innerHTML=features.map(f=>`<a class="system-link" href="#features/${f.id}" data-feature="${f.id}"><i></i>${f.title}</a>`).join('');
$('#feature-grid').innerHTML=features.map(f=>`<button class="feature-card" data-feature="${f.id}" aria-label="${f.title} 상세 보기"><span class="feature-num">${f.num}</span><div><h2>${f.title}</h2><p>${f.sub}</p><span class="status">${f.status}</span></div><span class="feature-open">↗</span></button>`).join('');
$('#calibration-table').innerHTML=calibration.map(r=>`<tr><td>${r.fps} FPS</td><td>${r.hz} Hz</td><td>${r.speed.toFixed(5)} %</td><td>${r.energy.toFixed(4)} J</td><td><span class="pass">통과</span></td></tr>`).join('');
$('#integrated-table').innerHTML=integrated.map(([label,stop,mean,inner])=>`<tr><td>${label}</td><td>${stop.toFixed(5)} m</td><td>${mean.toFixed(3)} kN</td><td>${inner.toFixed(3)} kN</td><td><span class="pass">통과</span></td></tr>`).join('');
$$('[data-field]').forEach(el=>el.addEventListener(el.type==='range'?'input':'change',()=>{stopAnimation();const key=el.dataset.field,c=fieldMap[key],v=Number(el.value);if(!Number.isFinite(v)||el.value===''){syncInputs();toast('범위 안의 숫자를 입력하세요.');return;}params[key]=clamp(v,c[2],c[3]);if(key==='cargoMu')params.cargoDynamicMu=Math.min(params.cargoMu,params.cargoDynamicMu);if(key==='cargoDynamicMu')params.cargoDynamicMu=Math.min(params.cargoMu,params.cargoDynamicMu);$$('.preset').forEach(e=>e.classList.remove('active'));render();}));
$$('[data-control]').forEach(el=>el.addEventListener('click',()=>{$$('[data-control]').forEach(e=>{const active=e===el;e.classList.toggle('active',active);e.setAttribute('aria-selected',String(active));});$$('.control-group').forEach(e=>e.hidden=e.id!==`control-${el.dataset.control}`);}));
const presets={default:{},eccentric:{offsetY:.2,height:.75},slippery:{speed:2,brake:2,roadMu:.08,cargoMu:.12,cargoDynamicMu:.1},shallow:{depth:.3}};
$$('[data-preset]').forEach(el=>el.addEventListener('click',()=>{stopAnimation();params={...DEFAULTS,...presets[el.dataset.preset]};$$('.preset').forEach(e=>e.classList.toggle('active',e===el));render();}));
$('#reset').addEventListener('click',()=>{$('[data-preset="default"]').click();toast('기준 적재 조건으로 돌아왔습니다.');});
$('#raise').addEventListener('click',()=>animateLift(Math.min(1.5,params.height+.45)));
$('#lower').addEventListener('click',()=>animateLift(Math.max(0,params.height-.45)));
$('#replay').addEventListener('click',()=>{if(animation?.type==='replay'){stopAnimation();render();return;}stopAnimation();animation={type:'replay',start:performance.now()};$('#replay').textContent='Ⅱ 재생 중지';tick();});
$('#show-model').addEventListener('click',()=>{$('#model-details').open=true;$('#model-details').scrollIntoView({behavior:'smooth'});});
for(const mode of ['iso','plan'])$(`#view-${mode}`).addEventListener('click',()=>{view=mode;for(const id of ['iso','plan']){$(`#view-${id}`).classList.toggle('selected',id===mode);$(`#view-${id}`).setAttribute('aria-pressed',String(id===mode));}render();});
$$('.feature-card').forEach(el=>el.addEventListener('click',()=>{history.replaceState(null,'',`#features/${el.dataset.feature}`);selectFeature(el.dataset.feature,true);}));
addEventListener('hashchange',route);
addEventListener('keydown',event=>{if(event.repeat||event.ctrlKey||event.altKey||event.metaKey||document.activeElement.isContentEditable||['INPUT','TEXTAREA','SELECT'].includes(document.activeElement.tagName)||!$('#page-lab').classList.contains('active'))return;if(event.key.toLowerCase()==='e'){$('#raise').click();event.preventDefault();}if(event.key.toLowerCase()==='q'){$('#lower').click();event.preventDefault();}});
route();render();
