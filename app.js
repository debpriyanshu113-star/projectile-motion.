(() => {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const canvas = $('scene'), ctx = canvas.getContext('2d');
  const defaults = { speed:10, angle:45, height:0, gravity:9.81, drag:false, cd:.47, mass:1 };
  const state = { ...defaults, time:0, playing:false, rate:1, path:[], flight:null, lastFrame:null, options:{trajectory:true,vectors:true,apex:true,landing:true,grid:true,axes:true} };
  const controls = [ ['speed','speedRange','speedInput'], ['angle','angleRange','angleInput'], ['height','heightRange','heightInput'], ['gravity','gravityRange','gravityInput'] ];
  let view = { sx:1, sy:1, ox:60, oy:0, w:0, h:0 };
  const finite = n => Number.isFinite(n) ? n : 0;
  const fmt = (n, decimals=2) => finite(n).toFixed(decimals);
  const clamp = (n, min, max) => Math.max(min, Math.min(max, finite(n)));
  const rad = d => d * Math.PI / 180;
  function inputFill(input) { const pct = (input.value-input.min)/(input.max-input.min)*100; input.style.setProperty('--fill', `${pct}%`); }
  function bindPair(key, rangeId, inputId) {
    const range = $(rangeId), num = $(inputId), set = value => { const val = clamp(parseFloat(value), +range.min, +range.max); state[key]=val; range.value=val; num.value=val; inputFill(range); recalculate(); };
    range.addEventListener('input', () => set(range.value));
    num.addEventListener('change', () => set(num.value)); inputFill(range);
  }
  function idealFlight() {
    const t = rad(state.angle), vx = state.speed*Math.cos(t), vy0=state.speed*Math.sin(t), g=state.gravity;
    let discriminant = vy0*vy0 + 2*g*state.height;
    let tf = discriminant >= 0 ? (vy0 + Math.sqrt(discriminant))/g : 0;
    if (state.height === 0 && vy0 <= 0) tf=0;
    tf=Math.max(0, finite(tf));
    const apexTime = vy0 > 0 ? vy0/g : 0;
    return { tf, range:vx*tf, maxHeight:state.height+(vy0>0?vy0*vy0/(2*g):0), apexTime, vx, vy0 };
  }
  function sampleDrag() {
    const dt=.002, theta=rad(state.angle), rho=1.225, area=.01;
    let x=0,y=state.height,vx=state.speed*Math.cos(theta),vy=state.speed*Math.sin(theta),t=0,maxY=y,apexT=0,path=[{t,x,y,vx,vy}];
    if (y<=0 && vy<=0) return { tf:0, range:0,maxHeight:y,apexTime:0,vx,vy0:vy,path };
    let highest=vy;
    while (t<120 && (t===0 || y>=0)) { const speed=Math.hypot(vx,vy); const k=.5*rho*state.cd*area/state.mass; vx += -k*speed*vx*dt; vy += (-state.gravity-k*speed*vy)*dt; x += vx*dt; y += vy*dt; t+=dt; if(vy<0 && highest>=0) apexT=t; highest=vy; if(y>maxY) maxY=y; path.push({t,x,y:Math.max(0,y),vx,vy}); if(y<0) break; }
    const last=path[path.length-1], prev=path[path.length-2] || last;
    if(last.y===0 && prev.y>0) { const fraction=prev.y/(prev.y-last.y); last.t=prev.t+(last.t-prev.t)*fraction; last.x=prev.x+(last.x-prev.x)*fraction; }
    return {tf:last.t,range:last.x,maxHeight:maxY,apexTime:apexT,vx:state.speed*Math.cos(theta),vy0:state.speed*Math.sin(theta),path};
  }
  function pointAt(t) {
    const f=state.flight; t=clamp(t,0,f.tf);
    if (!state.drag) { const y=Math.max(0,state.height+f.vy0*t-.5*state.gravity*t*t); return {t,x:f.vx*t,y,vx:f.vx,vy:f.vy0-state.gravity*t}; }
    const path=f.path; if(!path.length) return {t:0,x:0,y:state.height,vx:0,vy:0}; let i=path.findIndex(p=>p.t>=t); if(i<1) return path[Math.max(0,i)]; const a=path[i-1],b=path[i],q=(t-a.t)/(b.t-a.t); return {t,x:a.x+(b.x-a.x)*q,y:a.y+(b.y-a.y)*q,vx:a.vx+(b.vx-a.vx)*q,vy:a.vy+(b.vy-a.vy)*q};
  }
  function recalculate() { state.playing=false; state.time=0; state.lastFrame=null; state.flight=state.drag?sampleDrag():idealFlight(); state.path=state.drag?state.flight.path:makeIdealPath(); updateStats(); render(); }
  function makeIdealPath(){ const f=state.flight, count=Math.max(2,Math.ceil(f.tf/.01)); return Array.from({length:count+1},(_,i)=>pointAt(f.tf*i/count)); }
  function setupCanvas() { const r=canvas.getBoundingClientRect(),dpr=devicePixelRatio||1; canvas.width=Math.max(1,Math.round(r.width*dpr)); canvas.height=Math.max(1,Math.round(r.height*dpr)); ctx.setTransform(dpr,0,0,dpr,0,0); view.w=r.width;view.h=r.height; updateView(); render(); }
  function updateView(){ const f=state.flight||idealFlight(), maxX=Math.max(1,Math.abs(f.range)), maxY=Math.max(1,f.maxHeight); const pad={l:75,r:35,t:45,b:48}; const usableW=Math.max(1,view.w-pad.l-pad.r), usableH=Math.max(1,view.h-pad.t-pad.b); const scale=Math.min(usableW/(maxX*1.14),usableH/(maxY*1.22)); view.sx=view.sy=Math.max(1,scale); view.ox=pad.l+(f.range<0?usableW-(-f.range)*view.sx*.07:0); view.oy=view.h-pad.b; }
  function screen(p){ return {x:view.ox+p.x*view.sx,y:view.oy-p.y*view.sy}; }
  function drawGrid(){ const {w,h,ox,oy,sx,sy}=view; ctx.save(); ctx.fillStyle='#081320';ctx.fillRect(0,0,w,h); const target=80/sx; const step=Math.pow(10,Math.floor(Math.log10(target))); const grid=target/step<2?step:target/step<5?2*step:5*step; const startX=Math.floor((-ox/sx)/grid)*grid,endX=Math.ceil((w-ox)/sx/grid)*grid; const startY=0,endY=Math.ceil(oy/sy/grid)*grid; if(state.options.grid){ctx.strokeStyle='#14303b';ctx.fillStyle='#284754';ctx.lineWidth=1; for(let x=startX;x<=endX;x+=grid){const px=ox+x*sx;ctx.beginPath();ctx.moveTo(px,0);ctx.lineTo(px,h);ctx.stroke();} for(let y=startY;y<=endY;y+=grid){const py=oy-y*sy;ctx.beginPath();ctx.moveTo(0,py);ctx.lineTo(w,py);ctx.stroke();}} if(state.options.axes){ctx.strokeStyle='#38707b';ctx.lineWidth=1.2;ctx.beginPath();ctx.moveTo(0,oy);ctx.lineTo(w,oy);ctx.moveTo(ox,0);ctx.lineTo(ox,h);ctx.stroke();ctx.fillStyle='#759da7';ctx.font='10px DM Mono';for(let x=startX;x<=endX;x+=grid){if(Math.abs(x)>1e-6)ctx.fillText(fmt(x,1),ox+x*sx+3,oy+14);}for(let y=grid;y<=endY;y+=grid)ctx.fillText(fmt(y,1),ox-31,oy-y*sy-3);}ctx.restore(); }
  function drawPath(){ const path=state.path; if(!state.options.trajectory||path.length<2)return; ctx.save();ctx.beginPath();path.forEach((p,i)=>{const q=screen(p);i?ctx.lineTo(q.x,q.y):ctx.moveTo(q.x,q.y);});ctx.strokeStyle='#74d0ff';ctx.lineWidth=1.8;ctx.shadowColor='#30c4fc';ctx.shadowBlur=5;ctx.stroke();ctx.restore(); }
  function marker(p,label,color='#15e4e5'){const q=screen(p);ctx.save();ctx.strokeStyle=color;ctx.fillStyle='#08131ae0';ctx.lineWidth=1;ctx.beginPath();ctx.arc(q.x,q.y,5,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.fillStyle=color;ctx.font='11px Inter';ctx.fillText(label,q.x+9,q.y-8);ctx.restore();}
  function drawMarkers(){const f=state.flight;if(state.options.landing)marker({x:f.range,y:0},`R = ${fmt(f.range)} m`,'#19e1d9');if(state.options.apex&&f.maxHeight>state.height+.001){const p=pointAt(f.apexTime);marker({x:p.x,y:f.maxHeight},`apex ${fmt(f.maxHeight)} m`,'#f6c86d');} const launch=screen({x:0,y:state.height});ctx.save();ctx.fillStyle='#19e4e5';ctx.beginPath();ctx.arc(launch.x,launch.y,4,0,Math.PI*2);ctx.fill();ctx.fillStyle='#acdbdf';ctx.font='11px Inter';ctx.fillText('launch',launch.x+8,launch.y+16);ctx.restore();}
  function drawProjectile(p){const q=screen(p);ctx.save();const g=ctx.createRadialGradient(q.x-2,q.y-2,1,q.x,q.y,9);g.addColorStop(0,'#efffff');g.addColorStop(.3,'#56ffff');g.addColorStop(1,'#009cac');ctx.fillStyle=g;ctx.shadowColor='#16eff3';ctx.shadowBlur=15;ctx.beginPath();ctx.arc(q.x,q.y,6,0,Math.PI*2);ctx.fill();if(state.options.vectors){const mag=Math.hypot(p.vx,p.vy)||1,scale=42/mag,ex=q.x+p.vx*scale,ey=q.y-p.vy*scale;ctx.strokeStyle='#f5d767';ctx.lineWidth=1.5;ctx.shadowBlur=0;ctx.beginPath();ctx.moveTo(q.x,q.y);ctx.lineTo(ex,ey);ctx.stroke();ctx.fillStyle='#f5d767';ctx.beginPath();ctx.arc(ex,ey,2.5,0,Math.PI*2);ctx.fill();}ctx.restore();}
  function render(){if(!state.flight)return;drawGrid();drawPath();drawMarkers();drawProjectile(pointAt(state.time));}
  function updateStats(){const p=pointAt(state.time),f=state.flight,progress=f.tf?state.time/f.tf*100:0;const values=[['v (speed)',`${fmt(Math.hypot(p.vx,p.vy))} m/s`],['vₓ',`${fmt(p.vx)} m/s`],['vᵧ',`${fmt(p.vy)} m/s`],['h (height)',`${fmt(p.y)} m`],['x (range)',`${fmt(p.x)} m`],['t (elapsed)',`${fmt(p.t)} s`],['tᶠ (flight)',`${fmt(f.tf)} s`],['flight %',`${fmt(progress,0)} %`],['R (predicted)',`${fmt(f.range)} m`],['hₘₐₓ',`${fmt(f.maxHeight)} m`],['tₐₚₑₓ',`${fmt(f.apexTime)} s`]];$('stats').innerHTML=values.map(([k,v])=>`<div><dt>${k}:</dt><dd>${v}</dd></div>`).join('');}
  function setTime(t){state.time=clamp(t,0,state.flight.tf);if(state.time>=state.flight.tf)state.playing=false;updateStats();render();updatePlayButton();}
  function updatePlayButton(){const btn=$('playPause');btn.textContent=state.playing?'Ⅱ':state.time>=state.flight.tf?'↻':'▶';btn.setAttribute('aria-label',state.playing?'Pause simulation':state.time>=state.flight.tf?'Restart simulation':'Launch simulation');}
  function frame(now){if(state.playing){if(state.lastFrame!==null)setTime(state.time+(now-state.lastFrame)/1000*state.rate);state.lastFrame=now;}else state.lastFrame=null;requestAnimationFrame(frame);}
  function download(name,text,type){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([text],{type}));a.download=name;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}
  function exportCsv(){const f=state.flight, rows=['Time (s),Horizontal position (m),Vertical position (m),Horizontal velocity (m/s),Vertical velocity (m/s),Speed (m/s)'];const n=Math.max(1,Math.ceil(f.tf/.01));for(let i=0;i<=n;i++){const p=pointAt(f.tf*i/n);rows.push([p.t,p.x,p.y,p.vx,p.vy,Math.hypot(p.vx,p.vy)].map(x=>finite(x).toFixed(5)).join(','));}rows.push('',`Summary: flight time,${f.tf.toFixed(5)}`,`Summary: range,${f.range.toFixed(5)}`,`Summary: maximum height,${f.maxHeight.toFixed(5)}`,`Summary: time to apex,${f.apexTime.toFixed(5)}`);download('projectile-trajectory.csv',rows.join('\n'),'text/csv');}
  controls.forEach(([a,b,c])=>bindPair(a,b,c));
  $('dragToggle').addEventListener('change',e=>{state.drag=e.target.checked;$('dragControls').hidden=!state.drag;recalculate();});
  [['dragRange','cd','dragOutput'],['massRange','mass','massOutput']].forEach(([id,key,out])=>{inputFill($(id));$(id).addEventListener('input',e=>{state[key]=+e.target.value;$(out).value=state[key].toFixed(2);inputFill(e.target);recalculate();});});
  $('earthBtn').addEventListener('click',()=>{ $('gravityInput').value=$('gravityRange').value=9.81;inputFill($('gravityRange'));state.gravity=9.81;recalculate(); });
  document.querySelectorAll('[data-option]').forEach(x=>x.addEventListener('change',()=>{state.options[x.dataset.option]=x.checked;render();}));
  $('playbackSpeed').addEventListener('change',e=>state.rate=+e.target.value);
  $('playPause').addEventListener('click',()=>{if(state.time>=state.flight.tf)setTime(0);state.playing=!state.playing;state.lastFrame=null;updatePlayButton();});
  $('restart').addEventListener('click',()=>{state.playing=false;setTime(0);}); $('stepBack').addEventListener('click',()=>{state.playing=false;setTime(state.time-.01);});$('stepForward').addEventListener('click',()=>{state.playing=false;setTime(state.time+.01);});
  $('saveConfig').addEventListener('click',()=>download('projectile-configuration.json',JSON.stringify({speed:state.speed,angle:state.angle,height:state.height,gravity:state.gravity,airResistance:state.drag,dragCoefficient:state.cd,mass:state.mass},null,2),'application/json'));$('exportCsv').addEventListener('click',exportCsv);
  $('resetParams').addEventListener('click',()=>{Object.assign(state,defaults);controls.forEach(([key,r,n])=>{$(r).value=$(n).value=state[key];inputFill($(r));});$('dragToggle').checked=false;$('dragControls').hidden=true;$('dragRange').value=state.cd;$('massRange').value=state.mass;recalculate();});$('homeBtn').addEventListener('click',()=>$('resetParams').click());
  $('infoBtn').addEventListener('click',()=>$('infoDialog').showModal());document.querySelector('.dialog-close').addEventListener('click',()=>$('infoDialog').close());
  new ResizeObserver(setupCanvas).observe(canvas);state.flight=idealFlight();recalculate();requestAnimationFrame(frame);
})();
