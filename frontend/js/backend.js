// ── Physics constants ──
const VEHICLE_LEN   = 4.5;
const ACCEL_O       = 2.0;
const BRAKE_DECEL   = 4.5;
const LATERAL_TIME  = 2.5;
const SAFETY_MARGIN = 8.0;
const REACTION_TIME = 1.0;
const SUB_DT        = 0.05;

// Safe-distance polynomial (quadratic fit)
const speedRef = [40,60,70,80,90,100,110,120];
const distRef  = [15,25,35,45,60,75,90,110];
function polyFit2(xs, ys) {
  // least-squares quadratic via normal equations
  const n = xs.length;
  let s0=n, s1=0, s2=0, s3=0, s4=0, t0=0, t1=0, t2=0;
  for (let i=0;i<n;i++){
    const x=xs[i], y=ys[i];
    s1+=x; s2+=x*x; s3+=x*x*x; s4+=x*x*x*x;
    t0+=y; t1+=x*y; t2+=x*x*y;
  }
  // [s0,s1,s2][a0]=[t0]
  // [s1,s2,s3][a1]=[t1]
  // [s2,s3,s4][a2]=[t2]
  const A=[[s0,s1,s2],[s1,s2,s3],[s2,s3,s4]];
  const b=[t0,t1,t2];
  // Gaussian elimination
  for(let col=0;col<3;col++){
    let max=col, mv=Math.abs(A[col][col]);
    for(let r=col+1;r<3;r++) if(Math.abs(A[r][col])>mv){max=r;mv=Math.abs(A[r][col]);}
    [A[col],A[max]]=[A[max],A[col]];[b[col],b[max]]=[b[max],b[col]];
    for(let r=col+1;r<3;r++){
      const f=A[r][col]/A[col][col];
      for(let c=col;c<3;c++) A[r][c]-=f*A[col][c];
      b[r]-=f*b[col];
    }
  }
  const x=[0,0,0];
  for(let i=2;i>=0;i--){x[i]=b[i];for(let j=i+1;j<3;j++)x[i]-=A[i][j]*x[j];x[i]/=A[i][i];}
  return x; // [c, b, a] for a*x^2+b*x+c
}
const [pc, pb, pa] = polyFit2(speedRef, distRef);
function safeDist(vKmh){ return Math.max(5.0, pa*vKmh*vKmh + pb*vKmh + pc); }
function toMs(v){ return v/3.6; }
function toKmh(v){ return v*3.6; }

function integratePhaseA(vCms, vXms, distX, dPass, gapCL){
  let oPos=0, oSpeed=vCms, xDist=distX, tElapsed=0;
  const ttcs=[], clrnces=[], abrtOk=[];
  while(oPos < dPass && tElapsed < 30.0){
    oSpeed += ACCEL_O * SUB_DT;
    oPos   += oSpeed  * SUB_DT;
    xDist  -= vXms    * SUB_DT;
    tElapsed += SUB_DT;
    const closing = oSpeed + vXms;
    const ttcNow  = (closing>0 && xDist>0) ? xDist/closing : 0;
    const tRemA   = Math.max(0, dPass-oPos)/oSpeed;
    const tRem    = tRemA + LATERAL_TIME;
    const clrNow  = xDist - (vXms*tRem) - (Math.max(0,dPass-oPos) + oSpeed*LATERAL_TIME);
    const gapNow  = gapCL - oPos;
    const sAb     = (oSpeed*REACTION_TIME) + ((oSpeed*oSpeed - vCms*vCms)/(2*BRAKE_DECEL));
    ttcs.push(ttcNow); clrnces.push(clrNow);
    abrtOk.push(sAb < (gapNow + 5.0));
  }
  const abortIdx = abrtOk.reduce((acc,v,i)=>v?[...acc,i]:acc,[]);
  return {
    tA: tElapsed, vOPeakMs: oSpeed,
    minTTC:  ttcs.length    ? Math.min(...ttcs)    : 0,
    minClr:  clrnces.length ? Math.min(...clrnces) : -999,
    tLastAbort: abortIdx.length ? abortIdx[abortIdx.length-1]*SUB_DT : 0
  };
}

function runSimulation(){
  const vL    = parseFloat(document.getElementById('v_L').value)    || 0;
  const vC    = parseFloat(document.getElementById('v_C').value)    || 0;
  const vX    = parseFloat(document.getElementById('v_X').value)    || 0;
  const gapCL = parseFloat(document.getElementById('gap_CL').value) || 0;
  const distX = parseFloat(document.getElementById('dist_X').value) || 0;

  const vCms  = toMs(vC), vXms = toMs(vX);
  const dPass = gapCL + 2*VEHICLE_LEN;
  const dyn   = integratePhaseA(vCms, vXms, distX, dPass, gapCL);
  const tTotal  = dyn.tA + LATERAL_TIME;
  const vOPkKmh = toKmh(dyn.vOPeakMs);
  const minStartGap = VEHICLE_LEN + 10;

  const reasons = [];
  if(dyn.minClr < 5.0)            reasons.push("Oncoming car too close at end of move");
  if(dyn.minTTC < tTotal*1.2)     reasons.push("TTC too low — insufficient time margin");
  if(gapCL < minStartGap)         reasons.push("Too close to lead car to swing out");

  const safe = reasons.length === 0;

  // Verdict banner
  const banner = document.getElementById('verdict-banner');
  banner.className = 'verdict ' + (safe ? 'safe' : 'unsafe');
  document.getElementById('verdict-label').textContent = safe ? 'All checks passed' : 'Unsafe conditions detected';
  document.getElementById('verdict-title').textContent = safe ? '>>> SAFE <<<' : '!!! UNSAFE !!!';
  const rEl = document.getElementById('verdict-reasons');
  rEl.innerHTML = reasons.map(r=>`<div class="reason-item">${r}</div>`).join('');

  // Metrics
  document.getElementById('m-speed').textContent  = vOPkKmh.toFixed(1);
  document.getElementById('m-time').textContent   = tTotal.toFixed(2);
  document.getElementById('m-clr').textContent    = dyn.minClr.toFixed(2);
  document.getElementById('m-abort').textContent  = dyn.tLastAbort.toFixed(2);

  // Bars (animate after paint)
  setTimeout(()=>{
    document.getElementById('bar-speed').style.width  = Math.min(100, vOPkKmh/160*100)+'%';
    document.getElementById('bar-time').style.width   = Math.min(100, tTotal/20*100)+'%';
    const clrPct = Math.min(100, Math.max(0, (dyn.minClr+50)/150*100));
    document.getElementById('bar-clr').style.width    = clrPct+'%';
    document.getElementById('bar-clr').style.background = dyn.minClr<5 ? 'var(--red)' : dyn.minClr<20 ? 'var(--amber)' : 'var(--green)';
    document.getElementById('bar-abort').style.width  = Math.min(100, dyn.tLastAbort/10*100)+'%';
  }, 50);

  // Physics detail rows
  const minTTCNeeded = tTotal * 1.2;
  const clrNeeded    = 5.0;
  const rows = [
    ['d_pass (overtake distance)',  `${dPass.toFixed(2)} m`,        null],
    ['Phase A time (t_A)',          `${dyn.tA.toFixed(2)} s`,       null],
    ['Lateral change time (t_B)',   `${LATERAL_TIME} s`,            null],
    ['Total manoeuvre time',        `${tTotal.toFixed(2)} s`,       null],
    ['Peak closing speed',          `${(vOPkKmh + vX).toFixed(1)} km/h`, null],
    ['Min TTC (need ≥ '+minTTCNeeded.toFixed(2)+'s)', dyn.minTTC.toFixed(2)+' s', dyn.minTTC >= minTTCNeeded ? 'ok' : 'bad'],
    ['Min clearance (need ≥ 5 m)',  dyn.minClr.toFixed(2)+' m',   dyn.minClr >= clrNeeded ? 'ok' : 'bad'],
    ['Last abort at',               dyn.tLastAbort.toFixed(2)+' s', dyn.tLastAbort > 0 ? 'ok' : 'warn'],
    ['Gap vs min start gap ('+minStartGap.toFixed(1)+'m)', gapCL.toFixed(2)+' m', gapCL >= minStartGap ? 'ok' : 'bad'],
    ['safe_dist(v_L='+vL+')',       safeDist(vL).toFixed(1)+' m',  null],
  ];
  document.getElementById('phys-rows').innerHTML = rows.map(([k,v,cls])=>
    `<div class="phys-row">
       <span class="phys-key">${k}</span>
       <span class="phys-val ${cls||''}">${v}</span>
     </div>`
  ).join('');

  const el = document.getElementById('results');
  el.style.display = 'none';
  el.classList.remove('visible');
  void el.offsetWidth;
  el.style.display = 'block';
  el.classList.add('visible');
  el.scrollIntoView({behavior:'smooth', block:'start'});
}

// Allow Enter key to submit
document.addEventListener('keydown', e => { if(e.key==='Enter') runSimulation(); });
