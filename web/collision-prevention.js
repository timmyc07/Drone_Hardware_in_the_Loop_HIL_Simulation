import { rayAabb } from './obstacle-geometry.js';
import { relativeSweep } from './peer-geometry.js';
export const DEFAULT_COLLISION_PREVENTION = Object.freeze({ enabled:false, clearanceM:1, responseDelaySeconds:.6, brakeAccelerationMps2:2 });
const exact=(v,keys)=>v && [Object.prototype,null].includes(Object.getPrototypeOf(v)) && Reflect.ownKeys(v).length===keys.length && keys.every(k=>Object.hasOwn(v,k));
const pair=v=>Array.isArray(v)&&v.length===2&&Reflect.ownKeys(v).length===3&&Array.from(v).every(x=>Number.isFinite(x)&&Math.abs(x)<=3.000001);
export function validateCollisionPreventionSettings(value=DEFAULT_COLLISION_PREVENTION) {
  if (!exact(value,Object.keys(DEFAULT_COLLISION_PREVENTION)) || typeof value.enabled!=='boolean') throw Error('Invalid collision prevention settings');
  for (const [key,min,max] of [['clearanceM',0,20],['responseDelaySeconds',0,5],['brakeAccelerationMps2',.1,3]])
    if (!Number.isFinite(value[key]) || value[key]<min || value[key]>max) throw Error(`Invalid collision prevention ${key}`);
  return {...value};
}
export function validateCollisionPreventionState(value) {
  if (value===null) return null;
  if (!exact(value,['applicable','active','reason','obstacleType','obstacleId','desiredVelocityNe','constrainedVelocityNe','commandedAccelerationNe','clearanceM']) ||
    typeof value.applicable!=='boolean'||typeof value.active!=='boolean'||
    !['clear','unsupported-mode','obstacle','peer','insufficient-clearance'].includes(value.reason)||
    ![null,'obstacle','nist','peer'].includes(value.obstacleType)||
    !(value.obstacleId===null || typeof value.obstacleId==='string')||
    !['desiredVelocityNe','constrainedVelocityNe','commandedAccelerationNe'].every(k=>pair(value[k]))||
    !Number.isFinite(value.clearanceM)||value.clearanceM<0||value.clearanceM>20) throw Error('Invalid collision prevention state');
  const identity = value.obstacleType===null ? value.obstacleId===null : typeof value.obstacleId==='string' && (value.obstacleType==='obstacle' ? /^(?:[0-9]|[1-4][0-9])$/.test(value.obstacleId) : value.obstacleType==='nist' ? value.obstacleId==='nist-lane' : /^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(value.obstacleId));
  if (!identity || (!value.applicable && (value.active||value.reason!=='unsupported-mode')) || (value.applicable && value.reason==='unsupported-mode') ||
    (['clear','unsupported-mode'].includes(value.reason) && (value.active||value.obstacleType!==null)) ||
    (['obstacle','peer','insufficient-clearance'].includes(value.reason) && !value.active) ||
    (value.reason==='peer' && value.obstacleType!=='peer') ||
    (value.reason==='obstacle' && !['obstacle','nist'].includes(value.obstacleType))) throw Error('Inconsistent collision prevention state');
  return structuredClone(value);
}
export function collisionPreventionInactive(settings) {
  return settings.enabled ? {applicable:false,active:false,reason:'unsupported-mode',obstacleType:null,obstacleId:null,desiredVelocityNe:[0,0],constrainedVelocityNe:[0,0],commandedAccelerationNe:[0,0],clearanceM:settings.clearanceM}:null;
}
const limit=(v,max)=>{const scale=Math.min(1,max/Math.max(1e-12,Math.hypot(...v)));return v.map(x=>x*scale);};
// Known scene geometry only: no sensor coverage, terrain protection or route planning.
export function constrainHorizontalMotion({settings,positionNed,velocityNe,desiredVelocityNe,radius,obstacles,nistCollider,peers}) {
  const a=settings.brakeAccelerationMps2, margin=radius+settings.clearanceM;
  const orderedPeers=[...peers].sort((x,y)=>x.id<y.id?-1:x.id>y.id?1:0);
  function segment(start,end,t0,t1,padding) {
    let nearest=null;
    const hit=(fraction,type,id)=>{if(fraction!==null && (!nearest||fraction<nearest.fraction)) nearest={fraction,type,id};};
    obstacles.forEach((box,i)=>{
      const interval=rayAabb(start,end.map((v,j)=>v-start[j]),{low:box.low.map(v=>v-settings.clearanceM-padding),high:box.high.map(v=>v+settings.clearanceM+padding)});
      if(interval && Math.max(0,interval.enter)<=Math.min(1,interval.exit)) hit(Math.max(0,interval.enter),'obstacle',String(i));
    });
    if(nistCollider) hit(nistCollider.sweep(start,end,margin+padding),'nist','nist-lane');
    for(const peer of orderedPeers) {
      const from=peer.position.map((v,i)=>v+peer.velocity[i]*t0),to=peer.position.map((v,i)=>v+peer.velocity[i]*t1);
      if (![...from,...to].every(x=>Number.isFinite(x)&&Math.abs(x)<=1e9)) return {type:null,id:null};
      hit(relativeSweep(start,end,margin+padding,from,to,peer.radius),'peer',peer.id);
    }
    return nearest;
  }
  function trajectory(w) {
    let p=[...positionNed],v=[...velocityNe],time=0;
    function phase(duration,acceleration) {
      // The parabola lies within a*dt²/8 of each chord; inflate the swept envelope accordingly.
      const count=Math.max(1,Math.min(64,Math.ceil(duration/.15))),dt=duration/count;
      for(let i=0;i<count;i++) {
        const next=[p[0]+v[0]*dt+acceleration[0]*dt*dt/2,p[1]+v[1]*dt+acceleration[1]*dt*dt/2,p[2]];
        const padding=Math.hypot(...acceleration)*dt*dt/8;
        if (!Number.isFinite(time+dt) || !next.every(x=>Number.isFinite(x)&&Math.abs(x)<=1e9) || !Number.isFinite(padding) || margin+padding>1000)
          return {type:null,id:null};
        const obstacle=segment(p,next,time,time+dt,padding);
        if(obstacle)return obstacle;
        p=next;v=v.map((x,j)=>x+acceleration[j]*dt);time+=dt;
      }
      return null;
    }
    // The velocity controller is proportional (2.2/s); reserve its exponential braking tail as travel.
    const delay = settings.responseDelaySeconds + 1 / 2.2;
    const change=w.map((x,i)=>x-v[i]),changeTime=Math.hypot(...change)/a,stopTime=Math.hypot(...w)/a;
    if (![delay,changeTime,stopTime].every(Number.isFinite)) return {type:null,id:null};
    let obstacle=phase(delay,[0,0]);
    if(obstacle)return obstacle;
    if(changeTime>0) {obstacle=phase(changeTime,change.map(x=>x/changeTime));if(obstacle)return obstacle;}
    if(stopTime>0)return phase(stopTime,w.map(x=>-x/stopTime));
    return null;
  }
  let constrained=[...desiredVelocityNe],threat=trajectory(constrained),reason='clear';
  if(threat) {
    const stopping=trajectory([0,0]);
    if(stopping || threat.type===null) {constrained=[0,0];reason='insufficient-clearance';threat=stopping || threat;}
    else {
      constrained=[0,0];reason=threat.type==='peer'?'peer':'obstacle';
    }
  }
  return {applicable:true,active:!!threat,reason,obstacleType:threat?.type??null,obstacleId:threat?.id??null,desiredVelocityNe:[...desiredVelocityNe],constrainedVelocityNe:constrained,commandedAccelerationNe:limit(constrained.map((x,i)=>x-velocityNe[i]),a/2.2).map(x=>x*2.2),clearanceM:settings.clearanceM};
}
