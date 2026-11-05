import { MULTIROTOR_AIRFRAMES } from './rotor-geometry.js';
import { cameraMetadata, validateCameraMetadata } from './camera.js';

const copy = value => structuredClone(value);
const UUID = /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/;
export const NIST_EVENT_TYPES = ['nistStart','nistCapture','nistScore','nistLanding','nistEnd'];
export const NIST_END_REASONS = ['manual','time-limit','vehicle-collision','terrain-coverage','storage-limit','simulation-error'];
const positionTargets = [['1','2A'],['1','2A'],['1','2A'],['1','3A'],['1','2A'],['2','3A'],['1','2A'],['2','1C'],['L','1A']];
const leftTargets = ['1A','1B','2B','3B','3C','3D','2D','1D','1A'];
const rightTargets = ['1A','1D','2D','3D','3C','3B','2B','1B','1A'];
const instructions = [
  '起飛於 1 上方懸停','原位左偏航 360°','原位右偏航 360°','於 1 上方向上爬升','於 1 上方下降','向前至 2 上方','向後至 1 上方','向前至 2 上方並左偏航 180°','向前至起降點上方並右偏航 180°','降落圓內，至少一腳位於圓內',
  '起降點上方懸停','左繞 1 號 90°','向左橫移至 2 號','向左橫移至 3 號','左繞 3 號 90°','左繞 3 號 90°','向左橫移至 2 號','向左橫移至 1 號','左繞 1 號 90°','降落圓內，至少一腳位於圓內',
  '起降點上方懸停','右繞 1 號 90°','向右橫移至 2 號','向右橫移至 3 號','右繞 3 號 90°','右繞 3 號 90°','向右橫移至 2 號','向右橫移至 1 號','右繞 1 號 90°','降落圓內，至少一腳位於圓內',
];
export const NIST_STEPS = Object.freeze(instructions.map((instruction, index) => {
  const phase = ['position','left','right'][Math.floor(index / 10)], local = index % 10;
  const targets = local === 9 ? [] : phase === 'position' ? positionTargets[local] : [(phase === 'left' ? leftTargets : rightTargets)[local]];
  const slots = targets.map((targetId,i) => Object.freeze({id:`s${String(index+1).padStart(2,'0')}-${i+1}`,targetId,stepIndex:index}));
  return Object.freeze({number:index+1,stepIndex:index,phase,instruction,slots:Object.freeze(slots),slotIds:Object.freeze(slots.map(slot=>slot.id)),landingMaxScore:local===9?(phase==='position'?2:1):null});
}));
export const NIST_SLOTS = Object.freeze(NIST_STEPS.flatMap(step=>step.slots));
export const NIST_DEFINITION = Object.freeze({id:'nist-level-1-2023A',scoresheet:'2023A',guide:'2024A1',maxScore:40,assessment:'manual',steps:NIST_STEPS,slots:NIST_SLOTS});
function exact(value, keys) {
  if (!value || ![Object.prototype,null].includes(Object.getPrototypeOf(value)) || Reflect.ownKeys(value).length!==keys.length || keys.some(key=>!Object.hasOwn(value,key))) throw new Error('NIST 資料欄位不正確');
}
function integer(value,min,max) { if (!Number.isInteger(value)||value<min||value>max) throw new Error('NIST 數值超出有效範圍'); return value; }
function same(a,b) {
  if (typeof a==='number') return Number.isFinite(b)&&Math.abs(a-b)<=1e-9*Math.max(1,Math.abs(a));
  if(a===null||typeof a!=='object') return a===b;
  return b&&typeof b==='object'&&Object.keys(a).length===Object.keys(b).length&&Object.keys(a).every(k=>Object.hasOwn(b,k)&&same(a[k],b[k]));
}
export function validateNistEventValue(type,value) {
  if(type==='nistStart') { exact(value,['sessionId','durationSeconds']); if(!UUID.test(value.sessionId)) throw new Error('NIST 練習識別碼不正確'); integer(value.durationSeconds,30,1800); }
  else if(type==='nistCapture') {
    exact(value,['slotId','captureId','metadata','sha256']);
    if(!NIST_SLOTS.some(slot=>slot.id===value.slotId)||!UUID.test(value.captureId)||typeof value.sha256!=='string'||!/^[0-9a-f]{64}$/.test(value.sha256)) throw new Error('NIST 拍攝資料不正確');
    const metadata=validateCameraMetadata(value.metadata);
    if(!MULTIROTOR_AIRFRAMES.includes(metadata.airframe)&&metadata.airframe!=="vtol") throw new Error("NIST 影像需要可原地懸停的載具");
    if(!same(metadata,value.metadata)||metadata.captureId!==value.captureId) throw new Error('NIST 拍攝資訊不正確');
  } else if(type==='nistScore') { exact(value,['slotId','value']); if(!NIST_SLOTS.some(slot=>slot.id===value.slotId)) throw new Error('NIST 拍攝槽不正確'); integer(value.value,0,1); }
  else if(type==='nistLanding') {
    exact(value,['phase','value','positionNed','bodyQuaternionWxyz','status']);
    if(!['position','left','right'].includes(value.phase)||value.status!=='landed') throw new Error('NIST 降落評分需要已降落狀態');
    integer(value.value,0,value.phase==='position'?2:1);
    for(const [key,length] of [['positionNed',3],['bodyQuaternionWxyz',4]]) if(!Array.isArray(value[key])||value[key].length!==length||Reflect.ownKeys(value[key]).length!==length+1||Array.from({length},(_,i)=>!Object.hasOwn(value[key],i)||!Number.isFinite(value[key][i])||Math.abs(value[key][i])>1e9).some(Boolean)) throw new Error('NIST 降落姿態不正確');
    if(Math.abs(value.bodyQuaternionWxyz.reduce((sum,v)=>sum+v*v,0)-1)>1e-6) throw new Error('NIST 降落姿態四元數不正確');
  } else if(type==='nistEnd') { exact(value,['reason']); if(!NIST_END_REASONS.includes(value.reason)) throw new Error('NIST 結束原因不正確'); }
  else throw new Error('未知的 NIST 事件');
  return copy(value);
}
export function createNistSession() {
  let status;
  function reset() { status={phase:'idle',sessionId:null,vehicleId:null,startTick:null,endTick:null,durationSeconds:null,captures:[],scores:{},landings:[],totalScore:0,termination:null,assessment:'manual'}; }
  reset();
  function getState(tick=status.endTick??status.startTick??0) { return copy({...status,elapsedSeconds:status.startTick===null?0:(Math.min(tick,status.endTick??tick)-status.startTick)*.005}); }
  return {
    reset,getState,
    vehicleId: () => status.phase === "running" ? status.vehicleId : null,
    deadlineTick: () => status.phase === "running" ? status.startTick + status.durationSeconds*200 : null,
    apply(event,frame,scene) {
      const {type,tick,vehicleId}=event, value=validateNistEventValue(type,event.value);
      integer(tick,0,360000);
      if(frame.tick!==tick||Math.abs(event.time-tick*.005)>1e-7) throw new Error('NIST 事件缺少對應世界時刻');
      const source=frame.vehicles.find(v=>v.id===vehicleId)?.snapshot;
      if(!source) throw new Error('找不到 NIST 載具');
      if(type==='nistStart') {
        if(!MULTIROTOR_AIRFRAMES.includes(source.airframe)&&source.airframe!=='vtol') throw new Error('NIST Level 1 需要可原地懸停的多旋翼或 VTOL 載具');
        if(status.phase!=='idle'||!scene.nistLane||source.status!=='landed'||tick>=360000) throw new Error('NIST 練習需要指定場地與地面載具；新練習前請重設');
        status={...status,phase:'running',sessionId:value.sessionId,vehicleId,startTick:tick,durationSeconds:value.durationSeconds};
      } else {
        if(status.phase==='idle'||status.vehicleId!==vehicleId||tick<status.startTick) throw new Error('NIST 練習身分或時刻不一致');
        if(type!=='nistScore'&&status.phase!=='running') throw new Error('NIST 練習已結束');
        const deadline = Math.min(360000,status.startTick+status.durationSeconds*200);
        if(type==='nistEnd' && (tick>deadline || (value.reason==='time-limit' && tick!==deadline))) throw new Error('NIST 結束時刻不符合時間上限');
        if(type!=='nistEnd'&&status.phase==='running'&&tick>=deadline) throw new Error('NIST 練習時間已到');
        if(type==='nistCapture') {
          if(status.captures.some(c=>c.slotId===value.slotId||c.captureId===value.captureId)) throw new Error('NIST 拍攝槽或影像識別碼已使用');
          const metadata=cameraMetadata(source,value.metadata.config,scene.geography,{sourceVehicleId:vehicleId,worldTick:tick,captureId:value.captureId});
          if(!same(metadata,value.metadata)) throw new Error('NIST 影像姿態或曝光身分不一致');
          status.captures.push({...value,tick});
        } else if(type==='nistScore') {
          if(!status.captures.some(c=>c.slotId===value.slotId)||Object.hasOwn(status.scores,value.slotId)) throw new Error('NIST 評分需要尚未評分的已拍影像');
          status.scores[value.slotId]=value.value;
        } else if(type==='nistLanding') {
          if(status.landings.some(l=>l.phase===value.phase)||source.status!=='landed'||!same(value.positionNed,source.position)||!same(value.bodyQuaternionWxyz,source.quaternion)) throw new Error('NIST 降落證據不一致');
          status.landings.push({...value,tick});
        } else if(type==='nistEnd') { status.phase='ended'; status.endTick=tick; status.termination=value.reason; }
        status.totalScore=Object.values(status.scores).reduce((a,b)=>a+b,0)+status.landings.reduce((a,l)=>a+l.value,0);
      }
      return getState(tick);
    },
  };
}


export function validateNistReport(report) {
  exact(report,['format','version','definition','session']);
  if(report.format!=='flight-lab-nist-report'||report.version!==1||!same(NIST_DEFINITION,report.definition)) throw new Error('不支援的 NIST 報告格式、版本或練習定義');
  const s=report.session;
  exact(s,['phase','sessionId','vehicleId','startTick','endTick','durationSeconds','captures','scores','landings','totalScore','termination','assessment','elapsedSeconds']);
  if(!['idle','running','ended'].includes(s.phase)||s.assessment!=='manual') throw new Error('NIST 報告必須標示人工判讀');
  const dense=(v,max)=>{if(!Array.isArray(v)||v.length>max||Reflect.ownKeys(v).length!==v.length+1||Array.from({length:v.length},(_,i)=>!Object.hasOwn(v,i)).some(Boolean))throw new Error('NIST 報告清單格式不正確');};
  dense(s.captures,36);dense(s.landings,3);
  if(!s.scores||![Object.prototype,null].includes(Object.getPrototypeOf(s.scores))) throw new Error('NIST 分數格式不正確');
  integer(s.totalScore,0,40);
  if(!Number.isFinite(s.elapsedSeconds)||s.elapsedSeconds<0) throw new Error('NIST 經過時間不正確');
  if(s.phase==='idle') {
    if([s.sessionId,s.vehicleId,s.startTick,s.endTick,s.durationSeconds,s.termination].some(x=>x!==null)||s.captures.length||s.landings.length||Reflect.ownKeys(s.scores).length||s.totalScore||s.elapsedSeconds) throw new Error('NIST 未開始狀態不一致');
    return copy(report);
  }
  if(typeof s.sessionId!=='string'||!UUID.test(s.sessionId)||typeof s.vehicleId!=='string'||!/^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(s.vehicleId)) throw new Error('NIST 報告身分不正確');
  integer(s.startTick,0,360000);integer(s.durationSeconds,30,1800);
  if(Math.abs(s.elapsedSeconds*200-Math.round(s.elapsedSeconds*200))>1e-7||s.elapsedSeconds>s.durationSeconds||s.startTick+s.elapsedSeconds*200>360000+1e-7) throw new Error('NIST 報告超過計時範圍');
  if(s.phase==='ended') {
    integer(s.endTick,s.startTick,360000);
    if(!NIST_END_REASONS.includes(s.termination)||Math.abs(s.elapsedSeconds-(s.endTick-s.startTick)*.005)>1e-7) throw new Error('NIST 結束狀態不一致');
    if(s.termination==='time-limit'&&s.endTick!==Math.min(360000,s.startTick+s.durationSeconds*200)) throw new Error('NIST 結束時刻不符合時間上限');
  } else if(s.endTick!==null||s.termination!==null||s.elapsedSeconds>=s.durationSeconds||s.startTick+s.elapsedSeconds*200>=360000) throw new Error('NIST 執行狀態不一致');
  const maxTick=s.endTick??s.startTick+Math.round(s.elapsedSeconds*200), slots=new Set(), ids=new Set();
  for(const capture of s.captures) {
    exact(capture,['slotId','captureId','metadata','sha256','tick']);
    const {tick,...value}=capture;validateNistEventValue('nistCapture',value);integer(tick,s.startTick,maxTick);
    if(tick>=s.startTick+s.durationSeconds*200) throw new Error('NIST 影像晚於拍攝期限');
    const m=capture.metadata;
    if(slots.has(capture.slotId)||ids.has(capture.captureId)||m.sourceVehicleId!==s.vehicleId||m.worldTick!==tick||Math.abs(m.time-tick*.005)>1e-7) throw new Error('NIST 拍攝槽或曝光身分不一致');
    slots.add(capture.slotId);ids.add(capture.captureId);
  }
  let total=0;
  for(const key of Reflect.ownKeys(s.scores)) {if(!slots.has(key))throw new Error('NIST 分數缺少對應影像');total+=integer(s.scores[key],0,1);}
  const phases=new Set();
  for(const landing of s.landings) {
    exact(landing,['phase','value','positionNed','bodyQuaternionWxyz','status','tick']);
    const {tick,...value}=landing;validateNistEventValue('nistLanding',value);integer(tick,s.startTick,maxTick);
    if(tick>=s.startTick+s.durationSeconds*200) throw new Error('NIST 降落晚於練習期限');
    if(phases.has(landing.phase)||Math.abs(landing.bodyQuaternionWxyz.reduce((sum,v)=>sum+v*v,0)-1)>1e-6) throw new Error('NIST 降落證據或姿態不正確');
    phases.add(landing.phase);total+=landing.value;
  }
  if(total!==s.totalScore) throw new Error('NIST 總分不一致');
  return copy(report);
}
export function nistReportToJson(status) { return JSON.stringify(validateNistReport({format:'flight-lab-nist-report',version:1,definition:NIST_DEFINITION,session:status}),null,2); }
export function parseNistReport(text) {
  if(typeof text!=='string'||new TextEncoder().encode(text).length>256*1024) throw new Error('NIST 報告不得超過 256 KiB');
  let report;try{report=JSON.parse(text);}catch{throw new Error('NIST 報告不是有效 JSON');}
  return validateNistReport(report);
}
