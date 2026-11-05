import test from 'node:test';import assert from 'node:assert/strict';
import {createWorld,defaultWorldConfig} from '../world.js';import {DEFAULT_SCENE} from '../scene-data.js';
import {DEFAULT_COLLISION_PREVENTION} from '../collision-prevention.js';import {getVehicleEnvelope} from '../peer-geometry.js';
const settings={...DEFAULT_COLLISION_PREVENTION,enabled:true};
function world(enabled=true,wall=true) {
  const scene={...structuredClone(DEFAULT_SCENE),obstacles:wall?[{north:8,east:0,baseAltitude:0,width:1,depth:10,height:10}]:[]};
  return createWorld(defaultWorldConfig({}, {mode:'position',collisionPrevention:{...settings,enabled}},scene));
}
const state=w=>w.snapshot().vehicles[0].snapshot;
const advance=(w,n)=>{for(let i=0;i<n;i++)w.step();return state(w);};
test('enabled controller physically stops before the wall while off really crashes and the original target remains',()=>{
  for(const enabled of [false,true]) {
    const w=world(enabled);w.command('drone-1','position',{north:20,east:0,altitude:2});
    let previous=state(w).position,active=false;
    for(let i=0;i<4000;i++) {
      w.step();const s=state(w);active ||= !!s.collisionPrevention?.active;
      assert.ok(Math.hypot(...s.position.map((v,j)=>v-previous[j]))<.1);previous=s.position;
    }
    const s=state(w);assert.deepEqual(s.navigation.targetPosition,[20,0]);
    if(enabled){assert.equal(s.status,'flying');assert.ok(Math.hypot(...s.velocity)<.05);assert.ok(7.5-s.position[0]>=getVehicleEnvelope(w.configuration.vehicles[0].config).radius+settings.clearanceM-.02);assert.equal(active,true);}
    else {assert.equal(s.status,'crashed');assert.equal(s.navigation.reason,'obstacle-collision');}
  }
});
test('empty scene remains flyable and same-tick settings/modes/reset have coherent state',()=>{
  const w=world(true,false);w.command('drone-1','position',{north:20,east:0,altitude:2});
  assert.ok(advance(w,400).position[0]>1);
  w.command('drone-1','mode','attitude');assert.equal(state(w).collisionPrevention.reason,'unsupported-mode');
  w.command('drone-1','mode','position');assert.equal(state(w).collisionPrevention.applicable,true);
  w.command('drone-1','collisionPrevention',{...settings,clearanceM:2});assert.equal(state(w).collisionPrevention.clearanceM,2);
  w.command('drone-1','collisionPrevention',DEFAULT_COLLISION_PREVENTION);assert.equal(state(w).collisionPrevention,null);
  const before=w.snapshot();assert.throws(()=>w.command('drone-1','collisionPrevention'));assert.deepEqual(w.snapshot(),before);assert.throws(()=>w.command('drone-1','collisionPrevention',{...settings,clearanceM:NaN}));assert.deepEqual(w.snapshot(),before);
});
test('the whole fleet observes committed peer positions independently of processing order and rolls back atomically',()=>{
  const config=structuredClone(defaultWorldConfig({}, {mode:'position',collisionPrevention:settings},{...structuredClone(DEFAULT_SCENE),obstacles:[]}));
  const peer=structuredClone(config.vehicles[0]);peer.id='b';peer.spawn.north=12;peer.home=[12,0];config.vehicles.push(peer);
  const worlds=[createWorld(config),createWorld({...structuredClone(config),vehicles:[...config.vehicles].reverse()})];
  for(const w of worlds){w.command('drone-1','position',{north:12,east:0,altitude:2});w.command('b','position',{north:0,east:0,altitude:2});advance(w,1200);}
  const byId=w=>w.snapshot().vehicles.slice().sort((a,b)=>a.id<b.id?-1:1).map(v=>({id:v.id,snapshot:v.snapshot}));
  assert.deepEqual(byId(worlds[0]),byId(worlds[1]));assert.equal(worlds[0].snapshot().collisions.length,0);
  const w=worlds[0],before=w.snapshot(),restore=w.checkpoint();advance(w,10);restore();assert.deepEqual(w.snapshot(),before);
});
test('failed propulsion never gains artificial braking authority or suppresses physical collisions',()=>{
  const w=world(false);w.command('drone-1','position',{north:20,east:0,altitude:2});advance(w,550);
  w.command('drone-1','collisionPrevention',settings);
  for(let i=0;i<4;i++)w.command('drone-1','fault',{index:i,efficiency:0},'motor');
  const s=advance(w,1000);assert.equal(s.status,'crashed');
});

for (const [airframe,heading,target,box] of [
  ['quadrotor',Math.PI/2,[20,0],{north:8,east:0,width:1,depth:30}],
  ['hexarotor',-Math.PI/2,[0,20],{north:0,east:8,width:30,depth:1}],
  ['octorotor',Math.PI/4,[20,20],{north:8,east:0,width:1,depth:30}],
  ['vtol',Math.PI/2,[0,20],{north:0,east:8,width:30,depth:1}],
]) test(`${airframe} rotated ${heading} physically brakes on its commanded horizontal approach`,()=>{
  const input=structuredClone(defaultWorldConfig({airframe},{mode:'position',collisionPrevention:settings},{...structuredClone(DEFAULT_SCENE),obstacles:[{...box,height:15,baseAltitude:0}]}));
  input.vehicles[0].spawn.heading=heading;
  const w=createWorld(input);w.command('drone-1','position',{north:target[0],east:target[1],altitude:2});
  let active=false,travelled=0;
  for(let i=0;i<3000;i++){w.step();const s=state(w);active ||= !!s.collisionPrevention.active;travelled=Math.max(travelled,Math.hypot(...s.position.slice(0,2)));}
  const s=state(w),axis=box.width===1?0:1;
  assert.equal(s.status,'flying');assert.equal(active,true);assert.ok(travelled>1);
  assert.ok(7.5-s.position[axis]>=getVehicleEnvelope(input.vehicles[0].config).radius+settings.clearanceM-.03);
  assert.ok(Math.hypot(...s.velocity.slice(0,2))<.1);
  assert.deepEqual(s.navigation.targetPosition,target);
});

for (const [name,altitude,baseAltitude,height,blocked] of [
  ['clear above obstacle',6,0,3,false],['clear below elevated obstacle',2,5,2,false],
  ['upper clearance envelope intersects obstacle',4.2,0,3,true],['lower clearance envelope intersects elevated obstacle',4,5,2,true],
]) test(`${name} preserves the expanded AABB vertical boundary during true horizontal flight`,()=>{
  const input=structuredClone(defaultWorldConfig({}, {mode:'position',collisionPrevention:settings},{...structuredClone(DEFAULT_SCENE),obstacles:[{north:8,east:0,width:1,depth:30,baseAltitude,height}]}));
  input.vehicles[0].spawn.altitude=altitude;
  const w=createWorld(input);w.command('drone-1','position',{north:20,east:0,altitude});
  let active=false;for(let i=0;i<2400;i++){w.step();active ||= state(w).collisionPrevention.active;}
  const s=state(w);assert.equal(s.status,'flying');assert.equal(active,blocked);
  if(blocked) assert.ok(s.position[0]<7.5);else assert.ok(s.position[0]>12);
  assert.ok(Math.abs(-s.position[2]-altitude)<.02);
});

test('minimum legal braking acceleration remains flyable and survives native record validation and independent replay',async()=>{
  const {RECORDING_VERSION,MODEL_VERSION,compactWorldSnapshot,worldSensorFrame,validateRecording,recordingToJson}=await import('../recording.js');
  const {replayWorldRecording}=await import('../../tools/replay-world.mjs');
  const config=defaultWorldConfig({}, {mode:'position',collisionPrevention:{...settings,brakeAccelerationMps2:.1}},{...structuredClone(DEFAULT_SCENE),obstacles:[{north:200,east:0,width:1,depth:30,baseAltitude:0,height:10}]});
  const w=createWorld(config),value={north:20,east:0,altitude:2};
  const record={format:'flight-lab',version:RECORDING_VERSION,modelVersion:MODEL_VERSION,createdAt:'2026-11-05T00:00:00.000Z',dt:.005,world:w.configuration,runtime:{speed:1,wallSeconds:1,p95LateMs:0,lateTicks:0,skippedWallSeconds:0,renderEnabled:true},events:[{type:'position',vehicleId:'drone-1',tick:0,time:0,value}],samples:[],sensorFrames:[],termination:null};
  w.command('drone-1','position',value);
  for(let i=0;i<=400;i++) {if(i%10===0){const frame=w.snapshot();record.samples.push(compactWorldSnapshot(frame));record.sensorFrames.push(worldSensorFrame(frame));}if(i<400)w.step();}
  assert.ok(state(w).position[0]>1);assert.equal(state(w).collisionPrevention.active,false);
  const clean=validateRecording(JSON.parse(recordingToJson(record)));
  assert.equal(replayWorldRecording(clean).passed,true);
});
