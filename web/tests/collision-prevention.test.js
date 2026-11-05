import test from 'node:test';
import assert from 'node:assert/strict';
import { DEFAULT_COLLISION_PREVENTION, validateCollisionPreventionSettings, validateCollisionPreventionState, constrainHorizontalMotion } from '../collision-prevention.js';
import { obstacleBounds } from '../obstacle-geometry.js';
import { createSimulation } from '../sim.js';
import { createNistLaneCollider } from '../nist-geometry.js';
const settings={...DEFAULT_COLLISION_PREVENTION,enabled:true};
const run=(patch={})=>constrainHorizontalMotion({settings,positionNed:[0,0,-2],velocityNe:[0,0],desiredVelocityNe:[3,0],radius:.37,obstacles:[],nistCollider:null,peers:[],...patch});
const box=(north,east,width=1,depth=1)=>obstacleBounds({north,east,width,depth,height:10,baseAltitude:0},.37);
test('strict complete settings and state contracts reject malformed controls',()=>{
  assert.deepEqual(validateCollisionPreventionSettings(),DEFAULT_COLLISION_PREVENTION);
  for(const value of [{enabled:true},{...settings,clearanceM:Infinity},{...settings,responseDelaySeconds:-1},{...settings,brakeAccelerationMps2:0},{...settings,extra:true}])assert.throws(()=>validateCollisionPreventionSettings(value));
  assert.equal(validateCollisionPreventionState(null),null);assert.deepEqual(validateCollisionPreventionState(run()),run());
  assert.throws(()=>validateCollisionPreventionState({...run(),desiredVelocityNe:[1,2,3]}));
  for(const patch of [{desiredVelocityNe:Array(2)},{commandedAccelerationNe:[1e308,0]},{reason:'peer',active:true,obstacleType:'peer',obstacleId:'=SUM(1,2)'},{reason:'peer',active:true,obstacleType:'peer',obstacleId:null},{active:true},{reason:'obstacle',active:true,obstacleType:'obstacle',obstacleId:'50'}])assert.throws(()=>validateCollisionPreventionState({...run(),...patch}));
});
test('open scene permits motion while obstacles constrain real requested speed',()=>{
  assert.equal(run().active,false);
  const stopped=run({obstacles:[box(3,0,1,10)]});
  assert.equal(stopped.active,true);assert.ok(stopped.constrainedVelocityNe[0]<3);assert.equal(stopped.obstacleType,'obstacle');
});
test('existing forward momentum is checked even when requesting reversal',()=>{
  const result=run({velocityNe:[3,0],desiredVelocityNe:[-3,0],obstacles:[box(3,0,1,10)]});
  assert.equal(result.reason,'insufficient-clearance');assert.ok(result.commandedAccelerationNe[0]<0);
});
test('turning trajectories check curved swept volume rather than only velocity rays',()=>{
  const result=run({settings:{...settings,clearanceM:0,responseDelaySeconds:0},velocityNe:[2,0],desiredVelocityNe:[0,2],obstacles:[box(1.9,.65,.1,.1)]});
  assert.equal(result.active,true);
});
test('relative peer prediction detects approaching vehicles and stable IDs resolve ties',()=>{
  const peer=id=>({id,position:[6,0,-2],velocity:[-2,0,0],radius:.4});
  const first=run({peers:[peer('b'),peer('a')]});
  assert.equal(first.obstacleType,'peer');assert.equal(first.obstacleId,'a');
  assert.deepEqual(first,run({peers:[peer('a'),peer('b')]}));
});
test('NIST collision prevention uses the shared swept mesh collider',()=>{
  const result=run({positionNed:[0,0,-.15],desiredVelocityNe:[3,0],nistCollider:createNistLaneCollider()});
  assert.equal(result.active,true);assert.equal(result.obstacleType,'nist');
});
test('unsupported plants and VTOL transition boundaries reject enabling without mutation',()=>{
  for(const [config,options] of [[{airframe:'fixedwing'},{}],[{startOnGround:true},{controlSource:'external'}]]) {
    const sim=createSimulation(config,options),before=sim.snapshot();
    assert.throws(()=>sim.setCollisionPrevention(settings));assert.deepEqual(sim.snapshot(),before);
    assert.throws(()=>createSimulation(config,{...options,collisionPrevention:settings}));
  }
  const vtol=createSimulation({airframe:'vtol'},{spawn:{north:0,east:0,altitude:12,heading:0}});
  vtol.setCollisionPrevention(settings);assert.throws(()=>vtol.setTransition('cruise'));
  vtol.setCollisionPrevention(DEFAULT_COLLISION_PREVENTION);vtol.setTransition('cruise');assert.throws(()=>vtol.setCollisionPrevention(settings));
});


test('minimum braking acceleration checks its full long horizon without blocking empty space or inventing an obstacle',()=>{
  const slow={...settings,brakeAccelerationMps2:.1};
  let sweeps=0,lastPoint;
  const clear=run({settings:slow,nistCollider:{sweep(start,end){sweeps++;lastPoint=end;return null;}}});
  assert.equal(clear.active,false);assert.deepEqual(validateCollisionPreventionState(clear),clear);
  assert.ok(sweeps<=192);assert.ok(lastPoint[0]>89.99);
  const far=run({settings:slow,obstacles:[box(200,0,1,10)]});assert.equal(far.active,false);validateCollisionPreventionState(far);
  const tail=run({settings:slow,obstacles:[box(80,0,1,10)]});assert.equal(tail.active,true);assert.equal(tail.obstacleType,'obstacle');validateCollisionPreventionState(tail);
  const unbounded=run({velocityNe:[1e308,0]});assert.equal(unbounded.reason,'insufficient-clearance');validateCollisionPreventionState(unbounded);
  const invalidPeer=run({peers:[{id:'fast',position:[0,100,-2],velocity:[1e150,0,0],radius:.4}]});
  assert.equal(invalidPeer.reason,'insufficient-clearance');validateCollisionPreventionState(invalidPeer);
});
