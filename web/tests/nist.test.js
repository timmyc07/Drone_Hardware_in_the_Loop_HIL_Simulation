import test from 'node:test';
import assert from 'node:assert/strict';
import { randomUUID } from 'node:crypto';
import { NIST_STEPS,NIST_SLOTS,createNistSession,nistReportToJson,parseNistReport } from '../nist.js';
import { createWorld,defaultWorldConfig } from '../world.js';
import { cameraMetadata,DEFAULT_CAMERA_CONFIG } from '../camera.js';

function fixture(airframe = "quadrotor") {
  const config=structuredClone(defaultWorldConfig({startOnGround:true,airframe})); config.scene.nistLane=true;
  const world=createWorld(config), frame=world.snapshot(), vehicleId=frame.vehicles[0].id, session=createNistSession();
  const apply=(type,value,at=frame)=>session.apply({type,value,vehicleId,tick:at.tick,time:at.time},at,config.scene);
  const start=()=>apply('nistStart',{sessionId:randomUUID(),durationSeconds:600});
  const capture=slotId=>{const captureId=randomUUID();return {slotId,captureId,sha256:'a'.repeat(64),metadata:cameraMetadata(frame.vehicles[0].snapshot,DEFAULT_CAMERA_CONFIG,null,{captureId,sourceVehicleId:vehicleId,worldTick:frame.tick})};};
  return {world,frame,session,apply,start,capture};
}
test('official level 1 mapping has 30 steps, 36 unique captures and 40 possible points',()=>{
  assert.equal(NIST_STEPS.length,30);assert.equal(NIST_SLOTS.length,36);assert.equal(new Set(NIST_SLOTS.map(s=>s.id)).size,36);
  assert.deepEqual(NIST_STEPS.slice(0,9).map(s=>s.slots.map(x=>x.targetId)),[['1','2A'],['1','2A'],['1','2A'],['1','3A'],['1','2A'],['2','3A'],['1','2A'],['2','1C'],['L','1A']]);
  assert.equal(NIST_STEPS.reduce((a,s)=>a+(s.landingMaxScore??0),36),40);
});
test('session binds manual scores to unique captured evidence; ended sessions permit pending scores only',()=>{
  const f=fixture();f.start();assert.throws(()=>f.apply('nistScore',{slotId:NIST_SLOTS[0].id,value:1}));
  for(const slot of NIST_SLOTS)f.apply('nistCapture',f.capture(slot.id));
  assert.throws(()=>f.apply('nistCapture',f.capture(NIST_SLOTS[0].id)));
  const landed=structuredClone(f.frame);landed.vehicles[0].snapshot.status='landed';
  for(const phase of ['position','left','right'])f.apply('nistLanding',{phase,value:phase==='position'?2:1,positionNed:landed.vehicles[0].snapshot.position,bodyQuaternionWxyz:landed.vehicles[0].snapshot.quaternion,status:'landed'},landed);
  f.apply('nistEnd',{reason:'manual'});
  for(const slot of NIST_SLOTS)f.apply('nistScore',{slotId:slot.id,value:1});
  const state=f.session.getState();assert.equal(state.totalScore,40);assert.equal(state.assessment,'manual');assert.equal(state.phase,'ended');
  assert.throws(()=>f.apply('nistCapture',f.capture(NIST_SLOTS[0].id)));assert.throws(()=>f.apply('nistScore',{slotId:NIST_SLOTS[0].id,value:0}));
  f.session.reset();assert.equal(f.session.getState().phase,'idle');
});
test('rejects false image pose, unknown identities, fake landing and capture after deadline',()=>{
  const f=fixture();f.start();const bad=f.capture(NIST_SLOTS[0].id);bad.metadata.pose.positionNed[0]+=1;
  assert.throws(()=>f.apply('nistCapture',bad));
  assert.throws(()=>f.apply('nistLanding',{phase:'position',value:2,positionNed:[1,0,0],bodyQuaternionWxyz:[1,0,0,0],status:'landed'}));
  const future=structuredClone(f.frame);future.tick=120000;future.time=600;
  assert.throws(()=>f.apply('nistCapture',f.capture(NIST_SLOTS[0].id),future));
  assert.equal(f.session.getState(200).elapsedSeconds,1);
});

test('strict report round trips partial evidence and rejects tampered identities, scores and sparse landing poses',()=>{
  const f=fixture();f.start();f.apply('nistCapture',f.capture('s01-1'));f.apply('nistScore',{slotId:'s01-1',value:0});f.apply('nistEnd',{reason:'manual'});
  const artifact=parseNistReport(nistReportToJson(f.session.getState()));
  assert.equal(artifact.session.captures.length,1);assert.equal(artifact.definition.guide,'2024A1');
  for(const mutate of [
    r=>r.extra=true,r=>r.version=2,r=>r.definition.maxScore=41,r=>r.session.totalScore=1,
    r=>r.session.captures.push(structuredClone(r.session.captures[0])),
    r=>r.session.captures[0].metadata.sourceVehicleId='another',
    r=>r.session.captures[0].metadata.worldTick=1,
    r=>r.session.scores['s01-2']=1,r=>r.session.scores['s01-1']=2,
    r=>r.session.endTick=10,r=>r.session.termination='fake',r=>r.session.assessment='automatic',
    r=>r.session.phase='idle',r=>r.session.sessionId='fake',r=>r.session.elapsedSeconds=.001,
  ]) {const bad=structuredClone(artifact);mutate(bad);assert.throws(()=>parseNistReport(JSON.stringify(bad)),undefined,mutate.toString());}
  assert.throws(()=>parseNistReport('{'));
});

test('only hovering airframes can start; rejecting fixed wing leaves session untouched',()=>{
  const fixed=fixture('fixedwing'), before=fixed.session.getState();
  assert.throws(()=>fixed.start(),/原地懸停/);assert.deepEqual(fixed.session.getState(),before);
  for(const airframe of ['quadrotor','hexarotor','octorotor','vtol']) {
    const f=fixture(airframe);assert.equal(f.start().phase,'running');
  }
});

test('late manual end cannot hide a missing deadline event in recording reconstruction',()=>{
  const f=fixture();f.start();const later=structuredClone(f.frame);later.tick=120001;later.time=later.tick*.005;
  const before=f.session.getState();
  assert.throws(()=>f.apply('nistEnd',{reason:'manual'},later),/時間上限/);assert.deepEqual(f.session.getState(),before);
  const deadline=structuredClone(f.frame);deadline.tick=120000;deadline.time=600;
  f.apply('nistEnd',{reason:'time-limit'},deadline);assert.equal(f.session.getState().endTick,120000);
});

test('an airborne initial ready snapshot cannot start the ground-only exercise',()=>{
  const config=structuredClone(defaultWorldConfig({startOnGround:false}));config.scene.nistLane=true;
  const world=createWorld(config),frame=world.snapshot(),session=createNistSession();
  assert.equal(frame.vehicles[0].snapshot.status,'ready');assert.ok(frame.vehicles[0].snapshot.ground.clearanceM>0);
  assert.throws(()=>session.apply({type:'nistStart',vehicleId:frame.vehicles[0].id,tick:0,time:0,value:{sessionId:randomUUID(),durationSeconds:600}},frame,config.scene),/地面/);
  assert.equal(session.getState().phase,'idle');
});
