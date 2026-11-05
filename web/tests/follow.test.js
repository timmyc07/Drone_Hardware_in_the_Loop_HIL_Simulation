import test from 'node:test';
import assert from 'node:assert/strict';
import {DEFAULT_FOLLOW,validateFollowSettings,validateFollowState,validateFollowRoster} from '../follow.js';
import {createSimulation} from '../sim.js';

const settings=extra=>({...structuredClone(DEFAULT_FOLLOW),leaderId:'leader',...extra});
const active={active:true,leaderId:'leader',reason:'',sourceTick:0,targetPosition:[5,0,-2]};

test('follow settings and runtime states reject malformed, sparse, nonfinite and inconsistent values',()=>{
  const copy=validateFollowSettings();copy.offset.x=10;assert.equal(DEFAULT_FOLLOW.offset.x,-5);
  for(const bad of [null,{}, {...settings(),extra:1},settings({leaderId:'=SUM(A1)'}),settings({leaderId:''}),settings({frame:'body'}),settings({maxDistanceM:0}),settings({maxDistanceM:501}),settings({lostAction:'land'}),settings({yawMode:'auto'}),settings({offset:{x:0,y:0,altitude:Infinity}}),settings({offset:{x:101,y:0,altitude:0}}),settings({offset:{x:0,y:0,altitude:0,z:0}})]) assert.throws(()=>validateFollowSettings(bad));
  assert.equal(validateFollowState(null),null);
  assert.deepEqual(validateFollowState(active),active);
  for(const bad of [undefined,{}, {...active,sourceTick:-1},{...active,sourceTick:360001},{...active,reason:'pilot'},{...active,active:false},{...active,leaderId:null},{...active,targetPosition:Array(3)},{...active,targetPosition:[0,NaN,0]},{...active,extra:0}]) assert.throws(()=>validateFollowState(bad));
  assert.equal(validateFollowState({...active,active:false,reason:'pilot'}).reason,'pilot');
});

test('configured roster edges reject self, unknown and inactive multi-hop cycles',()=>{
  const roster=(edges)=>Object.entries(edges).map(([id,leaderId])=>({id,setup:{follow:settings({leaderId})}}));
  validateFollowRoster(roster({a:'b',b:'c',c:null}));
  for(const edges of [{a:'a'},{a:'missing'},{a:'b',b:'a'},{a:'b',b:'c',c:'a'}])assert.throws(()=>validateFollowRoster(roster(edges)));
});

test('simulation source clock is consumed at its exact committed tick and stale loss holds without reactivation',()=>{
  const sim=createSimulation({}, {follow:settings({offset:{x:-5,y:0,altitude:0}})}),leader=createSimulation({}, {spawn:{north:5,east:0,altitude:2,heading:0},home:[5,0]});
  sim.setFollowSource({tick:0,leaderId:'leader',snapshot:leader.snapshot()});sim.startFollow();
  sim.step();assert.equal(sim.snapshot().follow.active,true);assert.equal(sim.snapshot().follow.sourceTick,0);
  sim.step();const lost=sim.snapshot();assert.equal(lost.follow.reason,'leader-stale');assert.equal(lost.navigation.mode,'position');
  sim.setFollowSource({tick:2,leaderId:'leader',snapshot:{...leader.snapshot(),time:sim.snapshot().time}});sim.step();assert.deepEqual(sim.snapshot().follow,lost.follow);
  sim.setFollowSettings(settings());assert.equal(sim.snapshot().follow,null);sim.reset();assert.equal(sim.snapshot().follow,null);assert.deepEqual(sim.getFollowSettings(),settings({offset:{x:-5,y:0,altitude:0}}));
  assert.throws(()=>createSimulation({airframe:'fixedwing'},{follow:settings()}));
  const external=createSimulation({startOnGround:true}, {controlSource:'external'});assert.throws(()=>external.setFollowSettings(settings()));assert.throws(()=>external.startFollow());assert.equal(external.snapshot().follow,null);
});

test('missing, stale and malformed truth sources reject follow before changing any flight state',()=>{
  const leader=createSimulation({}, {spawn:{north:5,east:0,altitude:2,heading:0},home:[5,0]}).snapshot();
  const source={tick:0,leaderId:'leader',snapshot:leader};
  for(const [reason,input] of [
    ['leader-missing',null],
    ['leader-stale',{...source,tick:1}],
    ['leader-invalid',{...source,snapshot:{...leader,velocity:Array(3)}}],
    ['leader-invalid',{...source,snapshot:{...leader,position:[NaN,0,-2]}}],
    ['leader-disarmed',{...source,snapshot:{...leader,armed:false}}],
    ['leader-crashed',{...source,snapshot:{...leader,status:'crashed'}}],
    ['leader-failsafe',{...source,snapshot:{...leader,status:'failsafe'}}],
    ['leader-link-loss',{...source,snapshot:{...leader,faults:{...leader.faults,linkLost:true}}}],
    ['leader-landed',{...source,snapshot:{...leader,status:'landed'}}],
  ]) {
    const sim=createSimulation({}, {follow:settings()});sim.setFollowSource(input);const before=sim.snapshot();assert.throws(()=>sim.startFollow(),new RegExp(reason));assert.deepEqual(sim.snapshot(),before);
  }
});
