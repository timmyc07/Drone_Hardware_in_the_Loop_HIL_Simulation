import test from 'node:test';
import assert from 'node:assert/strict';
import {createWorld,defaultWorldConfig,validateWorldConfig} from '../world.js';
import {DEFAULT_SCENE} from '../scene-data.js';
import {DEFAULT_FOLLOW} from '../follow.js';
import {DEFAULT_COLLISION_PREVENTION} from '../collision-prevention.js';

const settings=extra=>({...structuredClone(DEFAULT_FOLLOW),leaderId:'leader',...extra});
function fleet(airframe='quadrotor') {
  const value=structuredClone(defaultWorldConfig({airframe},{mode:'position'},{...structuredClone(DEFAULT_SCENE),obstacles:[]}));
  value.vehicles[0].id='follower';
  const leader=structuredClone(value.vehicles[0]);leader.id='leader';leader.config.airframe='quadrotor';leader.spawn.north=5;leader.home=[5,0];value.vehicles.push(leader);return value;
}
const actor=(w,id='follower')=>w.getVehicle(id).snapshot;
const advance=(w,ticks)=>{for(let i=0;i<ticks;i++)w.step();return w.snapshot();};
function start(w,extra){w.command('follower','follow',settings(extra));w.command('follower','startFollow');}
const wrap=x=>Math.atan2(Math.sin(x),Math.cos(x));

for(const airframe of ['quadrotor','hexarotor','octorotor','vtol']) test(`${airframe} physically follows translating and climbing leader with bounded acceleration and small steady offset error`,()=>{
  const w=createWorld(fleet(airframe));start(w,{offset:{x:-5,y:0,altitude:1}});
  let sum=0,vertical=0,count=0,previous=actor(w).position;
  for(let i=0;i<2400;i++) {
    if(i%20===0)w.command('leader','position',{north:5+1.2*i*.005,east:.4*i*.005,altitude:2+Math.min(2,i*.005*.25)});
    const before=actor(w,'leader'),frame=w.step(),f=frame.vehicles[0].snapshot,l=frame.vehicles[1].snapshot;
    assert.equal(f.follow.sourceTick,i);assert.deepEqual(f.follow.targetPosition,[before.position[0]-5,before.position[1],before.position[2]-1]);
    assert.equal(f.follow.active,true);assert.ok(Math.hypot(...f.position.map((v,j)=>v-previous[j]))<.05);previous=f.position;
    const {roll,pitch}=f.target;assert.ok(Math.hypot(Math.tan(roll)*9.80665,Math.tan(pitch)*9.80665)<3.01);
    if(i>1600){sum+=Math.hypot(f.position[0]-l.position[0]+5,f.position[1]-l.position[1]);vertical+=Math.abs(f.position[2]-l.position[2]+1);count++;}
  }
  assert.ok(sum/count<.2,`horizontal mean ${sum/count}`);assert.ok(vertical/count<.15,`height mean ${vertical/count}`);assert.ok(actor(w).position[0]>5);
});

test('world configuration and commands validate all edges atomically; settings never arm or launch ground followers',()=>{
  const input=fleet(),w=createWorld(input);assert.equal(actor(w).follow,null);
  for(const invalid of [undefined,settings({leaderId:'follower'}),settings({leaderId:'missing'}),settings({offset:{x:NaN,y:0,altitude:0}})]) {const before=w.snapshot();assert.throws(()=>w.command('follower','follow',invalid));assert.deepEqual(w.snapshot(),before);}
  w.command('follower','follow',settings());const before=w.snapshot();assert.throws(()=>w.command('leader','follow',settings({leaderId:'follower'})));assert.deepEqual(w.snapshot(),before);
  const cyclic=fleet();cyclic.vehicles[0].setup.follow=settings();cyclic.vehicles[1].setup.follow=settings({leaderId:'follower'});assert.throws(()=>validateWorldConfig(cyclic));
  const ground=fleet();ground.vehicles[0].config.startOnGround=true;ground.vehicles[0].spawn.altitude=0;const g=createWorld(ground);g.command('follower','follow',settings());assert.throws(()=>g.command('follower','startFollow'));advance(g,100);assert.equal(actor(g).ground.clearanceM,0);assert.equal(actor(g).armed,false);
  g.command('follower','takeoff');advance(g,600);g.command('follower','mode','follow');assert.equal(actor(g).follow.active,true);
  const initial=fleet();initial.vehicles[0].setup.follow=settings();initial.vehicles[0].setup.mode='follow';assert.equal(actor(createWorld(initial)).follow.active,true);initial.vehicles[0].config.startOnGround=true;initial.vehicles[0].spawn.altitude=0;assert.throws(()=>createWorld(initial));
  const wing=fleet('fixedwing'),fw=createWorld(wing);assert.throws(()=>fw.command('follower','follow',settings()));assert.throws(()=>fw.command('follower','startFollow'));
});

test('same-tick active edits are atomic, manual conflicts reject, stop captures truth and safety cannot be cancelled',()=>{
  const w=createWorld(fleet());start(w);advance(w,200);let before=w.snapshot();
  for(const [type,value] of [['targets',{altitude:5}],['position',{north:0,east:0,altitude:3}],['follow',settings({offset:{x:0,y:0,altitude:-10}})]]){assert.throws(()=>w.command('follower',type,value));assert.deepEqual(w.snapshot(),before);}
  w.command('follower','follow',settings({offset:{x:-4,y:1,altitude:1}}));assert.equal(actor(w).follow.active,true);assert.deepEqual(actor(w).follow.targetPosition,[actor(w,'leader').position[0]-4,actor(w,'leader').position[1]+1,actor(w,'leader').position[2]-1]);
  const position=actor(w).position;w.command('follower','stopFollow');const stopped=actor(w);assert.equal(stopped.follow.reason,'pilot');assert.equal(stopped.navigation.mode,'position');assert.deepEqual(stopped.navigation.targetPosition,position.slice(0,2));advance(w,20);assert.deepEqual(actor(w).follow,stopped.follow);
  w.command('follower','startFollow');w.command('follower','mode','position');assert.equal(actor(w).follow.reason,'mode-change');w.command('follower','position',{north:0,east:0,altitude:2});
  w.command('follower','startFollow');before=w.snapshot();const restore=w.checkpoint();w.command('follower','follow',settings({offset:{x:-6,y:0,altitude:0}}));advance(w,10);restore();assert.deepEqual(w.snapshot(),before);
  w.command('follower','fault',{lost:true},'link');w.step();assert.equal(actor(w).navigation.reason,'link-loss');assert.equal(actor(w).follow.reason,'safety-action');w.command('follower','stopFollow');assert.equal(actor(w).navigation.mode,'rtl');assert.throws(()=>w.command('follower','startFollow'));
});

test('leader-heading offsets use frozen yaw, rotate physically through a turn, and yaw modes track without teleporting',()=>{
  const input=fleet();input.vehicles[1].spawn.heading=Math.PI/2;const w=createWorld(input);start(w,{frame:'leader-heading',offset:{x:-5,y:1,altitude:1},yawMode:'leader'});assert.ok(Math.hypot(...actor(w).follow.targetPosition.map((v,i)=>v-[4,-5,-3][i]))<1e-10);advance(w,2400);
  assert.ok(Math.abs(wrap(actor(w).attitude[2]-actor(w,'leader').attitude[2]))<.03);
  w.command('leader','mode','attitude');w.command('leader','targets',{yawRate:.25});let previous=actor(w).quaternion;
  for(let i=0;i<800;i++){const leader=actor(w,'leader');w.step();const f=actor(w),h=leader.attitude[2];assert.ok(Math.abs(f.follow.targetPosition[0]-(leader.position[0]-5*Math.cos(h)-Math.sin(h)))<1e-10);assert.ok(Math.abs(f.follow.targetPosition[1]-(leader.position[1]-5*Math.sin(h)+Math.cos(h)))<1e-10);assert.ok(Math.hypot(...f.quaternion.map((v,j)=>v-previous[j]))<.02);previous=f.quaternion;}
  w.command('leader','targets',{yawRate:0});advance(w,2200);const f=actor(w),l=actor(w,'leader');assert.ok(Math.hypot(...f.position.map((v,j)=>v-f.follow.targetPosition[j]))<.4);assert.ok(Math.abs(wrap(f.attitude[2]-l.attitude[2]))<.03);
  w.command('follower','follow',settings({frame:'leader-heading',offset:{x:-5,y:1,altitude:1},yawMode:'face-leader'}));advance(w,1800);const face=actor(w),leader=actor(w,'leader');assert.ok(Math.abs(wrap(face.attitude[2]-Math.atan2(leader.position[1]-face.position[1],leader.position[0]-face.position[0])))<.04);
  w.command('follower','follow',settings({yawMode:'hold'}));const heading=actor(w).attitude[2];advance(w,1400);assert.ok(Math.abs(wrap(actor(w).attitude[2]-heading))<.05);
});

test('processing order and renamed leader-before-follower IDs use exactly one committed source; checkpoint restores source history',()=>{
  const a=fleet(),b=structuredClone(a);b.vehicles.reverse();const worlds=[createWorld(a),createWorld(b)];for(const w of worlds){start(w);w.command('leader','position',{north:15,east:3,altitude:4});advance(w,500);}assert.deepEqual(worlds[0].snapshot(),worlds[1].snapshot());
  const renamed=fleet();renamed.vehicles[0].id='z-follower';renamed.vehicles[1].id='a-leader';const other=createWorld(renamed);other.command('z-follower','follow',settings({leaderId:'a-leader'}));other.command('z-follower','startFollow');other.command('a-leader','position',{north:15,east:3,altitude:4});advance(other,0);
  for(let i=0;i<500;i++)other.step();const original=actor(worlds[0]),changed=actor(other,'z-follower');assert.deepEqual(changed.position,original.position);assert.deepEqual(changed.velocity,original.velocity);assert.deepEqual(changed.follow.targetPosition,original.follow.targetPosition);
  const restore=other.checkpoint(),saved=other.snapshot();advance(other,20);restore();assert.deepEqual(other.snapshot(),saved);
});

for(const lostAction of ['hold','rtl'])test(`leader disarm and link loss stop with ${lostAction}, preserve last target, and never automatically restart`,()=>{
  const w=createWorld(fleet());start(w,{lostAction});advance(w,100);const last=actor(w).follow;w.command('leader','disarm');w.step();assert.equal(actor(w).follow.reason,'leader-disarmed');assert.equal(actor(w).navigation.mode,lostAction==='hold'?'position':'rtl');assert.deepEqual(actor(w).follow.targetPosition,last.targetPosition);w.command('leader','arm');advance(w,50);assert.equal(actor(w).follow.active,false);
  const linked=createWorld(fleet());start(linked,{lostAction});linked.command('leader','fault',{lost:true},'link');linked.step();assert.equal(actor(linked).follow.reason,'leader-link-loss');assert.equal(actor(linked).navigation.mode,lostAction==='hold'?'position':'rtl');
});

test('target distance and moving inclusion/exclusion boundaries stop follow without aborting the world tick',()=>{
  for(const kind of ['range','radius','inclusion','exclusion','height']){
    const input=fleet();if(kind==='radius')input.vehicles[0].config.geofenceRadius=7;if(kind==='height')input.vehicles[0].config.geofenceHeight=3;
    const w=createWorld(input);if(kind==='inclusion')w.command('follower','geofenceZones',{enabled:true,zones:[{id:'allow',shape:'circle',kind:'inclusion',center:[0,0],radiusM:7}]});if(kind==='exclusion')w.command('follower','geofenceZones',{enabled:true,zones:[{id:'block',shape:'circle',kind:'exclusion',center:[8,0],radiusM:1}]});
    start(w,{maxDistanceM:kind==='range'?6:100});w.command('leader','position',{north:kind==='range'?30:20,east:0,altitude:kind==='height'?5:2});let stopped=false;
    for(let i=0;i<2400;i++){w.step();if(!actor(w).follow.active){stopped=true;break;}}
    assert.equal(stopped,true,kind);assert.equal(actor(w).follow.reason,kind==='range'?'target-range':'target-geofence');const tick=w.snapshot().tick;w.step();assert.equal(w.snapshot().tick,tick+1);assert.equal(actor(w).navigation.mode,'position');
  }
});

test('VTOL start guards and active cruise rejection are enforced in the world commands',()=>{
  const w=createWorld(fleet('vtol'));start(w);let before=w.snapshot();assert.throws(()=>w.command('follower','transition','cruise'));assert.deepEqual(w.snapshot(),before);
  w.command('follower','stopFollow');w.command('follower','position',{north:0,east:0,altitude:12});advance(w,1800);w.command('follower','transition','cruise');w.command('follower','follow',settings());before=w.snapshot();assert.throws(()=>w.command('follower','startFollow'));assert.deepEqual(w.snapshot(),before);
});

test('leader static-obstacle crash stops the follower at the next committed source and leaves the world running',()=>{
  const input=fleet();input.scene.obstacles=[{north:8,east:0,width:1,depth:10,baseAltitude:0,height:5}];const w=createWorld(input);start(w);w.command('leader','position',{north:20,east:0,altitude:2});
  let crashed=false;for(let i=0;i<1800;i++){w.step();if(actor(w,'leader').status==='crashed'){crashed=true;break;}}assert.equal(crashed,true);assert.equal(actor(w).follow.active,true);w.step();assert.equal(actor(w).follow.reason,'leader-crashed');assert.equal(actor(w).navigation.mode,'position');const p=actor(w).position;advance(w,600);assert.ok(Math.hypot(...actor(w).position.map((v,i)=>v-p[i]))<.6);
});

test('follow uses existing physical collision prevention and retains the original requested target while braking',()=>{
  const input=fleet();input.vehicles[1].spawn.north=12;input.vehicles[1].home=[12,0];input.vehicles[0].setup.collisionPrevention={...DEFAULT_COLLISION_PREVENTION,enabled:true};input.scene.obstacles=[{north:8,east:0,width:1,depth:20,baseAltitude:0,height:5}];const w=createWorld(input);start(w);w.command('leader','position',{north:25,east:0,altitude:2});let active=false;
  for(let i=0;i<3000;i++){w.step();active ||= actor(w).collisionPrevention.active;}
  const f=actor(w);assert.equal(active,true);assert.equal(f.follow.active,true);assert.equal(f.status,'flying');assert.ok(f.position[0]<6);assert.ok(f.follow.targetPosition[0]>19);assert.ok(Math.hypot(...f.velocity.slice(0,2))<.1);assert.ok(Math.hypot(...f.collisionPrevention.commandedAccelerationNe)<=3.000001);
});

test('candidate terrain coverage and ground rejection stop locally without aborting world physics',()=>{
  const input=fleet();input.scene.terrain={kind:'dem',north:-10,east:-10,cellSize:10,rows:5,cols:3,heights:Array(15).fill(0)};
  input.scene.geography={origin:{latitudeDeg:25.033,longitudeDeg:121.5654,altitudeM:20,verticalDatum:'unspecified'},source:{name:'test.tif',sha256:'a'.repeat(64),crs:'EPSG:4326',width:3,height:5,band:1,pixelInterpretation:'point',heightUnit:'m',scale:1,offset:0,resampling:'bilinear',window:[0,0,3,5]}};
  const w=createWorld(input);start(w,{offset:{x:10,y:5,altitude:0}});w.command('leader','position',{north:28,east:0,altitude:2});let stopped=false;
  for(let i=0;i<2000;i++){w.step();if(!actor(w).follow.active){stopped=true;break;}}assert.equal(stopped,true);assert.equal(actor(w).follow.reason,'target-terrain');const tick=w.snapshot().tick;advance(w,50);assert.equal(w.snapshot().tick,tick+50);
  const ground=createWorld(fleet());start(ground,{offset:{x:-5,y:0,altitude:-1}});ground.command('leader','mode','land');stopped=false;for(let i=0;i<1400;i++){ground.step();if(!actor(ground).follow.active){stopped=true;break;}}assert.equal(stopped,true);assert.equal(actor(ground).follow.reason,'target-ground');ground.step();
});

test('critical battery and own geofence safety take priority over active follow and cannot be cleared by stop',()=>{
  const input=fleet();Object.assign(input.vehicles[0].config,{batteryCapacityWh:.1,batteryInitialSoc:.2001,lowBatteryThreshold:.2,criticalBatteryThreshold:.1999});const w=createWorld(input);start(w);advance(w,2);assert.equal(actor(w).navigation.mode,'land');assert.equal(actor(w).navigation.reason,'critical-battery');assert.equal(actor(w).follow.reason,'safety-action');w.command('follower','stopFollow');assert.equal(actor(w).navigation.reason,'critical-battery');assert.throws(()=>w.command('follower','mode','follow'));
  const fenceInput=fleet();Object.assign(fenceInput.vehicles[0].config,{geofenceRadius:1,drag:3});const f=createWorld(fenceInput);start(f);f.command('follower','gust');let stopped=false;for(let i=0;i<1000;i++){f.step();if(!actor(f).follow.active){stopped=true;break;}}assert.equal(stopped,true);assert.equal(actor(f).navigation.reason,'geofence');assert.equal(actor(f).follow.reason,'safety-action');
});
