import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {NIST_LANE,createNistLaneGroup,createNistLaneCollider} from '../nist-geometry.js';
const collider=createNistLaneCollider();
const add=(p,d,s)=>p.map((v,i)=>v+d[i]*s);
test('fixed SI lane has 15 open buckets with prescribed axes',()=>{
 assert.equal(NIST_LANE.targets.length,15);
 for(const t of NIST_LANE.targets){assert.equal(t.baseNED[0]-(t.axisNED[0] ? Math.sign(t.axisNED[0])*.185:0),t.station*3);assert.ok(Math.abs(Math.hypot(...t.axisNED)-1)<1e-12);const origin=add(t.openingNED,t.axisNED,.15);const distance=collider.ray(origin,t.axisNED.map(v=>-v));assert.ok(Math.abs(distance-.4)<1e-6,`${t.id} center reaches bottom: ${distance}`);}
});
test('CPU rays use exact renderer triangles on both sides',()=>{
 const group=createNistLaneGroup();const raycaster=new THREE.Raycaster();
 const toThree=p=>new THREE.Vector3(p[1],-p[2],-p[0]);
 for(const t of NIST_LANE.targets){for(const offset of [0,.07,.14,.2]){const origin=add(t.openingNED,t.axisNED,.4);origin[1]+=offset;const direction=t.axisNED.map(v=>-v);raycaster.set(toThree(origin),toThree(direction));const expected=raycaster.intersectObject(group,true)[0]?.distance??null;const actual=collider.ray(origin,direction);assert.equal(actual===null,expected===null);if(expected!==null)assert.ok(Math.abs(actual-expected)<1e-7);}}
 const bottom=NIST_LANE.targets[0].baseNED;assert.ok(collider.ray(add(bottom,[0,0,1],.02),[0,0,-1])<.02);
});
test('sphere sweep catches thin walls, bottom, contact and stationary overlap',()=>{
 const t=NIST_LANE.targets[0],start=add(t.baseNED,t.axisNED,1),end=add(t.baseNED,t.axisNED,-1);
 const hit=collider.sweep(start,end,.02);assert.ok(Math.abs(hit-.49)<1e-6,`${hit}`);
 assert.equal(collider.sweep(t.baseNED,t.baseNED,.02),0);assert.equal(collider.overlaps(t.baseNED,.02),true);
 assert.equal(collider.overlaps(add(t.baseNED,t.axisNED,.1),.01),false);
 assert.equal(collider.sweep([20,0,-1],[21,0,-1],.1),null);
 assert.equal(collider.sweep(add(t.baseNED,t.axisNED,.02),start,.02),0);
 assert.ok(collider.sweep([3,-1,-.3],[3,1,-.3],.01)!==null);
});
test('sensor ray hot path supports 200Hz batches without browser globals',()=>{
 assert.equal(typeof document,'undefined');const before=performance.now();for(let i=0;i<2000;i++)collider.ray([3,0,-1],[0,0,1],10);const ms=performance.now()-before;assert.ok(ms<1000,`${ms}ms / 2000 rays`);console.log(`NIST ray benchmark: ${ms.toFixed(1)}ms / 2000 rays`);
});

test('rangefinder and dependent flow use NIST bottom; scene spawn/edit reject apparatus overlap',async()=>{
 const {createSensorSuite}=await import('../sensors.js');
 const suite=createSensorSuite({latency:0,rangefinderNoiseM:0,opticalFlowNoiseRadS:0});suite.setNistLane(true);
 const state={time:0,position:[3,0,-1],velocity:[0,1,0],quaternion:[1,0,0,0],rates:[0,0,0]};
 const reading=suite.update(state);assert.ok(Math.abs(reading.rangefinder.distanceM-.85)<1e-6);assert.ok(Math.abs(reading.opticalFlow.groundDistanceM-.85)<1e-6);
 const restore=suite.checkpoint();suite.setNistLane(false);restore();assert.ok(Math.abs(suite.update({...state,time:1}).rangefinder.distanceM-.85)<1e-6);
 const {createWorld,defaultWorldConfig}=await import('../world.js');
 const config=structuredClone(defaultWorldConfig());config.scene={...config.scene,nistLane:true};config.vehicles[0].spawn={north:3,east:0,altitude:.3,heading:0};config.vehicles[0].home=[3,0];
 assert.throws(()=>createWorld(config),/NIST apparatus/);
 config.scene.nistLane=false;const world=createWorld(config);assert.throws(()=>world.setScene({...config.scene,nistLane:true}),/NIST apparatus/);
});

test('NIST world advances and restores checkpoints with shared CPU geometry',async()=>{
 const {createWorld,defaultWorldConfig}=await import('../world.js');
 const config=structuredClone(defaultWorldConfig());config.scene.nistLane=true;
 const world=createWorld(config);world.step();const restore=world.checkpoint();const before=world.snapshot();world.step();restore();assert.deepEqual(world.snapshot(),before);world.step();
});

test('mounting cleats contact bucket back planes and post without entering bucket interiors',()=>{
 const group=createNistLaneGroup(),cleats=group.children.filter(o=>o.name==='NIST mounting cleat');assert.equal(cleats.length,12);
 cleats.forEach((mesh,i)=>{const station=Math.floor(i/4)+1,id=`${station}${['D','C','B','A'][i%4]}`,t=NIST_LANE.targets.find(t=>t.id===id),base=new THREE.Vector3(t.baseNED[1],-t.baseNED[2],-t.baseNED[0]),axis=new THREE.Vector3(t.axisNED[1],-t.axisNED[2],-t.axisNED[0]);const attr=mesh.geometry.getAttribute('position');let maximum=-Infinity,joinsPost=false;for(let j=0;j<attr.count;j++){const p=new THREE.Vector3().fromBufferAttribute(attr,j).applyMatrix4(mesh.matrixWorld);maximum=Math.max(maximum,p.clone().sub(base).dot(axis));if(Math.abs(p.x)<=.050001&&Math.abs(p.z+station*3)<=.050001&&p.y>0&&p.y<.15)joinsPost=true;}assert.ok(Math.abs(maximum+.003)<2e-8,`${id} back contact ${maximum}`);assert.ok(joinsPost,`${id} post connection`);});
});

test('top and side bucket meshes have no triangle crossings',()=>{
 const group=createNistLaneGroup(),meshes=group.children.filter(o=>o.userData.nistTarget?.startsWith('1'));
 const triangles=mesh=>{const attr=mesh.geometry.getAttribute('position'),out=[];for(let i=0;i<attr.count;i+=3){const t=new THREE.Triangle(...[0,1,2].map(j=>new THREE.Vector3().fromBufferAttribute(attr,i+j).applyMatrix4(mesh.matrixWorld)));if(t.getArea()>1e-12)out.push({t,box:new THREE.Box3().setFromPoints([t.a,t.b,t.c])});}return out;};
 const lists=meshes.map(triangles),ray=new THREE.Ray(),hit=new THREE.Vector3();
 for(let a=0;a<lists.length;a++)for(let b=a+1;b<lists.length;b++)for(const x of lists[a])for(const y of lists[b]){if(!x.box.intersectsBox(y.box))continue;for(const [u,v] of [[x.t.a,x.t.b],[x.t.b,x.t.c],[x.t.c,x.t.a],[y.t.a,y.t.b],[y.t.b,y.t.c],[y.t.c,y.t.a]]){const other=u===x.t.a||u===x.t.b||u===x.t.c?y.t:x.t;ray.set(u,v.clone().sub(u).normalize());const p=ray.intersectTriangle(other.a,other.b,other.c,false,hit);assert.ok(!p||p.distanceTo(u)>u.distanceTo(v)+1e-8,`${meshes[a].userData.nistTarget} intersects ${meshes[b].userData.nistTarget}`);}}
});
