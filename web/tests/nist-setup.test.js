import test from 'node:test';
import assert from 'node:assert/strict';
import { defaultWorldConfig } from '../world.js';
import { DEFAULT_GIMBAL_SETTINGS } from '../gimbal.js';
import { createNistTrainingWorld } from '../nist-setup.js';

test('NIST setup is pure, preserves aircraft and sensor configuration, and changes only its documented training defaults',()=>{
  const source=defaultWorldConfig({airframe:'hexarotor',mass:2.3,wind:4,windDirection:80},{sensors:{lidarRays:72,lidarChannels:8,lidarVerticalMinDeg:-40,lidarVerticalMaxDeg:20,radarAzimuthRays:31,radarElevationRays:9,dropout:.12,latency:.11}}).vehicles[0];
  const before=structuredClone(source), world=createNistTrainingWorld(source), vehicle=world.vehicles[0];
  assert.deepEqual(source,before);assert.ok(Object.isFrozen(source));assert.ok(Object.isFrozen(world));
  assert.deepEqual(vehicle.config,{...before.config,startOnGround:true,wind:0,windDirection:0});
  assert.equal(vehicle.id,source.id);assert.equal(vehicle.name,source.name);assert.equal(world.vehicles.length,1);
  assert.deepEqual(vehicle.spawn,{north:0,east:0,altitude:0,heading:0});assert.deepEqual(vehicle.home,[0,0]);
  assert.deepEqual(world.wind,{speed:0,direction:0});assert.equal(world.scene.nistLane,true);assert.deepEqual(world.scene.terrain,{kind:'flat'});
  assert.deepEqual(vehicle.setup.sensors,{...before.setup.sensors,lidarRays:12,lidarChannels:1,lidarVerticalMinDeg:0,lidarVerticalMaxDeg:0,radarAzimuthRays:1,radarElevationRays:1});
  assert.equal(vehicle.setup.mode,'position');assert.equal(vehicle.setup.training,'free');assert.deepEqual(vehicle.setup.mission,[]);assert.deepEqual(vehicle.setup.gimbal,DEFAULT_GIMBAL_SETTINGS);
  const custom=structuredClone(source);custom.id='training-drone';custom.name='自訂載具';
  assert.equal(createNistTrainingWorld(custom).vehicles[0].id,'training-drone');assert.equal(createNistTrainingWorld(custom).vehicles[0].name,'自訂載具');
});
