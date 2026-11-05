import test from 'node:test';
import assert from 'node:assert/strict';
import { createWorld, defaultWorldConfig } from '../world.js';
import { DEFAULT_SCENE } from '../scene-data.js';

const settings = {enabled:true,zones:[{id:'allowed',shape:'circle',kind:'inclusion',center:[0,0],radiusM:10}]};
const frame = world => world.snapshot().vehicles[0].snapshot;
const input = () => structuredClone(defaultWorldConfig({}, {mode:'position'}, {...structuredClone(DEFAULT_SCENE),obstacles:[]}));

test('world geofence settings are per vehicle, validated atomically and restored with checkpoints', () => {
  const config=input(),peer=structuredClone(config.vehicles[0]);peer.id='peer';peer.home=[20,0];peer.spawn.north=20;config.vehicles.push(peer);
  const world=createWorld(config),before=world.snapshot();
  assert.throws(()=>world.command('drone-1','geofenceZones'));
  assert.deepEqual(world.snapshot(),before);
  assert.throws(()=>world.command('drone-1','geofenceZones',{...settings,zones:[{...settings.zones[0],center:[50,0]}]}));
  assert.deepEqual(world.snapshot(),before);
  world.command('drone-1','geofenceZones',settings);
  assert.deepEqual(world.configuration.vehicles[0].setup.geofenceZones,settings);
  assert.equal(world.configuration.vehicles[1].setup.geofenceZones.enabled,false);
  settings.zones[0].radiusM=9;
  assert.equal(world.configuration.vehicles[0].setup.geofenceZones.zones[0].radiusM,10);
  settings.zones[0].radiusM=10;
  const saved=world.snapshot(),restore=world.checkpoint();
  world.command('drone-1','geofenceZones',{enabled:false,zones:[]});world.step();restore();
  assert.deepEqual(world.snapshot(),saved);
  assert.throws(()=>world.command('drone-1','position',{north:15,east:0,altitude:2}));
  assert.deepEqual(world.snapshot(),saved);
});

test('initial zones reject an excluded spawn and unsafe settings leave existing mission intact', () => {
  const config=input();config.vehicles[0].setup.geofenceZones={enabled:true,zones:[{id:'blocked',shape:'circle',kind:'exclusion',center:[0,0],radiusM:1}]};
  assert.throws(()=>createWorld(config));
  const world=createWorld(input());world.command('drone-1','mission',[{north:15,east:0,altitude:2,hold:0}]);
  const before=world.snapshot();assert.throws(()=>world.command('drone-1','geofenceZones',settings));assert.deepEqual(world.snapshot(),before);
  assert.equal(frame(world).navigation.waypoints.length,1);
});
