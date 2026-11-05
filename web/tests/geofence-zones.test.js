import test from 'node:test';
import assert from 'node:assert/strict';
import { DEFAULT_GEOFENCE_ZONES, validateGeofenceZones, geofenceContains, geofenceSegmentAllowed } from '../geofence-zones.js';
import { createSimulation } from '../sim.js';

const circle = (id, kind, center, radiusM) => ({ id, shape: 'circle', kind, center, radiusM });
const polygon = (id, kind, vertices) => ({ id, shape: 'polygon', kind, vertices });
const rectangle = (id, kind, n0, n1, e0, e1) => polygon(id, kind, [[n0,e0], [n1,e0], [n1,e1], [n0,e1]]);
const enabled = (...zones) => validateGeofenceZones({ enabled: true, zones });
const clone = value => structuredClone(value);

test('geofence settings are strict, bounded and deeply independent, including disabled drafts', () => {
  assert.deepEqual(validateGeofenceZones(), DEFAULT_GEOFENCE_ZONES);
  const input = { enabled: false, zones: [circle('draft', 'inclusion', [500,-500], .1)] };
  const result = validateGeofenceZones(input);
  result.zones[0].center[0] = 0;
  assert.equal(input.zones[0].center[0], 500);
  const valid = circle('circle', 'inclusion', [0,0], 10);
  for (const value of [null, [], {}, { enabled: 1, zones: [] }, { enabled: true, zones: [] },
    { enabled: false, zones: [], extra: true }, { enabled: false, zones: new Array(1) },
    { enabled: false, zones: Array.from({ length: 21 }, (_, i) => ({ ...valid, id: `zone-${i}` })) },
    { enabled: true, zones: [valid, valid] }]) assert.throws(() => validateGeofenceZones(value), /Geofence|geofence/);
  for (const patch of [{ id: '' }, { id: '-bad' }, { id: 'x'.repeat(33) }, { shape: 'square' }, { kind: 'allowed' },
    { center: [NaN,0] }, { center: [0,Infinity] }, { center: [0,501] }, { center: new Array(2) },
    { center: [0,0,0] }, { radiusM: .099 }, { radiusM: 501 }, { radiusM: '1' }, { extra: 1 }])
    assert.throws(() => enabled({ ...valid, ...patch }), /Geofence|geofence/);
  const symbol = clone(valid); symbol[Symbol('hidden')] = true;
  assert.throws(() => enabled(symbol), /geofence/);
  const extended = [0,0]; extended.extra = true;
  assert.throws(() => enabled({ ...valid, center: extended }), /Geofence/);
  assert.equal(enabled(circle('max', 'inclusion', [-500,500], 500)).zones.length, 1);
});

test('simple polygon validation accepts winding and concavity and rejects malformed or intersecting boundaries', () => {
  const vertices = [[0,0],[4,0],[4,4],[2,2],[0,4]];
  for (const points of [vertices, [...vertices].reverse(), [[0,0],[1,0],[2,0],[2,2],[0,2]]])
    assert.equal(enabled(polygon('ok', 'inclusion', points)).zones[0].vertices.length, points.length);
  for (const points of [[], [[0,0],[1,1]], new Array(3), [[0,0],[2,0],[0,0]],
    [[0,0],[1,0],[2,0]], [[0,0],[2,2],[0,2],[2,0]],
    [[0,0],[4,0],[2,0],[2,2],[0,2]], [[0,0],[4,0],[4,4],[2,0],[0,4]],
    [[0,0],[1,0],[1,1],[0,0]], [[0,0],[501,0],[0,1]],
    Array.from({ length: 33 }, (_, i) => [Math.cos(i), Math.sin(i)])])
    assert.throws(() => enabled(polygon('invalid', 'exclusion', points)), /Geofence/);
});

test('inclusions form a union and exclusions win including their circle and polygon boundaries', () => {
  const settings = enabled(circle('a', 'inclusion', [0,0], 5), circle('b', 'inclusion', [8,0], 5),
    circle('hole', 'exclusion', [4,0], .5), rectangle('box', 'exclusion', -2,-1,-1,1));
  for (const point of [[-5,0],[0,0],[8,0],[13,0],[4,1]]) assert.equal(geofenceContains(settings, point), true);
  for (const point of [[-5.001,0],[4,0],[4.5,0],[-2,0],[-1,1],[-1.5,0],[14,0]])
    assert.equal(geofenceContains(settings, point), false);
  const exclusions = enabled(circle('hole', 'exclusion', [10,0], 1));
  assert.equal(geofenceContains(exclusions, [-500,-500]), true);
  assert.equal(geofenceContains(exclusions, [11,0]), false);
  assert.equal(geofenceContains({ ...exclusions, enabled: false }, [10,0]), true);
});

test('polygon vertices and collinear edges follow the same inclusion/exclusion policy', () => {
  const include = enabled(rectangle('box', 'inclusion', -2,2,-2,2));
  for (const point of [[-2,-2],[2,0],[0,0]]) assert.equal(geofenceContains(include, point), true);
  assert.equal(geofenceContains(include, [2.0001,0]), false);
  assert.equal(geofenceSegmentAllowed(include, [-2,-2], [2,-2]), true);
  const exclude = enabled(rectangle('box', 'exclusion', -2,2,-2,2));
  assert.equal(geofenceSegmentAllowed(exclude, [-3,-2], [3,-2]), false);
  assert.equal(geofenceSegmentAllowed(exclude, [-3,-3], [-3,-3]), true);
  assert.equal(geofenceSegmentAllowed(exclude, [0,0], [0,0]), false);
});

test('segment sweeps detect narrow crossings, tangent circles, concave gaps and union seams without sampling', () => {
  const thin = enabled(rectangle('thin', 'exclusion', 1.234567,1.234568,-5,5));
  assert.equal(geofenceSegmentAllowed(thin, [-100,0], [100,0]), false);
  const round = enabled(circle('round', 'exclusion', [0,0], 1));
  assert.equal(geofenceSegmentAllowed(round, [-100,1], [100,1]), false);
  assert.equal(geofenceSegmentAllowed(round, [-100,1.001], [100,1.001]), true);
  const tangent = enabled(circle('round', 'exclusion', [.123,.456], .1));
  assert.equal(geofenceSegmentAllowed(tangent, [34.64917480106864,-196.54134326534978], [-68.63382776648862,394.5023280319833]), false);
  const concave = enabled(polygon('u', 'inclusion', [[0,0],[4,0],[4,4],[3,4],[3,1],[1,1],[1,4],[0,4]]));
  assert.equal(geofenceSegmentAllowed(concave, [.5,3], [3.5,3]), false);
  assert.equal(geofenceSegmentAllowed(concave, [.5,.5], [3.5,.5]), true);
  for (const start of [2,2.000001]) {
    const union = enabled(rectangle('a', 'inclusion', 0,2,-1,1), rectangle('b', 'inclusion', start,4,-1,1));
    assert.equal(geofenceSegmentAllowed(union, [1,0], [3,0]), start === 2);
  }
  const overlap = enabled(circle('a', 'inclusion', [0,0], 2), circle('b', 'inclusion', [3,0], 2));
  assert.equal(geofenceSegmentAllowed(overlap, [0,0], [3,0]), true);
  assert.equal(geofenceSegmentAllowed(overlap, [0,0], [10,0]), false);
});

function fenceAcrossFirstStep(configuration) {
  const baseline = createSimulation(configuration), expected = baseline.step(.02);
  const [north, east] = expected.position, length = Math.hypot(north, east);
  assert.ok(length > 1e-5);
  const along = [north / length, east / length], across = [-along[1], along[0]];
  const vertices = [[.45,-1],[.55,-1],[.55,1],[.45,1]].map(([a,b]) =>
    along.map((v,i) => v * length * a + across[i] * b));
  return { expected, settings: enabled(polygon('narrow', 'exclusion', vertices)) };
}

test('all five airframes sweep the true CG across a narrow exclusion without altering physical motion', () => {
  for (const airframe of ['quadrotor','hexarotor','octorotor','fixedwing','vtol']) {
    const configuration = { airframe, wind: 20 }, { expected, settings } = fenceAcrossFirstStep(configuration);
    assert.equal(geofenceContains(settings, expected.position), true);
    const sim = createSimulation(configuration, { geofenceZones: settings }), next = sim.step(.02);
    assert.deepEqual(next.position, expected.position, airframe);
    assert.deepEqual(next.velocity, expected.velocity, airframe);
    assert.equal(next.navigation.fenceBreached, true, airframe);
    assert.equal(next.navigation.reason, 'geofence', airframe);
    assert.equal(next.navigation.mode, 'rtl', airframe);
    assert.equal(Object.hasOwn(next, 'geofenceZones'), false);
  }
});

test('home, initial spawn, current position and active targets validate atomically with nonzero homes', () => {
  const options = { spawn: { north: 50, east: 20, altitude: 2, heading: 0 }, home: [50,20] };
  const settings = enabled(circle('area', 'inclusion', [50,20], 10));
  const sim = createSimulation({}, options), before = sim.snapshot();
  sim.setGeofenceZones(settings);
  assert.deepEqual(sim.getGeofenceZones(), settings);
  assert.deepEqual(sim.snapshot(), before);
  const input = clone(settings); sim.setGeofenceZones(input); input.zones[0].center[0] = 0;
  const output = sim.getGeofenceZones(); output.zones[0].center[0] = 0;
  assert.deepEqual(sim.getGeofenceZones(), settings);
  for (const invalid of [undefined, { enabled: true, zones: [] }, enabled(circle('bad', 'inclusion', [0,0], 1)),
    enabled(circle('home-hole', 'exclusion', [50,20], .1))]) {
    assert.throws(() => sim.setGeofenceZones(invalid), /Geofence|geofence|Home/);
    assert.deepEqual(sim.getGeofenceZones(), settings);
    assert.deepEqual(sim.snapshot(), before);
  }
  assert.throws(() => createSimulation({}, { ...options, geofenceZones: enabled(circle('bad', 'inclusion', [0,0], 1)) }), /Home/);
  assert.throws(() => createSimulation({}, { home: [50,20], geofenceZones: settings }), /spawn/);
  const offsetSpawn = createSimulation({}, { ...options, spawn: { ...options.spawn, north: 60 } });
  assert.throws(() => offsetSpawn.setGeofenceZones(enabled(circle('area', 'inclusion', [50,20], 5))), /spawn/);
  sim.setMode('position');
  sim.setPositionTarget({ north: 58, east: 20, altitude: 2 });
  assert.throws(() => sim.setGeofenceZones(enabled(circle('area', 'inclusion', [50,20], 5))), /Waypoint/);
  assert.deepEqual(sim.getGeofenceZones(), settings);
});

test('position and mission endpoints obey zones plus the original radius and altitude fence', () => {
  const settings = enabled(circle('area', 'inclusion', [0,0], 50), rectangle('wall', 'exclusion', 4,6,-10,10));
  const sim = createSimulation({ geofenceRadius: 20, geofenceHeight: 10 }, { geofenceZones: settings });
  for (const point of [{ north: 5, east: 0, altitude: 2 }, { north: 21, east: 0, altitude: 2 }, { north: 1, east: 0, altitude: 11 }]) {
    assert.throws(() => sim.setPositionTarget(point), /geofence/);
    assert.throws(() => sim.setMission([{ ...point, hold: 0 }]), /geofence/);
  }
  sim.setMission([{ north: 10, east: 0, altitude: 2, hold: 0 }]);
  sim.setMode('mission');
  assert.equal(sim.snapshot().navigation.waypoints.length, 1);
  assert.throws(() => sim.setGeofenceZones(enabled(circle('tighter', 'inclusion', [0,0], 8))), /Waypoint/);
  const other = createSimulation();
  other.setPositionTarget({ north: 10, east: 0, altitude: 2 });
  assert.throws(() => other.setGeofenceZones(enabled(circle('area', 'inclusion', [0,0], 5))), /Waypoint/);
  other.setMode('attitude');
  other.setMission([{ north: 10, east: 0, altitude: 2, hold: 0 }]);
  assert.throws(() => other.setGeofenceZones(enabled(circle('area', 'inclusion', [0,0], 5))), /Waypoint/);
});

test('zone changes and true breach history survive deterministic rollback; reset restores initial zones', () => {
  const config = { wind: 20, noise: .1, turbulence: 1, seed: 9 };
  const { settings } = fenceAcrossFirstStep(config), sim = createSimulation(config, { geofenceZones: settings });
  const before = sim.snapshot(), rollback = sim.checkpoint();
  const first = sim.step(.02);
  assert.equal(first.navigation.fenceBreached, true);
  sim.setGeofenceZones({ enabled: false, zones: settings.zones });
  rollback();
  assert.deepEqual(sim.snapshot(), before);
  assert.deepEqual(sim.getGeofenceZones(), settings);
  assert.deepEqual(sim.step(.02), first);
  sim.setGeofenceZones({ enabled: false, zones: [] });
  sim.reset();
  assert.deepEqual(sim.getGeofenceZones(), settings);
  assert.deepEqual(sim.snapshot(), before);
  assert.deepEqual(sim.step(.02), first);
});

test('critical battery keeps landing precedence while crossings still latch the fence history', () => {
  const configuration = { wind: 20, batteryInitialSoc: .05 };
  const { settings } = fenceAcrossFirstStep(configuration);
  const sim = createSimulation(configuration, { geofenceZones: settings }), next = sim.step(.02);
  assert.equal(next.navigation.fenceBreached, true);
  assert.equal(next.navigation.mode, 'land');
  assert.equal(next.navigation.reason, 'critical-battery');
  assert.throws(() => sim.setMode('position'), /Safety|Critical/);
});

test('a latched historical breach during pilot landing cannot suppress a later breach in any pilot mode', () => {
  for (const mode of ['position', 'mission', 'attitude']) {
    const configuration = { wind: 20 }, { settings } = fenceAcrossFirstStep(configuration);
    const sim = createSimulation(configuration, { geofenceZones: settings });
    sim.setMode('land');
    const first = sim.step(.02);
    assert.equal(first.navigation.fenceBreached, true);
    assert.equal(first.navigation.reason, '');
    sim.setWind({ speed: 0, direction: 0 });
    sim.setMission([{ north: 0, east: 0, altitude: 2, hold: 0 }]);
    sim.setMode(mode);
    if (mode === 'position') sim.setPositionTarget({ north: 0, east: 0, altitude: 2 });
    if (mode === 'attitude') sim.setTargets({ pitch: .2 });
    let next;
    for (let i = 0; i < 2000; i++) {
      next = sim.step();
      if (next.navigation.reason === 'geofence') break;
    }
    assert.equal(next.navigation.reason, 'geofence', mode);
    assert.equal(next.navigation.mode, 'rtl', mode);
    for (const override of ['position','mission','attitude']) assert.throws(() => sim.setMode(override), /Safety/);
    sim.setGeofenceZones({ enabled: false, zones: [] });
    assert.equal(sim.snapshot().navigation.fenceBreached, true);
    assert.throws(() => sim.setMode('position'), /Safety/);
  }
});

test('enabling zones rejects an excluded current position without mutating either simulation or settings', () => {
  const sim = createSimulation({ wind: 20 });
  for (let i = 0; i < 100; i++) sim.step();
  assert.ok(Math.hypot(...sim.snapshot().position.slice(0,2)) > .1);
  const before = sim.snapshot();
  assert.throws(() => sim.setGeofenceZones(enabled(circle('area', 'inclusion', [0,0], .1))), /Current/);
  assert.deepEqual(sim.snapshot(), before);
  assert.deepEqual(sim.getGeofenceZones(), DEFAULT_GEOFENCE_ZONES);
});

test('wind can carry an aircraft past a boundary and geofence settings never act as a hard wall', () => {
  const sim = createSimulation({ wind: 30 }, { geofenceZones: enabled(circle('small', 'inclusion', [0,0], .1)) });
  let next;
  for (let i = 0; i < 200; i++) { next = sim.step(.005); if (next.navigation.fenceBreached) break; }
  assert.equal(next.navigation.fenceBreached, true);
  assert.ok(Math.hypot(...next.position.slice(0,2)) > .1);
  assert.ok(Math.hypot(...next.velocity.slice(0,2)) > 0);
});

test('native external control rejects enabled zones but retains a disabled draft and rollback', () => {
  const settings = enabled(circle('area', 'inclusion', [0,0], 5));
  const options = { controlSource: 'external' }, configuration = { startOnGround: true };
  assert.throws(() => createSimulation(configuration, { ...options, geofenceZones: settings }), /External/);
  const draft = { ...settings, enabled: false }, sim = createSimulation(configuration, { ...options, geofenceZones: draft });
  assert.deepEqual(sim.getGeofenceZones(), draft);
  assert.throws(() => sim.setGeofenceZones(settings), /External/);
  assert.deepEqual(sim.getGeofenceZones(), draft);
  const rollback = sim.checkpoint(); sim.setGeofenceZones({ enabled: false, zones: [] }); rollback();
  assert.deepEqual(sim.getGeofenceZones(), draft);
});

test('runway takeoff cannot skip the original base geofence and aborts powered roll on breach', () => {
  const sim = createSimulation({ airframe: 'fixedwing', startOnGround: true, geofenceRadius: 1 });
  sim.takeoff();
  let next;
  for (let i = 0; i < 4000; i++) { next = sim.step(); if (next.navigation.fenceBreached) break; }
  assert.equal(next.navigation.fenceBreached, true);
  assert.equal(next.navigation.reason, 'geofence');
  assert.equal(next.navigation.mode, 'rtl');
  assert.equal(next.runway.takeoffPhase, 'aborted');
  assert.equal(next.runway.abortReason, 'safety');
  assert.equal(next.runway.brake, 1);
  const breachPosition = next.position[0];
  for (let i = 0; i < 2000; i++) next = sim.step();
  assert.equal(next.armed, false);
  assert.ok(next.position[0] > breachPosition);
  assert.ok(Math.hypot(...next.velocity) < .08);
});
