import test from "node:test";
import assert from "node:assert/strict";
import { createSimulation, AIRFRAME_PRESETS } from "../sim.js";
import { createSensorSuite } from "../sensors.js";
import { createWorld, defaultWorldConfig } from "../world.js";
import { getVehicleEnvelope } from "../peer-geometry.js";
import { obstacleBounds } from "../obstacle-geometry.js";

const box = (north, east, width, depth, baseAltitude, height) => ({
  north,
  east,
  width,
  depth,
  baseAltitude,
  height,
});
const bridge = [
  box(10, -4, 2, 2, 0, 6),
  box(10, 4, 2, 2, 0, 6),
  box(10, 0, 4, 10, 6, 1),
];
const quiet = {
  rangefinderNoiseM: 0,
  lidarNoiseM: 0,
  radarNoiseM: 0,
  radarVelocityNoiseMps: 0,
  lidarChannels: 1,
  lidarVerticalMinDeg: 0,
  lidarVerticalMaxDeg: 0,
  lidarRays: 12,
  radarAzimuthRays: 1,
  radarElevationRays: 1,
};
const truth = (position, quaternion = [1, 0, 0, 0]) => ({
  time: 0,
  position,
  quaternion,
  velocity: [0, 0, 0],
  rates: [0, 0, 0],
});
const near = (a, b, tol = 1e-8) =>
  assert.ok(Math.abs(a - b) < tol, `${a} != ${b}`);

test("a real elevated deck leaves a traversable gap, while its piers and underside are physical solids", () => {
  const sim = createSimulation(),
    reference = createSimulation();
  sim.setObstacles(bridge);
  for (const s of [sim, reference]) {
    s.setMode("position");
    s.setPositionTarget({ north: 20, east: 0, altitude: 2 });
  }
  let crossed = false;
  for (let i = 0; i < 3000; i++) {
    const actual = sim.step(),
      expected = reference.step();
    assert.deepEqual(actual, expected);
    if (actual.position[0] >= 8 && actual.position[0] <= 12) {
      crossed = true;
      assert.ok(-actual.position[2] + getVehicleEnvelope({}).radius < 6);
    }
  }
  assert.ok(crossed);
  assert.ok(sim.snapshot().position[0] > 19);
  const strike = createSimulation();
  strike.setObstacles([box(0, 0, 10, 10, 3, 1)]);
  strike.setTargets({ altitude: 6 });
  for (let i = 0; i < 1000 && strike.snapshot().status !== "crashed"; i++)
    strike.step();
  const impact = strike.snapshot();
  assert.equal(impact.status, "crashed");
  assert.equal(impact.navigation.reason, "obstacle-collision");
  assert.equal(impact.armed, false);
  assert.ok(-impact.position[2] > 2.4 && -impact.position[2] < 3);
  for (let i = 0; i < 100; i++) strike.step();
  assert.deepEqual(strike.snapshot().position, impact.position);
  const pier = createSimulation();
  pier.setObstacles(bridge);
  pier.setMode("position");
  pier.setPositionTarget({ north: 10, east: 4, altitude: 2 });
  for (let i = 0; i < 3000 && pier.snapshot().status !== "crashed"; i++)
    pier.step();
  assert.equal(pier.snapshot().navigation.reason, "obstacle-collision");
});

test("fixed-wing envelope sweeps thin solids continuously without changing unobstructed trim", () => {
  const clear = createSimulation(AIRFRAME_PRESETS.fixedwing),
    reference = createSimulation(AIRFRAME_PRESETS.fixedwing);
  clear.setObstacles([box(10, 0, 2, 20, 35, 1)]);
  for (let i = 0; i < 300; i++)
    assert.deepEqual(clear.step(), reference.step());
  const wing = createSimulation(AIRFRAME_PRESETS.fixedwing);
  const before = wing.snapshot(),
    radius = getVehicleEnvelope(AIRFRAME_PRESETS.fixedwing).radius;
  const stepTravel = before.velocity[0] * 0.02;
  // The old and new centers lie on opposite sides of a .01m sheet in one tick.
  // Its eastern edge is outside the old .25m CG envelope but within the wing envelope.
  const edge = radius - 0.005;
  const sheet = box(stepTravel / 2, edge + 0.005, 0.01, 0.01, 29, 2);
  const bounds = obstacleBounds(sheet);
  const distance = (p) =>
    Math.hypot(
      ...p.map(
        (v, i) => v - Math.max(bounds.low[i], Math.min(bounds.high[i], v)),
      ),
    );
  assert.ok(distance(before.position) > radius);
  wing.setObstacles([sheet]);
  const impact = wing.step(0.02);
  assert.ok(before.position[0] < stepTravel / 2 - 0.005);
  assert.ok(impact.position[0] > stepTravel / 2 + 0.005);
  assert.ok(distance(impact.position) > radius);
  assert.equal(impact.status, "crashed");
  assert.equal(impact.navigation.reason, "obstacle-collision");
  const external = createSimulation(
    { startOnGround: true },
    { controlSource: "external" },
  );
  const original = external.snapshot();
  assert.throws(() => external.setObstacles([]), /external|External/);
  assert.deepEqual(external.snapshot(), original);
});

test("all geometric sensors see elevated undersides and tops, clear gaps, and opaque inside/blind-zone blockers", () => {
  const sensor = (obstacles, state, config = {}) => {
    const s = createSensorSuite({ ...quiet, ...config });
    s.setObstacles(obstacles);
    return s.update(state);
  };
  const deck = box(0, 0, 10, 10, 6, 1);
  const upward = [0, 1, 0, 0];
  let reading = sensor([deck], truth([0, 0, -2], upward));
  near(reading.rangefinder.distanceM, 4);
  const noseUp = [Math.SQRT1_2, 0, Math.SQRT1_2, 0];
  reading = sensor([deck], truth([0, 0, -2], noseUp));
  near(reading.lidar.rangesM[6], 4);
  near(reading.radar.detections[0].rangeM, 4);
  const noseDown = [Math.SQRT1_2, 0, -Math.SQRT1_2, 0];
  reading = sensor([deck], truth([0, 0, -10], noseDown));
  near(reading.lidar.rangesM[6], 3);
  near(reading.radar.detections[0].rangeM, 3);
  reading = sensor([deck], truth([0, 0, -10]));
  near(reading.rangefinder.distanceM, 3);
  reading = sensor(bridge, truth([0, 0, -2]));
  assert.equal(reading.lidar.rangesM[6], null);
  assert.equal(reading.radar.valid, false);
  near(reading.rangefinder.distanceM, 2);
  const front = box(5, 0, 1, 10, 1, 4);
  reading = sensor([front], truth([0, 0, -2]));
  near(reading.lidar.rangesM[6], 4.5);
  near(reading.radar.detections[0].rangeM, 4.5);
  const inside = box(0, 0, 2, 2, 1, 3);
  reading = sensor([inside, front], truth([0, 0, -2]));
  assert.equal(reading.rangefinder.distanceM, null);
  assert.ok(reading.lidar.rangesM.every((r) => r === null));
  assert.equal(reading.radar.reason, "no-return");
  const close = box(0.1, 0, 0.1, 2, 1, 3);
  reading = sensor([close, front], truth([0, 0, -2]));
  assert.equal(reading.lidar.rangesM[6], null);
  assert.equal(reading.radar.reason, "no-return");
});

test("late-vehicle scene intersection is atomic and same-geometry weather edits remain available after a crash", () => {
  const input = structuredClone(defaultWorldConfig({}, { sensors: quiet }));
  const second = structuredClone(input.vehicles[0]);
  second.id = "z-last";
  second.name = "Second";
  second.spawn.east = 12;
  second.home = [0, 12];
  input.vehicles.push(second);
  const world = createWorld(input),
    before = world.snapshot(),
    initial = world.configuration;
  const update = {
    ...structuredClone(initial.scene),
    obstacles: [box(0, 12, 2, 2, 1, 3)],
  };
  assert.throws(() => world.setScene(update), /z-last/);
  assert.deepEqual(world.snapshot(), before);
  assert.deepEqual(world.configuration, initial);
  const invalidReset = structuredClone(input);
  invalidReset.scene.obstacles = update.obstacles;
  assert.throws(() => createWorld(invalidReset), /intersect/);
  const crashed = createWorld(defaultWorldConfig({}, { sensors: quiet }));
  const obstacle = box(2, 0, 1, 3, 0, 5);
  crashed.setScene({
    ...structuredClone(crashed.configuration.scene),
    obstacles: [obstacle],
  });
  crashed.command("drone-1", "mode", "position");
  crashed.command("drone-1", "position", { north: 5, east: 0, altitude: 2 });
  for (
    let i = 0;
    i < 2000 && crashed.getVehicle("drone-1").snapshot.status !== "crashed";
    i++
  )
    crashed.step();
  assert.equal(crashed.getVehicle("drone-1").snapshot.status, "crashed");
  const state = crashed.getVehicle("drone-1").snapshot;
  crashed.setScene({
    ...structuredClone(crashed.configuration.scene),
    environment: { ...crashed.configuration.scene.environment, fog: 0.5 },
  });
  assert.deepEqual(crashed.getVehicle("drone-1").snapshot, state);
});

test("spawn and scene changes reject the conservative swept envelope at corners", () => {
  const input = structuredClone(defaultWorldConfig({}, { sensors: quiet }));
  const world = createWorld(input),
    before = world.snapshot();
  const position = before.vehicles[0].snapshot.position;
  const radius = getVehicleEnvelope(input.vehicles[0].config).radius;
  const corner = box(
    position[0] + 0.8 * radius + 0.5,
    position[1] + 0.8 * radius + 0.5,
    1,
    1,
    -position[2] - 0.5,
    1,
  );
  assert.ok(Math.hypot(0.8 * radius, 0.8 * radius) > radius);
  const changed = {
    ...structuredClone(world.configuration.scene),
    obstacles: [corner],
  };
  assert.throws(() => world.setScene(changed), /intersect/);
  assert.deepEqual(world.snapshot(), before);
  assert.deepEqual(world.configuration.scene, input.scene);
  input.scene.obstacles = [corner];
  assert.throws(() => createWorld(input), /intersect/);
  corner.north = position[0] + radius + 0.51;
  input.scene.obstacles = [corner];
  assert.doesNotThrow(() => createWorld(input));
  assert.doesNotThrow(() =>
    world.setScene({ ...changed, obstacles: [corner] }),
  );
  world.step();
  assert.notEqual(world.snapshot().vehicles[0].snapshot.status, "crashed");
});

test("strict six-field obstacle setters reject missing bases and invalid tops without partial geometry changes", () => {
  const sim = createSimulation(),
    reference = createSimulation();
  const suite = createSensorSuite(quiet),
    original = createSensorSuite(quiet);
  const valid = box(5, 0, 1, 2, 1, 3);
  for (const actor of [sim, reference, suite, original])
    actor.setObstacles([valid]);
  const missing = { ...valid };
  delete missing.baseAltitude;
  const extra = { ...valid, unknown: 1 };
  const invalid = [
    missing,
    extra,
    { ...valid, baseAltitude: Infinity },
    { ...valid, baseAltitude: 1499, height: 2 },
  ];
  for (const candidate of invalid) {
    assert.throws(() => sim.setObstacles([valid, candidate]));
    assert.throws(() => suite.setObstacles([valid, candidate]));
  }
  for (let i = 0; i < 20; i++) assert.deepEqual(sim.step(), reference.step());
  assert.deepEqual(
    suite.update(truth([0, 0, -2])),
    original.update(truth([0, 0, -2])),
  );
});
