import test from "node:test";
import assert from "node:assert/strict";
import {
  createWorld,
  defaultWorldConfig,
  validateWorldConfig,
} from "../world.js";
import { createSimulation, AIRFRAME_PRESETS } from "../sim.js";
import { DEFAULT_SCENE } from "../scene-data.js";

const lean = {
  lidarRays: 12,
  lidarChannels: 1,
  lidarVerticalMinDeg: 0,
  lidarVerticalMaxDeg: 0,
  gpsNoiseM: 0,
  gpsVerticalNoiseM: 0,
  gpsVelocityNoiseMps: 0,
  lidarNoiseM: 0,
};
function vehicle(id, north = 0, east = 0, config = {}) {
  const value = structuredClone(
    defaultWorldConfig(config, { sensors: lean }).vehicles[0],
  );
  return {
    ...value,
    id,
    name: id,
    spawn: { ...value.spawn, north, east },
    home: [north, east],
  };
}
const worldConfig = (vehicles, scene = DEFAULT_SCENE) => ({
  scene,
  wind: { speed: 0, direction: 0 },
  vehicles,
});
const near = (a, b, tolerance = 0.05) =>
  assert.ok(Math.abs(a - b) < tolerance, `${a} != ${b}`);
function advance(world, seconds) {
  for (let i = 0; i < Math.round(seconds / 0.005); i++) world.step();
  return world.snapshot();
}

test("canonical fleets support all five real airframes, stable ID ordering and isolated commands at one fixed clock", () => {
  const configs = Object.values(AIRFRAME_PRESETS);
  const input = worldConfig(
    configs.map((config, i) => vehicle(`v${i}`, 0, i * 8, config)),
  );
  const world = createWorld(input),
    reordered = createWorld({
      ...input,
      vehicles: [...input.vehicles].reverse(),
    });
  for (let i = 0; i < 100; i++) {
    world.step();
    reordered.step();
  }
  assert.deepEqual(world.snapshot(), reordered.snapshot());
  assert.equal(world.snapshot().vehicles.length, 5);
  assert.ok(
    world
      .snapshot()
      .vehicles.every(
        (v) =>
          v.snapshot.time === world.snapshot().time &&
          v.sensors.time === world.snapshot().time,
      ),
  );
  const previous = world.getVehicle("v1");
  world.command("v0", "targets", { altitude: 3 });
  assert.deepEqual(world.getVehicle("v1"), previous);
  assert.equal(world.getVehicle("v0").snapshot.target.altitude, 3);
  assert.throws(() => world.command("missing", "takeoff"), /ID/);
  assert.throws(() => world.step(0.01), /fixed/);
  assert.ok(
    Object.isFrozen(world.configuration) &&
      Object.isFrozen(world.configuration.vehicles[0].config),
  );
});

test("single-vehicle world reuses the original default plant trajectory exactly", () => {
  const world = createWorld(defaultWorldConfig({}, { sensors: lean })),
    sim = createSimulation();
  world.command("drone-1", "targets", {
    roll: 0.15,
    pitch: -0.1,
    yawRate: 0.2,
  });
  sim.setTargets({ roll: 0.15, pitch: -0.1, yawRate: 0.2 });
  for (let i = 0; i < 1000; i++) {
    const w = world.step().vehicles[0].snapshot,
      s = sim.step();
    for (const key of Object.keys(s)) assert.deepEqual(w[key], s[key], key);
  }
});

test("peer collision crashes both at one committed tick and prevents further world advance", () => {
  const world = createWorld(
    worldConfig([vehicle("a", 0, -1), vehicle("b", 0, 1)]),
  );
  world.command("a", "targets", { roll: 0.2 });
  world.command("b", "targets", { roll: -0.2 });
  let result;
  for (let i = 0; i < 1000; i++) {
    result = world.step();
    if (result.collisions.length) break;
  }
  assert.equal(result.collisions.length, 1);
  assert.deepEqual(result.collisions[0].vehicleIds, ["a", "b"]);
  assert.equal(result.collisions[0].tick, result.tick);
  assert.ok(
    result.collisions[0].fraction >= 0 && result.collisions[0].fraction <= 1,
  );
  assert.ok(
    result.vehicles.every(
      (v) =>
        v.snapshot.status === "crashed" &&
        !v.snapshot.armed &&
        v.snapshot.motors.every((m) => m === 0) &&
        v.snapshot.navigation.reason === "vehicle-collision",
    ),
  );
  const before = world.snapshot();
  assert.throws(() => world.step(), /collision/);
  assert.deepEqual(world.snapshot(), before);
});

test("other vehicles really occlude range and lidar while each sensor excludes its own envelope", () => {
  const a = vehicle("a"),
    b = vehicle("b", 0, 5);
  const world = createWorld(worldConfig([a, b]));
  const scan = world.getVehicle("a").sensors.lidar;
  near(scan.rangesM[9], 5 - 0.38, 1e-8);
  const upper = vehicle("upper"),
    lower = vehicle("lower");
  upper.spawn.altitude = 5;
  lower.spawn.altitude = 2;
  const vertical = createWorld(worldConfig([upper, lower]));
  near(
    vertical.getVehicle("upper").sensors.rangefinder.distanceM,
    3 - 0.38,
    0.05,
  );
  near(
    createWorld(worldConfig([upper])).getVehicle("upper").sensors.rangefinder
      .distanceM,
    5,
    0.05,
  );
});

test("coverage failure rolls every vehicle, gimbal, RNG and sensor history back before clock commitment", () => {
  const terrain = {
    kind: "dem",
    north: -1,
    east: -1,
    cellSize: 1,
    rows: 3,
    cols: 3,
    heights: Array(9).fill(0),
  };
  const geography = {
    origin: {
      latitudeDeg: 25.033,
      longitudeDeg: 121.5654,
      altitudeM: 20,
      verticalDatum: "unspecified",
    },
    source: {
      name: "test.tif",
      sha256: "a".repeat(64),
      crs: "EPSG:4326",
      width: 3,
      height: 3,
      band: 1,
      pixelInterpretation: "point",
      heightUnit: "m",
      scale: 1,
      offset: 0,
      resampling: "bilinear",
      window: [0, 0, 3, 3],
    },
  };
  const a = vehicle("a", -0.5, -0.5, { noise: 0.1, turbulence: 0.1, seed: 4 }),
    b = vehicle("b", 0.5, 0.5, { noise: 0.1, turbulence: 0.1, seed: 7 });
  a.spawn.altitude = 5;
  b.spawn.altitude = 2;
  const world = createWorld(
    worldConfig([a, b], { ...DEFAULT_SCENE, terrain, geography }),
  );
  world.command("a", "gimbal", {
    mode: "body",
    target: { yaw: 1, pitch: 0.2, roll: 0 },
    maxRate: { yaw: 0.2, pitch: 0.2, roll: 0.2 },
  });
  world.command("b", "targets", { roll: 0.3 });
  let caught = false;
  for (let i = 0; i < 2000; i++) {
    const before = world.snapshot();
    try {
      world.step();
    } catch (error) {
      assert.equal(error.code, "TERRAIN_COVERAGE");
      assert.equal(error.vehicleId, "b");
      assert.deepEqual(world.snapshot(), before);
      assert.throws(() => world.step(), { code: "TERRAIN_COVERAGE" });
      assert.deepEqual(world.snapshot(), before);
      caught = true;
      break;
    }
  }
  assert.ok(caught);
});

test("world checkpoint restores post-physics sensor/trainer/gimbal progress and canonical mutations", () => {
  const world = createWorld(worldConfig([vehicle("a"), vehicle("b", 5, 0)]));
  const before = world.snapshot(),
    config = world.configuration,
    rollback = world.checkpoint();
  world.command("a", "gimbal", {
    mode: "body",
    target: { yaw: 1, pitch: 0, roll: 0 },
    maxRate: { yaw: 0.5, pitch: 0.5, roll: 0.5 },
  });
  world.command("a", "sensorFault", { sensor: "gps", failed: true });
  world.setWind({ speed: 1, direction: 90 });
  advance(world, 0.5);
  rollback();
  assert.deepEqual(world.snapshot(), before);
  assert.deepEqual(world.configuration, config);
  assert.throws(rollback, /already/);
  const reference = createWorld(config);
  assert.deepEqual(world.step(), reference.step());
});

test("scene weather preserves progress while invalid fleet changes reject atomically", () => {
  const world = createWorld(worldConfig([vehicle("a"), vehicle("b", 5, 0)]));
  advance(world, 0.5);
  const before = world.snapshot();
  world.setScene({
    ...DEFAULT_SCENE,
    environment: { ...DEFAULT_SCENE.environment, fog: 0.2 },
  });
  assert.deepEqual(world.getVehicle("a").training, before.vehicles[0].training);
  const stable = world.snapshot(),
    config = world.configuration;
  assert.throws(
    () =>
      world.setScene({
        ...DEFAULT_SCENE,
        terrain: {
          kind: "heightfield",
          north: 40,
          east: 40,
          cellSize: 1,
          rows: 3,
          cols: 3,
          heights: Array(9).fill(0),
        },
      }),
    /reset/,
  );
  assert.throws(() => world.setWind({ speed: -1, direction: 0 }));
  assert.deepEqual(world.snapshot(), stable);
  assert.deepEqual(world.configuration, config);
  assert.throws(
    () =>
      validateWorldConfig(
        worldConfig([vehicle("same"), vehicle("same", 5, 0)]),
      ),
    /unique/,
  );
  assert.throws(
    () =>
      validateWorldConfig(worldConfig([vehicle("a"), vehicle("b", 0.1, 0)])),
    /overlap/,
  );
  assert.throws(
    () =>
      validateWorldConfig({
        ...worldConfig([vehicle("a")]),
        wind: { speed: 1, direction: 0 },
      }),
    /wind/,
  );
  assert.throws(() => validateWorldConfig(worldConfig(Array(2))), /fields/);
});

test("independent home missions and RTL share time with genuine fixed-wing flight", () => {
  const a = vehicle("a", 0, 0),
    b = vehicle("b", 0, 10, AIRFRAME_PRESETS.hexarotor),
    c = vehicle("c", 0, 20, AIRFRAME_PRESETS.fixedwing);
  const world = createWorld(worldConfig([a, b, c]));
  world.command("a", "mission", [
    { north: 5, east: 0, altitude: 2, hold: 0.2 },
  ]);
  world.command("a", "mode", "mission");
  world.command("b", "mission", [
    { north: -5, east: 10, altitude: 2, hold: 0.2 },
  ]);
  world.command("b", "mode", "mission");
  advance(world, 12);
  assert.equal(world.getVehicle("a").snapshot.navigation.completed, true);
  assert.equal(world.getVehicle("b").snapshot.navigation.completed, true);
  world.command("a", "mode", "rtl");
  advance(world, 18);
  const landed = world.getVehicle("a").snapshot,
    other = world.getVehicle("b").snapshot,
    wing = world.getVehicle("c").snapshot;
  assert.equal(landed.armed, false);
  assert.equal(
    world.configuration.vehicles[0].setup.mode,
    landed.navigation.mode,
  );
  assert.equal(world.getVehicle("a").setup.mode, landed.navigation.mode);
  assert.equal(landed.navigation.completed, true);
  near(landed.position[0], 0, 0.7);
  near(landed.position[1], 0, 0.7);
  assert.equal(other.armed, true);
  near(other.position[0], -5, 0.3);
  near(other.position[1], 10, 0.3);
  assert.equal(wing.status, "flying");
  assert.ok(wing.position[0] > 350);
  near(-wing.position[2], 30, 0.3);
  assert.ok(
    wing.flight.liftN > wing.propulsion.thrustN,
    "wing support remains aerodynamic",
  );
  assert.equal(world.snapshot().collisions.length, 0);
});

test("a late vehicle rejecting a shared course edit rolls back earlier vehicle scene mutations", () => {
  const scene = {
    ...DEFAULT_SCENE,
    customGates: [{ north: 4, east: 0, altitude: 2, yaw: 0, radius: 1 }],
  };
  const a = vehicle("a"),
    b = vehicle("b", 0, 10);
  b.setup.training = "custom";
  const config = worldConfig([a, b], scene),
    world = createWorld(config),
    reference = createWorld(config);
  const before = world.snapshot(),
    original = world.configuration;
  assert.throws(
    () =>
      world.setScene({
        ...scene,
        customGates: [],
        obstacles: [{ north: 0, east: 1, width: 0.1, depth: 0.1, height: 3 }],
      }),
    /gate/,
  );
  assert.deepEqual(world.snapshot(), before);
  assert.deepEqual(world.configuration, original);
  for (let i = 0; i < 20; i++) assert.deepEqual(world.step(), reference.step());
});

test("radar uses each peer's committed physical velocity, and a radar fault never substitutes controller feedback", () => {
  const a = vehicle("a"),
    b = vehicle("b", 10, 0);
  for (const v of [a, b])
    Object.assign(v.setup.sensors, {
      radarAzimuthRays: 1,
      radarElevationRays: 1,
      radarNoiseM: 0,
      radarVelocityNoiseMps: 0,
    });
  const world = createWorld(worldConfig([a, b]));
  world.command("b", "targets", { pitch: -0.08 });
  const state = advance(world, 0.5),
    source = state.vehicles.find((v) => v.id === "a"),
    target = state.vehicles.find((v) => v.id === "b");
  const reading = source.sensors.radar;
  assert.equal(reading.valid, true);
  assert.equal(reading.detections.length, 1);
  assert.ok(
    target.snapshot.velocity[0] > 0.05,
    "the peer must really move under its own plant",
  );
  near(reading.time, state.time, 1e-8);
  near(
    reading.detections[0].rangeRateMps,
    target.snapshot.velocity[0] - source.snapshot.velocity[0],
    1e-8,
  );
  assert.deepEqual(reading.velocityNedMps, source.snapshot.velocity);
  const physical = world.getVehicle("a").snapshot;
  world.command("a", "sensorFault", { sensor: "radar", failed: true });
  assert.equal(world.getVehicle("a").sensors.radar.reason, "fault");
  assert.deepEqual(world.getVehicle("a").snapshot, physical);
  assert.equal(world.step().vehicles[0].snapshot.armed, true);
});
