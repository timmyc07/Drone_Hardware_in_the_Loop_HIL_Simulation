import test from "node:test";
import assert from "node:assert/strict";
import { createSimulation, DEFAULT_CONFIG, validateConfig } from "../sim.js";

const advance = (sim, seconds, dt = 0.005) => {
  for (let i = 0; i < Math.round(seconds / dt); i++) sim.step(dt);
  return sim.snapshot();
};
const near = (actual, expected, tolerance = 1e-8) =>
  assert.ok(
    Math.abs(actual - expected) < tolerance,
    `${actual} differs from ${expected}`,
  );

test("20 second equilibrium balances gravity and reports specific force", () => {
  const s = advance(createSimulation(), 20);
  near(s.position[2], -2);
  near(Math.hypot(...s.velocity), 0);
  near(s.imu.accel[2], -9.80665);
  near(
    s.motors.reduce((a, b) => a + b * b, 0) * DEFAULT_CONFIG.maxMotorThrust,
    DEFAULT_CONFIG.mass * 9.80665,
  );
  assert.equal(s.metrics.steps, 4000);
  assert.equal(s.status, "flying");
  near(s.metrics.rmse, 0);
  near(s.metrics.peakError, 0);
  near(s.metrics.altitudeRmse, 0);
});

test("positive roll converges with positive East acceleration and altitude hold", () => {
  const sim = createSimulation();
  sim.setTargets({ roll: Math.PI / 18 });
  const s = advance(sim, 6);
  near(s.attitude[0], Math.PI / 18, 0.001);
  assert.ok(s.velocity[1] > 1);
  near(s.attitude[1], 0);
  near(s.position[2], -2, 0.01);
});

test("pitch and yaw body signs follow FRD/NED conventions", () => {
  const sim = createSimulation();
  sim.setTargets({ pitch: 0.1 });
  let s = advance(sim, 4);
  near(s.attitude[1], 0.1, 0.003);
  assert.ok(s.velocity[0] < -1);
  sim.reset();
  sim.setTargets({ yawRate: 0.3 });
  s = advance(sim, 12);
  assert.ok(Math.abs(s.attitude[2]) > 0.5);
  near(s.rates[2], 0.3, 0.01);
});

test("roll scenario applies between 2 and 6 seconds, then settles", () => {
  const sim = createSimulation();
  sim.setScenario("step");
  assert.equal(advance(sim, 1).target.roll, 0);
  const duringStep = advance(sim, 3);
  near(duringStep.target.roll, Math.PI / 18);
  assert.ok(duringStep.metrics.rmse > 0.01);
  assert.ok(duringStep.metrics.peakError > 0.15);
  let s = advance(sim, 6);
  assert.equal(s.target.roll, 0);
  near(s.attitude[0], 0, 0.003);
  sim.reset();
  near(advance(sim, 2).metrics.rmse, 0);
});

test("seeded noisy sensors and packet drops are repeatable; reset repeats trajectory", () => {
  const config = { noise: 1, wind: 2, latency: 0.025, dropout: 0.1, seed: 123 };
  const a = createSimulation(config),
    b = createSimulation(config);
  const result = advance(a, 3);
  assert.deepEqual(result, advance(b, 3));
  assert.ok(result.metrics.dropped > 0);
  a.reset();
  assert.deepEqual(result, advance(a, 3));
  assert.notDeepEqual(
    result.imu,
    advance(createSimulation({ ...config, seed: 124 }), 3).imu,
  );
});

test("latency is visible and complete dropout latches failsafe, spins down, and hits ground", () => {
  const delayed = advance(createSimulation({ latency: 0.1 }), 1);
  near(delayed.metrics.sampleAge, 0.105, 1e-8);
  assert.equal(delayed.status, "flying");
  const sim = createSimulation({ dropout: 1 });
  let s = advance(sim, 1);
  assert.equal(s.status, "failsafe");
  assert.ok(s.motors.every((v) => v < 0.001));
  assert.ok(s.velocity[2] > 1, "gravity causes downward freefall");
  near(s.velocity[2], 9.80665 * 0.45, 0.8);
  s = advance(sim, 2);
  assert.equal(s.position[2], 0);
  assert.equal(s.velocity[2], 0);
  assert.equal(s.metrics.dropped, 600);
  assert.ok(s.pwm.every((v) => v === 1000));
  assert.equal(s.status, "failsafe");
  assert.equal(
    advance(createSimulation({ latency: 0.6 }), 0.7).status,
    "failsafe",
  );
});

test("long disturbed flight preserves a unit quaternion, finite state and bounded motors", () => {
  const sim = createSimulation({ noise: 0.2, wind: 3 });
  sim.setTargets({ roll: 0.15, pitch: -0.12, yawRate: 0.2 });
  sim.gust();
  const s = advance(sim, 60);
  near(Math.hypot(...s.quaternion), 1, 1e-12);
  assert.ok([...s.position, ...s.velocity, ...s.rates].every(Number.isFinite));
  assert.ok(s.motors.every((v) => v >= 0 && v <= 1));
  assert.ok(s.pwm.every((v) => v >= 1000 && v <= 2000));
  assert.ok(s.metrics.peakError < 0.5);
  assert.equal(s.navigation.fenceBreached, true);
});

test("RK4 step refinement agrees for a mixed attitude maneuver", () => {
  const a = createSimulation(),
    b = createSimulation();
  a.setTargets({ roll: 0.1, pitch: -0.1 });
  b.setTargets({ roll: 0.1, pitch: -0.1 });
  const coarse = advance(a, 3, 0.01),
    fine = advance(b, 3, 0.005);
  coarse.position.forEach((v, i) => near(v, fine.position[i], 0.03));
  coarse.attitude.forEach((v, i) => near(v, fine.attitude[i], 0.002));
});

test("configuration, targets, scenarios and integration dt reject invalid input atomically", () => {
  for (const config of [
    { mass: 0 },
    { wind: NaN },
    { latency: Infinity },
    { dropout: 1.1 },
    { seed: 1.2 },
    { secret: 1 },
    { maxMotorThrust: 0.1 },
    null,
  ]) {
    assert.throws(() => validateConfig(config));
  }
  assert.deepEqual(validateConfig(), DEFAULT_CONFIG);
  const sim = createSimulation();
  const initial = sim.snapshot();
  for (const target of [
    { roll: 1 },
    { altitude: -1 },
    { yawRate: NaN },
    { foo: 1 },
    { altitude: 4, pitch: Infinity },
  ])
    assert.throws(() => sim.setTargets(target));
  for (const dt of [0, -1, NaN, Infinity, 0.021])
    assert.throws(() => sim.step(dt));
  assert.throws(() => sim.setScenario("unknown"));
  assert.deepEqual(sim.snapshot(), initial);
});

test("snapshots are independent and structured-clone friendly", () => {
  const sim = createSimulation();
  const snapshot = sim.snapshot();
  assert.deepEqual(structuredClone(snapshot), snapshot);
  snapshot.position[2] = 100;
  snapshot.target.roll = 100;
  snapshot.imu.gyro[0] = 100;
  near(sim.snapshot().position[2], -2);
  near(sim.snapshot().target.roll, 0);
  near(sim.snapshot().imu.gyro[0], 0);
});

test("motor states follow first-order rotor-speed decay rather than thrust decay", () => {
  const sim = createSimulation({ dropout: 1 });
  const before = advance(sim, 0.55);
  const after = sim.step();
  const speedRatio = after.motors[0] / before.motors[0];
  near(speedRatio, Math.exp(-0.005 / DEFAULT_CONFIG.motorTau), 3e-7);
  const thrustRatio = after.motors[0] ** 2 / before.motors[0] ** 2;
  near(thrustRatio, Math.exp((-2 * 0.005) / DEFAULT_CONFIG.motorTau), 5e-7);
});

test("gust scenario repeats for one second every eight seconds; manual gust still applies", () => {
  const sim = createSimulation();
  sim.setScenario("gust");
  assert.deepEqual(sim.snapshot().wind, [4, 3, -1]);
  assert.deepEqual(advance(sim, 0.5).wind, [4, 3, -1]);
  assert.deepEqual(advance(sim, 0.5).wind, [0, 0, 0]);
  assert.deepEqual(advance(sim, 1).wind, [0, 0, 0]);
  assert.deepEqual(advance(sim, 6.5).wind, [4, 3, -1]);
  assert.deepEqual(advance(sim, 1).wind, [0, 0, 0]);
  sim.setScenario("hover");
  sim.gust();
  assert.deepEqual(advance(sim, 0.5).wind, [4, 3, -1]);
  assert.deepEqual(advance(sim, 0.6).wind, [0, 0, 0]);
});

test("divergent integration throws before replacing the last finite physical state", () => {
  const sim = createSimulation({
    mass: 0.1,
    arm: 0.05,
    maxMotorThrust: 200,
    rateKp: 2,
    rateKi: 1,
    attitudeKp: 20,
    motorTau: 0.01,
  });
  sim.setTargets({ roll: 0.5, pitch: -0.5, yawRate: 2 });
  let previous = sim.snapshot(),
    caught = false;
  for (let i = 0; i < 1000; i++) {
    try {
      previous = sim.step(0.02);
    } catch (error) {
      assert.match(error.message, /integration diverged/);
      const after = sim.snapshot();
      for (const key of [
        "position",
        "velocity",
        "quaternion",
        "rates",
        "motors",
        "time",
      ])
        assert.deepEqual(after[key], previous[key]);
      const checkFinite = (value) => {
        if (typeof value === "number") assert.ok(Number.isFinite(value));
        else if (value && typeof value === "object")
          Object.values(value).forEach(checkFinite);
      };
      checkFinite(after);
      caught = true;
      break;
    }
  }
  assert.ok(
    caught,
    "aggressive small-airframe configuration must exercise divergence guard",
  );
});

test("position hold rejects steady wind and follows a position/altitude command", () => {
  const sim = createSimulation({ wind: 4, windDirection: 90, noise: 0.2 });
  sim.setMode("position");
  let s = advance(sim, 20);
  assert.ok(Math.hypot(...s.position.slice(0, 2)) < 0.3);
  sim.setPositionTarget({ north: 6, east: -4, altitude: 3 });
  s = advance(sim, 25);
  assert.ok(Math.hypot(s.position[0] - 6, s.position[1] + 4) < 0.4);
  near(s.position[2], -3, 0.05);
  assert.equal(s.navigation.mode, "position");
  assert.deepEqual(s.navigation.targetPosition, [6, -4]);
});

test("position mode captures current location and rotates control through current yaw", () => {
  const sim = createSimulation();
  sim.setTargets({ yawRate: 0.4 });
  advance(sim, 4);
  sim.setMode("position");
  sim.setPositionTarget({ north: 4, east: 0, altitude: 2 });
  let s = advance(sim, 20);
  near(s.position[0], 4, 0.3);
  near(s.position[1], 0, 0.3);
  const captured = s.position.slice(0, 2);
  sim.setMode("attitude");
  assert.equal(sim.snapshot().target.roll, 0);
  assert.equal(sim.snapshot().target.pitch, 0);
  sim.setMode("position");
  assert.deepEqual(sim.snapshot().navigation.targetPosition, captured);
  sim.setMode("attitude");
  sim.setPositionTarget({ north: 1, east: 2, altitude: 3 });
  sim.setMode("position");
  assert.deepEqual(sim.snapshot().navigation.targetPosition, [1, 2]);
});

test("mission reaches waypoints, respects hold time, and holds final location", () => {
  const sim = createSimulation();
  sim.setMission([
    { north: 0, east: 0, altitude: 2, hold: 1 },
    { north: 3, east: 3, altitude: 3, hold: 1 },
  ]);
  sim.setMode("mission");
  assert.equal(advance(sim, 0.5).navigation.index, 0);
  assert.equal(advance(sim, 1).navigation.index, 1);
  const s = advance(sim, 28.5);
  assert.equal(s.navigation.completed, true);
  assert.equal(s.navigation.index, 1);
  assert.ok(Math.hypot(s.position[0] - 3, s.position[1] - 3) < 0.3);
  near(s.position[2], -3, 0.05);
});

test("RTL climbs first, returns home, then descends smoothly and disarms", () => {
  const sim = createSimulation({ wind: 2 });
  sim.setPositionTarget({ north: 8, east: 5, altitude: 2 });
  sim.setMode("position");
  advance(sim, 20);
  sim.setMode("rtl");
  let s = sim.step();
  assert.ok(s.target.altitude >= 2.9);
  assert.ok(
    Math.hypot(...s.navigation.targetPosition) > 5,
    "climb holds current horizontal location",
  );
  let sawLanding = false;
  for (let i = 0; i < 10000 && s.armed; i++) {
    const before = s;
    s = sim.step();
    if (s.navigation.mode === "land" && !sawLanding) {
      sawLanding = true;
      assert.ok(Math.hypot(...s.position.slice(0, 2)) < 1);
      assert.ok(
        Math.abs(s.target.altitude + before.position[2]) < 0.02,
        "landing starts at current height",
      );
    }
  }
  assert.ok(sawLanding);
  assert.equal(s.status, "landed");
  assert.equal(s.armed, false);
  assert.equal(s.navigation.completed, true);
  assert.ok(Math.hypot(...s.position.slice(0, 2)) < 1);
  assert.ok(s.motors.every((v) => v === 0));
  const location = [...s.position];
  assert.deepEqual(advance(sim, 2).position, location);
});

test("takeoff after disarm/ground starts position hold and returns to two meters", () => {
  const sim = createSimulation();
  sim.disarm();
  assert.equal(advance(sim, 2).status, "landed");
  sim.takeoff();
  const s = advance(sim, 15);
  assert.equal(s.armed, true);
  assert.equal(s.navigation.mode, "position");
  near(s.position[2], -2, 0.02);
});

test("geofence breach latches a return and rejects outside mission points", () => {
  const sim = createSimulation({ geofenceRadius: 3, geofenceHeight: 3 });
  assert.throws(
    () => sim.setMission([{ north: 4, east: 0, altitude: 2, hold: 0 }]),
    /geofence/,
  );
  assert.throws(
    () => sim.setPositionTarget({ north: 0, east: 0, altitude: 4 }),
    /geofence/,
  );
  sim.setTargets({ roll: 0.15 });
  let s = advance(sim, 6);
  assert.equal(s.navigation.fenceBreached, true);
  assert.equal(s.navigation.reason, "geofence");
  assert.throws(() => sim.setMode("attitude"), /Safety/);
  s = advance(sim, 40);
  assert.equal(s.navigation.reason, "geofence");
  assert.equal(s.armed, false);
  assert.ok(Math.hypot(...s.position.slice(0, 2)) < 1);
});

test("battery integrates energy and its voltage/current obey simplified pack sag", () => {
  const sim = createSimulation();
  const initial = sim.snapshot();
  const s = advance(sim, 20);
  assert.ok(s.battery.soc < initial.battery.soc);
  near(s.battery.usedWh, (initial.battery.power * 20) / 3600, 1e-8);
  near(s.battery.power, s.battery.voltage * s.battery.current, 1e-8);
  near(
    s.battery.voltage +
      s.battery.current * DEFAULT_CONFIG.batteryInternalResistance,
    DEFAULT_CONFIG.batteryCells * (3.3 + 0.9 * s.battery.soc),
  );
  assert.ok(s.battery.remainingSeconds > 0);
});

test("low battery returns; critical battery lands without a target jump and prevents rearming", () => {
  const low = createSimulation({ batteryInitialSoc: 0.19 });
  assert.equal(low.step().navigation.reason, "low-battery");
  const sim = createSimulation({ batteryInitialSoc: 0.069 });
  let s = sim.step();
  assert.equal(s.navigation.reason, "critical-battery");
  assert.equal(s.navigation.mode, "land");
  near(s.target.altitude, 1.9975);
  assert.throws(() => sim.setMode("rtl"), /Critical/);
  s = advance(sim, 15);
  assert.equal(s.armed, false);
  assert.equal(s.status, "landed");
  assert.throws(() => sim.arm(), /critical battery/);
  assert.throws(() => sim.takeoff());
});

test("motor efficiency fault changes physical forces; link loss returns and clear does not erase stale failsafe", () => {
  const baseline = createSimulation(),
    sim = createSimulation();
  sim.injectFault("motor", { index: 0, efficiency: 0.5 });
  const a = advance(baseline, 0.1),
    b = advance(sim, 0.1);
  assert.ok(b.position[2] > a.position[2] + 0.001);
  assert.ok(Math.hypot(...b.rates) > 0.1);
  assert.equal(b.faults.motorEfficiency[0], 0.5);
  sim.clearFaults();
  assert.deepEqual(sim.snapshot().faults.motorEfficiency, [1, 1, 1, 1]);
  sim.injectFault("link", { lost: true });
  assert.equal(sim.step().navigation.reason, "link-loss");
  sim.clearFaults();
  sim.setMode("position");
  assert.equal(sim.snapshot().faults.linkLost, false);
  const stale = createSimulation({ dropout: 1 });
  advance(stale, 1);
  stale.clearFaults();
  assert.equal(stale.snapshot().status, "failsafe");
  assert.throws(() => stale.arm(), /failsafe/);
});

test("static box collision crashes and disarms; only reset releases crash latch", () => {
  const sim = createSimulation();
  sim.setObstacles([{ north: 2, east: 0, width: 1, depth: 2, height: 5 }]);
  sim.setPositionTarget({ north: 5, east: 0, altitude: 2 });
  sim.setMode("position");
  const s = advance(sim, 10);
  assert.equal(s.status, "crashed");
  assert.equal(s.navigation.reason, "obstacle-collision");
  assert.equal(s.armed, false);
  assert.ok(s.motors.every((v) => v === 0));
  let frozen = s;
  for (let i = 0; i < 100; i++) frozen = sim.step();
  for (const key of [
    "position",
    "velocity",
    "quaternion",
    "attitude",
    "rates",
    "motors",
  ])
    assert.deepEqual(frozen[key], s[key]);
  assert.ok(frozen.time > s.time);
  assert.ok(frozen.battery.usedWh >= s.battery.usedWh);
  assert.throws(() => sim.arm(), /crash/);
  assert.throws(() => sim.takeoff(), /crash/);
  assert.equal(sim.reset().status, "ready");
  assert.equal(sim.step().status, "flying");
});

test("wind direction and bandlimited turbulence are deterministic and seed dependent", () => {
  const directed = createSimulation({ wind: 3, windDirection: 90 });
  near(directed.snapshot().wind[0], 0);
  near(directed.snapshot().wind[1], 3);
  const config = { turbulence: 2, seed: 7 };
  const a = createSimulation(config),
    b = createSimulation(config);
  const first = a.step().wind;
  assert.ok(
    Math.hypot(...first) < 0.1,
    "turbulence is filtered rather than instant white noise",
  );
  b.step();
  assert.deepEqual(advance(a, 10), advance(b, 10));
  assert.notDeepEqual(
    a.snapshot().wind,
    advance(createSimulation({ ...config, seed: 8 }), 10.005).wind,
  );
  a.reset();
  near(Math.hypot(...a.snapshot().wind), 0);
});

test("new navigation/fault/obstacle/config inputs validate atomically, including sparse arrays", () => {
  for (const value of [
    { batteryCells: 2.5 },
    { batteryInitialSoc: 2 },
    { lowBatteryThreshold: 0.1, criticalBatteryThreshold: 0.2 },
    { turbulence: NaN },
    { windDirection: 400 },
    { geofenceRadius: -1 },
  ])
    assert.throws(() => validateConfig(value));
  const sim = createSimulation(),
    initial = sim.snapshot();
  for (const value of [
    null,
    { north: 1, east: 0 },
    { north: Infinity, east: 0, altitude: 2 },
    { north: 501, east: 0, altitude: 2 },
  ])
    assert.throws(() => sim.setPositionTarget(value));
  for (const value of [
    [],
    Array(2),
    Array(51).fill({ north: 0, east: 0, altitude: 2, hold: 0 }),
    [{ north: 0, east: 0, altitude: 2, hold: -1 }],
  ])
    assert.throws(() => sim.setMission(value));
  for (const value of [
    Array(1),
    [{ north: 0, east: 0, width: 0, depth: 1, height: 1 }],
  ])
    assert.throws(() => sim.setObstacles(value));
  assert.throws(() => sim.setMode("mission"));
  assert.throws(() => sim.setMode("unknown"));
  for (const value of [
    { index: 0.5, efficiency: 0.5 },
    { index: 4, efficiency: 0.5 },
    { index: 0, efficiency: NaN },
  ])
    assert.throws(() => sim.injectFault("motor", value));
  assert.throws(() => sim.injectFault("link", { lost: 1 }));
  assert.deepEqual(sim.snapshot(), initial);
});
