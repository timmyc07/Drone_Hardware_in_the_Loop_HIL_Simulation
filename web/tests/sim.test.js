import test from "node:test";
import assert from "node:assert/strict";
import {
  createSimulation,
  DEFAULT_CONFIG,
  validateConfig,
  RUNWAY,
} from "../sim.js";

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
  sim.setObstacles([
    { north: 2, east: 0, width: 1, depth: 2, height: 5, baseAltitude: 0 },
  ]);
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
    [{ north: 0, east: 0, width: 0, depth: 1, height: 1, baseAltitude: 0 }],
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

test("airframe presets validate and reject coerced enum/target object values atomically", () => {
  for (const airframe of ["quadrotor", "fixedwing", "vtol"]) {
    const sim = createSimulation({ airframe });
    assert.equal(sim.snapshot().airframe, airframe);
    assert.equal(sim.snapshot().motors.length, 4);
    assert.equal(sim.snapshot().pwm.length, 4);
  }
  for (const airframe of [new String("fixedwing"), ["fixedwing"], "unknown"])
    assert.throws(() => validateConfig({ airframe }));
  assert.throws(
    () =>
      validateConfig({
        airframe: "fixedwing",
        mass: 20,
        wingArea: 0.1,
        cruiseAirspeed: 8,
      }),
    /trim/,
  );
  const sim = createSimulation(),
    before = sim.snapshot();
  for (const value of [
    new Date(),
    new Map(),
    { airspeed: NaN },
    { airspeed: 41 },
  ])
    assert.throws(() => sim.setTargets(value));
  assert.deepEqual(sim.snapshot(), before);
  assert.throws(() => sim.setTransition("cruise"), /VTOL/);
});

test("fixed wing sustains actual trimmed level flight with wing lift and pusher drag balance", () => {
  const sim = createSimulation({ airframe: "fixedwing", geofenceRadius: 500 });
  const s = advance(sim, 20);
  near(s.flight.airspeedMps, 15, 0.05);
  near(s.position[2], -30, 0.1);
  assert.ok(s.position[0] > 290);
  assert.ok(s.flight.liftN > 11.5 && s.flight.liftN < 12);
  near(s.propulsion.thrustN, s.flight.dragN, 0.05);
  assert.ok(s.motors.every((value) => value === 0));
  assert.equal(s.flight.stalled, false);
  assert.equal(s.flight.transition, "cruise");
});

test("fixed wing ailerons produce a coordinated banked turn rather than translating a quad model", () => {
  const sim = createSimulation({ airframe: "fixedwing" });
  sim.setTargets({ roll: 0.3 });
  const s = advance(sim, 10);
  near(s.attitude[0], 0.3, 0.01);
  near(s.position[2], -30, 0.2);
  assert.ok(s.attitude[2] > 1.5);
  assert.ok(s.position[1] > 70);
  assert.ok(Math.abs(s.flight.betaRad) < 0.03);
  assert.ok(s.flight.liftN > 1.03 * 1.2 * 9.80665);
  near(s.flight.airspeedMps, 15, 0.1);
});

test("low airspeed/high pitch causes real wing stall, lost altitude and ground impact", () => {
  const sim = createSimulation({ airframe: "fixedwing" });
  sim.setTargets({ airspeed: 3, pitch: 0.3 });
  let stalled = false,
    minLift = Infinity;
  for (let i = 0; i < 10 * 200; i++) {
    const s = sim.step();
    if (s.flight.stalled) {
      stalled = true;
      minLift = Math.min(minLift, s.flight.liftN);
    }
  }
  assert.ok(stalled);
  assert.ok(minLift < 1.2 * 9.80665 * 0.8);
  assert.ok(sim.snapshot().position[2] > -20);
  const s = advance(sim, 8);
  assert.equal(s.status, "crashed");
  assert.equal(s.navigation.reason, "ground-impact");
});

test("fixed wing 60m square with one-second holds flies every leg and completes without capture-orbit deadlock", () => {
  const sim = createSimulation({ airframe: "fixedwing" });
  const waypoints = [
    [60, 0],
    [60, 60],
    [0, 60],
    [0, 0],
  ].map(([north, east]) => ({ north, east, altitude: 30, hold: 1 }));
  sim.setMission(waypoints);
  sim.setMode("mission");
  const closest = Array(4).fill(Infinity),
    visited = new Set();
  let done;
  for (let i = 0; i < 90 * 200; i++) {
    const s = sim.step();
    visited.add(s.navigation.index);
    waypoints.forEach(
      (p, index) =>
        (closest[index] = Math.min(
          closest[index],
          Math.hypot(s.position[0] - p.north, s.position[1] - p.east),
        )),
    );
    assert.equal(s.flight.stalled, false);
    if (s.navigation.completed) {
      done = s;
      break;
    }
  }
  assert.ok(
    done,
    "square mission must finish rather than orbit an unreachable capture circle",
  );
  assert.deepEqual([...visited], [0, 1, 2, 3]);
  assert.ok(closest.every((value) => value < 8));
  assert.equal(done.status, "flying");
  assert.ok(done.time < 60);
  near(done.position[2], -30, 0.5);
  assert.ok(
    done.flight.airspeedMps > 8 && done.flight.airspeedMps < 11,
    "short-leg planner selects a physically flyable safe speed",
  );
});

test("fixed wing position and RTL remain wing-borne loiters instead of impossible hover/vertical landing", () => {
  const sim = createSimulation({ airframe: "fixedwing" });
  sim.setPositionTarget({ north: 70, east: 50, altitude: 35 });
  sim.setMode("position");
  let s = advance(sim, 50);
  let radius = Math.hypot(s.position[0] - 70, s.position[1] - 50);
  assert.ok(radius > 40 && radius < 70);
  near(s.position[2], -35, 0.2);
  assert.ok(s.flight.airspeedMps > 14);
  sim.setMode("rtl");
  s = advance(sim, 55);
  radius = Math.hypot(...s.position.slice(0, 2));
  assert.ok(radius > 40 && radius < 70);
  assert.equal(s.navigation.completed, true);
  assert.equal(s.navigation.mode, "rtl");
  assert.equal(s.armed, true);
  assert.equal(s.status, "flying");
});

test("fixed wing flies home runway entry, flare, touchdown and physical ground rollout", () => {
  const sim = createSimulation({ airframe: "fixedwing" });
  sim.setMode("land");
  let touchdown,
    sawFlare = false,
    previous = sim.snapshot(),
    last;
  for (let i = 0; i < 140 * 200; i++) {
    const s = sim.step();
    assert.ok(
      Math.hypot(...s.position.map((v, j) => v - previous.position[j])) < 0.2,
      "no landing teleport",
    );
    if (s.position[2] < 0 && s.position[2] > -2 && s.velocity[2] > 0)
      sawFlare = true;
    if (s.position[2] === 0 && !touchdown) touchdown = s;
    previous = s;
    last = s;
    if (!s.armed) break;
  }
  assert.ok(sawFlare && touchdown);
  assert.ok(
    Math.abs(touchdown.position[0]) < 10 && Math.abs(touchdown.position[1]) < 3,
    "touchdown is near the home runway",
  );
  assert.ok(
    Math.hypot(...touchdown.velocity.slice(0, 2)) > 5,
    "wheels contact while still rolling",
  );
  assert.equal(last.status, "landed");
  assert.equal(last.armed, false);
  assert.equal(last.navigation.completed, true);
  assert.ok(
    last.position[0] > touchdown.position[0] + 10,
    "rollout integrates actual forward motion",
  );
  assert.ok(Math.abs(last.position[0]) < 40);
  const stopped = advance(sim, 2);
  assert.equal(stopped.target.altitude, 0);
  assert.equal(stopped.target.airspeed, 0);
  sim.takeoff();
  assert.equal(sim.snapshot().runway.takeoffPhase, "runup");
});

test("VTOL conversion transfers support from real lift rotors to wing and back, then lands", () => {
  const sim = createSimulation({ airframe: "vtol" });
  const initial = sim.snapshot();
  sim.setTransition("hover");
  assert.deepEqual(sim.snapshot(), initial);
  assert.throws(() => sim.setTransition("cruise"), /8 meters/);
  assert.deepEqual(sim.snapshot(), initial);
  sim.setTargets({ altitude: 12 });
  advance(sim, 12);
  sim.setTransition("cruise");
  let minimum = Infinity,
    s;
  for (let i = 0; i < 10 * 200; i++) {
    s = sim.step();
    minimum = Math.min(minimum, -s.position[2]);
  }
  assert.ok(minimum > 11.5);
  assert.equal(s.flight.transitionPhase, "cruise");
  assert.equal(s.flight.transitionProgress, 1);
  assert.ok(s.flight.airspeedMps > 14);
  assert.ok(s.flight.liftN > 1.2 * 9.80665 * 0.9);
  assert.ok(s.motors.every((v) => v < 0.01));
  sim.setTransition("hover");
  s = advance(sim, 35);
  assert.equal(s.flight.transitionPhase, "hover");
  assert.equal(s.flight.transitionProgress, 0);
  assert.ok(Math.hypot(...s.velocity) < 0.1);
  assert.ok(s.motors.every((v) => v > 0.5));
  near(s.position[2], -12, 0.2);
  sim.setMode("land");
  s = advance(sim, 35);
  assert.equal(s.status, "landed");
  assert.equal(s.armed, false);
  assert.equal(s.target.altitude, 0);
  assert.equal(s.target.airspeed, 0);
});

test("VTOL RTL from cruise decelerates, returns home under rotors, and lands", () => {
  const sim = createSimulation({ airframe: "vtol" });
  sim.setTargets({ altitude: 12 });
  advance(sim, 12);
  sim.setTransition("cruise");
  advance(sim, 10);
  assert.ok(sim.snapshot().position[0] > 100);
  sim.setMode("rtl");
  const s = advance(sim, 115);
  assert.equal(s.status, "landed");
  assert.equal(s.navigation.completed, true);
  assert.equal(s.flight.transitionPhase, "hover");
  assert.ok(Math.hypot(...s.position.slice(0, 2)) < 1);
  assert.equal(s.armed, false);
});

test("pusher efficiency fault changes real propulsion force and fixed-wing flight energy", () => {
  const normal = createSimulation({ airframe: "fixedwing" }),
    faulted = createSimulation({ airframe: "fixedwing" });
  faulted.injectFault("pusher", { efficiency: 0 });
  const a = advance(normal, 5),
    b = advance(faulted, 5);
  near(b.propulsion.thrustN, 0);
  assert.ok(b.flight.airspeedMps < a.flight.airspeedMps - 2);
  assert.equal(b.faults.pusherEfficiency, 0);
  faulted.clearFaults();
  assert.equal(faulted.snapshot().faults.pusherEfficiency, 1);
  assert.throws(
    () => normal.injectFault("motor", { index: 0, efficiency: 0 }),
    /no lift rotors/,
  );
});

test("home runway approach corrects a three meter-per-second crosswind and completes rollout", () => {
  const sim = createSimulation({
    airframe: "fixedwing",
    wind: 3,
    windDirection: 90,
  });
  sim.setMode("land");
  let touchdown, final;
  for (let i = 0; i < 140 * 200; i++) {
    const s = sim.step();
    if (s.position[2] === 0 && !touchdown) touchdown = s;
    final = s;
    if (!s.armed) break;
  }
  assert.ok(touchdown);
  assert.ok(
    Math.abs(touchdown.position[1]) < 3,
    "crosswind final aligns with the Northbound runway",
  );
  assert.ok(Math.abs(touchdown.position[0]) < 15);
  assert.equal(final.status, "landed");
  assert.equal(final.armed, false);
  assert.ok(Math.abs(final.position[1]) < 3);
  assert.ok(Math.hypot(...final.velocity) < 0.01);
});

test("gust then reset exactly matches a fresh simulation for all airframes, including wing trim velocity", () => {
  for (const airframe of ["quadrotor", "fixedwing", "vtol"]) {
    const config = {
      airframe,
      wind: 2,
      windDirection: 35,
      noise: 0.1,
      turbulence: 0.2,
      seed: 123,
    };
    const sim = createSimulation(config);
    advance(sim, 0.1);
    sim.gust();
    assert.notDeepEqual(
      sim.snapshot().wind,
      createSimulation(config).snapshot().wind,
    );
    assert.deepEqual(sim.reset(), createSimulation(config).snapshot());
    const fresh = createSimulation(config);
    assert.deepEqual(sim.step(), fresh.step());
  }
});

test("ground starts require an explicit takeoff authorization even after arm and navigation commands", () => {
  for (const airframe of ["quadrotor", "fixedwing", "vtol"]) {
    const sim = createSimulation({ airframe, startOnGround: true });
    let s = sim.snapshot();
    assert.deepEqual(s.position, [0, 0, 0]);
    assert.deepEqual(s.velocity, [0, 0, 0]);
    assert.equal(s.armed, false);
    assert.equal(s.target.altitude, 0);
    assert.equal(s.runway.takeoffPhase, "idle");
    sim.arm();
    sim.setMode("position");
    sim.setMission([{ north: 0, east: 0, altitude: 0, hold: 0 }]);
    sim.setMode("mission");
    s = advance(sim, 2);
    assert.deepEqual(s.position, [0, 0, 0]);
    assert.equal(s.navigation.completed, false);
    assert.ok(s.motors.every((v) => v === 0));
    assert.equal(s.propulsion.motor, 0);
    sim.takeoff();
    s = advance(sim, 30);
    assert.ok(-s.position[2] > (airframe === "fixedwing" ? 27 : 1.9));
  }
  for (const startOnGround of [1, "true", null, new Boolean(true)])
    assert.throws(() => validateConfig({ startOnGround }));
  const airborne = createSimulation({ airframe: "fixedwing" }),
    before = airborne.snapshot();
  assert.throws(() => airborne.setMode("takeoff"), /on the runway/);
  assert.deepEqual(airborne.snapshot(), before);
});

test("runway takeoff integrates runup, roll, force-supported rotation and climbout without teleporting", () => {
  const sim = createSimulation({ airframe: "fixedwing", startOnGround: true });
  sim.setMode("takeoff");
  let previous = sim.snapshot(),
    liftOff,
    complete;
  const phases = [];
  for (let i = 0; i < 40 * 200; i++) {
    const s = sim.step();
    if (phases.at(-1) !== s.runway.takeoffPhase)
      phases.push(s.runway.takeoffPhase);
    assert.ok(
      Math.hypot(...s.position.map((v, j) => v - previous.position[j])) < 0.15,
      "position must come from integration",
    );
    assert.ok(s.position[2] <= 1e-9, "wheels never penetrate ground");
    if (s.runway.takeoffPhase === "runup" || s.runway.takeoffPhase === "roll")
      near(s.position[2], 0, 1e-6);
    if (s.runway.takeoffPhase === "rotate")
      assert.ok(s.flight.airspeedMps >= s.runway.rotationSpeedMps);
    if (!s.runway.weightOnWheels && !liftOff) {
      liftOff = s;
      assert.ok(s.position[0] > 1 && s.position[0] < RUNWAY.endNorth);
      assert.ok(
        s.flight.liftN + s.propulsion.thrustN * Math.sin(s.attitude[1]) >
          1.2 * 9.80665,
      );
    }
    if (s.runway.takeoffPhase === "complete") {
      complete = s;
      break;
    }
    previous = s;
  }
  assert.deepEqual(phases, ["runup", "roll", "rotate", "climb", "complete"]);
  assert.ok(liftOff && complete);
  assert.equal(complete.navigation.mode, "position");
  assert.ok(-complete.position[2] >= 28);
  assert.equal(complete.runway.abortReason, "");
  const settled = advance(sim, 15);
  near(settled.position[2], -30, 0.5);
  assert.equal(settled.status, "flying");
});

test("runway takeoff maintains the centerline in headwind, tailwind and three meter-per-second crosswind", () => {
  for (const windDirection of [0, 90, 180]) {
    const sim = createSimulation({
      airframe: "fixedwing",
      startOnGround: true,
      wind: 3,
      windDirection,
    });
    sim.takeoff();
    let done,
      maxCross = 0;
    for (let i = 0; i < 35 * 200; i++) {
      const s = sim.step();
      if (s.runway.weightOnWheels) {
        maxCross = Math.max(maxCross, Math.abs(s.position[1]));
        assert.equal(s.runway.onRunway, true);
      }
      assert.notEqual(s.runway.takeoffPhase, "aborted");
      assert.notEqual(s.status, "crashed");
      if (s.runway.takeoffPhase === "complete") {
        done = s;
        break;
      }
    }
    assert.ok(done);
    assert.ok(maxCross < 2);
    assert.ok(done.flight.airspeedMps > 13);
    assert.ok(-done.position[2] >= 28);
  }
});

test("pilot abort and disarm brake a moving aircraft over time, preserve phase data and allow a stopped retry", () => {
  for (const command of ["abortTakeoff", "disarm"]) {
    const sim = createSimulation({
      airframe: "fixedwing",
      startOnGround: true,
    });
    sim.takeoff();
    const moving = advance(sim, 1.5);
    assert.ok(moving.velocity[0] > 4);
    sim[command]();
    assert.deepEqual(
      sim.snapshot().velocity,
      moving.velocity,
      "abort cannot clear velocity",
    );
    const braking = sim.step();
    assert.ok(braking.velocity[0] > 0);
    assert.ok(braking.position[0] > moving.position[0]);
    let slowed = braking;
    for (let i = 0; i < 20; i++) {
      slowed = sim.step();
      assert.equal(slowed.target.altitude, 0);
      assert.equal(slowed.target.airspeed, 0);
      assert.equal(slowed.runway.takeoffPhase, "aborted");
      assert.equal(slowed.runway.brake, 1);
      assert.equal(slowed.propulsion.command, 0);
    }
    assert.ok(
      slowed.velocity[0] > 0 && slowed.velocity[0] < moving.velocity[0],
    );
    const stopped = advance(sim, 3);
    assert.equal(stopped.target.altitude, 0);
    assert.equal(stopped.target.airspeed, 0);
    assert.equal(stopped.runway.takeoffPhase, "aborted");
    assert.equal(stopped.runway.abortReason, "pilot");
    assert.equal(stopped.armed, false);
    assert.ok(Math.hypot(...stopped.velocity) < 0.01);
    assert.ok(stopped.position[0] > moving.position[0] + 1);
    assert.equal(stopped.runway.onRunway, true);
    sim.clearFaults();
    sim.takeoff();
    assert.equal(advance(sim, 25).runway.takeoffPhase, "complete");
  }
});

test("pusher failure aborts roll and inadequate thrust never produces timer-based liftoff", () => {
  const sim = createSimulation({ airframe: "fixedwing", startOnGround: true });
  sim.takeoff();
  advance(sim, 1.5);
  sim.injectFault("pusher", { efficiency: 0 });
  let s = sim.step();
  assert.equal(s.runway.takeoffPhase, "aborted");
  assert.equal(s.runway.abortReason, "pusher-failure");
  assert.ok(s.velocity[0] > 0);
  s = advance(sim, 4);
  assert.equal(s.armed, false);
  assert.ok(Math.hypot(...s.velocity) < 0.01);
  near(s.position[2], 0);
  sim.clearFaults();
  sim.takeoff();
  assert.equal(advance(sim, 25).runway.takeoffPhase, "complete");
  const weak = createSimulation({ airframe: "fixedwing", startOnGround: true });
  weak.injectFault("pusher", { efficiency: 0.02 });
  weak.takeoff();
  for (let i = 0; i < 15 * 200; i++) {
    s = weak.step();
    near(s.position[2], 0, 1e-7);
    assert.equal(s.runway.weightOnWheels, true);
  }
  assert.equal(s.runway.takeoffPhase, "aborted");
  assert.equal(s.runway.abortReason, "insufficient-airspeed");
  assert.equal(s.armed, false);
});

test("large tailwind cannot masquerade as forward rotation airspeed; remaining-runway braking stops before the end", () => {
  const sim = createSimulation({
    airframe: "fixedwing",
    startOnGround: true,
    wind: 20,
  });
  sim.takeoff();
  let aborted, stopped;
  for (let i = 0; i < 30 * 200; i++) {
    const s = sim.step();
    near(s.position[2], 0, 1e-7);
    assert.equal(s.runway.weightOnWheels, true);
    assert.ok(
      s.position[0] < RUNWAY.endNorth,
      "abort must leave enough runway to stop",
    );
    assert.ok(!["rotate", "climb", "complete"].includes(s.runway.takeoffPhase));
    if (s.runway.takeoffPhase === "aborted" && !aborted) aborted = s;
    if (aborted && Math.hypot(...s.velocity) < 0.01) {
      stopped = s;
      break;
    }
  }
  assert.ok(aborted && stopped);
  assert.equal(aborted.runway.abortReason, "runway-end");
  assert.ok(aborted.velocity[0] > 20);
  assert.ok(stopped.position[0] > aborted.position[0] + 30);
  assert.equal(stopped.armed, false);
  assert.equal(stopped.runway.onRunway, true);
});

test("runway heading-control loss aborts rather than rotating in an uncontrolled crosswind", () => {
  const sim = createSimulation({
    airframe: "fixedwing",
    startOnGround: true,
    wind: 15,
    windDirection: 90,
  });
  sim.takeoff();
  let stopped;
  for (let i = 0; i < 4 * 200; i++) {
    const s = sim.step();
    near(s.position[2], 0, 1e-6);
    if (s.runway.takeoffPhase === "aborted") {
      stopped = s;
      break;
    }
  }
  assert.ok(stopped);
  assert.equal(stopped.runway.abortReason, "runway-excursion");
  assert.equal(stopped.armed, false);
  assert.equal(stopped.propulsion.command, 0);
});

test("active runway phases reject competing command mutations atomically and explicit airborne abort chooses a safe flight mode", () => {
  const sim = createSimulation({ airframe: "fixedwing", startOnGround: true });
  sim.takeoff();
  let before = sim.snapshot();
  for (const action of [
    () => sim.setTargets({ airspeed: 0 }),
    () => sim.setMode("position"),
    () => sim.setPositionTarget({ north: 5, east: 0, altitude: 30 }),
    () => sim.setMission([{ north: 20, east: 0, altitude: 30, hold: 0 }]),
    () => sim.takeoff(),
  ]) {
    assert.throws(action);
    assert.deepEqual(sim.snapshot(), before);
  }
  advance(sim, 4);
  assert.equal(sim.snapshot().runway.takeoffPhase, "climb");
  sim.abortTakeoff();
  assert.equal(sim.snapshot().runway.takeoffPhase, "aborted");
  assert.equal(sim.snapshot().navigation.mode, "position");
  assert.equal(sim.snapshot().armed, true);
  assert.equal(advance(sim, 20).status, "flying");
});

test("runway takeoff liftoff and climb agree when the RK4 integration step is refined", () => {
  const a = createSimulation({ airframe: "fixedwing", startOnGround: true }),
    b = createSimulation({ airframe: "fixedwing", startOnGround: true });
  a.takeoff();
  b.takeoff();
  const coarse = advance(a, 12, 0.005),
    fine = advance(b, 12, 0.0025);
  near(coarse.position[0], fine.position[0], 0.3);
  near(coarse.position[2], fine.position[2], 0.1);
  near(coarse.flight.airspeedMps, fine.flight.airspeedMps, 0.1);
  assert.equal(coarse.runway.takeoffPhase, "climb");
  assert.equal(fine.runway.takeoffPhase, "climb");
});

test("full fixed-wing runway cycle flies mission, returns, lands, stops and takes off again with remaining runway", () => {
  const sim = createSimulation({ airframe: "fixedwing", startOnGround: true });
  sim.takeoff();
  advance(sim, 28);
  assert.equal(sim.snapshot().runway.takeoffPhase, "complete");
  sim.setMission(
    [
      [60, 0],
      [60, 60],
      [0, 60],
      [0, 0],
    ].map(([north, east]) => ({ north, east, altitude: 30, hold: 1 })),
  );
  sim.setMode("mission");
  let missionDone = false;
  for (let i = 0; i < 160 * 200; i++) {
    const s = sim.step();
    assert.equal(s.status, "flying");
    if (s.navigation.completed) {
      missionDone = true;
      break;
    }
  }
  assert.ok(missionDone);
  sim.setMode("rtl");
  let s = advance(sim, 60);
  assert.equal(s.navigation.completed, true);
  assert.equal(s.navigation.mode, "rtl");
  sim.setMode("land");
  for (let i = 0; i < 150 * 200; i++) {
    s = sim.step();
    if (!s.armed) break;
  }
  assert.equal(s.status, "landed");
  assert.equal(s.armed, false);
  assert.equal(s.runway.onRunway, true);
  assert.ok(s.runway.distanceRemainingM > 25);
  const departure = s.position[0];
  sim.takeoff();
  s = advance(sim, 25);
  assert.equal(s.runway.takeoffPhase, "complete");
  assert.ok(s.position[0] > departure + 100);
  assert.ok(-s.position[2] > 27);
});

test("pusher failure just after liftoff glides onto the remaining runway and brakes to a stop", () => {
  const sim = createSimulation({ airframe: "fixedwing", startOnGround: true });
  sim.takeoff();
  advance(sim, 4);
  const airborne = sim.snapshot();
  assert.equal(airborne.runway.takeoffPhase, "climb");
  assert.ok(airborne.position[2] < -1);
  sim.injectFault("pusher", { efficiency: 0 });
  let previous = airborne,
    stopped;
  for (let i = 0; i < 25 * 200; i++) {
    const s = sim.step();
    assert.ok(
      Math.hypot(...s.position.map((v, j) => v - previous.position[j])) < 0.15,
    );
    assert.equal(s.runway.takeoffPhase, "aborted");
    assert.equal(s.runway.abortReason, "pusher-failure");
    assert.equal(s.propulsion.thrustN, 0);
    assert.notEqual(s.status, "crashed");
    previous = s;
    if (!s.armed) {
      stopped = s;
      break;
    }
  }
  assert.ok(stopped);
  assert.equal(stopped.status, "landed");
  assert.equal(stopped.runway.onRunway, true);
  assert.ok(stopped.position[0] > airborne.position[0] + 20);
  assert.ok(stopped.position[0] < RUNWAY.endNorth);
  assert.ok(Math.hypot(...stopped.velocity) < 0.01);
});

test("external plant validates commands atomically and consumes each tick once", () => {
  const sim = createSimulation(
    { startOnGround: true },
    { controlSource: "external" },
  );
  const initial = sim.snapshot();
  for (const invoke of [
    () => sim.step(),
    () => sim.arm(),
    () => sim.setTargets({ altitude: 2 }),
    () => sim.setMode("position"),
    () => sim.takeoff(),
  ])
    assert.throws(invoke);
  const input = {
    sequence: 1,
    timeUs: 0,
    armed: true,
    rotorSpeedCommands: [0, 0, 0, 0],
  };
  for (const bad of [
    { ...input, timeUs: 1 },
    { ...input, sequence: 0 },
    { ...input, armed: 1 },
    { ...input, rotorSpeedCommands: [0, NaN, 0, 0] },
    { ...input, rotorSpeedCommands: Array(4) },
  ]) {
    assert.throws(() => sim.applyExternalActuators(bad));
    assert.deepEqual(sim.snapshot(), initial);
  }
  sim.applyExternalActuators(input);
  assert.throws(() => sim.applyExternalActuators({ ...input, sequence: 2 }));
  sim.stepExternal();
  assert.equal(sim.readSensors().timeUs, 4000);
  assert.throws(() => sim.stepExternal());
  assert.throws(() => sim.applyExternalActuators({ ...input, timeUs: 4000 }));
  sim.applyExternalActuators({ ...input, sequence: 2, timeUs: 4000 });
  sim.stepExternal();
  near(sim.snapshot().time, 0.008);
  assert.deepEqual(sim.snapshot().target, initial.target);
  sim.reset();
  assert.deepEqual(sim.snapshot(), initial);
});

function externalAdvance(sim, seconds, speeds, dt = 0.004) {
  for (let i = 0; i < Math.round(seconds / dt); i++) {
    const sequence = (sim.snapshot().externalInput?.sequence ?? 0) + 1;
    sim.applyExternalActuators({
      sequence,
      timeUs: sim.readSensors().timeUs,
      armed: true,
      rotorSpeedCommands: speeds,
    });
    sim.stepExternal(dt);
  }
  return sim.snapshot();
}

test("external sensors report current supported specific force and collective physically lifts off", () => {
  const sim = createSimulation(
    { startOnGround: true },
    { controlSource: "external" },
  );
  assert.deepEqual(sim.readSensors().specificForceFrd, [0, 0, -9.80665]);
  externalAdvance(sim, 0.2, [0, 0, 0, 0]);
  assert.deepEqual(sim.readSensors().specificForceFrd, [0, 0, -9.80665]);
  const state = externalAdvance(sim, 1, [0.7, 0.7, 0.7, 0.7]);
  assert.ok(state.position[2] < -0.5);
  assert.ok(state.velocity[2] < -1);
  assert.deepEqual(sim.readSensors().positionNed, state.position);
  assert.equal(sim.readSensors().timeUs, 1200000);
  const sensors = sim.readSensors();
  sensors.positionNed[0] = 999;
  assert.equal(sim.readSensors().positionNed[0], 0);
});

test("external individual rotor torques match FR RR RL FL signs", () => {
  const signs = [
    [-1, 1, 1],
    [-1, -1, -1],
    [1, -1, 1],
    [1, 1, -1],
  ];
  for (let rotor = 0; rotor < 4; rotor++) {
    const sim = createSimulation(
      { startOnGround: true },
      { controlSource: "external" },
    );
    const speeds = [0, 0, 0, 0];
    speeds[rotor] = 0.6;
    const state = externalAdvance(sim, 0.04, speeds);
    state.rates.forEach((value, axis) =>
      assert.equal(Math.sign(value), signs[rotor][axis]),
    );
  }
});

test("external RK4 collective flight converges across integration steps without internal guidance", () => {
  const run = (dt) =>
    externalAdvance(
      createSimulation({ startOnGround: true }, { controlSource: "external" }),
      1,
      [0.65, 0.65, 0.65, 0.65],
      dt,
    );
  const a = run(0.004),
    b = run(0.002);
  near(a.position[2], b.position[2], 0.002);
  near(a.velocity[2], b.velocity[2], 0.002);
  assert.throws(() => createSimulation({}, { controlSource: "external" }));
  assert.throws(() =>
    createSimulation(
      { airframe: "fixedwing", startOnGround: true },
      { controlSource: "external" },
    ),
  );
});

test("external plant retains physical motor faults and empty-battery inhibition", () => {
  const options = { controlSource: "external" };
  const healthy = createSimulation({ startOnGround: true }, options);
  const damaged = createSimulation({ startOnGround: true }, options);
  damaged.injectFault("motor", { index: 0, efficiency: 0 });
  const a = externalAdvance(healthy, 0.1, [0.7, 0.7, 0.7, 0.7]);
  const b = externalAdvance(damaged, 0.1, [0.7, 0.7, 0.7, 0.7]);
  assert.ok(Math.hypot(...b.rates) > Math.hypot(...a.rates) + 0.1);
  const empty = createSimulation(
    { startOnGround: true, batteryInitialSoc: 0 },
    options,
  );
  const c = externalAdvance(empty, 0.1, [1, 1, 1, 1]);
  assert.equal(c.armed, false);
  assert.deepEqual(c.motors, [0, 0, 0, 0]);
  assert.deepEqual(c.position, [0, 0, 0]);
});

test("gimbal setup/reset is deterministic across airframes and runtime demand advances only on physical ticks", () => {
  const initial = {
    mode: "ned",
    target: { yaw: 0.2, pitch: 0.3, roll: 0.1 },
    maxRate: { yaw: 1, pitch: 1, roll: 1 },
  };
  for (const airframe of ["quadrotor", "fixedwing", "vtol"]) {
    const sim = createSimulation({ airframe }, { gimbal: initial }),
      first = sim.snapshot();
    assert.ok(first.gimbal.trackingErrorRad < 1e-8);
    const before = sim.snapshot().gimbal.actual;
    sim.setGimbal({ ...initial, target: { yaw: 0.8, pitch: 0.1, roll: 0 } });
    assert.deepEqual(sim.snapshot().gimbal.actual, before);
    sim.step(0.005);
    const after = sim.snapshot().gimbal.actual;
    for (const axis of ["yaw", "pitch", "roll"])
      assert.ok(Math.abs(after[axis] - before[axis]) <= 0.005 + 1e-12);
    sim.reset();
    assert.deepEqual(sim.snapshot(), first);
  }
});

test("gimbal motion never changes physics or controller noise and external SITL keeps its fixed default mount", () => {
  const config = { noise: 0.2, wind: 1, seed: 51 },
    a = createSimulation(config),
    b = createSimulation(config);
  b.setGimbal({
    mode: "ned",
    target: { yaw: 1, pitch: 0.5, roll: 0.2 },
    maxRate: { yaw: 1, pitch: 1, roll: 1 },
  });
  a.setTargets({ roll: 0.12 });
  b.setTargets({ roll: 0.12 });
  for (let i = 0; i < 1000; i++) {
    a.step();
    b.step();
  }
  const { gimbal: ga, ...pa } = a.snapshot(),
    { gimbal: gb, ...pb } = b.snapshot();
  assert.deepEqual(pa, pb);
  assert.notDeepEqual(ga.actual, gb.actual);
  const external = createSimulation(
      { startOnGround: true },
      { controlSource: "external" },
    ),
    before = external.snapshot().gimbal;
  assert.throws(
    () =>
      external.setGimbal({
        mode: "body",
        target: { yaw: 1, pitch: 0, roll: 0 },
        maxRate: { yaw: 1, pitch: 1, roll: 1 },
      }),
    /disabled/,
  );
  assert.throws(
    () =>
      createSimulation(
        { startOnGround: true },
        { controlSource: "external", gimbal: {} },
      ),
    /fixed camera mount/,
  );
  externalAdvance(external, 0.1, [0.7, 0.7, 0.7, 0.7]);
  assert.deepEqual(external.snapshot().gimbal.actual, before.actual);
});

test("six and eight rotor presets hover with all real channels and track roll, pitch and yaw with correct signs", () => {
  for (const airframe of ["hexarotor", "octorotor"]) {
    const sim = createSimulation({ airframe }),
      count = airframe === "hexarotor" ? 6 : 8;
    let s = advance(sim, 20);
    near(s.position[2], -2, 1e-8);
    assert.equal(s.motors.length, count);
    assert.equal(s.pwm.length, count);
    assert.equal(s.faults.motorEfficiency.length, count);
    assert.equal(s.flight.liftN, 0);
    assert.equal(s.propulsion.thrustN, 0);
    near(
      s.battery.power,
      DEFAULT_CONFIG.batteryBasePower +
        DEFAULT_CONFIG.batteryMotorPower *
          s.motors.reduce((sum, speed) => sum + speed ** 3, 0),
      1e-8,
    );
    sim.setTargets({ roll: 0.15, pitch: -0.1, yawRate: 0.1 });
    s = advance(sim, 12);
    near(s.attitude[0], 0.15, 0.015);
    near(s.attitude[1], -0.1, 0.015);
    near(s.rates[2], 0.1, 0.025);
    assert.ok(s.position[0] > 1);
    assert.ok(s.position[1] > 1);
    assert.ok(s.attitude[2] > 0.5);
  }
});

test("six and eight rotor position navigation completes a mission then returns and lands with real motors", () => {
  for (const airframe of ["hexarotor", "octorotor"]) {
    const sim = createSimulation({ airframe, wind: 2, geofenceRadius: 300 });
    sim.setMode("position");
    let s = advance(sim, 15);
    assert.ok(Math.hypot(s.position[0], s.position[1]) < 1);
    near(s.position[2], -2, 0.1);
    sim.setMission([
      { north: 4, east: 3, altitude: 3, hold: 1 },
      { north: -2, east: 4, altitude: 3, hold: 0.5 },
    ]);
    sim.setMode("mission");
    s = advance(sim, 35);
    assert.equal(s.navigation.completed, true);
    assert.ok(Math.hypot(s.position[0] + 2, s.position[1] - 4) < 1);
    sim.setMode("rtl");
    s = advance(sim, 55);
    assert.equal(s.status, "landed");
    assert.equal(s.armed, false);
    assert.ok(Math.hypot(s.position[0], s.position[1]) < 1);
    assert.ok(s.motors.every((v) => v < 1e-8));
  }
});

test("each new rotor fault produces the geometry-predicted physical torque without fault-aware command reallocation", () => {
  for (const airframe of ["hexarotor", "octorotor"]) {
    const count = airframe === "hexarotor" ? 6 : 8,
      offset = airframe === "hexarotor" ? Math.PI / 6 : Math.PI / 8;
    for (let index = 0; index < count; index++) {
      const sim = createSimulation({
        airframe,
        attitudeKp: 0,
        rateKp: 0,
        rateKi: 0,
        altitudeKp: 0,
        altitudeKd: 0,
        drag: 0,
      });
      const before = sim.snapshot();
      sim.injectFault("motor", { index, efficiency: 0 });
      const after = sim.step(0.0001),
        angle = offset + (index * 2 * Math.PI) / count;
      const expected = [
        Math.sin(angle),
        -Math.cos(angle),
        index % 2 === 0 ? -1 : 1,
      ];
      after.rates.forEach((rate, axis) => {
        if (Math.abs(expected[axis]) < 1e-10) assert.ok(Math.abs(rate) < 1e-8);
        else assert.equal(Math.sign(rate), Math.sign(expected[axis]));
      });
      assert.ok(after.velocity[2] > 0);
      assert.deepEqual(after.motors, before.motors);
      assert.equal(after.faults.motorEfficiency[index], 0);
      sim.clearFaults();
      assert.deepEqual(
        sim.snapshot().faults.motorEfficiency,
        Array(count).fill(1),
      );
      const safe = sim.snapshot();
      assert.throws(() =>
        sim.injectFault("motor", { index: count, efficiency: 0 }),
      );
      assert.deepEqual(sim.snapshot(), safe);
    }
  }
});

test("new multirotor inputs enforce count-specific thrust capacity, ground authorization and external quad isolation", () => {
  for (const airframe of ["hexarotor", "octorotor"]) {
    assert.doesNotThrow(() =>
      validateConfig({ airframe, mass: 2, maxMotorThrust: 4 }),
    );
    assert.throws(
      () => validateConfig({ airframe, maxMotorThrust: 0.1 }),
      /weight/,
    );
    assert.throws(
      () =>
        createSimulation(
          { airframe, startOnGround: true },
          { controlSource: "external" },
        ),
      /quadrotor/,
    );
    const sim = createSimulation({ airframe, startOnGround: true });
    sim.arm();
    sim.setMode("position");
    assert.deepEqual(advance(sim, 1).position, [0, 0, 0]);
    sim.takeoff();
    const s = advance(sim, 12);
    near(s.position[2], -2, 0.1);
    assert.equal(s.armed, true);
    assert.throws(() => sim.injectFault("pusher", { efficiency: 0 }), /pusher/);
    assert.throws(() => sim.setTransition("cruise"), /VTOL/);
    sim.reset();
    assert.deepEqual(
      sim.snapshot(),
      createSimulation({ airframe, startOnGround: true }).snapshot(),
    );
    const stale = createSimulation({ airframe, dropout: 1 });
    const stopped = advance(stale, 10);
    assert.equal(stopped.status, "failsafe");
    assert.equal(stopped.position[2], 0);
    assert.ok(stopped.motors.every((v) => v < 1e-8));
    near(Math.hypot(...stopped.quaternion), 1, 1e-12);
  }
});

function terrainPlateau(height) {
  const heights = Array(81).fill(0);
  for (let r = 2; r <= 6; r++)
    for (let c = 2; c <= 6; c++) heights[r * 9 + c] = height;
  return {
    kind: "heightfield",
    north: -40,
    east: 40,
    cellSize: 10,
    rows: 9,
    cols: 9,
    heights,
  };
}

test("multirotors and hovering VTOL follow real terrain AGL, land on raised or negative ground and take off locally", () => {
  for (const airframe of ["quadrotor", "hexarotor", "octorotor", "vtol"])
    for (const height of [3, -5]) {
      const sim = createSimulation(
        { airframe },
        { terrain: terrainPlateau(height) },
      );
      sim.setMode("position");
      sim.setPositionTarget({
        north: 0,
        east: 80,
        altitude: 2,
        altitudeReference: "agl",
      });
      let minimum = Infinity,
        s;
      for (let i = 0; i < 14000; i++) {
        s = sim.step();
        minimum = Math.min(minimum, s.ground.clearanceM);
      }
      assert.equal(s.status, "flying");
      near(s.ground.heightM, height, 1e-8);
      near(s.ground.clearanceM, 2, 0.05);
      near(s.target.altitude, height + 2, 0.05);
      assert.equal(s.navigation.altitudeReference, "agl");
      assert.equal(s.navigation.positionAltitude, 2);
      assert.ok(
        minimum > 1,
        "AGL tracking must avoid ground while crossing the slope",
      );
      sim.setMode("land");
      s = advance(sim, 25);
      assert.equal(s.status, "landed");
      assert.equal(s.armed, false);
      near(s.position[2], -height, 1e-8);
      near(s.ground.clearanceM, 0, 1e-8);
      assert.equal(s.navigation.altitudeReference, "datum");
      near(s.target.altitude, height, 1e-8);
      const imu = sim.readSensors().specificForceFrd;
      near(Math.hypot(...imu), 9.80665, 1e-8);
      sim.takeoff();
      s = advance(sim, 15);
      assert.equal(s.status, "flying");
      near(s.ground.clearanceM, 2, 0.05);
    }
});

test("continuous terrain sweep catches a thin ridge even when both RK4 endpoints are above the mesh and freezes crash state", () => {
  const terrain = {
    kind: "heightfield",
    north: 299.5,
    east: -1,
    cellSize: 1,
    rows: 3,
    cols: 3,
    heights: [0, 0, 0, 0, 31, 0, 0, 0, 0],
  };
  const sim = createSimulation({ airframe: "fixedwing" }, { terrain }),
    free = createSimulation({ airframe: "fixedwing" });
  let before, hit;
  for (let i = 0; i < 1100; i++) {
    before = sim.snapshot();
    free.step(0.02);
    hit = sim.step(0.02);
    if (hit.status === "crashed") break;
  }
  assert.equal(hit.navigation.reason, "terrain-impact");
  assert.equal(hit.armed, false);
  assert.ok(before.ground.clearanceM > 1);
  assert.ok(free.snapshot().position[0] > hit.position[0]);
  const endpoint = free.snapshot();
  const h = 31 * Math.max(0, 1 - Math.abs(endpoint.position[0] - 300.5));
  assert.ok(
    -endpoint.position[2] - h > 1,
    "a height-at-endpoint collision test would miss this ridge",
  );
  const impactPosition = hit.position,
    impactAttitude = hit.attitude;
  advance(sim, 1);
  assert.deepEqual(sim.snapshot().position, impactPosition);
  assert.deepEqual(sim.snapshot().attitude, impactAttitude);
});

test("terrain commands, AGL mode guards and altitude-reference changes are atomic and reset restores constructor ground", () => {
  const fenced = createSimulation({}, { terrain: terrainPlateau(1) });
  fenced.setMode("position");
  fenced.setPositionTarget({
    north: 0,
    east: 80,
    altitude: 98,
    altitudeReference: "agl",
  });
  const oldFenceState = fenced.snapshot();
  assert.throws(() => fenced.setTerrain(terrainPlateau(3)), /geofence/);
  assert.deepEqual(fenced.snapshot(), oldFenceState);
  const initial = terrainPlateau(3),
    sim = createSimulation({}, { terrain: initial });
  sim.setMode("position");
  sim.setPositionTarget({
    north: 0,
    east: 80,
    altitude: 2,
    altitudeReference: "agl",
  });
  advance(sim, 70);
  const safe = sim.snapshot();
  assert.throws(() => sim.setTerrain(terrainPlateau(6)), /intersect/);
  assert.deepEqual(sim.snapshot(), safe);
  const oldPosition = safe.position;
  sim.setTerrain(terrainPlateau(4));
  assert.deepEqual(sim.snapshot().position, oldPosition);
  near(sim.snapshot().ground.heightM, 4, 1e-8);
  const snapshot = sim.snapshot();
  assert.throws(
    () =>
      sim.setPositionTarget({
        north: 0,
        east: 80,
        altitude: 98,
        altitudeReference: "agl",
      }),
    /geofence/,
  );
  assert.deepEqual(sim.snapshot(), snapshot);
  assert.throws(() =>
    sim.setPositionTarget({
      north: 0,
      east: 80,
      altitude: 2,
      altitudeReference: "bad",
    }),
  );
  assert.deepEqual(sim.snapshot(), snapshot);
  sim.setTargets({ altitude: 6 });
  assert.equal(sim.snapshot().navigation.altitudeReference, "datum");
  assert.equal(sim.snapshot().navigation.positionAltitude, 6);
  sim.setMode("attitude");
  assert.throws(
    () =>
      sim.setPositionTarget({
        north: 0,
        east: 80,
        altitude: 2,
        altitudeReference: "agl",
      }),
    /position/,
  );
  sim.reset();
  assert.deepEqual(
    sim.snapshot(),
    createSimulation({}, { terrain: initial }).snapshot(),
  );
  const wing = createSimulation({ airframe: "fixedwing" });
  wing.setMode("position");
  assert.throws(
    () =>
      wing.setPositionTarget({
        north: 0,
        east: 0,
        altitude: 2,
        altitudeReference: "agl",
      }),
    /AGL/,
  );
  const vtol = createSimulation({ airframe: "vtol" });
  vtol.setMode("position");
  vtol.setPositionTarget({
    north: 0,
    east: 0,
    altitude: 12,
    altitudeReference: "agl",
  });
  advance(vtol, 10);
  const hover = vtol.snapshot();
  assert.throws(() => vtol.setTransition("cruise"), /datum/);
  assert.deepEqual(vtol.snapshot(), hover);
  vtol.setTargets({ altitude: 12 });
  assert.doesNotThrow(() => vtol.setTransition("cruise"));
  advance(vtol, 15);
  vtol.setTransition("hover");
  let transitionTail = null;
  for (let i = 0; i < 3000; i++) {
    const current = vtol.step();
    if (
      current.flight.transitionProgress > 0 &&
      current.flight.transitionProgress < 0.01
    ) {
      transitionTail = current;
      break;
    }
  }
  assert.ok(
    transitionTail,
    "real VTOL transition must reach a nonzero tail below one percent",
  );
  assert.throws(
    () =>
      vtol.setPositionTarget({
        north: transitionTail.position[0],
        east: transitionTail.position[1],
        altitude: 2,
        altitudeReference: "agl",
      }),
    /AGL/,
  );
  assert.deepEqual(vtol.snapshot(), transitionTail);
  for (
    let i = 0;
    i < 3000 && vtol.snapshot().flight.transitionProgress !== 0;
    i++
  )
    vtol.step();
  assert.equal(vtol.snapshot().flight.transitionProgress, 0);
  assert.doesNotThrow(() =>
    vtol.setPositionTarget({
      north: vtol.snapshot().position[0],
      east: vtol.snapshot().position[1],
      altitude: 2,
      altitudeReference: "agl",
    }),
  );
  const external = createSimulation(
    { startOnGround: true },
    { controlSource: "external" },
  );
  assert.deepEqual(external.snapshot().ground, {
    heightM: 0,
    normalNed: [0, 0, -1],
    clearanceM: 0,
  });
  assert.throws(() => external.setTerrain({ kind: "flat" }), /disabled/);
  assert.throws(
    () =>
      createSimulation(
        { startOnGround: true },
        { controlSource: "external", terrain: initial },
      ),
    /flat/,
  );
});

test("wing-borne VTOL collides with terrain instead of gaining a fictitious hillside landing", () => {
  const terrain = {
    kind: "heightfield",
    north: 220,
    east: -1,
    cellSize: 1,
    rows: 3,
    cols: 3,
    heights: [0, 0, 0, 0, 100, 0, 0, 0, 0],
  };
  const sim = createSimulation(
    { airframe: "vtol", geofenceRadius: 500 },
    { terrain },
  );
  sim.setTargets({ altitude: 12 });
  advance(sim, 10);
  sim.setTransition("cruise");
  const s = advance(sim, 35);
  assert.equal(s.status, "crashed");
  assert.equal(s.navigation.reason, "terrain-impact");
  assert.equal(s.armed, false);
  assert.ok(s.position[0] < 222);
});

const boundedDem = (halfSize = 10, height = 0) => ({
  kind: "dem",
  north: -halfSize,
  east: -halfSize,
  cellSize: halfSize,
  rows: 3,
  cols: 3,
  heights: Array(9).fill(height),
});

test("DEM coverage rejection preserves the complete last tick and random/controller history", () => {
  const config = {
    noise: 0.2,
    turbulence: 0.3,
    latency: 0.02,
    dropout: 0.1,
    seed: 71,
  };
  const sim = createSimulation(config, { terrain: boundedDem(1) });
  const reference = createSimulation(config, { terrain: boundedDem(50) });
  sim.setTargets({ roll: 0.2 });
  reference.setTargets({ roll: 0.2 });
  let rejected = false;
  for (let i = 0; i < 2000; i++) {
    const before = sim.snapshot();
    try {
      sim.step();
    } catch (error) {
      assert.equal(error.code, "TERRAIN_COVERAGE");
      assert.deepEqual(
        sim.snapshot(),
        before,
        "all published state must remain atomic",
      );
      assert.throws(() => sim.step(), { code: "TERRAIN_COVERAGE" });
      assert.deepEqual(sim.snapshot(), before);
      sim.setTerrain(boundedDem(50));
      assert.deepEqual(
        sim.step(),
        reference.step(),
        "retry must reproduce noise, sensor queues, PID and physics",
      );
      rejected = true;
      break;
    }
    assert.deepEqual(sim.snapshot(), reference.step());
  }
  assert.ok(rejected, "physical roll must reach the surveyed boundary");
});

test("DEM scene replacement rejects missing current or AGL coverage atomically and reset restores the constructor surface", () => {
  const sim = createSimulation(
    { startOnGround: true },
    { terrain: boundedDem(10, 6) },
  );
  assert.equal(sim.snapshot().ground.heightM, 6);
  assert.equal(sim.snapshot().position[2], -6);
  sim.takeoff();
  advance(sim, 10);
  near(sim.snapshot().ground.clearanceM, 2, 0.05);
  sim.setMode("position");
  sim.setPositionTarget({
    north: 5,
    east: 0,
    altitude: 3,
    altitudeReference: "agl",
  });
  const before = sim.snapshot();
  assert.throws(() => sim.setTerrain({ ...boundedDem(10), north: 20 }), {
    code: "TERRAIN_COVERAGE",
  });
  assert.deepEqual(sim.snapshot(), before);
  assert.throws(() => sim.setTerrain(boundedDem(1)), {
    code: "TERRAIN_COVERAGE",
  });
  assert.deepEqual(sim.snapshot(), before);
  sim.reset();
  assert.deepEqual(
    sim.snapshot(),
    createSimulation(
      { startOnGround: true },
      { terrain: boundedDem(10, 6) },
    ).snapshot(),
  );
});

test("DEM slopes retain real wing collision and reject unsupported runway operations without flattening", () => {
  const terrain = {
    kind: "dem",
    north: -10,
    east: -10,
    cellSize: 50,
    rows: 3,
    cols: 3,
    heights: [0, 0, 0, 80, 80, 80, 100, 100, 100],
  };
  assert.throws(
    () =>
      createSimulation(
        { airframe: "fixedwing", startOnGround: true },
        { terrain },
      ),
    /runway/,
  );
  const sim = createSimulation({ airframe: "fixedwing" }, { terrain });
  const before = sim.snapshot();
  for (const mode of ["takeoff", "land"]) {
    assert.throws(() => sim.setMode(mode), /runway/);
    assert.deepEqual(sim.snapshot(), before);
  }
  let state;
  for (let i = 0; i < 2000; i++) {
    state = sim.step();
    if (state.status === "crashed") break;
  }
  assert.equal(state.status, "crashed");
  assert.equal(state.navigation.reason, "terrain-impact");
  assert.equal(state.armed, false);
  assert.ok(
    state.ground.heightM > 20,
    "real relief must remain in the runway area",
  );
  assert.throws(
    () =>
      createSimulation(
        { startOnGround: true },
        { controlSource: "external", terrain: boundedDem() },
      ),
    /flat/,
  );
});

test("covered genuinely flat DEM supports the full physical fixed-wing runway flight cycle", () => {
  const terrain = {
    kind: "dem",
    north: -500,
    east: -500,
    cellSize: 50,
    rows: 21,
    cols: 21,
    heights: Array(21 ** 2).fill(0),
  };
  const sim = createSimulation(
    { airframe: "fixedwing", startOnGround: true },
    { terrain },
  );
  sim.takeoff();
  let last = sim.snapshot(),
    sawGroundRoll = false,
    sawAirborne = false;
  for (let i = 0; i < 40 * 200; i++) {
    const next = sim.step();
    assert.ok(
      Math.hypot(...next.position.map((v, j) => v - last.position[j])) < 0.2,
    );
    if (next.position[0] > 1 && next.ground.clearanceM === 0)
      sawGroundRoll = true;
    if (next.ground.clearanceM > 5) sawAirborne = true;
    last = next;
    if (next.runway.takeoffPhase === "complete") break;
  }
  assert.ok(sawGroundRoll && sawAirborne);
  assert.equal(last.runway.takeoffPhase, "complete");
  sim.setMode("land");
  let touchdown = false;
  for (let i = 0; i < 140 * 200; i++) {
    last = sim.step();
    if (last.runway.weightOnWheels && last.position[0] > -5) touchdown = true;
    if (!last.armed) break;
  }
  assert.ok(touchdown);
  assert.equal(last.status, "landed");
  assert.equal(last.armed, false);
  assert.equal(last.ground.clearanceM, 0);
  assert.equal(last.runway.onRunway, true);
});

function deepValley() {
  const profile = [0, 0, 0, -30, -60, -90, -110, -110, -110, -110, -110];
  return {
    kind: "dem",
    north: -10,
    east: -10,
    cellSize: 10,
    rows: profile.length,
    cols: 3,
    heights: profile.flatMap((height) => [height, height, height]),
  };
}

test("DEM negative datum commands and missions reach a real valley while AGL remains a nonnegative clearance", () => {
  const sim = createSimulation({}, { terrain: deepValley() });
  sim.setMode("position");
  sim.setPositionTarget({ north: 60, east: 0, altitude: 2 });
  advance(sim, 35);
  near(sim.snapshot().position[0], 60, 0.2);
  // Descend in controlled increments; a 110m instantaneous demand is not terrain avoidance.
  for (const altitude of [-8, -28, -48, -68, -88, -108]) {
    sim.setTargets({ altitude });
    advance(sim, 8);
  }
  let s = sim.snapshot();
  near(-s.position[2], -108, 0.05);
  near(s.ground.heightM, -110);
  near(s.ground.clearanceM, 2, 0.05);
  assert.equal(s.navigation.altitudeReference, "datum");
  assert.equal(s.status, "flying");
  const before = s;
  assert.throws(
    () =>
      sim.setPositionTarget({
        north: 60,
        east: 0,
        altitude: -1,
        altitudeReference: "agl",
      }),
    /altitude/,
  );
  assert.deepEqual(sim.snapshot(), before);
  sim.setPositionTarget({
    north: 60,
    east: 0,
    altitude: 2,
    altitudeReference: "agl",
  });
  advance(sim, 5);
  near(sim.snapshot().ground.clearanceM, 2, 0.05);
  sim.setMission([{ north: 70, east: 0, altitude: -108, hold: 0.5 }]);
  sim.setMode("mission");
  s = advance(sim, 18);
  assert.equal(s.navigation.completed, true);
  near(s.position[0], 70, 0.3);
  near(-s.position[2], -108, 0.05);
  assert.equal(s.navigation.altitudeReference, "datum");
});

test("negative datum trust boundaries depend on active DEM without broadening flat or legacy targets", () => {
  for (const terrain of [
    { kind: "flat" },
    {
      kind: "heightfield",
      north: 40,
      east: 40,
      cellSize: 1,
      rows: 3,
      cols: 3,
      heights: Array(9).fill(0),
    },
  ]) {
    const sim = createSimulation({}, { terrain });
    sim.setMode("position");
    const before = sim.snapshot();
    assert.throws(() => sim.setTargets({ altitude: -1 }), /altitude/);
    assert.throws(
      () => sim.setPositionTarget({ north: 0, east: 0, altitude: -1 }),
      /altitude/,
    );
    assert.throws(
      () => sim.setMission([{ north: 0, east: 0, altitude: -1, hold: 0 }]),
      /altitude/,
    );
    assert.deepEqual(sim.snapshot(), before);
  }
  const sim = createSimulation({}, { terrain: deepValley() });
  sim.setMode("position");
  assert.doesNotThrow(() => sim.setTargets({ altitude: -1000 }));
  assert.doesNotThrow(() =>
    sim.setPositionTarget({ north: 60, east: 0, altitude: -1000 }),
  );
  assert.doesNotThrow(() =>
    sim.setMission([{ north: 60, east: 0, altitude: -1000, hold: 0 }]),
  );
  const before = sim.snapshot();
  for (const altitude of [-1000.01, 500.01, NaN]) {
    assert.throws(() => sim.setTargets({ altitude }), /altitude/);
    assert.throws(
      () => sim.setPositionTarget({ north: 60, east: 0, altitude }),
      /altitude/,
    );
    assert.throws(
      () => sim.setMission([{ north: 60, east: 0, altitude, hold: 0 }]),
      /altitude/,
    );
    assert.deepEqual(sim.snapshot(), before);
  }
});

// Original 3x3 center samples from taipei-z13-6862-3507.tif, imported at radius250/grid65.
// Source SHA256 af41ac17ed1abbf4ea7e673da6f5810bfde88c21189647a75b158244f59856a0.
const taipeiHomeSlope = {
  kind: "dem",
  north: -7.8125,
  east: -7.8125,
  cellSize: 7.8125,
  rows: 3,
  cols: 3,
  heights: [
    -4.607935982917155, -4.406080355844082, -3.9394520658867833,
    -0.17518156259455964, 0, 0.5260885110098741, 4.186640878653549,
    4.203068660435662, 4.585304043578333,
  ],
};

test("real Taipei slope supports motor spool and physical rotor takeoff without flattening or a false landing impact", () => {
  for (const airframe of ["quadrotor", "hexarotor", "octorotor", "vtol"]) {
    const sim = createSimulation(
      { airframe, startOnGround: true },
      { terrain: taipeiHomeSlope },
    );
    const original = sim.snapshot();
    assert.ok(
      original.ground.normalNed[2] > -Math.cos(Math.PI / 12),
      "real home slope exceeds gentle-landing limit",
    );
    assert.equal(original.ground.clearanceM, 0);
    sim.takeoff();
    const first = sim.step();
    assert.notEqual(first.status, "crashed");
    assert.equal(first.armed, true);
    assert.ok(
      first.motors.every((value) => value > 0 && value < 0.2),
      "motors must actually spool, never teleport to flight speed",
    );
    let previous = first,
      lifted = false;
    for (let i = 1; i < 4000; i++) {
      const s = sim.step();
      assert.notEqual(s.status, "crashed");
      assert.ok(
        s.ground.clearanceM >= -1e-8,
        "support must not penetrate the surveyed surface",
      );
      assert.ok(
        Math.hypot(...s.position.map((v, j) => v - previous.position[j])) <
          0.03,
        "liftoff must be continuous",
      );
      if (s.ground.clearanceM > 0.05) lifted = true;
      previous = s;
    }
    assert.ok(lifted);
    assert.equal(previous.armed, true);
    near(previous.ground.clearanceM, 2, 0.03);
    near(Math.hypot(...previous.velocity), 0, 0.02);
    near(Math.hypot(previous.position[0], previous.position[1]), 0, 0.05);
    assert.ok(
      previous.ground.normalNed[2] > -Math.cos(Math.PI / 12),
      "the surveyed slope must remain steep",
    );
    assert.deepEqual(
      createSimulation(
        { airframe, startOnGround: true },
        { terrain: taipeiHomeSlope },
      ).snapshot(),
      original,
    );
  }
});

test("existing slope support does not excuse a new airborne steep landing or invent unlimited static friction", () => {
  const landing = createSimulation({}, { terrain: taipeiHomeSlope });
  landing.setMode("land");
  let s;
  for (let i = 0; i < 3000; i++) {
    s = landing.step();
    if (s.status === "crashed") break;
  }
  assert.equal(s.status, "crashed");
  assert.equal(s.navigation.reason, "terrain-impact");
  assert.equal(s.armed, false);
  const steep = {
    kind: "dem",
    north: -10,
    east: -10,
    cellSize: 10,
    rows: 3,
    cols: 3,
    heights: [-10, -10, -10, 0, 0, 0, 10, 10, 10],
  };
  const slipping = createSimulation(
    { startOnGround: true },
    { terrain: steep },
  );
  slipping.takeoff();
  const specific = slipping.readSensors().specificForceFrd;
  assert.ok(
    specific[0] < -0.5,
    "finite mu=.8 cannot cancel all downslope gravity on a 45-degree surface",
  );
});

test("physical spawn and per-vehicle home drive rotated flight, local navigation, geofence and RTL without renderer offsets", () => {
  const options = {
    spawn: { north: 100, east: 50, altitude: 2, heading: Math.PI / 2 },
    home: [100, 50],
  };
  const sim = createSimulation({ geofenceRadius: 30 }, options);
  const initial = sim.snapshot();
  assert.deepEqual(initial.position, [100, 50, -2]);
  near(initial.attitude[2], Math.PI / 2);
  assert.deepEqual(initial.navigation.home, [100, 50]);
  options.spawn.north = 200;
  options.home[0] = 200;
  sim.setMode("position");
  sim.setPositionTarget({ north: 108, east: 50, altitude: 2 });
  advance(sim, 18);
  near(sim.snapshot().position[0], 108, 0.2);
  assert.equal(sim.snapshot().navigation.fenceBreached, false);
  const before = sim.snapshot();
  assert.throws(
    () => sim.setMission([{ north: 0, east: 0, altitude: 2, hold: 0 }]),
    /geofence/,
  );
  assert.deepEqual(sim.snapshot(), before);
  sim.setMode("rtl");
  const landed = advance(sim, 35);
  assert.equal(landed.navigation.completed, true);
  assert.equal(landed.armed, false);
  near(landed.position[0], 100, 0.7);
  near(landed.position[1], 50, 0.7);
  sim.reset();
  assert.deepEqual(sim.snapshot(), initial);
});

test("spawn AGL and manual datum limits use actual home terrain while external plants reject offset frames", () => {
  const heights = Array(9).fill(0);
  heights[4] = 100;
  const terrain = {
    kind: "heightfield",
    north: 40,
    east: 40,
    cellSize: 10,
    rows: 3,
    cols: 3,
    heights,
  };
  const options = {
    terrain,
    home: [50, 50],
    spawn: { north: 50, east: 50, altitude: 2, heading: 0 },
  };
  const sim = createSimulation({ geofenceHeight: 50 }, options);
  assert.equal(sim.snapshot().ground.heightM, 100);
  assert.equal(sim.snapshot().position[2], -102);
  assert.doesNotThrow(() => sim.setTargets({ altitude: 149 }));
  assert.throws(() => sim.setTargets({ altitude: 151 }), /altitude/);
  sim.setMode("position");
  assert.doesNotThrow(() =>
    sim.setPositionTarget({ north: 50, east: 50, altitude: 149 }),
  );
  assert.throws(
    () => sim.setPositionTarget({ north: 50, east: 50, altitude: 151 }),
    /altitude|geofence/,
  );
  const wing = createSimulation(
    { airframe: "fixedwing" },
    {
      spawn: { north: 20, east: 30, altitude: 30, heading: Math.PI / 2 },
      home: [20, 30],
    },
  );
  near(wing.snapshot().velocity[0], 0);
  near(wing.snapshot().velocity[1], 15);
  const highWing = createSimulation(
    { airframe: "fixedwing" },
    { ...options, spawn: { ...options.spawn, altitude: 30 } },
  );
  highWing.takeoff();
  assert.equal(highWing.snapshot().navigation.positionAltitude, 130);
  assert.throws(
    () =>
      createSimulation(
        { startOnGround: true },
        { controlSource: "external", home: [0, 0] },
      ),
    /default spawn/,
  );
  assert.throws(
    () =>
      createSimulation(
        { startOnGround: true },
        { spawn: { north: 0, east: 0, altitude: 1, heading: 0 } },
      ),
    /spawn/,
  );
});

test("opaque simulation rollback includes gimbal filters, wind configuration and internal/external input clocks", () => {
  const config = { noise: 0.2, turbulence: 0.3, seed: 51 };
  const sim = createSimulation(config),
    reference = createSimulation(config);
  const before = sim.snapshot(),
    rollback = sim.checkpoint();
  sim.setWind({ speed: 3, direction: 90 });
  sim.gust();
  sim.setGimbal({
    mode: "body",
    target: { yaw: 1, pitch: 0.2, roll: 0.1 },
    maxRate: { yaw: 0.3, pitch: 0.3, roll: 0.3 },
  });
  advance(sim, 0.5);
  rollback();
  assert.deepEqual(sim.snapshot(), before);
  assert.throws(rollback, /already/);
  for (let i = 0; i < 30; i++) assert.deepEqual(sim.step(), reference.step());
  const external = createSimulation(
      { startOnGround: true },
      { controlSource: "external" },
    ),
    restore = external.checkpoint();
  external.applyExternalActuators({
    sequence: 1,
    timeUs: 0,
    armed: true,
    rotorSpeedCommands: [0.7, 0.7, 0.7, 0.7],
  });
  external.stepExternal();
  restore();
  assert.equal(external.readSensors().timeUs, 0);
  assert.doesNotThrow(() =>
    external.applyExternalActuators({
      sequence: 1,
      timeUs: 0,
      armed: true,
      rotorSpeedCommands: [0.7, 0.7, 0.7, 0.7],
    }),
  );
});
