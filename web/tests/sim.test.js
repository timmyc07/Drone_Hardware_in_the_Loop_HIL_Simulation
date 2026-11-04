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
    assert.ok(-s.position[2] > (airframe === "quadrotor" ? 1.9 : 27));
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
