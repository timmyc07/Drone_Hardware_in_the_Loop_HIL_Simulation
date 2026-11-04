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
  assert.ok(s.metrics.peakError < 0.2);
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
