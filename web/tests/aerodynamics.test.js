import test from "node:test";
import assert from "node:assert/strict";
import { aerodynamicForces, wingTrim, stallSpeed } from "../aerodynamics.js";
import { AIRFRAME_PRESETS, createSimulation } from "../sim.js";
const config = AIRFRAME_PRESETS.fixedwing;
const near = (a, b, t = 1e-7) =>
  assert.ok(Math.abs(a - b) < t, `${a} differs from ${b}`);
const bodyVelocity = (speed, alpha) => [
  speed * Math.cos(alpha),
  0,
  speed * Math.sin(alpha),
];

test("solved level trim balances weight, drag and pitching moment with real wing force", () => {
  const trim = wingTrim(config),
    wing = trim.aero;
  near(wing.liftN + trim.thrust * Math.sin(trim.alpha), config.mass * 9.80665);
  near(trim.thrust * Math.cos(trim.alpha), wing.dragN);
  near(wing.momentBody[1], 0, 1e-6);
  assert.ok(trim.thrust > 0 && trim.thrust < config.maxPusherThrust);
  assert.ok(Math.abs(trim.alpha) < 0.2);
});
test("lift scales with dynamic pressure and post-stall lift coefficient falls while drag increases", () => {
  const normal = aerodynamicForces(config, bodyVelocity(15, 0.2));
  const fast = aerodynamicForces(config, bodyVelocity(30, 0.2));
  near(fast.liftN, normal.liftN * 4);
  near(fast.dynamicPressurePa, normal.dynamicPressurePa * 4);
  const stalled = aerodynamicForces(config, bodyVelocity(15, 0.6));
  assert.equal(stalled.stalled, true);
  assert.ok(stalled.liftN < normal.liftN);
  assert.ok(stalled.dragN > normal.dragN * 3);
  assert.ok(
    stallSpeed(config) > 0 && stallSpeed(config) < config.cruiseAirspeed,
  );
});
test("aerodynamic lift and sideforce do no work along relative wind; drag always removes energy", () => {
  const velocity = [14, 3, 2];
  const wing = aerodynamicForces(
    config,
    velocity,
    [0.1, 0.2, -0.1],
    [0.2, -0.1, 0.3],
  );
  near(
    wing.forceBody.reduce((sum, v, i) => sum + v * velocity[i], 0),
    -wing.dragN * Math.hypot(...velocity),
  );
  assert.ok(wing.dragN >= 0);
});
test("control surface moments vanish at zero airspeed and have correct control signs in flight", () => {
  const zero = aerodynamicForces(config, [0, 0, 0], [0, 0, 0], [0.3, 0.3, 0.3]);
  near(Math.hypot(...zero.forceBody, ...zero.momentBody), 0);
  const baseline = aerodynamicForces(config, [15, 0, 0]);
  for (let axis = 0; axis < 3; axis++) {
    const controls = [0, 0, 0];
    controls[axis] = 0.1;
    assert.ok(
      aerodynamicForces(config, [15, 0, 0], [0, 0, 0], controls).momentBody[
        axis
      ] > baseline.momentBody[axis],
    );
  }
});

test("pure spanwise wind has no fictitious chordwise lift or alpha discontinuity at zero forward speed", () => {
  const left = aerodynamicForces(config, [-1e-15, -6, 0]),
    right = aerodynamicForces(config, [1e-15, -6, 0]);
  near(left.alphaRad, 0);
  near(right.alphaRad, 0);
  near(left.liftN, 0);
  near(right.liftN, 0);
  near(left.forceBody[0], 0);
  near(right.forceBody[0], 0);
  assert.ok(
    left.forceBody[1] > 0 && right.forceBody[1] > 0,
    "drag opposes the lateral relative velocity",
  );
});

test("sub-millimetre-per-second airflow has no resolved direction despite roundoff", () => {
  const velocity = [
    5.296836005976857e-8, -1.8105805148785488e-10, -2.4933150901456834e-13,
  ];
  for (const speed of [0, Math.hypot(...velocity), 0.000999, 0.001]) {
    const flow = aerodynamicForces(config, [
      -speed / Math.sqrt(2),
      speed / Math.sqrt(2),
      0,
    ]);
    assert.equal(flow.alphaRad, 0);
    assert.equal(flow.betaRad, 0);
    near(flow.airspeedMps, speed, 1e-15);
    near(flow.dynamicPressurePa, 0.6125 * speed ** 2, 1e-15);
  }
  for (const perturbation of [-1e-15, 0, 1e-15]) {
    const flow = aerodynamicForces(
      config,
      velocity.map((v) => v + perturbation),
    );
    assert.equal(flow.alphaRad, 0);
    assert.equal(flow.betaRad, 0);
    assert.ok([...flow.forceBody, ...flow.momentBody].every(Number.isFinite));
  }
});

test("resolved airflow retains geometric angles in forward, reverse, vertical and spanwise flight", () => {
  for (const velocity of [
    [0.002, 0.001, -0.001],
    [14, 3, 2],
    [-10, 2, 1],
    [0, 2, 10],
    [0, -6, 0],
  ]) {
    const flow = aerodynamicForces(config, velocity);
    const [u, v, w] = velocity;
    near(flow.alphaRad, Math.hypot(u, w) > 0 ? Math.atan2(w, u) : 0, 1e-15);
    near(flow.betaRad, Math.asin(v / Math.hypot(...velocity)), 1e-15);
  }
});

test("near-spanwise sideslip retains the small angle to the span without asin cancellation", () => {
  for (const lateral of [-6, 6]) {
    const flow = aerodynamicForces(config, [1e-7, lateral, 1e-7]);
    const spanAngle = Math.atan2(Math.SQRT2 * 1e-7, Math.abs(lateral));
    near(Math.PI / 2 - Math.abs(flow.betaRad), spanAngle, 1e-15);
  }
});

test("all five airframes share the near-static and resolved airflow angle convention", () => {
  for (const preset of Object.values(AIRFRAME_PRESETS)) {
    const sim = createSimulation({ ...preset, startOnGround: true });
    for (const speed of [
      0, 5.2968669508507203e-8, 0.000999, 0.001, 0.001001, 5,
    ]) {
      sim.setWind({ speed, direction: 45 });
      const flight = sim.snapshot().flight;
      near(flight.airspeedMps, speed, 1e-15);
      near(flight.alphaRad, speed > 0.001 ? Math.PI : 0, 1e-15);
      near(flight.betaRad, speed > 0.001 ? -Math.PI / 4 : 0, 1e-15);
    }
  }
});
