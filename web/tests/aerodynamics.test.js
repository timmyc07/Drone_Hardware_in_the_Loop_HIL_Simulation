import test from "node:test";
import assert from "node:assert/strict";
import { aerodynamicForces, wingTrim, stallSpeed } from "../aerodynamics.js";
import { AIRFRAME_PRESETS } from "../sim.js";
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
