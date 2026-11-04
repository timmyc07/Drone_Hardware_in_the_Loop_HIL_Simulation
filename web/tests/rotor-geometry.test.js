import test from "node:test";
import assert from "node:assert/strict";
import {
  AIRFRAMES,
  MULTIROTOR_AIRFRAMES,
  ROTOR_LAYOUTS,
  rotorCount,
  getRotorLayout,
} from "../rotor-geometry.js";
test("shared FRD layouts have balanced, full-rank regular geometry and truthful spin/torque signs", () => {
  assert.deepEqual(AIRFRAMES, [
    "quadrotor",
    "fixedwing",
    "vtol",
    "hexarotor",
    "octorotor",
  ]);
  assert.deepEqual(MULTIROTOR_AIRFRAMES, [
    "quadrotor",
    "hexarotor",
    "octorotor",
  ]);
  for (const airframe of AIRFRAMES) {
    const layout = getRotorLayout(airframe, 0.3),
      n = rotorCount(airframe);
    assert.equal(layout.length, n);
    assert.ok(Object.isFrozen(layout));
    const rows = [
      layout.map(() => 1),
      layout.map((r) => r.rollScale),
      layout.map((r) => r.pitchScale),
      layout.map((r) => r.yawTorqueSign),
    ];
    for (let a = 0; a < 4; a++)
      for (let b = 0; b < 4; b++)
        assert.ok(
          Math.abs(
            rows[a].reduce((sum, v, i) => sum + v * rows[b][i], 0) -
              (a === b ? n : 0),
          ) < 1e-12,
        );
    layout.forEach((r, i) => {
      assert.equal(r.index, i);
      assert.equal(r.spin, r.yawTorqueSign === 1 ? "ccw" : "cw");
      assert.ok(Math.abs(Math.hypot(...r.positionFrd) - 0.3) < 1e-15);
      assert.ok(
        Math.abs((r.rollScale * 0.3) / Math.SQRT2 + r.positionFrd[1]) < 1e-15,
      );
      assert.ok(Object.isFrozen(r.positionFrd));
    });
  }
  assert.equal(ROTOR_LAYOUTS.fixedwing, ROTOR_LAYOUTS.quadrotor);
  assert.equal(ROTOR_LAYOUTS.vtol, ROTOR_LAYOUTS.quadrotor);
  assert.deepEqual(
    getRotorLayout("quadrotor").map((r) => r.rollScale),
    [-1, -1, 1, 1],
  );
  assert.deepEqual(
    getRotorLayout("quadrotor").map((r) => r.pitchScale),
    [1, -1, -1, 1],
  );
  for (const bad of ["hexa", null, new String("quadrotor"), ["quadrotor"]])
    assert.throws(() => rotorCount(bad));
  for (const bad of [0, -1, NaN, Infinity, 3])
    assert.throws(() => getRotorLayout("hexarotor", bad));
});
