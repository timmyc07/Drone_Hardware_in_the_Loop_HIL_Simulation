import test from "node:test";
import assert from "node:assert/strict";
import {
  createGimbal,
  DEFAULT_GIMBAL_SETTINGS,
  GIMBAL_LIMITS,
  validateGimbalSettings,
  validateGimbalState,
  gimbalRotation,
} from "../gimbal.js";
import { bodyToNedMatrix } from "../sim.js";
const rad = (x) => (x * Math.PI) / 180;
const settings = (mode = "body", target = { yaw: 0, pitch: 0, roll: 0 }) => ({
  ...structuredClone(DEFAULT_GIMBAL_SETTINGS),
  mode,
  target,
});
function quaternion(yaw = 0, pitch = 0, roll = 0) {
  const cy = Math.cos(yaw / 2),
    sy = Math.sin(yaw / 2),
    cp = Math.cos(-pitch / 2),
    sp = Math.sin(-pitch / 2),
    cr = Math.cos(roll / 2),
    sr = Math.sin(roll / 2);
  return [
    cy * cp * cr + sy * sp * sr,
    cy * cp * sr - sy * sp * cr,
    cy * sp * cr + sy * cp * sr,
    sy * cp * cr - cy * sp * sr,
  ];
}
const multiply = (a, b) =>
  Array.from({ length: 9 }, (_, i) => {
    const r = Math.floor(i / 3) * 3,
      c = i % 3;
    return a[r] * b[c] + a[r + 1] * b[c + 3] + a[r + 2] * b[c + 6];
  });
const near = (actual, expected, tolerance = 1e-10) =>
  assert.ok(
    Math.abs(actual - expected) < tolerance,
    `${actual} vs ${expected}`,
  );
function settle(gimbal, q, seconds = 3, dt = 0.005) {
  for (let t = 0; t < seconds - 1e-8; t += dt) gimbal.update(dt, q);
  return gimbal.snapshot();
}
test("body mount follows aircraft while NED mode restores full orientation after a three-axis aircraft rotation", () => {
  const q = quaternion(rad(40), rad(20), rad(15)),
    target = { yaw: rad(10), pitch: rad(15), roll: rad(5) };
  const body = createGimbal(settings("body", target));
  body.update(0.005, q);
  assert.deepEqual(body.snapshot().actual, target);
  near(body.snapshot().trackingErrorRad, 0);
  const stabilized = createGimbal(settings("ned", target));
  const state = settle(stabilized, q);
  assert.ok(state.trackingErrorRad < 1e-10);
  const world = multiply(bodyToNedMatrix(q), gimbalRotation(state.actual)),
    desired = gimbalRotation(target);
  world.forEach((x, i) => near(x, desired[i]));
  validateGimbalState(state, q);
});
test("joint travel is limited by simulation time, holds when unstepped and cannot wrap through the yaw hard stop", () => {
  const g = createGimbal(
    settings("body", { yaw: rad(179), pitch: 0, roll: 0 }),
  );
  g.set(settings("body", { yaw: rad(-179), pitch: 0, roll: 0 }));
  near(g.snapshot().actual.yaw, rad(179));
  const before = g.snapshot();
  assert.deepEqual(g.snapshot(), before);
  g.update(0.01, [1, 0, 0, 0]);
  near(g.snapshot().actual.yaw, rad(178.1));
  assert.equal(g.snapshot().rateLimited.yaw, true);
  const final = settle(g, [1, 0, 0, 0], 4);
  near(final.actual.yaw, rad(-179));
  assert.equal(final.rateLimited.yaw, false);
  const endpoint = createGimbal(
    settings("body", { yaw: -Math.PI, pitch: 0, roll: 0 }),
  );
  endpoint.set(settings("ned"));
  endpoint.update(0.005, quaternion(-Math.PI, 0, 0));
  near(endpoint.snapshot().actual.yaw, -Math.PI);
  assert.ok(endpoint.snapshot().trackingErrorRad < 1e-10);
  const a = createGimbal(settings()),
    b = createGimbal(settings());
  const demand = settings("body", { yaw: 1, pitch: 0.5, roll: 0.3 });
  a.set(demand);
  b.set(demand);
  settle(a, [1, 0, 0, 0], 0.4, 0.005);
  settle(b, [1, 0, 0, 0], 0.4, 0.01);
  for (const axis of ["yaw", "pitch", "roll"])
    near(a.snapshot().actual[axis], b.snapshot().actual[axis]);
});
test("NED stabilization reports physical roll saturation and orientation error instead of claiming a held pose", () => {
  const q = quaternion(0, 0, rad(90)),
    g = createGimbal(settings("ned"));
  const state = settle(g, q);
  near(state.actual.roll, -Math.PI / 4);
  assert.equal(state.limited.roll, true);
  near(state.trackingErrorRad, Math.PI / 4);
  validateGimbalState(state, q);
});
test("both Euler poles preserve a feasible previous yaw and solve the observable roll combination", () => {
  for (const sign of [-1, 1]) {
    const g = createGimbal(settings("body", { yaw: 0.4, pitch: 0, roll: 0 }));
    g.set(settings("ned"));
    const q = quaternion(0, (sign * Math.PI) / 2, 0);
    const state = settle(g, q);
    near(state.actual.yaw, 0.4);
    assert.ok(state.trackingErrorRad < 1e-8);
    assert.ok(Object.values(state.actual).every(Number.isFinite));
    validateGimbalState(state, q);
  }
  const g = createGimbal(settings("body", { yaw: 1, pitch: 0, roll: 0 }));
  g.set(settings("ned"));
  const q = quaternion(0, -Math.PI / 2, 0);
  const state = settle(g, q);
  near(state.actual.yaw, Math.PI / 4);
  near(state.actual.roll, -Math.PI / 4);
  assert.ok(state.trackingErrorRad < 1e-8);
});
test("settings and snapshots are strict, cloned and atomic at every public trust boundary", () => {
  assert.ok(Object.isFrozen(DEFAULT_GIMBAL_SETTINGS.target));
  assert.ok(Object.isFrozen(GIMBAL_LIMITS.roll));
  const initialPose = [1, 0, 0, 0],
    isolated = createGimbal(DEFAULT_GIMBAL_SETTINGS, initialPose);
  initialPose[0] = 0;
  assert.doesNotThrow(() => isolated.reset());
  const g = createGimbal(),
    before = g.snapshot();
  for (const value of [
    null,
    [],
    new Date(),
    {},
    { ...settings(), extra: 1 },
    { ...settings(), target: { yaw: 0, pitch: NaN, roll: 0 } },
    { ...settings(), maxRate: { yaw: 0, pitch: 1, roll: 1 } },
  ])
    assert.throws(() => g.set(value));
  assert.deepEqual(g.snapshot(), before);
  for (const [dt, q] of [
    [NaN, [1, 0, 0, 0]],
    [0.1, [1, 0, 0, 0]],
    [0.005, [0, 0, 0, 0]],
    [0.005, Array(4)],
  ])
    assert.throws(() => g.update(dt, q));
  assert.deepEqual(g.snapshot(), before);
  const copy = validateGimbalState(before, [1, 0, 0, 0]);
  copy.actual.yaw = 1;
  assert.deepEqual(g.snapshot(), before);
  assert.throws(() =>
    validateGimbalState({ ...before, trackingErrorRad: 0.1 }, [1, 0, 0, 0]),
  );
  assert.throws(() =>
    validateGimbalState({ ...before, actual: { ...before.actual, roll: 1 } }),
  );
  assert.throws(() =>
    validateGimbalState({
      ...before,
      limited: { yaw: 0, pitch: false, roll: false },
    }),
  );
  assert.throws(() => validateGimbalState({ ...before, extra: 1 }));
  assert.throws(() => validateGimbalSettings({ ...settings(), mode: "world" }));
});
