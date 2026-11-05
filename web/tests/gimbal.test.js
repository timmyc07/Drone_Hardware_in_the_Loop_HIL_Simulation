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
test("both Euler poles minimize joint travel while preserving the observable roll combination", () => {
  for (const sign of [-1, 1]) {
    const g = createGimbal(settings("body", { yaw: 0.4, pitch: 0, roll: 0 }));
    g.set(settings("ned"));
    const q = quaternion(0, (sign * Math.PI) / 2, 0);
    const state = settle(g, q);
    near(state.actual.yaw, 0.2);
    assert.ok(state.trackingErrorRad < 1e-8);
    assert.ok(Object.values(state.actual).every(Number.isFinite));
    validateGimbalState(state, q);
  }
  const g = createGimbal(settings("body", { yaw: 1, pitch: 0, roll: 0 }));
  g.set(settings("ned"));
  const q = quaternion(0, -Math.PI / 2, 0);
  const state = settle(g, q);
  near(state.actual.yaw, 0.5);
  near(state.actual.roll, -0.5);
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

test("opaque rollback restores NED joint history and the remembered body pose used by later commands", () => {
  const initial = settings("ned", { yaw: 0.2, pitch: 0.3, roll: 0.1 });
  const g = createGimbal(initial),
    reference = createGimbal(initial);
  const first = quaternion(0.6, 0.1, 0);
  g.update(0.01, first);
  reference.update(0.01, first);
  const before = g.snapshot(),
    restore = g.checkpoint();
  g.update(0.02, quaternion(-0.7, -0.2, 0.3));
  g.set(settings("body", { yaw: -1, pitch: 0.2, roll: 0 }));
  restore();
  assert.deepEqual(g.snapshot(), before);
  assert.throws(restore, /already/);
  const next = settings("ned", { yaw: 0.5, pitch: 0.4, roll: 0 });
  assert.deepEqual(g.set(next), reference.set(next));
  assert.deepEqual(
    g.update(0.01, quaternion(0.8, 0, 0.1)),
    reference.update(0.01, quaternion(0.8, 0, 0.1)),
  );
});

function rotationError(actual, expected) {
  const chord = Math.hypot(...actual.map((value, i) => value - expected[i]));
  return 2 * Math.asin(Math.min(1, chord / (2 * Math.sqrt(2))));
}
function achieved(q, actual) {
  return multiply(bodyToNedMatrix(q), gimbalRotation(actual));
}

test("near-nadir ULP perturbations do not create roll-clipped two-radian pointing errors", () => {
  const pair = [
    [0.999997067768133, -1.3902470784241835e-7, 4.1641869872153523e-7, -0.002421663672546967],
    [0.999997067768133, -1.3902470744330812e-7, 4.1641869892604954e-7, -0.0024216636725467957],
  ];
  const input = settings("ned", { yaw: 0, pitch: Math.PI / 2, roll: 0 });
  const states = pair.map((q) => createGimbal(input, q).snapshot());
  for (const [index, state] of states.entries()) {
    assert.ok(state.trackingErrorRad < 1e-6);
    near(rotationError(achieved(pair[index], state.actual), gimbalRotation(input.target)),
      state.trackingErrorRad, 1e-12);
    assert.equal(state.limited.pitch, true);
  }
  for (const axis of ["yaw", "pitch", "roll"])
    near(states[0].actual[axis], states[1].actual[axis], 1e-9);
  near(states[0].trackingErrorRad, states[1].trackingErrorRad, 1e-9);
});

test("bounded analytic fit is no worse than an independent coarse orientation grid or old scalar clipping", () => {
  for (let index = 0; index < 24; index++) {
    const q = quaternion(index * 0.71 - 2.8, Math.sin(index * 1.3) * 1.5,
      Math.cos(index * 0.9) * 2.4);
    const target = { yaw: Math.sin(index) * 2.8, pitch: Math.cos(index * 0.7) * 1.5,
      roll: Math.sin(index * 0.3) * Math.PI / 4 };
    const g = createGimbal(settings("ned", target), q);
    const desired = gimbalRotation(target), state = g.snapshot();
    const actualError = rotationError(achieved(q, state.actual), desired);
    near(actualError, state.trackingErrorRad, 1e-12);
    const relative = multiply(
      [0, 3, 6, 1, 4, 7, 2, 5, 8].map((i) => bodyToNedMatrix(q)[i]), desired,
    );
    const clipped = {
      yaw: Math.atan2(relative[3], relative[0]),
      pitch: Math.atan2(relative[6], Math.hypot(relative[0], relative[3])),
      roll: Math.max(-Math.PI / 4, Math.min(Math.PI / 4, Math.atan2(relative[7], relative[8]))),
    };
    assert.ok(actualError <= rotationError(achieved(q, clipped), desired) + 1e-12);
    for (let yi = -6; yi <= 6; yi++)
      for (let pi = -4; pi <= 4; pi++)
        for (let ri = -2; ri <= 2; ri++) {
          const trial = { yaw: yi * Math.PI / 6, pitch: pi * Math.PI / 8, roll: ri * Math.PI / 8 };
          assert.ok(actualError <= rotationError(achieved(q, trial), desired) + 1e-12,
            `case ${index}: analytic fit worse than grid at ${JSON.stringify(trial)}`);
        }
    validateGimbalState(state, q);
  }
});

test("yaw hard-stop ties choose the same endpoint across ULP perturbations", () => {
  const states = [-4e-16, 4e-16].map(perturbation => {
    const g = createGimbal(settings("body"));g.set(settings("ned"));
    const q = quaternion(Math.PI, 0.3, 0);q[0] += perturbation;
    return settle(g, q, 4);
  });
  near(states[0].actual.yaw, -Math.PI);near(states[1].actual.yaw, -Math.PI);
  for (const state of states) assert.ok(state.trackingErrorRad < 1e-12);
});

test("both pitch poles retain equivalent raw joints across tiny body perturbations without swallowing real yaw", () => {
  for (const sign of [-1, 1]) {
    const input = settings("ned", { yaw: 0, pitch: sign * Math.PI / 2, roll: 0 });
    const a = createGimbal(input), b = createGimbal(input);
    for (let tick = 0; tick < 2000; tick++) {
      const q = quaternion(0, 5e-6 * Math.sin(tick / 20), 0);
      const perturbed = [...q]; perturbed[1] += 4e-16; perturbed[3] -= 2e-16;
      const first = a.update(0.005, q), second = b.update(0.005, perturbed);
      for (const axis of ["yaw", "pitch", "roll"])
        near(first.actual[axis], second.actual[axis], 1e-9);
      near(first.actual.yaw, 0, 1e-9);near(first.actual.roll, 0, 1e-9);
    }
    const q = quaternion(0.1, 0, 0), state = settle(a, q);
    assert.ok(Math.abs(state.actual.yaw) + Math.abs(state.actual.roll) > 0.05);
    assert.ok(rotationError(achieved(q, state.actual), gimbalRotation(input.target)) < 1e-12);
  }
});

test("long near-nadir hover remains continuous across ULP-sized attitude perturbations", () => {
  const input = settings("ned", { yaw: 0, pitch: Math.PI / 2, roll: 0 });
  const a = createGimbal(input), b = createGimbal(input);
  let previous = a.snapshot();
  for (let tick = 0; tick < 60000; tick++) {
    const q = quaternion(-0.005, 8e-7 * Math.cos(tick / 1000), 8e-7 * Math.sin(tick / 1000));
    const perturbed = [...q]; perturbed[1] += 4e-16; perturbed[2] -= 2e-16;
    const first = a.update(0.005, q), second = b.update(0.005, perturbed);
    // Euler coordinates are ill-conditioned near a pole; compare the physical rotations.
    assert.ok(rotationError(achieved(q, first.actual), achieved(perturbed, second.actual)) < 1e-9);
    for (const axis of ["yaw", "pitch", "roll"])
      assert.ok(Math.abs(first.actual[axis] - previous.actual[axis]) <= first.maxRate[axis] * 0.005 + 1e-12);
    assert.ok(first.trackingErrorRad < 2e-6);
    previous = first;
  }
});
