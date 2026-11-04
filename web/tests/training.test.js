import test from "node:test";
import assert from "node:assert/strict";
import { createTrainer } from "../training.js";

const sample = (time, position = [0, 0, -2], extra = {}) => ({
  time,
  position,
  velocity: [0, 0, 0],
  status: "flying",
  armed: true,
  ...extra,
});

test("gate crossing uses the ordered plane and interpolated position", () => {
  const t = createTrainer("gates");
  t.update(sample(0, [10, 0, -2]));
  assert.equal(t.update(sample(1, [7, 0, -2])).passed, 0); // backward
  assert.equal(t.update(sample(2, [9, 3, -2])).misses, 1); // outside
  t.update(sample(3, [0, 0, -2]));
  assert.equal(t.update(sample(4, [16, 0, -2])).passed, 1); // large jump
  assert.equal(t.update(sample(5, [0, 9, -2])).passed, 1); // misses second gate
  t.update(sample(6, [8, 7, -2]));
  assert.equal(t.update(sample(7, [8, 9, -2])).passed, 2);
  t.update(sample(8, [1, 8, -2]));
  assert.equal(t.update(sample(9, [-1, 8, -2])).passed, 3);
  t.update(sample(10, [0, 1, -2]));
  const done = t.update(sample(11, [0, -1, -2]));
  assert.equal(done.complete, true);
  assert.deepEqual(t.update(sample(20)), done);
});

test("hover must be continuous and reset when time goes backwards", () => {
  const t = createTrainer("hover");
  t.update(sample(0));
  assert.equal(t.update(sample(9)).complete, false);
  t.update(sample(10, [2, 0, -2]));
  t.update(sample(11));
  assert.equal(t.update(sample(20)).complete, false);
  assert.equal(t.update(sample(21)).complete, true);
  const restarted = t.update(sample(1));
  assert.equal(restarted.elapsed, 0);
  assert.equal(restarted.complete, false);
});

test("landing requires descent and two continuous seconds on the ground", () => {
  const t = createTrainer("landing");
  t.update(sample(0, [0, 0, 0]));
  assert.equal(t.update(sample(10, [0, 0, 0])).complete, false);
  t.update(sample(11, [0, 0, -1]));
  t.update(sample(12, [0.5, 0, 0]));
  assert.equal(t.update(sample(14, [0.5, 0, 0])).score, 950);
});

test("crash latches, fence penalty occurs once, and invalid input is rejected", () => {
  assert.throws(() => createTrainer("nope"));
  const t = createTrainer("hover");
  assert.throws(() => t.update(sample(0, [NaN, 0, 0])));
  t.update(sample(0));
  t.update(sample(1, [0, 0, -2], { navigation: { fenceBreached: true } }));
  assert.equal(
    t.update(sample(10, [0, 0, -2], { navigation: { fenceBreached: true } }))
      .score,
    900,
  );
  t.reset();
  assert.equal(
    t.update(sample(1, [0, 0, -2], { status: "crashed" })).reason,
    "crashed",
  );
  assert.equal(t.update(sample(2)).reason, "crashed");
  t.reset();
  assert.equal(t.update(sample(0)).reason, "");
});

test("free flight never completes or scores", () => {
  const t = createTrainer();
  t.update(sample(0));
  const result = t.update(sample(100));
  assert.equal(result.score, 0);
  assert.equal(result.complete, false);
});

test("course compatibility rejects unknown and unsuitable aircraft", async () => {
  const { isCourseSupported } = await import("../training.js");
  assert.equal(isCourseSupported("circuit", "fixedwing"), true);
  assert.equal(isCourseSupported("hover", "fixedwing"), false);
  assert.equal(isCourseSupported("free", "vtol"), true);
  assert.equal(isCourseSupported("unknown", "vtol"), false);
  assert.equal(isCourseSupported("circuit", "unknown"), false);
  assert.throws(() => createTrainer("hover", "fixedwing"));
  assert.throws(() => createTrainer("free", "unknown"));
});

test("circuit uses ordered positive plane crossings with interpolated positions", () => {
  const t = createTrainer("circuit", "fixedwing");
  t.update(sample(0, [110, 0, -30]));
  assert.equal(t.update(sample(1, [90, 0, -30])).passed, 0);
  assert.equal(t.update(sample(2, [110, 50, -30])).misses, 1);
  t.update(sample(3, [80, 0, -30]));
  assert.equal(t.update(sample(4, [120, 0, -30])).passed, 1);
  t.update(sample(5, [100, 100, -30]));
  assert.equal(t.update(sample(6, [100, 140, -30])).passed, 2);
  t.update(sample(7, [20, 120, -30]));
  assert.equal(t.update(sample(8, [-20, 120, -30])).passed, 3);
  t.update(sample(9, [0, 20, -30]));
  assert.equal(t.update(sample(10, [0, -20, -30])).complete, true);
});
