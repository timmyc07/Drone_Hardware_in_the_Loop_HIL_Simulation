import test from "node:test";
import assert from "node:assert/strict";
import {
  validateObstacles,
  obstacleBounds,
  rayAabb,
} from "../obstacle-geometry.js";

const box = {
  north: 20,
  east: 0,
  width: 8,
  depth: 18,
  height: 1,
  baseAltitude: 8,
};

test("obstacles require six canonical finite fields and bounded datum/top without mutating input", () => {
  const reversed = Object.fromEntries(Object.entries(box).reverse());
  const canonical = validateObstacles([reversed]);
  assert.deepEqual(Object.keys(canonical[0]), Object.keys(box));
  canonical[0].north = 0;
  assert.equal(reversed.north, 20);
  assert.deepEqual(validateObstacles([]), []);
  for (const input of [
    null,
    {},
    Array(1),
    Array(51).fill(box),
    [{ ...box, baseAltitude: undefined }],
    [{ ...box, baseAltitude: -1001 }],
    [{ ...box, baseAltitude: 1500 }],
    [{ ...box, height: 0 }],
    [{ ...box, north: Infinity }],
    [{ ...box, east: NaN }],
    [{ ...box, width: 101 }],
    [{ ...box, extra: 1 }],
    [{ ...box, [Symbol("extra")]: 1 }],
    [Object.create(box)],
  ])
    assert.throws(() => validateObstacles(input));
  const missing = { ...box };
  delete missing.baseAltitude;
  assert.throws(() => validateObstacles([missing]));
  const extra = [box];
  extra.extra = 1;
  assert.throws(() => validateObstacles(extra));
  assert.equal(
    validateObstacles([{ ...box, baseAltitude: 1499 }])[0].baseAltitude,
    1499,
  );
  assert.equal(
    validateObstacles([{ ...box, baseAltitude: -1000 }])[0].baseAltitude,
    -1000,
  );
});

test("NED bounds preserve elevated and negative datum boxes with explicit collision padding", () => {
  assert.deepEqual(obstacleBounds(box), {
    low: [16, -9, -9],
    high: [24, 9, -8],
  });
  assert.deepEqual(obstacleBounds(box, 0.25), {
    low: [15.75, -9.25, -9.25],
    high: [24.25, 9.25, -7.75],
  });
  assert.deepEqual(obstacleBounds({ ...box, baseAltitude: -20 }), {
    low: [16, -9, 19],
    high: [24, 9, 20],
  });
});

test("raw slab intervals retain inside, behind, parallel, tangent and zero-direction semantics", () => {
  const bounds = { low: [-1, -2, -3], high: [1, 2, 3] };
  assert.deepEqual(rayAabb([-4, 0, 0], [1, 0, 0], bounds), {
    enter: 3,
    exit: 5,
  });
  assert.deepEqual(rayAabb([0, 0, 0], [1, 0, 0], bounds), {
    enter: -1,
    exit: 1,
  });
  assert.deepEqual(rayAabb([4, 0, 0], [1, 0, 0], bounds), {
    enter: -5,
    exit: -3,
  });
  assert.deepEqual(rayAabb([-4, 2, 0], [1, 0, 0], bounds), {
    enter: 3,
    exit: 5,
  });
  assert.equal(rayAabb([-4, 2.01, 0], [1, 0, 0], bounds), null);
  assert.deepEqual(rayAabb([0, 0, 0], [0, 0, 0], bounds), {
    enter: -Infinity,
    exit: Infinity,
  });
  assert.equal(rayAabb([2, 0, 0], [0, 0, 0], bounds), null);
  assert.equal(rayAabb([-4, -4, 0], [1, 0.1, 0], bounds), null);
  assert.deepEqual(rayAabb([-4, 0, 0], [8, 0, 0], bounds), {
    enter: 3 / 8,
    exit: 5 / 8,
  });
});
