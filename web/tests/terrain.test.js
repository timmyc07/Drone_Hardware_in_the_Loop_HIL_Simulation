import test from "node:test";
import assert from "node:assert/strict";
import {
  createTerrain,
  validateTerrain,
  DEFAULT_TERRAIN,
  TERRAIN_MESH_EXTENT,
} from "../terrain.js";

function field(overrides = {}) {
  return {
    kind: "heightfield",
    north: 250,
    east: 50,
    cellSize: 10,
    rows: 3,
    cols: 3,
    heights: [0, 0, 0, 0, 10, 0, 0, 0, 0],
    ...overrides,
  };
}
function near(actual, expected, epsilon = 1e-8) {
  assert.ok(Math.abs(actual - expected) < epsilon, `${actual} != ${expected}`);
}
function vectorNear(actual, expected) {
  actual.forEach((value, i) => near(value, expected[i]));
}

test("query boundaries reject invalid coordinates, vectors and ranges without changing terrain", () => {
  for (const terrain of [createTerrain(), createTerrain(field())]) {
    const before = JSON.stringify(terrain.data),
      geometry = terrain.mesh();
    for (const bad of [NaN, Infinity, -Infinity, "0", undefined]) {
      assert.throws(() => terrain.sample(bad, 0));
      assert.throws(() => terrain.sample(0, bad));
    }
    const inherited = [0, 0, 0];
    delete inherited[0];
    Object.setPrototypeOf(inherited, { 0: 0 });
    for (const bad of [
      undefined,
      null,
      {},
      [0, 0],
      [0, 0, 0, 0],
      Array(3),
      inherited,
      [0, NaN, 0],
      [0, 0, Infinity],
      new Float64Array(3),
    ]) {
      assert.throws(() => terrain.raycast(bad, [0, 0, 1], 0, 10));
      assert.throws(() => terrain.raycast([0, 0, -1], bad, 0, 10));
      assert.throws(() => terrain.sweep(bad, [0, 0, 0]));
      assert.throws(() => terrain.sweep([0, 0, -1], bad));
    }
    for (const direction of [
      [0, 0, 0],
      [0, 0, 2],
      [1, 1, 1],
      [0, 0, 1 + 2e-8],
    ])
      assert.throws(() => terrain.raycast([0, 0, -1], direction, 0, 10));
    for (const range of [
      [-1, 10],
      [2, 1],
      [0, Infinity],
      [NaN, 10],
    ])
      assert.throws(() => terrain.raycast([0, 0, -1], [0, 0, 1], ...range));
    assert.throws(() =>
      terrain.sweep([-Number.MAX_VALUE, 0, 0], [Number.MAX_VALUE, 0, 0]),
    );
    assert.equal(JSON.stringify(terrain.data), before);
    assert.equal(terrain.mesh(), geometry);
    assert.equal(terrain.raycast([0, 0, -1], [0, 0, 1], 0, 10).distanceM, 1);
  }
});

test("terrain is strict, cloned and frozen; boundaries and overlapping runway cells stay flat", () => {
  assert.deepEqual(validateTerrain(), DEFAULT_TERRAIN);
  const input = field(),
    canonical = validateTerrain(input);
  input.heights[4] = 20;
  assert.equal(canonical.heights[4], 10);
  assert.ok(Object.isFrozen(canonical) && Object.isFrozen(canonical.heights));
  const sparse = Array(9);
  sparse[4] = 1;
  for (const data of [
    null,
    [],
    { kind: "other" },
    { kind: "flat", heights: [] },
    field({ rows: 2.5 }),
    field({ cols: 66 }),
    field({ cellSize: 0.5 }),
    field({ cellSize: 51 }),
    field({ north: 490 }),
    field({ east: -501 }),
    field({ heights: sparse }),
    field({ heights: [0, 0, 0, 0, NaN, 0, 0, 0, 0] }),
    field({ heights: [0, 0, 0, 0, Infinity, 0, 0, 0, 0] }),
    field({ heights: [0, 0, 0, 0, -51, 0, 0, 0, 0] }),
    field({ heights: [0, 0, 0, 0, 151, 0, 0, 0, 0] }),
    field({ heights: [1, 0, 0, 0, 10, 0, 0, 0, 0] }),
  ])
    assert.throws(() => validateTerrain(data));
  // The protected rectangle lies wholly between E vertices, so testing only vertices would miss it.
  assert.throws(() =>
    validateTerrain(field({ north: 0, east: -50, cellSize: 50 })),
  );
  assert.doesNotThrow(() => validateTerrain(field({ north: 210 })));
});

test("sampling uses the fixed A-D crease and air-facing NED normals", () => {
  const terrain = createTerrain(field());
  near(terrain.sample(257.5, 52.5).heightM, 2.5);
  near(terrain.sample(252.5, 57.5).heightM, 2.5);
  near(terrain.sample(255, 55).heightM, 5); // Bilinear interpolation would return 2.5.
  vectorNear(terrain.sample(257.5, 52.5).normalNed, [
    0,
    -Math.SQRT1_2,
    -Math.SQRT1_2,
  ]);
  vectorNear(terrain.sample(252.5, 57.5).normalNed, [
    -Math.SQRT1_2,
    0,
    -Math.SQRT1_2,
  ]);
  assert.deepEqual(terrain.sample(249.9, 55), {
    heightM: 0,
    normalNed: [0, 0, -1],
  });
  const result = terrain.sample(255, 55);
  result.normalNed[0] = 9;
  assert.notEqual(terrain.sample(255, 55).normalNed[0], 9);
});

test("raycasts hit both sides, seams, vertical rays and exterior flat ground without a phantom valley plane", () => {
  const terrain = createTerrain(field());
  for (const [n, e] of [
    [255, 55],
    [257.5, 52.5],
    [252.5, 57.5],
    [260, 60],
    [250, 60],
    [270, 70],
  ]) {
    const height = terrain.sample(n, e).heightM;
    near(
      terrain.raycast([n, e, -20], [0, 0, 1], 0, 100).distanceM,
      20 - height,
    );
    near(
      terrain.raycast([n, e, 20], [0, 0, -1], 0, 100).distanceM,
      20 + height,
    );
  }
  const outside = terrain.raycast([0, 0, -20], [0, 0, 1], 0, 30);
  assert.deepEqual(outside, {
    distanceM: 20,
    positionNed: [0, 0, 0],
    normalNed: [0, 0, -1],
  });
  assert.equal(terrain.raycast([260, 60, -20], [0, 0, 1], 0, 9), null);
  const valley = createTerrain(
    field({ heights: [0, 0, 0, 0, -10, 0, 0, 0, 0] }),
  );
  near(valley.raycast([260, 60, -20], [0, 0, 1], 0, 100).distanceM, 30);
  near(valley.raycast([260, 60, -20], [0, 0, 1], 25, 100).distanceM, 30);
  const slope = terrain.raycast([245, 60, -5], [1, 0, 0], 0, 40);
  near(slope.distanceM, 10);
  const diagonal = terrain.raycast(
    [245, 45, -5],
    [Math.SQRT1_2, Math.SQRT1_2, 0],
    0,
    100,
  );
  near(diagonal.distanceM, 10 * Math.SQRT2);
});

test("sweeps enter terrain, including ridges between airborne endpoints, and allow takeoff and tangency", () => {
  const terrain = createTerrain(field());
  near(terrain.sweep([245, 60, -5], [275, 60, -5]).fraction, 1 / 3);
  assert.equal(terrain.sweep([260, 60, -10], [260, 60, -20]), null);
  assert.equal(terrain.sweep([260, 60, -10], [260, 60, -10]), null);
  assert.equal(terrain.sweep([245, 60, -10], [275, 60, -10]), null);
  assert.equal(terrain.sweep([260, 60, -10], [260, 60, 0]).fraction, 0);
  near(terrain.sweep([260, 60, -20], [260, 60, 0]).fraction, 0.5);
  const flat = createTerrain();
  assert.equal(flat.sweep([0, 0, 0], [1, 0, 0]), null);
  assert.equal(flat.sweep([0, 0, 0], [0, 0, -1]), null);
  assert.equal(flat.sweep([0, 0, 0], [0, 0, 1]).fraction, 0);
  near(flat.sweep([0, 0, -1], [0, 0, 1]).fraction, 0.5);
});

test("mesh has one continuous up-facing surface, exact samples and four exterior bands", () => {
  for (const terrain of [
    createTerrain(),
    createTerrain(field()),
    createTerrain(
      field({
        rows: 4,
        cols: 5,
        heights: [0, 0, 0, 0, 0, 0, 4, -3, 2, 0, 0, 1, 6, 2, 0, 0, 0, 0, 0, 0],
      }),
    ),
  ]) {
    const { positionsNed: positions, indices } = terrain.mesh(),
      edges = new Map();
    assert.ok(Object.isFrozen(positions) && Object.isFrozen(indices));
    let area = 0;
    for (let k = 0; k < indices.length; k += 3) {
      const points = indices
        .slice(k, k + 3)
        .map((index) => positions.slice(index * 3, index * 3 + 3));
      const [a, b, c] = points;
      const crossD =
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
      assert.ok(crossD < 0);
      area -= crossD / 2;
      const center = [0, 1, 2].map((axis) => (a[axis] + b[axis] + c[axis]) / 3);
      near(-center[2], terrain.sample(center[0], center[1]).heightM);
      for (let j = 0; j < 3; j++) {
        const key = [indices[k + j], indices[k + ((j + 1) % 3)]]
          .sort((x, y) => x - y)
          .join(",");
        edges.set(key, (edges.get(key) || 0) + 1);
      }
    }
    near(area, (TERRAIN_MESH_EXTENT * 2) ** 2, 1e-5);
    for (const [key, count] of edges) {
      assert.ok(count === 1 || count === 2);
      if (count === 1) {
        const [a, b] = key
          .split(",")
          .map((index) =>
            positions.slice(Number(index) * 3, Number(index) * 3 + 3),
          );
        assert.ok(
          [0, 1].some(
            (axis) =>
              Math.abs(a[axis]) === TERRAIN_MESH_EXTENT && a[axis] === b[axis],
          ),
        );
      }
    }
  }
});

test("maximum-resolution diagonal rays walk grid boundaries with bounded work", () => {
  const terrain = createTerrain(
    field({
      north: -500,
      east: -500,
      rows: 65,
      cols: 65,
      heights: Array(65 * 65).fill(0),
    }),
  );
  let coordinateReads = 0;
  const direction = new Proxy([Math.SQRT1_2, Math.SQRT1_2, 0], {
    get(target, key) {
      if (["0", "1", "2"].includes(key)) coordinateReads++;
      return Reflect.get(target, key);
    },
  });
  assert.equal(terrain.raycast([-501, -501, -1], direction, 0, 1000), null);
  assert.ok(
    coordinateReads < 3000,
    `DDA read ${coordinateReads} ray coordinates`,
  );
});

test("DDA nearest hits agree with an independent triangle intersection reference", () => {
  const terrain = createTerrain(
    field({
      rows: 5,
      cols: 5,
      heights: [
        0, 0, 0, 0, 0, 0, 7, -4, 3, 0, 0, -6, 11, -2, 0, 0, 4, -8, 6, 0, 0, 0,
        0, 0, 0,
      ],
    }),
  );
  const { positionsNed, indices } = terrain.mesh();
  const subtract = (a, b) => a.map((value, i) => value - b[i]);
  const cross = (a, b) => [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
  const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
  function reference(origin, direction, min, max) {
    let best = Infinity;
    for (let k = 0; k < indices.length; k += 3) {
      const [a, b, c] = indices
        .slice(k, k + 3)
        .map((i) => positionsNed.slice(i * 3, i * 3 + 3));
      const ab = subtract(b, a),
        ac = subtract(c, a),
        p = cross(direction, ac),
        det = dot(ab, p);
      if (Math.abs(det) < 1e-12) continue;
      const offset = subtract(origin, a),
        u = dot(offset, p) / det;
      if (u < -1e-9 || u > 1 + 1e-9) continue;
      const q = cross(offset, ab),
        v = dot(direction, q) / det;
      if (v < -1e-9 || u + v > 1 + 1e-9) continue;
      const distance = dot(ac, q) / det;
      if (distance >= min && distance <= max) best = Math.min(best, distance);
    }
    return Number.isFinite(best) ? best : null;
  }
  let seed = 401;
  const random = () => (seed = (1664525 * seed + 1013904223) >>> 0) / 2 ** 32;
  for (let i = 0; i < 600; i++) {
    const origin = [
      230 + random() * 80,
      30 + random() * 80,
      -20 + random() * 40,
    ];
    const delta = [random() * 2 - 1, random() * 2 - 1, random() * 2 - 1];
    const direction = delta.map((value) => value / Math.hypot(...delta));
    const min = random() * 3,
      max = 100;
    const expected = reference(origin, direction, min, max);
    const actual = terrain.raycast(origin, direction, min, max);
    if (expected === null) assert.equal(actual, null);
    else near(actual?.distanceM, expected, 1e-7);
  }
});
