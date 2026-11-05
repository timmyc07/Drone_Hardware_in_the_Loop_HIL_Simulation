import test from "node:test";
import assert from "node:assert/strict";
import { createSensorSuite } from "../sensors.js";
import {
  PROXIMITY_AXES,
  PROXIMITY_DEFAULT_CONFIG,
  validateProximityConfig,
  proximityMetadata,
  validateProximityReading,
  exportProximityData,
  PROXIMITY_CSV_HEADER,
} from "../proximity.js";
const state = (time = 0, extra = {}) => ({
  time,
  position: [0, 0, -10],
  velocity: [0, 0, 0],
  quaternion: [1, 0, 0, 0],
  rates: [0, 0, 0],
  ...extra,
});
const box = (
  north,
  east,
  baseAltitude = 0,
  height = 20,
  width = 2,
  depth = 2,
) => ({ north, east, baseAltitude, height, width, depth });
const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-8, `${a} != ${b}`);
const suite = (config = {}, seed = 42) =>
  createSensorSuite({ proximityNoiseM: 0, ...config }, seed);

test("six immutable FRD axes, strict configuration and metadata", () => {
  assert.deepEqual(
    PROXIMITY_AXES.map((v) => v.id),
    ["front", "back", "right", "left", "up", "down"],
  );
  assert.ok(
    PROXIMITY_AXES.every(
      (v) => Object.isFrozen(v) && Object.isFrozen(v.direction),
    ),
  );
  assert.deepEqual(validateProximityConfig(), PROXIMITY_DEFAULT_CONFIG);
  assert.deepEqual(proximityMetadata({ gpsHz: 5 }), {
    minM: 0.1,
    maxM: 40,
    warningM: 3,
  });
  for (const bad of [
    null,
    [],
    { extra: 1 },
    { proximityHz: NaN },
    { proximityHz: 21 },
    { proximityMinM: 4 },
    { proximityMinM: 40 },
    { proximityWarningM: 41 },
    { [Symbol()]: 1 },
  ])
    assert.throws(() => validateProximityConfig(bad));
});
test("CG-origin six walls, flat terrain and rotated body directions", () => {
  const s = suite();
  s.setObstacles([
    box(5, 0),
    box(-7, 0),
    box(0, 9),
    box(0, -11),
    box(0, 0, 13, 2, 2, 2),
  ]);
  assert.deepEqual(s.update(state()).proximity.rangesM, [4, 6, 8, 10, 3, 10]);
  const rotated = s.update(
    state(0.1, { quaternion: [Math.SQRT1_2, 0, 0, Math.SQRT1_2] }),
  ).proximity;
  [8, 10, 6, 4, 3, 10].forEach((n, i) => near(rotated.rangesM[i], n));
  const tilted = suite().update(
    state(0, {
      quaternion: [Math.cos(Math.PI / 8), 0, Math.sin(Math.PI / 8), 0],
    }),
  ).proximity;
  near(tilted.rangesM[1], Math.sqrt(200));
  near(tilted.rangesM[5], Math.sqrt(200));
});
test("heightfield ground, own exclusion, peers and blind-zone occlusion", () => {
  const s = suite();
  s.setTerrain({
    kind: "heightfield",
    north: 40,
    east: 40,
    cellSize: 10,
    rows: 5,
    cols: 5,
    heights: Array.from({ length: 25 }, (_, i) => (i === 12 ? 4 : 0)),
  });
  near(s.update(state(0, { position: [60, 60, -10] })).proximity.rangesM[5], 6);
  s.setPeers(
    [
      { id: "own", position: [0, 0, -10], velocity: [0, 0, 0], radius: 1 },
      { id: "peer", position: [5, 0, -10], velocity: [0, 0, 0], radius: 1 },
    ],
    "own",
  );
  near(s.update(state(0.1)).proximity.rangesM[0], 4);
  s.setObstacles([box(0.055, 0, 9, 2, 0.01, 1), box(5, 0)]);
  const r = s.update(state(0.2)).proximity;
  assert.equal(r.rangesM[0], null);
  assert.equal(r.tooClose[0], true);
  assert.equal(r.valid, true);
  assert.equal(r.reason, "");
});
test("seed, independent streams, delay, dropout, fault, reset and checkpoint", () => {
  const a = suite({ proximityNoiseM: 0.1 }),
    b = suite({ proximityNoiseM: 0.1 });
  for (let i = 0; i < 10; i++)
    assert.deepEqual(a.update(state(i / 10)), b.update(state(i / 10)));
  a.reset();
  assert.deepEqual(
    a.update(state()),
    suite({ proximityNoiseM: 0.1 }).update(state()),
  );
  const restore = a.checkpoint(),
    first = a.update(state(0.1));
  restore();
  assert.deepEqual(a.update(state(0.1)), first);
  const delayed = suite({ latency: 0.2 });
  assert.equal(delayed.update(state()).proximity.reason, "startup");
  const r = delayed.update(state(0.2, { position: [0, 0, -20] })).proximity;
  assert.equal(r.time, 0);
  assert.deepEqual(r.positionNed, [0, 0, -10]);
  near(r.rangesM[5], 10);
  delayed.setFault("proximity", true);
  assert.equal(delayed.snapshot().proximity.positionNed, null);
  assert.equal(delayed.update(state(0.4)).proximity.reason, "fault");
  const drop = suite({ dropout: 1 }).update(state()).proximity;
  assert.equal(drop.reason, "dropout");
  assert.deepEqual(drop.positionNed, [0, 0, -10]);
  const changed = suite({ proximityHz: 1, proximityNoiseM: 10 }),
    unchanged = suite({ proximityHz: 20, proximityNoiseM: 0 });
  for (let i = 0; i < 30; i++) {
    const x = changed.update(state(i / 20)),
      y = unchanged.update(state(i / 20));
    for (const key of [
      "gps",
      "barometer",
      "magnetometer",
      "rangefinder",
      "lidar",
      "opticalFlow",
      "radar",
    ])
      assert.deepEqual(x[key], y[key]);
  }
});
test("strict reading trust boundaries, cloned values and all invalid states", () => {
  const raw = suite().update(state()).proximity;
  const clean = validateProximityReading(raw);
  clean.rangesM[5] = 1;
  assert.equal(raw.rangesM[5], 10);
  for (const mutate of [
    (v) => (v.extra = 1),
    (v) => (v.time = NaN),
    (v) => (v.valid = false),
    (v) => (v.reason = "fault"),
    (v) => (v.rangesM[0] = NaN),
    (v) => delete v.tooClose[0],
    (v) => (v.tooClose[5] = true),
    (v) => (v.quaternion = [2, 0, 0, 0]),
    (v) => (v.positionNed = null),
    (v) => (v.rangesM.extra = 1),
    (v) => Object.defineProperty(v, "hidden", { value: 1 }),
    (v) => (v[Symbol()] = 1),
  ]) {
    const bad = structuredClone(raw);
    mutate(bad);
    assert.throws(() => validateProximityReading(bad));
  }
  assert.throws(() => validateProximityReading(raw, { proximityMaxM: 50 }));
  for (const reason of ["startup", "fault", "dropout", "no-return"]) {
    const invalid = {
      ...raw,
      valid: false,
      reason,
      rangesM: Array(6).fill(null),
      tooClose: Array(6).fill(false),
      ...(reason === "startup" ? { positionNed: null, quaternion: null } : {}),
    };
    assert.deepEqual(validateProximityReading(invalid), invalid);
    if (reason === "dropout" || reason === "no-return")
      assert.throws(() =>
        validateProximityReading({
          ...invalid,
          positionNed: null,
          quaternion: null,
        }),
      );
  }
  const noReturn = suite({ proximityMaxM: 5 }).update(state()).proximity;
  assert.equal(noReturn.reason, "no-return");
  assert.equal(noReturn.valid, false);
});
test("JSON preserves observation identity and CSV always has six FRD rows", () => {
  const r = suite().update(state()).proximity,
    identity = { sourceVehicleId: "drone-2", worldTick: 20 };
  const json = JSON.parse(exportProximityData("json", r, identity));
  assert.deepEqual(Object.keys(json), [
    "format",
    "version",
    "sourceVehicleId",
    "worldTick",
    "reading",
  ]);
  assert.equal(json.format, "flight-lab-proximity");
  assert.equal(json.worldTick, 20);
  assert.deepEqual(json.reading, r);
  for (const reading of [r, suite().snapshot().proximity]) {
    const rows = exportProximityData("csv", reading, identity)
      .split("\n")
      .map((v) => v.split(","));
    assert.deepEqual(rows[0], PROXIMITY_CSV_HEADER);
    assert.equal(rows.length, 7);
    rows.slice(1).forEach((row, i) => {
      assert.equal(row.length, rows[0].length);
      assert.equal(row[15], PROXIMITY_AXES[i].id);
    });
  }
  for (const bad of [
    { ...identity, worldTick: -1 },
    { ...identity, worldTick: 0.1 },
    { ...identity, extra: 1 },
    { ...identity, sourceVehicleId: "=formula" },
  ])
    assert.throws(() => exportProximityData("csv", r, bad));
  assert.throws(() => exportProximityData("pcd", r, identity));
  assert.throws(() => exportProximityData("json", { ...r, time: 1 }, identity));
});
