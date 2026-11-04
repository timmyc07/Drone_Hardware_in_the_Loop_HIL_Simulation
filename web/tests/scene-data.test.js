import test from "node:test";
import assert from "node:assert/strict";
import {
  DEFAULT_SCENE,
  validateScene,
  sceneToJson,
  parseSceneJson,
  SCENE_MAX_IMPORT_BYTES,
} from "../scene-data.js";
const gate = { north: 5, east: 0, altitude: 2, yaw: 0, radius: 1 };
test("scene artifacts round trip all geometry with isolated clones and frozen defaults", () => {
  const scene = {
    ...DEFAULT_SCENE,
    name: "自訂場景",
    customGates: [gate],
    obstacles: [{ north: 8, east: 0, width: 2, depth: 2, height: 4 }],
  };
  const parsed = parseSceneJson(sceneToJson(scene));
  assert.deepEqual(parsed, scene);
  parsed.customGates[0].north = 99;
  assert.equal(gate.north, 5);
  assert.ok(Object.isFrozen(DEFAULT_SCENE.environment));
  assert.ok(Object.isFrozen(DEFAULT_SCENE.customGates));
  assert.equal(Object.hasOwn(validateScene(scene), "training"), false);
});
test("scene boundary rejects unknown, missing, sparse and invalid geometry and artifact headers", () => {
  for (const scene of [
    null,
    [],
    { ...DEFAULT_SCENE, extra: 1 },
    { ...DEFAULT_SCENE, name: " " },
    { ...DEFAULT_SCENE, name: "x".repeat(81) },
    {
      ...DEFAULT_SCENE,
      environment: { ...DEFAULT_SCENE.environment, extra: 1 },
    },
    { ...DEFAULT_SCENE, training: "custom" },
    { ...DEFAULT_SCENE, customGates: Array(1) },
    { ...DEFAULT_SCENE, customGates: Array(51).fill(gate) },
    { ...DEFAULT_SCENE, customGates: [{ ...gate, radius: 0.2 }] },
    { ...DEFAULT_SCENE, customGates: [{ ...gate, yaw: 4 }] },
    { ...DEFAULT_SCENE, obstacles: Array(1) },
    {
      ...DEFAULT_SCENE,
      obstacles: [{ north: 0, east: 0, width: -1, depth: 1, height: 1 }],
    },
  ])
    assert.throws(() => validateScene(scene));
  const missing = { ...DEFAULT_SCENE };
  delete missing.customGates;
  assert.throws(() => validateScene(missing));
  assert.throws(() => validateScene({ ...DEFAULT_SCENE, training: "hover" }));
  const artifact = JSON.parse(sceneToJson(DEFAULT_SCENE));
  for (const patch of [
    { version: 1 },
    { version: 2 },
    { frame: "ENU" },
    { units: "cm" },
    { format: "other" },
    { extra: 1 },
  ])
    assert.throws(() =>
      parseSceneJson(JSON.stringify({ ...artifact, ...patch })),
    );
  assert.throws(() => parseSceneJson(" ".repeat(SCENE_MAX_IMPORT_BYTES + 1)));
  assert.throws(() => parseSceneJson("中".repeat(SCENE_MAX_IMPORT_BYTES / 2)));
});

test("DEM scene round trips its full grid and provenance and enforces covered zero home", () => {
  const geography = {
    origin: {
      latitudeDeg: 25,
      longitudeDeg: 121,
      altitudeM: 100,
      verticalDatum: "orthometric",
    },
    source: {
      name: "terrain.tif",
      sha256: "a".repeat(64),
      crs: "EPSG:4326",
      width: 256,
      height: 256,
      band: 1,
      pixelInterpretation: "area",
      heightUnit: "m",
      scale: 1,
      offset: 0,
      resampling: "bilinear",
      window: [0, 0, 256, 256],
    },
  };
  const terrain = {
    kind: "dem",
    north: -256,
    east: -256,
    rows: 129,
    cols: 129,
    cellSize: 4,
    heights: Array.from(
      { length: 129 * 129 },
      (_, i) => ((i % 129) - 64) * 0.375,
    ),
  };
  const scene = { ...DEFAULT_SCENE, terrain, geography };
  const text = sceneToJson(scene);
  assert.equal(JSON.parse(text).version, 4);
  assert.deepEqual(parseSceneJson(text), scene);
  assert.ok(new TextEncoder().encode(text).length <= SCENE_MAX_IMPORT_BYTES);
  for (const invalid of [
    { ...scene, geography: null },
    { ...DEFAULT_SCENE, geography },
    {
      ...scene,
      terrain: { ...terrain, heights: terrain.heights.map((h) => h + 1) },
    },
    { ...scene, terrain: { ...terrain, north: 1, cellSize: 1 } },
    { ...scene, training: "runway" },
  ])
    assert.throws(() => validateScene(invalid));
  const missing = { ...scene };
  delete missing.geography;
  assert.throws(() => validateScene(missing));
});
