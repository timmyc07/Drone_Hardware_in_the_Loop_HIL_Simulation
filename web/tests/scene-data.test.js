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
    training: "custom",
    customGates: [gate],
    obstacles: [{ north: 8, east: 0, width: 2, depth: 2, height: 4 }],
  };
  const parsed = parseSceneJson(sceneToJson(scene));
  assert.deepEqual(parsed, scene);
  parsed.customGates[0].north = 99;
  assert.equal(gate.north, 5);
  assert.ok(Object.isFrozen(DEFAULT_SCENE.environment));
  assert.ok(Object.isFrozen(DEFAULT_SCENE.customGates));
  for (const frame of ["quadrotor", "fixedwing", "vtol"])
    assert.equal(validateScene(scene, frame).training, "custom");
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
  assert.throws(() =>
    validateScene({ ...DEFAULT_SCENE, training: "hover" }, "fixedwing"),
  );
  const artifact = JSON.parse(sceneToJson(DEFAULT_SCENE));
  for (const patch of [
    { version: 1 },
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
