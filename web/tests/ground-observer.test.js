import test from "node:test";
import assert from "node:assert/strict";
import { PerspectiveCamera, Vector3 } from "three";
import { createTerrain } from "../terrain.js";
import {
  DEFAULT_GROUND_OBSERVER,
  validateGroundObserver,
  groundObserverEye,
  applyGroundObserverCamera,
} from "../ground-observer.js";

const config = (values = {}) => ({ ...DEFAULT_GROUND_OBSERVER, ...values });
const dem = createTerrain({
  kind: "dem",
  north: -10,
  east: -10,
  cellSize: 10,
  rows: 2,
  cols: 2,
  heights: [20, 30, 40, 50],
});

test("ground observer requires exactly four canonical finite settings and copies them", () => {
  const input = config();
  const result = validateGroundObserver(input);
  assert.deepEqual(result, input);
  result.north = 30;
  assert.equal(input.north, -8);
  assert.ok(Object.isFrozen(DEFAULT_GROUND_OBSERVER));
  assert.deepEqual(validateGroundObserver(Object.assign(Object.create(null), input)), input);
  for (const value of [
    undefined,
    null,
    [],
    {},
    Object.create(input),
    { ...input, extra: 1 },
    { ...input, [Symbol("extra")]: 1 },
  ])
    assert.throws(() => validateGroundObserver(value));
  for (const key of Object.keys(input)) {
    const missing = config();
    delete missing[key];
    assert.throws(() => validateGroundObserver(missing));
    for (const value of [NaN, Infinity, -Infinity, "1", undefined, null])
      assert.throws(() => validateGroundObserver(config({ [key]: value })));
  }
});

test("ground observer accepts inclusive bounds and rejects every out-of-range field", () => {
  for (const [key, lower, upper] of [
    ["north", -500, 500],
    ["east", -500, 500],
    ["eyeHeightM", 0.5, 10],
    ["fovDeg", 15, 100],
  ]) {
    assert.equal(validateGroundObserver(config({ [key]: lower }))[key], lower);
    assert.equal(validateGroundObserver(config({ [key]: upper }))[key], upper);
    assert.throws(() => validateGroundObserver(config({ [key]: lower - 0.001 })));
    assert.throws(() => validateGroundObserver(config({ [key]: upper + 0.001 })));
  }
});

test("observer eye uses terrain datum plus eye height and requires actual DEM coverage", () => {
  assert.deepEqual(groundObserverEye(config(), createTerrain()), [-8, -8, -1.6]);
  assert.deepEqual(groundObserverEye(config(), dem), [-8, -8, -27.6]);
  assert.deepEqual(groundObserverEye(config({ north: 0, east: 0 }), dem), [0, 0, -51.6]);
  for (const values of [
    { north: -10.001 },
    { east: -10.001 },
    { north: 0.001 },
    { east: 0.001 },
  ])
    assert.throws(() => groundObserverEye(config(values), dem), /terrain coverage/);
  const field = createTerrain({
    kind: "heightfield", north: 250, east: 50, cellSize: 10,
    rows: 3, cols: 3, heights: [0, 0, 0, 0, 10, 0, 0, 0, 0],
  });
  assert.deepEqual(groundObserverEye(config({ north: 255, east: 55 }), field), [255, 55, -6.6]);
  assert.deepEqual(groundObserverEye(config(), field), [-8, -8, -1.6]);
});

test("moving or selecting another target changes gaze while the observer eye remains fixed", () => {
  const camera = new PerspectiveCamera(42, 1, 0.05, 2000);
  const eye = groundObserverEye(config(), dem);
  const expectedEye = [-8, 27.6, 8];
  for (const positionNed of [[0, 0, -3], [20, 5, -10], [-50, 30, -40]]) {
    const [north, east, down] = positionNed;
    const target = new Vector3(east, -down, -north);
    const before = target.clone();
    applyGroundObserverCamera(camera, eye, target, 50);
    assert.deepEqual(camera.position.toArray(), expectedEye);
    assert.ok(camera.getWorldDirection(new Vector3()).distanceTo(
      target.clone().sub(camera.position).normalize(),
    ) < 1e-12);
    assert.deepEqual(target, before);
  }
  assert.deepEqual(eye, [-8, -8, -27.6]);
});

test("optical zoom changes projection without moving the eye or following the target", () => {
  const camera = new PerspectiveCamera(42, 1, 0.05, 2000);
  const eye = groundObserverEye(config(), createTerrain());
  const target = new Vector3(0, 10, 0);
  applyGroundObserverCamera(camera, eye, target, 100);
  const position = camera.position.toArray();
  const orientation = camera.quaternion.toArray();
  const wideProjection = camera.projectionMatrix.elements[0];
  applyGroundObserverCamera(camera, eye, target, 15);
  assert.equal(camera.fov, 15);
  assert.ok(camera.projectionMatrix.elements[0] > wideProjection);
  assert.deepEqual(camera.position.toArray(), position);
  assert.deepEqual(camera.quaternion.toArray(), orientation);
});

test("coincident, directly overhead and directly below targets retain finite camera orientation", () => {
  const camera = new PerspectiveCamera(42, 1, 0.05, 2000);
  const eye = groundObserverEye(config(), createTerrain());
  for (const y of [1.6, 101.6, -98.4]) {
    applyGroundObserverCamera(camera, eye, new Vector3(-8, y, 8), 50);
    camera.updateMatrixWorld();
    assert.ok(camera.quaternion.toArray().every(Number.isFinite));
    assert.ok(camera.matrixWorld.elements.every(Number.isFinite));
    assert.ok(Math.abs(camera.quaternion.length() - 1) < 1e-12);
    assert.deepEqual(camera.position.toArray(), [-8, 1.6, 8]);
  }
});
