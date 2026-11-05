import test from "node:test";
import assert from "node:assert/strict";
import * as THREE from "three";
import { obstacleVisualPlacement } from "../scene.js";

test("elevated bridge deck leaves air below and spans its NED footprint", () => {
  const placement = obstacleVisualPlacement({
    north: 20,
    east: 0,
    width: 8,
    depth: 18,
    baseAltitude: 8,
    height: 1,
  });
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(...placement.size),
    new THREE.MeshBasicMaterial(),
  );
  mesh.position.set(...placement.center);
  mesh.updateMatrixWorld();
  const bounds = new THREE.Box3().setFromObject(mesh);
  assert.deepEqual(bounds.min.toArray(), [-9, 8, -24]);
  assert.deepEqual(bounds.max.toArray(), [9, 9, -16]);
  assert.equal(
    new THREE.Ray(
      new THREE.Vector3(0, 5, 0),
      new THREE.Vector3(0, 0, -1),
    ).intersectBox(bounds, new THREE.Vector3()),
    null,
  );
  assert.notEqual(
    new THREE.Ray(
      new THREE.Vector3(0, 8.5, 0),
      new THREE.Vector3(0, 0, -1),
    ).intersectBox(bounds, new THREE.Vector3()),
    null,
  );
  mesh.geometry.dispose();
  mesh.material.dispose();
});
