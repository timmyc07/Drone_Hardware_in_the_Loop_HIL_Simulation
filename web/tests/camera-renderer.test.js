import test from "node:test";
import assert from "node:assert/strict";
import * as THREE from "three";
import { createCameraRenderer } from "../camera-renderer.js";
import { validateCameraFrame } from "../camera.js";

function fixture({ defer = false, failPass = -1 } = {}) {
  let target = null,
    clearColor = new THREE.Color("#123456"),
    clearAlpha = 0.4;
  let viewport = new THREE.Vector4(1, 2, 100, 80),
    scissor = new THREE.Vector4(2, 3, 70, 60),
    scissorTest = true;
  const targets = [],
    passes = [],
    resolvers = [];
  let lost = false;
  const scene = new THREE.Scene();
  scene.background = new THREE.Color("#456789");
  scene.fog = new THREE.Fog("#456789", 10, 100);
  const object = (name, id, points = false) => {
    const mesh = points
      ? new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial())
      : new THREE.Mesh(new THREE.BoxGeometry(), new THREE.MeshBasicMaterial());
    mesh.name = name;
    if (id) mesh.userData.cameraClass = id;
    scene.add(mesh);
    return mesh;
  };
  const ground = object("ground", 1),
    obstacle = object("obstacle", 5),
    gate = object("gate", 6),
    ownship = object("aircraft", 0),
    annotation = object("marker", 0),
    rain = object("rain", 0, true);
  rain.userData.cameraRgbOnly = true;
  const ownGroup = new THREE.Group();
  ownGroup.userData.cameraExclude = true;
  scene.add(ownGroup);
  ownGroup.add(ownship);
  ownship.userData.cameraClass = 5;
  ownship.userData.cameraRgbOnly = true;
  const renderer = {
    getContext: () => ({ isContextLost: () => lost }),
    domElement: new EventTarget(),
    autoClear: false,
    autoClearColor: false,
    autoClearDepth: false,
    autoClearStencil: false,
    outputColorSpace: THREE.LinearSRGBColorSpace,
    toneMapping: THREE.ACESFilmicToneMapping,
    toneMappingExposure: 0.7,
    clippingPlanes: [new THREE.Plane(new THREE.Vector3(1, 0, 0), 0)],
    localClippingEnabled: true,
    xr: { enabled: true },
    debug: { checkShaderErrors: false, onShaderError: null },
    getRenderTarget: () => target,
    getActiveCubeFace: () => 0,
    getActiveMipmapLevel: () => 0,
    getViewport: (v) => v.copy(viewport),
    getScissor: (v) => v.copy(scissor),
    getScissorTest: () => scissorTest,
    getClearColor: (c) => c.copy(clearColor),
    getClearAlpha: () => clearAlpha,
    setViewport: (v) => {
      viewport = v.clone();
    },
    setScissor: (v) => {
      scissor = v.clone();
    },
    setScissorTest: (v) => {
      scissorTest = v;
    },
    setClearColor: (color, alpha) => {
      clearColor =
        color instanceof THREE.Color ? color.clone() : new THREE.Color(color);
      clearAlpha = alpha;
    },
    setRenderTarget: (value) => {
      target = value;
      if (value && !targets.includes(value)) targets.push(value);
    },
    clear() {},
    render(world, camera) {
      const pass = targets.indexOf(target),
        visible = [];
      world.traverseVisible((item) => {
        if (item.isMesh || item.isPoints) visible.push(item.name);
      });
      passes.push({
        pass,
        visible,
        position: camera.position.clone(),
        direction: camera.getWorldDirection(new THREE.Vector3()),
        materials: [ground.material, obstacle.material, gate.material],
      });
      if (pass === failPass) throw new Error("injected render failure");
    },
    readRenderTargetPixelsAsync(renderTarget, x, y, width, height, buffer) {
      const pass = targets.indexOf(renderTarget);
      for (let row = 0; row < height; row++)
        for (let col = 0; col < width; col++) {
          const offset = (row * width + col) * 4;
          if (pass === 0) buffer.set([row, col, 77, 255], offset);
          else if (pass === 1) {
            const mm = (row + 1) * 1000;
            buffer.set(
              [
                Math.floor(mm / 65536),
                Math.floor(mm / 256) % 256,
                mm % 256,
                255,
              ],
              offset,
            );
          } else buffer.set([(col % 6) + 1, 0, 0, 255], offset);
        }
      return defer
        ? new Promise((resolve) => resolvers.push(() => resolve(buffer)))
        : Promise.resolve(buffer);
    },
  };
  const original = {
    background: scene.background,
    fog: scene.fog,
    groundMaterial: ground.material,
    clipping: renderer.clippingPlanes,
  };
  return {
    renderer,
    scene,
    ground,
    obstacle,
    gate,
    ownship,
    annotation,
    rain,
    passes,
    targets,
    original,
    resolvers,
    lose() {
      lost = true;
    },
  };
}
const pose = {
  time: 1,
  airframe: "quadrotor",
  position: [1, 2, -3],
  quaternion: [Math.SQRT1_2, 0, 0, Math.SQRT1_2],
};
const config = { width: 64, height: 48, pitchDeg: 0 };

test("capture uses physical camera pose, excludes annotations/ownship, restores state synchronously and flips GPU rows", async () => {
  const f = fixture({ defer: true }),
    capture = createCameraRenderer(f.renderer, f.scene);
  const pending = capture.captureCamera(pose, config);
  assert.equal(f.passes.length, 3);
  assert.deepEqual(f.passes[0].visible, ["ground", "obstacle", "gate", "rain"]);
  assert.deepEqual(f.passes[1].visible, ["ground", "obstacle", "gate"]);
  assert.deepEqual(f.passes[2].visible, ["ground", "obstacle", "gate"]);
  assert.deepEqual(f.passes[0].position.toArray(), [2, 3, -1]);
  assert.ok(
    f.passes[0].direction.distanceTo(new THREE.Vector3(1, 0, 0)) < 1e-12,
  );
  assert.equal(f.ground.material, f.original.groundMaterial);
  assert.equal(f.ownship.visible, true);
  assert.equal(f.annotation.visible, true);
  assert.equal(f.scene.background, f.original.background);
  assert.equal(f.scene.fog, f.original.fog);
  assert.equal(f.renderer.getRenderTarget(), null);
  assert.equal(f.renderer.xr.enabled, true);
  assert.equal(f.renderer.autoClear, false);
  assert.equal(f.renderer.clippingPlanes, f.original.clipping);
  assert.equal(f.renderer.localClippingEnabled, true);
  assert.equal(f.renderer.toneMapping, THREE.ACESFilmicToneMapping);
  assert.equal(f.renderer.outputColorSpace, THREE.LinearSRGBColorSpace);
  assert.deepEqual(
    f.renderer.getViewport(new THREE.Vector4()).toArray(),
    [1, 2, 100, 80],
  );
  assert.equal(f.renderer.getScissorTest(), true);
  assert.equal(f.targets[0].texture.colorSpace, THREE.SRGBColorSpace);
  assert.equal(f.targets[1].texture.colorSpace, THREE.NoColorSpace);
  assert.ok(f.targets.every((t) => t.samples === 0));
  await assert.rejects(
    capture.captureCamera(pose, config),
    /already in progress/,
  );
  f.resolvers.forEach((resolve) => resolve());
  const frame = await pending;
  validateCameraFrame(frame);
  assert.deepEqual([...frame.rgbRgba.slice(0, 4)], [47, 0, 77, 255]);
  assert.equal(frame.depthM[0], 48);
  assert.equal(frame.depthM[47 * 64], 1);
  assert.equal(frame.classes[0], 1);
  let disposed = 0;
  f.targets.forEach((t) => t.addEventListener("dispose", () => disposed++));
  capture.dispose();
  assert.equal(disposed, 3);
  await assert.rejects(capture.captureCamera(pose, config), /disposed/);
});

test("render exceptions restore every mesh/material and release busy; context loss fails before rendering", async () => {
  const f = fixture({ failPass: 1 }),
    capture = createCameraRenderer(f.renderer, f.scene);
  await assert.rejects(
    capture.captureCamera(pose, config),
    /injected render failure/,
  );
  assert.equal(f.ground.material, f.original.groundMaterial);
  assert.equal(f.ownship.visible, true);
  assert.equal(f.rain.visible, true);
  assert.equal(f.scene.background, f.original.background);
  assert.equal(f.scene.fog, f.original.fog);
  assert.equal(f.renderer.getRenderTarget(), null);
  f.lose();
  const count = f.passes.length;
  await assert.rejects(capture.captureCamera(pose, config), /context lost/);
  assert.equal(f.passes.length, count);
  capture.dispose();
});
