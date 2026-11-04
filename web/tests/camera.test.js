import test from "node:test";
import assert from "node:assert/strict";
import {
  DEFAULT_CAMERA_CONFIG,
  CAMERA_CLASSES,
  CAMERA_DATASET_MAX_FRAMES,
  CAMERA_DATASET_MAX_BYTES,
  CAMERA_DATASET_MAX_IMPORT_BYTES,
  validateCameraConfig,
  cameraMetadata,
  validateCameraFrame,
  cameraPreview,
  cameraFrameBytes,
  cameraDatasetToJson,
  parseCameraDataset,
} from "../camera.js";

const snapshot = {
  time: 2,
  airframe: "quadrotor",
  position: [10, 20, -2],
  quaternion: [1, 0, 0, 0],
};
const close = (a, b) => assert.ok(Math.abs(a - b) < 1e-10, `${a} != ${b}`);
function frame(config = {}) {
  const metadata = cameraMetadata(snapshot, {
      width: 64,
      height: 48,
      ...config,
    }),
    count = metadata.width * metadata.height;
  const result = {
    ...metadata,
    rgbRgba: new Uint8Array(count * 4),
    depthM: new Float32Array(count),
    classes: new Uint8Array(count),
  };
  result.rgbRgba.set([20, 80, 150, 255], 0);
  result.depthM[0] = 2;
  result.classes[0] = 1;
  return result;
}
test("camera configuration validates strictly without mutating defaults", () => {
  const before = { ...DEFAULT_CAMERA_CONFIG };
  for (const value of [
    null,
    [],
    { width: "320" },
    { height: 48.5 },
    { width: 641 },
    { height: 481 },
    { horizontalFovDeg: Infinity },
    { nearM: 0 },
    { farM: 501 },
    { pitchDeg: 91 },
    { yawDeg: NaN },
    { unknown: 1 },
  ])
    assert.throws(() => validateCameraConfig(value));
  assert.deepEqual(DEFAULT_CAMERA_CONFIG, before);
  assert.deepEqual(validateCameraConfig(), before);
  assert.equal(validateCameraConfig({ pitchDeg: 90 }).pitchDeg, 90);
});
test("pinhole intrinsics use pixel centers and convert horizontal to vertical FOV", () => {
  const result = cameraMetadata(snapshot, {
    width: 320,
    height: 240,
    horizontalFovDeg: 90,
    pitchDeg: 0,
  });
  close(result.intrinsics.fx, 160);
  close(result.intrinsics.fy, 160);
  close(result.intrinsics.cx, 159.5);
  close(result.intrinsics.cy, 119.5);
  close(
    result.intrinsics.verticalFovDeg,
    (2 * Math.atan(0.75) * 180) / Math.PI,
  );
  assert.deepEqual(result.pose.positionNed, [10, 20, -2]);
  result.pose.positionNed[0] = 99;
  assert.equal(snapshot.position[0], 10);
});
test("optical axes, downward pitch and right yaw rotate consistently into NED", () => {
  const base = cameraMetadata(snapshot, { pitchDeg: 0 }).pose.opticalToNed;
  assert.deepEqual(base, [0, 0, 1, 1, 0, 0, 0, 1, 0]);
  const down = cameraMetadata(snapshot, { pitchDeg: 90 }).pose.opticalToNed;
  [0, 0, 1].forEach((value, row) => close(down[row * 3 + 2], value));
  [-1, 0, 0].forEach((value, row) => close(down[row * 3 + 1], value));
  const right = cameraMetadata(snapshot, { pitchDeg: 0, yawDeg: 90 }).pose
    .opticalToNed;
  [0, 1, 0].forEach((value, row) => close(right[row * 3 + 2], value));
  const yawed = cameraMetadata(
    { ...snapshot, quaternion: [Math.SQRT1_2, 0, 0, Math.SQRT1_2] },
    { pitchDeg: 0 },
  ).pose.opticalToNed;
  right.forEach((value, i) => close(yawed[i], value));
  for (const change of [
    { quaternion: [0, 0, 0, 0] },
    { position: [0, Infinity, 0] },
    { time: -1 },
    { airframe: "unknown" },
  ])
    assert.throws(() => cameraMetadata({ ...snapshot, ...change }));
});
test("camera frame validation checks metadata, lengths, finite depth and exact classes", () => {
  const valid = frame(),
    copy = validateCameraFrame(valid);
  assert.equal(cameraFrameBytes(valid), 64 * 48 * 9);
  copy.depthM[0] = 3;
  assert.equal(valid.depthM[0], 2);
  for (const mutate of [
    (f) => (f.intrinsics.fx = 1),
    (f) => (f.pose.opticalToNed[0] = 1),
    (f) => (f.depthM = new Float32Array(1)),
    (f) => (f.rgbRgba = new Uint8ClampedArray(f.rgbRgba)),
    (f) => (f.depthM[0] = NaN),
    (f) => (f.depthM[0] = 501),
    (f) => (f.depthM[0] = 0.01),
    (f) => (f.classes[0] = 7),
    (f) => (f.classes[0] = 0),
    (f) => (f.depthM[0] = 0),
    (f) => (f.extra = "unknown"),
    (f) => (f.pose.extra = 1),
  ]) {
    const input = frame();
    mutate(input);
    assert.throws(() => validateCameraFrame(input));
  }
});
test("depth and class previews preserve invalid pixels and exact class colors", () => {
  const input = frame();
  const rgb = cameraPreview(input, "rgb");
  assert.deepEqual([...rgb.slice(0, 4)], [20, 80, 150, 255]);
  rgb[0] = 99;
  assert.equal(input.rgbRgba[0], 20);
  const depth = cameraPreview(input, "depth");
  assert.ok(depth[0] > 0);
  assert.deepEqual([...depth.slice(4, 8)], [0, 0, 0, 255]);
  const classes = cameraPreview(input, "segmentation");
  assert.deepEqual([...classes.slice(0, 4)], [...CAMERA_CLASSES[1].color, 255]);
  assert.throws(() => cameraPreview(input, "infrared"));
});
test("camera datasets round trip exact byte buffers and little-endian float32 depth", () => {
  const input = [frame(), frame({ pitchDeg: 90 })];
  input[0].depthM[0] = Math.fround(2.345);
  const text = cameraDatasetToJson(input),
    encoded = JSON.parse(text);
  const depth = Uint8Array.from(atob(encoded.frames[0].depthM), (c) =>
    c.charCodeAt(0),
  );
  assert.equal(
    new DataView(depth.buffer).getFloat32(0, true),
    input[0].depthM[0],
  );
  assert.deepEqual(parseCameraDataset(text), input);
  const reordered = {
    ...encoded,
    encoding: Object.fromEntries(Object.entries(encoded.encoding).reverse()),
  };
  reordered.frames[0].config = Object.fromEntries(
    Object.entries(reordered.frames[0].config).reverse(),
  );
  assert.deepEqual(parseCameraDataset(JSON.stringify(reordered)), input);
  reordered.frames[0].intrinsics.verticalFovDeg += 1e-14;
  assert.doesNotThrow(() => parseCameraDataset(JSON.stringify(reordered)));
});
test("camera dataset trust boundaries reject malformed buffers, schemas and limits", () => {
  const valid = JSON.parse(cameraDatasetToJson([frame()]));
  for (const mutate of [
    (d) => (d.version = 2),
    (d) => (d.format = "other"),
    (d) => (d.extra = 1),
    (d) => (d.classes[1].name = "wrong"),
    (d) => (d.frames[0].rgbRgba = "!"),
    (d) => (d.frames[0].depthM = d.frames[0].classes),
    (d) => (d.frames[0].classes = "A".repeat(d.frames[0].classes.length)),
    (d) => (d.frames[0].config.width = 640),
    (d) => (d.frames[0].intrinsics.cx = 0),
    (d) => (d.frames = Array(CAMERA_DATASET_MAX_FRAMES + 1).fill(d.frames[0])),
  ]) {
    const input = structuredClone(valid);
    mutate(input);
    assert.throws(() => parseCameraDataset(JSON.stringify(input)));
  }
  assert.throws(() =>
    cameraDatasetToJson(Array(CAMERA_DATASET_MAX_FRAMES + 1).fill(frame())),
  );
  const large = frame({ width: 640, height: 480 });
  assert.throws(
    () =>
      cameraDatasetToJson(
        Array(
          Math.floor(CAMERA_DATASET_MAX_BYTES / cameraFrameBytes(large)) + 1,
        ).fill(large),
      ),
    /byte limit/,
  );
  assert.throws(
    () => parseCameraDataset(" ".repeat(CAMERA_DATASET_MAX_IMPORT_BYTES + 1)),
    /import limit/,
  );
  assert.deepEqual(parseCameraDataset(cameraDatasetToJson([])), []);
});
