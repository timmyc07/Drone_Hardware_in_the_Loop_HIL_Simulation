import { gimbalRotation, validateGimbalState } from "./gimbal.js";
import { validateGeography } from "./geography.js";
import { AIRFRAMES } from "./rotor-geometry.js";

export const DEFAULT_CAMERA_CONFIG = Object.freeze({
  width: 320,
  height: 240,
  horizontalFovDeg: 90,
  nearM: 0.1,
  farM: 200,
});
export const CAMERA_DATASET_MAX_FRAMES = 30;
export const CAMERA_DATASET_MAX_BYTES = 24 * 1024 * 1024;
export const CAMERA_DATASET_MAX_IMPORT_BYTES = 40 * 1024 * 1024;
export const CAMERA_CLASSES = Object.freeze(
  [
    ["sky", "天空或無有效回波", [0, 0, 0]],
    ["ground", "地面", [94, 130, 90]],
    ["runway", "跑道", [150, 150, 150]],
    ["vegetation", "植被", [30, 190, 80]],
    ["structure", "結構", [80, 140, 230]],
    ["obstacle", "障礙", [245, 155, 60]],
    ["gate", "訓練門", [210, 70, 220]],
    ["aircraft", "其他飛行器", [90, 180, 245]],
  ].map(([name, label, color], id) =>
    Object.freeze({ id, name, label, color: Object.freeze(color) }),
  ),
);
const encoding = Object.freeze({
  rgbRgba: "base64-rgba8",
  depthM: "base64-float32-le-meters",
  classes: "base64-uint8",
});
const classLegend = CAMERA_CLASSES.map(({ id, name, color }) => ({
  id,
  name,
  color,
}));
const same = (value, expected, tolerance = 0) => {
  if (
    typeof expected === "number" &&
    typeof value === "number" &&
    Number.isFinite(value)
  )
    return (
      Math.abs(value - expected) <= tolerance * Math.max(1, Math.abs(expected))
    );
  if (expected === null || typeof expected !== "object")
    return value === expected;
  if (Array.isArray(expected))
    return (
      Array.isArray(value) &&
      value.length === expected.length &&
      expected.every((item, i) => same(value[i], item, tolerance))
    );
  return (
    value &&
    typeof value === "object" &&
    !Array.isArray(value) &&
    Object.keys(value).length === Object.keys(expected).length &&
    Object.keys(expected).every(
      (key) =>
        Object.hasOwn(value, key) && same(value[key], expected[key], tolerance),
    )
  );
};
const plain = (value) => {
  if (
    !value ||
    typeof value !== "object" ||
    Array.isArray(value) ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value))
  )
    throw new Error("Camera data must be a plain object");
  return value;
};
const finite = (value, min, max) => {
  if (!Number.isFinite(value) || value < min || value > max)
    throw new Error("Invalid camera number");
  return value;
};
const vector = (value, length) => {
  if (!Array.isArray(value) || value.length !== length)
    throw new Error("Invalid camera vector");
  return Array.from(value, (v) => finite(v, -1e9, 1e9));
};
export function validateCameraConfig(value = {}) {
  plain(value);
  if (
    Object.keys(value).some((key) => !Object.hasOwn(DEFAULT_CAMERA_CONFIG, key))
  )
    throw new Error("Unknown camera setting");
  const result = { ...DEFAULT_CAMERA_CONFIG, ...value };
  for (const [key, min, max] of [
    ["width", 64, 640],
    ["height", 48, 480],
    ["horizontalFovDeg", 30, 140],
    ["nearM", 0.05, 5],
    ["farM", 10, 500],
  ])
    finite(result[key], min, max);
  if (
    !Number.isInteger(result.width) ||
    !Number.isInteger(result.height) ||
    result.farM <= result.nearM
  )
    throw new Error("Invalid camera resolution or clip planes");
  return result;
}
const multiply = (a, b) =>
  Array.from({ length: 9 }, (_, index) => {
    const row = Math.floor(index / 3),
      col = index % 3;
    return (
      a[row * 3] * b[col] +
      a[row * 3 + 1] * b[col + 3] +
      a[row * 3 + 2] * b[col + 6]
    );
  });
export function cameraMetadata(
  snapshot,
  input = DEFAULT_CAMERA_CONFIG,
  geography = null,
  identity,
) {
  plain(snapshot);
  plain(identity);
  if (
    typeof identity.sourceVehicleId !== "string" ||
    !/^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(identity.sourceVehicleId) ||
    !Number.isSafeInteger(identity.worldTick) ||
    identity.worldTick < 0 ||
    Object.keys(identity).some(
      (key) => !["sourceVehicleId", "worldTick"].includes(key),
    )
  )
    throw new Error("Invalid camera source identity");
  const config = validateCameraConfig(input),
    { width, height, horizontalFovDeg, nearM, farM } = config;
  const positionNed = vector(snapshot.position, 3),
    bodyQuaternionWxyz = vector(snapshot.quaternion, 4);
  if (Math.abs(Math.hypot(...bodyQuaternionWxyz) - 1) > 1e-5)
    throw new Error("Camera pose requires a unit quaternion");
  if (!AIRFRAMES.includes(snapshot.airframe))
    throw new Error("Invalid camera airframe");
  const [w, x, y, z] = bodyQuaternionWxyz;
  const body = [
    1 - 2 * (y * y + z * z),
    2 * (x * y - w * z),
    2 * (x * z + w * y),
    2 * (x * y + w * z),
    1 - 2 * (x * x + z * z),
    2 * (y * z - w * x),
    2 * (x * z - w * y),
    2 * (y * z + w * x),
    1 - 2 * (x * x + y * y),
  ];
  const gimbal = validateGimbalState(snapshot.gimbal, bodyQuaternionWxyz);
  // One acquired joint pose drives all optical outputs; a requested target is not a pose.
  const mount = multiply(
    gimbalRotation(gimbal.actual),
    [0, 0, 1, 1, 0, 0, 0, 1, 0],
  );
  const fx = width / (2 * Math.tan((horizontalFovDeg * Math.PI) / 360));
  return {
    version: 4,
    sourceVehicleId: identity.sourceVehicleId,
    worldTick: identity.worldTick,
    geography: geography === null ? null : validateGeography(geography),
    time: finite(snapshot.time, 0, 1800.001),
    airframe: snapshot.airframe,
    config,
    gimbal,
    width,
    height,
    nearM,
    farM,
    intrinsics: {
      fx,
      fy: fx,
      cx: (width - 1) / 2,
      cy: (height - 1) / 2,
      horizontalFovDeg,
      verticalFovDeg: (2 * Math.atan(height / (2 * fx)) * 180) / Math.PI,
    },
    pose: {
      positionNed,
      bodyQuaternionWxyz,
      opticalToNed: multiply(body, mount),
    },
    pixelOrder: "top-left-row-major",
    depthDefinition: "camera-z-meters-zero-invalid",
  };
}
function checkFrame(value) {
  plain(value);
  plain(value.pose);
  const metadata = cameraMetadata(
    {
      time: value.time,
      airframe: value.airframe,
      position: value.pose.positionNed,
      quaternion: value.pose.bodyQuaternionWxyz,
      gimbal: value.gimbal,
    },
    value.config,
    value.geography,
    { sourceVehicleId: value.sourceVehicleId, worldTick: value.worldTick },
  );
  for (const key of Object.keys(metadata)) {
    // Trigonometric results can differ by a few ULPs across browser and Node engines.
    if (
      !same(
        value[key],
        metadata[key],
        ["intrinsics", "pose", "gimbal"].includes(key) ? 1e-12 : 0,
      )
    )
      throw new Error(
        "Camera metadata does not match its configuration and pose",
      );
  }
  if (
    Object.keys(value).some(
      (key) =>
        !Object.hasOwn(metadata, key) &&
        !["rgbRgba", "depthM", "classes"].includes(key),
    )
  )
    throw new Error("Unknown camera frame field");
  const count = metadata.width * metadata.height;
  if (
    !(value.rgbRgba instanceof Uint8Array) ||
    value.rgbRgba.length !== count * 4 ||
    !(value.depthM instanceof Float32Array) ||
    value.depthM.length !== count ||
    !(value.classes instanceof Uint8Array) ||
    value.classes.length !== count
  )
    throw new Error("Invalid camera pixel buffers");
  for (let i = 0; i < count; i++) {
    const depth = value.depthM[i],
      id = value.classes[i];
    if (
      !Number.isFinite(depth) ||
      depth < 0 ||
      (depth !== 0 &&
        (depth < metadata.nearM - 0.0011 || depth > metadata.farM + 0.0011)) ||
      id >= CAMERA_CLASSES.length ||
      (depth === 0) !== (id === 0)
    )
      throw new Error("Invalid camera depth or class ID");
  }
  return metadata;
}
export function cameraFrameBytes(frame) {
  return checkFrame(frame).width * frame.height * 9;
}
export function validateCameraFrame(frame) {
  return {
    ...checkFrame(frame),
    rgbRgba: new Uint8Array(frame.rgbRgba),
    depthM: new Float32Array(frame.depthM),
    classes: new Uint8Array(frame.classes),
  };
}
export function cameraPreview(frame, type) {
  checkFrame(frame);
  if (type === "rgb") return new Uint8ClampedArray(frame.rgbRgba);
  if (!["depth", "segmentation"].includes(type))
    throw new Error("Unknown camera preview type");
  const output = new Uint8ClampedArray(frame.width * frame.height * 4);
  for (let i = 0; i < frame.depthM.length; i++) {
    const shade =
      frame.depthM[i] === 0
        ? 0
        : Math.round(
            255 *
              Math.max(
                0,
                Math.min(
                  1,
                  1 -
                    (frame.depthM[i] - frame.nearM) /
                      (frame.farM - frame.nearM),
                ),
              ),
          );
    output.set(
      type === "depth"
        ? [shade, shade, shade, 255]
        : [...CAMERA_CLASSES[frame.classes[i]].color, 255],
      i * 4,
    );
  }
  return output;
}
const toBase64 = (bytes) => {
  let binary = "";
  for (let i = 0; i < bytes.length; i += 32768)
    binary += String.fromCharCode(...bytes.subarray(i, i + 32768));
  return btoa(binary);
};
const fromBase64 = (value, length) => {
  if (
    typeof value !== "string" ||
    value.length !== 4 * Math.ceil(length / 3) ||
    !/^[A-Za-z0-9+/]*={0,2}$/.test(value)
  )
    throw new Error("Invalid camera base64 buffer");
  const binary = atob(value);
  if (binary.length !== length) throw new Error("Invalid camera buffer length");
  return Uint8Array.from(binary, (c) => c.charCodeAt(0));
};
function depthBytes(depth) {
  const bytes = new Uint8Array(depth.length * 4),
    view = new DataView(bytes.buffer);
  depth.forEach((value, i) => view.setFloat32(i * 4, value, true));
  return bytes;
}
export function cameraDatasetToJson(frames) {
  if (!Array.isArray(frames) || frames.length > CAMERA_DATASET_MAX_FRAMES)
    throw new Error("Camera dataset frame limit exceeded");
  let size = 0;
  const encoded = Array.from(frames, (frame) => {
    const metadata = checkFrame(frame);
    size += frame.width * frame.height * 9;
    if (size > CAMERA_DATASET_MAX_BYTES)
      throw new Error("Camera dataset byte limit exceeded");
    return {
      ...metadata,
      rgbRgba: toBase64(frame.rgbRgba),
      depthM: toBase64(depthBytes(frame.depthM)),
      classes: toBase64(frame.classes),
    };
  });
  return JSON.stringify({
    format: "flight-lab-camera-dataset",
    version: 4,
    encoding,
    classes: classLegend,
    frames: encoded,
  });
}
export function parseCameraDataset(text) {
  if (typeof text !== "string" || text.length > CAMERA_DATASET_MAX_IMPORT_BYTES)
    throw new Error("Camera dataset import limit exceeded");
  const data = plain(JSON.parse(text));
  if (
    data.format !== "flight-lab-camera-dataset" ||
    data.version !== 4 ||
    !same(data.encoding, encoding) ||
    !same(data.classes, classLegend) ||
    !Array.isArray(data.frames) ||
    data.frames.length > CAMERA_DATASET_MAX_FRAMES ||
    Object.keys(data).some(
      (key) =>
        !["format", "version", "encoding", "classes", "frames"].includes(key),
    )
  )
    throw new Error("Unsupported camera dataset format");
  let size = 0;
  return Array.from(data.frames, (input) => {
    plain(input);
    plain(input.pose);
    const config = validateCameraConfig(input.config),
      count = config.width * config.height;
    size += count * 9;
    if (size > CAMERA_DATASET_MAX_BYTES)
      throw new Error("Camera dataset byte limit exceeded");
    const bytes = fromBase64(input.depthM, count * 4),
      view = new DataView(bytes.buffer),
      depthM = new Float32Array(count);
    for (let i = 0; i < count; i++) depthM[i] = view.getFloat32(i * 4, true);
    const frame = {
      ...input,
      rgbRgba: fromBase64(input.rgbRgba, count * 4),
      depthM,
      classes: fromBase64(input.classes, count),
    };
    checkFrame(frame);
    return frame;
  });
}
