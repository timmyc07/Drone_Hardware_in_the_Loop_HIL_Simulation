import { rayPeerSphere } from "./peer-geometry.js";

export const MAX_INSPECTION_TARGETS = 16;
export const INSPECTION_REASONS = Object.freeze([
  "range",
  "angle",
  "frustum",
  "pixels",
  "speed",
  "occluded",
]);
const TARGET_KEYS = [
  "id",
  "name",
  "obstacleIndex",
  "face",
  "u",
  "v",
  "width",
  "height",
  "minRangeM",
  "maxRangeM",
  "maxAngleDeg",
  "minPixels",
  "maxSpeedMps",
];
const FACES = Object.freeze({
  north: { normal: [1, 0, 0], right: [0, -1, 0], down: [0, 0, 1] },
  south: { normal: [-1, 0, 0], right: [0, 1, 0], down: [0, 0, 1] },
  east: { normal: [0, 1, 0], right: [1, 0, 0], down: [0, 0, 1] },
  west: { normal: [0, -1, 0], right: [-1, 0, 0], down: [0, 0, 1] },
  top: { normal: [0, 0, -1], right: [0, 1, 0], down: [-1, 0, 0] },
});
function plain(value) {
  if (
    !value ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value))
  )
    throw new Error("Inspection requires a plain object");
  return value;
}
function exact(value, keys) {
  plain(value);
  if (
    Reflect.ownKeys(value).length !== keys.length ||
    Reflect.ownKeys(value).some((key) => !keys.includes(key))
  )
    throw new Error("Invalid inspection fields");
}
function number(value, min, max, label) {
  if (!Number.isFinite(value) || value < min || value > max)
    throw new Error(`Invalid inspection ${label}`);
  return value;
}
function integer(value, min, max, label) {
  number(value, min, max, label);
  if (!Number.isInteger(value))
    throw new Error(`Inspection ${label} must be an integer`);
  return value;
}
function dense(value, min, max, label) {
  if (
    !Array.isArray(value) ||
    value.length < min ||
    value.length > max ||
    Reflect.ownKeys(value).length !== value.length + 1 ||
    !Array.from({ length: value.length }, (_, i) =>
      Object.hasOwn(value, i),
    ).every(Boolean)
  )
    throw new Error(`Invalid inspection ${label}`);
  return value;
}
const vector = (value, length, label) =>
  dense(value, length, length, label).map((v) => number(v, -1e9, 1e9, label));
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const difference = (a, b) => a.map((v, i) => v - b[i]);
function checkedObstacles(value) {
  const limits = {
    north: [-500, 500],
    east: [-500, 500],
    width: [0.01, 100],
    depth: [0.01, 100],
    height: [0.01, 500],
  };
  return dense(value, 0, 50, "obstacles").map((box) => {
    exact(box, Object.keys(limits));
    return Object.fromEntries(
      Object.entries(limits).map(([key, bounds]) => [
        key,
        number(box[key], ...bounds, `obstacle ${key}`),
      ]),
    );
  });
}
function faceGeometry(face, box) {
  const basis = FACES[face];
  const center = [box.north, box.east, -box.height / 2];
  if (face === "north" || face === "south")
    center[0] += (basis.normal[0] * box.width) / 2;
  else if (face === "east" || face === "west")
    center[1] += (basis.normal[1] * box.depth) / 2;
  else center[2] = -box.height;
  return {
    ...basis,
    center,
    width: face === "east" || face === "west" ? box.width : box.depth,
    height: face === "top" ? box.width : box.height,
  };
}
function checkedTarget(value, obstacles) {
  exact(value, TARGET_KEYS);
  if (
    typeof value.id !== "string" ||
    !/^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(value.id)
  )
    throw new Error("Invalid inspection target ID");
  if (
    typeof value.name !== "string" ||
    !value.name.trim() ||
    value.name.length > 40 ||
    /[\u0000-\u001f\u007f]/.test(value.name)
  )
    throw new Error("Invalid inspection target name");
  const obstacleIndex = integer(
    value.obstacleIndex,
    0,
    obstacles.length - 1,
    "obstacle index",
  );
  if (typeof value.face !== "string" || !Object.hasOwn(FACES, value.face))
    throw new Error("Invalid inspection face");
  const result = {
    id: value.id,
    name: value.name,
    obstacleIndex,
    face: value.face,
  };
  for (const [key, min, max] of [
    ["u", -100, 100],
    ["v", -100, 100],
    ["width", 0.05, 100],
    ["height", 0.05, 100],
    ["minRangeM", 0.1, 200],
    ["maxRangeM", 0.2, 500],
    ["maxAngleDeg", 0, 89],
    ["minPixels", 1, 640],
    ["maxSpeedMps", 0, 100],
  ])
    result[key] = number(value[key], min, max, key);
  if (result.minRangeM >= result.maxRangeM)
    throw new Error("Inspection range minimum must be below maximum");
  const face = faceGeometry(result.face, obstacles[obstacleIndex]);
  if (
    Math.abs(result.u) + result.width / 2 > face.width / 2 ||
    Math.abs(result.v) + result.height / 2 > face.height / 2
  )
    throw new Error("Inspection panel must fit its host face");
  return result;
}
export function validateInspectionTargets(value, obstacles) {
  const boxes = checkedObstacles(obstacles),
    ids = new Set();
  return dense(value, 0, MAX_INSPECTION_TARGETS, "targets").map((raw) => {
    const target = checkedTarget(raw, boxes);
    if (ids.has(target.id))
      throw new Error("Inspection target IDs must be unique");
    ids.add(target.id);
    return target;
  });
}
function panel(target, obstacles) {
  const face = faceGeometry(target.face, obstacles[target.obstacleIndex]);
  const centerNed = face.center.map(
    (v, i) => v + target.u * face.right[i] + target.v * face.down[i],
  );
  return {
    centerNed,
    normalNed: [...face.normal],
    rightNed: [...face.right],
    downNed: [...face.down],
    cornersNed: [
      [-1, -1],
      [1, -1],
      [1, 1],
      [-1, 1],
    ].map(([u, v]) =>
      centerNed.map(
        (x, i) =>
          x +
          ((u * target.width) / 2) * face.right[i] +
          ((v * target.height) / 2) * face.down[i],
      ),
    ),
  };
}
export function inspectionPanel(target, obstacles) {
  const boxes = checkedObstacles(obstacles);
  return panel(checkedTarget(target, boxes), boxes);
}
export function inspectionTemplate(kind = "tower") {
  if (!["tower", "facade"].includes(kind))
    throw new Error("Unknown inspection template");
  const obstacles = [
    {
      north: 20,
      east: 0,
      width: kind === "tower" ? 4 : 2,
      depth: kind === "tower" ? 4 : 18,
      height: 10,
    },
  ];
  const make = (id, name, face, u = 0) => ({
    id,
    name,
    obstacleIndex: 0,
    face,
    u,
    v: 0,
    width: 2,
    height: 2,
    minRangeM: 2,
    maxRangeM: 30,
    maxAngleDeg: 45,
    minPixels: 12,
    maxSpeedMps: 1,
  });
  const targets =
    kind === "tower"
      ? ["south", "east", "north", "west", "top"].map((face) =>
          make(
            `tower-${face}`,
            `塔體 ${{ south: "南側", east: "東側", north: "北側", west: "西側", top: "頂面" }[face]}`,
            face,
          ),
        )
      : [-5, 0, 5].map((u, i) =>
          make(`facade-${i + 1}`, `立面 ${i + 1}`, "south", u),
        );
  return {
    name: kind === "tower" ? "通用塔體巡檢" : "通用立面巡檢",
    obstacles,
    inspectionTargets: validateInspectionTargets(targets, obstacles),
  };
}
function checkedMetadata(metadata) {
  plain(metadata);
  plain(metadata.pose);
  plain(metadata.intrinsics);
  const position = vector(metadata.pose.positionNed, 3, "camera position"),
    rotation = vector(metadata.pose.opticalToNed, 9, "optical rotation");
  const columns = [0, 1, 2].map((i) => [
    rotation[i],
    rotation[i + 3],
    rotation[i + 6],
  ]);
  const det = dot(columns[0], [
    columns[1][1] * columns[2][2] - columns[1][2] * columns[2][1],
    columns[1][2] * columns[2][0] - columns[1][0] * columns[2][2],
    columns[1][0] * columns[2][1] - columns[1][1] * columns[2][0],
  ]);
  if (
    columns.some((a, i) =>
      columns.some((b, j) => Math.abs(dot(a, b) - Number(i === j)) > 1e-5),
    ) ||
    Math.abs(det - 1) > 1e-5
  )
    throw new Error("Inspection optical transform must be a proper rotation");
  const width = integer(metadata.width, 64, 640, "image width"),
    height = integer(metadata.height, 48, 480, "image height");
  const near = number(metadata.nearM, 0.05, 5, "near plane"),
    far = number(metadata.farM, 10, 500, "far plane");
  if (near >= far) throw new Error("Invalid inspection clip planes");
  const { fx, fy, cx, cy } = metadata.intrinsics;
  number(fx, 1e-9, 1e9, "fx");
  number(fy, 1e-9, 1e9, "fy");
  number(cx, 0, width - 1, "cx");
  number(cy, 0, height - 1, "cy");
  return { position, columns, width, height, near, far, fx, fy, cx, cy };
}
function rayBox(origin, direction, box, maximum) {
  const low = [
      box.north - box.width / 2,
      box.east - box.depth / 2,
      -box.height,
    ],
    high = [box.north + box.width / 2, box.east + box.depth / 2, 0];
  let enter = -Infinity,
    exit = Infinity;
  for (let i = 0; i < 3; i++) {
    if (Math.abs(direction[i]) < 1e-12) {
      if (origin[i] < low[i] || origin[i] > high[i]) return null;
    } else {
      const a = (low[i] - origin[i]) / direction[i],
        b = (high[i] - origin[i]) / direction[i];
      enter = Math.max(enter, Math.min(a, b));
      exit = Math.min(exit, Math.max(a, b));
    }
  }
  const distance = Math.max(0, enter);
  return exit >= distance && distance <= maximum ? distance : null;
}
export function assessInspection(
  target,
  obstacles,
  terrain,
  metadata,
  peers = [],
  sourceVelocityNed = [0, 0, 0],
) {
  const boxes = checkedObstacles(obstacles),
    checked = checkedTarget(target, boxes),
    geometry = panel(checked, boxes),
    camera = checkedMetadata(metadata);
  if (!terrain || typeof terrain.raycast !== "function")
    throw new Error("Inspection requires compiled terrain");
  if (
    terrain.data?.kind === "dem" &&
    (typeof terrain.contains !== "function" ||
      [camera.position, ...geometry.cornersNed].some(
        (point) => !terrain.contains(point[0], point[1]),
      ))
  )
    throw new Error("Inspection camera and panel must be covered by the DEM");
  const velocity = vector(sourceVelocityNed, 3, "source velocity");
  dense(peers, 0, 8, "peers").forEach((peer) => {
    plain(peer);
    vector(peer.position, 3, "peer position");
    number(peer.radius, 0.001, 1000, "peer radius");
  });
  const offset = difference(camera.position, geometry.centerNed),
    rangeM = Math.hypot(...offset),
    speedMps = Math.hypot(...velocity);
  const angleDeg =
    rangeM === 0
      ? null
      : (Math.acos(
          Math.max(-1, Math.min(1, dot(geometry.normalNed, offset) / rangeM)),
        ) *
          180) /
        Math.PI;
  const projected = geometry.cornersNed.map((point) => {
    const delta = difference(point, camera.position),
      local = camera.columns.map((axis) => dot(axis, delta));
    return {
      x: (camera.fx * local[0]) / local[2] + camera.cx,
      y: (camera.fy * local[1]) / local[2] + camera.cy,
      z: local[2],
    };
  });
  const inFrustum = projected.every(
    (p) =>
      p.z >= camera.near &&
      p.z <= camera.far &&
      p.x >= 0 &&
      p.x <= camera.width - 1 &&
      p.y >= 0 &&
      p.y <= camera.height - 1,
  );
  let minPixels = projected.every((p) => p.z > 0)
    ? Math.min(
        ...projected.map((p, i) =>
          Math.hypot(
            p.x - projected[(i + 1) % 4].x,
            p.y - projected[(i + 1) % 4].y,
          ),
        ),
      )
    : null;
  if (!Number.isFinite(minPixels)) minPixels = null;
  let visibleSamples = 0;
  // These 25 specified points are sampled visibility, not a guarantee for every panel pixel.
  for (let row = 0; row < 5; row++)
    for (let column = 0; column < 5; column++) {
      const point = geometry.centerNed.map(
        (v, i) =>
          v +
          (column / 4 - 0.5) * checked.width * geometry.rightNed[i] +
          (row / 4 - 0.5) * checked.height * geometry.downNed[i],
      );
      const delta = difference(point, camera.position),
        distance = Math.hypot(...delta);
      if (distance === 0) continue;
      const direction = delta.map((v) => v / distance),
        tolerance = 1e-6 * Math.max(1, distance),
        maximum = distance + tolerance;
      const host = rayBox(
        camera.position,
        direction,
        boxes[checked.obstacleIndex],
        maximum,
      );
      if (host === null || Math.abs(host - distance) > tolerance) continue;
      if (terrain.raycast(camera.position, direction, 0, maximum) !== null)
        continue;
      if (
        boxes.some(
          (box, i) =>
            i !== checked.obstacleIndex &&
            rayBox(camera.position, direction, box, maximum) !== null,
        )
      )
        continue;
      if (
        peers.some(
          (peer) =>
            rayPeerSphere(camera.position, direction, peer, 0, maximum) !==
            null,
        )
      )
        continue;
      visibleSamples++;
    }
  const checks = {
    range: rangeM >= checked.minRangeM && rangeM <= checked.maxRangeM,
    angle: angleDeg !== null && angleDeg <= checked.maxAngleDeg,
    frustum: inFrustum,
    pixels: minPixels !== null && minPixels >= checked.minPixels,
    speed: speedMps <= checked.maxSpeedMps,
    occluded: visibleSamples === 25,
  };
  const reasons = INSPECTION_REASONS.filter((reason) => !checks[reason]);
  return {
    targetId: checked.id,
    passed: reasons.length === 0,
    reasons,
    rangeM,
    angleDeg,
    minPixels,
    speedMps,
    visibleSamples,
    totalSamples: 25,
  };
}
