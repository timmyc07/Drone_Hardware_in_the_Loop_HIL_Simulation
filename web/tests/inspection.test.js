import test from "node:test";
import assert from "node:assert/strict";
import {
  MAX_INSPECTION_TARGETS,
  INSPECTION_REASONS,
  validateInspectionTargets,
  inspectionPanel,
  inspectionTemplate,
  assessInspection,
} from "../inspection.js";
import { createTerrain } from "../terrain.js";
import {
  DEFAULT_SCENE,
  SCENE_VERSION,
  validateScene,
  sceneToJson,
  parseSceneJson,
} from "../scene-data.js";

const clone = structuredClone;
const boxes = [{ north: 10, east: 0, width: 2, depth: 4, height: 4 }];
const target = {
  id: "panel-1",
  name: "Panel",
  obstacleIndex: 0,
  face: "south",
  u: 0,
  v: 0,
  width: 1,
  height: 1,
  minRangeM: 1,
  maxRangeM: 30,
  maxAngleDeg: 45,
  minPixels: 8,
  maxSpeedMps: 1,
};
const camera = {
  width: 320,
  height: 240,
  nearM: 0.1,
  farM: 200,
  pose: { positionNed: [0, 0, -2], opticalToNed: [0, 0, 1, 1, 0, 0, 0, 1, 0] },
  intrinsics: { fx: 160, fy: 160, cx: 159.5, cy: 119.5 },
};
const near = (a, b, tolerance = 1e-10) =>
  assert.ok(Math.abs(a - b) < tolerance, `${a} differs from ${b}`);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
function assess(
  t = target,
  obstacles = boxes,
  terrain = createTerrain(),
  metadata = camera,
  peers = [],
  velocity = [0, 0, 0],
) {
  return assessInspection(t, obstacles, terrain, metadata, peers, velocity);
}
function lookingAt(position, point = [9, 0, -2]) {
  const delta = point.map((v, i) => v - position[i]),
    length = Math.hypot(...delta),
    forward = delta.map((v) => v / length);
  const unscaledRight = cross([0, 0, 1], forward),
    rLength = Math.hypot(...unscaledRight),
    right = unscaledRight.map((v) => v / rLength),
    down = cross(forward, right);
  return {
    ...clone(camera),
    pose: {
      positionNed: position,
      opticalToNed: Array.from(
        { length: 9 },
        (_, i) => [right, down, forward][i % 3][Math.floor(i / 3)],
      ),
    },
  };
}

test("inspection targets are strict canonical face attachments with unique IDs and bounded host fitting", () => {
  const canonical = validateInspectionTargets([target], boxes);
  assert.deepEqual(canonical, [target]);
  assert.equal(MAX_INSPECTION_TARGETS, 16);
  canonical[0].name = "Changed";
  assert.equal(target.name, "Panel");
  const mutations = [
    (t) => {
      t.id = "";
    },
    (t) => {
      t.id = "a,b";
    },
    (t) => {
      t.name = " ";
    },
    (t) => {
      t.name = "a\n";
    },
    (t) => {
      t.obstacleIndex = 1;
    },
    (t) => {
      t.obstacleIndex = 0.5;
    },
    (t) => {
      t.face = "bottom";
    },
    (t) => {
      t.face = ["south"];
    },
    (t) => {
      t.width = 0.01;
    },
    (t) => {
      t.height = 5;
    },
    (t) => {
      t.u = 2;
    },
    (t) => {
      t.v = 2;
    },
    (t) => {
      t.minRangeM = t.maxRangeM;
    },
    (t) => {
      t.maxAngleDeg = 90;
    },
    (t) => {
      t.minPixels = 0;
    },
    (t) => {
      t.maxSpeedMps = -1;
    },
    (t) => {
      t.u = NaN;
    },
    (t) => {
      t.extra = true;
    },
    (t) => {
      t[Symbol("extra")] = 1;
    },
    (t) => {
      delete t.name;
    },
  ];
  for (const mutate of mutations) {
    const bad = clone(target);
    mutate(bad);
    assert.throws(() => validateInspectionTargets([bad], boxes));
  }
  for (const bad of [null, Array(1), Array(17).fill(target), [target, target]])
    assert.throws(() => validateInspectionTargets(bad, boxes));
  assert.throws(() => validateInspectionTargets([target], []));
  assert.throws(() => validateInspectionTargets([target], Array(1)));
  assert.throws(() =>
    validateInspectionTargets([target], [{ ...boxes[0], width: -1 }]),
  );
  const edge = { ...target, u: 1.5, v: 1.5 };
  assert.equal(validateInspectionTargets([edge], boxes)[0].u, 1.5);
});

test("all five panel bases preserve viewer right/down and return isolated TL/TR/BR/BL corners", () => {
  const host = [{ north: 20, east: 30, width: 6, depth: 8, height: 10 }];
  const expected = {
    north: [
      [23, 29, -4],
      [1, 0, 0],
    ],
    south: [
      [17, 31, -4],
      [-1, 0, 0],
    ],
    east: [
      [21, 34, -4],
      [0, 1, 0],
    ],
    west: [
      [19, 26, -4],
      [0, -1, 0],
    ],
    top: [
      [19, 31, -10],
      [0, 0, -1],
    ],
  };
  for (const [face, [center, normal]] of Object.entries(expected)) {
    const panel = inspectionPanel(
      { ...target, face, u: 1, v: 1, width: 2, height: 2 },
      host,
    );
    assert.deepEqual(panel.centerNed, center);
    assert.deepEqual(panel.normalNed, normal);
    near(dot(panel.normalNed, panel.rightNed), 0);
    near(dot(panel.normalNed, panel.downNed), 0);
    cross(panel.rightNed, panel.downNed).forEach((v, i) => near(v, -normal[i]));
    panel.cornersNed.forEach((point, i) => {
      const offset = point.map((v, axis) => v - center[axis]);
      near(dot(offset, panel.rightNed), [-1, 1, 1, -1][i]);
      near(dot(offset, panel.downNed), [-1, -1, 1, 1][i]);
    });
    panel.centerNed[0] = 0;
    panel.rightNed[0] = 100;
    assert.deepEqual(
      inspectionPanel(
        { ...target, face, u: 1, v: 1, width: 2, height: 2 },
        host,
      ).centerNed,
      center,
    );
  }
});

test("front-facing observation uses exact projection, range and speed with serializable assessment", () => {
  const result = assess();
  assert.equal(result.passed, true);
  assert.deepEqual(result.reasons, []);
  assert.equal(result.targetId, target.id);
  assert.equal(result.rangeM, 9);
  assert.equal(result.angleDeg, 0);
  near(result.minPixels, 160 / 9);
  assert.equal(result.speedMps, 0);
  assert.equal(result.visibleSamples, 25);
  assert.equal(result.totalSamples, 25);
  assert.deepEqual(JSON.parse(JSON.stringify(result)), result);
  assert.deepEqual(
    assess({ ...target, minRangeM: 9, maxAngleDeg: 0, maxSpeedMps: 0 }).reasons,
    [],
  );
  assert.deepEqual(assess({ ...target, maxRangeM: 8 }).reasons, ["range"]);
  assert.deepEqual(assess({ ...target, minPixels: 20 }).reasons, ["pixels"]);
  assert.deepEqual(
    assess(target, boxes, createTerrain(), camera, [], [0, 2, 0]).reasons,
    ["speed"],
  );
});

test("four-corner clip checks use camera Z independently of slant range and reject a partial panel", () => {
  const oblique = lookingAt([0, 6, -2]);
  assert.deepEqual(
    assess({ ...target, maxAngleDeg: 20 }, boxes, createTerrain(), oblique)
      .reasons,
    ["angle"],
  );
  const side = clone(camera);
  side.pose.positionNed[1] = 20;
  assert.ok(
    assess(
      { ...target, maxAngleDeg: 89 },
      boxes,
      createTerrain(),
      side,
    ).reasons.includes("frustum"),
  );
  const close = clone(camera);
  close.pose.positionNed[0] = 5;
  close.nearM = 5;
  assert.deepEqual(assess(target, boxes, createTerrain(), close).reasons, [
    "frustum",
  ]);
  const cropped = clone(camera);
  cropped.intrinsics.fx = 3000;
  assert.ok(
    assess(target, boxes, createTerrain(), cropped).reasons.includes("frustum"),
  );
  const partial = clone(camera);
  partial.intrinsics.cx = 319;
  assert.ok(
    assess(target, boxes, createTerrain(), partial).reasons.includes("frustum"),
    "center in image does not prove all four corners fit",
  );
  const far = clone(camera);
  far.farM = 10;
  const shifted = [{ ...boxes[0], north: 20 }];
  assert.deepEqual(assess(target, shifted, createTerrain(), far).reasons, [
    "frustum",
  ]);
});

test("host faces remain occluders and zero-range or rear-facing views never earn credit", () => {
  const rear = lookingAt([20, 0, -2]);
  const result = assess(target, boxes, createTerrain(), rear);
  assert.equal(result.passed, false);
  assert.ok(result.reasons.includes("angle"));
  assert.ok(result.reasons.includes("occluded"));
  assert.equal(result.visibleSamples, 0);
  const coincident = clone(camera);
  coincident.pose.positionNed = [9, 0, -2];
  const zero = assess(target, boxes, createTerrain(), coincident);
  assert.equal(zero.rangeM, 0);
  assert.equal(zero.angleDeg, null);
  assert.equal(zero.minPixels, null);
  assert.equal(zero.visibleSamples, 0);
  assert.equal(zero.passed, false);
  assert.deepEqual(JSON.parse(JSON.stringify(zero)), zero);
});

test("a near-clipped box or a retained peer blocks targets and a thin peer exposes the explicit 25-point policy", () => {
  const wall = { north: 0.025, east: 0, width: 0.01, depth: 2, height: 4 };
  const blocked = assess(target, [...boxes, wall]);
  assert.equal(blocked.visibleSamples, 0);
  assert.deepEqual(blocked.reasons, ["occluded"]);
  const peer = {
    id: "peer",
    position: [4, 0, -2],
    velocity: [0, 0, 0],
    radius: 1,
    crashed: true,
  };
  assert.equal(
    assess(target, boxes, createTerrain(), camera, [peer]).visibleSamples,
    0,
  );
  const partial = assess(target, boxes, createTerrain(), camera, [
    { ...peer, radius: 0.1 },
  ]);
  assert.equal(partial.visibleSamples, 24);
  assert.deepEqual(partial.reasons, ["occluded"]);
  assert.equal(
    assess(target, boxes, createTerrain(), camera, [
      { ...peer, position: [12, 0, -2], radius: 0.1 },
    ]).visibleSamples,
    25,
  );
});

test("real DEM triangles occlude a panel while a negative DEM does not invent a flat datum occluder", () => {
  const terrain = (heights) =>
    createTerrain({
      kind: "dem",
      north: -5,
      east: -5,
      cellSize: 5,
      rows: 5,
      cols: 3,
      heights: heights.flatMap((h) => [h, h, h]),
    });
  assert.equal(
    assess(target, boxes, terrain([0, 0, 3, 0, 0])).visibleSamples,
    0,
  );
  const valley = terrain([-3, -3, -3, -3, -3]);
  const low = lookingAt([0, 0, 2]);
  const result = assess(target, boxes, valley, low);
  assert.equal(result.passed, true);
  assert.equal(result.visibleSamples, 25);
  const flat = assess(target, boxes, createTerrain(), low);
  assert.equal(flat.visibleSamples, 0);
});

test("assessment validates camera transforms, dense velocities and peers without mutating any caller data", () => {
  const original = clone({ target, boxes, camera }),
    badCases = [
      (m) => {
        m.pose.positionNed = Array(3);
      },
      (m) => {
        m.pose.opticalToNed[0] = NaN;
      },
      (m) => {
        m.pose.opticalToNed = [1, 0, 0, 0, 1, 0, 0, 0, -1];
      },
      (m) => {
        m.pose.opticalToNed[0] = 2;
      },
      (m) => {
        m.width = 320.5;
      },
      (m) => {
        m.nearM = 0;
      },
      (m) => {
        m.intrinsics.fx = 0;
      },
      (m) => {
        m.intrinsics.cx = NaN;
      },
    ];
  for (const mutate of badCases) {
    const bad = clone(camera);
    mutate(bad);
    assert.throws(() => assess(target, boxes, createTerrain(), bad));
  }
  assert.throws(() =>
    assess(target, boxes, createTerrain(), camera, [], [0, NaN, 0]),
  );
  assert.throws(() => assess(target, boxes, createTerrain(), camera, Array(1)));
  assert.throws(() =>
    assess(target, boxes, createTerrain(), camera, [
      { position: [4, 0, -2], radius: 0 },
    ]),
  );
  assert.throws(() => assess(target, boxes, {}, camera));
  assert.deepEqual({ target, boxes, camera }, original);
  assert.deepEqual(INSPECTION_REASONS, [
    "range",
    "angle",
    "frustum",
    "pixels",
    "speed",
    "occluded",
  ]);
});

test("tower and facade templates are viable physical boxes with matching schema-5 face panels", () => {
  for (const kind of ["tower", "facade"]) {
    const template = inspectionTemplate(kind),
      cam = clone(camera);
    cam.pose.positionNed[2] = -5;
    assert.equal(template.obstacles.length, 1);
    assert.ok(template.inspectionTargets.length > 0);
    for (const t of template.inspectionTargets)
      inspectionPanel(t, template.obstacles);
    const front = template.inspectionTargets.filter((t) => t.face === "south");
    assert.ok(
      front.every(
        (t) => assess(t, template.obstacles, createTerrain(), cam).passed,
      ),
    );
    const scene = { ...clone(DEFAULT_SCENE), ...template };
    assert.equal(SCENE_VERSION, 5);
    assert.deepEqual(validateScene(scene), scene);
    assert.deepEqual(parseSceneJson(sceneToJson(scene)), scene);
    const artifact = JSON.parse(sceneToJson(scene));
    artifact.version = 4;
    assert.throws(() => parseSceneJson(JSON.stringify(artifact)));
    const missing = clone(scene);
    delete missing.inspectionTargets;
    assert.throws(() => validateScene(missing));
    const returned = inspectionTemplate(kind);
    returned.obstacles[0].north = 0;
    assert.equal(inspectionTemplate(kind).obstacles[0].north, 20);
  }
  assert.throws(() => inspectionTemplate("bridge"));
  assert.ok(Object.isFrozen(DEFAULT_SCENE.inspectionTargets));
});

test("unknown DEM coverage cannot be scored as an unobstructed inspection", () => {
  const grid = {
    kind: "dem",
    north: -5,
    east: -5,
    cellSize: 5,
    rows: 3,
    cols: 3,
    heights: Array(9).fill(0),
  };
  const compiled = createTerrain(grid);
  assert.throws(() => assess(target, boxes, compiled), /covered/);
  const big = createTerrain({ ...grid, north: -10, east: -10, cellSize: 10 });
  const outside = clone(camera);
  outside.pose.positionNed = [-20, 0, -2];
  assert.throws(() => assess(target, boxes, big, outside), /covered/);
  const geography = {
    origin: {
      latitudeDeg: 25,
      longitudeDeg: 121,
      altitudeM: 100,
      verticalDatum: "orthometric",
    },
    source: {
      name: "grid.tif",
      sha256: "a".repeat(64),
      crs: "EPSG:4326",
      width: 3,
      height: 3,
      band: 1,
      pixelInterpretation: "point",
      heightUnit: "m",
      scale: 1,
      offset: 0,
      resampling: "bilinear",
      window: [0, 0, 3, 3],
    },
  };
  const scene = {
    ...clone(DEFAULT_SCENE),
    terrain: grid,
    geography,
    obstacles: boxes,
  };
  assert.doesNotThrow(
    () => validateScene(scene),
    "the existing obstacle contract is unchanged",
  );
  assert.throws(
    () => validateScene({ ...scene, inspectionTargets: [target] }),
    /covered/,
  );
});
