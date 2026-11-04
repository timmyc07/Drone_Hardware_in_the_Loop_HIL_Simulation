import test from "node:test";
import assert from "node:assert/strict";
import {
  MAX_RADAR_DETECTIONS,
  RADAR_DEFAULT_CONFIG,
  RADAR_BOUNDS,
  RADAR_CSV_HEADER,
  validateRadarConfig,
  radarMetadata,
  radarDirections,
  validateRadarReading,
  compactRadarReading,
  exportRadarData,
} from "../radar.js";

const clone = structuredClone;
const near = (actual, expected) =>
  assert.ok(
    Math.abs(actual - expected) < 1e-12,
    `${actual} differs from ${expected}`,
  );
const config = { radarAzimuthRays: 3, radarElevationRays: 3 };
function reading(custom = config) {
  const directions = radarDirections(custom),
    center = Math.floor(directions.length / 2),
    d = directions[center];
  return {
    time: 1,
    valid: true,
    reason: "",
    positionNed: [10, 20, -5],
    quaternion: [Math.SQRT1_2, 0, 0, Math.SQRT1_2],
    velocityNedMps: [1, 2, -3],
    detections: [
      {
        beam: center,
        rangeM: 10,
        azimuthRad: d.azimuthRad,
        elevationRad: d.elevationRad,
        rangeRateMps: -2,
      },
    ],
    ...radarMetadata(custom),
  };
}
function invalid(reason, nullPose = reason === "startup") {
  return {
    ...reading(),
    valid: false,
    reason,
    detections: [],
    ...(nullPose
      ? { positionNed: null, quaternion: null, velocityNedMps: null }
      : {}),
  };
}
function rejects(value, mutations, options = config) {
  for (const mutate of mutations) {
    const bad = clone(value);
    mutate(bad);
    assert.throws(
      () => validateRadarReading(bad, options),
      undefined,
      mutate.toString(),
    );
  }
}
const context = { sourceVehicleId: "drone-2", worldTick: 400 };

test("radar configuration is canonical, bounded and independent of non-radar sensor fields", () => {
  assert.deepEqual(validateRadarConfig(), RADAR_DEFAULT_CONFIG);
  assert.equal(Object.keys(RADAR_DEFAULT_CONFIG).length, 9);
  assert.ok(Object.isFrozen(RADAR_DEFAULT_CONFIG));
  assert.ok(Object.isFrozen(RADAR_BOUNDS));
  assert.ok(Object.values(RADAR_BOUNDS).every(Object.isFrozen));
  assert.deepEqual(
    Object.keys(validateRadarConfig({ radarMaxM: 200, radarHz: 7.5 })),
    Object.keys(RADAR_DEFAULT_CONFIG),
  );
  assert.deepEqual(
    radarMetadata({ gpsHz: 5, ...config }),
    radarMetadata(config),
  );
  assert.deepEqual(
    radarDirections({ gpsHz: 5, ...config }),
    radarDirections(config),
  );
  const input = { radarHz: 5 },
    checked = validateRadarConfig(input);
  checked.radarHz = 10;
  assert.equal(input.radarHz, 5);
  for (const bad of [
    null,
    [],
    1,
    { unknown: 1 },
    { radarHz: undefined },
    { radarHz: NaN },
    { radarHz: 0 },
    { radarHz: 21 },
    { radarAzimuthRays: 0 },
    { radarAzimuthRays: 82 },
    { radarAzimuthRays: 1.5 },
    { radarElevationRays: 42 },
    { radarElevationRays: 1.5 },
    { radarHorizontalFovDeg: 0 },
    { radarHorizontalFovDeg: 181 },
    { radarVerticalFovDeg: 91 },
    { radarMinM: 10, radarMaxM: 10 },
    { radarNoiseM: -1 },
    { radarVelocityNoiseMps: Infinity },
    { [Symbol("extra")]: 1 },
  ])
    assert.throws(() => validateRadarConfig(bad));
  assert.throws(() => radarMetadata({ radarUnknown: 1 }));
});

test("fixed body-forward angular sampling includes full FOV edges, correct FRD elevation and a centered single ray", () => {
  const directions = radarDirections(config);
  assert.equal(directions.length, 9);
  near(directions[0].azimuthRad, -Math.PI / 6);
  near(directions[0].elevationRad, -Math.PI / 18);
  near(directions[8].azimuthRad, Math.PI / 6);
  near(directions[8].elevationRad, Math.PI / 18);
  assert.ok(
    directions[0].direction[2] > 0,
    "negative elevation points down in FRD",
  );
  assert.ok(
    directions[8].direction[2] < 0,
    "positive elevation points up in FRD",
  );
  assert.equal(directions[4].azimuthRad, 0);
  assert.equal(directions[4].elevationRad, 0);
  near(directions[4].direction[0], 1);
  near(directions[4].direction[1], 0);
  near(directions[4].direction[2], 0);
  for (const [i, d] of directions.entries()) {
    assert.equal(d.beam, i);
    near(Math.hypot(...d.direction), 1);
  }
  const single = radarDirections({
    radarAzimuthRays: 1,
    radarElevationRays: 1,
  })[0];
  assert.equal(single.azimuthRad, 0);
  assert.equal(single.elevationRad, 0);
  near(single.direction[0], 1);
  const even = radarDirections({ radarAzimuthRays: 2, radarElevationRays: 2 });
  assert.equal(even.length, 4);
  near(even[0].azimuthRad, -Math.PI / 6);
  near(even[3].azimuthRad, Math.PI / 6);
  const maximum = radarDirections({
    radarAzimuthRays: 81,
    radarElevationRays: 41,
    radarHorizontalFovDeg: 180,
    radarVerticalFovDeg: 90,
  });
  assert.equal(MAX_RADAR_DETECTIONS, 3321);
  assert.equal(maximum.length, MAX_RADAR_DETECTIONS);
  near(maximum[0].azimuthRad, -Math.PI / 2);
  near(maximum.at(-1).elevationRad, Math.PI / 4);
});

test("full and six-field compact readings preserve custom grids, acquired poses and signed range rates without aliases", () => {
  for (const options of [
    config,
    {
      radarAzimuthRays: 1,
      radarElevationRays: 1,
      radarHorizontalFovDeg: 61,
      radarVerticalFovDeg: 33,
    },
    {
      radarAzimuthRays: 4,
      radarElevationRays: 2,
      radarHorizontalFovDeg: 107,
      radarVerticalFovDeg: 17,
    },
  ]) {
    const raw = reading(options),
      clean = validateRadarReading(raw, options),
      compact = compactRadarReading(raw);
    assert.equal(compact.length, 6);
    assert.deepEqual(compact[5], [[raw.detections[0].beam, 10, -2]]);
    assert.deepEqual(
      validateRadarReading(JSON.parse(JSON.stringify(compact)), options),
      clean,
    );
    assert.deepEqual(
      validateRadarReading(JSON.parse(JSON.stringify(raw)), options),
      clean,
    );
    assert.equal(clean.detections[0].rangeRateMps, -2);
    clean.positionNed[0] = 999;
    clean.velocityNedMps[0] = 999;
    clean.detections[0].rangeM = 99;
    compact[2][0] = 888;
    compact[5][0][1] = 88;
    assert.equal(raw.positionNed[0], 10);
    assert.equal(raw.velocityNedMps[0], 1);
    assert.equal(raw.detections[0].rangeM, 10);
  }
  const largeRelativeVelocity = reading();
  largeRelativeVelocity.detections[0].rangeRateMps = 4e9;
  assert.equal(
    validateRadarReading(largeRelativeVelocity, config).detections[0]
      .rangeRateMps,
    4e9,
  );
});

test("startup, fault, dropout and no-return retain distinct acquisition-pose hygiene through the codec", () => {
  for (const raw of [
    invalid("startup"),
    invalid("fault", true),
    invalid("fault", false),
    invalid("dropout"),
    invalid("no-return"),
  ]) {
    const canonical = validateRadarReading(raw, config),
      compact = compactRadarReading(raw);
    assert.deepEqual(validateRadarReading(compact, config), canonical);
    assert.equal(canonical.valid, false);
    assert.deepEqual(canonical.detections, []);
  }
  for (const reason of ["dropout", "no-return"])
    assert.throws(
      () => validateRadarReading(invalid(reason, true), config),
      /require/,
    );
  assert.throws(
    () => validateRadarReading(invalid("startup", false), config),
    /Startup/,
  );
  rejects(invalid("fault", true), [
    (v) => {
      v.positionNed = [0, 0, 0];
    },
    (v) => {
      v.quaternion = [1, 0, 0, 0];
    },
    (v) => {
      v.velocityNedMps = [0, 0, 0];
    },
  ]);
});

test("full-reading trust boundaries reject malformed metadata, non-grid angles, duplicate beams and hidden identity payloads", () => {
  rejects(reading(), [
    (v) => {
      v.time = NaN;
    },
    (v) => {
      v.time = -1;
    },
    (v) => {
      v.valid = 1;
    },
    (v) => {
      v.reason = "dropout";
    },
    (v) => {
      v.reason = "invented";
    },
    (v) => {
      v.detections = [];
    },
    (v) => {
      v.detections = Array(1);
    },
    (v) => {
      v.detections.push(clone(v.detections[0]));
    },
    (v) => {
      v.detections[0].beam = 9;
    },
    (v) => {
      v.detections[0].beam = 1.5;
    },
    (v) => {
      v.detections[0].rangeM = 0.01;
    },
    (v) => {
      v.detections[0].rangeM = 101;
    },
    (v) => {
      v.detections[0].rangeRateMps = Infinity;
    },
    (v) => {
      v.detections[0].rangeRateMps = 1e10 + 1;
    },
    (v) => {
      v.detections[0].azimuthRad = 1e-15;
    },
    (v) => {
      v.detections[0].elevationRad = NaN;
    },
    (v) => {
      v.detections[0].targetVehicleId = "ground-truth";
    },
    (v) => {
      v.detections[0].rcs = 1;
    },
    (v) => {
      v.azimuthCount = 2;
    },
    (v) => {
      v.elevationCount = 2;
    },
    (v) => {
      v.minM = 0.1;
    },
    (v) => {
      v.maxM = 101;
    },
    (v) => {
      v.horizontalFovRad = 1;
    },
    (v) => {
      v.verticalFovRad = Infinity;
    },
    (v) => {
      v.positionNed = Array(3);
    },
    (v) => {
      v.velocityNedMps = [0, 0, NaN];
    },
    (v) => {
      v.quaternion = [2, 0, 0, 0];
    },
    (v) => {
      v.positionNed = null;
    },
    (v) => {
      v.extra = true;
    },
    (v) => {
      v[Symbol("extra")] = true;
    },
    (v) => {
      Object.defineProperty(v, "hidden", { value: true });
    },
  ]);
  assert.throws(() => compactRadarReading({ ...reading(), azimuthCount: 0 }));
});

test("compact-reading trust boundaries reject sparse carriers, invalid reason codes, poses and tuples", () => {
  const compact = compactRadarReading(reading());
  rejects(compact, [
    (v) => {
      v.pop();
    },
    (v) => {
      delete v[3];
    },
    (v) => {
      v.extra = 1;
    },
    (v) => {
      v[0] = "1";
    },
    (v) => {
      v[1] = 5;
    },
    (v) => {
      v[1] = -1;
    },
    (v) => {
      v[1] = 0.5;
    },
    (v) => {
      v[2] = null;
    },
    (v) => {
      v[3] = Array(4);
    },
    (v) => {
      v[4] = [Infinity, 0, 0];
    },
    (v) => {
      v[5] = Array(1);
    },
    (v) => {
      v[5][0] = Array(3);
    },
    (v) => {
      v[5][0].pop();
    },
    (v) => {
      v[5][0][0] = -1;
    },
    (v) => {
      v[5][0][1] = null;
    },
    (v) => {
      v[5][0][2] = NaN;
    },
    (v) => {
      v[5].push(clone(v[5][0]));
    },
  ]);
});

test("maximum readings have one ordered return per beam and reject excess, reordered or mismatched-grid data", () => {
  const options = { radarAzimuthRays: 81, radarElevationRays: 41 },
    raw = reading(options);
  raw.detections = radarDirections(options).map((d) => ({
    beam: d.beam,
    rangeM: 50,
    azimuthRad: d.azimuthRad,
    elevationRad: d.elevationRad,
    rangeRateMps: d.beam % 2 ? -3 : 3,
  }));
  assert.equal(
    validateRadarReading(raw, options).detections.length,
    MAX_RADAR_DETECTIONS,
  );
  const compact = compactRadarReading(raw);
  assert.deepEqual(validateRadarReading(compact, options), raw);
  rejects(
    raw,
    [
      (v) => {
        v.detections.push(clone(v.detections[0]));
      },
      (v) => {
        v.detections.reverse();
      },
    ],
    options,
  );
  assert.throws(() => validateRadarReading(compact, config));
});

test("JSON radar exports preserve acquisition time, source identity, export tick and only the radar configuration", () => {
  const raw = reading(),
    fullSensors = { gpsHz: 5, originLatitudeDeg: 25.033, ...config };
  const output = JSON.parse(exportRadarData("json", raw, context, fullSensors));
  assert.deepEqual(Object.keys(output), [
    "format",
    "version",
    "sourceVehicleId",
    "exportWorldTick",
    "frame",
    "detectionFrame",
    "units",
    "config",
    "reading",
  ]);
  assert.equal(output.format, "flight-lab-radar");
  assert.equal(output.version, 1);
  assert.equal(output.sourceVehicleId, "drone-2");
  assert.equal(output.exportWorldTick, 400);
  assert.equal(output.reading.time, 1);
  assert.equal(output.frame, "NED");
  assert.equal(output.detectionFrame, "FRD");
  assert.equal(output.units, "m");
  assert.deepEqual(output.config, validateRadarConfig(config));
  assert.deepEqual(output.reading, raw);
  assert.equal(Object.hasOwn(output.config, "originLatitudeDeg"), false);
  for (const owner of [
    null,
    {},
    { ...context, sourceVehicleId: "=formula" },
    { ...context, sourceVehicleId: "a,b" },
    { ...context, sourceVehicleId: "a".repeat(33) },
    { ...context, worldTick: 1.5 },
    { ...context, worldTick: 360001 },
    { ...context, worldTick: 199 },
    { ...context, extra: true },
  ])
    assert.throws(() => exportRadarData("json", raw, owner, config));
  const final = reading();
  let accumulatedTime = 0;
  for (let tick = 0; tick < 360000; tick++) accumulatedTime += 0.005;
  final.time = accumulatedTime;
  assert.doesNotThrow(() =>
    exportRadarData("json", final, { ...context, worldTick: 360000 }, config),
  );
  assert.throws(() => exportRadarData("pcd", raw, context, config));
});

test("CSV keeps signed Doppler values and emits one metadata-only row for every zero-return state", () => {
  const raw = reading(),
    rows = exportRadarData("csv", raw, context, config)
      .split("\n")
      .map((row) => row.split(","));
  assert.deepEqual(rows[0], RADAR_CSV_HEADER);
  assert.equal(rows[0].length, 26);
  assert.equal(rows.length, 2);
  assert.equal(rows[1][0], "drone-2");
  assert.equal(rows[1][1], "400");
  assert.equal(rows[1][2], "1");
  assert.equal(rows[1][25], "-2");
  assert.equal(rows[1][21], String(raw.detections[0].beam));
  for (const zero of [
    invalid("startup"),
    invalid("fault", true),
    invalid("dropout"),
    invalid("no-return"),
  ]) {
    const csv = exportRadarData("csv", zero, context, config)
      .split("\n")
      .map((row) => row.split(","));
    assert.equal(csv.length, 2);
    assert.equal(csv[1].length, 26);
    assert.equal(csv[1][3], "0");
    assert.equal(csv[1][4], zero.reason);
    assert.ok(
      csv[1].slice(21).every((v) => v === ""),
      "no-return must not manufacture a zero-range target",
    );
    if (zero.positionNed === null)
      assert.ok(csv[1].slice(5, 15).every((v) => v === ""));
    else assert.equal(csv[1][5], "10");
  }
});
