import test from "node:test";
import { localToGeographic } from "../geography.js";
import assert from "node:assert/strict";
import {
  createSensorSuite,
  validateSensorConfig,
  DEFAULT_SENSOR_CONFIG,
  lidarPoints,
  MAX_LIDAR_POINTS,
  assertGeographicSensors,
} from "../sensors.js";

const quiet = {
  gpsNoiseM: 0,
  gpsVerticalNoiseM: 0,
  gpsVelocityNoiseMps: 0,
  barometerNoisePa: 0,
  magnetometerNoiseMicroT: 0,
  rangefinderNoiseM: 0,
  lidarNoiseM: 0,
  opticalFlowNoiseRadS: 0,
};
const planar = {
  lidarChannels: 1,
  lidarVerticalMinDeg: 0,
  lidarVerticalMaxDeg: 0,
};
const state = (time = 0, extra = {}) => ({
  time,
  position: [0, 0, -2],
  velocity: [0, 0, 0],
  quaternion: [1, 0, 0, 0],
  rates: [0, 0, 0],
  ...extra,
});
const near = (a, b, tolerance = 1e-8) =>
  assert.ok(Math.abs(a - b) < tolerance, `${a} differs from ${b}`);

test("range, lidar and point clouds share raised and negative terrain without a phantom datum plane", () => {
  for (const height of [4, -4]) {
    const heights = Array(25).fill(0);
    heights[12] = height;
    const terrain = {
      kind: "heightfield",
      north: 40,
      east: 40,
      cellSize: 10,
      rows: 5,
      cols: 5,
      heights,
    };
    const suite = createSensorSuite({
      ...quiet,
      latency: 0,
      lidarChannels: 1,
      lidarRays: 12,
      lidarVerticalMinDeg: -90,
      lidarVerticalMaxDeg: -90,
    });
    suite.setTerrain(terrain);
    const reading = suite.update(state(0, { position: [60, 60, -10] }));
    near(reading.rangefinder.distanceM, 10 - height);
    reading.lidar.rangesM.forEach((r) => near(r, 10 - height));
    lidarPoints(reading.lidar).forEach((p) => near(p[2], -height));
    const closeReading = suite.update(
      state(0.1, { position: [60, 60, -height - 0.05] }),
    );
    assert.equal(closeReading.rangefinder.valid, false);
    assert.equal(closeReading.rangefinder.reason, "no-return");
  }
});

test("terrain changes preserve delayed acquisitions, faults and invalid-change atomicity", () => {
  const heights = Array(25).fill(0);
  heights[12] = 4;
  const terrain = {
    kind: "heightfield",
    north: 40,
    east: 40,
    cellSize: 10,
    rows: 5,
    cols: 5,
    heights,
  };
  const suite = createSensorSuite({
    ...quiet,
    latency: 0.2,
    rangefinderHz: 10,
  });
  suite.setTerrain(terrain);
  suite.setFault("gps", true);
  suite.update(state(0, { position: [60, 60, -10] }));
  const valley = structuredClone(terrain);
  valley.heights[12] = -4;
  suite.setTerrain(valley);
  suite.update(state(0.1, { position: [60, 60, -10] }));
  near(
    suite.update(state(0.2, { position: [60, 60, -10] })).rangefinder.distanceM,
    6,
  );
  const current = suite.update(state(0.3, { position: [60, 60, -10] }));
  near(current.rangefinder.distanceM, 14);
  assert.equal(current.faults.gps, true);
  const bad = structuredClone(terrain);
  bad.heights[0] = 1;
  const before = suite.snapshot();
  assert.throws(() => suite.setTerrain(bad));
  assert.deepEqual(suite.snapshot(), before);
});
const roll = (angle) => [Math.cos(angle / 2), Math.sin(angle / 2), 0, 0];
const yaw = (angle) => [Math.cos(angle / 2), 0, 0, Math.sin(angle / 2)];

test("WGS84 GPS reference is assumed origin; north/east/up signs and velocity are correct", () => {
  const suite = createSensorSuite(quiet);
  let sample = suite.update(state());
  near(sample.gps.latitudeDeg, DEFAULT_SENSOR_CONFIG.originLatitudeDeg);
  near(sample.gps.longitudeDeg, DEFAULT_SENSOR_CONFIG.originLongitudeDeg);
  near(sample.gps.altitudeM, 22);
  sample = suite.update(
    state(0.2, { position: [100, 100, -12], velocity: [1, 2, -3] }),
  );
  assert.ok(sample.gps.latitudeDeg > 25.033);
  assert.ok(sample.gps.longitudeDeg > 121.5654);
  near(sample.gps.altitudeM, 32);
  assert.deepEqual(sample.gps.velocityNedMps, [1, 2, -3]);
  const equator = createSensorSuite({
    ...quiet,
    originLatitudeDeg: 0,
    originLongitudeDeg: 0,
    originAltitudeM: 0,
  });
  const zero = equator.update(state(0, { position: [0, 0, 0] })).gps;
  near(zero.latitudeDeg, 0);
  near(zero.longitudeDeg, 0);
  near(zero.altitudeM, 0);
  near(
    equator.update(state(0.2, { position: [0, (6378137 * Math.PI) / 180, 0] }))
      .gps.longitudeDeg,
    1,
  );
});
test("ISA barometric pressure and inversion agree at sea level and known height", () => {
  const suite = createSensorSuite({ ...quiet, originAltitudeM: 0 });
  let reading = suite.update(state(0, { position: [0, 0, 0] })).barometer;
  near(reading.pressurePa, 101325);
  near(reading.temperatureK, 288.15);
  near(reading.altitudeM, 0);
  reading = suite.update(state(0.05, { position: [0, 0, -1000] })).barometer;
  near(reading.altitudeM, 1000, 1e-7);
  near(reading.temperatureK, 281.65);
  near(reading.pressurePa, 101325 * (1 - (0.0065 * 1000) / 288.15) ** 5.25588);
});
test("magnetometer rotates known NED field into body frame without claiming WMM", () => {
  const suite = createSensorSuite({
    ...quiet,
    magneticFieldMicroT: 50,
    magneticInclinationDeg: 0,
  });
  assert.deepEqual(
    suite.update(state()).magnetometer.fieldBodyMicroT,
    [50, 0, 0],
  );
  const field = suite.update(state(0.05, { quaternion: yaw(Math.PI / 2) }))
    .magnetometer.fieldBodyMicroT;
  near(field[0], 0);
  near(field[1], -50);
  near(field[2], 0);
});
test("rangefinder tilted body-down ray hits ground at slant distance and nearest box first", () => {
  const suite = createSensorSuite(quiet);
  near(
    suite.update(state(0, { quaternion: roll(Math.PI / 3) })).rangefinder
      .distanceM,
    4,
  );
  suite.setObstacles([{ north: 0, east: 0, width: 2, depth: 2, height: 1 }]);
  near(suite.update(state(0.05)).rangefinder.distanceM, 1);
  let reading = suite.update(
    state(0.1, { quaternion: roll(Math.PI) }),
  ).rangefinder;
  assert.equal(reading.valid, false);
  assert.equal(reading.reason, "no-return");
  assert.equal(reading.distanceM, null);
  suite.setObstacles([{ north: 0, east: 0, width: 2, depth: 2, height: 1.95 }]);
  assert.equal(
    suite.update(state(0.15)).rangefinder.distanceM,
    null,
    "blind-zone obstruction must not reveal ground behind it",
  );
});
test("lidar returns sparse body-plane ranges and transforms points to absolute NED", () => {
  for (const count of [36, 72]) {
    const suite = createSensorSuite({ ...quiet, ...planar, lidarRays: count });
    let reading = suite.update(state()).lidar;
    assert.equal(reading.valid, false);
    assert.equal(reading.reason, "no-return");
    assert.equal(reading.rangesM.length, count);
    assert.ok(reading.rangesM.every((v) => v === null));
    assert.deepEqual(lidarPoints(reading), []);
    suite.setObstacles([{ north: 5, east: 0, width: 2, depth: 2, height: 4 }]);
    reading = suite.update(state(0.1)).lidar;
    near(reading.rangesM[count / 2], 4);
    assert.ok(reading.rangesM.some((v) => v === null));
    const points = lidarPoints(reading);
    assert.ok(points.length > 0);
    assert.ok(
      points.some(
        (p) =>
          Math.abs(p[0] - 4) < 1e-8 && Math.abs(p[1]) < 1e-8 && p[2] === -2,
      ),
    );
    const tilted = suite.update(
      state(0.2, { quaternion: roll(Math.PI / 4) }),
    ).lidar;
    assert.ok(
      lidarPoints(tilted).some((p) => Math.abs(p[2]) < 1e-8),
      "tilted scan rays hit ground",
    );
  }
});
test("optical flow uses declared image axes, body velocity and rotational rate signs", () => {
  const suite = createSensorSuite(quiet);
  let reading = suite.update(
    state(0, { velocity: [2, 4, 0], rates: [0.1, 0.2, 0] }),
  ).opticalFlow;
  assert.deepEqual(reading.flowRadS, [-1.9, 1.2]);
  near(reading.groundDistanceM, 2);
  reading = suite.update(
    state(0.05, { quaternion: roll(Math.PI) }),
  ).opticalFlow;
  assert.equal(reading.valid, false);
  assert.equal(reading.reason, "no-return");
  assert.equal(reading.flowRadS, null);
});
test("channel cadence holds acquisition timestamp and never fabricates skipped poses", () => {
  const suite = createSensorSuite(quiet);
  assert.equal(suite.snapshot().gps.reason, "startup");
  suite.update(state());
  let sample = suite.update(state(0.045));
  near(sample.time, 0.045);
  near(sample.gps.time, 0);
  near(sample.barometer.time, 0);
  sample = suite.update(state(0.05));
  near(sample.barometer.time, 0.05);
  near(sample.gps.time, 0);
  sample = suite.update(state(0.2));
  near(sample.gps.time, 0.2);
  near(sample.barometer.time, 0.2);
  sample = suite.update(state(5, { position: [10, 0, -2] }));
  near(sample.gps.time, 5);
  const repeat = suite.update(state(5));
  assert.deepEqual(repeat, sample);
  const fractional = createSensorSuite({ ...quiet, gpsHz: 7 });
  fractional.update(state());
  near(fractional.update(state(0.145)).gps.time, 0.145);
});
test("latency delivers old acquisition times, dropout is explicit, fault recovery requires fresh data", () => {
  const suite = createSensorSuite({ ...quiet, latency: 0.1 });
  assert.equal(suite.update(state()).gps.reason, "startup");
  let sample = suite.update(state(0.1));
  near(sample.gps.time, 0);
  assert.equal(sample.gps.valid, true);
  suite.setFault("gps", true);
  sample = suite.snapshot();
  assert.equal(sample.gps.valid, false);
  assert.equal(sample.gps.reason, "fault");
  suite.update(state(0.2));
  suite.setFault("gps", false);
  sample = suite.update(state(0.2));
  assert.equal(sample.gps.valid, false);
  assert.equal(sample.gps.reason, "startup");
  sample = suite.update(state(0.3));
  assert.equal(sample.gps.valid, true);
  near(sample.gps.time, 0.2);
  const dropped = createSensorSuite({ ...quiet, dropout: 1 }).update(state());
  for (const name of Object.keys(dropped.faults)) {
    assert.equal(dropped[name].reason, "dropout");
    assert.equal(dropped[name].valid, false);
  }
});
test("rangefinder fault invalidates dependent flow without faulting the flight controller", () => {
  const suite = createSensorSuite(quiet);
  suite.update(state());
  suite.setFault("rangefinder", true);
  const sample = suite.update(state(0.05));
  assert.equal(sample.rangefinder.reason, "fault");
  assert.equal(sample.opticalFlow.valid, false);
  assert.equal(sample.opticalFlow.reason, "no-return");
  assert.equal(sample.faults.opticalFlow, false);
});
test("seeded noise/dropout are deterministic, reset repeats, snapshots do not alias", () => {
  const config = { latency: 0.2, dropout: 0.2 };
  const a = createSensorSuite(config, 123),
    b = createSensorSuite(config, 123);
  for (let i = 0; i <= 400; i++)
    assert.deepEqual(a.update(state(i * 0.005)), b.update(state(i * 0.005)));
  const first = a.snapshot();
  a.reset();
  for (let i = 0; i <= 400; i++) a.update(state(i * 0.005));
  assert.deepEqual(a.snapshot(), first);
  first.gps.latitudeDeg = 0;
  first.faults.gps = true;
  first.lidar.rangesM.fill(99);
  assert.notEqual(a.snapshot().gps.latitudeDeg, 0);
  assert.equal(a.snapshot().faults.gps, false);
  const other = createSensorSuite(config, 124);
  for (let i = 0; i <= 400; i++) other.update(state(i * 0.005));
  assert.notDeepEqual(a.snapshot(), other.snapshot());
});
test("configuration and input trust boundaries reject invalid data without partial mutation", () => {
  assert.deepEqual(validateSensorConfig(), DEFAULT_SENSOR_CONFIG);
  for (const config of [
    null,
    [],
    { gpsHz: 0 },
    { lidarRays: 12.5 },
    { lidarMinM: 10, lidarMaxM: 1 },
    { originLatitudeDeg: 90 },
    { noise: 1 },
    { latency: Infinity },
  ])
    assert.throws(() => validateSensorConfig(config));
  for (const seed of [-1, 1.5, NaN])
    assert.throws(() => createSensorSuite({}, seed));
  const suite = createSensorSuite(quiet);
  suite.update(state());
  const before = suite.snapshot();
  for (const snapshot of [
    null,
    state(-1),
    state(0.1, { quaternion: [0, 0, 0, 0] }),
    state(0.1, { velocity: [NaN, 0, 0] }),
  ])
    assert.throws(() => suite.update(snapshot));
  assert.throws(() =>
    suite.setObstacles([{ north: 0, east: 0, width: -1, depth: 1, height: 1 }]),
  );
  assert.throws(() => suite.setObstacles(Array(1)));
  assert.throws(() => suite.setFault("unknown", true));
  assert.throws(() => suite.setFault("gps", 1));
  assert.deepEqual(suite.snapshot(), before);
  suite.update(state(1));
  assert.throws(() => suite.update(state(0.5)), /backwards/);
});
test("maximum rate/latency queues stay bounded while latest readings keep their acquisition age", () => {
  const suite = createSensorSuite({
    ...quiet,
    latency: 1,
    gpsHz: 20,
    barometerHz: 20,
    magnetometerHz: 20,
    rangefinderHz: 20,
    lidarHz: 20,
    opticalFlowHz: 20,
  });
  let sample;
  for (let i = 0; i <= 2000; i++) sample = suite.update(state(i * 0.005));
  for (const name of Object.keys(sample.faults)) near(sample[name].time, 9);
  assert.equal(sample.gps.valid, true);
});
test("lidarPoints uses acquisition translation and rotation rather than current vehicle pose", () => {
  const suite = createSensorSuite({ ...quiet, ...planar, lidarRays: 36 });
  suite.setObstacles([{ north: 10, east: 25, width: 2, depth: 2, height: 4 }]);
  const sample = suite.update(
    state(0, { position: [10, 20, -2], quaternion: yaw(Math.PI / 2) }),
  );
  near(sample.lidar.rangesM[18], 4);
  assert.ok(
    lidarPoints(sample.lidar).some(
      (p) =>
        Math.abs(p[0] - 10) < 1e-8 && Math.abs(p[1] - 24) < 1e-8 && p[2] === -2,
    ),
  );
});

test("three channel geometry is channel-major with analytic ground and wall intersections", () => {
  const suite = createSensorSuite({
    ...quiet,
    lidarChannels: 3,
    lidarRays: 12,
    lidarVerticalMinDeg: -30,
    lidarVerticalMaxDeg: 30,
  });
  let reading = suite.update(state()).lidar;
  assert.equal(reading.channelCount, 3);
  assert.equal(reading.azimuthCount, 12);
  assert.equal(reading.rangesM.length, 36);
  near(reading.elevationStartRad, -Math.PI / 6);
  near(reading.elevationStepRad, Math.PI / 6);
  for (let i = 0; i < 12; i++) near(reading.rangesM[i], 4);
  assert.ok(reading.rangesM.slice(12).every((value) => value === null));
  for (const point of lidarPoints(reading)) near(point[2], 0);
  suite.setObstacles([{ north: 5, east: 0, width: 2, depth: 2, height: 6 }]);
  reading = suite.update(state(0.1)).lidar;
  near(reading.rangesM[6], 4); // Down ray reaches ground before the wall.
  near(reading.rangesM[12 + 6], 4);
  near(reading.rangesM[24 + 6], 4 / Math.cos(Math.PI / 6));
  const points = lidarPoints(reading);
  assert.ok(
    points.some(
      (point) =>
        Math.abs(point[0] - 4) < 1e-8 &&
        Math.abs(point[2] + 2 + 4 * Math.tan(Math.PI / 6)) < 1e-8,
    ),
  );
  suite.setObstacles([
    { north: 0.1, east: 0, width: 0.1, depth: 1, height: 6 },
  ]);
  reading = suite.update(state(0.2)).lidar;
  for (let channel = 0; channel < 3; channel++)
    assert.equal(
      reading.rangesM[channel * 12 + 6],
      null,
      "blind-zone box occludes all layers behind it",
    );
});

test("multi-layer scan uses acquisition rotation and delayed pose without relabeling time", () => {
  const suite = createSensorSuite({
    ...quiet,
    latency: 0.2,
    lidarChannels: 3,
    lidarRays: 12,
    lidarVerticalMinDeg: -30,
    lidarVerticalMaxDeg: 30,
  });
  suite.update(
    state(0, { position: [10, 20, -2], quaternion: roll(Math.PI / 2) }),
  );
  const reading = suite.update(state(0.2, { position: [100, 200, -20] })).lidar;
  assert.equal(reading.time, 0);
  assert.deepEqual(reading.positionNed, [10, 20, -2]);
  near(reading.rangesM[9], 2 / Math.cos(Math.PI / 6));
  near(reading.rangesM[12 + 9], 2);
  near(reading.rangesM[24 + 9], 2 / Math.cos(Math.PI / 6));
  const points = lidarPoints(reading);
  assert.ok(points.length > 0);
  for (const point of points) {
    near(point[2], 0);
    assert.ok(Math.abs(point[0] - 10) < 50 && Math.abs(point[1] - 20) < 50);
  }
});

test("maximum 3D budget, vertical endpoint channels and single-channel scans remain bounded", () => {
  assert.equal(MAX_LIDAR_POINTS, 5760);
  const suite = createSensorSuite({
    ...quiet,
    lidarChannels: 32,
    lidarRays: 180,
    lidarVerticalMinDeg: -90,
    lidarVerticalMaxDeg: 90,
  });
  const reading = suite.update(state()).lidar;
  assert.equal(reading.rangesM.length, MAX_LIDAR_POINTS);
  for (const range of reading.rangesM.slice(0, 180)) near(range, 2);
  assert.ok(reading.rangesM.slice(-180).every((value) => value === null));
  assert.ok(lidarPoints(reading).length <= MAX_LIDAR_POINTS);
  const downward = createSensorSuite({
    ...quiet,
    lidarChannels: 1,
    lidarVerticalMinDeg: -90,
    lidarVerticalMaxDeg: -90,
  }).update(state()).lidar;
  assert.equal(downward.elevationStepRad, 0);
  for (const range of downward.rangesM) near(range, 2);
  for (const point of lidarPoints(downward)) {
    near(point[0], 0);
    near(point[1], 0);
    near(point[2], 0);
  }
});

test("3D lidar fault/dropout metadata stays complete and seeded delayed acquisitions repeat", () => {
  const config = {
    lidarChannels: 4,
    lidarRays: 24,
    lidarVerticalMinDeg: -45,
    lidarVerticalMaxDeg: 15,
    latency: 0.2,
    dropout: 0.3,
  };
  const a = createSensorSuite(config, 333),
    b = createSensorSuite(config, 333);
  for (let i = 0; i < 100; i++)
    assert.deepEqual(a.update(state(i * 0.005)), b.update(state(i * 0.005)));
  const before = a.snapshot();
  a.reset();
  for (let i = 0; i < 100; i++) a.update(state(i * 0.005));
  assert.deepEqual(a.snapshot(), before);
  a.setFault("lidar", true);
  let reading = a.snapshot().lidar;
  assert.equal(reading.channelCount, 4);
  assert.equal(reading.azimuthCount, 24);
  assert.equal(reading.rangesM.length, 96);
  assert.equal(reading.reason, "fault");
  assert.equal(reading.positionNed, null);
  assert.deepEqual(lidarPoints(reading), []);
  a.setFault("lidar", false);
  reading = a.update(state(0.5)).lidar;
  assert.equal(reading.reason, "startup");
  assert.equal(reading.rangesM.length, 96);
  assert.deepEqual(lidarPoints(reading), []);
  const dropout = createSensorSuite({
    ...quiet,
    ...config,
    dropout: 1,
    latency: 0,
  }).update(state()).lidar;
  assert.equal(dropout.reason, "dropout");
  assert.equal(dropout.rangesM.length, 96);
  assert.deepEqual(lidarPoints(dropout), []);
});

test("3D configuration and point reconstruction reject sparse, nonunit and inconsistent metadata", () => {
  for (const config of [
    { lidarChannels: 0 },
    { lidarChannels: 33 },
    { lidarChannels: 1.1 },
    { lidarChannels: 1 },
    { lidarVerticalMinDeg: 10, lidarVerticalMaxDeg: -10 },
    { lidarVerticalMinDeg: -91 },
    { lidarChannels: 2, lidarVerticalMinDeg: 0, lidarVerticalMaxDeg: 0 },
  ])
    assert.throws(() => validateSensorConfig(config));
  const reading = createSensorSuite(quiet).update(state()).lidar;
  const sparse = Array(reading.rangesM.length);
  for (const change of [
    { rangesM: sparse },
    { rangesM: [0] },
    { channelCount: 33 },
    { azimuthCount: 12.5 },
    { quaternion: [2, 0, 0, 0] },
    { quaternion: Array(4) },
    { angleStartRad: 0 },
    { angleStepRad: 0.1 },
    { elevationStepRad: 0 },
    { elevationStartRad: Math.PI / 2 },
    { time: NaN },
    { valid: 1 },
    { reason: "fault" },
    { rangesM: reading.rangesM.map((value) => (value === null ? null : 201)) },
  ])
    assert.throws(() => lidarPoints({ ...reading, ...change }));
  const startup = createSensorSuite().snapshot().lidar;
  assert.throws(() => lidarPoints({ ...startup, azimuthCount: 180 }));
  assert.throws(() =>
    lidarPoints({ ...startup, rangesM: Array(startup.rangesM.length) }),
  );
  assert.throws(() => lidarPoints({ ...startup, reason: "no-return" }));
  assert.throws(() =>
    lidarPoints({
      ...startup,
      positionNed: [0, 0, -2],
      quaternion: [1, 0, 0, 0],
    }),
  );
  assert.throws(() => lidarPoints({ ...startup, reason: "dropout" }));
  assert.deepEqual(lidarPoints(startup), []);
});

test("GPS shares geographic projection and preserves declared vertical datum on valid, delayed and failed acquisitions", () => {
  for (const verticalDatum of ["unspecified", "ellipsoidal", "orthometric"]) {
    const origin = {
      latitudeDeg: 25.033,
      longitudeDeg: 179.999,
      altitudeM: 123,
      verticalDatum,
    };
    const config = {
      ...quiet,
      originLatitudeDeg: origin.latitudeDeg,
      originLongitudeDeg: origin.longitudeDeg,
      originAltitudeM: origin.altitudeM,
      originVerticalDatum: verticalDatum,
    };
    const suite = createSensorSuite(config);
    assert.equal(suite.snapshot().gps.verticalDatum, verticalDatum);
    const pose = state(0, { position: [-500, 500, -12] });
    const gps = suite.update(pose).gps;
    const expected = localToGeographic(-500, 500, origin);
    assert.equal(gps.latitudeDeg, expected.latitudeDeg);
    assert.equal(gps.longitudeDeg, expected.longitudeDeg);
    assert.equal(gps.altitudeM, 135);
    assert.equal(gps.verticalDatum, verticalDatum);
    suite.setFault("gps", true);
    const failed = suite.update(state(0.2)).gps;
    assert.equal(failed.valid, false);
    assert.equal(failed.verticalDatum, verticalDatum);
    assert.equal(failed.altitudeM, null);
    assert.equal(suite.reset().gps.verticalDatum, verticalDatum);
    const delayed = createSensorSuite({ ...config, latency: 0.1, dropout: 1 });
    assert.equal(delayed.update(state()).gps.reason, "startup");
    const dropped = delayed.update(state(0.1)).gps;
    assert.equal(dropped.reason, "dropout");
    assert.equal(dropped.verticalDatum, verticalDatum);
    assert.equal(dropped.latitudeDeg, null);
  }
  for (const value of [
    null,
    0,
    "msl",
    "Ellipsoidal",
    new String("orthometric"),
  ])
    assert.throws(
      () => validateSensorConfig({ originVerticalDatum: value }),
      /originVerticalDatum/,
    );
});

test("geographic sensor origin and datum must match DEM provenance atomically", () => {
  const geography = {
    origin: {
      latitudeDeg: 25.033,
      longitudeDeg: 121.5654,
      altitudeM: 20,
      verticalDatum: "orthometric",
    },
    source: {
      name: "terrain.tif",
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
  const config = validateSensorConfig({ originVerticalDatum: "orthometric" });
  const before = structuredClone({ geography, config });
  assert.doesNotThrow(() => assertGeographicSensors({ geography }, config));
  assert.doesNotThrow(() => assertGeographicSensors({ geography: null }, {}));
  for (const [key, value] of [
    ["originLatitudeDeg", 25],
    ["originLongitudeDeg", 121],
    ["originAltitudeM", 21],
    ["originVerticalDatum", "ellipsoidal"],
  ])
    assert.throws(
      () => assertGeographicSensors({ geography }, { ...config, [key]: value }),
      /must match/,
    );
  assert.deepEqual({ geography, config }, before);
});
