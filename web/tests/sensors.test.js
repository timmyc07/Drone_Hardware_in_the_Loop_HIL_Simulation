import test from "node:test";
import assert from "node:assert/strict";
import {
  createSensorSuite,
  validateSensorConfig,
  DEFAULT_SENSOR_CONFIG,
  lidarPoints,
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
    const suite = createSensorSuite({ ...quiet, lidarRays: count });
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
  const suite = createSensorSuite({ ...quiet, lidarRays: 36 });
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
