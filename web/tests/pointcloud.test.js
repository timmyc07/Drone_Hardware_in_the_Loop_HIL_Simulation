import test from "node:test";
import assert from "node:assert/strict";
import { PCDLoader } from "three/addons/loaders/PCDLoader.js";
import { createSensorSuite, lidarPoints } from "../sensors.js";
import { exportLidarCsv, exportLidarPcd } from "../pointcloud.js";

const input = {
  time: 0.4,
  position: [3, -2, -4],
  velocity: [0, 0, 0],
  quaternion: [Math.cos(0.2), 0, 0, Math.sin(0.2)],
  rates: [0, 0, 0],
};
function scan() {
  const suite = createSensorSuite({
    lidarChannels: 3,
    lidarRays: 12,
    lidarVerticalMinDeg: -45,
    lidarVerticalMaxDeg: 45,
    lidarNoiseM: 0,
  });
  suite.setObstacles([{ north: 8, east: -2, width: 2, depth: 10, height: 10 }]);
  return suite.update(input).lidar;
}

test("CSV and PCD preserve channel order, acquisition pose and exact NED point values", () => {
  const reading = scan();
  const expected = lidarPoints(reading);
  assert.ok(expected.length > 12);
  const rows = exportLidarCsv(reading)
    .trim()
    .split("\n")
    .map((row) => row.split(","));
  assert.deepEqual(rows.shift(), [
    "time_s",
    "channel",
    "azimuth_rad",
    "elevation_rad",
    "range_m",
    "north_m",
    "east_m",
    "down_m",
  ]);
  assert.equal(rows.length, expected.length);
  const indices = reading.rangesM.flatMap((range, i) =>
    range === null ? [] : [i],
  );
  rows.forEach((row, i) => {
    const values = row.map(Number);
    assert.equal(values[0], input.time);
    assert.equal(values[1], Math.floor(indices[i] / reading.azimuthCount));
    assert.equal(values[4], reading.rangesM[indices[i]]);
    assert.deepEqual(values.slice(5), expected[i]);
  });
  const pcd = exportLidarPcd(reading);
  assert.match(pcd, /^# frame NED/m);
  assert.ok(
    pcd.includes(
      `VIEWPOINT ${[...input.position, ...input.quaternion].join(" ")}`,
    ),
  );
  assert.ok(pcd.includes(`POINTS ${expected.length}\nDATA ascii\n`));
  const lines = pcd.split("DATA ascii\n")[1].trim().split("\n");
  lines.forEach((line, i) => {
    const values = line.split(" ").map(Number);
    assert.deepEqual(values.slice(0, 3), expected[i]);
    assert.equal(values[3], Number(rows[i][1]));
    assert.equal(values[4], Number(rows[i][4]));
  });
  // Read our export with an independently maintained, already installed PCD reader.
  const cloud = new PCDLoader().parse(new TextEncoder().encode(pcd).buffer);
  const position = cloud.geometry.getAttribute("position");
  assert.equal(position.count, expected.length);
  expected
    .flat()
    .forEach((value, i) =>
      assert.ok(Math.abs(position.array[i] - value) < 2e-6),
    );
  cloud.geometry.dispose();
  cloud.material.dispose();
});

test("empty and invalid scans cannot invent point returns or inject a PCD header", () => {
  const suite = createSensorSuite();
  const startup = suite.snapshot().lidar;
  assert.equal(exportLidarCsv(startup).trim().split("\n").length, 1);
  assert.match(exportLidarPcd(startup), /POINTS 0\nDATA ascii\n$/);
  assert.match(exportLidarPcd(startup), /acquisition_pose unavailable/);
  for (const mutate of [
    (r) => (r.time = "0\nDATA binary"),
    (r) => (r.quaternion = [0, 0, 0, 0]),
    (r) => (r.channelCount = 100000),
    (r) => delete r.rangesM[0],
    (r) => (r.positionNed = [0, "bad", 0]),
    (r) => (r.angleStepRad = 1),
  ]) {
    const reading = scan();
    mutate(reading);
    assert.throws(() => exportLidarPcd(reading));
    assert.throws(() => exportLidarCsv(reading));
  }
});
