import test from "node:test";
import assert from "node:assert/strict";
import { calculateSurveyCamera, surveyFootprint } from "../survey-camera.js";
import { cameraMetadata } from "../camera.js";
import { createSimulation } from "../sim.js";
import { buildSurveyRoute } from "../survey.js";

const options = { altitudeM: 30, planeAltitudeM: 10, frontOverlapPct: 70, sideOverlapPct: 60 };
const close = (a, b) => assert.ok(Math.abs(a - b) < 1e-9, `${a} != ${b}`);

test("survey camera derives physical footprint, GSD and overlap distances without mutation", () => {
  const camera = Object.freeze({ width: 320, height: 240, horizontalFovDeg: 90 });
  const input = Object.freeze({ ...options });
  const plan = calculateSurveyCamera(camera, input);
  close(plan.heightAbovePlaneM, 20);
  close(plan.footprintAcrossM, 40);
  close(plan.footprintAlongM, 30);
  close(plan.gsdMPerPixel, 0.125);
  close(plan.spacingM, 16);
  close(plan.triggerDistanceM, 9);
  plan.cameraConfig.width = 64;
  assert.equal(camera.width, 320);
  assert.deepEqual(input, options);
});

test("footprint agrees with independent optical pixel-boundary projection at cardinal and oblique yaw", () => {
  for (const angleDeg of [0, 37, 90, 180]) {
    for (const config of [ {}, { width: 64, height: 480, horizontalFovDeg: 140 }, { width: 640, height: 48, horizontalFovDeg: 30 }]) {
      const plan = calculateSurveyCamera(config, { ...options, frontOverlapPct: config.height === 480 ? 80 : 0 }),
        snapshot = createSimulation().snapshot(),
        yaw = angleDeg * Math.PI / 180;
      snapshot.position = [13, -7, -options.altitudeM];
      snapshot.quaternion = [Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)];
      snapshot.gimbal.target = { ...snapshot.gimbal.target, yaw: 0, pitch: Math.PI / 2, roll: 0 };
      snapshot.gimbal.actual = { ...snapshot.gimbal.target };
      const metadata = cameraMetadata(snapshot, config, null, {
        sourceVehicleId: "drone-1", worldTick: 0, captureId: crypto.randomUUID(),
      });
      const projected = [[-0.5, -0.5], [metadata.width - 0.5, -0.5],
        [metadata.width - 0.5, metadata.height - 0.5], [-0.5, metadata.height - 0.5]]
        .map(([u, v]) => {
          const { fx, fy, cx, cy } = metadata.intrinsics,
            optical = [(u - cx) / fx, (v - cy) / fy, 1],
            matrix = metadata.pose.opticalToNed,
            ray = [0, 1, 2].map((row) => optical.reduce((sum, value, col) => sum + matrix[row * 3 + col] * value, 0)),
            distance = (-options.planeAltitudeM - snapshot.position[2]) / ray[2];
          return { north: snapshot.position[0] + distance * ray[0], east: snapshot.position[1] + distance * ray[1] };
        });
      const corners = surveyFootprint({ north: 13, east: -7 }, angleDeg, plan);
      corners.forEach((corner, i) => {
        close(corner.north, projected[i].north);
        close(corner.east, projected[i].east);
      });
      close(Math.hypot(corners[1].north - corners[0].north, corners[1].east - corners[0].east), plan.footprintAcrossM);
    }
  }
});

test("survey camera rejects malformed, unknown, nonfinite and out-of-range options", () => {
  for (const input of [null, [], new Date(), {}, { ...options, extra: 1 },
    Object.assign(Object.create({}), options), { ...options, [Symbol()]: 1 }])
    assert.throws(() => calculateSurveyCamera({}, input));
  for (const key of Object.keys(options)) {
    for (const value of [NaN, Infinity, -Infinity, "1", null, undefined])
      assert.throws(() => calculateSurveyCamera({}, { ...options, [key]: value }));
  }
  for (const change of [{ altitudeM: 1501 }, { planeAltitudeM: -1001 }, { frontOverlapPct: -1 },
    { sideOverlapPct: 96 }, { altitudeM: 10 }, { altitudeM: 9 }])
    assert.throws(() => calculateSurveyCamera({}, { ...options, ...change }));
  assert.throws(() => calculateSurveyCamera({ width: 1 }, options), /相機/);
  assert.doesNotThrow(() => calculateSurveyCamera({}, Object.assign(Object.create(null), options)));
});

test("clip boundaries are strict and derived distances are rejected rather than clamped", () => {
  for (const altitudeM of [5, 100])
    assert.throws(() => calculateSurveyCamera({ nearM: 5, farM: 100 }, { ...options, planeAltitudeM: 0, altitudeM }));
  assert.doesNotThrow(() => calculateSurveyCamera({ nearM: 5, farM: 100 }, { ...options, planeAltitudeM: 0, altitudeM: 5.001 }));
  assert.doesNotThrow(() => calculateSurveyCamera({ nearM: 5, farM: 100 }, { ...options, planeAltitudeM: 0, altitudeM: 99.999 }));
  for (const [config, change] of [
    [{}, { altitudeM: 10.2, sideOverlapPct: 95 }],
    [{ width: 640, height: 48 }, { altitudeM: 11, frontOverlapPct: 95 }],
    [{ farM: 500 }, { altitudeM: 400, sideOverlapPct: 0 }],
    [{ width: 64, height: 480 }, { altitudeM: 100, frontOverlapPct: 0 }],
  ]) assert.throws(() => calculateSurveyCamera(config, { ...options, ...change }));
  const shifted = calculateSurveyCamera({}, { ...options, altitudeM: -480, planeAltitudeM: -500 });
  close(shifted.heightAbovePlaneM, 20);
  for (const [altitudeM, planeAltitudeM] of [[-980, -1000], [1500, 1480]])
    close(calculateSurveyCamera({}, { ...options, altitudeM, planeAltitudeM }).heightAbovePlaneM, 20);
});

test("route spacing never exceeds camera-derived spacing", () => {
  const plan = calculateSurveyCamera({}, options),
    polygon = [{ north: 0, east: 0 }, { north: 100, east: 0 }, { north: 100, east: 75 }, { north: 0, east: 75 }];
  for (const angleDeg of [0, 37, 90, 180]) {
    const route = buildSurveyRoute(polygon, { spacingM: plan.spacingM, altitudeM: plan.altitudeM, angleDeg });
    assert.ok(route.actualSpacingM <= plan.spacingM);
    assert.ok(1 - route.actualSpacingM / plan.footprintAcrossM >= plan.sideOverlapPct / 100 - 1e-12);
  }
});

test("footprint validates geometry and leaves center and plan unchanged", () => {
  const plan = Object.freeze(calculateSurveyCamera({}, options)), center = Object.freeze({ north: 0, east: 0 });
  assert.equal(surveyFootprint(center, 0, plan).length, 4);
  for (const [point, angle, input] of [[null, 0, plan], [{ north: NaN, east: 0 }, 0, plan],
    [center, Infinity, plan], [center, 0, null], [center, 0, { footprintAlongM: -1, footprintAcrossM: 2 }]])
    assert.throws(() => surveyFootprint(point, angle, input));
});
