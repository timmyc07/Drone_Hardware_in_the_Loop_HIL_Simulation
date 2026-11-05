import test from "node:test";
import assert from "node:assert/strict";
import { createWorld, defaultWorldConfig } from "../world.js";
import { DEFAULT_SCENE, sceneToJson, parseSceneJson } from "../scene-data.js";
import { DEFAULT_GIMBAL_SETTINGS } from "../gimbal.js";
import {
  inspectionTemplate,
  INSPECTION_REPORT_VERSION,
} from "../inspection.js";
import {
  cameraMetadata,
  cameraDatasetToJson,
  hashCameraFrame,
} from "../camera.js";
import {
  RECORDING_VERSION,
  MODEL_VERSION,
  compactWorldSnapshot,
  worldSensorFrame,
  validateRecording,
  recordingToJson,
  recordingInspectionReport,
} from "../recording.js";
import { replayWorldRecording } from "../../tools/replay-world.mjs";
import { verifyInspectionReport } from "../../tools/verify-inspection.mjs";
import { getVehicleEnvelope } from "../peer-geometry.js";

const clone = structuredClone;
const cameraConfig = {
  width: 64,
  height: 48,
  horizontalFovDeg: 100,
  nearM: 0.1,
  farM: 100,
};
const uuid = (n) => `00000000-0000-4000-8000-${String(n).padStart(12, "0")}`;

function fixture() {
  const scene = { ...clone(DEFAULT_SCENE), ...inspectionTemplate("bridge") };
  const input = clone(
    defaultWorldConfig(
      {},
      {
        sensors: {
          lidarRays: 12,
          lidarChannels: 1,
          lidarVerticalMinDeg: 0,
          lidarVerticalMaxDeg: 0,
          radarAzimuthRays: 1,
          radarElevationRays: 1,
        },
        gimbal: {
          ...clone(DEFAULT_GIMBAL_SETTINGS),
          target: { yaw: 0, pitch: -Math.PI / 2, roll: 0 },
        },
      },
      scene,
    ),
  );
  input.vehicles[0].spawn = { north: 20, east: 0, altitude: 4, heading: 0 };
  input.vehicles[0].home = [20, 0];
  const peer = clone(input.vehicles[0]);
  peer.id = "drone-2";
  peer.name = "橋外同伴";
  peer.spawn.east = 12;
  peer.home = [20, 12];
  input.vehicles.push(peer);
  const world = createWorld(input),
    frames = [];
  const record = {
    format: "flight-lab",
    version: RECORDING_VERSION,
    modelVersion: MODEL_VERSION,
    createdAt: "2026-11-05T00:00:00.000Z",
    dt: 0.005,
    world: clone(world.configuration),
    runtime: {
      speed: 1,
      wallSeconds: 0.2,
      p95LateMs: 0,
      lateTicks: 0,
      skippedWallSeconds: 0,
      renderEnabled: true,
    },
    events: [],
    samples: [],
    sensorFrames: [],
    termination: null,
  };
  function capture() {
    const state = world.snapshot();
    const index =
      record.samples.at(-1)?.tick === state.tick
        ? record.samples.length - 1
        : record.samples.length;
    record.samples[index] = compactWorldSnapshot(state);
    record.sensorFrames[index] = worldSensorFrame(state);
  }
  function step(count) {
    for (let i = 0; i < count; i++) world.step();
  }
  function changeScene(value, retain = true) {
    const { tick, time } = world.snapshot();
    world.setScene(value);
    record.events.push({ tick, time, type: "scene", value: clone(value) });
    if (retain) capture();
  }
  async function inspect(targetId, id) {
    const state = world.snapshot(),
      source = state.vehicles[0];
    const metadata = cameraMetadata(
      source.snapshot,
      cameraConfig,
      scene.geography,
      {
        sourceVehicleId: source.id,
        worldTick: state.tick,
        captureId: uuid(id),
      },
    );
    const count = metadata.width * metadata.height;
    // Synthetic typed pixels exercise the hashing protocol; they are not GPU evidence.
    const frame = {
      ...metadata,
      rgbRgba: new Uint8Array(count * 4),
      depthM: new Float32Array(count),
      classes: new Uint8Array(count),
    };
    frames.push(frame);
    const value = {
      captureId: uuid(id),
      targetId,
      config: cameraConfig,
      sha256: await hashCameraFrame(frame),
    };
    world.command(source.id, "inspection", value);
    record.events.push({
      tick: state.tick,
      time: state.time,
      type: "inspection",
      vehicleId: source.id,
      value: clone(value),
      receipt: world.inspectionReport(source.id).receipts.at(-1),
    });
    capture();
  }
  capture();
  return { world, record, frames, capture, step, changeScene, inspect };
}

async function observed() {
  const f = fixture();
  f.step(7);
  await f.inspect("bridge-under-south", 1);
  await f.inspect("bridge-under-north", 2);
  return f;
}
function safeScene(f) {
  const next = clone(f.world.configuration.scene);
  next.obstacles[0].baseAltitude = 9;
  return next;
}
const inspection = (record, index = -1) =>
  record.samples.at(index).vehicles[0].snapshot.inspection;

test("bridge record22 preserves two actual upward exposures at tick7 and independently replays all world states", async () => {
  const f = await observed();
  f.step(13);
  f.capture();
  const clean = validateRecording(f.record);
  assert.equal(RECORDING_VERSION, 22);
  assert.equal(MODEL_VERSION, "0.16.0");
  assert.deepEqual(
    clean.samples.map((frame) => frame.tick),
    [0, 7, 20],
  );
  assert.equal(clean.samples[1].vehicles.length, 2);
  for (const event of clean.events) {
    assert.equal(event.tick, 7);
    assert.equal(event.receipt.worldTick, 7);
    assert.equal(event.receipt.metadata.gimbal.actual.pitch, -Math.PI / 2);
    assert.equal(event.receipt.assessment.passed, true);
    assert.equal(event.receipt.assessment.visibleSamples, 25);
  }
  assert.equal(inspection(clean).passed, 2);
  assert.equal(inspection(clean).attempts, 2);
  assert.equal(inspection(clean).total, 4);
  assert.deepEqual(
    validateRecording(JSON.parse(recordingToJson(f.record))),
    clean,
  );
  const replay = replayWorldRecording(clean);
  assert.equal(replay.passed, true);
  assert.equal(replay.maximum.allState, 0);
  assert.equal(replay.maximum.sensors, 0);
  const report = recordingInspectionReport(clean, 1, "drone-1");
  assert.equal(report.version, INSPECTION_REPORT_VERSION);
  assert.equal(report.version, 2);
  assert.deepEqual(report, f.world.inspectionReport("drone-1"));
  const verification = await verifyInspectionReport(
    report,
    cameraDatasetToJson(f.frames),
  );
  assert.equal(verification.passed, true);
  assert.equal(verification.matched, 2);
});

test("safe exact-tick bridge geometry changes clear inspection receipts and replay correctly", async () => {
  const f = await observed();
  f.step(1);
  f.changeScene(safeScene(f));
  f.step(12);
  f.capture();
  const clean = validateRecording(f.record);
  assert.deepEqual(
    clean.samples.map((frame) => frame.tick),
    [0, 7, 8, 20],
  );
  assert.equal(inspection(clean, 1).passed, 2);
  assert.equal(inspection(clean, 2).attempts, 0);
  assert.equal(inspection(clean).latest, null);
  const before = recordingInspectionReport(clean, 1, "drone-1"),
    after = recordingInspectionReport(clean, 2, "drone-1");
  assert.equal(before.receipts.length, 2);
  assert.equal(after.receipts.length, 0);
  assert.equal(after.scene.obstacles[0].baseAltitude, 9);
  assert.equal(replayWorldRecording(clean).passed, true);
});

test("a safe bridge change at unretained tick8 is rejected even when next retained frame20 is coherent", async () => {
  const f = await observed();
  f.step(1);
  f.changeScene(safeScene(f), false);
  f.step(12);
  f.capture();
  assert.deepEqual(
    f.record.samples.map((frame) => frame.tick),
    [0, 7, 20],
  );
  assert.equal(inspection(f.record).attempts, 0);
  assert.throws(() => validateRecording(f.record), /retained world tick/);
});

test("unsafe bridge changes intersecting either vehicle are rejected atomically and as forged recordings", async () => {
  const f = await observed();
  const unsafeSource = clone(f.world.configuration.scene);
  unsafeSource.obstacles[0].baseAltitude = 3.5;
  const unsafePeer = clone(f.world.configuration.scene);
  unsafePeer.obstacles.push({
    north: 20,
    east: 12,
    width: 2,
    depth: 2,
    height: 2,
    baseAltitude: 3,
  });
  for (const next of [unsafeSource, unsafePeer]) {
    const before = clone({
      snapshot: f.world.snapshot(),
      config: f.world.configuration,
      report: f.world.inspectionReport("drone-1"),
    });
    assert.throws(() => f.world.setScene(next), /intersect|obstacle/i);
    assert.deepEqual(
      {
        snapshot: f.world.snapshot(),
        config: f.world.configuration,
        report: f.world.inspectionReport("drone-1"),
      },
      before,
    );
  }
  f.step(1);
  f.changeScene(safeScene(f));
  f.step(12);
  f.capture();
  validateRecording(f.record);
  for (const next of [unsafeSource, unsafePeer]) {
    const bad = clone(f.record);
    bad.events.at(-1).value = next;
    assert.throws(
      () => validateRecording(bad),
      /intersects a retained vehicle envelope/,
    );
  }
});

test("recorded scene changes reject conservative corner overlaps and accept clear bounds", async () => {
  const f = await observed();
  f.step(1);
  const position = f.world.snapshot().vehicles[0].snapshot.position;
  const radius = getVehicleEnvelope(
    f.world.configuration.vehicles[0].config,
  ).radius;
  const clear = safeScene(f);
  clear.obstacles.push({
    north: position[0] + radius + 0.51,
    east: position[1] + 0.8 * radius + 0.5,
    width: 1,
    depth: 1,
    height: 1,
    baseAltitude: -position[2] - 0.5,
  });
  f.changeScene(clear);
  f.step(12);
  f.capture();
  assert.doesNotThrow(() => validateRecording(f.record));
  const bad = clone(f.record);
  const corner = bad.events.at(-1).value.obstacles.at(-1);
  corner.north = position[0] + 0.8 * radius + 0.5;
  assert.ok(Math.hypot(0.8 * radius, 0.8 * radius) > radius);
  assert.throws(
    () => validateRecording(bad),
    /intersects a retained vehicle envelope/,
  );
});

test("bridge artifacts reject missing base, Scene5, record15 and report1 without compatibility conversion", async () => {
  const f = await observed();
  f.step(13);
  f.capture();
  const bad = clone(f.record);
  delete bad.world.scene.obstacles[0].baseAltitude;
  assert.throws(() => validateRecording(bad), /obstacle/i);
  assert.throws(
    () => validateRecording({ ...f.record, version: 15 }),
    /version/i,
  );
  const sceneArtifact = JSON.parse(sceneToJson(f.world.configuration.scene));
  sceneArtifact.version = 5;
  assert.throws(
    () => parseSceneJson(JSON.stringify(sceneArtifact)),
    /artifact|version/i,
  );
  const report = f.world.inspectionReport("drone-1");
  await assert.rejects(
    verifyInspectionReport(
      { ...report, version: 1 },
      cameraDatasetToJson(f.frames),
    ),
  );
});
