import { createLocalDirectory } from "../../companion/local-directory.js";
import test from "node:test";
import assert from "node:assert/strict";
import { writeFile, rm } from "node:fs/promises";
import { join } from "node:path";
import { spawnSync } from "node:child_process";
import { createWorld, defaultWorldConfig } from "../world.js";
import { DEFAULT_SCENE } from "../scene-data.js";
import { DEFAULT_GIMBAL_SETTINGS } from "../gimbal.js";
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
} from "../recording.js";
import { replayWorldRecording } from "../../tools/replay-world.mjs";
import { verifyInspectionReport } from "../../tools/verify-inspection.mjs";

const clone = structuredClone;
const uuid = (n) => `00000000-0000-4000-8000-${String(n).padStart(12, "0")}`;
const cameraConfig = {
  width: 64,
  height: 48,
  horizontalFovDeg: 90,
  nearM: 0.1,
  farM: 100,
};
function pixelFrame(metadata, marker = 1) {
  const count = metadata.width * metadata.height;
  return {
    ...metadata,
    rgbRgba: new Uint8Array(count * 4).fill(marker),
    depthM: new Float32Array(count),
    classes: new Uint8Array(count),
  };
}
function fixture(peerPosition = [0, 5]) {
  const scene = {
    ...clone(DEFAULT_SCENE),
    obstacles: [
      { baseAltitude: 0, north: 10, east: 0, width: 2, depth: 4, height: 4 },
    ],
    inspectionTargets: [
      {
        id: "panel",
        name: "Panel",
        obstacleIndex: 0,
        face: "south",
        u: 0,
        v: 0,
        width: 1,
        height: 1,
        minRangeM: 1,
        maxRangeM: 30,
        maxAngleDeg: 60,
        minPixels: 2,
        maxSpeedMps: 2,
      },
    ],
  };
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
      },
      scene,
    ),
  );
  if (peerPosition) {
    const peer = clone(input.vehicles[0]);
    peer.id = "drone-2";
    peer.name = "Peer";
    peer.spawn.north = peerPosition[0];
    peer.spawn.east = peerPosition[1];
    peer.home = [...peerPosition];
    input.vehicles.push(peer);
  }
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
      wallSeconds: 0.1,
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
    const state = world.snapshot(),
      index =
        record.samples.at(-1)?.tick === state.tick
          ? record.samples.length - 1
          : record.samples.length;
    record.samples[index] = compactWorldSnapshot(state);
    record.sensorFrames[index] = worldSensorFrame(state);
  }
  function command(type, value, vehicleId = "drone-1") {
    const { tick, time } = world.snapshot();
    if (type === "scene") world.setScene(value);
    else world.command(vehicleId, type === "operatorGimbal" ? "gimbal" : type, value);
    const event = {
      tick,
      time,
      type,
      ...(type === "scene" ? {} : { vehicleId }),
      value: clone(value),
    };
    if (type === "inspection")
      event.receipt = world.inspectionReport(vehicleId).receipts.at(-1);
    record.events.push(event);
    capture();
    return event;
  }
  async function inspect(id = 1, vehicleId = "drone-1") {
    const state = world.snapshot(),
      actor = state.vehicles.find((v) => v.id === vehicleId);
    const frame = pixelFrame(
      cameraMetadata(
        actor.snapshot,
        cameraConfig,
        world.configuration.scene.geography,
        {
          sourceVehicleId: vehicleId,
          worldTick: state.tick,
          captureId: uuid(id),
        },
      ),
      id,
    );
    const sha256 = await hashCameraFrame(frame);
    frames.push(frame);
    return command(
      "inspection",
      { captureId: uuid(id), targetId: "panel", config: cameraConfig, sha256 },
      vehicleId,
    );
  }
  function step(count) {
    for (let i = 0; i < count; i++) {
      world.step();
      if (world.snapshot().tick % 10 === 0) capture();
    }
  }
  capture();
  return { world, record, frames, capture, command, inspect, step };
}
async function observedFixture(peer) {
  const f = fixture(peer);
  f.step(7);
  await f.inspect();
  f.step(13);
  f.capture();
  return f;
}
const state = (record, frame = -1, actor = 0) =>
  record.samples.at(frame).vehicles[actor].snapshot;

test("inspection receipts retain the exact off-cadence exposure tick and independently replay every world state", async () => {
  const f = await observedFixture(),
    clean = validateRecording(f.record);
  assert.equal(RECORDING_VERSION, 22);
  assert.deepEqual(
    clean.samples.map((frame) => frame.tick),
    [0, 7, 10, 20],
  );
  const e = clean.events[0];
  assert.equal(e.type, "inspection");
  assert.equal(e.tick, 7);
  assert.equal(e.receipt.worldTick, 7);
  assert.equal(e.receipt.assessment.passed, true);
  assert.equal(state(clean).inspection.passed, 1);
  assert.deepEqual(
    validateRecording(JSON.parse(recordingToJson(f.record))),
    clean,
  );
  const replay = replayWorldRecording(clean);
  assert.equal(replay.passed, true);
  assert.equal(replay.ticks, 20);
  assert.equal(replay.maximum.allState, 0);
});

test("same-tick later gimbal settings and scene clearing preserve legitimate exposure event order", async () => {
  const f = fixture();
  f.step(7);
  const captured = await f.inspect();
  f.command("gimbal", {
    ...clone(DEFAULT_GIMBAL_SETTINGS),
    target: { yaw: 0.6, pitch: 0, roll: 0 },
  });
  f.command("scene", {
    ...clone(f.world.configuration.scene),
    environment: { ...f.world.configuration.scene.environment, fog: 0.2 },
  });
  f.step(13);
  f.capture();
  const clean = validateRecording(f.record);
  assert.equal(captured.receipt.metadata.gimbal.target.yaw, 0);
  assert.equal(state(clean, 1).gimbal.target.yaw, 0.6);
  assert.equal(state(clean).inspection.attempts, 0);
  assert.equal(state(clean).inspection.latest, null);
  assert.equal(replayWorldRecording(clean).passed, true);
});

test("same-tick operator gimbal before and after inspection and scene clearing preserve legitimate exposure event order", async () => {
  const f = fixture();
  f.step(7);
  f.command("operatorGimbal", { ...clone(DEFAULT_GIMBAL_SETTINGS), target: { yaw: 0.2, pitch: 0, roll: 0 } });
  const captured = await f.inspect();
  f.command("operatorGimbal", {
    ...clone(DEFAULT_GIMBAL_SETTINGS),
    target: { yaw: 0.6, pitch: 0, roll: 0 },
  });
  f.command("scene", {
    ...clone(f.world.configuration.scene),
    environment: { ...f.world.configuration.scene.environment, fog: 0.2 },
  });
  f.step(13);
  f.capture();
  const clean = validateRecording(f.record);
  assert.equal(captured.receipt.metadata.gimbal.target.yaw, 0.2);
  assert.equal(state(clean, 1).gimbal.target.yaw, 0.6);
  assert.equal(state(clean).inspection.attempts, 0);
  assert.equal(state(clean).inspection.latest, null);
  assert.equal(replayWorldRecording(clean).passed, true);
});

test("recording rejects forged inspection metadata, configuration, assessment, summaries and absent exposure evidence", async () => {
  const f = await observedFixture();
  const mutations = [
    (r) => {
      delete r.events[0].receipt;
    },
    (r) => {
      r.events[0].receipt.metadata.pose.positionNed[0] += 1;
    },
    (r) => {
      r.events[0].receipt.metadata.intrinsics.fx += 1;
    },
    (r) => {
      r.events[0].receipt.metadata.worldTick++;
    },
    (r) => {
      r.events[0].value.config.horizontalFovDeg = 80;
    },
    (r) => {
      r.events[0].receipt.assessment.rangeM += 1;
    },
    (r) => {
      r.events[0].receipt.assessment.reasons = {};
    },
    (r) => {
      r.events[0].receipt.assessment.visibleSamples = 24;
    },
    (r) => {
      r.events[0].receipt.sourceVehicleId = "drone-2";
    },
    (r) => {
      r.events[0].receipt.sha256 = "b".repeat(64);
    },
    (r) => {
      state(r).inspection.attempts++;
    },
    (r) => {
      delete state(r).inspection;
    },
    (r) => {
      const index = r.samples.findIndex((frame) => frame.tick === 7);
      r.samples.splice(index, 1);
      r.sensorFrames.splice(index, 1);
    },
  ];
  for (const mutate of mutations) {
    const bad = clone(f.record);
    mutate(bad);
    assert.throws(() => validateRecording(bad), undefined, mutate.toString());
  }
});

test("receipt geometry depends on the captured peer positions and capture IDs are globally unique", async () => {
  const f = await observedFixture([4, 0]);
  assert.equal(f.record.events[0].receipt.assessment.passed, false);
  assert.ok(f.record.events[0].receipt.assessment.reasons.includes("occluded"));
  const wrongPeer = clone(f.record),
    exposure = wrongPeer.samples.find((frame) => frame.tick === 7);
  exposure.vehicles[1].snapshot.position = [0, 5, -2];
  assert.throws(() => validateRecording(wrongPeer), /geometry/);
  const other = fixture();
  other.step(7);
  await other.inspect(1);
  await other.inspect(2, "drone-2");
  const duplicate = clone(other.record),
    second = duplicate.events[1];
  second.value.captureId =
    second.receipt.captureId =
    second.receipt.metadata.captureId =
      uuid(1);
  assert.throws(() => validateRecording(duplicate), /capture|duplicate/i);
});

test("standalone inspection verification matches actual dataset bytes and metadata without asserting GPU origin", async () => {
  const f = await observedFixture(),
    report = f.world.inspectionReport("drone-1");
  const text = cameraDatasetToJson(f.frames),
    result = await verifyInspectionReport(report, text);
  assert.deepEqual(result, {
    passed: true,
    sourceVehicleId: "drone-1",
    receipts: 1,
    matched: 1,
    missingCaptureIds: [],
    mismatchedCaptureIds: [],
  });
  const missing = await verifyInspectionReport(report, cameraDatasetToJson([]));
  assert.equal(missing.passed, false);
  assert.deepEqual(missing.missingCaptureIds, [uuid(1)]);
  const changed = clone(f.frames);
  changed[0].rgbRgba[0] ^= 1;
  const mismatch = await verifyInspectionReport(
    report,
    cameraDatasetToJson(changed),
  );
  assert.equal(mismatch.passed, false);
  assert.deepEqual(mismatch.mismatchedCaptureIds, [uuid(1)]);
  const changedMetadata = clone(report);
  changedMetadata.receipts[0].metadata.pose.positionNed[0] += 0.1;
  const metadataMismatch = await verifyInspectionReport(changedMetadata, text);
  assert.equal(metadataMismatch.passed, false);
  assert.deepEqual(metadataMismatch.mismatchedCaptureIds, [uuid(1)]);
  const empty = fixture().world.inspectionReport("drone-1");
  assert.equal(
    (await verifyInspectionReport(empty, cameraDatasetToJson([]))).passed,
    false,
  );
});

test("standalone inspection report schemas reject bad identities, hashes, capture lists and summaries", async () => {
  const f = await observedFixture(),
    report = f.world.inspectionReport("drone-1"),
    text = cameraDatasetToJson(f.frames);
  for (const mutate of [
    (r) => {
      r.version = 1;
    },
    (r) => {
      r.extra = true;
    },
    (r) => {
      r.sourceVehicleId = "=formula";
    },
    (r) => {
      r.receipts = Array(1);
    },
    (r) => {
      r.receipts.push(clone(r.receipts[0]));
    },
    (r) => {
      r.receipts[0].sha256 = "A".repeat(64);
    },
    (r) => {
      r.receipts[0].sourceVehicleId = "drone-2";
    },
    (r) => {
      r.receipts[0].metadata.captureId = uuid(2);
    },
    (r) => {
      r.receipts[0].metadata.worldTick++;
    },
    (r) => {
      r.receipts[0].targetId = "missing";
    },
    (r) => {
      r.receipts[0].assessment.reasons = {};
    },
    (r) => {
      r.receipts[0].assessment.passed = false;
    },
    (r) => {
      r.summary.passed = 0;
    },
  ]) {
    const bad = clone(report);
    mutate(bad);
    await assert.rejects(
      verifyInspectionReport(bad, text),
      undefined,
      mutate.toString(),
    );
  }
  await assert.rejects(verifyInspectionReport(report, "{}"));
});

test("inspection verification CLI returns useful success, mismatch and missing-file exit statuses", async (t) => {
  const f = await observedFixture(),
    directory = await createLocalDirectory("inspection-verify-");
  t.after(() => rm(directory, { recursive: true, force: true }));
  const report = join(directory, "report.json"),
    dataset = join(directory, "frames.json");
  await writeFile(report, JSON.stringify(f.world.inspectionReport("drone-1")));
  await writeFile(dataset, cameraDatasetToJson(f.frames));
  const cli = new URL("../../tools/verify-inspection.mjs", import.meta.url);
  const run = (...args) =>
    spawnSync(process.execPath, [cli.pathname, ...args], {
      encoding: "utf8",
      timeout: 5000,
    });
  const success = run(report, dataset);
  assert.equal(success.status, 0, success.stderr);
  assert.equal(JSON.parse(success.stdout).passed, true);
  await writeFile(dataset, cameraDatasetToJson([]));
  const missing = run(report, dataset);
  assert.equal(missing.status, 1);
  assert.deepEqual(JSON.parse(missing.stdout).missingCaptureIds, [uuid(1)]);
  const unavailable = run(report, join(directory, "absent.json"));
  assert.equal(unavailable.status, 1);
  assert.match(unavailable.stderr, /ENOENT/);
  assert.equal(run().status, 1);
});
