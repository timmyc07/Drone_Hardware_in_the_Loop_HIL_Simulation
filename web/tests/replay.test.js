import test from "node:test";
import assert from "node:assert/strict";
import {
  replayFrameAtTime,
  replayEventFrame,
  replayEventIndices,
  adjacentReplayEvent,
} from "../replay.js";
import { createWorld, defaultWorldConfig } from "../world.js";
import { DEFAULT_SCENE } from "../scene-data.js";
import {
  RECORDING_VERSION,
  MODEL_VERSION,
  compactWorldSnapshot,
  worldSensorFrame,
  validateRecording,
  recordingWorldAt,
  recordingInspectionReport,
} from "../recording.js";

const timeline = () => ({
  samples: [0, 7, 10, 20].map((tick) => ({ tick, time: tick * 0.005 })),
  events: [
    { tick: 0, type: "arm", vehicleId: "alpha" },
    { tick: 7, type: "inspection", vehicleId: "alpha" },
    { tick: 7, type: "gimbal", vehicleId: "alpha" },
    { tick: 7, type: "wind" },
    { tick: 10, type: "gimbal", vehicleId: "beta" },
    { tick: 20, type: "wind" },
  ],
});

test("time seeks use the actual uneven sample times, floor gaps and clamp endpoints", () => {
  const record = timeline();
  for (const [time, expected] of [
    [-10, 0],
    [0, 0],
    [0.034, 0],
    [0.035, 1],
    [0.049, 1],
    [0.05, 2],
    [0.099, 2],
    [0.1, 3],
    [100, 3],
  ])
    assert.equal(replayFrameAtTime(record, time), expected, `${time} seconds`);
  const shifted = {
    samples: record.samples.map((s) => ({ ...s, time: s.time + 12 })),
  };
  assert.equal(replayFrameAtTime(shifted, 0), 0);
  assert.equal(replayFrameAtTime(shifted, 12.04), 1);
  assert.equal(replayFrameAtTime(shifted, 100), 3);
  for (const value of [NaN, Infinity, -Infinity, undefined, null, "0"])
    assert.throws(() => replayFrameAtTime(record, value), /time/);
  assert.throws(() => replayFrameAtTime({ samples: [] }, 0), /frames/);
  assert.equal(replayFrameAtTime({ samples: [{ time: 12 }] }, 99), 0);
});

test("event seeks preserve original indices and same-tick final frame identity", () => {
  const record = timeline();
  assert.deepEqual(
    record.events.map((_, i) => replayEventFrame(record, i)),
    [0, 1, 1, 1, 2, 3],
  );
  record.events.splice(4, 0, { tick: 8, type: "gust", vehicleId: "alpha" });
  assert.equal(replayEventFrame(record, 4), 2);
  for (const value of [-1, 0.5, 100, NaN, undefined, "0"])
    assert.throws(() => replayEventFrame(record, value), /event index/);
  assert.throws(
    () => replayEventFrame({ ...record, samples: [] }, 0),
    /frames/,
  );
});

test("filters intersect type and exact vehicle while global selection cannot collide with vehicle IDs", () => {
  const record = timeline();
  assert.deepEqual(replayEventIndices(record), [0, 1, 2, 3, 4, 5]);
  assert.deepEqual(replayEventIndices(record, { type: "gimbal" }), [2, 4]);
  assert.deepEqual(
    replayEventIndices(record, { vehicleId: "alpha" }),
    [0, 1, 2],
  );
  assert.deepEqual(
    replayEventIndices(record, { type: "gimbal", vehicleId: "alpha" }),
    [2],
  );
  assert.deepEqual(replayEventIndices(record, { vehicleId: null }), [3, 5]);
  assert.deepEqual(
    replayEventIndices(record, { type: "arm", vehicleId: null }),
    [],
  );
  assert.deepEqual(replayEventIndices(record, { vehicleId: "__world__" }), []);
  assert.deepEqual(replayEventIndices(record, { type: "unknown" }), []);
});

test("adjacent navigation walks duplicate ticks in source order without getting stuck", () => {
  const record = timeline(),
    indices = replayEventIndices(record);
  const next = (frame, direction, selected) =>
    adjacentReplayEvent(record, indices, frame, direction, selected);
  assert.equal(next(1, 1), 1);
  assert.equal(next(1, -1), 3);
  assert.equal(next(1, 1, 1), 2);
  assert.equal(next(1, 1, 2), 3);
  assert.equal(next(1, 1, 3), 4);
  assert.equal(next(1, -1, 3), 2);
  assert.equal(next(1, -1, 2), 1);
  assert.equal(next(1, -1, 1), 0);
  assert.equal(next(0, -1, 0), null);
  assert.equal(next(3, 1, 5), null);
  assert.equal(
    next(2, 1, 1),
    4,
    "a stale selected event does not override current time",
  );
  assert.equal(
    adjacentReplayEvent(record, [2, 4], 1, 1, 1),
    2,
    "a filtered-out selection is ignored",
  );
  assert.equal(adjacentReplayEvent(record, [], 0, 1), null);
  for (const direction of [0, 2, "1", NaN])
    assert.throws(
      () => adjacentReplayEvent(record, indices, 0, direction),
      /direction/,
    );
  for (const index of [-1, 0.5, 4, NaN])
    assert.throws(
      () => adjacentReplayEvent(record, indices, index, 1),
      /frame index/,
    );
  for (const selected of [-2, 0.5, 6, NaN])
    assert.throws(() => next(0, 1, selected), /selected/);
});

test("sparse samples keep events atomic even when several event ticks map to one frame", () => {
  const record = timeline();
  record.events = [7, 8, 9, 10].map((tick) => ({ tick, type: "gust" }));
  const indices = replayEventIndices(record);
  assert.deepEqual(
    indices.map((i) => replayEventFrame(record, i)),
    [1, 2, 2, 2],
  );
  assert.equal(adjacentReplayEvent(record, indices, 2, 1, 1), 2);
  assert.equal(adjacentReplayEvent(record, indices, 2, -1, 2), 1);
  assert.equal(adjacentReplayEvent(record, indices, 2, 1), 3);
});

test("navigation and filtering leave frozen source arrays untouched", () => {
  const record = timeline();
  for (const sample of record.samples) Object.freeze(sample);
  for (const event of record.events) Object.freeze(event);
  Object.freeze(record.samples);
  Object.freeze(record.events);
  Object.freeze(record);
  const indices = Object.freeze(
    replayEventIndices(record, { vehicleId: "alpha" }),
  );
  assert.equal(replayFrameAtTime(record, 0.04), 1);
  assert.equal(replayEventFrame(record, 2), 1);
  assert.equal(adjacentReplayEvent(record, indices, 1, 1, 1), 2);
  assert.deepEqual(record, timeline());
});

test("frame and adjacent event searches stay logarithmic for 36,001 frames and 100,000 events", () => {
  let reads = 0;
  const counted = (array) =>
    new Proxy(array, {
      get(target, key, receiver) {
        if (/^\d+$/.test(String(key))) reads++;
        return Reflect.get(target, key, receiver);
      },
    });
  const record = {
    samples: counted(
      Array.from({ length: 36001 }, (_, i) => ({
        tick: i * 10,
        time: i * 0.05,
      })),
    ),
    events: counted(
      Array.from({ length: 100000 }, (_, i) => ({ tick: i * 3, type: "gust" })),
    ),
  };
  const indices = counted(Array.from({ length: 100000 }, (_, i) => i));
  assert.equal(replayFrameAtTime(record, 900.024), 18000);
  assert.ok(reads < 20, `${reads} sample reads`);
  reads = 0;
  assert.equal(replayEventFrame(record, 60001), 18001);
  assert.ok(reads < 40, `${reads} frame/event reads`);
  reads = 0;
  assert.equal(adjacentReplayEvent(record, indices, 18000, 1), 60000);
  assert.ok(reads < 80, `${reads} frame/event/index reads`);
  reads = 0;
  assert.equal(adjacentReplayEvent(record, indices, 18001, 1, 60001), 60002);
  assert.ok(reads < 80, `${reads} selected event reads`);
});

function worldRecord() {
  const scene = {
    ...structuredClone(DEFAULT_SCENE),
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
  const world = createWorld(
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
  const record = {
    format: "flight-lab",
    version: RECORDING_VERSION,
    modelVersion: MODEL_VERSION,
    createdAt: "2026-11-05T00:00:00.000Z",
    dt: 0.005,
    world: structuredClone(world.configuration),
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
    const state = world.snapshot();
    const index =
      record.samples.at(-1)?.tick === state.tick
        ? record.samples.length - 1
        : record.samples.length;
    record.samples[index] = compactWorldSnapshot(state);
    record.sensorFrames[index] = worldSensorFrame(state);
  }
  function stepTo(tick) {
    while (world.snapshot().tick < tick) world.step();
    capture();
  }
  function command(type, value) {
    const { tick, time } = world.snapshot();
    const global = type === "scene" || type === "wind";
    if (type === "scene") world.setScene(value);
    else if (type === "wind") world.setWind(value);
    else world.command("drone-1", type, value);
    const event = {
      tick,
      time,
      type,
      value: structuredClone(value),
      ...(!global ? { vehicleId: "drone-1" } : {}),
    };
    if (type === "inspection")
      event.receipt = world.inspectionReport("drone-1").receipts.at(-1);
    record.events.push(event);
    capture();
  }
  capture();
  stepTo(7);
  // Receipt metadata tests world history; no synthetic pixels are presented as GPU evidence.
  command("inspection", {
    captureId: "00000000-0000-4000-8000-000000000001",
    targetId: "panel",
    config: {
      width: 64,
      height: 48,
      horizontalFovDeg: 90,
      nearM: 0.1,
      farM: 100,
    },
    sha256: "a".repeat(64),
  });
  command("gimbal", {
    ...structuredClone(world.configuration.vehicles[0].setup.gimbal),
    target: { yaw: 0.3, pitch: -0.1, roll: 0 },
  });
  command("wind", { speed: 3, direction: 45 });
  command("mission", [{ north: 1, east: 1, altitude: 3, hold: 0 }]);
  command("training", "hover");
  command("sensorConfig", {
    ...world.configuration.vehicles[0].setup.sensors,
    lidarRays: 24,
  });
  stepTo(10);
  stepTo(20);
  command("scene", {
    ...structuredClone(world.configuration.scene),
    name: "Updated scene",
    environment: { ...world.configuration.scene.environment, fog: 0.2 },
  });
  return validateRecording(record);
}

test("backward seeks restore recorded world, acquisition, training and inspection epochs without mutation", () => {
  const record = worldRecord(),
    before = structuredClone(record);
  assert.deepEqual(
    record.samples.map((frame) => frame.tick),
    [0, 7, 10, 20],
  );
  const at = (time) =>
    recordingWorldAt(record, replayFrameAtTime(record, time));
  const last = at(100),
    initial = at(0),
    exposure = at(0.035),
    middle = at(0.05);
  assert.equal(last.scene.name, "Updated scene");
  assert.equal(last.vehicles[0].inspection.attempts, 0);
  assert.equal(initial.scene.name, record.world.scene.name);
  assert.deepEqual(initial.wind, { speed: 0, direction: 0 });
  assert.deepEqual(initial.vehicles[0].setup.mission, []);
  assert.equal(initial.vehicles[0].training.course, "free");
  assert.equal(initial.vehicles[0].setup.sensors.lidarRays, 12);
  assert.equal(exposure.tick, 7);
  assert.deepEqual(exposure.wind, { speed: 3, direction: 45 });
  assert.equal(exposure.vehicles[0].setup.mission.length, 1);
  assert.equal(exposure.vehicles[0].setup.gimbal.target.yaw, 0.3);
  assert.equal(exposure.vehicles[0].training.course, "hover");
  assert.equal(exposure.vehicles[0].setup.sensors.lidarRays, 24);
  assert.equal(middle.vehicles[0].sensors.lidar.rangesM.length, 24);
  assert.deepEqual(
    middle.vehicles[0].snapshot,
    record.samples[2].vehicles[0].snapshot,
  );
  assert.deepEqual(
    middle.vehicles[0].sensors,
    record.sensorFrames[2].vehicles[0].sensors,
  );
  const report = (frame) => recordingInspectionReport(record, frame, "drone-1");
  assert.equal(report(3).receipts.length, 0);
  assert.equal(report(1).receipts.length, 1);
  assert.equal(report(0).receipts.length, 0);
  assert.deepEqual(at(0.035), exposure);
  assert.deepEqual(record, before);
});
