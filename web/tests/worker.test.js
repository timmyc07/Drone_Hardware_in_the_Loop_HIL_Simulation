import test from "node:test";
import assert from "node:assert/strict";
import { Worker } from "node:worker_threads";
import { createWorld, defaultWorldConfig } from "../world.js";
import { validateRecording } from "../recording.js";
import { DEFAULT_CAMERA_CONFIG, hashCameraFrame } from "../camera.js";

async function harness(t, deferHash = false) {
  const url = new URL("../worker.js", import.meta.url).href;
  const worker = new Worker(
    `
    import {parentPort} from 'node:worker_threads';
    let now=0, tick, releaseDigest;
    if (${deferHash}) {
      const nativeCrypto = globalThis.crypto;
      Object.defineProperty(globalThis, 'crypto', {value: {
        randomUUID: () => nativeCrypto.randomUUID(),
        subtle: {digest: async (...args) => {
          parentPort.postMessage({type:'hashStarted'});
          await new Promise(resolve => {releaseDigest = resolve;});
          return nativeCrypto.subtle.digest(...args);
        }},
      }});
    }
    globalThis.performance={now:()=>now};
    globalThis.setTimeout=fn=>{tick=fn;};
    globalThis.postMessage=data=>parentPort.postMessage(data);
    globalThis.onmessage=null;
    parentPort.on('message',data=>{
      if(data.releaseHash){releaseDigest?.();parentPort.postMessage({type:'advanced',requestId:data.requestId});}
      else if(data.advance!==undefined){for(let i=0;i<data.advance;i++){now+=100;tick();}parentPort.postMessage({type:'advanced',requestId:data.requestId});}
      else globalThis.onmessage({data});
    });
    await import(${JSON.stringify(url)});
  `,
    { eval: true, type: "module" },
  );
  t.after(() => worker.terminate());
  let sequence = 0,
    latest;
  const waiting = new Map();
  let resolveHash;
  const hashStarted = new Promise((resolve) => {
    resolveHash = resolve;
  });
  const ready = new Promise((resolve, reject) => {
    worker.on("error", reject);
    worker.on("message", (data) => {
      if (data.type === "hashStarted") resolveHash();
      if (data.type === "state") {
        latest = data;
        resolve(data);
      }
      if (
        ["apiReply", "advanced"].includes(data.type) &&
        waiting.has(data.requestId)
      ) {
        waiting.get(data.requestId)(data);
        waiting.delete(data.requestId);
      }
    });
  });
  await ready;
  const send = (command) =>
    new Promise((resolve, reject) => {
      const requestId = ++sequence;
      const timeout = setTimeout(() => {
        waiting.delete(requestId);
        reject(new Error("Worker reply timeout"));
      }, 5000);
      waiting.set(requestId, (data) => {
        clearTimeout(timeout);
        resolve(data);
      });
      worker.postMessage({ ...command, requestId });
    });
  const call = async (command) => {
    const reply = await send(command);
    assert.equal(reply.ok, true, reply.error);
    return reply.value;
  };
  return {
    send,
    call,
    advance: (count) => send({ advance: count }),
    hashStarted,
    releaseHash: () => send({ releaseHash: true }),
    get latest() {
      return latest;
    },
  };
}
function fleet(count = 3, ground = true) {
  const input = structuredClone(
    defaultWorldConfig(
      { startOnGround: ground },
      {
        sensors: {
          lidarRays: 12,
          lidarChannels: 1,
          lidarVerticalMinDeg: 0,
          lidarVerticalMaxDeg: 0,
        },
      },
    ),
  );
  const template = input.vehicles[0];
  input.vehicles = Array.from({ length: count }, (_, i) => ({
    ...structuredClone(template),
    id: `drone-${i + 1}`,
    name: `載具${i + 1}`,
    config: { ...template.config, seed: 40 + i },
    spawn: { ...template.spawn, east: i * 12 },
    home: [0, i * 12],
  }));
  return input;
}
function replay(record) {
  const world = createWorld(record.world);
  let eventIndex = 0;
  for (const frame of record.samples) {
    while (world.snapshot().tick < frame.tick) {
      while (
        eventIndex < record.events.length &&
        record.events[eventIndex].tick === world.snapshot().tick
      )
        apply(record.events[eventIndex++]);
      world.step();
    }
    while (
      eventIndex < record.events.length &&
      record.events[eventIndex].tick === frame.tick
    )
      apply(record.events[eventIndex++]);
    const actual = world.snapshot();
    for (const [i, v] of actual.vehicles.entries()) {
      assert.deepEqual(
        JSON.parse(JSON.stringify(v.snapshot)),
        JSON.parse(JSON.stringify(frame.vehicles[i].snapshot)),
      );
      assert.deepEqual(
        v.sensors,
        record.sensorFrames[record.samples.indexOf(frame)].vehicles[i].sensors,
      );
    }
    assert.deepEqual(actual.collisions, frame.collisions);
  }
  function apply(event) {
    if (event.type === "scene") world.setScene(event.value);
    else if (event.type === "wind") world.setWind(event.value);
    else if (event.type !== "render")
      world.command(event.vehicleId, event.type, event.value, event.fault);
  }
}
test("world worker targets explicit IDs, switches view without resetting, and exports one replayable clock", async (t) => {
  const h = await harness(t),
    input = fleet();
  let value = await h.call({ type: "resetWorld", value: input });
  assert.equal(value.world.vehicles.length, 3);
  for (const type of [
    "inspect",
    "arm",
    "targets",
    "sensorConfig",
    "pointCloudExport",
  ])
    assert.equal((await h.send({ type, value: { altitude: 2 } })).ok, false);
  await h.call({ type: "arm", vehicleId: "drone-2" });
  value = await h.call({ type: "takeoff", vehicleId: "drone-2" });
  assert.equal(value.snapshot.armed, true);
  assert.equal(value.world.vehicles[0].snapshot.armed, false);
  await h.call({ type: "play" });
  await h.advance(30);
  await h.call({ type: "pause" });
  const before = await h.call({ type: "inspectWorld" });
  const count = h.latest.sampleCount;
  value = await h.call({ type: "selectVehicle", vehicleId: "drone-2" });
  assert.deepEqual(value.world, before.world);
  assert.equal(h.latest.sampleCount, count);
  assert.equal(value.activeVehicleId, "drone-2");
  assert.ok(value.snapshot.position[2] < -0.2);
  assert.equal(value.world.vehicles[0].snapshot.position[2], 0);
  await h.call({
    type: "sensorFault",
    vehicleId: "drone-1",
    value: { sensor: "gps", failed: true },
  });
  value = await h.call({ type: "inspect", vehicleId: "drone-1" });
  assert.equal(value.sensors.gps.reason, "fault");
  assert.notEqual(value.world.vehicles[1].sensors.gps.reason, "fault");
  const cloud = await h.call({
    type: "pointCloudExport",
    vehicleId: "drone-2",
    value: "csv",
  });
  assert.equal(cloud.sourceVehicleId, "drone-2");
  assert.ok(cloud.text.startsWith("vehicle_id,world_tick,"));
  const record = validateRecording(await h.call({ type: "apiExport" }));
  assert.equal(record.world.vehicles.length, 3);
  assert.ok(
    record.events.every(
      (e) => ["scene", "wind", "render"].includes(e.type) || e.vehicleId,
    ),
  );
  assert.ok(
    record.samples.every((frame) =>
      frame.vehicles.every((v) => v.snapshot.metrics.steps === frame.tick),
    ),
  );
  replay(record);
});
test("invalid roster resets are atomic and setup, wind and training changes remain correctly scoped", async (t) => {
  const h = await harness(t),
    input = fleet();
  await h.call({ type: "resetWorld", value: input });
  await h.call({ type: "arm", vehicleId: "drone-1" });
  const before = await h.call({ type: "inspectWorld" });
  const invalid = structuredClone(input);
  invalid.vehicles[1].id = invalid.vehicles[0].id;
  assert.equal(
    (await h.send({ type: "resetWorld", value: invalid })).ok,
    false,
  );
  assert.deepEqual(
    (await h.call({ type: "inspectWorld" })).world,
    before.world,
  );
  assert.equal(
    (await h.send({ type: "selectVehicle", vehicleId: "absent" })).ok,
    false,
  );
  await h.call({ type: "training", vehicleId: "drone-2", value: "gates" });
  await h.call({ type: "wind", value: { speed: 3, direction: 90 } });
  const scene = {
    ...input.scene,
    environment: { ...input.scene.environment, fog: 0.2 },
  };
  let result = await h.call({ type: "scene", value: scene });
  assert.equal(result.world.vehicles[0].training.course, "free");
  assert.equal(result.world.vehicles[1].training.course, "gates");
  assert.ok(
    result.world.vehicles.every(
      (v) => v.config.wind === 3 && v.config.windDirection === 90,
    ),
  );
  assert.equal(result.world.scene.environment.fog, 0.2);
  const record = validateRecording(await h.call({ type: "apiExport" }));
  assert.equal(
    record.events.find((e) => e.type === "training").vehicleId,
    "drone-2",
  );
  assert.ok(
    !Object.hasOwn(
      record.events.find((e) => e.type === "wind"),
      "vehicleId",
    ),
  );
  replay(record);
});
test("rendering and selection have no effect on physics or delayed sensor replay", async (t) => {
  async function run(render) {
    const h = await harness(t),
      input = fleet(3, false);
    input.vehicles[0].setup.sensors = {
      ...input.vehicles[0].setup.sensors,
      latency: 0.1,
      dropout: 0.2,
    };
    await h.call({ type: "resetWorld", value: input });
    await h.call({ type: "render", value: render });
    await h.call({ type: "play" });
    await h.advance(10);
    await h.call({
      type: "selectVehicle",
      vehicleId: render ? "drone-1" : "drone-3",
    });
    await h.advance(10);
    await h.call({ type: "pause" });
    return validateRecording(await h.call({ type: "apiExport" }));
  }
  const a = await run(true),
    b = await run(false);
  assert.deepEqual(a.samples, b.samples);
  assert.deepEqual(a.sensorFrames, b.sensorFrames);
  replay(a);
  replay(b);
});
test("collision pauses the whole world and its final frame remains exportable", async (t) => {
  const h = await harness(t),
    input = fleet(2, false);
  input.vehicles[0].spawn = { north: 0, east: -2, altitude: 2, heading: 0 };
  input.vehicles[0].home = [0, -2];
  input.vehicles[1].spawn = { north: 0, east: 2, altitude: 2, heading: 0 };
  input.vehicles[1].home = [0, 2];
  await h.call({ type: "resetWorld", value: input });
  for (const id of ["drone-1", "drone-2"]) {
    await h.call({ type: "mode", vehicleId: id, value: "position" });
    await h.call({
      type: "position",
      vehicleId: id,
      value: { north: 0, east: 0, altitude: 2 },
    });
  }
  await h.call({ type: "play" });
  await h.advance(150);
  const final = await h.call({ type: "inspectWorld" });
  assert.equal(final.running, false);
  assert.equal(final.termination.reason, "vehicle-collision");
  assert.equal(final.world.collisions.length, 1);
  assert.ok(final.world.vehicles.every((v) => v.snapshot.status === "crashed"));
  assert.equal((await h.send({ type: "play" })).ok, false);
  assert.equal((await h.send({ type: "arm", vehicleId: "drone-1" })).ok, false);
  const record = validateRecording(await h.call({ type: "apiExport" }));
  assert.equal(record.samples.at(-1).tick, final.world.tick);
  replay(record);
});

test("radar exports identify the requested vehicle and retain delayed acquisition data without inventing returns", async (t) => {
  const h = await harness(t),
    input = fleet(2, false);
  for (const v of input.vehicles)
    v.setup.sensors = { ...v.setup.sensors, latency: 0.25 };
  await h.call({ type: "resetWorld", value: input });
  assert.equal(
    (await h.send({ type: "radarExport", value: "json" })).ok,
    false,
  );
  await h.call({ type: "play" });
  await h.advance(10);
  await h.call({ type: "pause" });
  const inspected = await h.call({ type: "inspect", vehicleId: "drone-2" });
  const json = await h.call({
    type: "radarExport",
    vehicleId: "drone-2",
    value: "json",
  });
  const artifact = JSON.parse(json.text);
  assert.equal(json.sourceVehicleId, "drone-2");
  assert.equal(artifact.sourceVehicleId, "drone-2");
  assert.equal(artifact.exportWorldTick, inspected.world.tick);
  assert.deepEqual(artifact.reading, inspected.sensors.radar);
  assert.ok(artifact.reading.time < inspected.world.time);
  const csv = await h.call({
    type: "radarExport",
    vehicleId: "drone-2",
    value: "csv",
  });
  assert.ok(csv.text.startsWith("source_vehicle_id,export_world_tick,"));
  assert.equal(
    (await h.send({ type: "radarExport", vehicleId: "drone-2", value: "pcd" }))
      .ok,
    false,
  );
  await h.call({
    type: "sensorFault",
    vehicleId: "drone-2",
    value: { sensor: "radar", failed: true },
  });
  const faulted = JSON.parse(
    (await h.call({ type: "radarExport", vehicleId: "drone-2", value: "json" }))
      .text,
  );
  assert.equal(faulted.reading.reason, "fault");
  assert.equal(faulted.reading.detections.length, 0);
  const empty = await h.call({
    type: "radarExport",
    vehicleId: "drone-2",
    value: "csv",
  });
  const rows = empty.text
    .trimEnd()
    .split("\n")
    .map((r) => r.split(","));
  assert.equal(rows.length, 2);
  assert.ok(rows[1].slice(-5).every((v) => v === ""));
  const raw = await h.call({ type: "apiExport" });
  assert.equal(
    typeof raw.sensorFrames.at(-1).vehicles[1].sensors.faults,
    "number",
  );
  const record = validateRecording(raw);
  assert.equal(record.version, 15);
  replay(record);
});

function inspectionFleet() {
  const input = fleet(2, false);
  input.scene.obstacles = [
    { baseAltitude: 0, north: 10, east: 0, width: 2, depth: 4, height: 4 },
  ];
  input.scene.inspectionTargets = [
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
      minPixels: 8,
      maxSpeedMps: 2,
    },
  ];
  return input;
}
const beginInspection = (h) =>
  h.call({
    type: "inspectionBegin",
    vehicleId: "drone-1",
    value: { targetId: "panel", config: DEFAULT_CAMERA_CONFIG },
  });
// Typed zero-depth protocol fixtures exercise binding and hashing, never GPU rendering or image quality.
function protocolFrame(metadata) {
  const pixels = metadata.width * metadata.height;
  const rgbRgba = new Uint8Array(pixels * 4);
  for (let i = 3; i < rgbRgba.length; i += 4) rgbRgba[i] = 255;
  return {
    ...structuredClone(metadata),
    rgbRgba,
    depthM: new Float32Array(pixels),
    classes: new Uint8Array(pixels),
  };
}
const submitInspection = (h, lease, frame = protocolFrame(lease.metadata)) =>
  h.send({
    type: "inspectionSubmit",
    vehicleId: "drone-1",
    value: { captureId: lease.captureId, frame },
  });

test("inspection lease freezes the whole fleet, rejects mutations and mismatched tickets, and releases failed submissions", async (t) => {
  const h = await harness(t);
  await h.call({ type: "resetWorld", value: inspectionFleet() });
  await h.call({ type: "play" });
  await h.advance(2);
  const lease = await beginInspection(h);
  await h.advance(5);
  assert.deepEqual((await h.call({ type: "inspectWorld" })).world, lease.world);
  for (const command of [
    { type: "targets", vehicleId: "drone-2", value: { altitude: 3 } },
    { type: "scene", value: lease.world.scene },
    { type: "wind", value: { speed: 1, direction: 0 } },
    { type: "play" },
    { type: "speed", value: 2 },
  ])
    assert.equal((await h.send(command)).ok, false);
  const selected = await h.call({
    type: "selectVehicle",
    vehicleId: "drone-2",
  });
  assert.equal(selected.activeVehicleId, "drone-2");
  assert.equal(selected.world.tick, lease.world.tick);
  for (const command of [
    {
      type: "inspectionSubmit",
      vehicleId: "drone-2",
      value: {
        captureId: lease.captureId,
        frame: protocolFrame(lease.metadata),
      },
    },
    {
      type: "inspectionCancel",
      vehicleId: "drone-2",
      value: { captureId: lease.captureId },
    },
    {
      type: "inspectionSubmit",
      vehicleId: "drone-1",
      value: {
        captureId: "00000000-0000-4000-8000-000000000001",
        frame: protocolFrame(lease.metadata),
      },
    },
  ])
    assert.equal((await h.send(command)).ok, false);
  await h.advance(1);
  assert.equal(
    (await h.call({ type: "inspectWorld" })).world.tick,
    lease.world.tick,
  );
  const wrong = protocolFrame(lease.metadata);
  wrong.worldTick++;
  assert.equal((await submitInspection(h, lease, wrong)).ok, false);
  assert.equal((await h.call({ type: "inspectWorld" })).running, true);
  await h.advance(1);
  assert.ok(
    (await h.call({ type: "inspectWorld" })).world.tick > lease.world.tick,
  );
  assert.equal(
    (await h.call({ type: "inspectionReport", vehicleId: "drone-1" })).summary
      .attempts,
    0,
  );
  assert.equal(
    (
      await h.send({
        type: "inspection",
        vehicleId: "drone-1",
        value: {
          captureId: lease.captureId,
          targetId: "panel",
          config: DEFAULT_CAMERA_CONFIG,
          sha256: "a".repeat(64),
        },
      })
    ).ok,
    false,
  );
});

test("successful inspection submissions hash actual buffers and retain replayable schema15 receipts without resuming an explicit pause", async (t) => {
  const h = await harness(t);
  await h.call({ type: "resetWorld", value: inspectionFleet() });
  await h.call({ type: "play" });
  await h.advance(1);
  const lease = await beginInspection(h),
    frame = protocolFrame(lease.metadata);
  const expectedHash = await hashCameraFrame(frame);
  await h.call({ type: "pause" });
  const reply = await submitInspection(h, lease, frame);
  assert.equal(reply.ok, true, reply.error);
  assert.deepEqual(reply.value.summary, {
    passed: 1,
    total: 1,
    attempts: 1,
    complete: true,
  });
  assert.equal(reply.value.receipts[0].sha256, expectedHash);
  assert.equal(reply.value.receipts[0].metadata.captureId, lease.captureId);
  assert.equal(reply.value.receipts[0].metadata.sourceVehicleId, "drone-1");
  assert.equal(reply.value.receipts[0].metadata.worldTick, lease.world.tick);
  await h.advance(3);
  const inspected = await h.call({ type: "inspectWorld" });
  assert.equal(inspected.running, false);
  assert.equal(inspected.world.tick, lease.world.tick);
  assert.equal(inspected.world.vehicles[1].snapshot.inspection.attempts, 0);
  assert.equal((await submitInspection(h, lease, frame)).ok, false);
  const record = validateRecording(await h.call({ type: "apiExport" }));
  assert.equal(record.version, 15);
  const event = record.events.find((event) => event.type === "inspection");
  assert.equal(event.tick, lease.world.tick);
  assert.equal(event.value.sha256, expectedHash);
  assert.deepEqual(event.receipt, reply.value.receipts[0]);
  assert.equal(record.samples.at(-1).tick, event.tick);
  assert.equal(record.samples.at(-1).vehicles[0].snapshot.inspection.passed, 1);
  replay(record);
});

test("inspection cancellation, reset and 15-second expiry invalidate tickets without adding evidence or paused wall-time steps", async (t) => {
  const h = await harness(t);
  const input = inspectionFleet();
  await h.call({ type: "resetWorld", value: input });
  await h.call({ type: "play" });
  let lease = await beginInspection(h);
  await h.call({
    type: "inspectionCancel",
    vehicleId: "drone-1",
    value: { captureId: lease.captureId },
  });
  assert.equal((await h.call({ type: "inspectWorld" })).running, true);
  assert.equal((await submitInspection(h, lease)).ok, false);
  lease = await beginInspection(h);
  await h.advance(149);
  assert.equal(
    (await h.call({ type: "inspectWorld" })).world.tick,
    lease.world.tick,
  );
  await h.advance(1);
  const expired = await h.call({ type: "inspectWorld" });
  assert.equal(expired.running, true);
  assert.ok(
    expired.world.tick > lease.world.tick &&
      expired.world.tick <= lease.world.tick + 20,
  );
  assert.equal((await submitInspection(h, lease)).ok, false);
  lease = await beginInspection(h);
  await h.call({ type: "resetWorld", value: input });
  assert.equal((await submitInspection(h, lease)).ok, false);
  const reset = await h.call({ type: "inspectWorld" });
  assert.equal(reset.running, false);
  assert.equal(reset.world.tick, 0);
  assert.equal(reset.world.vehicles[0].snapshot.inspection.attempts, 0);
  assert.equal((await h.call({ type: "apiExport" })).events.length, 0);
});

test("reset or cancel while the real buffer digest is pending cannot commit stale inspection evidence or resume a replacement world", async (t) => {
  for (const cancel of [false, true]) {
    const h = await harness(t, true),
      input = inspectionFleet();
    await h.call({ type: "resetWorld", value: input });
    await h.call({ type: "play" });
    const lease = await beginInspection(h);
    const pending = submitInspection(h, lease);
    await h.hashStarted;
    if (cancel) {
      await h.call({ type: "pause" });
      await h.call({
        type: "inspectionCancel",
        vehicleId: "drone-1",
        value: { captureId: lease.captureId },
      });
    } else await h.call({ type: "resetWorld", value: input });
    await h.releaseHash();
    assert.equal((await pending).ok, false);
    const inspected = await h.call({ type: "inspectWorld" });
    assert.equal(inspected.running, false);
    assert.equal(inspected.world.tick, 0);
    assert.equal(inspected.world.vehicles[0].snapshot.inspection.attempts, 0);
    assert.equal((await h.call({ type: "apiExport" })).events.length, 0);
  }
});
