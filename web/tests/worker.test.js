import test from "node:test";
import assert from "node:assert/strict";
import { Worker } from "node:worker_threads";
import { readFile } from "node:fs/promises";
import { createWorld, defaultWorldConfig } from "../world.js";
import { validateRecording } from "../recording.js";
import { DEFAULT_CAMERA_CONFIG, hashCameraFrame } from "../camera.js";

async function harness(t, deferHash = false, limits = null) {
  let url = new URL("../worker.js", import.meta.url).href;
  if (limits) {
    // Exercise terminal branches with the real worker clock without running thirty minutes.
    let source = await readFile(new URL(url), "utf8");
    source = source.replace(/from "(\.\/[^"]+)"/g, (_, path) => `from "${new URL(path, url).href}"`);
    if (limits.rejectInstructor) source = source.replace('if (command.type === "wind") world.setWind(command.value);', 'if (command.type === "gust") throw new Error("test command failure"); if (command.type === "wind") world.setWind(command.value);');
    if (limits.time) source = source.replace("LIMIT = 1800", `LIMIT = ${limits.time}`);
    if (limits.events) source = source.replaceAll("100000", String(limits.events));
    if (limits.bytes) source = source.replace("DATA_LIMIT = 96 * 1024 * 1024", `DATA_LIMIT = ${limits.bytes}`);
    url = `data:text/javascript;base64,${Buffer.from(source).toString("base64")}`;
  }
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
  await h.call({type:"instructorApply",value:{name:"碰撞停止",events:[{tick:360000,type:"wind",value:{speed:0,direction:0}}]}});
  await h.call({ type: "play" });
  await h.advance(150);
  const final = await h.call({ type: "inspectWorld" });
  assert.equal(final.running, false);
  assert.equal(final.termination.reason, "vehicle-collision");
  assert.equal(final.instructor.phase, "cancelled");
  assert.equal(final.instructor.reason, "vehicle-collision");
  assert.deepEqual(final.instructor.receipts, []);
  assert.equal(final.world.collisions.length, 1);
  assert.ok(final.world.vehicles.every((v) => v.snapshot.status === "crashed"));
  assert.equal((await h.send({ type: "play" })).ok, false);
  assert.equal((await h.send({ type: "arm", vehicleId: "drone-1" })).ok, false);
  const record = validateRecording(await h.call({ type: "apiExport" }));
  assert.equal(record.samples.at(-1).tick, final.world.tick);
  replay(record);
  const nextPlan={name:"終止後拒絕",events:[{tick:360000,type:"wind",value:{speed:0,direction:0}}]};
  assert.equal((await h.send({type:"instructorApply",value:nextPlan})).ok,false);
  assert.deepEqual(await h.call({type:"instructorStatus"}),final.instructor);
  assert.deepEqual(await h.call({type:"instructorReport"}),final.instructor);
  assert.deepEqual(await h.call({type:"instructorCancel"}),final.instructor);
  await h.call({type:"resetWorld",value:fleet(1)});
  assert.equal((await h.call({type:"instructorApply",value:nextPlan})).phase,"armed");
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
  assert.equal(record.version, 16);
  replay(record);
});

test("proximity exports preserve the requested vehicle, delayed pose and fault state", async (t) => {
  const h = await harness(t), input = fleet(2, false);
  for (const v of input.vehicles)
    Object.assign(v.setup.sensors, { latency: 0.25, proximityNoiseM: 0 });
  await h.call({ type: "resetWorld", value: input });
  assert.equal((await h.send({ type: "proximityExport", value: "json" })).ok, false);
  assert.equal((await h.send({ type: "proximityExport", vehicleId: "missing", value: "json" })).ok, false);
  await h.call({ type: "play" });
  await h.advance(10);
  await h.call({ type: "pause" });
  const inspected = await h.call({ type: "inspect", vehicleId: "drone-2" });
  const exported = await h.call({ type: "proximityExport", vehicleId: "drone-2", value: "json" });
  const artifact = JSON.parse(exported.text);
  assert.equal(exported.sourceVehicleId, "drone-2");
  assert.equal(artifact.worldTick, inspected.world.tick);
  assert.equal(artifact.sourceVehicleId, "drone-2");
  assert.deepEqual(artifact.reading, inspected.sensors.proximity);
  assert.ok(artifact.reading.time < inspected.world.time);
  const csv = await h.call({ type: "proximityExport", vehicleId: "drone-2", value: "csv" });
  assert.equal(csv.text.split("\n").length, 7);
  assert.ok(csv.text.startsWith("source_vehicle_id,observation_world_tick,"));
  assert.equal((await h.send({ type: "proximityExport", vehicleId: "drone-2", value: "pcd" })).ok, false);
  await h.call({ type: "sensorFault", vehicleId: "drone-2", value: { sensor: "proximity", failed: true } });
  const faulted = JSON.parse((await h.call({ type: "proximityExport", vehicleId: "drone-2", value: "json" })).text);
  assert.equal(faulted.reading.reason, "fault");
  assert.ok(faulted.reading.rangesM.every(v => v === null));
  assert.ok(faulted.reading.tooClose.every(v => v === false));
  const record = validateRecording(await h.call({ type: "apiExport" }));
  assert.equal(record.version, 16);
  assert.equal(record.sensorFrames.at(-1).vehicles[1].sensors.faults.proximity, true);
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

test("successful inspection submissions hash actual buffers and retain replayable schema16 receipts without resuming an explicit pause", async (t) => {
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
  assert.equal(record.version, 16);
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

test('instructor schedules native schema16 events in tick order independent of speed, rendering, selection and pause', async (t) => {
  const records=[];
  for (const fast of [false,true]) {
    const h=await harness(t);
    await h.call({type:'resetWorld',value:fleet(2)});
    const plan={name:'定時故障',events:[
      {tick:2,vehicleId:'drone-2',type:'sensorFault',value:{sensor:'gps',failed:true}},
      {tick:2,type:'wind',value:{speed:3,direction:90}},
      {tick:3,vehicleId:'drone-2',type:'sensorFault',value:{sensor:'gps',failed:false}},
      {tick:4,vehicleId:'drone-1',type:'fault',fault:'link',value:{lost:true}},
      {tick:5,vehicleId:'drone-1',type:'clearFaults'},
    ]};
    assert.equal((await h.call({type:'instructorApply',value:plan})).phase,'armed');
    assert.equal((await h.send({type:'instructorApply',value:plan})).ok,false);
    await h.call({type:'selectVehicle',vehicleId:'drone-2'});
    await h.advance(2);
    assert.equal((await h.call({type:'inspectWorld'})).world.tick,0);
    if(fast){await h.call({type:'speed',value:4});await h.call({type:'render',value:false});}
    await h.call({type:'play'});await h.advance(fast?1:4);await h.call({type:'pause'});
    const state=await h.call({type:'inspectWorld'}), status=await h.call({type:'instructorStatus'});
    assert.equal(status.phase,'complete');assert.deepEqual(state.instructor,status);
    assert.deepEqual(status.receipts.map(r=>r.tick),[2,2,3,4,5]);
    const record=validateRecording(await h.call({type:'apiExport'}));
    replay(record);
    assert.equal(record.version,16);assert.equal(Object.hasOwn(record,'instructor'),false);
    assert.deepEqual(record.events.filter(e=>e.type!=='render').map(({time,...e})=>e),plan.events);
    records.push(record);
  }
  assert.deepEqual(records[0].samples.at(-1),records[1].samples.at(-1));
});

test('instructor cancellation, lease freezing and atomic reset cannot leave ghost events', async(t)=>{
  const h=await harness(t);await h.call({type:'resetWorld',value:inspectionFleet()});
  const plan={name:'取消',events:[{tick:2,vehicleId:'drone-1',type:'gust'},{tick:100,vehicleId:'drone-1',type:'clearFaults'}]};
  await h.call({type:'instructorApply',value:plan});
  const lease=await h.call({type:'inspectionBegin',vehicleId:'drone-1',value:{targetId:'panel',config:DEFAULT_CAMERA_CONFIG}});
  assert.equal((await h.send({type:'instructorCancel'})).ok,false);
  assert.equal((await h.send({type:'instructorApply',value:plan})).ok,false);
  await h.advance(1);assert.equal((await h.call({type:'instructorStatus'})).nextIndex,0);
  await h.call({type:'inspectionCancel',vehicleId:'drone-1',value:{captureId:lease.captureId}});
  await h.call({type:'play'});await h.advance(1);await h.call({type:'pause'});
  assert.equal((await h.call({type:'instructorStatus'})).receipts.length,1);
  const before=await h.call({type:'instructorStatus'});
  assert.equal((await h.send({type:'resetWorld',value:{}})).ok,false);
  assert.deepEqual(await h.call({type:'instructorStatus'}),before);
  const cancelled=await h.call({type:'instructorCancel'});assert.equal(cancelled.phase,'cancelled');
  await h.call({type:'play'});await h.advance(6);await h.call({type:'pause'});
  assert.equal((await h.call({type:'apiExport'})).events.length,1);
  await h.call({type:'resetWorld',value:fleet(1)});
  assert.equal((await h.call({type:'instructorStatus'})).phase,'idle');
  await h.call({type:'play'});await h.advance(6);await h.call({type:'pause'});
  assert.equal((await h.call({type:'apiExport'})).events.length,0);
});

test('instructor executes the terminal time tick before stopping and cancels later events',async(t)=>{
  const h=await harness(t,false,{time:0.01});
  await h.call({type:'resetWorld',value:fleet(1)});
  await h.call({type:'instructorApply',value:{name:'時間邊界',events:[{tick:2,type:'wind',value:{speed:2,direction:0}},{tick:3,vehicleId:'drone-1',type:'gust'}]}});
  await h.call({type:'play'});await h.advance(1);
  const state=await h.call({type:'inspectWorld'});
  assert.equal(state.world.tick,2);assert.equal(state.running,false);assert.equal(state.instructor.phase,'cancelled');
  assert.equal(state.instructor.reason,'time-limit');assert.equal(state.instructor.receipts.length,1);
  const record=validateRecording(await h.call({type:'apiExport'}));assert.equal(record.events.length,1);replay(record);
});

test('instructor event capacity records the final allowed mutation and cancels same-tick pending events',async(t)=>{
  const h=await harness(t,false,{events:2});await h.call({type:'resetWorld',value:fleet(1)});
  await h.call({type:'instructorApply',value:{name:'容量',events:[
    {tick:1,type:'wind',value:{speed:1,direction:0}},
    {tick:1,type:'wind',value:{speed:2,direction:0}},
    {tick:1,type:'wind',value:{speed:3,direction:0}},
  ]}});
  await h.call({type:'play'});await h.advance(1);
  const status=await h.call({type:'instructorStatus'}),state=await h.call({type:'inspectWorld'});
  assert.equal(status.phase,'cancelled');assert.equal(status.reason,'storage-limit');assert.equal(status.receipts.length,2);
  assert.equal(state.worldConfig.wind.speed,2);assert.equal(state.running,false);
  assert.equal((await h.send({type:'instructorApply',value:{name:'禁止',events:[{tick:2,type:'wind',value:{speed:3,direction:0}}]}})).ok,false);
  assert.equal((await h.call({type:'instructorCancel'})).phase,'cancelled');
  const record=validateRecording(await h.call({type:'apiExport'}));assert.equal(record.events.length,2);replay(record);
});

test('instructor byte capacity cancels future events while keeping the recorded mutation replayable',async(t)=>{
  const h=await harness(t,false,{bytes:50000});await h.call({type:'resetWorld',value:fleet(1)});
  await h.call({type:'instructorApply',value:{name:'容量',events:[{tick:1,type:'wind',value:{speed:2,direction:90}},{tick:100000,type:'wind',value:{speed:9,direction:0}}]}});
  await h.call({type:'play'});await h.advance(30);
  const state=await h.call({type:'inspectWorld'});assert.equal(state.running,false);assert.equal(state.instructor.phase,'cancelled');assert.equal(state.instructor.reason,'storage-limit');
  assert.equal(state.worldConfig.wind.speed,2);
  const record=validateRecording(await h.call({type:'apiExport'}));assert.equal(record.events.length,1);replay(record);
});

test('instructor callback failure pauses the world without rolling back earlier same-tick events or running later ones',async(t)=>{
  const h=await harness(t,false,{rejectInstructor:true});await h.call({type:'resetWorld',value:fleet(1)});
  await h.call({type:'instructorApply',value:{name:'錯誤停止',events:[
    {tick:1,type:'wind',value:{speed:1,direction:0}},
    {tick:1,vehicleId:'drone-1',type:'gust'},
    {tick:1,type:'wind',value:{speed:3,direction:0}},
  ]}});
  await h.call({type:'play'});await h.advance(1);
  const state=await h.call({type:'inspectWorld'});
  assert.equal(state.running,false);assert.equal(state.world.tick,1);assert.equal(state.worldConfig.wind.speed,1);
  assert.equal(h.latest.message,'instructor-error');assert.equal(state.instructor.phase,'error');
  assert.deepEqual(state.instructor.receipts.map(receipt=>receipt.status),['applied','failed']);
  await h.advance(3);assert.deepEqual(await h.call({type:'instructorStatus'}),state.instructor);
  await h.call({type:'play'});await h.advance(1);await h.call({type:'pause'});
  const resumed=await h.call({type:'inspectWorld'});
  assert.ok(resumed.world.tick>1);assert.equal(resumed.worldConfig.wind.speed,1);
  assert.deepEqual(resumed.instructor,state.instructor);
  const record=validateRecording(await h.call({type:'apiExport'}));assert.equal(record.events.length,1);replay(record);
});

test('instructor applies lift motor and pusher faults to a VTOL through native commands with exact replay',async(t)=>{
  const h=await harness(t),input=fleet(1);
  input.vehicles[0].config.airframe='vtol';
  await h.call({type:'resetWorld',value:input});
  const events=[
    {tick:1,vehicleId:'drone-1',type:'fault',fault:'motor',value:{index:3,efficiency:0.25}},
    {tick:1,vehicleId:'drone-1',type:'fault',fault:'pusher',value:{efficiency:0.4}},
  ];
  await h.call({type:'instructorApply',value:{name:'複合推進故障',events}});
  await h.call({type:'play'});await h.advance(1);await h.call({type:'pause'});
  const state=await h.call({type:'inspectWorld'});
  assert.equal(state.instructor.phase,'complete');
  assert.deepEqual(state.instructor.receipts.map(receipt=>receipt.status),['applied','applied']);
  assert.deepEqual(state.snapshot.faults.motorEfficiency,[1,1,1,0.25]);
  assert.equal(state.snapshot.faults.pusherEfficiency,0.4);
  const record=validateRecording(await h.call({type:'apiExport'}));
  assert.deepEqual(record.events.map(({time,...event})=>event),events);
  replay(record);
});
