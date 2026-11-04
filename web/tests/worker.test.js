import test from "node:test";
import assert from "node:assert/strict";
import { Worker } from "node:worker_threads";
import { createWorld, defaultWorldConfig } from "../world.js";
import { validateRecording } from "../recording.js";

async function harness(t) {
  const url = new URL("../worker.js", import.meta.url).href;
  const worker = new Worker(
    `
    import {parentPort} from 'node:worker_threads';
    let now=0, tick;
    globalThis.performance={now:()=>now};
    globalThis.setTimeout=fn=>{tick=fn;};
    globalThis.postMessage=data=>parentPort.postMessage(data);
    globalThis.onmessage=null;
    parentPort.on('message',data=>{
      if(data.advance!==undefined){for(let i=0;i<data.advance;i++){now+=100;tick();}parentPort.postMessage({type:'advanced',requestId:data.requestId});}
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
  const ready = new Promise((resolve, reject) => {
    worker.on("error", reject);
    worker.on("message", (data) => {
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
  assert.equal(record.version, 13);
  replay(record);
});
