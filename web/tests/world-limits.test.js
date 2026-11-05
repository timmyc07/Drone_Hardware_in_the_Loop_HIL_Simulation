import test from "node:test";
import assert from "node:assert/strict";
import { Worker } from "node:worker_threads";
import { createWorld, defaultWorldConfig } from "../world.js";
import { validateRecording } from "../recording.js";

const DATA_LIMIT = 96 * 1024 * 1024;
const lowRays = {
  lidarRays: 12,
  lidarChannels: 1,
  lidarVerticalMinDeg: 0,
  lidarVerticalMaxDeg: 0,
};
const clone = structuredClone;

// Only wall time is accelerated; the unmodified Worker still performs every 200 Hz step.
async function harness(t) {
  const url = new URL("../worker.js", import.meta.url).href;
  const worker = new Worker(
    `
    import { parentPort } from 'node:worker_threads';
    let now = 0, scheduled;
    globalThis.performance = { now: () => now };
    globalThis.setTimeout = fn => { scheduled = fn; };
    globalThis.postMessage = data => parentPort.postMessage(data);
    globalThis.onmessage = null;
    parentPort.on('message', data => {
      if (data.advance !== undefined) {
        for (let i = 0; i < data.advance; i++) { now += 100; scheduled(); }
        parentPort.postMessage({ type: 'advanced', requestId: data.requestId });
      } else globalThis.onmessage({ data });
    });
    await import(${JSON.stringify(url)});
  `,
    { eval: true, type: "module" },
  );
  let sequence = 0,
    latest;
  const pending = new Map(),
    errors = [];
  t.after(() => {
    for (const request of pending.values()) clearTimeout(request.timer);
    return worker.terminate();
  });
  const ready = new Promise((resolve, reject) => {
    worker.on("error", (error) => {
      reject(error);
      for (const request of pending.values()) {
        clearTimeout(request.timer);
        request.reject(error);
      }
      pending.clear();
    });
    worker.on("message", (message) => {
      if (message.type === "state") {
        latest = message;
        resolve();
      }
      if (message.type === "error") errors.push(message.message);
      const request = pending.get(message.requestId);
      if (request && ["apiReply", "advanced"].includes(message.type)) {
        clearTimeout(request.timer);
        pending.delete(message.requestId);
        request.resolve(message);
      }
    });
  });
  await ready;
  function send(command) {
    return new Promise((resolve, reject) => {
      const requestId = ++sequence;
      const timer = setTimeout(() => {
        pending.delete(requestId);
        reject(new Error("World limit Worker reply timed out"));
      }, 120000);
      pending.set(requestId, { resolve, reject, timer });
      worker.postMessage({ ...command, requestId });
    });
  }
  async function call(command) {
    const reply = await send(command);
    assert.equal(reply.ok, true, reply.error);
    return reply.value;
  }
  return {
    send,
    call,
    errors,
    advance: (count) => send({ advance: count }),
    get latest() {
      return latest;
    },
    async untilStopped(maxCallbacks, batch = 100) {
      for (let i = 0; i < maxCallbacks; i += batch) {
        await send({ advance: Math.min(batch, maxCallbacks - i) });
        const value = await call({ type: "inspectWorld" });
        if (!value.running) return value;
      }
      assert.fail("Worker did not reach the expected production limit");
    },
  };
}
function fleet(count, { ground = false, sensors = lowRays } = {}) {
  const input = clone(
    defaultWorldConfig(
      { startOnGround: ground, batteryBasePower: 0 },
      { sensors },
    ),
  );
  const template = input.vehicles[0];
  input.vehicles = Array.from({ length: count }, (_, i) => ({
    ...clone(template),
    id: `drone-${i + 1}`,
    name: `Vehicle ${i + 1}`,
    config: { ...template.config, seed: 50 + i },
    spawn: { ...template.spawn, east: 12 * i },
    home: [0, 12 * i],
  }));
  return input;
}
function assertFinalWorld(record, stopped) {
  assert.equal(record.version, 16);
  assert.equal(record.samples.at(-1).tick, stopped.world.tick);
  assert.equal(record.samples.at(-1).time, stopped.world.time);
  assert.equal(record.sensorFrames.at(-1).tick, stopped.world.tick);
  assert.equal(record.sensorFrames.at(-1).time, stopped.world.time);
  assert.equal(
    record.samples.at(-1).vehicles.length,
    stopped.world.vehicles.length,
  );
  for (const [i, vehicle] of stopped.world.vehicles.entries()) {
    const saved = record.samples.at(-1).vehicles[i];
    assert.equal(saved.id, vehicle.id);
    assert.equal(saved.snapshot.time, stopped.world.time);
    assert.equal(saved.snapshot.metrics.steps, stopped.world.tick);
    assert.deepEqual(
      JSON.parse(JSON.stringify(saved.snapshot)),
      JSON.parse(JSON.stringify(vehicle.snapshot)),
    );
    assert.deepEqual(
      record.sensorFrames.at(-1).vehicles[i].sensors,
      vehicle.sensors,
    );
  }
}

test(
  "the actual 30 minute Worker bound retains all 360000 physical steps and its final world frame",
  { timeout: 120000 },
  async (t) => {
    const started = performance.now(),
      h = await harness(t);
    await h.call({
      type: "resetWorld",
      value: fleet(1, { ground: true, sensors: { ...lowRays, dropout: 1 } }),
    });
    await h.call({ type: "speed", value: 4 });
    await h.call({ type: "play" });
    const stopped = await h.untilStopped(4600);
    assert.equal(
      h.latest.message,
      "limit",
      "the time fixture must not stop at the separate storage cap",
    );
    assert.equal(h.latest.storageLimited, false);
    assert.equal(stopped.running, false);
    assert.ok(Math.abs(stopped.world.time - 1800) < 1e-7);
    assert.equal(stopped.world.tick, 360000);
    assert.equal(h.latest.sampleCount, 36001);
    const replay = await h.send({ type: "play" });
    assert.equal(replay.ok, false);
    assert.match(replay.error, /30/);
    const raw = await h.call({ type: "apiExport" });
    const retainedBytes = Buffer.byteLength(JSON.stringify(raw.samples)) +
      Buffer.byteLength(JSON.stringify(raw.sensorFrames)) - 2 +
      (raw.events.length ? Buffer.byteLength(JSON.stringify(raw.events)) - 1 : 0);
    t.diagnostic(`30 minute retained ${retainedBytes} bytes across ${raw.samples.length} frames at tick ${stopped.world.tick}`);
    const record = validateRecording(raw);
    assert.equal(record.samples.length, 36001);
    assertFinalWorld(record, stopped);
    await h.advance(10);
    assert.deepEqual(
      (await h.call({ type: "inspectWorld" })).world,
      stopped.world,
    );
    t.diagnostic(
      `Production 30 minute bound completed in ${((performance.now() - started) / 1000).toFixed(2)} real seconds`,
    );
  },
);

test(
  "four maximum-density scanners reach the actual 96 MiB retained cap and keep the whole final world exportable",
  { timeout: 120000 },
  async (t) => {
    const started = performance.now(),
      h = await harness(t);
    const input = fleet(4, {
      sensors: { lidarRays: 180, lidarChannels: 32, lidarHz: 20 },
    });
    input.scene.obstacles = input.vehicles.flatMap((v) => {
      const east = v.spawn.east;
      return [
        { baseAltitude: 0, north: 4, east, width: 0.1, depth: 8, height: 6 },
        { baseAltitude: 0, north: -4, east, width: 0.1, depth: 8, height: 6 },
        {
          baseAltitude: 0,
          north: 0,
          east: east + 4,
          width: 8,
          depth: 0.1,
          height: 6,
        },
        {
          baseAltitude: 0,
          north: 0,
          east: east - 4,
          width: 8,
          depth: 0.1,
          height: 6,
        },
      ];
    });
    await h.call({ type: "resetWorld", value: input });
    await h.call({ type: "speed", value: 4 });
    await h.call({ type: "play" });
    const stopped = await h.untilStopped(1000, 10);
    assert.equal(stopped.running, false);
    assert.equal(h.latest.storageLimited, true);
    assert.equal(h.latest.message, "storage-limit");
    assert.ok(stopped.world.time > 0 && stopped.world.time < 1800);
    for (const command of [
      { type: "play" },
      { type: "targets", vehicleId: "drone-1", value: { altitude: 3 } },
      { type: "wind", value: { speed: 1, direction: 0 } },
      { type: "sensorConfig", vehicleId: "drone-2", value: lowRays },
    ]) {
      const rejected = await h.send(command);
      assert.equal(rejected.ok, false);
      assert.match(rejected.error, /上限/);
    }
    const selected = await h.call({
      type: "selectVehicle",
      vehicleId: "drone-4",
    });
    assert.equal(selected.activeVehicleId, "drone-4");
    assert.deepEqual(selected.world, stopped.world);
    const raw = await h.call({ type: "apiExport" });
    const bytes =
      Buffer.byteLength(JSON.stringify(raw.samples)) +
      Buffer.byteLength(JSON.stringify(raw.sensorFrames)) -
      2 +
      (raw.events.length
        ? Buffer.byteLength(JSON.stringify(raw.events)) - 1
        : 0);
    const lastFrameBytes =
      Buffer.byteLength(JSON.stringify(raw.samples.at(-1))) +
      Buffer.byteLength(JSON.stringify(raw.sensorFrames.at(-1))) +
      2;
    assert.ok(
      bytes >= DATA_LIMIT,
      `retained ${bytes} bytes must reach the unchanged 96 MiB production limit`,
    );
    assert.ok(
      bytes - DATA_LIMIT < lastFrameBytes,
      "overshoot is bounded by one atomic world sample",
    );
    const record = validateRecording(raw);
    assert.equal(record.samples.length, h.latest.sampleCount);
    assertFinalWorld(record, stopped);
    assert.ok(
      record.sensorFrames
        .at(-1)
        .vehicles.every(
          (v) =>
            v.sensors.lidar.rangesM.length === 5760 && v.sensors.lidar.valid,
        ),
    );
    await h.advance(10);
    assert.deepEqual(
      (await h.call({ type: "inspectWorld" })).world,
      stopped.world,
    );
    t.diagnostic(
      `Production 96 MiB cap retained ${bytes} bytes across ${record.samples.length} frames at tick ${stopped.world.tick} in ${((performance.now() - started) / 1000).toFixed(2)} real seconds`,
    );
  },
);

test(
  "one DEM coverage failure atomically pauses every Worker vehicle at the last committed world tick",
  { timeout: 30000 },
  async (t) => {
    const h = await harness(t),
      input = fleet(2);
    input.scene.terrain = {
      kind: "dem",
      north: -5,
      east: -5,
      cellSize: 5,
      rows: 3,
      cols: 3,
      heights: Array(9).fill(0),
    };
    input.scene.geography = {
      origin: {
        latitudeDeg: 25.033,
        longitudeDeg: 121.5654,
        altitudeM: 123,
        verticalDatum: "orthometric",
      },
      source: {
        name: "small-dem.tif",
        sha256: "c".repeat(64),
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
    input.vehicles.forEach((v, i) => {
      v.spawn = { north: i ? 2 : -2, east: 0, altitude: 2, heading: 0 };
      v.home = [v.spawn.north, 0];
      v.config.noise = 0.1;
      v.config.turbulence = 0.2;
      v.setup.sensors = {
        ...v.setup.sensors,
        originAltitudeM: 123,
        originVerticalDatum: "orthometric",
        latency: 0.1,
        dropout: 0.2,
      };
    });
    const reference = createWorld(input);
    reference.command("drone-2", "targets", { roll: 0.2 });
    let lastCommitted,
      failed = false;
    for (let i = 0; i < 2000; i++) {
      lastCommitted = reference.snapshot();
      try {
        reference.step();
      } catch (error) {
        assert.equal(error.code, "TERRAIN_COVERAGE");
        assert.equal(error.vehicleId, "drone-2");
        failed = true;
        break;
      }
    }
    assert.equal(failed, true);
    assert.deepEqual(reference.snapshot(), lastCommitted);
    await h.call({ type: "resetWorld", value: input });
    await h.call({
      type: "targets",
      vehicleId: "drone-2",
      value: { roll: 0.2 },
    });
    await h.call({ type: "play" });
    const stopped = await h.untilStopped(100);
    assert.equal(stopped.running, false);
    assert.equal(h.latest.message, "terrain-limit");
    assert.deepEqual(stopped.termination, {
      reason: "terrain-coverage",
      time: lastCommitted.time,
    });
    assert.deepEqual(stopped.world, {
      ...lastCommitted,
      scene: input.scene,
      wind: input.wind,
    });
    assert.ok(h.errors.some((message) => /全體模擬已暫停/.test(message)));
    await h.advance(50);
    assert.deepEqual(
      (await h.call({ type: "inspectWorld" })).world,
      stopped.world,
    );
    const record = validateRecording(await h.call({ type: "apiExport" }));
    assertFinalWorld(record, stopped);
    assert.deepEqual(record.termination, stopped.termination);
    assert.ok(
      record.sensorFrames
        .at(-1)
        .vehicles.every((v) => v.sensors.gps.verticalDatum === "orthometric"),
    );
  },
);
