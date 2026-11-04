import test from "node:test";
import assert from "node:assert/strict";
import { Worker } from "node:worker_threads";
import { validateRecording } from "../recording.js";

test("worker runs, pauses, records simulation-time samples and exports validated data", async (t) => {
  const url = new URL("../worker.js", import.meta.url).href;
  const worker = new Worker(
    `
    import { parentPort } from 'node:worker_threads';
    globalThis.postMessage = data => parentPort.postMessage(data);
    globalThis.onmessage = null;
    parentPort.on('message', data => globalThis.onmessage({data}));
    await import(${JSON.stringify(url)});
  `,
    { eval: true, type: "module" },
  );
  t.after(() => worker.terminate());
  const next = (predicate) =>
    new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        worker.off("message", receive);
        reject(new Error("Worker reply timeout"));
      }, 3000);
      function receive(data) {
        if (predicate(data)) {
          clearTimeout(timer);
          worker.off("message", receive);
          resolve(data);
        }
      }
      worker.on("message", receive);
    });
  await next((d) => d.type === "state");
  let reply = next((d) => d.type === "state" && d.message === "reset");
  worker.postMessage({ type: "reset", config: { seed: 18 }, scenario: "step" });
  const resetState = await reply;
  assert.equal(resetState.config.seed, 18);
  assert.equal(resetState.scenario, "step");
  reply = next((d) => d.type === "apiReply" && d.requestId === 1);
  worker.postMessage({ type: "inspect", requestId: 1 });
  const inspected = await reply;
  assert.equal(inspected.ok, true);
  assert.equal(inspected.value.sensors.barometer.valid, true);
  assert.ok(inspected.value.sensors.barometer.pressurePa > 0);
  reply = next((d) => d.type === "apiReply" && d.requestId === 2);
  worker.postMessage({
    type: "sensorFault",
    value: { sensor: "gps", failed: true },
    requestId: 2,
  });
  const faulted = await reply;
  assert.equal(faulted.ok, true);
  assert.equal(faulted.value.sensors.gps.valid, false);
  assert.equal(faulted.value.sensors.gps.reason, "fault");
  reply = next((d) => d.type === "apiReply" && d.requestId === 3);
  worker.postMessage({
    type: "sensorFault",
    value: { sensor: "gps", failed: false },
    requestId: 3,
  });
  assert.equal((await reply).ok, true);
  reply = next((d) => d.type === "apiReply" && d.requestId === 4);
  worker.postMessage({
    type: "sensorConfig",
    value: { gpsNoiseM: 0 },
    requestId: 4,
  });
  assert.equal((await reply).ok, true);
  reply = next(
    (d) => d.type === "state" && d.snapshot.navigation.waypoints.length === 1,
  );
  worker.postMessage({
    type: "mission",
    value: [{ north: 2, east: 0, altitude: 2, hold: 1 }],
  });
  await reply;
  reply = next(
    (d) => d.type === "state" && d.snapshot.navigation.mode === "mission",
  );
  worker.postMessage({ type: "mode", value: "mission" });
  await reply;
  reply = next(
    (d) => d.type === "state" && d.snapshot.training.course === "hover",
  );
  worker.postMessage({ type: "training", value: "hover" });
  await reply;
  reply = next(
    (d) => d.type === "state" && d.setup.environment.preset === "field",
  );
  worker.postMessage({
    type: "environment",
    value: { preset: "field", timeOfDay: 18, fog: 0.2, rain: 0.1 },
  });
  await reply;
  worker.postMessage({ type: "render", value: false });
  reply = next((d) => d.type === "state" && d.snapshot.time >= 0.15);
  worker.postMessage({ type: "play" });
  const active = await reply;
  assert.ok(active.running);
  assert.ok(active.samples.length > 0);
  reply = next((d) => d.type === "state" && !d.running);
  worker.postMessage({ type: "pause" });
  const paused = await reply;
  reply = next((d) => d.type === "recording");
  worker.postMessage({ type: "export", kind: "json" });
  const exported = await reply;
  const record = validateRecording(exported.recording);
  assert.equal(record.config.seed, 18);
  assert.equal(record.scenario, "step");
  assert.equal(record.version, 3);
  assert.equal(record.sensorFrames.length, record.samples.length);
  assert.equal(record.sensorFrames[0].time, record.samples[0].time);
  assert.equal(record.setup.sensors.gpsNoiseM, 1.5);
  assert.ok(
    record.events.some(
      (e) => e.type === "sensorConfig" && e.value.gpsNoiseM === 0,
    ),
  );
  assert.ok(record.sensorFrames.at(-1).gps.valid);
  assert.equal(record.samples[0].navigation.waypoints.length, 1);
  assert.equal(record.samples[0].navigation.mode, "mission");
  assert.equal(record.samples[0].training.course, "hover");
  assert.equal(record.runtime.renderEnabled, false);
  assert.ok(record.samples.at(-1).battery.usedWh > 0);
  assert.ok(
    record.events.some(
      (e) => e.type === "environment" && e.value.preset === "field",
    ),
  );
  assert.equal(record.samples[0].time, 0);
  assert.ok(record.samples.at(-1).time <= paused.snapshot.time);
  assert.equal(record.samples.at(-1).time, paused.snapshot.time);
  for (let i = 1; i < record.samples.length - 1; i++)
    assert.ok(
      Math.abs(record.samples[i].time - record.samples[i - 1].time - 0.05) <
        1e-8,
    );
  reply = next((d) => d.type === "error");
  worker.postMessage({ type: "targets", value: { roll: Infinity } });
  assert.match((await reply).message, /roll/);
  reply = next((d) => d.type === "apiReply" && d.requestId === 5);
  worker.postMessage({
    type: "targets",
    value: { roll: Infinity },
    requestId: 5,
  });
  assert.equal((await reply).ok, false);
  reply = next((d) => d.type === "apiReply" && d.requestId === 6);
  worker.postMessage({ type: "gust", value: 1n, fault: 1n, requestId: 6 });
  assert.equal((await reply).ok, true);
  reply = next((d) => d.type === "apiReply" && d.requestId === 7);
  worker.postMessage({ type: "apiExport", requestId: 7 });
  const apiRecord = validateRecording((await reply).value);
  assert.equal(apiRecord.events.at(-1).type, "gust");
  assert.equal(Object.hasOwn(apiRecord.events.at(-1), "value"), false);
  assert.equal(Object.hasOwn(apiRecord.events.at(-1), "fault"), false);
  reply = next((d) => d.type === "state" && d.message === "reset");
  worker.postMessage({
    type: "reset",
    config: { geofenceRadius: 1 },
    scenario: "hover",
  });
  const reset = await reply;
  assert.equal(reset.snapshot.navigation.mode, "attitude");
  assert.equal(reset.snapshot.navigation.waypoints.length, 0);
  assert.equal(reset.setup.environment.preset, "field");
});

test(
  "worker pauses at the 30 minute recording limit without losing the final state",
  { timeout: 15000 },
  async (t) => {
    const url = new URL("../worker.js", import.meta.url).href;
    const worker = new Worker(
      `
    import { parentPort } from 'node:worker_threads';
    let now = 0;
    globalThis.performance = { now: () => now };
    globalThis.setTimeout = fn => setImmediate(() => { now += 100; fn(); });
    globalThis.postMessage = data => parentPort.postMessage(data);
    globalThis.onmessage = null;
    parentPort.on('message', data => globalThis.onmessage({data}));
    await import(${JSON.stringify(url)});
  `,
      { eval: true, type: "module" },
    );
    t.after(() => worker.terminate());
    const finished = new Promise((resolve, reject) => {
      worker.on("error", reject);
      worker.on("message", (data) => {
        if (data.message === "reset") {
          worker.postMessage({ type: "speed", value: 4 });
          worker.postMessage({ type: "play" });
        }
        if (data.message === "limit") resolve(data);
      });
    });
    const state = await finished;
    assert.equal(state.running, false);
    assert.ok(Math.abs(state.snapshot.time - 1800) < 1e-7);
    assert.equal(state.sampleCount, 36001);
    assert.equal(state.snapshot.metrics.steps, 360000);
  },
);
