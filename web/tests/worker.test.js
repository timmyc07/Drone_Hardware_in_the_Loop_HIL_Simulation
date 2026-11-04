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
  assert.equal(record.version, 5);
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
  let requestSerial = 100;
  const call = async (command) => {
    const requestId = ++requestSerial;
    const response = next(
      (d) => d.type === "apiReply" && d.requestId === requestId,
    );
    worker.postMessage({ ...command, requestId });
    return response;
  };
  const wing = await call({
    type: "reset",
    config: { airframe: "fixedwing", geofenceRadius: 300 },
  });
  assert.equal(wing.ok, true);
  assert.equal(wing.value.snapshot.airframe, "fixedwing");
  assert.equal(wing.value.setup.training, "free");
  assert.ok(wing.value.snapshot.flight.airspeedMps > 10);
  assert.equal(
    (await call({ type: "targets", value: { airspeed: 16 } })).value.snapshot
      .target.airspeed,
    16,
  );
  assert.equal((await call({ type: "transition", value: "cruise" })).ok, false);
  const failedPusher = await call({
    type: "fault",
    fault: "pusher",
    value: { efficiency: 0 },
  });
  assert.equal(failedPusher.ok, true);
  assert.equal(failedPusher.value.snapshot.faults.pusherEfficiency, 0);
  const wingRecord = validateRecording(
    (await call({ type: "apiExport" })).value,
  );
  assert.equal(wingRecord.samples[0].airframe, "fixedwing");
  assert.ok(wingRecord.events.some((e) => e.fault === "pusher"));
  const vtol = await call({
    type: "reset",
    config: { airframe: "vtol", geofenceRadius: 300 },
  });
  assert.equal(vtol.ok, true);
  assert.equal(vtol.value.snapshot.flight.transition, "hover");
  assert.equal((await call({ type: "transition", value: "cruise" })).ok, false);
  await call({ type: "targets", value: { altitude: 12 } });
  await call({ type: "speed", value: 4 });
  const atTransitionHeight = next(
    (d) => d.type === "state" && -d.snapshot.position[2] >= 9,
  );
  await call({ type: "play" });
  await atTransitionHeight;
  await call({ type: "pause" });
  const transition = await call({ type: "transition", value: "cruise" });
  assert.equal(transition.ok, true);
  const vtolRecord = validateRecording(
    (await call({ type: "apiExport" })).value,
  );
  assert.equal(vtolRecord.samples[0].airframe, "vtol");
  assert.equal(vtolRecord.events.at(-1).type, "transition");
  const ground = await call({
    type: "reset",
    config: { airframe: "fixedwing", startOnGround: true },
    setup: { training: "runway" },
  });
  assert.equal(ground.ok, true);
  assert.equal(ground.value.snapshot.armed, false);
  assert.equal(ground.value.snapshot.position[2], 0);
  assert.equal(ground.value.snapshot.runway.weightOnWheels, true);
  assert.equal(ground.value.running, false);
  assert.equal(
    (
      await call({
        type: "reset",
        config: { airframe: "fixedwing", startOnGround: "true" },
      })
    ).ok,
    false,
  );
  const runup = await call({ type: "takeoff" });
  assert.equal(runup.ok, true);
  assert.equal(runup.value.snapshot.navigation.mode, "takeoff");
  assert.equal(runup.value.snapshot.armed, true);
  assert.equal(runup.value.running, false);
  const aborted = await call({ type: "abortTakeoff", value: 1n });
  assert.equal(aborted.ok, true);
  assert.equal(aborted.value.snapshot.runway.takeoffPhase, "aborted");
  const groundRecord = validateRecording(
    (await call({ type: "apiExport" })).value,
  );
  assert.equal(groundRecord.config.startOnGround, true);
  assert.equal(groundRecord.setup.training, "runway");
  assert.equal(groundRecord.samples.at(-1).runway.abortReason, "pilot");
  assert.equal(groundRecord.events.at(-1).type, "abortTakeoff");
  assert.equal(Object.hasOwn(groundRecord.events.at(-1), "value"), false);
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
    let configured = false;
    const finished = new Promise((resolve, reject) => {
      worker.on("error", reject);
      worker.on("message", (data) => {
        if (data.message === "reset") {
          if (!configured) {
            configured = true;
            // Keep payload small so this exercises the time limit rather than the separate data cap.
            worker.postMessage({
              type: "reset",
              setup: { sensors: { lidarRays: 12, dropout: 1 } },
            });
            return;
          }
          worker.postMessage({ type: "speed", value: 4 });
          worker.postMessage({ type: "play" });
        }
        if (data.message === "limit") resolve(data);
        if (data.message === "storage-limit")
          reject(
            new Error("Time-limit fixture reached storage capacity first"),
          );
      });
    });
    const state = await finished;
    assert.equal(state.running, false);
    assert.ok(Math.abs(state.snapshot.time - 1800) < 1e-7);
    assert.equal(state.sampleCount, 36001);
    assert.equal(state.snapshot.metrics.steps, 360000);
  },
);

test(
  "dense scans stop at the data limit and still export the final state",
  { timeout: 20000 },
  async (t) => {
    const url = new URL("../worker.js", import.meta.url).href;
    const worker = new Worker(
      `
    import {parentPort} from 'node:worker_threads';
    let now=0;
    globalThis.performance={now:()=>now};
    globalThis.setTimeout=fn=>setImmediate(()=>{now+=100;fn();});
    globalThis.postMessage=data=>parentPort.postMessage(data);
    globalThis.onmessage=null;
    parentPort.on('message',data=>globalThis.onmessage({data}));
    await import(${JSON.stringify(url)});
  `,
      { eval: true, type: "module" },
    );
    t.after(() => worker.terminate());
    let configured = false;
    const finished = new Promise((resolve, reject) => {
      worker.on("error", reject);
      worker.on("message", (data) => {
        if (data.type === "error") reject(new Error(data.message));
        if (data.message === "reset") {
          if (!configured) {
            configured = true;
            worker.postMessage({
              type: "reset",
              setup: {
                sensors: { lidarRays: 180 },
                obstacles: [
                  { north: 4, east: 0, width: 0.1, depth: 12, height: 6 },
                  { north: -4, east: 0, width: 0.1, depth: 12, height: 6 },
                  { north: 0, east: 4, width: 12, depth: 0.1, height: 6 },
                  { north: 0, east: -4, width: 12, depth: 0.1, height: 6 },
                ],
              },
            });
          } else {
            worker.postMessage({ type: "speed", value: 4 });
            worker.postMessage({ type: "play" });
          }
        }
        if (data.message === "storage-limit") resolve(data);
      });
    });
    const stopped = await finished;
    assert.equal(stopped.running, false);
    assert.equal(stopped.storageLimited, true);
    assert.ok(stopped.snapshot.time > 0 && stopped.snapshot.time < 1800);
    const call = (command) =>
      new Promise((resolve) => {
        const receive = (data) => {
          if (
            data.type === "apiReply" &&
            data.requestId === command.requestId
          ) {
            worker.off("message", receive);
            resolve(data);
          }
        };
        worker.on("message", receive);
        worker.postMessage(command);
      });
    assert.equal((await call({ type: "play", requestId: 1 })).ok, false);
    const exported = await call({ type: "apiExport", requestId: 2 });
    const record = validateRecording(exported.value);
    assert.equal(record.samples.at(-1).time, stopped.snapshot.time);
    assert.equal(record.sensorFrames.at(-1).time, stopped.snapshot.time);
    assert.equal(record.samples.length, stopped.sampleCount);
  },
);

test(
  "environment changes after an air-started fixed wing lands and begins runway takeoff",
  { timeout: 20000 },
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
    parentPort.on('message', data => globalThis.onmessage({ data }));
    await import(${JSON.stringify(url)});
  `,
      { eval: true, type: "module" },
    );
    t.after(() => worker.terminate());
    const next = (predicate) =>
      new Promise((resolve, reject) => {
        const timer = setTimeout(() => {
          cleanup();
          reject(new Error("Worker reply timeout"));
        }, 15000);
        const cleanup = () => {
          clearTimeout(timer);
          worker.off("message", receive);
          worker.off("error", failed);
        };
        const receive = (data) => {
          if (predicate(data)) {
            cleanup();
            resolve(data);
          }
        };
        const failed = (error) => {
          cleanup();
          reject(error);
        };
        worker.on("message", receive);
        worker.once("error", failed);
      });
    let serial = 0;
    const call = (command) => {
      const requestId = ++serial;
      const result = next(
        (data) => data.type === "apiReply" && data.requestId === requestId,
      );
      worker.postMessage({ ...command, requestId });
      return result;
    };
    await next((data) => data.type === "state");
    assert.equal(
      (
        await call({
          type: "reset",
          config: { airframe: "fixedwing", startOnGround: false },
          scenario: "hover",
        })
      ).ok,
      true,
    );
    assert.equal((await call({ type: "mode", value: "land" })).ok, true);
    await call({ type: "speed", value: 4 });
    const landed = next(
      (data) =>
        data.type === "state" &&
        data.snapshot.status === "landed" &&
        !data.snapshot.armed &&
        data.snapshot.runway.onRunway,
    );
    assert.equal((await call({ type: "play" })).ok, true);
    await landed;
    assert.equal((await call({ type: "pause" })).ok, true);
    const takeoff = await call({ type: "takeoff" });
    assert.equal(takeoff.ok, true);
    assert.equal(takeoff.value.snapshot.navigation.mode, "takeoff");
    const environment = { preset: "field", timeOfDay: 18, fog: 0.2, rain: 0.1 };
    const changed = await call({ type: "environment", value: environment });
    assert.equal(changed.ok, true);
    assert.deepEqual(changed.value.setup.environment, environment);
    const invalid = await call({
      type: "environment",
      value: { ...environment, fog: 2 },
    });
    assert.equal(invalid.ok, false);
    assert.deepEqual(
      (await call({ type: "inspect" })).value.setup.environment,
      environment,
    );
    const exported = await call({ type: "apiExport" });
    assert.equal(exported.ok, true);
    const recording = validateRecording(exported.value);
    assert.equal(recording.version, 5);
    assert.ok(
      recording.events.some(
        (event) =>
          event.type === "environment" && event.value.preset === "field",
      ),
    );
  },
);
