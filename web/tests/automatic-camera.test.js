import test from "node:test";
import assert from "node:assert/strict";
import { createAutomaticCamera } from "../automatic-camera.js";

const settings = { mode: "time", interval: 1, maxFrames: 30 };
const observation = (patch = {}) => ({ vehicleId: "drone-1", epoch: 0, worldTick: 0,
  time: 0, position: [0, 0, -1], running: true, airborne: true, available: true,
  busy: false, nowMs: 0, ...patch });
const frame = (o, patch = {}) => ({ captureId: "12345678-1234-4123-8123-123456789abc",
  sourceVehicleId: o.vehicleId, worldTick: o.worldTick, time: o.time,
  pose: { positionNed: [...o.position] }, ...patch });
const flush = async () => { for (let i = 0; i < 8; i++) await Promise.resolve(); };
function setup(options = {}) {
  let current = observation();
  const captured = [], saved = [], changes = [];
  const api = createAutomaticCamera({ capture: (id) => {
    assert.equal(id, current.vehicleId);
    captured.push(current);
    return options.capture ? options.capture(current) : Promise.resolve(frame(current));
  }, save: (value) => { options.save?.(value); saved.push(value); },
  onChange: (value) => { changes.push(value); return options.onChange?.(value); } });
  return { api, captured, saved, changes,
    start(s = settings, o = observation()) { current = o; return api.start(s, o); },
    observe(o) { current = o; return api.observe(o); } };
}

test("time uses simulation clock and enforces 500 ms wall interval without catch-up", async () => {
  const h = setup(); h.start(); await flush();
  h.observe(observation({ worldTick: 1, time: 10, nowMs: 499 }));
  assert.equal(h.captured.length, 1);
  h.observe(observation({ worldTick: 2, time: 10, nowMs: 500 })); await flush();
  h.observe(observation({ worldTick: 3, time: 10.5, nowMs: 5000 }));
  assert.equal(h.captured.length, 2);
  h.observe(observation({ worldTick: 4, time: 11, nowMs: 5001 })); await flush();
  assert.equal(h.saved.length, 3);
});

test("distance accumulates turns, ignores vertical motion, repeated ticks and hover", async () => {
  const h = setup(); h.start({ ...settings, mode: "distance", interval: 2 }); await flush();
  h.observe(observation({ worldTick: 1, time: 1, nowMs: 500, position: [1, 0, -2] }));
  h.observe(observation({ worldTick: 1, time: 1, nowMs: 600, position: [1, 0, -2] }));
  h.observe(observation({ worldTick: 2, time: 2, nowMs: 1000, position: [1, 0, -20] }));
  h.observe(observation({ worldTick: 3, time: 3, nowMs: 1500, position: [1, 0, -20] }));
  assert.equal(h.captured.length, 1);
  h.observe(observation({ worldTick: 4, time: 4, nowMs: 2000, position: [1, 1, -20] })); await flush();
  assert.equal(h.saved.length, 2);
  assert.equal(h.api.getState().last.distanceM, 2);
});

test("ground and paused starts wait; pause/resume and external busy do not fail", async () => {
  const h = setup(); h.start(settings, observation({ running: false, airborne: false }));
  assert.equal(h.api.getState().phase, "paused");
  h.observe(observation({ worldTick: 1, time: 1, nowMs: 500, airborne: false }));
  h.observe(observation({ worldTick: 2, time: 2, nowMs: 1000, busy: true }));
  assert.equal(h.captured.length, 0);
  h.observe(observation({ worldTick: 3, time: 3, nowMs: 1500 })); await flush();
  h.observe(observation({ worldTick: 3, time: 3, nowMs: 2000, running: false }));
  assert.equal(h.api.getState().phase, "paused");
  h.observe(observation({ worldTick: 4, time: 4, nowMs: 2500 })); await flush();
  assert.equal(h.saved.length, 2);
});

test("single pending keeps distance observed during GPU delay", async () => {
  let resolve;
  const h = setup({ capture: (o) => new Promise((done) => { resolve = () => done(frame(o)); }) });
  h.start({ ...settings, mode: "distance", interval: 2 });
  h.observe(observation({ worldTick: 1, time: 1, nowMs: 500, position: [2, 0, -1] }));
  h.observe(observation({ worldTick: 2, time: 2, nowMs: 1000, position: [2, 2, -1] }));
  assert.equal(h.captured.length, 1);
  resolve(); await flush();
  h.observe(observation({ worldTick: 2, time: 2, nowMs: 1500, position: [2, 2, -1] }));
  assert.equal(h.captured.length, 2);
  resolve(); await flush();
  assert.equal(h.api.getState().last.distanceM, 4);
});

test("stop discards pending result and blocks start until GPU completion", async () => {
  let resolve;
  const h = setup({ capture: (o) => new Promise((done) => { resolve = () => done(frame(o)); }) });
  h.start(); h.api.stop();
  assert.throws(() => h.start());
  resolve(); await flush();
  assert.equal(h.saved.length, 0);
  h.start(settings, observation({ nowMs: 500 })); resolve(); await flush();
  assert.equal(h.saved.length, 1);
});

for (const patch of [{ epoch: 1 }, { vehicleId: "drone-2" }, { available: false },
  { time: 0 }, { worldTick: 0, time: 2 }]) {
  test(`identity/reset/clock change stops and discards pending ${JSON.stringify(patch)}`, async () => {
    let resolve;
    const h = setup({ capture: (o) => new Promise((done) => { resolve = () => done(frame(o)); }) });
    h.start(settings, observation({ worldTick: 1, time: 1 }));
    h.observe(observation({ worldTick: 2, time: 2, nowMs: 500, ...patch }));
    assert.equal(h.api.getState().active, false);
    resolve(); await flush(); assert.equal(h.saved.length, 0);
  });
}

test("maxFrames completes and preserves true frame object", async () => {
  const h = setup(); h.start({ ...settings, maxFrames: 1 }); await flush();
  assert.equal(h.api.getState().phase, "complete");
  h.observe(observation({ time: 9, worldTick: 9, nowMs: 9000 }));
  assert.equal(h.saved.length, 1);
});

for (const patch of [{ sourceVehicleId: "other" }, { worldTick: 2 }, { time: 0.01 },
  { time: NaN }, { pose: { positionNed: [0, 0, 0] } }, { pose: { positionNed: [0, Infinity, -1] } },
  { captureId: "" }]) {
  test(`rejects mismatched frame ${JSON.stringify(patch)}`, async () => {
    const h = setup({ capture: (o) => Promise.resolve(frame(o, patch)) });
    h.start(); await flush();
    assert.equal(h.api.getState().phase, "error");
    assert.equal(h.saved.length, 0);
    assert.equal(h.api.getState().pending, false);
  });
}

test("save capacity errors preserve previous saved count; capture rejection handled", async () => {
  let attempts = 0;
  const h = setup({ save: () => { if (++attempts === 2) throw new Error("容量不足"); } });
  h.start(); await flush();
  h.observe(observation({ time: 1, worldTick: 1, nowMs: 500 })); await flush();
  assert.equal(h.api.getState().saved, 1);
  assert.equal(h.api.getState().reason, "容量不足");
  const rejected = setup({ capture: () => Promise.reject(new Error("GPU failed")) });
  rejected.start(); await flush();
  assert.equal(rejected.api.getState().phase, "error");
});

test("settings/observation/state are copied and notifications only report changes", async () => {
  const h = setup(); const s = { ...settings }; const o = observation();
  h.start(s, o); s.maxFrames = 1; o.position[0] = 999;
  await flush();
  assert.equal(h.api.getState().saved, 1);
  assert.equal(h.api.getState().active, true);
  const state = h.api.getState(); state.settings.maxFrames = 1; state.last.position[0] = 888;
  assert.equal(h.api.getState().last.position[0], 0);
  const count = h.changes.length;
  h.observe(observation()); h.observe(observation());
  assert.equal(h.changes.length, count);
  assert.equal("pose" in h.api.getState().last, false);
});

test("strict validation rejects bad settings and observation", () => {
  for (const s of [{ ...settings, interval: 0.1 }, { ...settings, maxFrames: 31 },
    { ...settings, mode: "other" }, { ...settings, unexpected: true },
    { ...settings, mode: "distance", interval: 501 }]) assert.throws(() => setup().start(s));
  for (const patch of [{ time: Infinity }, { position: [0, 1] }, { running: 1 },
    { epoch: "1" }, { worldTick: 0.5 }]) assert.throws(() => setup().start(settings, observation(patch)));
});

test("reentrant notifications cannot double capture and can cancel pending capture", async () => {
  let api; let captures = 0;
  api = createAutomaticCamera({ capture: () => { captures++; return Promise.resolve(frame(observation())); },
    save() {}, onChange: (s) => { if (s.pending) api.observe(observation()); } });
  api.start(settings, observation()); await flush(); assert.equal(captures, 1);
  api = createAutomaticCamera({ capture: () => { captures++; return Promise.resolve(frame(observation())); }, save() {},
    onChange: (s) => { if (s.pending && s.active) api.stop(); } });
  api.start(settings, observation()); await flush(); assert.equal(captures, 2);
  assert.equal(api.getState().pending, false);
});

test("capture freezes before notification advances observation; saves original frame", async () => {
  let api, capturedFrame, savedFrame;
  let advanced = false;
  api = createAutomaticCamera({ capture: () => { capturedFrame = frame(observation()); return Promise.resolve(capturedFrame); },
    save: (value) => { savedFrame = value; }, onChange: (s) => {
      if (s.pending && !advanced) { advanced = true; api.observe(observation({ worldTick: 1, time: 1, nowMs: 500 })); }
    } });
  api.start(settings, observation()); await flush();
  assert.equal(savedFrame, capturedFrame);
  assert.equal(api.getState().last.worldTick, 0);
});

test("synchronous capture failure and asynchronous observer errors are contained", async () => {
  const h = setup({ capture: () => { throw new Error("capture failed"); },
    onChange: () => Promise.reject(new Error("UI failed")) });
  h.start(); await flush();
  assert.equal(h.api.getState().phase, "error");
  assert.equal(h.api.getState().reason, "capture failed");
  assert.equal(h.api.getState().pending, false);
});

test("sparse positions and negative clocks are rejected at observation boundary", () => {
  const sparse = [0, , -1];
  for (const patch of [{ position: sparse }, { time: -1 }, { nowMs: -1 }]) {
    assert.throws(() => setup().start(settings, observation(patch)));
  }
});

test("sparse frame positions cannot pass pose validation", async () => {
  const h = setup({ capture: (o) => Promise.resolve(frame(o, {
    pose: { positionNed: [0, , -1] },
  })) });
  h.start(); await flush();
  assert.equal(h.api.getState().phase, "error");
  assert.equal(h.saved.length, 0);
});

test("settings require exactly three own keys on a plain object", async () => {
  const inherited = Object.create(settings);
  const custom = Object.assign(Object.create({ extra: true }), settings);
  const array = Object.assign([], settings);
  const symbol = { ...settings, [Symbol("extra")]: true };
  const hidden = Object.defineProperty({ ...settings }, "extra", { value: true });
  for (const value of [inherited, custom, array, symbol, hidden, { mode: "time", interval: 1 }]) {
    assert.throws(() => setup().start(value));
  }
  const h = setup();
  h.start(Object.assign(Object.create(null), settings)); await flush();
  assert.equal(h.saved.length, 1);
});

test("terminal pause snapshot contributes final airborne distance exactly once", async () => {
  const h = setup();
  h.start({ ...settings, mode: "distance", interval: 2 }); await flush();
  const paused = observation({ worldTick: 1, time: 1, nowMs: 500,
    running: false, position: [2, 0, -1] });
  h.observe(paused); h.observe(paused);
  assert.equal(h.captured.length, 1);
  h.observe({ ...paused, running: true, nowMs: 1000 }); await flush();
  assert.equal(h.saved.length, 2);
  assert.equal(h.api.getState().last.distanceM, 2);
});
