import test from "node:test";
import assert from "node:assert/strict";
import { createAutomaticCameraControls } from "../automatic-camera-controls.js";

function setup() {
  class Element {
    constructor(value = "") {
      this.value = value;
      this.textContent = "";
      this.disabled = false;
      this.listeners = {};
    }
    addEventListener(name, callback) { this.listeners[name] = callback; }
    dispatch(name) { if (!this.disabled) this.listeners[name](); }
  }
  const elements = Object.fromEntries(
    ["mode", "interval", "limit", "start", "stop", "status", "interval-label"]
      .map((name) => [name, new Element()]),
  );
  elements.mode.value = "time";
  elements.interval.value = "2";
  elements.limit.value = "10";
  globalThis.document = { getElementById: (id) => elements[id.replace(/^camera-auto-/, "")] };
  const context = { available: true, busy: false, vehicleName: "一號機" };
  const starts = [];
  let stops = 0;
  const controls = createAutomaticCameraControls({
    start: (settings) => starts.push(settings),
    stop: () => { stops++; },
    getContext: () => context,
  });
  return { elements, context, starts, controls, get stops() { return stops; } };
}
const idle = { active: false, pending: false, phase: "idle", settings: null };

test("validates settings, including blank values, and starts with selected settings", () => {
  const { elements: e, starts } = setup();
  for (const value of ["", " ", "0.4", "301", "Infinity"]) {
    e.interval.value = value;
    e.start.dispatch("click");
    assert.match(e.status.textContent, /設定無效/);
  }
  e.interval.value = "2";
  e.limit.value = "1.5";
  e.start.dispatch("click");
  assert.equal(starts.length, 0);
  e.mode.value = "distance";
  e.mode.dispatch("change");
  assert.match(e["interval-label"].textContent, /公尺/);
  e.interval.value = "0.25";
  e.limit.value = "30";
  e.start.dispatch("click");
  assert.deepEqual(starts, [{ mode: "distance", interval: 0.25, maxFrames: 30 }]);
});

test("refresh locks active and pending controls, allows stop and preserves edits", () => {
  const result = setup();
  const { elements: e, controls } = result;
  e.interval.value = "12.5";
  controls.refresh({ active: true, pending: true, phase: "paused", vehicleId: "alpha", saved: 2, settings: { maxFrames: 10 }, last: { time: 4, worldTick: 20 } });
  assert.equal(e.interval.value, "12.5");
  assert.equal(e.mode.disabled, true);
  assert.equal(e.start.disabled, true);
  assert.equal(e.stop.disabled, false);
  assert.match(e.status.textContent, /暫停.*alpha.*2 \/ 10.*世界步 20/);
  e.stop.dispatch("click");
  assert.equal(result.stops, 1);
  controls.refresh({ ...idle, pending: true });
  assert.equal(e.stop.disabled, true);
  assert.equal(e.start.disabled, true);
  controls.refresh({ ...idle, phase: "complete" });
  assert.equal(e.start.disabled, false);
  assert.equal(e.interval.value, "12.5");
});

test("unavailable replay disables start and reports reason while active stop remains available", () => {
  const { elements: e, context, controls } = setup();
  context.available = false;
  context.reason = "重播時無法拍攝";
  controls.refresh(idle);
  assert.equal(e.start.disabled, true);
  assert.match(e.status.textContent, /重播/);
  controls.refresh({ ...idle, active: true, phase: "waiting" });
  assert.equal(e.stop.disabled, false);
});

test("synchronous start errors are displayed", () => {
  const { elements: e } = setup();
  const controls = createAutomaticCameraControls({ start: () => { throw new Error("相機忙碌"); }, stop() {}, getContext: () => ({ available: true, busy: false }) });
  controls.refresh(idle);
  e.start.dispatch("click");
  assert.match(e.status.textContent, /相機忙碌/);
  controls.refresh(idle);
  controls.refresh(idle);
  assert.match(e.status.textContent, /相機忙碌/);
  e.limit.dispatch("input");
  assert.doesNotMatch(e.status.textContent, /相機忙碌/);
});


test("feedback survives refresh until editing or active state", () => {
  const { elements: e, controls } = setup();
  e.interval.value = "";
  e.start.dispatch("click");
  controls.refresh(idle);
  controls.refresh(idle);
  assert.match(e.status.textContent, /設定無效/);
  e.interval.value = "2";
  e.interval.dispatch("input");
  assert.doesNotMatch(e.status.textContent, /設定無效/);
  e.limit.value = "";
  e.start.dispatch("click");
  controls.refresh({ ...idle, active: true, phase: "waiting" });
  assert.doesNotMatch(e.status.textContent, /設定無效/);
  controls.refresh({ ...idle, last: { time: 0.30000000000000004, worldTick: 3 } });
  assert.match(e.status.textContent, /0\.300 秒/);
});
