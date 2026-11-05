import test from "node:test";
import assert from "node:assert/strict";
import { initOperator } from "../operator.js";

function setup(hash = "#session=12345678-1234-4123-8123-123456789abc") {
  class Element {
    value = ""; textContent = ""; disabled = false; checked = false; hidden = false; dataset = {}; handlers = {}; options = [];
    classList = { toggle() {} };
    addEventListener(name, fn) { this.handlers[name] = fn; }
    replaceChildren() { this.options = []; this.value = ""; }
    add(option) { this.options.push(option); if (this.options.length === 1) this.value = option.value; }
    getContext() { return { putImageData() {} }; }
  }
  globalThis.Option = class { constructor(text, value) { this.text = text; this.value = value; } };
  globalThis.ImageData = class { constructor(data, width, height) { Object.assign(this, { data, width, height }); } };
  const elements = new Map();
  const doc = { hidden: false, getElementById(id) { if (!elements.has(id)) elements.set(id, new Element()); return elements.get(id); } };
  let tick, callback, calls = [], current = { connection: "connected", context: { available: true, vehicles: [{ id: "a", name: "A", airframe: "quadrotor", status: "ready", positionNed: [0, 1, 2], gimbal: {} }], worldTick: 3, time: 1, running: false }, bindingId: "one", vehicleId: "a", frame: null, pendingFrame: false, pendingGimbal: false, lastResult: null, error: null };
  const win = { location: { hash }, setInterval(fn) { tick = fn; return 1; }, clearInterval() { tick = null; }, addEventListener() {} };
  const client = { getState: () => current, bind: (id) => (calls.push(["bind", id]), true), requestFrame: () => (calls.push(["frame"]), true), setGimbal: (v) => (calls.push(["gimbal", v]), true), disconnect() {}, close() {} };
  const result = initOperator(doc, win, ({ onChange }) => { callback = onChange; return client; });
  return { elements, calls, result, doc, tick: () => tick?.(), update(value) { current = { ...current, ...value }; callback(current); } };
}
test("invalid session does not open a channel", () => {
  const { elements } = setup("#session=bad");
  assert.match(elements.get("error-status").textContent, /session/);
  assert.equal(elements.get("capture-button").disabled, true);
});
test("gimbal commands use radians and show only acknowledged targets", () => {
  const { elements: e, calls, update } = setup();
  assert.equal(e.get("gimbal-pitch").value, 15);
  e.get("gimbal-yaw").value = "90"; e.get("gimbal-pitch").value = "-30"; e.get("gimbal-roll").value = "0";
  e.get("gimbal-form").handlers.submit({ preventDefault() {} });
  assert.ok(Math.abs(calls[0][1].target.yaw - Math.PI / 2) < 1e-12);
  assert.match(e.get("apply-result").textContent, /尚未/);
  update({ lastResult: { gimbal: calls[0][1], appliedWorldTick: 7 } });
  assert.match(e.get("apply-result").textContent, /tick 7.*暫停/);
});
test("automatic capture skips hidden documents and pending frames", () => {
  const { elements: e, calls, tick, update, result, doc } = setup();
  e.get("auto-capture").checked = true;
  tick(); assert.equal(calls.length, 1);
  update({ pendingFrame: true }); tick(); assert.equal(calls.length, 1);
  update({ pendingFrame: false }); doc.hidden = true; tick(); assert.equal(calls.length, 1);
  result.close(); tick(); assert.equal(calls.length, 1);
});

test("retains stale frame after reconnect until a new binding clears it", async () => {
  const { cameraMetadata, DEFAULT_CAMERA_CONFIG } = await import("../camera.js");
  const { createSimulation } = await import("../sim.js");
  const { elements: e, update } = setup();
  const metadata = cameraMetadata({ time: 1, airframe: "quadrotor", position: [0, 0, 0], quaternion: [1, 0, 0, 0], gimbal: createSimulation().snapshot().gimbal }, DEFAULT_CAMERA_CONFIG, null, { sourceVehicleId: "a", worldTick: 3, captureId: "12345678-1234-4123-8123-123456789abc" });
  const pixels = metadata.width * metadata.height;
  const frame = { ...metadata, rgbRgba: new Uint8Array(pixels * 4), depthM: new Float32Array(pixels), classes: new Uint8Array(pixels) };
  update({ frame });
  update({ connection: "stale", bindingId: null, vehicleId: null });
  assert.match(e.get("frame-status").textContent, /過期/);
  update({ connection: "connected" });
  assert.match(e.get("frame-status").textContent, /過期.*重新綁定/);
  assert.equal(e.get("capture-button").disabled, true);
  assert.equal(e.get("frame-placeholder").hidden, true);
  update({ bindingId: "two", vehicleId: "a", frame: null });
  assert.match(e.get("frame-status").textContent, /等待影像/);
  assert.equal(e.get("frame-placeholder").hidden, false);
});

test("new binding initializes current gimbal once without overwriting edits", () => {
  const { elements: e, update } = setup();
  const vehicle = { id: "a", name: "A", airframe: "quadrotor", status: "ready", positionNed: [0, 1, 2], gimbal: { mode: "ned", target: { yaw: Math.PI / 4, pitch: 35 * Math.PI / 180, roll: 0 }, actual: { yaw: 0, pitch: Math.PI / 6, roll: 0 } } };
  const context = { available: true, vehicles: [vehicle], worldTick: 3, time: 1, running: false };
  update({ bindingId: "two", context });
  assert.equal(e.get("gimbal-mode").value, "ned");
  assert.equal(e.get("gimbal-yaw").value, 45);
  assert.equal(e.get("gimbal-pitch").value, 35);
  assert.match(e.get("telemetry").textContent, /雲台實際.*30\.0/);
  e.get("gimbal-yaw").value = "12";
  update({ context: { ...context, worldTick: 4 } });
  assert.equal(e.get("gimbal-yaw").value, "12");
});
