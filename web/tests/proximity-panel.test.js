import test from "node:test";
import assert from "node:assert/strict";
import { createProximityPanel } from "../proximity-panel.js";

function setup(exportData = () => {}) {
  class Element {
    textContent = "";
    className = "";
    disabled = false;
    addEventListener(_, fn) { this.click = fn; }
  }
  const names = ["meta", "error", "export-json", "export-csv", ...Array.from({ length: 6 }, (_, i) => [`range-${i}`, `state-${i}`]).flat()];
  const elements = Object.fromEntries(names.map((name) => [name, new Element()]));
  globalThis.document = { getElementById: (id) => elements[id.replace(/^proximity-/, "")] };
  return { elements, panel: createProximityPanel({ exportData }) };
}
const reading = () => ({ time: 2, valid: true, reason: "", positionNed: [0, 0, 0], quaternion: [1, 0, 0, 0], minM: 0.2, maxM: 20, warningM: 2, rangesM: [null, 1, 4, null, null, null], tooClose: [true, false, false, false, false, false] });
const context = { available: true, sourceVehicleId: "alpha", worldTick: 4, time: 2.1, maxAgeS: 0.5 };

test("shows too close, warnings, measured ranges and gaps with timing", () => {
  const { elements: e, panel } = setup();
  panel.render(reading(), context);
  assert.match(e["state-0"].textContent, /太近/);
  assert.equal(e["state-0"].className, "proximity-danger");
  assert.match(e["state-1"].textContent, /警示/);
  assert.equal(e["range-2"].textContent, "4.00");
  assert.equal(e["state-3"].textContent, "無回波");
  assert.match(e.meta.textContent, /alpha.*2\.000.*0\.100.*2\.00/);
});

test("clears stale, fault, absent and external data without declaring safety", () => {
  const { elements: e, panel } = setup();
  const noReturn = { ...reading(), valid: false, reason: "no-return", rangesM: Array(6).fill(null), tooClose: Array(6).fill(false) };
  panel.render(noReturn, context);
  assert.equal(e["state-0"].textContent, "無回波");
  panel.render(reading(), { ...context, time: 3 });
  assert.match(e["state-0"].textContent, /過期/);
  assert.equal(e["range-1"].textContent, "—");
  panel.render({ ...noReturn, reason: "fault", positionNed: null, quaternion: null }, context);
  assert.match(e["state-1"].textContent, /資訊不足/);
  assert.equal(e["export-json"].disabled, false);
  panel.render(reading(), { ...context, available: false });
  assert.equal(e["state-0"].textContent, "未提供");
  assert.equal(e["export-json"].disabled, true);
  panel.render(null, context);
  assert.match(e.meta.textContent, /未提供/);
  panel.render({}, context);
  assert.match(e.meta.textContent, /格式錯誤/);
  assert.equal(e["export-json"].disabled, true);
  assert.equal(e["export-csv"].disabled, true);
});

test("export buttons invoke requested format and report failures", async () => {
  const calls = [];
  const { elements: e, panel } = setup((format) => { calls.push(format); if (format === "csv") throw Error("failed"); });
  panel.render(reading(), context);
  await e["export-json"].click();
  await e["export-csv"].click();
  assert.deepEqual(calls, ["json", "csv"]);
  assert.match(e.error.textContent, /匯出近接資料失敗/);
  panel.render(reading(), context);
  assert.match(e.error.textContent, /匯出近接資料失敗/);
  panel.render(reading(), { ...context, sourceVehicleId: "beta" });
  assert.equal(e.error.textContent, "");
});
