import test from "node:test";
import assert from "node:assert/strict";
import { createReplayControls } from "../replay-controls.js";

function setup() {
  class Element {
    constructor() {
      this.value = "";
      this.textContent = "";
      this.hidden = false;
      this.disabled = false;
      this.options = [];
      this.listeners = {};
    }
    setAttribute(name, value) {
      this[name] = value;
    }
    addEventListener(name, fn) {
      this.listeners[name] = fn;
    }
    append(option) {
      this.options.push(option);
    }
    remove(index) {
      this.options.splice(index, 1);
    }
    dispatch(name) {
      if (!this.disabled) this.listeners[name]();
    }
  }
  const names = [
    "controls",
    "frame",
    "position",
    "seconds",
    "error",
    "event-type",
    "event-vehicle",
    "event-details",
    "first",
    "previous",
    "next",
    "last",
    "event-previous",
    "event-next",
    "seek-time",
  ];
  const elements = Object.fromEntries(
    names.map((name) => [name, new Element()]),
  );
  elements["event-type"].append(new Element());
  elements["event-vehicle"].append(new Element());
  globalThis.document = {
    getElementById: (id) => elements[id.replace(/^replay-/, "")],
    createElement: () => new Element(),
  };
  const seeks = [],
    controls = createReplayControls({ seek: (index) => seeks.push(index) });
  return { elements, seeks, controls };
}
const recording = () => ({
  world: { vehicles: [{ id: "alpha" }, { id: "beta" }] },
  samples: [
    { tick: 0, time: 0 },
    { tick: 10, time: 0.05 },
    { tick: 20, time: 0.1 },
  ],
  events: [
    { tick: 5, time: 0.025, type: "arm", vehicleId: "alpha" },
    { tick: 5, time: 0.025, type: "gust" },
    { tick: 20, time: 0.1, type: "arm", vehicleId: "beta" },
  ],
});

test("frame controls seek, clamp by time, reset and hide", () => {
  const { elements: e, seeks, controls } = setup();
  controls.setRecording(recording());
  assert.equal(e.controls.hidden, false);
  assert.equal(e.first.disabled, true);
  e.next.dispatch("click");
  assert.equal(seeks.at(-1), 1);
  assert.equal(e.frame["aria-valuetext"], "影格 2 / 3，0.050 秒");
  e.last.dispatch("click");
  assert.equal(seeks.at(-1), 2);
  assert.equal(e.next.disabled, true);
  e.frame.value = "0";
  e.frame.dispatch("input");
  assert.equal(seeks.at(-1), 0);
  e.seconds.value = "0.075";
  e["seek-time"].dispatch("click");
  assert.equal(seeks.at(-1), 1);
  e.seconds.value = "";
  e["seek-time"].dispatch("click");
  assert.equal(seeks.at(-1), 1);
  assert.match(e.error.textContent, /有效時間/);
  controls.setRecording(null);
  assert.equal(e.controls.hidden, true);
  controls.setRecording({
    ...recording(),
    samples: [recording().samples[0]],
    events: [],
  });
  assert.equal(e.first.disabled, true);
  assert.equal(e.last.disabled, true);
  assert.equal(e["event-next"].disabled, true);
});

test("same-tick events retain source order, filters and manual seek clear selection", () => {
  const { elements: e, seeks, controls } = setup();
  controls.setRecording(recording());
  e["event-next"].dispatch("click");
  assert.equal(seeks.at(-1), 1);
  assert.match(e["event-details"].textContent, /事件 1 \/ 3：解鎖/);
  e["event-next"].dispatch("click");
  assert.equal(seeks.at(-1), 1);
  assert.match(e["event-details"].textContent, /事件 2 \/ 3：陣風/);
  e["event-previous"].dispatch("click");
  assert.match(e["event-details"].textContent, /事件 1 \/ 3：解鎖/);
  e["event-vehicle"].value = "@world";
  e["event-vehicle"].dispatch("change");
  assert.match(e["event-details"].textContent, /事件：1/);
  e["event-previous"].dispatch("click");
  assert.match(e["event-details"].textContent, /陣風/);
  e.frame.value = "1";
  e.frame.dispatch("input");
  assert.match(e["event-details"].textContent, /尚未選擇/);
  e["event-type"].value = "arm";
  e["event-type"].dispatch("change");
  assert.match(e["event-details"].textContent, /事件：0/);
  assert.equal(e["event-next"].disabled, true);
});

test("inspection details use concise translated fields and clear old filters", () => {
  const { elements: e, controls } = setup();
  const record = recording();
  record.world.vehicles[0].name = "一號機";
  record.world.scene = { inspectionTargets: [{ id: "pier", name: "橋墩" }] };
  record.events = [
    {
      tick: 0,
      time: 0,
      type: "scene",
      value: {
        inspectionTargets: [{ id: "pier", name: "橋墩西側" }],
        terrain: "x".repeat(1_000_000),
      },
    },
    {
      tick: 5,
      time: 0.025,
      type: "inspection",
      vehicleId: "alpha",
      value: { targetId: "pier" },
      receipt: {
        assessment: { passed: false, reasons: ["range", "occluded"] },
      },
    },
  ];
  controls.setRecording(record);
  assert.equal(e["event-type"].options[1].textContent, "巡檢拍攝");
  e["event-type"].value = "inspection";
  e["event-type"].dispatch("change");
  e["event-next"].dispatch("click");
  assert.equal(
    e["event-details"].textContent,
    "事件 1 / 1：巡檢拍攝 · 0.025 秒 · 一號機（alpha） · 目標 橋墩西側（pier） · 未通過：距離不符、目標被遮擋",
  );
  assert.ok(e["event-details"].textContent.length < 120);
  controls.setRecording(null);
  assert.equal(e["event-type"].options.length, 1);
  assert.equal(e["event-vehicle"].options.length, 1);
  assert.equal(e["event-details"].textContent, "");
});
