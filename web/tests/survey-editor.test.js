import test from "node:test";
import assert from "node:assert/strict";
import { createSurveyEditor } from "../survey-editor.js";

class Element {
  constructor() {
    this.children = [];
    this.value = "";
    this.checked = false;
    this.disabled = false;
    this.textContent = "";
  }
  append(...children) {
    this.children.push(...children);
  }
  replaceChildren(...children) {
    this.children = children;
  }
}

test("survey preview protects mission draft, readonly and per-vehicle state", async () => {
  const ids = [
    "tool", "spacing", "angle", "altitude", "reverse", "add", "example",
    "preview", "use", "clear", "vertices", "boundary", "status",
    "camera-enabled", "plane-altitude", "front-overlap", "side-overlap",
  ];
  const elements = Object.fromEntries(ids.map((id) => ["survey-" + id, new Element()]));
  const previous = globalThis.document;
  globalThis.document = {
    getElementById: (id) => elements[id],
    createElement: () => new Element(),
  };
  const context = {
    vehicleId: "a", readonly: false, blocked: false, home: [0, 0],
    radius: 100, minAltitude: 0, maxAltitude: 100,
  };
  const used = [];
  try {
    const editor = createSurveyEditor({
      getContext: () => context,
      onChange() {},
      onUse: (points) => used.push(points),
    });
    elements["survey-add"].onclick();
    assert.equal(elements["survey-boundary"].open, true);
    const row = elements["survey-vertices"].children[0];
    const input = row.children[0].children[0];
    assert.equal(editor.refresh(), false);
    assert.equal(elements["survey-vertices"].children[0], row);
    input.value = "2";
    input.oninput();
    assert.equal(editor.extent(), 2);
    input.value = "501";
    input.oninput();
    assert.equal(editor.extent(), 0);
    input.value = "";
    input.oninput();
    assert.equal(editor.extent(), 0);
    editor.refresh();
    assert.equal(elements["survey-vertices"].children[0].children[0].children[0], input);
    context.blocked = true;
    assert.equal(editor.refresh(), false);
    assert.equal(input.disabled, true);
    context.blocked = false;
    editor.refresh();
    assert.equal(input.disabled, false);
    for (let i = 1; i < 21; i++) elements["survey-add"].onclick();
    assert.equal(elements["survey-vertices"].children.length, 20);
    assert.match(elements["survey-status"].textContent, /20/);
    elements["survey-example"].onclick();
    elements["survey-preview"].onclick();
    assert.equal(elements["survey-use"].disabled, false);
    await elements["survey-use"].onclick();
    assert.equal(used.length, 1);
    elements["survey-spacing"].value = "0";
    elements["survey-spacing"].oninput();
    await elements["survey-use"].onclick();
    assert.equal(used.length, 1);
    elements["survey-preview"].onclick();
    assert.equal(elements["survey-use"].disabled, true);
    elements["survey-spacing"].value = "6";
    elements["survey-spacing"].oninput();
    elements["survey-preview"].onclick();
    context.readonly = true;
    editor.refresh();
    assert.equal(editor.extent(), 0);
    assert.match(elements["survey-status"].textContent, /重播唯讀/);
    await elements["survey-use"].onclick();
    assert.equal(used.length, 1);
    context.readonly = false;
    context.vehicleId = "b";
    editor.refresh();
    assert.equal(elements["survey-vertices"].children.length, 0);
    context.vehicleId = "a";
    editor.refresh();
    assert.equal(elements["survey-vertices"].children.length, 4);
    context.radius = 1;
    context.minAltitude = 3;
    context.maxAltitude = 20;
    editor.refresh();
    assert.equal(elements["survey-altitude"].min, 3);
    assert.equal(elements["survey-altitude"].max, 20);
    await elements["survey-use"].onclick();
    assert.equal(used.length, 1);
    elements["survey-preview"].onclick();
    assert.equal(elements["survey-use"].disabled, true);
    context.radius = 100;
    context.blocked = true;
    editor.refresh();
    elements["survey-example"].onclick();
    assert.equal(elements["survey-vertices"].children.length, 4);
  } finally {
    globalThis.document = previous;
  }
});

test("camera preview invalidates on settings and camera changes and protects asynchronous apply", async () => {
  const ids = ["tool", "spacing", "angle", "altitude", "reverse", "add", "example", "preview", "use", "clear", "vertices", "boundary", "status", "camera-enabled", "plane-altitude", "front-overlap", "side-overlap"];
  const elements = Object.fromEntries(ids.map((id) => ["survey-" + id, new Element()]));
  const previous = globalThis.document;
  globalThis.document = { getElementById: (id) => elements[id], createElement: () => new Element() };
  const context = { vehicleId: "a", readonly: false, blocked: false, cameraAvailable: true, cameraBlocked: false, cameraConfig: {}, home: [0, 0], radius: 100, minAltitude: -100, maxAltitude: 100 };
  let rejectApply;
  let calls = 0;
  try {
    const editor = createSurveyEditor({ getContext: () => context, onChange() {}, onUse: () => { calls++; return new Promise((resolve, reject) => { rejectApply = reject; }); } });
    elements["survey-example"].onclick();
    elements["survey-preview"].onclick();
    context.cameraConfig = { horizontalFovDeg: 85 };
    editor.refresh();
    assert.equal(elements["survey-use"].disabled, false);
    elements["survey-camera-enabled"].checked = true;
    elements["survey-camera-enabled"].oninput();
    assert.equal(elements["survey-spacing"].disabled, true);
    for (const plane of ["-2", "2"]) {
      elements["survey-plane-altitude"].value = plane;
      elements["survey-plane-altitude"].oninput();
      elements["survey-preview"].onclick();
      assert.equal(elements["survey-use"].disabled, false);
    }
    elements["survey-altitude"].value = "10";
    elements["survey-altitude"].oninput();
    elements["survey-angle"].value = "37";
    elements["survey-angle"].oninput();
    elements["survey-plane-altitude"].value = "0";
    elements["survey-plane-altitude"].oninput();
    elements["survey-preview"].onclick();
    assert.ok(editor.extent() > 20, "地圖範圍須包含超出區域的相機足跡");
    context.cameraBlocked = true;
    editor.refresh();
    assert.equal(elements["survey-use"].disabled, true);
    context.cameraBlocked = false;
    editor.refresh();
    assert.equal(elements["survey-use"].disabled, false);
    context.cameraConfig = { horizontalFovDeg: 80 };
    await elements["survey-use"].onclick();
    assert.equal(calls, 0);
    editor.refresh();
    assert.equal(elements["survey-use"].disabled, true);
    elements["survey-preview"].onclick();
    const pending = elements["survey-use"].onclick();
    assert.equal(elements["survey-preview"].disabled, true);
    await elements["survey-use"].onclick();
    assert.equal(calls, 1);
    rejectApply(new Error("套用失敗"));
    await pending;
    assert.match(elements["survey-status"].textContent, /套用失敗/);
    assert.equal(elements["survey-use"].disabled, false);
    context.cameraAvailable = false;
    editor.refresh();
    assert.equal(elements["survey-use"].disabled, true);
    assert.match(elements["survey-status"].textContent, /僅內建/);
    context.vehicleId = "b";
    editor.refresh();
    context.vehicleId = "a";
    editor.refresh();
    assert.equal(elements["survey-camera-enabled"].checked, true);
    assert.equal(elements["survey-plane-altitude"].value, "0");
  } finally {
    globalThis.document = previous;
  }
});


test("pending apply cannot replace the status of another vehicle or replay", async () => {
  const ids = ["tool", "spacing", "angle", "altitude", "reverse", "add", "example", "preview", "use", "clear", "vertices", "boundary", "status", "camera-enabled", "plane-altitude", "front-overlap", "side-overlap"];
  const elements = Object.fromEntries(ids.map((id) => ["survey-" + id, new Element()]));
  const previous = globalThis.document;
  globalThis.document = { getElementById: (id) => elements[id], createElement: () => new Element() };
  const context = { vehicleId: "a", readonly: false, blocked: false, cameraAvailable: true, cameraBlocked: false, cameraConfig: {}, home: [0, 0], radius: 100, minAltitude: 0, maxAltitude: 100 };
  let settle;
  try {
    const editor = createSurveyEditor({ getContext: () => context, onChange() {}, onUse: () => new Promise((resolve, reject) => { settle = { resolve, reject }; }) });
    elements["survey-example"].onclick();
    elements["survey-preview"].onclick();
    const first = elements["survey-use"].onclick();
    context.vehicleId = "b";
    editor.refresh();
    const vehicleStatus = elements["survey-status"].textContent;
    settle.resolve();
    await first;
    assert.equal(elements["survey-status"].textContent, vehicleStatus);
    context.vehicleId = "a";
    editor.refresh();
    const second = elements["survey-use"].onclick();
    context.readonly = true;
    editor.refresh();
    settle.reject(new Error("舊草稿失敗"));
    await second;
    assert.match(elements["survey-status"].textContent, /重播唯讀/);
  } finally {
    globalThis.document = previous;
  }
});
