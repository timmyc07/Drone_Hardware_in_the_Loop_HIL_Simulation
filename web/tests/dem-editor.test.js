import test from "node:test";
import assert from "node:assert/strict";
import { createDemEditor } from "../dem-editor.js";

class Element {
  constructor(value = "") {
    this.value = value;
    this.disabled = false;
    this.textContent = "";
    this.files = [];
    this.listeners = {};
  }
  addEventListener(name, fn) {
    this.listeners[name] = fn;
  }
}
class FakeWorker {
  static last;
  constructor() {
    FakeWorker.last = this;
  }
  postMessage(message) {
    this.message = message;
  }
  terminate() {
    this.terminated = true;
  }
}

test("DEM preview cancels stale work, enforces readonly and applies only confirmed preview", async () => {
  const ids = [
    "dem-file",
    "dem-radius",
    "dem-grid",
    "dem-latitude",
    "dem-longitude",
    "dem-band",
    "dem-unit",
    "dem-datum",
    "dem-preview",
    "dem-cancel",
    "dem-apply",
    "dem-status",
  ];
  const elements = Object.fromEntries(ids.map((id) => [id, new Element()]));
  Object.assign(elements["dem-radius"], { value: "250" });
  elements["dem-grid"].value = "65";
  elements["dem-band"].value = "1";
  elements["dem-unit"].value = "auto";
  elements["dem-datum"].value = "auto";
  elements["dem-file"].files = [
    {
      name: "local.tif",
      size: 16,
      arrayBuffer: async () => new ArrayBuffer(16),
    },
  ];
  const previousDocument = globalThis.document,
    previousWorker = globalThis.Worker;
  globalThis.document = { getElementById: (id) => elements[id] };
  globalThis.Worker = FakeWorker;
  let applied;
  try {
    const editor = createDemEditor({
      getReadonly: () => false,
      onApply: async (value) => {
        applied = value;
      },
    });
    await elements["dem-preview"].onclick();
    const old = FakeWorker.last,
      id = old.message.requestId;
    editor.setReadonly(true);
    assert.equal(old.terminated, true);
    old.onmessage({ data: { type: "result", requestId: id, result: {} } });
    assert.equal(elements["dem-apply"].disabled, true);
    editor.setReadonly(false);
    await elements["dem-preview"].onclick();
    const worker = FakeWorker.last;
    const preview = {
      terrain: {
        kind: "dem",
        north: -1,
        east: -1,
        rows: 2,
        cols: 2,
        cellSize: 2,
        heights: [0, 1, -1, 0],
      },
      geography: {
        origin: {
          latitudeDeg: 25,
          longitudeDeg: 121,
          altitudeM: 10,
          verticalDatum: "unspecified",
        },
        source: {
          name: "local.tif",
          crs: "EPSG:4326",
          width: 10,
          height: 10,
          band: 1,
          heightUnit: "m",
        },
      },
    };
    worker.onmessage({
      data: {
        type: "result",
        requestId: worker.message.requestId,
        result: preview,
      },
    });
    assert.equal(elements["dem-apply"].disabled, false);
    assert.match(elements["dem-status"].textContent, /local\.tif/);
    await elements["dem-apply"].onclick();
    assert.deepEqual(applied, preview);
    elements["dem-radius"].value = "200";
    elements["dem-radius"].listeners.input();
    assert.equal(elements["dem-apply"].disabled, true);
    applied = undefined;
    await elements["dem-apply"].onclick();
    assert.equal(applied, undefined);
    elements["dem-file"].files = [
      { name: "large.tif", size: 64 * 1024 * 1024 + 1 },
    ];
    await elements["dem-preview"].onclick();
    assert.match(elements["dem-status"].textContent, /64 MiB/);
  } finally {
    globalThis.document = previousDocument;
    globalThis.Worker = previousWorker;
  }
});
