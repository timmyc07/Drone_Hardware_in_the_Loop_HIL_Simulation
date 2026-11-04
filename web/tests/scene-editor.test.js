import test from "node:test";
import assert from "node:assert/strict";
import { createSceneEditor } from "../scene-editor.js";

class Element {
  constructor() {
    this.children = [];
    this.value = "";
    this.disabled = false;
    this.textContent = "";
    this.classList = { toggle() {} };
  }
  append(...children) {
    this.children.push(...children);
  }
  replaceChildren(...children) {
    this.children = [...children];
  }
  getContext() {
    return {
      fillRect() {},
      beginPath() {},
      moveTo() {},
      lineTo() {},
      stroke() {},
      arc() {},
      fill() {},
      fillText() {},
    };
  }
}
test("numeric input updates exported draft, retains subsequent edits and supports readonly", async () => {
  const ids = [
    "scene-dirty",
    "terrain-north",
    "terrain-east",
    "terrain-cell",
    "terrain-heights",
    "terrain-error",
    "terrain-flat",
    "terrain-hill",
    "terrain-ridge",
    "terrain-valley",
    "scene-name",
    "scene-course",
    "scene-tool",
    "scene-add-obstacle",
    "scene-add-gate",
    "scene-template",
    "scene-remove",
    "scene-up",
    "scene-down",
    "scene-apply",
    "scene-import",
    "scene-map",
    "scene-map-range",
    "scene-object-list",
    "scene-object-fields",
    "scene-export",
    "scene-import-file",
  ];
  const elements = Object.fromEntries(ids.map((id) => [id, new Element()]));
  elements["scene-course"].options = ["free", "custom"].map((value) => ({
    value,
  }));
  elements["scene-map"].width = 600;
  elements["scene-map"].height = 420;
  elements["scene-map-range"].value = "50";
  const previous = globalThis.document;
  globalThis.document = {
    getElementById: (id) => elements[id],
    createElement: () => new Element(),
  };
  let exported, submitted;
  try {
    const editor = createSceneEditor({
      getAirframe: () => "quadrotor",
      apply: async (value) => {
        submitted = value;
      },
      confirm: async () => true,
      download: (text) => {
        exported = JSON.parse(text).scene;
      },
      notify: () => {},
    });
    elements["terrain-ridge"].onclick();
    elements["scene-export"].onclick();
    assert.equal(Math.max(...exported.terrain.heights), 10);
    elements["terrain-valley"].onclick();
    elements["scene-export"].onclick();
    assert.equal(Math.min(...exported.terrain.heights), -8);
    elements["terrain-flat"].onclick();
    elements["scene-export"].onclick();
    assert.equal(exported.terrain.kind, "flat");
    elements["terrain-hill"].onclick();
    elements["scene-export"].onclick();
    assert.equal(exported.terrain.kind, "heightfield");
    assert.equal(Math.max(...exported.terrain.heights), 12);
    const terrainText = elements["terrain-heights"].value;
    elements["terrain-heights"].value = "invalid";
    elements["terrain-heights"].oninput();
    assert.equal(elements["scene-export"].disabled, true);
    elements["terrain-heights"].value = terrainText;
    elements["terrain-heights"].oninput();
    assert.equal(elements["scene-export"].disabled, false);
    elements["scene-add-obstacle"].onclick();
    let field = elements["scene-object-fields"].children[0].children[0];
    field.value = "10";
    field.oninput();
    assert.equal(elements["scene-dirty"].textContent, "草稿尚未套用");
    elements["scene-export"].onclick();
    assert.equal(exported.obstacles[0].north, 10);
    field.value = "";
    field.oninput();
    assert.equal(elements["scene-export"].disabled, true);
    field.onblur();
    assert.equal(Number(field.value), 10);
    assert.equal(elements["scene-export"].disabled, false);
    elements["scene-add-gate"].onclick();
    const fields = elements["scene-object-fields"].children;
    fields[3].children[0].value = "90";
    fields[3].children[0].oninput();
    fields[4].children[0].value = "4";
    fields[4].children[0].oninput();
    elements["scene-export"].onclick();
    assert.equal(exported.customGates[0].yaw, Math.PI / 2);
    assert.equal(exported.customGates[0].radius, 4);
    await elements["scene-apply"].onclick();
    elements["scene-name"].value = "後續修改";
    elements["scene-name"].oninput({ target: elements["scene-name"] });
    editor.markApplied(submitted, submitted);
    elements["scene-export"].onclick();
    assert.equal(exported.name, "後續修改");
    editor.setScene(submitted, true);
    assert.equal(elements["scene-name"].disabled, true);
    editor.setScene(submitted, false);
    elements["scene-export"].onclick();
    assert.equal(exported.name, "後續修改");
  } finally {
    globalThis.document = previous;
  }
});
