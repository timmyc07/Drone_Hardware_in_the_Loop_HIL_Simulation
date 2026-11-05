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

test("survey preview protects mission draft, readonly and per-vehicle state", () => {
  const ids = [
    "tool", "spacing", "angle", "altitude", "reverse", "add", "example",
    "preview", "use", "clear", "vertices", "boundary", "status",
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
    elements["survey-use"].onclick();
    assert.equal(used.length, 1);
    elements["survey-spacing"].value = "0";
    elements["survey-spacing"].oninput();
    elements["survey-use"].onclick();
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
    elements["survey-use"].onclick();
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
    elements["survey-use"].onclick();
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
