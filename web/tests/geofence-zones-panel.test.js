import test from 'node:test';
import assert from 'node:assert/strict';
import { createGeofenceZonesPanel, zonePoints } from '../geofence-zones-panel.js';

class Element {
  constructor() { this.children = []; this.value = ''; this.checked = false; this.disabled = false; this.textContent = ''; }
  append(...children) { this.children.push(...children); }
  replaceChildren(...children) { this.children = children; }
  add(option) { this.children.push(option); }
}

test('zone drafts survive vehicle changes and reject invalid input before applying', async () => {
  const elements = Object.fromEntries(['enabled', 'rows', 'status', 'add-circle', 'add-polygon', 'apply'].map(id => ['zones-' + id, new Element()]));
  const oldDocument = globalThis.document, oldOption = globalThis.Option;
  globalThis.document = { getElementById: id => elements[id], createElement: () => new Element() };
  globalThis.Option = class { constructor(text, value) { this.text = text; this.value = value; } };
  const calls = [];
  try {
    const panel = createGeofenceZonesPanel({ apply: async (...args) => calls.push(args) });
    const initial = { enabled: false, zones: [] };
    panel.open('a', initial);
    elements['zones-add-circle'].onclick();
    const row = elements['zones-rows'].children[0];
    row.children[2].children[0].value = '';
    row.children[2].children[0].oninput();
    await elements['zones-apply'].onclick();
    assert.equal(calls.length, 0);
    assert.match(elements['zones-status'].textContent, /無法套用/);
    panel.open('b', initial);
    assert.equal(elements['zones-rows'].children.length, 0);
    panel.open('a', initial);
    assert.equal(elements['zones-rows'].children[0].children[2].children[0].value, '');
    const input = elements['zones-rows'].children[0].children[2].children[0];
    input.value = '0'; input.oninput();
    elements['zones-enabled'].checked = true; elements['zones-enabled'].onchange();
    await elements['zones-apply'].onclick();
    assert.equal(calls[0][0], 'a');
    assert.deepEqual(calls[0][1].zones[0].center, [0, 0]);
    assert.match(elements['zones-status'].textContent, /已套用/);
    assert.equal(zonePoints(calls[0][1].zones[0]).length, 64);
    elements['zones-add-polygon'].onclick();
    const polygon = elements['zones-rows'].children[1];
    polygon.children[2].children[0].value = '0,0\n0,10\n10,0';
    polygon.children[2].children[0].oninput();
    await elements['zones-apply'].onclick();
    assert.deepEqual(calls[1][1].zones[1].vertices, [[0, 0], [0, 10], [10, 0]]);
    elements['zones-rows'].children[0].children.at(-1).onclick();
    assert.equal(elements['zones-rows'].children.length, 1);
  } finally { globalThis.document = oldDocument; globalThis.Option = oldOption; }
});


test('clean drafts follow external changes, dirty drafts survive, and removed IDs are cleared', () => {
  const elements = Object.fromEntries(['enabled', 'rows', 'status', 'add-circle', 'add-polygon', 'apply'].map(id => ['zones-' + id, new Element()]));
  const oldDocument = globalThis.document, oldOption = globalThis.Option;
  globalThis.document = { getElementById: id => elements[id], createElement: () => new Element() };
  globalThis.Option = class { constructor(text, value) { this.text = text; this.value = value; } };
  try {
    const panel = createGeofenceZonesPanel({ apply: async () => {} });
    const settings = radiusM => ({ enabled: true, zones: [{ id: 'zone-1', shape: 'circle', kind: 'inclusion', center: [0, 0], radiusM }] });
    panel.open('a', settings(10));
    panel.open('a', settings(20));
    const radius = () => elements['zones-rows'].children[0].children[4].children[0];
    assert.equal(radius().value, '20');
    radius().value = '30'; radius().oninput();
    panel.open('a', settings(40));
    assert.equal(radius().value, '30');
    assert.match(elements['zones-status'].textContent, /未套用/);
    panel.retain([]);
    panel.open('a', settings(50));
    assert.equal(radius().value, '50');
    assert.match(elements['zones-status'].textContent, /一致/);
  } finally { globalThis.document = oldDocument; globalThis.Option = oldOption; }
});

test('replay shows recorded settings read only without replacing live drafts', async () => {
  const elements = Object.fromEntries(['enabled', 'rows', 'status', 'add-circle', 'add-polygon', 'apply'].map(id => ['zones-' + id, new Element()]));
  const oldDocument = globalThis.document, oldOption = globalThis.Option;
  globalThis.document = { getElementById: id => elements[id], createElement: () => new Element() };
  globalThis.Option = class { constructor(text, value) { this.text = text; this.value = value; } };
  let calls = 0;
  try {
    const panel = createGeofenceZonesPanel({ apply: async () => calls++ });
    const settings = radiusM => ({ enabled: true, zones: [{ id: 'zone-1', shape: 'circle', kind: 'inclusion', center: [0, 0], radiusM }] });
    panel.open('a', settings(10));
    let row = elements['zones-rows'].children[0];
    row.children[4].children[0].value = '30'; row.children[4].children[0].oninput();
    panel.open('a', settings(20), true);
    row = elements['zones-rows'].children[0];
    assert.equal(row.children[4].children[0].value, '20');
    assert.ok(row.children.every(child => child.disabled || child.children[0]?.disabled));
    for (const name of ['enabled', 'add-circle', 'add-polygon', 'apply']) assert.equal(elements['zones-' + name].disabled, true);
    assert.match(elements['zones-status'].textContent, /重播.*唯讀/);
    elements['zones-add-circle'].onclick();
    row.children.at(-1).onclick();
    await elements['zones-apply'].onclick();
    assert.equal(calls, 0);
    assert.equal(elements['zones-rows'].children.length, 1);
    panel.open('a', settings(10));
    assert.equal(elements['zones-rows'].children[0].children[4].children[0].value, '30');
    assert.match(elements['zones-status'].textContent, /未套用/);
  } finally { globalThis.document = oldDocument; globalThis.Option = oldOption; }
});
