import test from 'node:test';
import assert from 'node:assert/strict';
import { createFollowPanel, formatFollow } from '../follow-panel.js';
import { DEFAULT_FOLLOW } from '../follow.js';

function setup() {
  class Element {
    constructor(text = '', value = '') { this.textContent = text; this.value = value; this.options = []; this.disabled = false; }
    replaceChildren(...children) { this.options = children; }
    add(option) { this.options.push(option); }
  }
  const elements = Object.fromEntries(['leader','frame','x','y','altitude','distance','lost','yaw','apply','status'].map(k => [k, new Element()]));
  globalThis.Option = Element;
  globalThis.document = { getElementById: id => elements[id.replace('follow-', '')] };
  const applied = [];
  return { elements, applied, panel: createFollowPanel({ apply: async (id, settings) => applied.push([id, settings]) }) };
}
const vehicles = [{ id: 'a', name: '甲', snapshot: { position: [0, 0, -3] } }, { id: 'b', name: '乙', snapshot: { position: [3, 4, -3] } }];
const settings = leaderId => ({ ...structuredClone(DEFAULT_FOLLOW), leaderId });
test('per-vehicle drafts survive switching and refresh, replay is readonly, deleted IDs are cleared', async () => {
  const { elements: e, panel, applied } = setup();
  panel.open('a', settings('b'), vehicles);
  assert.deepEqual(e.leader.options.map(o => o.value), ['', 'b']);
  e.x.value = '-12'; e.x.oninput();
  panel.open('b', settings(null), vehicles);
  assert.equal(e.x.value, '-5');
  panel.open('a', settings('b'), vehicles);
  assert.equal(e.x.value, '-12');
  panel.open('a', { ...settings('b'), maxDistanceM: 80 }, vehicles);
  assert.equal(e.x.value, '-12');
  panel.open('a', { ...settings('b'), offset: { x: 4, y: 0, altitude: 0 } }, vehicles, true);
  assert.equal(e.x.value, '4'); assert.equal(e.apply.disabled, true);
  await e.apply.onclick(); assert.equal(applied.length, 0);
  panel.open('a', settings('b'), vehicles);
  assert.equal(e.x.value, '-12');
  await e.apply.onclick(); assert.equal(applied[0][1].offset.x, -12);
  panel.open('a', { ...settings('b'), offset: { x: 7, y: 0, altitude: 0 } }, vehicles);
  assert.equal(e.x.value, '7');
  panel.retain(['b']);
  panel.open('a', settings('b'), vehicles);
  assert.equal(e.x.value, '-5');
});
test('invalid values cannot be applied and state reports leader, reason, distance and target', async () => {
  const { elements: e, panel, applied } = setup();
  panel.open('a', settings('b'), vehicles);
  e.distance.value = ''; e.distance.oninput(); await e.apply.onclick();
  assert.equal(applied.length, 0); assert.match(e.status.textContent, /無法套用/);
  const text = formatFollow(settings('b'), { active: false, leaderId: 'b', reason: 'leader-landed', targetPosition: [1, 2, -4] }, vehicles[0].snapshot, vehicles);
  assert.match(text, /乙.*領航機已降落.*距離 5\.0 m.*北 1\.0、東 2\.0、高 4\.0/);
});
