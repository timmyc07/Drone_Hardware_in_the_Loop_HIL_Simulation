import test from 'node:test';
import assert from 'node:assert/strict';
import { createNistPanel } from '../nist-panel.js';

class Element {
  constructor(tag = '') { this.tag = tag; this.children = []; this.value = ''; this.textContent = ''; this.disabled = false; }
  append(child) { this.children.push(child); if (this.tag === 'select' && this.children.length === 1) this.value = child.value; }
  replaceChildren(...children) { this.children = []; children.forEach(child => this.append(child)); }
}
function setup(overrides = {}) {
  const ids = 'form availability prepare duration custom start end report export clear files step instruction slots landing evidence status message'.split(' ');
  const e = Object.fromEntries(ids.map(id => [id, new Element(id === 'step' ? 'select' : '')])); e.duration.value = '300'; e.custom.value = '5';
  globalThis.document = { getElementById: id => e[id.slice(5)], createElement: tag => new Element(tag) };
  const ctx = { available: true, laneReady: true, vehicleId: 'v1', worldTick: 0, epoch: 0, landed: true, canStart: true, canCapture: true, canScore: true };
  const idle = { phase: 'idle', captures: [], scores: {}, landings: [], totalScore: 0, elapsedSeconds: 0 };
  const callbacks = Object.fromEntries('prepareLane start capture score landing end report preview exportDataset clearDataset verifyDatasets'.split(' ').map(name => [name, async () => idle]));
  const panel = createNistPanel({ ...callbacks, ...overrides, getContext: () => ctx });
  return { e, ctx, idle, panel };
}
test('manual scoring requires capture; ended session allows scoring only', () => {
  const { e, panel, idle } = setup();
  panel.refresh({ ...idle, phase: 'running' });
  assert.match(e.status.textContent, /人工判讀/);
  assert.equal(e.slots.children[0].children[3].disabled, true);
  panel.refresh({ ...idle, phase: 'ended', captures: [{ slotId: 's01-1', captureId: 'c1' }] });
  assert.equal(e.slots.children[0].children[1].disabled, true);
  assert.equal(e.slots.children[0].children[3].disabled, false);
});
test('custom duration accepts whole seconds only within range', () => {
  let started; const { e } = setup({ start: async value => { started = value; } });
  e.duration.value = 'custom'; e.custom.value = '0.1'; e.start.onclick(); assert.match(e.message.textContent, /30 至 1800/); assert.equal(started, undefined);
  e.custom.value = '0.501'; e.start.onclick(); assert.equal(started, undefined);
  e.custom.value = '0.5'; e.start.onclick(); assert.deepEqual(started, { durationSeconds: 30 });
});
test('epoch change ignores late operation results', async () => {
  let resolve; const { e, ctx, panel, idle } = setup({ start: () => new Promise(r => { resolve = r; }) });
  e.start.onclick(); ctx.epoch++; panel.refresh(idle); resolve({ ...idle, phase: 'running' });
  await new Promise(r => setImmediate(r)); assert.match(e.status.textContent, /尚未開始/); assert.equal(e.start.disabled, false);
});

test('clock refresh preserves buttons and terminal context permits only scoring', () => {
  const { e, ctx, panel, idle } = setup();
  const state = { ...idle, phase: 'running', captures: [{ slotId: 's01-1', captureId: 'c1' }] };
  panel.refresh(state); const button = e.slots.children[0].children[3];
  panel.refresh({ ...state, elapsedSeconds: 1 });
  assert.equal(e.slots.children[0].children[3], button);
  assert.match(e.status.textContent, /1 \/ /);
  Object.assign(ctx, { canStart: false, canCapture: false, canScore: true, reason: '時間到' });
  panel.refresh({ ...state, phase: 'ended' });
  assert.equal(e.start.disabled, true); assert.equal(e.slots.children[0].children[1].disabled, true);
  assert.equal(e.slots.children[0].children[3].disabled, false); assert.match(e.availability.textContent, /時間到/);
  ctx.canScore = false; panel.refresh({ ...state, phase: 'ended' });
  assert.equal(e.slots.children[0].children[3].disabled, true);
});

test('dataset verification displays its success message', async () => {
  const { e } = setup({ verifyDatasets: async () => ({ message: '已核對 2 份資料集：3/4 張影像符合報告。' }) });
  e.files.files = [{ name: 'a.json' }, { name: 'b.json' }]; e.files.onchange();
  await new Promise(r => setImmediate(r));
  assert.equal(e.message.textContent, '已核對 2 份資料集：3/4 張影像符合報告。');
});
