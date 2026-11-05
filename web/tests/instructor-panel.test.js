import test from 'node:test';
import assert from 'node:assert/strict';
import { createInstructorPanel } from '../instructor-panel.js';

class Element {
  constructor(tag = '') { this.tag = tag; this.children = []; this.dataset = {}; this.listeners = {}; this.value = ''; this.textContent = ''; this.disabled = false; }
  append(child) { if (child.parent) child.remove(); this.children.push(child); child.parent = this; if (this.tag === 'select' && this.children.length === 1) this.value = child.value; }
  replaceChildren(...children) { this.children.forEach(c => { c.parent = null; }); this.children = []; children.forEach(c => this.append(c)); }
  remove() { if (this.parent) this.parent.children.splice(this.parent.children.indexOf(this), 1); this.parent = null; }
  addEventListener(type, fn) { (this.listeners[type] ||= []).push(fn); }
  dispatch(type, target = this) { for (const fn of this.listeners[type] || []) fn({target}); if (this.parent) this.parent.dispatch(type, target); }
  querySelector(selector) { const match = selector.match(/^\[data-field="(.+)"\]$/); for (const child of this.children) { if (match ? child.dataset.field === match[1] : selector === `.${child.className}`) return child; const result = child.querySelector(selector); if (result) return result; } return null; }
  get options() { return this.children; }
}
function setup() {
  const ids = 'rows message form name availability add example sort preview import file export apply cancel report status receipts'.split(' ');
  const elements = Object.fromEntries(ids.map(id => [id, new Element()])); elements.name.value = 'Test';
  globalThis.document = { getElementById: id => elements[id.slice('instructor-'.length)], createElement: tag => new Element(tag) };
  const ctx = { available: true, worldTick: 0, running: true, activeVehicleId: 'v1', vehicles: [{ id: 'v1', config: { airframe: 'quad' } }] };
  let applied;
  const idle = {phase:'idle',plan:null,appliedAtTick:null,nextIndex:0,receipts:[],reason:''};
  const panel = createInstructorPanel({ getContext: () => ctx, download: () => {}, apply: async plan => { applied = plan; return {...idle, phase:'armed', plan}; }, cancel: async () => ({...idle, phase:'cancelled'}) });
  return {elements, ctx, panel, applied: () => applied, idle};
}
test('draft validation, scheduling, actual status and cancellation', async () => {
  const {elements: e, ctx, panel, applied, idle} = setup();
  e.example.onclick(); e.preview.onclick(); assert.equal(e.apply.disabled, false);
  await e.apply.onclick(); await new Promise(resolve => setImmediate(resolve));
  assert.equal(applied().events[0].value.failed, true); assert.equal(e.apply.disabled, true); assert.equal(e.cancel.disabled, false);
  assert.match(e.status.textContent, /排程中/); assert.equal(e.receipts.children.length, 2);
  assert.match(e.receipts.children[0].textContent, /GPS故障/); assert.match(e.receipts.children[1].textContent, /GPS恢復/);
  panel.refresh({...idle, phase:'cancelled', plan:applied()}); assert.match(e.receipts.children[0].textContent, /已取消（未執行）/);
  panel.refresh({...idle, phase:'armed', plan:applied()});
  await e.cancel.onclick(); await new Promise(resolve => setImmediate(resolve)); assert.match(e.status.textContent, /取消/);
  e.preview.onclick(); assert.equal(e.apply.disabled, false);
  ctx.worldTick = 3000; panel.refresh(idle); assert.equal(e.apply.disabled, true);
});
test('invalid draft and failed import preserve existing rows', async () => {
  const {elements: e} = setup(); e.example.onclick();
  const time = e.rows.children[0].querySelector('[data-field="time"]'); time.value = '1.001'; time.dispatch('input'); e.preview.onclick();
  assert.equal(e.apply.disabled, true); assert.match(e.message.textContent, /0.005/);
  e.file.files = [{size: 10, text: async () => '{bad'}]; await e.file.onchange();
  assert.equal(e.rows.children.length, 2); assert.equal(time.value, '1.001'); assert.ok(e.message.textContent);
});
test('world reset rejects a late import without replacing the draft', async () => {
  const {elements: e, ctx, panel, idle} = setup();
  ctx.epoch = 0; panel.refresh(idle); e.example.onclick();
  const original = e.rows.children[0];
  let resolveText;
  e.file.files = [{size: 10, text: () => new Promise(resolve => { resolveText = resolve; })}];
  const importing = e.file.onchange();
  ctx.epoch = 1; panel.refresh(idle);
  resolveText('{"format":"flight-lab-instructor-plan","version":1,"plan":{"name":"new","events":[{"tick":200,"type":"wind","value":{"speed":1,"direction":0}}]}}');
  await importing;
  assert.equal(e.rows.children[0], original);
  assert.equal(e.rows.children.length, 2);
  assert.equal(e.name.value, 'Test');
});
