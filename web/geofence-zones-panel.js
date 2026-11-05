import { validateGeofenceZones } from './geofence-zones.js';

export function zonePoints(zone) {
  if (zone.shape === 'polygon') return zone.vertices;
  return Array.from({ length: 64 }, (_, i) => {
    const angle = i * Math.PI * 2 / 64;
    return [zone.center[0] + Math.cos(angle) * zone.radiusM, zone.center[1] + Math.sin(angle) * zone.radiusM];
  });
}

export function createGeofenceZonesPanel({ apply }) {
  const $ = (id) => document.getElementById(`zones-${id}`);
  const drafts = new Map();
  let id, busy = false, readonlyEntry;
  const entry = () => readonlyEntry || drafts.get(id);
  const changed = (entry) => JSON.stringify(entry.draft) !== JSON.stringify(entry.base);
  function status(message) { $('status').textContent = readonlyEntry ? '重播中的區域設定為唯讀。' : message || (changed(entry()) ? '有未套用的區域草稿。' : '已與目前設定一致。'); }
  function render() {
    const draft = entry().draft;
    const disabled = busy || !!readonlyEntry;
    $('enabled').checked = draft.enabled; $('enabled').disabled = disabled;
    $('rows').replaceChildren();
    draft.zones.forEach((zone, index) => {
      const row = document.createElement('div'); row.className = 'zone-row';
      const field = (label, value, handler, tag = 'input') => {
        const wrapper = document.createElement('label'); wrapper.textContent = label;
        const input = document.createElement(tag); input.value = value; input.disabled = disabled;
        input.oninput = () => { if (readonlyEntry || busy) return; handler(input.value); status(); };
        wrapper.append(input); row.append(wrapper); return input;
      };
      field('ID', zone.id, value => zone.id = value);
      const kind = field('類型', zone.kind, value => zone.kind = value, 'select');
      for (const [value, text] of [['inclusion', '允許區'], ['exclusion', '禁入區']]) kind.add(new Option(text, value));
      kind.value = zone.kind;
      if (zone.shape === 'circle') {
        field('中心北 (m)', zone.center[0], value => zone.center[0] = value);
        field('中心東 (m)', zone.center[1], value => zone.center[1] = value);
        field('半徑 (m)', zone.radiusM, value => zone.radiusM = value);
      } else field('頂點：每行北,東 (m)', zone.vertices, value => zone.vertices = value, 'textarea');
      const remove = document.createElement('button'); remove.type = 'button'; remove.textContent = '移除';
      remove.disabled = disabled;
      remove.onclick = () => { if (readonlyEntry || busy) return; draft.zones.splice(index, 1); render(); };
      row.append(remove); $('rows').append(row);
    });
    $('add-circle').disabled = $('add-polygon').disabled = draft.zones.length >= 20 || disabled;
    $('apply').disabled = disabled;
    status();
  }
  function raw(settings) {
    return { enabled: settings.enabled, zones: settings.zones.map(z => z.shape === 'circle'
      ? { ...z, center: z.center.map(String), radiusM: String(z.radiusM) }
      : { ...z, vertices: z.vertices.map(p => p.join(',')).join('\n') }) };
  }
  function value() {
    const number = (s) => s.trim() === '' ? NaN : Number(s);
    return validateGeofenceZones({ enabled: drafts.get(id).draft.enabled, zones: drafts.get(id).draft.zones.map(z => z.shape === 'circle'
      ? { ...z, center: z.center.map(number), radiusM: number(z.radiusM) }
      : { ...z, vertices: z.vertices.split('\n').map(line => line.split(',').map(number)) }) });
  }
  $('enabled').onchange = () => { if (readonlyEntry || busy) return; drafts.get(id).draft.enabled = $('enabled').checked; status(); };
  for (const shape of ['circle', 'polygon']) $('add-' + shape).onclick = () => {
    if (readonlyEntry || busy) return;
    const zones = drafts.get(id).draft.zones;
    if (zones.length >= 20) return;
    let n = 1; while (zones.some(z => z.id === `zone-${n}`)) n++;
    zones.push(shape === 'circle' ? { id: `zone-${n}`, kind: 'inclusion', shape, center: ['0', '0'], radiusM: '10' }
      : { id: `zone-${n}`, kind: 'inclusion', shape, vertices: '0,0\n0,10\n10,0' });
    render();
  };
  $('apply').onclick = async () => {
    if (busy || readonlyEntry) return;
    let message;
    try {
      const settings = value(), submittedId = id, submitted = drafts.get(id); busy = true; render();
      await apply(submittedId, settings);
      const saved = raw(settings);
      if (drafts.get(submittedId) === submitted) drafts.set(submittedId, { base: saved, draft: structuredClone(saved) });
      if (id === submittedId && drafts.has(id)) message = '區域圍欄已套用。';
    } catch (error) { message = `無法套用：${error.message}`; }
    finally { busy = false; if (entry()) { render(); if (message) status(message); } }
  };
  return {
    open(vehicleId, settings, readonly = false) {
      id = vehicleId;
      const base = raw(settings);
      if (readonly) { readonlyEntry = { base, draft: structuredClone(base) }; render(); return; }
      readonlyEntry = undefined;
      const existing = drafts.get(id);
      const draft = existing && changed(existing) ? existing.draft : structuredClone(base);
      drafts.set(id, { base, draft });
      render();
    },
    retain(vehicleIds) {
      const ids = new Set(vehicleIds);
      for (const vehicleId of drafts.keys()) if (!ids.has(vehicleId)) drafts.delete(vehicleId);
    },
  };
}
