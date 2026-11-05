import { NIST_STEPS } from './nist.js';

export function createNistPanel({ prepareLane, start, capture, score, landing, end, report, preview, exportDataset, clearDataset, verifyDatasets, getContext }) {
  const el = id => document.getElementById(`nist-${id}`);
  let state = { phase: 'idle', captures: [], scores: {}, landings: [], totalScore: 0, elapsedSeconds: 0 }, pending = false, generation = 0, lastContext, renderedKey;
  const phases = { position: '定位', left: '左繞', right: '右繞' };
  const reasons = { manual: '手動結束', 'time-limit': '時間到', 'vehicle-collision': '載具碰撞', 'terrain-coverage': '地形範圍', 'storage-limit': '儲存空間已滿', 'simulation-error': '模擬錯誤' };
  for (const step of NIST_STEPS) {
    const option = document.createElement('option'); option.value = String(step.stepIndex); option.textContent = `第 ${step.number} 步 · ${phases[step.phase]} · ${step.instruction}`; el('step').append(option);
  }
  function button(label, disabled, action) {
    const b = document.createElement('button'); b.type = 'button'; b.textContent = label; b.disabled = disabled; b.onclick = () => { if (!b.disabled) action(); }; return b;
  }
  async function operation(fn, arg) {
    const token = ++generation; pending = true; el('message').textContent = ''; render();
    try { const result = await fn(arg); if (token === generation) { if (result?.phase) state = result; if (result?.message) el('message').textContent = result.message; } }
    catch (error) { if (token === generation) el('message').textContent = error?.message || String(error); }
    finally { if (token === generation) { pending = false; render(); } }
  }
  function render() {
    const ctx = getContext(), active = ctx.available && !pending, running = state.phase === 'running';
    el('form').disabled = pending;
    el('availability').textContent = ctx.available ? `目前載具：${ctx.vehicleName || ctx.vehicleId || '無'} · ${ctx.laneReady ? '場地已載入' : '請載入場地'}${ctx.reason ? ` · ${ctx.reason}` : ''}` : `目前無法操作：${ctx.reason || '此模式不支援'}`;
    el('prepare').disabled = !active;
    el('start').disabled = !active || !ctx.laneReady || state.phase !== 'idle' || !ctx.vehicleId || ctx.canStart === false;
    el('end').disabled = !active || !running;
    el('report').disabled = pending || state.phase === 'idle';
    el('export').disabled = pending;
    el('clear').disabled = !active;
    el('files').disabled = pending;
    el('custom').disabled = pending || el('duration').value !== 'custom';
    el('status').textContent = `狀態：${({idle:'尚未開始',running:'練習中',ended:'已結束'})[state.phase]} · 綁定載具：${state.vehicleId || '無'} · ${state.totalScore || 0}/40 分 · 已拍 ${state.captures.length}/36 · 已判讀 ${Object.keys(state.scores).length}/36 · ${Math.floor(state.elapsedSeconds || 0)} / ${state.durationSeconds || 0} 秒${state.termination ? ` · ${reasons[state.termination] || state.termination}` : ''} · 人工判讀`;
    const evidence = ctx.evidence || {}; el('evidence').textContent = `影像核對：${evidence.verified || 0}/${evidence.required || 0} 匹配 · 保留 ${evidence.retained || 0} · 缺少 ${evidence.missing || 0}`;
    const step = NIST_STEPS[Number(el('step').value) || 0]; el('instruction').textContent = `第 ${step.number} 步：${step.instruction}`;
    const key = JSON.stringify([step.stepIndex, state.phase, state.captures.map(({ slotId, captureId }) => [slotId, captureId]), state.scores, state.landings, pending, ctx.available, ctx.canCapture, ctx.canScore, ctx.landed]);
    if (key === renderedKey) return;
    renderedKey = key;
    const slots = el('slots'); slots.replaceChildren();
    for (const slot of step.slots) {
      const entry = document.createElement('div'); entry.className = 'nist-slot';
      const title = document.createElement('span'); const image = state.captures.find(c => c.slotId === slot.id); const judged = Object.hasOwn(state.scores, slot.id);
      title.textContent = `目標 ${slot.targetId} · ${image ? '已拍攝' : '未拍攝'}${judged ? ` · ${state.scores[slot.id]} 分` : ''}`; entry.append(title);
      entry.append(button('拍攝', !active || !running || !ctx.canCapture || !!image, () => operation(capture, slot.id)));
      entry.append(button('檢視證據', pending || !image, () => operation(preview, image.captureId)));
      for (const value of [1, 0]) entry.append(button(value ? '環完整 1 分' : '不完整 0 分', !active || !ctx.canScore || !image || judged || state.phase === 'idle', () => operation(score, { slotId: slot.id, value })));
      slots.append(entry);
    }
    const target = el('landing'); target.replaceChildren();
    if (step.landingMaxScore) {
      const recorded = state.landings.find(item => item.phase === step.phase);
      const label = document.createElement('p'); label.textContent = recorded ? `已記錄降落 ${recorded.value} 分` : `人工判定降落（最高 ${step.landingMaxScore} 分；至少一腳位於圓內）`; target.append(label);
      for (let value = step.landingMaxScore; value >= 0; value--) target.append(button(`${value} 分`, !active || !running || !ctx.landed || !!recorded, () => operation(landing, { phase: step.phase, value })));
    }
  }
  el('step').onchange = render; el('duration').onchange = render;
  el('prepare').onclick = () => { if (!el('prepare').disabled) operation(prepareLane); };
  el('start').onclick = () => {
    if (el('start').disabled) return;
    const raw = el('duration').value === 'custom' ? Number(el('custom').value) * 60 : Number(el('duration').value);
    if (!Number.isInteger(raw) || raw < 30 || raw > 1800) { el('message').textContent = '時限須為 30 至 1800 秒的整數。'; return; }
    operation(start, { durationSeconds: raw });
  };
  el('end').onclick = () => { if (!el('end').disabled) operation(end); };
  el('report').onclick = () => { if (!el('report').disabled) operation(report); };
  el('export').onclick = () => { if (!el('export').disabled) operation(exportDataset); };
  el('clear').onclick = () => { if (!el('clear').disabled) operation(clearDataset); };
  el('files').onchange = () => { const files = [...el('files').files]; el('files').value = ''; if (files.length && !el('files').disabled) operation(verifyDatasets, files); };
  function refresh(next) {
    const ctx = getContext();
    if (lastContext && (lastContext.epoch !== ctx.epoch || lastContext.available !== ctx.available || ctx.worldTick < lastContext.worldTick)) { generation++; pending = false; }
    lastContext = { epoch: ctx.epoch, available: ctx.available, worldTick: ctx.worldTick };
    if (next && !pending) state = next;
    render();
  }
  refresh(state); return { refresh };
}
