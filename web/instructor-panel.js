import { SENSOR_NAMES } from "./sensors.js";
import { INSTRUCTOR_MAX_EVENTS, validateInstructorPlan, instructorPlanToJson, parseInstructorPlan, instructorReportToJson } from "./instructor.js";

export function createInstructorPanel({ apply, cancel, download, getContext }) {
  const el = (id) => document.getElementById(`instructor-${id}`);
  const rows = el("rows"), message = el("message"), form = el("form");
  let status = { phase: "idle", plan: null, appliedAtTick: null, nextIndex: 0, receipts: [], reason: "" };
  let preview = null, pending = false, generation = 0, contextKey = "", available, lastTick, epoch, renderedStatus;
  const sensorLabels = { gps: "GPS", barometer: "氣壓計", magnetometer: "磁力計", rangefinder: "向下測距", lidar: "光達", opticalFlow: "光流", radar: "雷達", proximity: "六向近接" };
  const types = [["sensorFault", "感測器故障 / 恢復"], ["motor", "馬達效率"], ["link", "通訊連結"], ["pusher", "推進器效率"], ["clearFaults", "清除機體故障"], ["gust", "陣風"], ["wind", "風"]];
  const option = (value, label) => { const o = document.createElement("option"); o.value = value; o.textContent = label; return o; };
  function field(row, title, name, kind, value, choices = []) {
    const label = document.createElement("label"); label.textContent = title;
    const input = document.createElement(kind === "select" ? "select" : "input");
    input.dataset.field = name;
    if (kind === "select") choices.forEach(([v, t]) => input.append(option(String(v), t)));
    else { input.type = kind; if (kind === "number") input.step = "any"; }
    input.value = String(value ?? ""); label.append(input); row.append(label); return input;
  }
  function addRow(event = {}) {
    if (rows.children.length >= INSTRUCTOR_MAX_EVENTS) return;
    const row = document.createElement("div"); row.className = "instructor-row";
    field(row, "世界時間（秒）", "time", "number", event.tick != null ? (event.tick / 200).toFixed(3) : ((getContext().worldTick + 200) / 200).toFixed(3)).step = "0.005";
    const type = event.type === "fault" ? event.fault : event.type || "sensorFault";
    field(row, "事件", "type", "select", type, types);
    const target = document.createElement("span"); target.className = "instructor-target"; row.append(target);
    const params = document.createElement("span"); params.className = "instructor-params"; row.append(params);
    const remove = document.createElement("button"); remove.type = "button"; remove.textContent = "刪除"; remove.addEventListener("click", () => { row.remove(); edited(); }); row.append(remove);
    rows.append(row);
    renderDetails(row, event);
  }
  function renderDetails(row, event = {}) {
    const type = row.querySelector('[data-field="type"]').value, target = row.querySelector(".instructor-target"), params = row.querySelector(".instructor-params");
    target.replaceChildren(); params.replaceChildren();
    if (type !== "wind") {
      const vehicles = getContext().vehicles || [], selected = event.vehicleId ?? getContext().activeVehicleId;
      const choices = vehicles.map((v) => [v.id, v.name || v.id]);
      if (selected != null && !choices.some(([id]) => String(id) === String(selected))) choices.push([selected, `${selected}（不可用）`]);
      field(target, "載具", "vehicleId", "select", selected, choices);
    }
    if (type === "sensorFault") {
      field(params, "感測器", "sensor", "select", event.value?.sensor || "gps", SENSOR_NAMES.map((s) => [s, sensorLabels[s]]));
      field(params, "狀態", "failed", "select", String(event.value?.failed ?? true), [["true", "故障"], ["false", "恢復"]]);
    } else if (type === "motor") {
      field(params, "馬達編號（從 0 起）", "index", "number", event.value?.index ?? 0);
      field(params, "效率", "efficiency", "number", event.value?.efficiency ?? 0);
    } else if (type === "pusher") field(params, "效率", "efficiency", "number", event.value?.efficiency ?? 0);
    else if (type === "link") field(params, "連結", "lost", "select", String(event.value?.lost ?? true), [["true", "中斷"], ["false", "恢復"]]);
    else if (type === "wind") {
      field(params, "風速", "speed", "number", event.value?.speed ?? 0);
      field(params, "方向（度）", "direction", "number", event.value?.direction ?? 0);
    }
  }
  function readPlan() {
    const events = [...rows.children].map((row) => {
      const value = (key) => row.querySelector(`[data-field="${key}"]`)?.value;
      const seconds = Number(value("time")), tick = Math.round(seconds * 200);
      if (!value("time") || !Number.isFinite(seconds) || Math.abs(seconds * 200 - tick) > 1e-7) throw Error("時間必須對齊 0.005 秒。");
      const type = value("type"), event = { tick };
      for (const key of ["index", "efficiency", "speed", "direction"]) {
        const raw = value(key);
        if (raw !== undefined && (raw.trim() === "" || !Number.isFinite(Number(raw)))) throw Error("請填入有效的數值。");
      }
      if (type !== "wind") event.vehicleId = value("vehicleId");
      if (type === "sensorFault") Object.assign(event, { type, value: { sensor: value("sensor"), failed: value("failed") === "true" } });
      else if (["motor", "link", "pusher"].includes(type)) {
        event.type = "fault"; event.fault = type;
        event.value = type === "link" ? { lost: value("lost") === "true" } : type === "motor" ? { index: Number(value("index")), efficiency: Number(value("efficiency")) } : { efficiency: Number(value("efficiency")) };
      } else if (type === "wind") Object.assign(event, { type, value: { speed: Number(value("speed")), direction: Number(value("direction")) } });
      else event.type = type;
      return event;
    });
    return { name: el("name").value, events };
  }
  function edited() { preview = null; message.textContent = ""; controls(); }
  function controls() {
    const ctx = getContext(), armed = status.phase === "armed";
    form.disabled = pending || armed;
    el("add").disabled = rows.children.length >= INSTRUCTOR_MAX_EVENTS;
    el("apply").disabled = pending || armed || !ctx.available || ctx.canApply === false || !preview || preview.events[0].tick <= ctx.worldTick;
    el("cancel").disabled = pending || !ctx.available || !armed;
    el("report").disabled = pending || !ctx.available;
  }
  function error(e) { message.textContent = e?.message || String(e); }
  function validated() { const plan = readPlan(); validateInstructorPlan(plan, getContext().vehicles); return plan; }
  rows.addEventListener("change", (e) => { if (e.target.dataset.field === "type") renderDetails(e.target.closest(".instructor-row")); edited(); });
  rows.addEventListener("input", edited); el("name").addEventListener("input", edited);
  el("add").onclick = () => { addRow(); edited(); };
  el("example").onclick = () => { const ctx = getContext(); rows.replaceChildren(); addRow({ tick: ctx.worldTick + 2000, vehicleId: ctx.activeVehicleId, type: "sensorFault", value: { sensor: "gps", failed: true } }); addRow({ tick: ctx.worldTick + 3000, vehicleId: ctx.activeVehicleId, type: "sensorFault", value: { sensor: "gps", failed: false } }); edited(); };
  el("sort").onclick = () => { [...rows.children].map((row, index) => ({ row, index, time: Number(row.querySelector('[data-field="time"]').value) })).sort((a, b) => a.time - b.time || a.index - b.index).forEach(({ row }) => rows.append(row)); edited(); };
  el("preview").onclick = () => { try { preview = validated(); if (preview.events[0].tick <= getContext().worldTick) throw Error("事件時間已過，請調整後再套用。"); message.textContent = `驗證通過：${preview.events.length} 個事件。`; } catch (e) { preview = null; error(e); } controls(); };
  el("export").onclick = () => { try { download(instructorPlanToJson(validated(), getContext().vehicles), "instructor-plan.json"); } catch (e) { error(e); } };
  el("report").onclick = () => { try { download(instructorReportToJson(status), "instructor-report.json"); } catch (e) { error(e); } };
  el("import").onclick = () => el("file").click();
  el("file").onchange = async () => {
    const file = el("file").files[0]; el("file").value = ""; if (!file) return;
    const token = ++generation; pending = true; controls();
    try {
      if (file.size > 65536) throw Error("計畫檔案不可超過 64 KiB。");
      const text = await file.text(); if (token !== generation) return;
      const plan = parseInstructorPlan(text, getContext().vehicles);
      rows.replaceChildren(); el("name").value = plan.name; plan.events.forEach(addRow); preview = null; message.textContent = "計畫已匯入，請重新驗證。";
    } catch (e) { if (token === generation) error(e); }
    finally { if (token === generation) { pending = false; controls(); } }
  };
  async function operation(fn, argument) {
    const token = ++generation; pending = true; message.textContent = ""; controls();
    try { const result = await fn(argument); if (token === generation) { status = result; preview = null; renderStatus(); } }
    catch (e) { if (token === generation) error(e); }
    finally { if (token === generation) { pending = false; controls(); } }
  }
  el("apply").onclick = () => { if (!el("apply").disabled) operation(apply, preview); };
  el("cancel").onclick = () => { if (!el("cancel").disabled) operation(cancel); };
  function eventSummary(event) {
    if (event.type === "sensorFault") return `${sensorLabels[event.value.sensor]}${event.value.failed ? "故障" : "恢復"}`;
    if (event.type === "fault") {
      if (event.fault === "motor") return `馬達 M${event.value.index + 1} 效率 ${event.value.efficiency * 100}%`;
      if (event.fault === "pusher") return `推進器效率 ${event.value.efficiency * 100}%`;
      if (event.fault === "link") return `連結${event.value.lost ? "中斷" : "恢復"}`;
    }
    if (event.type === "wind") return `風速 ${event.value.speed} m/s，方向 ${event.value.direction}°`;
    return types.find(([id]) => id === event.type)?.[1] || "事件";
  }
  function renderStatus() {
    const ctx = getContext();
    el("availability").textContent = ctx.available ? `世界時間 ${(ctx.worldTick / 200).toFixed(3)} 秒${ctx.running ? "" : "（暫停，排程等待模擬時間）"}${ctx.canApply === false && ctx.reason ? ` · ${ctx.reason}` : ""}` : `未提供教官排程：${ctx.reason || "目前模式不支援"}`;
    el("status").textContent = !ctx.available ? "未提供" : ({ idle: "尚無排程", armed: "排程中，等待模擬時間", complete: "已完成", cancelled: "已取消未來事件", error: "執行錯誤" }[status.phase] || status.phase) + (status.reason ? `：${status.reason}` : "");
    const statusKey = JSON.stringify([ctx.available, status]);
    if (statusKey === renderedStatus) return;
    renderedStatus = statusKey;
    const list = el("receipts"); list.replaceChildren();
    if (ctx.available) (status.plan?.events || []).forEach((event, index) => {
      const receipt = (status.receipts || []).find((entry) => entry.index === index);
      const item = document.createElement("li");
      item.textContent = `第 ${index + 1} 項 · ${(event.tick / 200).toFixed(3)} 秒 · ${event.vehicleId || "全域"} · ${eventSummary(event)} · ${receipt ? (receipt.status === "applied" ? "已執行" : `失敗：${receipt.error}`) : status.phase === "cancelled" ? "已取消（未執行）" : status.phase === "error" ? "未執行（排程已停止）" : "尚未執行"}`;
      list.append(item);
    });
  }
  function refresh(nextStatus) {
    const ctx = getContext(), key = JSON.stringify(ctx.vehicles || []);
    if ((available !== undefined && available !== ctx.available) || (lastTick !== undefined && ctx.worldTick < lastTick) || (contextKey && key !== contextKey) || (epoch !== undefined && epoch !== ctx.epoch)) { generation++; pending = false; preview = null; }
    lastTick = ctx.worldTick; epoch = ctx.epoch;
    available = ctx.available;
    if (key !== contextKey) {
      contextKey = key; preview = null;
      for (const row of rows.children) { const select = row.querySelector('[data-field="vehicleId"]'); if (!select) continue; const selected = select.value; select.replaceChildren(); (ctx.vehicles || []).forEach((v) => select.append(option(String(v.id), v.name || v.id))); if (selected && ![...select.options].some((o) => o.value === selected)) select.append(option(selected, `${selected}（不可用）`)); select.value = selected; }
    }
    if (nextStatus && !pending) status = nextStatus;
    renderStatus(); controls();
  }
  addRow(); refresh(status);
  return { refresh };
}
