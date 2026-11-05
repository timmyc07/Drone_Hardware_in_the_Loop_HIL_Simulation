import { cameraPreview, cameraDatasetToJson } from "./camera.js";
import { GIMBAL_LIMITS, DEFAULT_GIMBAL_SETTINGS } from "./gimbal.js";
import { createOperatorClient } from "./operator-channel.js";

const axes = ["yaw", "pitch", "roll"];
const degrees = (radians) => radians * 180 / Math.PI;
const fieldDegrees = (radians) => Number(degrees(radians).toFixed(3));
const angleText = (angles) => axes.map((axis) => degrees(angles[axis]).toFixed(1)).join(" / ");
const positionText = (position) => Array.isArray(position) ? position.map((n) => Number(n).toFixed(2)).join(" / ") : "—";
const labels = { connecting: "正在連線", connected: "已連線", busy: "已有另一位操作員使用連線", stale: "連線逾時，資訊可能過期", closed: "連線已關閉" };

export function initOperator(doc = document, win = window, createClient = createOperatorClient) {
  const $ = (id) => doc.getElementById(id);
  let client = null, state = null, shownFrame = null, shownBinding = null, initializedBinding = null, shownResult = null, localError = "", timer = null, bindingRequested = false;
  const session = new URLSearchParams(win.location.hash.slice(1)).get("session");
  const validSession = /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i.test(session || "");
  for (const axis of axes) {
    const field = $(`gimbal-${axis}`);
    [field.min, field.max] = GIMBAL_LIMITS[axis].map((value) => degrees(value));
    field.value = fieldDegrees(DEFAULT_GIMBAL_SETTINGS.target[axis]);
  }
  function draw(frame, type = $("view-select").value, canvas = $("frame-canvas")) {
    canvas.width = frame.width; canvas.height = frame.height;
    canvas.getContext("2d").putImageData(new ImageData(cameraPreview(frame, type), frame.width, frame.height), 0, 0);
  }
  function render() {
    const s = state || { connection: "closed" }, context = s.context, vehicles = context?.vehicles || [];
    $("connection-status").textContent = labels[s.connection] || "連線狀態未知";
    $("error-status").textContent = localError || s.error || (context && !context.available ? context.reason : "") || "";
    const select = $("vehicle-select"), selected = select.value;
    const signature = JSON.stringify(vehicles.map(({ id, name }) => [id, name]));
    if (select.dataset.vehicles !== signature) {
      select.replaceChildren();
      if (!vehicles.length) select.add(new Option("沒有可用載具", ""));
      for (const vehicle of vehicles) select.add(new Option(`${vehicle.name} (${vehicle.id})`, vehicle.id));
      if (vehicles.some((vehicle) => vehicle.id === selected)) select.value = selected;
      else if (vehicles.some((vehicle) => vehicle.id === s.vehicleId)) select.value = s.vehicleId;
      select.dataset.vehicles = signature;
    }
    const ready = s.connection === "connected" && context?.available;
    const bound = ready && !!s.vehicleId && !!s.bindingId;
    select.disabled = !ready || !vehicles.length;
    $("bind-button").disabled = !ready || !select.value;
    $("disconnect-button").disabled = !client || s.connection === "closed";
    for (const id of ["gimbal-mode", "gimbal-yaw", "gimbal-pitch", "gimbal-roll", "gimbal-apply"]) $(id).disabled = !bound || !!s.pendingGimbal;
    $("capture-button").disabled = !bound || !!s.pendingFrame;
    $("auto-capture").disabled = !bound;
    const vehicle = vehicles.find((item) => item.id === s.vehicleId);
    $("telemetry").textContent = vehicle ? `最新遙測 · ${vehicle.name} (${vehicle.id}) · ${vehicle.airframe} · ${vehicle.status} · NED ${positionText(vehicle.positionNed)} m${vehicle.gimbal?.actual && axes.every((axis) => Number.isFinite(vehicle.gimbal.actual[axis])) ? ` · 雲台實際 yaw / pitch / roll ${angleText(vehicle.gimbal.actual)}°` : ""} · tick ${context.worldTick} · 模擬時間 ${Number(context.time).toFixed(3)} s · ${context.running ? "運行中" : "暫停"}${s.connection === "stale" ? " · 已過期" : ""}` : "尚未綁定載具";
    if (s.bindingId && shownBinding !== s.bindingId) { shownBinding = s.bindingId; initializedBinding = null; shownFrame = null; shownResult = null; }
    const currentGimbal = vehicle?.gimbal;
    if (!bindingRequested && s.bindingId && initializedBinding !== s.bindingId && ["body", "ned"].includes(currentGimbal?.mode) && axes.every((axis) => Number.isFinite(currentGimbal.target?.[axis]))) {
      initializedBinding = s.bindingId;
      $("gimbal-mode").value = currentGimbal.mode;
      for (const axis of axes) $(`gimbal-${axis}`).value = fieldDegrees(currentGimbal.target[axis]);
    }
    if (s.frame && s.frame.sourceVehicleId === s.vehicleId) shownFrame = s.frame;
    if (s.lastResult && s.lastResult !== shownResult) {
      shownResult = s.lastResult;
      const gimbal = shownResult.gimbal;
      $("gimbal-mode").value = gimbal.mode;
      for (const axis of axes) $(`gimbal-${axis}`).value = fieldDegrees(gimbal.target[axis]);
    }
    $("apply-result").textContent = s.pendingGimbal ? "等待主模擬確認雲台目標…" : shownResult ? `已套用於 tick ${shownResult.appliedWorldTick} · ${shownResult.gimbal.mode.toUpperCase()} · 目標 yaw / pitch / roll ${angleText(shownResult.gimbal.target)}°${context?.running === false ? " · 模擬暫停，實際姿態需時間推進後才會改變" : ""}` : "尚未套用目標";
    const frame = shownFrame;
    $("frame-canvas").classList.toggle("visible", !!frame);
    $("frame-placeholder").hidden = !!frame;
    $("export-json").disabled = $("export-png").disabled = !frame;
    $("frame-status").textContent = `${s.pendingFrame ? "正在拍攝…" : frame ? "已顯示影像" : "等待影像"}${frame && (["stale", "closed"].includes(s.connection) || !s.bindingId || !s.vehicleId || frame.sourceVehicleId !== s.vehicleId) ? " · 影像已過期，等待重新綁定載具" : ""}`;
    if (frame) {
      try { draw(frame); } catch (error) { localError = `無法顯示影像：${error.message}`; $("error-status").textContent = localError; }
      $("exposure-metadata").textContent = `曝光 tick ${frame.worldTick} · 模擬時間 ${frame.time.toFixed(3)} s · 載具 ${frame.sourceVehicleId} · NED ${positionText(frame.pose.positionNed)} m · 雲台實際 yaw / pitch / roll ${angleText(frame.gimbal.actual)}° · ${frame.width} × ${frame.height} · capture ${frame.captureId}`;
    } else $("exposure-metadata").textContent = "尚無曝光資料";
  }
  function invoke(action) {
    localError = "";
    try { if (!action()) localError = "目前無法送出，請確認連線及載具綁定狀態。"; }
    catch (error) { localError = error.message; }
    render();
  }
  $("bind-button").addEventListener("click", () => { bindingRequested = true; invoke(() => client.bind($("vehicle-select").value)); bindingRequested = false; });
  $("gimbal-form").addEventListener("submit", (event) => {
    event.preventDefault();
    const target = Object.fromEntries(axes.map((axis) => [axis, Number($(`gimbal-${axis}`).value) * Math.PI / 180]));
    if (axes.some((axis) => !Number.isFinite(target[axis]) || target[axis] < GIMBAL_LIMITS[axis][0] || target[axis] > GIMBAL_LIMITS[axis][1])) { localError = "雲台角度超出允許範圍。"; render(); return; }
    invoke(() => client.setGimbal({ mode: $("gimbal-mode").value, target }));
  });
  $("capture-button").addEventListener("click", () => invoke(() => client.requestFrame()));
  $("view-select").addEventListener("change", render);
  function download(blob, name) { const url = win.URL.createObjectURL(blob), link = doc.createElement("a"); link.href = url; link.download = name; link.click(); win.setTimeout(() => win.URL.revokeObjectURL(url), 0); }
  $("export-json").addEventListener("click", () => { try { download(new Blob([cameraDatasetToJson([shownFrame])], { type: "application/json" }), "camera-frame.json"); } catch (error) { localError = error.message; render(); } });
  $("export-png").addEventListener("click", () => { try { const canvas = doc.createElement("canvas"); draw(shownFrame, "rgb", canvas); canvas.toBlob((blob) => { if (blob) download(blob, "camera-rgb.png"); else { localError = "無法匯出 PNG。"; render(); } }, "image/png"); } catch (error) { localError = error.message; render(); } });
  function close() { if (timer !== null) { win.clearInterval(timer); timer = null; } client?.close(); }
  $("disconnect-button").addEventListener("click", () => { if (timer !== null) { win.clearInterval(timer); timer = null; } client?.disconnect(); });
  win.addEventListener("pagehide", close, { once: true });
  if (!validSession) { localError = "連線網址缺少有效的 session，請從主模擬重新開啟操作員頁面。"; render(); return { close }; }
  try {
    client = createClient({ sessionId: session, onChange(next) { state = next; render(); } });
    state = client.getState();
    timer = win.setInterval(() => { if ($("auto-capture").checked && !doc.hidden && state?.connection === "connected" && state.context?.available && state.bindingId && state.vehicleId && !state.pendingFrame) client.requestFrame(); }, 1000);
  } catch (error) { localError = `無法初始化連線：${error.message}`; }
  render();
  return { close };
}
if (typeof document !== "undefined") initOperator();
