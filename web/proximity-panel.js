import { PROXIMITY_AXES, validateProximityReading } from "./proximity.js";

export function createProximityPanel({ exportData }) {
  const element = (name) => document.getElementById(`proximity-${name}`);
  const meta = element("meta"), error = element("error");
  const buttons = [element("export-json"), element("export-csv")];
  const rows = PROXIMITY_AXES.map((_, i) => ({
    range: element(`range-${i}`), state: element(`state-${i}`),
  }));
  buttons.forEach((button, i) => button.addEventListener("click", async () => {
    try {
      error.textContent = "";
      await exportData(i === 0 ? "json" : "csv");
    } catch {
      error.textContent = "匯出近接資料失敗，請稍後再試。";
    }
  }));
  function clear(state) {
    rows.forEach((row) => {
      row.range.textContent = "—";
      row.state.textContent = state;
      row.state.className = "";
    });
  }
  let lastSource, lastAvailable;
  function render(reading, context = {}) {
    const available = context.available !== false && reading != null;
    buttons.forEach((button) => { button.disabled = !available; });
    if (lastSource !== context.sourceVehicleId || lastAvailable !== available) error.textContent = "";
    lastSource = context.sourceVehicleId;
    lastAvailable = available;
    if (!available) {
      meta.textContent = "未提供近接資料";
      clear("未提供");
      return;
    }
    try {
      reading = validateProximityReading(reading);
    } catch {
      buttons.forEach((button) => { button.disabled = true; });
      meta.textContent = "近接資料格式錯誤";
      clear("資訊不足");
      return;
    }
    const age = Number.isFinite(context.time) ? context.time - reading.time : null;
    const stale = age === null || age < 0 || !Number.isFinite(context.maxAgeS) || age > context.maxAgeS;
    meta.textContent = `來源載具：${context.sourceVehicleId ?? "未提供"} · 測量時刻：${reading.time.toFixed(3)} 秒 · 齡期：${age === null || age < 0 ? "未知" : `${age.toFixed(3)} 秒`} · 警示門檻：${reading.warningM.toFixed(2)} m`;
    if (stale || (!reading.valid && reading.reason !== "no-return")) {
      clear(stale ? "資訊不足（已過期或時間未知）" : `資訊不足（${{ startup: "啟動中", fault: "故障", dropout: "資料丟失" }[reading.reason]}）`);
      return;
    }
    rows.forEach((row, i) => {
      const range = reading.rangesM[i];
      row.range.textContent = range === null ? "—" : range.toFixed(2);
      row.state.className = reading.tooClose[i] ? "proximity-danger" : range !== null && range <= reading.warningM ? "proximity-warning" : "";
      row.state.textContent = reading.tooClose[i] ? "太近（低於最小量測距離）" : range === null ? "無回波" : range <= reading.warningM ? "警示：接近物體" : "已測得距離";
    });
  }
  return { render };
}
