const phaseLabels = {
  idle: "尚未開始自動拍攝",
  waiting: "等待拍攝條件",
  paused: "模擬已暫停，等待繼續",
  capturing: "正在拍攝",
  complete: "自動拍攝已完成",
  stopped: "自動拍攝已停止",
  error: "自動拍攝發生錯誤",
};

export function createAutomaticCameraControls({ start, stop, getContext }) {
  const element = (name) => document.getElementById(`camera-auto-${name}`);
  const mode = element("mode");
  const interval = element("interval");
  const limit = element("limit");
  const startButton = element("start");
  const stopButton = element("stop");
  const label = element("interval-label");
  const message = element("status");
  let current = { active: false, pending: false, phase: "idle" };
  let feedback = "";

  function updateMode() {
    const distance = mode.value === "distance";
    label.textContent = distance ? "間隔（公尺）" : "間隔（秒）";
    interval.min = distance ? "0.25" : "0.5";
    interval.max = distance ? "500" : "300";
  }

  function refresh(status) {
    current = status;
    if (status.active) feedback = "";
    const context = getContext();
    const locked = status.active || status.pending;
    mode.disabled = interval.disabled = limit.disabled = locked;
    startButton.disabled = locked || !context.available || context.busy;
    stopButton.disabled = !status.active;
    const parts = [phaseLabels[status.phase] || phaseLabels.idle];
    if (status.vehicleId) parts.push(`載具：${status.vehicleId}`);
    if (status.settings) parts.push(`已保存 ${status.saved} / ${status.settings.maxFrames} 張`);
    if (status.last) parts.push(`${status.last.time.toFixed(3)} 秒 · 世界步 ${status.last.worldTick}`);
    if (status.reason) parts.push(status.reason);
    if (!context.available && context.reason) parts.push(context.reason);
    else if (context.busy && !locked) parts.push(context.reason || "相機忙碌中");
    message.textContent = feedback || parts.join(" · ");
  }

  function clearFeedback() {
    feedback = "";
    refresh(current);
  }
  mode.addEventListener("change", () => {
    updateMode();
    clearFeedback();
  });
  interval.addEventListener("input", clearFeedback);
  limit.addEventListener("input", clearFeedback);
  startButton.addEventListener("click", () => {
    const value = interval.value.trim();
    const count = limit.value.trim();
    const intervalValue = value ? Number(value) : NaN;
    const maxFrames = count ? Number(count) : NaN;
    const distance = mode.value === "distance";
    if (
      !Number.isFinite(intervalValue) ||
      intervalValue < (distance ? 0.25 : 0.5) ||
      intervalValue > (distance ? 500 : 300) ||
      !Number.isInteger(maxFrames) || maxFrames < 1 || maxFrames > 30
    ) {
      feedback = "設定無效：請輸入範圍內的間隔與 1 至 30 的整數張數。";
      message.textContent = feedback;
      return;
    }
    try {
      start({ mode: mode.value, interval: intervalValue, maxFrames });
      feedback = "";
      refresh(current);
    } catch (error) {
      feedback = `無法開始自動拍攝：${error.message || error}`;
      message.textContent = feedback;
    }
  });
  stopButton.addEventListener("click", () => stop());
  updateMode();
  refresh(current);
  return { refresh };
}
