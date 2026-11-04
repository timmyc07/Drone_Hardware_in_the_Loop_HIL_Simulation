import { DEFAULT_CONFIG, validateConfig } from "./sim.js";
import { createScene } from "./scene.js";
import { validateRecording, recordingToCsv } from "./recording.js";

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;
const storageKey = "flight-lab-config-v1";
let config = { ...DEFAULT_CONFIG };
try {
  const saved = localStorage.getItem(storageKey);
  if (saved) config = validateConfig(JSON.parse(saved));
} catch {
  /* Invalid or unavailable browser storage uses model defaults. */
}
let scenario = "hover";
let running = false;
let latest;
let history = [];
let replay;
let replayClock = 0;
let replayIndex = 0;
let rendering = true;
let scene;
let toastTimer;
let sampleCount = 0;
let timing = {};
let firstState = true;
const worker = new Worker(new URL("./worker.js", import.meta.url), {
  type: "module",
});

function toast(message) {
  $("toast").textContent = message;
  $("toast").hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => {
    $("toast").hidden = true;
  }, 5000);
}

function setPlaying(value) {
  running = value;
  $("play").innerHTML = value
    ? '<span aria-hidden="true">Ⅱ</span> 暫停模擬'
    : `<span aria-hidden="true">▶</span> ${replay ? "播放重播" : latest?.time > 0 ? "繼續模擬" : "開始模擬"}`;
  $("runtime-status").textContent = replay
    ? "紀錄重播"
    : value
      ? "運行中"
      : latest?.time > 0
        ? "已暫停"
        : "待命";
  $("run-label").textContent = replay
    ? "播放已記錄的狀態"
    : value
      ? "正在記錄實驗數據"
      : latest?.time > 0
        ? "實驗暫停，資料已保留"
        : "準備就緒，開始你的實驗";
  const label =
    latest?.status === "failsafe"
      ? "斷訊保護"
      : replay
        ? "紀錄重播"
        : value
          ? "模擬運行中"
          : latest?.time > 0
            ? "已暫停"
            : "準備就緒";
  $("view-status").innerHTML = `<span class="dot"></span>${label}`;
}

const targets = [
  {
    key: "altitude",
    label: "目標高度",
    min: 0,
    max: 20,
    step: 0.1,
    value: 2,
    unit: "m",
    factor: 1,
  },
  {
    key: "roll",
    label: "滾轉 Roll",
    min: -20,
    max: 20,
    step: 1,
    value: 0,
    unit: "°",
    factor: DEG,
  },
  {
    key: "pitch",
    label: "俯仰 Pitch",
    min: -20,
    max: 20,
    step: 1,
    value: 0,
    unit: "°",
    factor: DEG,
  },
  {
    key: "yawRate",
    label: "偏航角速度",
    min: -60,
    max: 60,
    step: 1,
    value: 0,
    unit: "°/s",
    factor: DEG,
  },
];
$("target-controls").innerHTML = targets
  .map(
    (t) =>
      `<div class="target-control"><label for="target-${t.key}">${t.label}<output id="output-${t.key}" for="target-${t.key}">${t.value.toFixed(1)} <span>${t.unit}</span></output></label><input id="target-${t.key}" type="range" min="${t.min}" max="${t.max}" step="${t.step}" value="${t.value}" /><div class="range-ends"><span>${t.min} ${t.unit}</span><span>${t.max} ${t.unit}</span></div></div>`,
  )
  .join("");
function resetTargets() {
  for (const t of targets) {
    $(`target-${t.key}`).value = t.value;
    $(`output-${t.key}`).textContent = `${t.value.toFixed(1)} ${t.unit}`;
  }
}
for (const t of targets) {
  $(`target-${t.key}`).addEventListener("input", (event) => {
    const value = Number(event.target.value);
    $(`output-${t.key}`).textContent = `${value.toFixed(1)} ${t.unit}`;
    worker.postMessage({
      type: "targets",
      value: { [t.key]: value / t.factor },
    });
  });
}

$("motor-bars").innerHTML = [1, 2, 3, 4]
  .map(
    (i) =>
      `<div class="motor-row"><span class="motor-name">M${i}</span><div class="motor-track"><div id="motor-${i}" class="motor-fill"></div></div><span class="motor-reading"><span id="pwm-${i}">—</span> <small>µs</small></span></div>`,
  )
  .join("");

const groups = [
  [
    "機體與馬達",
    [
      ["mass", "機體質量 (kg)", 0.1, 20, 0.1, 1],
      ["arm", "臂長 (m)", 0.05, 2, 0.01, 1],
      ["maxMotorThrust", "單馬達最大推力 (N)", 0.1, 200, 0.1, 1],
      ["motorTau", "馬達時間常數 (ms)", 10, 1000, 1, 1000],
      ["drag", "線性阻力 (N·s/m)", 0, 10, 0.05, 1],
    ],
  ],
  [
    "控制器",
    [
      ["attitudeKp", "姿態比例增益 (s⁻¹)", 0, 20, 0.1, 1],
      ["rateKp", "角速度比例增益", 0, 2, 0.01, 1],
      ["rateKi", "角速度積分增益", 0, 1, 0.01, 1],
      ["altitudeKp", "高度比例增益 (s⁻²)", 0, 20, 0.1, 1],
      ["altitudeKd", "高度微分增益 (s⁻¹)", 0, 20, 0.1, 1],
    ],
  ],
  [
    "環境與感測故障",
    [
      ["wind", "向北常風 (m/s)", 0, 30, 0.1, 1],
      ["noise", "每取樣雜訊標準差倍率", 0, 10, 0.1, 1],
      ["latency", "感測延遲 (ms)", 0, 2000, 5, 1000],
      ["dropout", "感測丟包率 (%)", 0, 100, 1, 100],
      ["seed", "隨機種子", 0, 4294967295, 1, 1],
    ],
  ],
];
const fields = groups.flatMap(([, list]) => list);
$("settings-fields").innerHTML = groups
  .map(
    ([name, list]) =>
      `<section class="settings-group"><h3>${name}</h3><div class="settings-grid">${list.map(([key, label, min, max, step]) => `<div class="settings-field"><label for="config-${key}">${label}</label><input id="config-${key}" name="${key}" type="number" min="${min}" max="${max}" step="${step}" required /></div>`).join("")}</div></section>`,
  )
  .join("");
function populateConfig(values) {
  for (const [key, , , , , factor] of fields)
    $(`config-${key}`).value = Number((values[key] * factor).toFixed(8));
  $("settings-error").textContent = "";
}
$("settings-open").onclick = () => {
  populateConfig(config);
  $("settings-dialog").showModal();
};
$("settings-default").onclick = () => populateConfig(DEFAULT_CONFIG);
$("settings-form").onsubmit = (event) => {
  event.preventDefault();
  try {
    const next = {};
    for (const [key, , , , , factor] of fields)
      next[key] = Number($(`config-${key}`).value) / factor;
    config = validateConfig(next);
    try {
      localStorage.setItem(storageKey, JSON.stringify(config));
    } catch {
      toast("此瀏覽器無法儲存設定，本次實驗仍會使用新參數。");
    }
    $("settings-dialog").close();
    newExperiment();
    toast("參數已套用，已建立新實驗。");
  } catch (error) {
    $("settings-error").textContent = `請檢查參數：${error.message}`;
  }
};
$("help").onclick = () => $("help-dialog").showModal();
document.querySelectorAll("[data-close]").forEach((button) => {
  button.onclick = () => $(button.dataset.close).close();
});

function newExperiment() {
  replay = undefined;
  document.body.classList.remove("replay");
  history = [];
  sampleCount = 0;
  resetTargets();
  document
    .querySelectorAll("#target-controls input, #gust")
    .forEach((element) => {
      element.disabled = false;
    });
  $("target-roll").disabled = scenario === "step";
  document.querySelector(".control-hint").textContent =
    scenario === "step"
      ? "階躍情境自動控制 Roll；其餘指令可即時調整。"
      : "指令即時生效；更換情境會建立新實驗。";
  worker.postMessage({ type: "reset", config, scenario });
  setPlaying(false);
  $("reset").setAttribute("aria-label", "重設實驗");
}
$("reset").onclick = () => {
  if (
    latest?.time > 0 &&
    !confirm("重設會清除目前尚未匯出的實驗紀錄。確定重設？")
  )
    return;
  newExperiment();
  scene?.resetView();
};
document.querySelectorAll("[data-scenario]").forEach((button) => {
  button.onclick = () => {
    if (
      latest?.time > 0 &&
      !confirm("切換情境會清除目前尚未匯出的實驗紀錄。確定切換？")
    )
      return;
    scenario = button.dataset.scenario;
    document.querySelectorAll("[data-scenario]").forEach((item) => {
      const active = item === button;
      item.classList.toggle("active", active);
      item.setAttribute("aria-pressed", String(active));
    });
    newExperiment();
  };
});
$("play").onclick = () => {
  if (replay) {
    if (replayIndex >= replay.samples.length - 1) {
      replayClock = replay.samples[0].time;
      replayIndex = 0;
      history = [];
    }
    setPlaying(!running);
  } else worker.postMessage({ type: running ? "pause" : "play" });
};
$("speed").onchange = (event) =>
  worker.postMessage({ type: "speed", value: Number(event.target.value) });
$("gust").onclick = () => {
  worker.postMessage({ type: "gust" });
  toast("已加入一秒陣風，模擬運行時生效。");
};
document.querySelectorAll("[data-camera]").forEach((button) => {
  button.onclick = () => {
    scene?.setCamera(button.dataset.camera);
    document.querySelectorAll("[data-camera]").forEach((item) => {
      const active = item === button;
      item.classList.toggle("active", active);
      item.setAttribute("aria-pressed", String(active));
    });
  };
});
$("render-toggle").onclick = () => {
  rendering = !rendering;
  scene?.setVisible(rendering);
  $("render-message").hidden = rendering;
  $("render-toggle").textContent = rendering ? "關閉渲染" : "開啟渲染";
  $("render-toggle").setAttribute("aria-pressed", String(rendering));
};

function download(text, extension, mime) {
  const url = URL.createObjectURL(new Blob([text], { type: mime }));
  const a = document.createElement("a");
  a.href = url;
  a.download = `flight-lab-${new Date().toISOString().replace(/[:.]/g, "-")}.${extension}`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
function exportRecording(kind, recording) {
  if (kind === "csv")
    download(
      "\uFEFF" + recordingToCsv(recording),
      "csv",
      "text/csv;charset=utf-8",
    );
  else download(JSON.stringify(recording), "json", "application/json");
  toast("實驗紀錄已匯出。");
}
for (const kind of ["csv", "json"])
  $(`export-${kind}`).onclick = () => {
    document.querySelector(".export-menu").open = false;
    if (replay) exportRecording(kind, replay);
    else worker.postMessage({ type: "export", kind });
  };
$("import-open").onclick = () => {
  document.querySelector(".export-menu").open = false;
  $("import-file").click();
};
$("import-file").onchange = async (event) => {
  const file = event.target.files?.[0];
  event.target.value = "";
  if (!file) return;
  if (file.size > 64 * 1024 * 1024) {
    toast("紀錄檔案超過 64 MB 上限。");
    return;
  }
  try {
    const record = validateRecording(JSON.parse(await file.text()));
    if (
      latest?.time > 0 &&
      !confirm("匯入將取代目前畫面。尚未匯出的紀錄請先匯出。繼續匯入？")
    )
      return;
    worker.postMessage({ type: "pause" });
    replay = record;
    config = { ...record.config };
    scenario = record.scenario;
    document.querySelectorAll("[data-scenario]").forEach((item) => {
      const active = item.dataset.scenario === scenario;
      item.classList.toggle("active", active);
      item.setAttribute("aria-pressed", String(active));
    });
    replayClock = record.samples[0].time;
    replayIndex = 0;
    history = [record.samples[0]];
    latest = record.samples[0];
    sampleCount = record.samples.length;
    timing = {};
    document.body.classList.add("replay");
    document
      .querySelectorAll("#target-controls input, #gust")
      .forEach((element) => {
        element.disabled = true;
      });
    setPlaying(false);
    updateTelemetry();
    toast("紀錄已載入。按「播放重播」查看；按重設返回新實驗。");
  } catch (error) {
    toast(`無法讀取紀錄：${error.message}`);
  }
};

worker.onmessage = ({ data }) => {
  if (data.type === "error") {
    setPlaying(false);
    toast(data.message);
    return;
  }
  if (data.type === "recording") {
    exportRecording(data.kind, data.recording);
    return;
  }
  if (replay) return;
  if (firstState) {
    firstState = false;
    worker.postMessage({ type: "reset", config, scenario });
  }
  latest = data.snapshot;
  timing = data.timing;
  sampleCount = data.sampleCount;
  if (data.message === "reset") history = [];
  history.push(...data.samples);
  while (history.length > 400) history.shift();
  $("play").disabled = false;
  setPlaying(data.running);
  updateTelemetry();
  if (data.message === "limit")
    toast("已完成 30 分鐘實驗並暫停，請匯出紀錄後重設。");
};
worker.onerror = (event) => {
  setPlaying(false);
  $("play").disabled = true;
  toast(`模擬執行緒無法運行：${event.message || "請重新載入頁面"}`);
};

function updateTelemetry() {
  if (!latest) return;
  const s = latest;
  if (replay)
    for (const t of targets) {
      const value = s.target[t.key] * t.factor;
      $(`target-${t.key}`).value = value;
      $(`output-${t.key}`).textContent = `${value.toFixed(1)} ${t.unit}`;
    }
  else if (scenario === "step") {
    $("target-roll").value = s.target.roll * DEG;
    $("output-roll").textContent = `${(s.target.roll * DEG).toFixed(1)} °`;
  }
  const minutes = Math.floor(s.time / 60);
  $("sim-time").textContent =
    `${String(minutes).padStart(2, "0")}:${(s.time % 60).toFixed(2).padStart(5, "0")}`;
  $("altitude-value").textContent = (-s.position[2]).toFixed(2);
  $("altitude-target").textContent = s.target.altitude.toFixed(1);
  $("roll-value").textContent = (s.attitude[0] * DEG).toFixed(2);
  $("pitch-value").textContent = (s.attitude[1] * DEG).toFixed(2);
  $("velocity-value").textContent = Math.hypot(...s.velocity).toFixed(2);
  $("distance-value").textContent = Math.hypot(
    ...s.position.slice(0, 2),
  ).toFixed(2);
  $("rmse-value").textContent = (s.metrics.rmse * DEG).toFixed(2);
  for (let i = 0; i < 4; i++) {
    $(`motor-${i + 1}`).style.width = `${s.motors[i] * 100}%`;
    $(`pwm-${i + 1}`).textContent = Math.round(s.pwm[i]);
  }
  $("rtf-value").textContent = replay
    ? "重播"
    : timing.realTimeFactor
      ? `${timing.realTimeFactor.toFixed(2)}×`
      : "—";
  $("jitter-value").textContent = replay
    ? "—"
    : `${(timing.p95LateMs || 0).toFixed(2)} ms`;
  $("age-value").textContent = `${(s.metrics.sampleAge * 1000).toFixed(0)} ms`;
  $("drop-value").textContent = s.metrics.dropped.toLocaleString();
  $("samples-value").textContent = sampleCount.toLocaleString();
  if (s.status === "failsafe") {
    $("runtime-status").textContent = "斷訊保護";
    $("view-status").innerHTML =
      '<span class="dot"></span>斷訊保護 · 輸出已停用';
  }
  drawChart();
}

const canvas = $("attitude-chart");
function drawChart() {
  const rect = canvas.getBoundingClientRect();
  if (!rect.width) return;
  const ratio = Math.min(devicePixelRatio || 1, 2);
  canvas.width = Math.round(rect.width * ratio);
  canvas.height = Math.round(rect.height * ratio);
  const ctx = canvas.getContext("2d");
  ctx.scale(ratio, ratio);
  const w = rect.width,
    h = rect.height;
  const end = Math.max(15, latest?.time || 0),
    start = end - 15;
  const visible = history.filter((s) => s.time >= start);
  const max = Math.max(
    15,
    ...visible.flatMap((s) => [
      Math.abs(s.attitude[0] * DEG),
      Math.abs(s.attitude[1] * DEG),
      Math.abs(s.target.roll * DEG),
    ]),
  );
  const extent = Math.ceil(max / 5) * 5;
  const x = (t) => 28 + ((t - start) / 15) * (w - 32);
  const y = (angle) => h / 2 - (angle / extent) * (h / 2 - 10);
  ctx.font = "8px monospace";
  ctx.lineWidth = 1;
  for (const value of [-extent, 0, extent]) {
    ctx.strokeStyle = value === 0 ? "#344a60" : "#23364a";
    ctx.beginPath();
    ctx.moveTo(28, y(value));
    ctx.lineTo(w, y(value));
    ctx.stroke();
    ctx.fillStyle = "#7890aa";
    ctx.fillText(String(value), 0, y(value) + 3);
  }
  for (const [color, get, dash] of [
    ["#75e1c6", (s) => s.attitude[0] * DEG, []],
    ["#f3b177", (s) => s.attitude[1] * DEG, []],
    ["#91a5bf", (s) => s.target.roll * DEG, [4, 4]],
  ]) {
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.6;
    ctx.setLineDash(dash);
    ctx.beginPath();
    visible.forEach((s, i) => {
      if (i === 0) ctx.moveTo(x(s.time), y(get(s)));
      else ctx.lineTo(x(s.time), y(get(s)));
    });
    ctx.stroke();
  }
  ctx.setLineDash([]);
  $("chart-start").textContent = `${start.toFixed(0)} s`;
  $("chart-end").textContent = `${end.toFixed(0)} s`;
}
new ResizeObserver(drawChart).observe(canvas);

try {
  scene = createScene($("viewport"));
} catch (error) {
  $("scene-error").hidden = false;
  $("scene-error").textContent =
    `3D 畫面無法啟動：${error.message}。請使用支援 WebGL 2 的瀏覽器；數值模擬與圖表仍可使用。`;
}
let previousFrame = performance.now();
let uiTime = 0;
function animate(now) {
  const elapsed = Math.min((now - previousFrame) / 1000, 0.1);
  previousFrame = now;
  if (replay && running) {
    replayClock += elapsed * Number($("speed").value);
    while (
      replayIndex + 1 < replay.samples.length &&
      replay.samples[replayIndex + 1].time <= replayClock
    ) {
      latest = replay.samples[++replayIndex];
      history.push(latest);
      if (history.length > 400) history.shift();
    }
    if (replayIndex === replay.samples.length - 1) setPlaying(false);
    if (now - uiTime > 50) {
      updateTelemetry();
      uiTime = now;
    }
  }
  if (latest && rendering)
    scene?.update(latest, running ? elapsed * Number($("speed").value) : 0);
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);
document.addEventListener("visibilitychange", () => {
  if (document.hidden && running) {
    if (replay) setPlaying(false);
    else worker.postMessage({ type: "pause" });
    toast("背景分頁已自動暫停。返回後可繼續。");
  }
});
window.addEventListener("beforeunload", (event) => {
  if (!replay && sampleCount > 20) {
    event.preventDefault();
    event.returnValue = "";
  }
});
