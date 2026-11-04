import { DEFAULT_CONFIG, validateConfig } from "./sim.js";
import { createScene } from "./scene.js";
import {
  validateRecording,
  recordingToCsv,
  recordingToJson,
} from "./recording.js";
import { readPilotInput } from "./input.js";
import { createFlightApi } from "./api.js";
import { DEFAULT_SENSOR_CONFIG, validateSensorConfig } from "./sensors.js";
import { TRAINING_COURSES } from "./training.js";
import { CAPABILITIES, SOURCES } from "./capabilities.js";

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;
const storageKey = "flight-lab-config-v1";
let config = { ...DEFAULT_CONFIG };
let sensorConfig = { ...DEFAULT_SENSOR_CONFIG };
let sensors;
let replaySensorIndex = 0;
try {
  const saved = localStorage.getItem(storageKey);
  if (saved) config = validateConfig(JSON.parse(saved));
  const savedSensors = localStorage.getItem("flight-lab-sensors-v1");
  if (savedSensors)
    sensorConfig = validateSensorConfig(JSON.parse(savedSensors));
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
let replayEventIndex = 0;
let rendering = true;
let scene;
let toastTimer;
let sampleCount = 0;
let storageLimited = false;
let exportedTime = 0;
const dirty = () =>
  !replay && sampleCount > 1 && (latest?.time || 0) > exportedTime + 1e-8;
function confirmAction(message) {
  const dialog = $("confirm-dialog");
  $("confirm-message").textContent = message;
  dialog.returnValue = "cancel";
  return new Promise((resolve) => {
    dialog.addEventListener(
      "close",
      () => resolve(dialog.returnValue === "confirm"),
      { once: true },
    );
    dialog.showModal();
  });
}
let timing = {};
let firstState = true;
const worker = new Worker(new URL("./worker.js", import.meta.url), {
  type: "module",
});
let requestId = 0;
const pendingApi = new Map();
const apiSubscribers = new Set();
function requestApi(command) {
  if (replay)
    return Promise.reject(new Error("請先離開重播模式再使用程式介面"));
  const id = ++requestId;
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {
      pendingApi.delete(id);
      reject(new Error("模擬器未在時間內回應"));
    }, 10000);
    pendingApi.set(id, { resolve, reject, timeout });
    try {
      worker.postMessage({ ...command, requestId: id });
    } catch (error) {
      clearTimeout(timeout);
      pendingApi.delete(id);
      reject(error);
    }
  });
}
function onStateSubscribe(callback) {
  apiSubscribers.add(callback);
  return () => apiSubscribers.delete(callback);
}
Object.defineProperty(window, "flightLab", {
  value: createFlightApi(requestApi, onStateSubscribe),
  writable: false,
  configurable: false,
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
    latest?.status === "crashed"
      ? "碰撞停機"
      : latest?.status === "failsafe"
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
groups.push([
  "導航、環境與電池",
  [
    ["windDirection", "風向 (°)", 0, 360, 1, 1],
    ["turbulence", "亂流 (m/s)", 0, 15, 0.1, 1],
    ["geofenceRadius", "圍欄半徑 (m)", 1, 500, 1, 1],
    ["geofenceHeight", "圍欄高度 (m)", 2, 500, 1, 1],
    ["batteryCapacityWh", "電池容量 (Wh)", 0.1, 1000, 0.1, 1],
    ["batteryInitialSoc", "初始電量 (0–1)", 0, 1, 0.01, 1],
    ["batteryBasePower", "基礎功率 (W)", 0, 200, 1, 1],
    ["batteryMotorPower", "馬達功率 (W)", 1, 1000, 1, 1],
    ["batteryInternalResistance", "內阻 (Ω)", 0, 2, 0.01, 1],
    ["batteryCells", "電池串數", 1, 12, 1, 1],
    ["lowBatteryThreshold", "低電量門檻", 0.01, 0.5, 0.01, 1],
    ["criticalBatteryThreshold", "危急電量門檻", 0, 0.3, 0.01, 1],
  ],
]);
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
  populateSensorConfig(sensorConfig);
  $("settings-dialog").showModal();
};
$("settings-default").onclick = () => {
  populateConfig(DEFAULT_CONFIG);
  populateSensorConfig(DEFAULT_SENSOR_CONFIG);
};
$("settings-form").onsubmit = async (event) => {
  event.preventDefault();
  try {
    const next = { ...config };
    for (const [key, , , , , factor] of fields)
      next[key] = Number($(`config-${key}`).value) / factor;
    const validated = validateConfig(next);
    const nextSensors = validateSensorConfig(
      Object.fromEntries(
        Object.keys(DEFAULT_SENSOR_CONFIG).map((key) => [
          key,
          Number($(`sensor-config-${key}`).value),
        ]),
      ),
    );
    if (
      dirty() &&
      !(await confirmAction("套用參數會清除尚未匯出的實驗紀錄。確定繼續？"))
    )
      return;
    config = validated;
    sensorConfig = nextSensors;
    try {
      localStorage.setItem(storageKey, JSON.stringify(config));
      localStorage.setItem(
        "flight-lab-sensors-v1",
        JSON.stringify(sensorConfig),
      );
    } catch {
      toast("此瀏覽器無法儲存設定，本次實驗仍會使用新參數。");
    }
    $("settings-dialog").close();
    newExperiment();
    worker.postMessage({ type: "sensorConfig", value: sensorConfig });
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
  setReplayControls(false);
  history = [];
  sampleCount = 0;
  exportedTime = 0;
  storageLimited = false;
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
$("reset").onclick = async () => {
  if (
    dirty() &&
    !(await confirmAction("重設會清除目前尚未匯出的實驗紀錄。確定重設？"))
  )
    return;
  newExperiment();
  scene?.resetView();
};
document.querySelectorAll("[data-scenario]").forEach((button) => {
  button.onclick = async () => {
    if (
      dirty() &&
      !(await confirmAction("切換情境會清除目前尚未匯出的實驗紀錄。確定切換？"))
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
  if (storageLimited && !replay) {
    toast("紀錄已達資料上限，請匯出後重設。");
    return;
  }
  if (replay) {
    if (replayIndex >= replay.samples.length - 1) {
      replayClock = replay.samples[0].time;
      replayIndex = 0;
      latest = replay.samples[0];
      history = [latest];
      restoreReplayPresentation();
      applyReplayEvents(replayClock);
      resetReplaySensors(replayClock);
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
  worker.postMessage({ type: "render", value: rendering });
  $("render-message").hidden = rendering;
  $("render-toggle").textContent = rendering ? "關閉渲染" : "開啟渲染";
  $("render-toggle").setAttribute("aria-pressed", String(rendering));
};

function download(text, extension, mime, filename) {
  const url = URL.createObjectURL(new Blob([text], { type: mime }));
  const a = document.createElement("a");
  a.href = url;
  a.download =
    filename ||
    `flight-lab-${new Date().toISOString().replace(/[:.]/g, "-")}.${extension}`;
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
  else download(recordingToJson(recording), "json", "application/json");
  if (!replay) exportedTime = recording.samples.at(-1)?.time || 0;
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
  if (file.size > 128 * 1024 * 1024) {
    toast("紀錄檔案超過 128 MiB 上限。");
    return;
  }
  try {
    const record = validateRecording(JSON.parse(await file.text()));
    if (
      dirty() &&
      !(await confirmAction(
        "匯入將取代目前畫面。尚未匯出的紀錄請先匯出。繼續匯入？",
      ))
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
    restoreReplayPresentation();
    applyReplayEvents(replayClock);
    resetReplaySensors(replayClock);
    history = [record.samples[0]];
    latest = record.samples[0];
    sampleCount = record.samples.length;
    timing = {};
    document.body.classList.add("replay");
    setReplayControls(true);
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
  if (data.type === "apiReply") {
    const pending = pendingApi.get(data.requestId);
    if (pending) {
      clearTimeout(pending.timeout);
      pendingApi.delete(data.requestId);
      if (data.ok) pending.resolve(data.value);
      else pending.reject(new Error(data.error));
    }
    return;
  }
  if (data.type === "sensorExport") {
    download(
      data.text,
      data.kind,
      data.kind === "csv" ? "text/csv;charset=utf-8" : "application/json",
      data.kind === "csv" ? "pointcloud.csv" : "sensor.json",
    );
    toast("感測器資料已匯出。");
    return;
  }
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
    worker.postMessage({
      type: "reset",
      config,
      scenario,
      setup: { ...data.setup, sensors: sensorConfig },
    });
    return;
  }
  latest = data.snapshot;
  if (data.sensors) {
    sensors = data.sensors;
    renderSensors();
  }
  if (data.setup?.sensors) sensorConfig = data.setup.sensors;
  timing = data.timing;
  sampleCount = data.sampleCount;
  if (data.message === "reset") {
    config = data.config;
    scenario = data.scenario;
    exportedTime = 0;
    $("target-roll").disabled = scenario === "step";
    document.querySelector(".control-hint").textContent =
      scenario === "step"
        ? "階躍情境自動控制 Roll；其餘指令可即時調整。"
        : "指令即時生效；更換情境會建立新實驗。";
    document.querySelectorAll("[data-scenario]").forEach((item) => {
      const active = item.dataset.scenario === scenario;
      item.classList.toggle("active", active);
      item.setAttribute("aria-pressed", String(active));
    });
    history = [];
    if (data.setup)
      for (const key of ["environment", "obstacles", "mission", "training"])
        showReplayEvent(key, data.setup[key]);
    scene?.update(latest, 0);
    scene?.resetView();
  }
  history.push(...data.samples);
  while (history.length > 400) history.shift();
  if (data.message === "reset") storageLimited = false;
  if (data.message === "storage-limit") storageLimited = true;
  $("play").disabled = storageLimited;
  setPlaying(data.running);
  updateTelemetry();
  if (data.message === "limit")
    toast("已完成 30 分鐘實驗並暫停，請匯出紀錄後重設。");
  if (data.message === "storage-limit")
    toast("紀錄接近 96 MiB 資料上限，已暫停，請匯出後重設。");
  const state = {
    snapshot: data.snapshot,
    sensors: data.sensors,
    setup: data.setup,
    running: data.running,
    timing: data.timing,
  };
  for (const callback of apiSubscribers) {
    try {
      callback(state);
    } catch (error) {
      console.error(error);
    }
  }
};
worker.onerror = (event) => {
  setPlaying(false);
  $("play").disabled = true;
  toast(`模擬執行緒無法運行：${event.message || "請重新載入頁面"}`);
};

function updateTelemetry() {
  if (!latest) return;
  const s = latest;
  for (const t of targets) {
    const value = s.target[t.key] * t.factor;
    $(`target-${t.key}`).value = value;
    $(`output-${t.key}`).textContent = `${value.toFixed(1)} ${t.unit}`;
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
  const nav = s.navigation;
  const modeLabels = {
    attitude: "姿態",
    position: "位置",
    mission: "任務",
    rtl: "返航",
    land: "降落",
  };
  const stateLabels = {
    ready: "待命",
    flying: "飛行中",
    landed: "已降落",
    failsafe: "保護模式",
    crashed: "碰撞停機",
  };
  const reasons = {
    geofence: "超出圍欄",
    "low-battery": "電量偏低",
    "critical-battery": "電量危急",
    "link-loss": "連線中斷",
    "stale-sensor": "感測資料逾時",
    "obstacle-collision": "撞上障礙物",
    rtl: "返航降落",
    crashed: "碰撞",
  };
  if (nav) $("flight-mode").value = nav.mode;
  $("flight-details").textContent = nav
    ? `模式：${modeLabels[nav.mode] || "未知"} · 航點 ${Math.min(nav.index + 1, nav.waypoints?.length || 0)}/${nav.waypoints?.length || 0} · ${nav.fenceBreached ? "超出圍欄" : "圍欄內"} ${reasons[nav.reason] || ""} · ${s.armed ? "已解鎖" : "已上鎖"} · ${stateLabels[s.status] || "未知"}`
    : `狀態：${stateLabels[s.status] || "未知"}`;
  if (s.status === "crashed") {
    $("runtime-status").textContent = "碰撞停機";
    $("view-status").innerHTML = '<span class="dot"></span>碰撞停機';
  }
  const battery = s.battery;
  $("battery-details").textContent = battery
    ? `電池 ${(battery.soc * 100).toFixed(0)}% · ${battery.voltage.toFixed(1)} V · ${battery.current.toFixed(1)} A`
    : "電池：—";
  $("fault-details").textContent = s.faults
    ? `故障：${s.faults.linkLost ? "連線中斷" : "連線正常"} · 馬達效率 ${s.faults.motorEfficiency.map((v) => `${Math.round(v * 100)}%`).join(" / ")}`
    : "故障：—";
  const training = s.training;
  $("training-status").textContent = training
    ? `${TRAINING_COURSES[training.course]?.title || training.course} · ${training.elapsed.toFixed(1)} 秒 · 通過 ${training.passed}/${training.total} · 分數 ${training.score} ${training.complete ? "· 完成" : ""} ${reasons[training.reason] || ""}`
    : "尚未開始";
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
      applyReplayEvents(latest.time);
      advanceReplaySensors(latest.time);
      history.push(latest);
      if (history.length > 400) history.shift();
    }
    if (replayIndex === replay.samples.length - 1) setPlaying(false);
    if (now - uiTime > 50) {
      updateTelemetry();
      uiTime = now;
    }
  }
  updatePilot(now);
  if (latest && rendering)
    scene?.update(latest, running ? elapsed * Number($("speed").value) : 0);
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) releasePilot();
  if (document.hidden && running) {
    if (replay) setPlaying(false);
    else worker.postMessage({ type: "pause" });
    toast("背景分頁已自動暫停。返回後可繼續。");
  }
});

const squareMission = () =>
  [
    [8, 0],
    [8, 8],
    [0, 8],
    [0, 0],
  ].map(([north, east]) => ({ north, east, altitude: 3, hold: 1 }));
let waypoints = squareMission();
const map = $("mission-map");
function drawMission() {
  const ctx = map.getContext("2d"),
    w = map.width,
    h = map.height;
  ctx.fillStyle = "#101c2d";
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = "#35566b";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 10; i++) {
    ctx.beginPath();
    ctx.moveTo((i * w) / 10, 0);
    ctx.lineTo((i * w) / 10, h);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, (i * h) / 10);
    ctx.lineTo(w, (i * h) / 10);
    ctx.stroke();
  }
  ctx.strokeStyle = "#ffbf78";
  ctx.fillStyle = "#ffbf78";
  ctx.lineWidth = 2;
  ctx.beginPath();
  waypoints.forEach((p, i) => {
    const x = ((p.east + 50) / 100) * w,
      y = ((50 - p.north) / 100) * h;
    if (i) ctx.lineTo(x, y);
    else ctx.moveTo(x, y);
  });
  ctx.stroke();
  waypoints.forEach((p, i) => {
    const x = ((p.east + 50) / 100) * w,
      y = ((50 - p.north) / 100) * h;
    ctx.beginPath();
    ctx.arc(x, y, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillText(String(i + 1), x + 7, y - 5);
  });
  scene?.setMission(waypoints);
}
function renderWaypoints() {
  const list = $("waypoint-list");
  list.replaceChildren();
  waypoints.forEach((point, index) => {
    const row = document.createElement("div");
    row.className = "waypoint-row";
    const title = document.createElement("strong");
    title.textContent = `${index + 1}`;
    row.append(title);
    for (const [key, label, min, max] of [
      ["north", "北", -50, 50],
      ["east", "東", -50, 50],
      ["altitude", "高度", 0, 100],
      ["hold", "停留秒", 0, 60],
    ]) {
      const wrapper = document.createElement("label");
      wrapper.textContent = label;
      const input = document.createElement("input");
      input.type = "number";
      input.min = min;
      input.max = max;
      input.step = "0.1";
      input.value = point[key];
      input.disabled = !!replay;
      input.onchange = () => {
        const value = Number(input.value);
        if (Number.isFinite(value) && value >= min && value <= max) {
          point[key] = value;
          drawMission();
        } else {
          input.value = point[key];
          toast(`${label}超出範圍`);
        }
      };
      wrapper.append(input);
      row.append(wrapper);
    }
    const remove = document.createElement("button");
    remove.type = "button";
    remove.textContent = "移除";
    remove.disabled = !!replay;
    remove.onclick = () => {
      waypoints.splice(index, 1);
      renderWaypoints();
    };
    row.append(remove);
    list.append(row);
  });
  drawMission();
}
map.onclick = (event) => {
  if (replay || waypoints.length >= 50) return;
  const rect = map.getBoundingClientRect();
  waypoints.push({
    north:
      Math.round((0.5 - (event.clientY - rect.top) / rect.height) * 1000) / 10,
    east:
      Math.round(((event.clientX - rect.left) / rect.width - 0.5) * 1000) / 10,
    altitude: Number($("position-altitude").value),
    hold: 1,
  });
  renderWaypoints();
};
$("waypoint-add").onclick = () => {
  if (waypoints.length < 50) {
    waypoints.push({
      north: 0,
      east: 0,
      altitude: Number($("position-altitude").value),
      hold: 1,
    });
    renderWaypoints();
  } else toast("最多 50 個航點。");
};
$("waypoint-square").onclick = () => {
  waypoints = squareMission();
  renderWaypoints();
};
$("mission-export").onclick = () =>
  download(JSON.stringify(waypoints, null, 2), "json", "application/json");
function applyMission(start) {
  if (!waypoints.length) return toast("請先新增航點。");
  worker.postMessage({ type: "mission", value: waypoints });
  worker.postMessage({ type: "mode", value: "mission" });
  $("flight-mode").value = "mission";
  if (start) worker.postMessage({ type: "play" });
}
$("mission-apply").onclick = () => applyMission(false);
$("mission-start").onclick = () => applyMission(true);
const warehouseBoxes = [
  { north: 12, east: 3, width: 2, depth: 3, height: 3 },
  { north: 12, east: -5, width: 3, depth: 2, height: 2 },
  { north: -8, east: 7, width: 3, depth: 2, height: 2.5 },
  { north: -8, east: -7, width: 2, depth: 3, height: 3 },
];
function applyEnvironment() {
  const value = {
    preset: $("environment-preset").value,
    timeOfDay: Number($("environment-time").value),
    fog: Number($("environment-fog").value),
    rain: Number($("environment-rain").value),
  };
  if (
    !Number.isFinite(value.timeOfDay) ||
    value.timeOfDay < 0 ||
    value.timeOfDay > 24
  )
    return toast("時間須介於 0 至 24 時。");
  const boxes = value.preset === "warehouse" ? warehouseBoxes : [];
  scene?.setEnvironment(value);
  scene?.setObstacles(boxes);
  worker.postMessage({ type: "environment", value });
  worker.postMessage({ type: "obstacles", value: boxes });
}
for (const id of [
  "environment-preset",
  "environment-time",
  "environment-fog",
  "environment-rain",
])
  $(id).onchange = applyEnvironment;
$("training-select").onchange = () => {
  const value = $("training-select").value;
  scene?.setGates(TRAINING_COURSES[value].gates);
  worker.postMessage({ type: "training", value });
};
$("flight-mode").onchange = (event) =>
  worker.postMessage({ type: "mode", value: event.target.value });
document
  .querySelectorAll("[data-command]")
  .forEach(
    (button) =>
      (button.onclick = () =>
        worker.postMessage({ type: button.dataset.command })),
  );
document.querySelectorAll("[data-mode]").forEach(
  (button) =>
    (button.onclick = () => {
      $("flight-mode").value = button.dataset.mode;
      worker.postMessage({ type: "mode", value: button.dataset.mode });
    }),
);
$("position-apply").onclick = () => {
  const value = {
    north: Number($("position-north").value),
    east: Number($("position-east").value),
    altitude: Number($("position-altitude").value),
  };
  worker.postMessage({ type: "mode", value: "position" });
  worker.postMessage({ type: "position", value });
  $("flight-mode").value = "position";
};
$("fault-apply").onclick = () =>
  worker.postMessage({
    type: "fault",
    fault: "motor",
    value: {
      index: Number($("fault-motor").value),
      efficiency: Number($("fault-efficiency").value),
    },
  });
$("fault-link").onclick = () =>
  worker.postMessage({ type: "fault", fault: "link", value: { lost: true } });
$("fault-clear").onclick = () => worker.postMessage({ type: "clearFaults" });
$("model-preset").onchange = (event) => {
  const presets = {
    micro: {
      mass: 0.3,
      arm: 0.1,
      maxMotorThrust: 3,
      motorTau: 0.03,
      batteryCapacityWh: 15,
    },
    research: {
      mass: 1.2,
      arm: 0.23,
      maxMotorThrust: 8,
      motorTau: 0.04,
      batteryCapacityWh: 60,
    },
    heavy: {
      mass: 5,
      arm: 0.5,
      maxMotorThrust: 30,
      motorTau: 0.08,
      batteryCapacityWh: 250,
    },
  };
  const values = presets[event.target.value];
  if (!values) return;
  for (const [key, value] of Object.entries(values)) {
    const field = fields.find((item) => item[0] === key);
    $(`config-${key}`).value = value * field[5];
  }
};
for (const button of document.querySelectorAll("[data-workspace]"))
  button.onclick = () => {
    for (const item of document.querySelectorAll("[data-workspace]")) {
      item.classList.toggle("active", item === button);
      item.setAttribute("aria-pressed", String(item === button));
    }
    for (const name of ["mission", "training", "sensors", "catalog"])
      $(`${name}-workspace`).hidden = button.dataset.workspace !== name;
    document.querySelector(".workspace").hidden =
      button.dataset.workspace === "catalog";
  };
const category = $("catalog-category"),
  status = $("catalog-status");
for (const value of [...new Set(CAPABILITIES.map((item) => item.category))])
  category.add(new Option(value, value));
const statusNames = {
  implemented: "已實作",
  partial: "部分實作",
  planned: "規劃中",
  external: "需外部整合",
};
for (const value of [...new Set(CAPABILITIES.map((item) => item.status))])
  status.add(new Option(statusNames[value] || value, value));
function renderCatalog() {
  const search = $("catalog-search").value.trim().toLowerCase();
  const selected = CAPABILITIES.filter(
    (item) =>
      (!category.value || item.category === category.value) &&
      (!status.value || item.status === status.value) &&
      `${item.title} ${item.description} ${item.category}`
        .toLowerCase()
        .includes(search),
  );
  $("catalog-counts").textContent =
    `顯示 ${selected.length} / ${CAPABILITIES.length} 項 · ` +
    Object.entries(statusNames)
      .map(
        ([key, label]) =>
          `${label} ${CAPABILITIES.filter((item) => item.status === key).length}`,
      )
      .join(" · ");
  const list = $("catalog-list");
  list.replaceChildren();
  for (const item of selected) {
    const article = document.createElement("article");
    article.className = "catalog-item";
    const title = document.createElement("strong");
    title.textContent = item.title;
    const badge = document.createElement("span");
    badge.textContent = `${item.category} · ${statusNames[item.status] || item.status}`;
    const description = document.createElement("p");
    description.textContent = item.description;
    article.append(title, badge, description);
    for (const url of item.sources) {
      const source = SOURCES.find((entry) => entry.url === url);
      const link = document.createElement("a");
      link.href = url;
      link.target = "_blank";
      link.rel = "noopener noreferrer";
      link.textContent = `${source?.name || "來源"} ↗`;
      article.append(link, " ");
    }
    list.append(article);
  }
}
for (const id of ["catalog-search", "catalog-category", "catalog-status"])
  $(id).addEventListener("input", renderCatalog);
const keys = new Set();
let pilotTime = 0;
let pilotWasActive = false;
function releasePilot() {
  keys.clear();
  if (pilotWasActive && !replay)
    worker.postMessage({
      type: "targets",
      value: { roll: 0, pitch: 0, yawRate: 0 },
    });
  pilotWasActive = false;
}
$("pilot-enable").onchange = (event) => {
  if (event.target.checked && !replay) {
    event.target.blur();
    $("flight-mode").value = "attitude";
    worker.postMessage({ type: "mode", value: "attitude" });
  } else releasePilot();
};
window.addEventListener("keydown", (event) => {
  if (
    !$("pilot-enable").checked ||
    replay ||
    event.target.closest("input,select,textarea,button,dialog")
  )
    return;
  if (
    [
      "KeyW",
      "KeyA",
      "KeyS",
      "KeyD",
      "ArrowUp",
      "ArrowDown",
      "ArrowLeft",
      "ArrowRight",
    ].includes(event.code)
  ) {
    keys.add(event.code);
    event.preventDefault();
  }
});
window.addEventListener("keyup", (event) => keys.delete(event.code));
document.addEventListener("focusin", (event) => {
  if (
    event.target.matches("input,select,textarea,button") ||
    event.target.closest("dialog")
  )
    releasePilot();
});
window.addEventListener("blur", releasePilot);
function updatePilot(now) {
  if (
    !$("pilot-enable").checked ||
    !running ||
    replay ||
    !latest ||
    document.hidden ||
    now - pilotTime < 50
  )
    return;
  const dt = pilotTime ? Math.min((now - pilotTime) / 1000, 0.1) : 0.05;
  pilotTime = now;
  let pad;
  try {
    pad = navigator.getGamepads?.()?.find((item) => item?.connected);
  } catch {
    $("pilot-enable").checked = false;
    releasePilot();
    toast("瀏覽器禁止讀取手把，已停用手動操控。");
    return;
  }
  const input = readPilotInput(keys, pad);
  const value = {
    roll: (input.roll * 20) / DEG,
    pitch: (input.pitch * 20) / DEG,
    yawRate: (input.yaw * 60) / DEG,
    altitude: Math.max(
      0,
      Math.min(100, latest.target.altitude + input.climb * 2 * dt),
    ),
  };
  worker.postMessage({ type: "targets", value });
  pilotWasActive = true;
  for (const t of targets) {
    const display = value[t.key] * t.factor;
    $(`target-${t.key}`).value = display;
    $(`output-${t.key}`).textContent = `${display.toFixed(1)} ${t.unit}`;
  }
}
function setReplayControls(disabled) {
  document
    .querySelectorAll(
      "#mission-workspace button:not(#mission-export), #training-workspace input, #training-workspace select, #sensor-fault-channel, #sensor-fault-on, #sensor-fault-off, #sensor-export-json, #sensor-export-csv, #flight-mode, [data-command], [data-mode], #position-apply, #fault-apply, #fault-link, #fault-clear, #pilot-enable",
    )
    .forEach((element) => {
      element.disabled = disabled;
    });
  if (disabled) {
    $("pilot-enable").checked = false;
    releasePilot();
  }
  if (typeof waypoints !== "undefined") renderWaypoints();
}
renderWaypoints();
renderCatalog();

function showReplayEvent(type, value) {
  if (type === "environment") {
    scene?.setEnvironment(value);
    $("environment-preset").value = value.preset;
    $("environment-time").value = value.timeOfDay;
    $("environment-fog").value = value.fog;
    $("environment-rain").value = value.rain;
  } else if (type === "obstacles") scene?.setObstacles(value);
  else if (type === "mission") {
    waypoints = structuredClone(value);
    renderWaypoints();
  } else if (type === "sensorConfig" || type === "sensors") {
    sensorConfig = value;
  } else if (type === "training") {
    $("training-select").value = value;
    scene?.setGates(TRAINING_COURSES[value].gates);
  }
}
function restoreReplayPresentation() {
  replayEventIndex = 0;
  sensorConfig = replay.setup.sensors || { ...DEFAULT_SENSOR_CONFIG };
  for (const key of ["environment", "obstacles", "mission", "training"])
    showReplayEvent(key, replay.setup[key]);
}
function applyReplayEvents(time) {
  while (
    replayEventIndex < replay.events.length &&
    replay.events[replayEventIndex].time <= time + 1e-8
  ) {
    const event = replay.events[replayEventIndex++];
    showReplayEvent(event.type, event.value);
  }
}

const sensorLabels = {
  gpsHz: "GPS 更新率 (Hz)",
  barometerHz: "氣壓計更新率 (Hz)",
  magnetometerHz: "磁力計更新率 (Hz)",
  rangefinderHz: "測距更新率 (Hz)",
  lidarHz: "LiDAR 更新率 (Hz)",
  opticalFlowHz: "光流更新率 (Hz)",
  latency: "感測延遲 (秒)",
  dropout: "丟包率 (0–1)",
  originLatitudeDeg: "原點緯度 (°)",
  originLongitudeDeg: "原點經度 (°)",
  originAltitudeM: "原點海拔 (m)",
  gpsNoiseM: "GPS 水平雜訊 (m)",
  gpsVerticalNoiseM: "GPS 垂直雜訊 (m)",
  gpsVelocityNoiseMps: "GPS 速度雜訊 (m/s)",
  barometerNoisePa: "氣壓雜訊 (Pa)",
  magneticFieldMicroT: "磁場強度 (µT)",
  magneticDeclinationDeg: "磁偏角 (°)",
  magneticInclinationDeg: "磁傾角 (°)",
  magnetometerNoiseMicroT: "磁力雜訊 (µT)",
  rangefinderMinM: "測距最小值 (m)",
  rangefinderMaxM: "測距最大值 (m)",
  rangefinderNoiseM: "測距雜訊 (m)",
  lidarRays: "LiDAR 射線數",
  lidarMinM: "LiDAR 最小值 (m)",
  lidarMaxM: "LiDAR 最大值 (m)",
  lidarNoiseM: "LiDAR 雜訊 (m)",
  opticalFlowNoiseRadS: "光流雜訊 (rad/s)",
};
const sensorActions = document.querySelector(
  "#sensors-workspace .feature-actions",
);
for (const [kind, label] of [
  ["json", "匯出目前感測狀態 JSON"],
  ["csv", "匯出目前 LiDAR 點雲 CSV"],
]) {
  const button = document.createElement("button");
  button.type = "button";
  button.id = `sensor-export-${kind}`;
  button.textContent = label;
  button.onclick = () => worker.postMessage({ type: "sensorExport", kind });
  sensorActions.append(button);
}
const sensorSettings = $("sensor-settings");
for (const key of Object.keys(DEFAULT_SENSOR_CONFIG)) {
  const field = document.createElement("div");
  field.className = "settings-field";
  const label = document.createElement("label");
  label.htmlFor = `sensor-config-${key}`;
  label.textContent = sensorLabels[key] || key;
  const input = document.createElement("input");
  input.type = "number";
  input.step = "any";
  input.required = true;
  input.id = label.htmlFor;
  field.append(label, input);
  sensorSettings.append(field);
}
function populateSensorConfig(values) {
  for (const key of Object.keys(DEFAULT_SENSOR_CONFIG))
    $(`sensor-config-${key}`).value = values[key];
}
const sensorNames = {
  gps: "GPS",
  barometer: "氣壓計",
  magnetometer: "磁力計",
  rangefinder: "向下測距",
  lidar: "LiDAR",
  opticalFlow: "光流",
};
const reasonsSensor = {
  startup: "尚未量測",
  "no-return": "無回波",
  dropout: "丟包",
  fault: "故障",
  unavailable: "無資料",
  range: "超出範圍",
};
const valueText = (value, digits = 2) =>
  Number.isFinite(value) ? value.toFixed(digits) : "—";
const vectorText = (value, digits = 2) =>
  Array.isArray(value)
    ? value.map((v) => valueText(v, digits)).join(" / ")
    : "—";
function renderSensors() {
  if (!sensors) return;
  const readings = $("sensor-readings");
  readings.replaceChildren();
  const detail = {
    gps: (s) => [
      `緯度 / 經度 ${valueText(s.latitudeDeg, 6)} / ${valueText(s.longitudeDeg, 6)} °`,
      `海拔 ${valueText(s.altitudeM)} m`,
      `NED 速度 ${vectorText(s.velocityNedMps)} m/s`,
    ],
    barometer: (s) => [
      `氣壓 ${valueText(s.pressurePa)} Pa`,
      `高度 ${valueText(s.altitudeM)} m`,
      `溫度 ${valueText(s.temperatureK)} K`,
    ],
    magnetometer: (s) => [`機體磁場 ${vectorText(s.fieldBodyMicroT)} µT`],
    rangefinder: (s) => [`距離 ${valueText(s.distanceM)} m`],
    lidar: (s) => [
      `有效回波 ${Array.isArray(s.rangesM) ? s.rangesM.filter(Number.isFinite).length : 0} / ${s.rangesM?.length || 0}`,
      `量測範圍 ${valueText(s.minM)}–${valueText(s.maxM)} m`,
    ],
    opticalFlow: (s) => [
      `光流 ${vectorText(s.flowRadS, 3)} rad/s`,
      `地面距離 ${valueText(s.groundDistanceM)} m`,
    ],
  };
  for (const [key, name] of Object.entries(sensorNames)) {
    const item = sensors[key];
    if (!item) continue;
    const card = document.createElement("article");
    card.className = "sensor-card";
    const title = document.createElement("h3");
    title.textContent = name;
    card.append(title);
    const state = document.createElement("p");
    state.textContent = `${item.valid ? "有效" : `無效（${reasonsSensor[item.reason] || "無資料"}）`} · 量測 ${valueText(item.time, 3)} s · 齡期 ${valueText(Number.isFinite(item.time) ? Math.max(0, sensors.time - item.time) : null, 3)} s${sensors.faults?.[key] ? " · 故障啟用" : ""}`;
    card.append(state);
    for (const line of detail[key](item)) {
      const p = document.createElement("p");
      p.textContent = line;
      card.append(p);
    }
    readings.append(card);
  }
  drawLidar(sensors.lidar);
  scene?.setSensorScan($("sensor-scan-visible").checked ? sensors.lidar : null);
}
function drawLidar(reading) {
  const canvas = $("lidar-map"),
    ctx = canvas.getContext("2d"),
    size = canvas.width,
    center = size / 2;
  ctx.clearRect(0, 0, size, size);
  ctx.strokeStyle = "#35566b";
  for (const radius of [0.25, 0.5, 0.75, 1]) {
    ctx.beginPath();
    ctx.arc(center, center, radius * (center - 15), 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.beginPath();
  ctx.moveTo(center, 0);
  ctx.lineTo(center, size);
  ctx.moveTo(0, center);
  ctx.lineTo(size, center);
  ctx.stroke();
  if (!reading?.valid || !Array.isArray(reading.rangesM)) return;
  ctx.fillStyle = "#ffbf78";
  reading.rangesM.forEach((range, i) => {
    if (!Number.isFinite(range)) return;
    const angle = reading.angleStartRad + i * reading.angleStepRad,
      radius = (range / reading.maxM) * (center - 15);
    ctx.fillRect(
      center + Math.sin(angle) * radius - 2,
      center - Math.cos(angle) * radius - 2,
      4,
      4,
    );
  });
}
$("sensor-scan-visible").onchange = () =>
  scene?.setSensorScan(
    $("sensor-scan-visible").checked ? sensors?.lidar : null,
  );
for (const [id, failed] of [
  ["sensor-fault-on", true],
  ["sensor-fault-off", false],
])
  $(id).onclick = () =>
    worker.postMessage({
      type: "sensorFault",
      value: { sensor: $("sensor-fault-channel").value, failed },
    });

function resetReplaySensors(time) {
  replaySensorIndex = 0;
  sensors = undefined;
  $("sensor-readings").replaceChildren();
  scene?.setSensorScan(null);
  advanceReplaySensors(time);
}
function advanceReplaySensors(time) {
  const frames = replay?.sensorFrames || [];
  let changed = false;
  while (
    replaySensorIndex < frames.length &&
    frames[replaySensorIndex].time <= time + 1e-8
  ) {
    sensors = frames[replaySensorIndex++];
    changed = true;
  }
  if (changed) renderSensors();
}
