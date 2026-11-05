import { DEFAULT_CONFIG, AIRFRAME_PRESETS, validateConfig } from "./sim.js";
import { MULTIROTOR_AIRFRAMES } from "./rotor-geometry.js";
import { createScene } from "./scene.js";
import { validateWorldConfig } from "./world.js";
import {
  validateRecording,
  recordingToCsv,
  recordingToJson,
  recordingWorldAt,
  recordingInspectionReport,
} from "./recording.js";
import { readPilotInput, takeoffPlayAction } from "./input.js";
import { exportVehiclePointCloud } from "./pointcloud.js";
import { exportRadarData } from "./radar.js";
import { exportProximityData, PROXIMITY_BOUNDS } from "./proximity.js";
import { createProximityPanel } from "./proximity-panel.js";
import {
  createHilClient,
  isCompanionOrigin,
  hilFlightReadiness,
  formatHilHealth,
  hilWebControlAllowed,
  validGroundStationPort,
  shouldFollowTakeoff,
  externalMissionIndex,
  externalMissionReachedCount,
  externalMissionControls,
} from "./hil-client.js";
import { createFlightApi } from "./api.js";
import {
  DEFAULT_CAMERA_CONFIG,
  validateCameraConfig,
  cameraPreview,
  cameraFrameBytes,
  cameraDatasetToJson,
  hashCameraFrame,
  parseCameraDataset,
  CAMERA_DATASET_MAX_FRAMES,
  CAMERA_DATASET_MAX_BYTES,
  CAMERA_DATASET_MAX_IMPORT_BYTES,
  CAMERA_CLASSES,
} from "./camera.js";
import { DEFAULT_SENSOR_CONFIG, validateSensorConfig } from "./sensors.js";
import {
  TRAINING_COURSES,
  TUTORIAL_STEPS,
  isCourseSupported,
  resolveTrainingGates,
} from "./training.js";
import { createSceneEditor } from "./scene-editor.js";
import { createDemEditor } from "./dem-editor.js";
import { createReplayControls } from "./replay-controls.js";
import { createSurveyEditor } from "./survey-editor.js";
import { createAutomaticCamera } from "./automatic-camera.js";
import { createAutomaticCameraControls } from "./automatic-camera-controls.js";
import { replayFrameAtTime } from "./replay.js";
import { validateScene } from "./scene-data.js";
import { createTerrain } from "./terrain.js";
import { CAPABILITIES, SOURCES } from "./capabilities.js";

const $ = (id) => document.getElementById(id);
const DEG = 180 / Math.PI;
const storageKey = "flight-lab-config-v1";
let config = { ...DEFAULT_CONFIG, startOnGround: true };
let sensorConfig = { ...DEFAULT_SENSOR_CONFIG };
let cameraConfig = { ...DEFAULT_CAMERA_CONFIG };
const ORIGIN_KEYS = [
  "originLatitudeDeg",
  "originLongitudeDeg",
  "originAltitudeM",
  "originVerticalDatum",
];
let sensors;

try {
  const saved = localStorage.getItem(storageKey);
  if (saved) config = validateConfig(JSON.parse(saved));
} catch {
  /* Invalid or unavailable storage uses model defaults. */
}
try {
  const savedSensors = localStorage.getItem("flight-lab-sensors-v1");
  if (savedSensors)
    sensorConfig = validateSensorConfig(JSON.parse(savedSensors));
} catch {
  /* Invalid or unavailable storage uses sensor defaults. */
}
try {
  const savedCamera = localStorage.getItem("flight-lab-camera-v2");
  if (savedCamera) cameraConfig = validateCameraConfig(JSON.parse(savedCamera));
} catch {
  /* Invalid or unavailable storage uses camera defaults. */
}

let worldState;
let worldConfig;
let automaticCamera;
let automaticCameraControls;
let cameraBusy = false;
let cameraDatasetBusy = false;
let surveyApplying = false;
let activeVehicleId = "drone-1";
const vehicleHistories = new Map();
const vehicleDrafts = new Map();
const vehicleSensorOrigins = new Map();
let rosterDraft;
let rosterKey = "";
let scenario = "hover";
let localSensorOrigin = Object.fromEntries(
  ORIGIN_KEYS.map((key) => [key, sensorConfig[key]]),
);
let running = false;
let latest;
let history = [];
let replay;
let replayPrevious;
let replayClock = 0;
let replayIndex = 0;
const replayControls = createReplayControls({ seek: seekReplayFrame });

let rendering = true;
let scene;
let sceneEditor;
let demEditor;
let lastRenderedSceneKey = "";
let lastRenderedSceneReadonly = false;
let toastTimer;
let sampleCount = 0;
let storageLimited = false;
let worldTermination = null;
let runwayCompatible = true;
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
let externalActive = false;
let hilClient;
let hilState;
function hilControllerName() {
  const kind = externalActive
    ? hilState?.controller?.kind
    : $("hil-type").value;
  return kind === "ardupilot" ? "ArduPilot" : "PX4";
}
let internalSaved;
const worker = new Worker(new URL("./worker.js", import.meta.url), {
  type: "module",
});
const globalWorkerCommands = new Set([
  "play",
  "pause",
  "speed",
  "render",
  "scene",
  "wind",
  "resetWorld",
  "selectVehicle",
  "inspectWorld",
  "inspectScene",
  "apiExport",
  "export",
]);
function sendWorker(command) {
  if (command.type === "reset") {
    if (!worldConfig) throw new Error("世界尚未準備完成");
    const value = structuredClone(worldConfig);
    const resetVehicleId = command.vehicleId || activeVehicleId;
    const vehicle = value.vehicles.find((item) => item.id === resetVehicleId);
    if (!vehicle) throw new Error("找不到指定載具");
    const nextConfig = command.config || config;
    value.wind = {
      speed: nextConfig.wind,
      direction: nextConfig.windDirection,
    };
    for (const item of value.vehicles) {
      item.config.wind = value.wind.speed;
      item.config.windDirection = value.wind.direction;
    }
    const previousAirframe = vehicle.config.airframe;
    vehicle.config = { ...nextConfig };
    vehicle.scenario = command.scenario || scenario;
    vehicle.spawn.altitude = nextConfig.startOnGround
      ? 0
      : nextConfig.airframe !== previousAirframe || !vehicle.spawn.altitude
        ? nextConfig.airframe === "fixedwing"
          ? 30
          : 2
        : vehicle.spawn.altitude;
    if (command.setup) {
      const { scene, ...setup } = command.setup;
      vehicle.setup = setup;
      if (scene) {
        const geographyChanged =
          JSON.stringify(value.scene.geography) !==
          JSON.stringify(scene.geography);
        value.scene = scene;
        for (const item of value.vehicles) {
          if (geographyChanged) {
            item.setup.mode = "attitude";
            item.setup.mission = [];
          }
          const origin = scene.geography?.origin;
          if (geographyChanged && origin)
            Object.assign(item.setup.sensors, {
              originLatitudeDeg: origin.latitudeDeg,
              originLongitudeDeg: origin.longitudeDeg,
              originAltitudeM: origin.altitudeM,
              originVerticalDatum: origin.verticalDatum,
            });
          else if (geographyChanged && vehicleSensorOrigins.has(item.id))
            Object.assign(
              item.setup.sensors,
              vehicleSensorOrigins.get(item.id),
            );
        }
      }
    } else {
      vehicle.setup.mode = "attitude";
      vehicle.setup.mission = [];
    }
    command = {
      type: "resetWorld",
      value,
      activeVehicleId: resetVehicleId,
      ...(command.requestId ? { requestId: command.requestId } : {}),
    };
  }
  worker.postMessage(
    globalWorkerCommands.has(command.type)
      ? command
      : { ...command, vehicleId: command.vehicleId || activeVehicleId },
  );
}
function renderVehicleRoster() {
  if (!worldState) return;
  const list = worldState.vehicles;
  const key = JSON.stringify(list.map((item) => [item.id, item.name]));
  const select = $("vehicle-select");
  if (key !== rosterKey) {
    select.replaceChildren(
      ...list.map((item) => new Option(item.name + " · " + item.id, item.id)),
    );
    rosterKey = key;
  }
  select.value = activeVehicleId;
  select.disabled = externalActive;
  $("vehicle-status").textContent = list
    .map(
      (item) =>
        item.name +
        "（" +
        item.id +
        "） · " +
        item.snapshot.airframe +
        " · " +
        item.snapshot.status +
        " · 離地 " +
        item.snapshot.ground.clearanceM.toFixed(1) +
        " m",
    )
    .join("　｜　");
  $("vehicle-add").disabled =
    !!replay || externalActive || rosterDraft?.vehicles.length >= 8;
  $("vehicle-apply").disabled = !!replay || externalActive;
}
function renderVehicleEditor() {
  const container = $("vehicle-editor");
  container.replaceChildren();
  if (!rosterDraft) return;
  for (const vehicle of rosterDraft.vehicles) {
    const row = document.createElement("div");
    row.className = "vehicle-editor-row";
    const add = (label, value, onChange, type = "text") => {
      const wrapper = document.createElement("label");
      wrapper.textContent = label;
      const input = document.createElement("input");
      input.type = type;
      input.value = value;
      if (type === "number") input.step = "any";
      input.disabled = !!replay || externalActive;
      input.oninput = () =>
        onChange(type === "number" ? Number(input.value) : input.value);
      wrapper.append(input);
      row.append(wrapper);
    };
    add("ID", vehicle.id, (value) => {
      vehicle.id = value;
    });
    add("名稱", vehicle.name, (value) => {
      vehicle.name = value;
    });
    const typeLabel = document.createElement("label");
    typeLabel.textContent = "機型";
    const type = document.createElement("select");
    for (const kind of [
      "quadrotor",
      "fixedwing",
      "vtol",
      "hexarotor",
      "octorotor",
    ])
      type.add(new Option(kind, kind));
    type.value = vehicle.config.airframe;
    type.disabled = !!replay || externalActive;
    type.onchange = () => {
      const ground = vehicle.config.startOnGround;
      vehicle.config = {
        ...AIRFRAME_PRESETS[type.value],
        startOnGround: ground,
        wind: rosterDraft.wind.speed,
        windDirection: rosterDraft.wind.direction,
      };
      vehicle.spawn.altitude = ground ? 0 : type.value === "fixedwing" ? 30 : 2;
      renderVehicleEditor();
    };
    typeLabel.append(type);
    row.append(typeLabel);
    const groundLabel = document.createElement("label"),
      ground = document.createElement("input");
    ground.type = "checkbox";
    ground.checked = vehicle.config.startOnGround;
    ground.disabled = !!replay || externalActive;
    ground.onchange = () => {
      vehicle.config.startOnGround = ground.checked;
      vehicle.spawn.altitude = ground.checked
        ? 0
        : vehicle.config.airframe === "fixedwing"
          ? 30
          : 2;
      renderVehicleEditor();
    };
    groundLabel.append("地面起始", ground);
    row.append(groundLabel);
    for (const [key, label] of [
      ["north", "起點北 m"],
      ["east", "起點東 m"],
      ["altitude", "起點離地 m"],
      ["heading", "航向 °"],
    ])
      add(
        label,
        key === "heading" ? vehicle.spawn[key] * DEG : vehicle.spawn[key],
        (value) => {
          vehicle.spawn[key] = key === "heading" ? value / DEG : value;
        },
        "number",
      );
    for (const [index, label] of [
      [0, "家點北 m"],
      [1, "家點東 m"],
    ])
      add(
        label,
        vehicle.home[index],
        (value) => {
          vehicle.home[index] = value;
        },
        "number",
      );
    const remove = document.createElement("button");
    remove.textContent = "移除";
    remove.disabled =
      !!replay || externalActive || rosterDraft.vehicles.length === 1;
    remove.onclick = () => {
      rosterDraft.vehicles.splice(rosterDraft.vehicles.indexOf(vehicle), 1);
      renderVehicleEditor();
    };
    row.append(remove);
    container.append(row);
  }
  renderVehicleRoster();
}
$("vehicle-add").onclick = () => {
  if (
    !rosterDraft ||
    replay ||
    externalActive ||
    rosterDraft.vehicles.length >= 8
  )
    return;
  const vehicle = structuredClone(rosterDraft.vehicles[0]);
  let index = 1;
  while (rosterDraft.vehicles.some((item) => item.id === "drone-" + index))
    index++;
  vehicle.id = "drone-" + index;
  vehicle.name = "飛行器 " + index;
  vehicle.spawn = {
    north: 0,
    east: 6 * (index - 1),
    altitude: vehicle.config.startOnGround
      ? 0
      : vehicle.config.airframe === "fixedwing"
        ? 30
        : 2,
    heading: 0,
  };
  vehicle.home = [vehicle.spawn.north, vehicle.spawn.east];
  vehicle.setup.mode = "attitude";
  vehicle.setup.mission = [];
  rosterDraft.vehicles.push(vehicle);
  renderVehicleEditor();
};
$("vehicle-apply").onclick = async () => {
  if (!rosterDraft || replay || externalActive) return;
  try {
    const value = validateWorldConfig(rosterDraft);
    if (
      dirty() &&
      !(await confirmAction(
        "套用機隊會重設整個實驗並清除尚未匯出的紀錄。確定繼續？",
      ))
    )
      return;
    const selected = value.vehicles.some((item) => item.id === activeVehicleId)
      ? activeVehicleId
      : value.vehicles[0].id;
    await requestApi({ type: "resetWorld", value, activeVehicleId: selected });
    vehicleHistories.clear();
    vehicleDrafts.clear();
    toast("機隊已套用。");
  } catch (error) {
    toast("無法套用機隊：" + error.message);
  }
};
$("vehicle-select").onchange = () => {
  const id = $("vehicle-select").value;
  if (externalActive) return;
  if (replay) {
    activeVehicleId = id;
    projectReplayWorld(true);
    return;
  }
  sendWorker({ type: "selectVehicle", vehicleId: id });
};
let lastAltitudeReference;
let requestId = 0;
const pendingApi = new Map();
const apiSubscribers = new Set();
function requestApi(command, internal = false) {
  if (externalActive && !internal)
    return Promise.reject(new Error("請先回到內建模擬再使用程式介面"));
  if (replay && !internal)
    return Promise.reject(new Error("請先離開重播模式再使用程式介面"));
  if (command.type === "getCameraConfig")
    return Promise.resolve({ ...cameraConfig });
  if (command.type === "setCameraConfig") {
    try {
      return Promise.resolve(applyCameraConfig(command.value));
    } catch (error) {
      return Promise.reject(error);
    }
  }
  if (["cameraAutoState", "cameraAutoStart", "cameraAutoStop"].includes(command.type)) {
    try {
      if (command.type === "cameraAutoStart")
        startAutomaticCamera(command.value, command.vehicleId);
      else if (command.type === "cameraAutoStop")
        automaticCamera.stop();
      return Promise.resolve(automaticCamera.getState());
    } catch (error) {
      return Promise.reject(error);
    }
  }
  if (command.type === "captureCamera") {
    if (automaticCamera?.getState().active || automaticCamera?.getState().pending)
      return Promise.reject(new Error("請先停止自動拍攝並等待目前影像完成。"));
    return captureCameraFrame(command.vehicleId).then((frame) => {
      showCameraFrame(frame);
      return frame;
    });
  }
  if (command.type === "captureInspection")
    return captureInspectionFrame(command.vehicleId, command.value);
  const id = ++requestId;
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {
      pendingApi.delete(id);
      reject(new Error("模擬器未在時間內回應"));
    }, 10000);
    pendingApi.set(id, { resolve, reject, timeout });
    try {
      sendWorker({ ...command, requestId: id });
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
  if (replay && !value) replayClock = replay.samples[replayIndex].time;
  $("replay-position").setAttribute("aria-live", value ? "off" : "polite");
  const takeoffAction = takeoffPlayAction(latest);
  const playLabel = replay
    ? "播放重播"
    : takeoffAction === "resume"
      ? "繼續起飛"
      : takeoffAction === "launch"
        ? "起飛並開始"
        : latest?.time > 0
          ? "繼續模擬"
          : "開始模擬";
  const externalTakeoff =
    externalActive &&
    hilState?.phase === "running" &&
    latest?.runway.weightOnWheels &&
    hilWebControlAllowed(hilState);
  $("play").innerHTML = externalTakeoff
    ? `<span aria-hidden="true">▶</span> ${hilFlightReadiness(hilState).length ? "等待飛控準備" : `${hilControllerName()} 起飛`}`
    : value
      ? '<span aria-hidden="true">Ⅱ</span> 暫停模擬'
      : `<span aria-hidden="true">▶</span> ${externalActive ? `繼續 ${hilControllerName()} 模擬` : playLabel}`;
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
  if (!externalActive && worldTermination && !value)
    $("run-label").textContent =
      worldTermination.reason === "vehicle-collision"
        ? "載具碰撞，世界已停止。"
        : "已到達高程資料邊界，請調整航線或更換地形範圍。";
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
  $("view-status").innerHTML =
    `<span class="dot"></span>${externalActive ? (hilState?.controller?.verified === true && hilState?.stats?.sensorPackets > 0 && hilState?.stats?.actuatorPackets > 0 ? `${hilControllerName()} SITL 閉環 · 植物模型真值` : `${hilControllerName()} 介接中`) : label}`;
  observeAutomaticCamera();
}

const targets = [
  {
    key: "airspeed",
    label: "目標空速",
    min: 0,
    max: 40,
    step: 0.5,
    value: 0,
    unit: "m/s",
    factor: 1,
  },
  {
    key: "altitude",
    label: "目標高度（基準面）",
    min: 0,
    max: 100,
    step: 0.1,
    value: 2,
    unit: "m",
    factor: 1,
  },
  {
    key: "roll",
    label: "滾轉 Roll",
    min: -30,
    max: 30,
    step: 1,
    value: 0,
    unit: "°",
    factor: DEG,
  },
  {
    key: "pitch",
    label: "俯仰 Pitch",
    min: -30,
    max: 30,
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
    sendWorker({
      type: "targets",
      value: { [t.key]: value / t.factor },
    });
  });
}

let displayedMotorLayout = "";
function updateMotorLayout(snapshot) {
  const kind = snapshot.airframe;
  const count = kind === "fixedwing" ? 0 : snapshot.motors.length;
  const key = `${kind}:${count}`;
  if (displayedMotorLayout === key) return;
  displayedMotorLayout = key;
  $("motor-bars").innerHTML = Array.from(
    { length: count },
    (_, i) =>
      `<div class="motor-row"><span class="motor-name">M${i + 1}</span><div class="motor-track"><div id="motor-${i + 1}" class="motor-fill"></div></div><span class="motor-reading"><span id="pwm-${i + 1}">—</span> <small id="motor-unit-${i + 1}">µs</small></span></div>`,
  ).join("");
  const select = $("fault-motor"),
    previous = select.value;
  select.replaceChildren(
    ...Array.from(
      { length: count },
      (_, i) => new Option(`M${i + 1}`, String(i)),
    ),
  );
  if ([...select.options].some((option) => option.value === previous))
    select.value = previous;
}

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
groups.push([
  "固定翼與巡航",
  [
    ["wingArea", "翼面積 (m²)", 0.1, 3, 0.01, 1],
    ["wingSpan", "翼展 (m)", 0.5, 5, 0.1, 1],
    ["wingChord", "平均翼弦 (m)", 0.1, 1, 0.01, 1],
    ["maxPusherThrust", "巡航最大推力 (N)", 0.5, 80, 0.1, 1],
    ["servoTau", "舵機時間常數 (ms)", 20, 500, 1, 1000],
    ["cruiseAirspeed", "巡航空速 (m/s)", 8, 35, 0.5, 1],
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
  $("config-airframe").value = values.airframe;
  $("config-start-on-ground").checked = values.startOnGround;
  $("model-preset").disabled = values.airframe !== "quadrotor";
  for (const [key, , , , , factor] of fields)
    $(`config-${key}`).value = Number((values[key] * factor).toFixed(8));
  $("settings-error").textContent = "";
}
$("settings-open").onclick = () => {
  if (!latest?.gimbal) return toast("模型正在初始化，請稍候再開啟參數。");
  populateConfig(config);
  populateSensorConfig(sensorConfig);
  populateCameraConfig(cameraConfig);
  for (const axis of ["yaw", "pitch", "roll"])
    $("gimbal-rate-" + axis).value = (
      latest.gimbal.maxRate[axis] * DEG
    ).toFixed(2);
  $("settings-dialog").showModal();
};
$("config-airframe").onchange = (event) => {
  const startOnGround = $("config-start-on-ground").checked;
  populateConfig({ ...AIRFRAME_PRESETS[event.target.value], startOnGround });
  $("model-preset").value = "";
};
$("settings-default").onclick = () => {
  const startOnGround = $("config-start-on-ground").checked;
  populateConfig({ ...DEFAULT_CONFIG, startOnGround });
  populateSensorConfig(DEFAULT_SENSOR_CONFIG);
  populateCameraConfig(DEFAULT_CAMERA_CONFIG);
  for (const axis of ["yaw", "pitch", "roll"])
    $("gimbal-rate-" + axis).value = 90;
};
$("settings-form").onsubmit = async (event) => {
  event.preventDefault();
  try {
    const selectedAirframe = $("config-airframe").value;
    const next =
      selectedAirframe === config.airframe
        ? { ...config }
        : { ...AIRFRAME_PRESETS[selectedAirframe] };
    next.airframe = selectedAirframe;
    next.startOnGround = $("config-start-on-ground").checked;
    for (const [key, , , , , factor] of fields)
      next[key] = Number($(`config-${key}`).value) / factor;
    const validated = validateConfig(next);
    const [width, height] = $("camera-resolution").value.split("x").map(Number);
    const nextCamera = validateCameraConfig({
      width,
      height,
      horizontalFovDeg: Number($("camera-fov").value),
      nearM: Number($("camera-near").value),
      farM: Number($("camera-far").value),
    });
    const nextSensors = validateSensorConfig(
      Object.fromEntries(
        Object.keys(DEFAULT_SENSOR_CONFIG).map((key) => [
          key,
          key === "originVerticalDatum"
            ? $(`sensor-config-${key}`).value
            : Number($(`sensor-config-${key}`).value),
        ]),
      ),
    );
    const maxRate = Object.fromEntries(
      ["yaw", "pitch", "roll"].map((axis) => [
        axis,
        Number($("gimbal-rate-" + axis).value) / DEG,
      ]),
    );
    if (
      Object.values(maxRate).some(
        (rate) => !Number.isFinite(rate) || rate < 1 / DEG || rate > 360 / DEG,
      )
    )
      throw new Error("雲台角速度必須介於 1 到 360 °/s。");
    const nextGimbal = {
      mode: latest.gimbal.mode,
      target: { ...latest.gimbal.target },
      maxRate,
    };
    const gimbalChanged = ["yaw", "pitch", "roll"].some(
      (axis) => Math.abs(maxRate[axis] - latest.gimbal.maxRate[axis]) > 1e-8,
    );
    const simulationChanged =
      JSON.stringify(validated) !== JSON.stringify(config) ||
      JSON.stringify(nextSensors) !== JSON.stringify(sensorConfig);
    if (
      simulationChanged &&
      dirty() &&
      !(await confirmAction("套用參數會清除尚未匯出的實驗紀錄。確定繼續？"))
    )
      return;
    if (!simulationChanged) {
      if (gimbalChanged)
        await requestApi({ type: "gimbal", value: nextGimbal });
      applyCameraConfig(nextCamera);
      $("settings-dialog").close();
      toast("相機設定已套用。");
      return;
    }
    config = validated;
    sensorConfig = nextSensors;
    if (!sceneEditor?.getApplied().geography)
      localSensorOrigin = Object.fromEntries(
        ORIGIN_KEYS.map((key) => [key, sensorConfig[key]]),
      );
    applyCameraConfig(nextCamera);
    try {
      localStorage.setItem(storageKey, JSON.stringify(config));
      localStorage.setItem(
        "flight-lab-sensors-v1",
        JSON.stringify({ ...sensorConfig, ...localSensorOrigin }),
      );
    } catch {
      toast("此瀏覽器無法儲存設定，本次實驗仍會使用新參數。");
    }
    $("settings-dialog").close();
    newExperiment();
    sendWorker({ type: "sensorConfig", value: sensorConfig });
    if (gimbalChanged) sendWorker({ type: "gimbal", value: nextGimbal });
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
  clearCameraPreview();
  if (replay && replayPrevious) {
    activeVehicleId = replayPrevious.activeVehicleId;
    config = replayPrevious.config;
    scenario = replayPrevious.scenario;
    replayPrevious = undefined;
  }
  replay = undefined;
  replayControls.setRecording(null);
  document.body.classList.remove("replay");
  setReplayControls(false);
  history = [];
  sampleCount = 0;
  exportedTime = 0;
  storageLimited = false;
  worldTermination = null;
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
  sendWorker({ type: "reset", config, scenario });
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
let launchPending = false;
async function launchFlight(vehicleId = activeVehicleId) {
  if (externalActive) {
    await sendHilCommand("takeoff");
    return;
  }
  if (launchPending || replay || storageLimited) return;
  if (!latest) {
    toast("模擬器尚未準備完成。");
    return;
  }
  launchPending = true;
  try {
    const active = ["runup", "roll", "rotate", "climb"].includes(
      latest.runway.takeoffPhase,
    );
    if (!active) await requestApi({ type: "takeoff", vehicleId });
    document.querySelector('[data-workspace="flight"]').click();
    if (!rendering) $("render-toggle").click();
    document.querySelector('[data-camera="follow"]').click();
    $("viewport").scrollIntoView({ behavior: "smooth", block: "center" });
    if (!running) await requestApi({ type: "play" });
  } catch (error) {
    toast(`無法起飛：${error.message}`);
  } finally {
    launchPending = false;
  }
}
$("play").onclick = () => {
  if (externalActive) {
    if (hilState?.phase === "paused") sendHilCommand("resume");
    else if (
      hilState?.phase === "running" &&
      latest?.runway.weightOnWheels &&
      hilWebControlAllowed(hilState)
    )
      sendHilCommand("takeoff");
    else if (hilState?.phase === "running") sendHilCommand("pause");
    return;
  }
  if (storageLimited && !replay) {
    toast("紀錄已達資料上限，請匯出後重設。");
    return;
  }
  if (replay) {
    if (replayIndex >= replay.samples.length - 1) {
      replayClock = replay.samples[0].time;
      replayIndex = 0;
      projectReplayWorld(true);
    }
    setPlaying(!running);
  } else if (running) sendWorker({ type: "pause" });
  else if (takeoffPlayAction(latest) !== "play") launchFlight();
  else sendWorker({ type: "play" });
};
$("speed").onchange = (event) =>
  sendWorker({ type: "speed", value: Number(event.target.value) });
$("gust").onclick = () => {
  sendWorker({ type: "gust" });
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
  if (!externalActive) sendWorker({ type: "render", value: rendering });
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
function seekReplayFrame(index) {
  if (!replay || externalActive) return;
  if (!Number.isInteger(index) || index < 0 || index >= replay.samples.length)
    throw new Error("無效的重播影格");
  replayIndex = index;
  setPlaying(false);
  clearCameraPreview();
  projectReplayWorld(true);
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
    else sendWorker({ type: "export", kind });
  };
$("import-open").onclick = () => {
  document.querySelector(".export-menu").open = false;
  $("import-file").click();
};
$("import-file").onchange = async (event) => {
  const file = event.target.files?.[0];
  event.target.value = "";
  if (!file) return;
  if (externalActive) {
    toast("請先返回內建模擬，再匯入飛行紀錄。");
    return;
  }
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
    if (externalActive) {
      toast("已進入外部飛控模式，取消匯入紀錄。");
      return;
    }
    sendWorker({ type: "pause" });
    clearCameraPreview();
    if (!replay)
      replayPrevious = {
        activeVehicleId,
        config: structuredClone(config),
        scenario,
      };
    replay = record;
    replayControls.setRecording(record);
    activeVehicleId = record.world.vehicles[0].id;
    document.querySelectorAll("[data-scenario]").forEach((item) => {
      const active = item.dataset.scenario === scenario;
      item.classList.toggle("active", active);
      item.setAttribute("aria-pressed", String(active));
    });
    replayClock = record.samples[0].time;
    replayIndex = 0;
    projectReplayWorld(true);
    sampleCount = record.samples.length;
    timing = {};
    document.body.classList.add("replay");
    setReplayControls(true);
    $("play").disabled = false;
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
  if (replay || externalActive) return;
  if (firstState) {
    firstState = false;
    worldConfig = data.worldConfig;
    activeVehicleId = data.activeVehicleId;
    sendWorker({
      type: "reset",
      config,
      scenario,
      setup: { ...data.setup, sensors: sensorConfig },
    });
    return;
  }
  worldState = data.world;
  worldConfig = data.worldConfig;
  if (!worldConfig.scene.geography)
    for (const item of worldConfig.vehicles)
      vehicleSensorOrigins.set(
        item.id,
        Object.fromEntries(
          ORIGIN_KEYS.map((key) => [key, item.setup.sensors[key]]),
        ),
      );
  if (!rosterDraft || data.message === "reset") {
    rosterDraft = structuredClone(worldConfig);
    renderVehicleEditor();
  } else {
    // Hidden setup follows committed commands while the visible roster edits stay in place.
    for (const draft of rosterDraft.vehicles) {
      const current = worldConfig.vehicles.find((item) => item.id === draft.id);
      if (current) draft.setup = structuredClone(current.setup);
    }
  }
  const vehicleChanged = data.activeVehicleId !== activeVehicleId;
  if (vehicleChanged) {
    vehicleHistories.set(activeVehicleId, history);
    vehicleDrafts.set(activeVehicleId, structuredClone(waypoints));
    activeVehicleId = data.activeVehicleId;
    history = vehicleHistories.get(activeVehicleId) || [];
    waypoints = structuredClone(
      vehicleDrafts.get(activeVehicleId) || data.snapshot.navigation.waypoints,
    );
    clearCameraPreview();
  }
  latest = data.snapshot;
  config = data.config;
  scenario = data.scenario;
  renderVehicleRoster();
  worldTermination = data.termination;
  if (data.sensors) {
    sensors = data.sensors;
    renderSensors();
  }
  if (data.setup?.sensors) {
    sensorConfig = data.setup.sensors;
    if (!data.setup.scene?.geography)
      localSensorOrigin = Object.fromEntries(
        ORIGIN_KEYS.map((key) => [key, sensorConfig[key]]),
      );
  }
  timing = data.timing;
  sampleCount = data.sampleCount;
  if (data.message === "reset") {
    lastAltitudeReference = undefined;
    clearCameraPreview();
    config = data.config;
    scenario = data.scenario;
    $("position-altitude").value = MULTIROTOR_AIRFRAMES.includes(
      config.airframe,
    )
      ? 3
      : Math.min(30, config.geofenceHeight);
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
    if (data.setup) {
      showReplayEvent("scene", data.setup.scene);
      showReplayEvent("mission", data.setup.mission);
    }
    scene?.update(latest, 0);
    scene?.resetView();
  }
  if (data.setup?.scene) renderScene(data.setup.scene, false);
  if (vehicleChanged) renderWaypoints();
  if (data.message === "reset") vehicleHistories.clear();
  for (const frame of data.worldSamples || [])
    for (const item of frame.vehicles) {
      const samples = vehicleHistories.get(item.id) || [];
      samples.push(item.snapshot);
      if (samples.length > 400) samples.shift();
      vehicleHistories.set(item.id, samples);
    }
  history = vehicleHistories.get(activeVehicleId) || [];
  drawMission();
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
    world: data.world,
    activeVehicleId: data.activeVehicleId,
    snapshot: data.snapshot,
    sensors: data.sensors,
    setup: data.setup,
    running: data.running,
    termination: data.termination,
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
  renderInspection();
  renderGimbal();
  if (!latest) return;
  const s = latest;
  $("target-altitude").min =
    !externalActive && sceneEditor?.getApplied().terrain.kind === "dem"
      ? -1000
      : 0;
  $("target-altitude").nextElementSibling.firstElementChild.textContent =
    `${$("target-altitude").min} m`;
  $("position-altitude").min =
    !externalActive &&
    $("position-reference").value === "datum" &&
    sceneEditor?.getApplied().terrain.kind === "dem"
      ? -1000
      : 0;
  $("view-clearance").textContent = s.ground.clearanceM.toFixed(2);
  for (const t of targets) {
    const value = s.target[t.key] * t.factor;
    $(`target-${t.key}`).value = value;
    $(`output-${t.key}`).textContent = `${value.toFixed(1)} ${t.unit}`;
  }
  const centiseconds = Math.round(s.time * 100);
  const minutes = Math.floor(centiseconds / 6000);
  $("sim-time").textContent =
    `${String(minutes).padStart(2, "0")}:${((centiseconds % 6000) / 100).toFixed(2).padStart(5, "0")}`;
  $("altitude-value").textContent = (-s.position[2]).toFixed(2);
  $("altitude-target").textContent = s.target.altitude.toFixed(1);
  $("altitude-target").parentElement.hidden = externalActive;
  $("rmse-value").closest(".metric-card").hidden = externalActive;
  document.querySelector(".target-panel").hidden = externalActive;
  $("flight-mode").parentElement.hidden = externalActive;
  document.querySelector(".legend-target").hidden = externalActive;
  $("model-rate").innerHTML = externalActive
    ? `<span class="dot"></span>${hilState?.clock?.stepUs == null ? "—" : hilState.clock.stepUs / 1000} ms 鎖步`
    : '<span class="dot"></span>200 Hz 模型';
  $("roll-value").textContent = (s.attitude[0] * DEG).toFixed(2);
  $("pitch-value").textContent = (s.attitude[1] * DEG).toFixed(2);
  $("velocity-value").textContent = Math.hypot(...s.velocity).toFixed(2);
  $("distance-value").textContent = Math.hypot(
    ...s.position.slice(0, 2),
  ).toFixed(2);
  $("rmse-value").textContent = (s.metrics.rmse * DEG).toFixed(2);
  updateMotorLayout(s);
  for (let i = 0; i < (s.airframe === "fixedwing" ? 0 : s.motors.length); i++) {
    const command = s.externalInput?.rotorSpeedCommands?.[i];
    $(`motor-${i + 1}`).style.width =
      `${externalActive ? (command ?? 0) * 100 : s.motors[i] * 100}%`;
    $(`pwm-${i + 1}`).textContent = externalActive
      ? command == null
        ? "—"
        : Math.round(command * 100)
      : Math.round(s.pwm[i]);
    $(`motor-unit-${i + 1}`).textContent = externalActive ? "%" : "µs";
  }
  $("motor-note").textContent = externalActive
    ? `${hilControllerName()} 馬達通道映射後的正規化轉速指令；模型映射與參數見介接紀錄。`
    : "3D 機體為示意；固定翼不使用升力旋翼，PWM 為模型換算值。";
  $("age-label").textContent = externalActive ? "馬達資料齡期" : "感測資料齡期";
  $("drop-label").textContent = externalActive ? "協定錯誤" : "模擬丟包";
  $("rtf-value").textContent = replay
    ? "重播"
    : timing.realTimeFactor
      ? `${timing.realTimeFactor.toFixed(2)}×`
      : "—";
  $("jitter-value").textContent = replay
    ? "—"
    : `${(timing.p95LateMs || 0).toFixed(2)} ms`;
  $("age-value").textContent = externalActive
    ? hilState?.stats?.lastActuatorAgeMs == null
      ? "—"
      : `${hilState.stats.lastActuatorAgeMs.toFixed(0)} ms`
    : `${(s.metrics.sampleAge * 1000).toFixed(0)} ms`;
  $("drop-value").textContent = externalActive
    ? String(
        hilState?.controller?.kind === "ardupilot"
          ? (hilState.stats?.invalidActuatorPackets ?? 0) +
              (hilState.stats?.frameMismatches ?? 0)
          : (hilState?.stats?.crcErrors ?? 0),
      )
    : s.metrics.dropped.toLocaleString();
  $("samples-value").textContent = externalActive
    ? "—"
    : sampleCount.toLocaleString();
  const kind = s.airframe;
  const wing = kind === "fixedwing" || kind === "vtol";
  $("airframe-eyebrow").textContent = {
    quadrotor: "QUADROTOR",
    hexarotor: "HEXAROTOR",
    octorotor: "OCTOROTOR",
    fixedwing: "FIXED WING",
    vtol: "LIFT+CRUISE VTOL",
  }[kind];
  $("airframe-status").textContent =
    `機型：${{ quadrotor: "四旋翼", hexarotor: "六旋翼", octorotor: "八旋翼", fixedwing: "固定翼", vtol: "升力加巡航 VTOL" }[kind] || "未知"}`;
  const f = s.flight,
    p = s.propulsion,
    surfaces = s.controlSurfaces;
  const phaseNames = {
    hover: "懸停",
    accelerating: "加速轉換",
    cruise: "巡航",
    decelerating: "減速轉換",
  };
  $("wing-telemetry").textContent = wing
    ? `空速 ${f.airspeedMps.toFixed(1)} m/s · 地速 ${f.groundspeedMps.toFixed(1)} m/s · 攻角 ${(f.alphaRad * DEG).toFixed(1)}° · ${f.stalled ? "翼面失速" : "翼面未失速"} · 巡航槳 ${(p.motor * 100).toFixed(0)}% / ${p.thrustN.toFixed(1)} N · 副翼 / 升降舵 / 方向舵 ${[surfaces.aileron, surfaces.elevator, surfaces.rudder].map((x) => (x * DEG).toFixed(1)).join(" / ")}°${kind === "vtol" ? ` · 轉換 ${phaseNames[f.transitionPhase]} ${(f.transitionProgress * 100).toFixed(0)}%` : ""}`
    : "空速與翼面狀態：不適用";
  $("wing-hint").textContent =
    kind === "fixedwing"
      ? "固定翼預設由 30 m、15 m/s 空中配平開始；可在參數中選擇地面起始，空中可復飛。位置及返航以盤旋保持，降落採固定航向進場，不能懸停。"
      : kind === "vtol"
        ? "VTOL 轉換進度表示控制分配；翼面升力持續計算。"
        : "";
  $("transition-controls").hidden = kind !== "vtol";
  for (const id of ["transition-hover", "transition-cruise"])
    $(id).disabled = !!replay || kind !== "vtol";
  $("fault-motor").disabled = $("fault-apply").disabled =
    !!replay || kind === "fixedwing";
  $("fault-pusher").disabled = !!replay || !wing;
  for (const option of $("training-select").options)
    option.disabled = !isCourseSupported(option.value, kind);
  const runway = s.runway;
  document.querySelector('[data-command="takeoff"]').textContent =
    kind === "fixedwing"
      ? runway.weightOnWheels
        ? "跑道起飛"
        : "復飛"
      : "起飛";
  const takeoffPhases = {
    idle: "待命",
    runup: "啟動檢查",
    roll: "滑跑",
    rotate: "抬輪",
    climb: "爬升",
    complete: "完成",
    aborted: "已中止",
  };
  const abortReasons = {
    pilot: "操作者中止",
    "pusher-failure": "巡航推進故障",
    "runway-end": "跑道終點",
    "runway-excursion": "偏離跑道",
    "insufficient-airspeed": "空速不足",
    safety: "安全保護",
  };
  $("runway-status").hidden = kind !== "fixedwing";
  $("runway-status").textContent =
    !externalActive && !runwayCompatible
      ? "目前地形不具備可用跑道；固定翼須由空中開始。"
      : `跑道：${runway.onRunway ? "跑道內" : "跑道外"} · ${runway.weightOnWheels ? "地面接觸" : "離地"} · ${takeoffPhases[runway.takeoffPhase]} · 抬輪空速門檻 ${runway.rotationSpeedMps.toFixed(1)} m/s · 剩餘 ${runway.distanceRemainingM.toFixed(1)} m · 側向偏離 ${runway.crossTrackM.toFixed(1)} m · 制動指令 ${(runway.brake * 100).toFixed(0)}%${runway.abortReason ? ` · ${abortReasons[runway.abortReason]}` : ""}`;
  $("abort-takeoff").disabled =
    !!replay ||
    kind !== "fixedwing" ||
    !["runup", "roll", "rotate", "climb"].includes(runway.takeoffPhase);
  $("abort-takeoff").textContent = runway.weightOnWheels
    ? "中止起飛"
    : "結束起飛並保持飛行";
  $("abort-hint").textContent =
    !$("abort-takeoff").disabled && !runway.weightOnWheels
      ? "空中結束起飛流程後會保持飛行。"
      : "";
  document.querySelector('[data-scenario="hover"] strong').textContent =
    kind === "fixedwing" ? "巡航基準" : "穩定懸停";
  document.querySelector('[data-scenario="hover"] small').textContent =
    kind === "fixedwing"
      ? config.startOnGround
        ? "固定翼由地面靜止狀態開始"
        : "固定翼由空中配平狀態開始"
      : "維持高度與水平姿態";
  $("flight-mode").querySelector('option[value="position"]').textContent =
    kind === "fixedwing" ? "位置盤旋" : "位置";
  $("target-airspeed").disabled = !!replay || !wing;
  const nav = s.navigation;
  const aglOption = $("position-reference").querySelector(
    'option[value="agl"]',
  );
  aglOption.disabled =
    kind === "fixedwing" ||
    (kind === "vtol" &&
      (s.flight.transition !== "hover" || s.flight.transitionProgress !== 0));
  if (nav.altitudeReference !== lastAltitudeReference) {
    $("position-reference").value = nav.altitudeReference;
    lastAltitudeReference = nav.altitudeReference;
  }
  $("ground-status").textContent =
    externalActive && s.controlSource !== "external"
      ? "等待外部飛控模型真值。"
      : "地表基準高度 " +
        s.ground.heightM.toFixed(1) +
        " m · 離地高度 " +
        s.ground.clearanceM.toFixed(1) +
        " m" +
        (externalActive
          ? " · 模型真值"
          : " · 位置目標 " +
            nav.positionAltitude.toFixed(1) +
            " m（" +
            (nav.altitudeReference === "agl" ? "離地" : "基準面") +
            "）");

  const modeLabels = {
    attitude: "姿態",
    takeoff: "跑道起飛",
    position: kind === "fixedwing" ? "位置盤旋" : "位置",
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
    "ground-impact": "非正常觸地",
    "takeoff-aborted": "起飛已中止",
    rtl: "返航降落",
    crashed: "碰撞",
  };
  if (nav) $("flight-mode").value = nav.mode;
  $("flight-details").textContent = externalActive
    ? `${hilControllerName()} 控制；植物模型真值。飛控估測及圍欄狀態未提供。`
    : nav
      ? `模式：${modeLabels[nav.mode] || "未知"} · 航點 ${Math.min(nav.index + 1, nav.waypoints?.length || 0)}/${nav.waypoints?.length || 0}${nav.completed && nav.mode === "mission" ? "（任務完成）" : ""} · ${nav.fenceBreached ? "超出圍欄" : "圍欄內"} ${reasons[nav.reason] || ""} · ${s.armed ? "已解鎖" : "已上鎖"} · ${stateLabels[s.status] || "未知"}`
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
    ? `故障：${s.faults.linkLost ? "連線中斷" : "連線正常"} · 馬達效率 ${s.faults.motorEfficiency.map((v) => `${Math.round(v * 100)}%`).join(" / ")}${wing ? ` · 巡航推進效率 ${Math.round(s.faults.pusherEfficiency * 100)}%` : ""}`
    : "故障：—";
  const training = s.training;
  renderTutorial();
  $("training-runway-hint").hidden = $("training-select").value !== "runway";
  $("training-status").textContent =
    training && !isCourseSupported(training.course, kind)
      ? "此練習不適用目前機型，請選擇其他練習。"
      : training
        ? `${training.course === "custom" ? "自訂賽道" : TRAINING_COURSES[training.course]?.title || training.course} · ${training.elapsed.toFixed(1)} 秒 · 通過 ${training.passed}/${training.total} · 分數 ${training.score} ${training.complete ? "· 完成" : ""} ${reasons[training.reason] || ""}`
        : "尚未開始";
  if (s.status === "failsafe") {
    $("runtime-status").textContent = "斷訊保護";
    $("view-status").innerHTML =
      '<span class="dot"></span>斷訊保護 · 輸出已停用';
  }
  if (externalActive) setExternalControls(true);
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
      ...(externalActive ? [] : [Math.abs(s.target.roll * DEG)]),
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
    ...(!externalActive
      ? [["#91a5bf", (s) => s.target.roll * DEG, [4, 4]]]
      : []),
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
    const lastIndex = replay.samples.length - 1;
    replayClock = Math.min(
      replay.samples[lastIndex].time,
      replayClock + elapsed * Number($("speed").value),
    );
    const nextIndex = replayFrameAtTime(replay, replayClock);
    if (
      nextIndex !== replayIndex &&
      (nextIndex === lastIndex || now - uiTime > 50)
    ) {
      replayIndex = nextIndex;
      projectReplayWorld(true);
      uiTime = now;
    }
    if (nextIndex === lastIndex) setPlaying(false);
  }
  updatePilot(now);
  updateCameraLive(now);
  if (latest && rendering) {
    const wallDt = running ? elapsed * Number($("speed").value) : 0;
    if (externalActive)
      scene?.update(latest, wallDt, externalMissionIndex(hilState?.mission));
    else if (worldState)
      scene?.updateWorld(worldState, activeVehicleId, wallDt);
  }
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);
document.addEventListener("visibilitychange", () => {
  observeAutomaticCamera();
  if (document.hidden) {
    releasePilot();
    $("camera-live").checked = false;
  }
  if (document.hidden && running) {
    if (externalActive) {
      if (hilState?.groundStation?.owner !== "gcs") sendHilCommand("pause");
      return;
    }
    if (replay) setPlaying(false);
    else sendWorker({ type: "pause" });
    toast("背景分頁已自動暫停。返回後可繼續。");
  }
});

const mapExtent = () => {
  const radius = externalActive && hilState?.mission?.limits
    ? hilState.mission.limits.radius
    : config.geofenceRadius;
  const base = Math.min(
    Math.max(1, radius),
    !externalActive && MULTIROTOR_AIRFRAMES.includes(config.airframe) ? 50 : 500,
  );
  const areaExtent = surveyEditor?.extent() || 0;
  return Math.min(500, Math.max(
    areaExtent ? Math.max(5, areaExtent * 1.15) : base,
    ...waypoints.map((p) => Math.max(Math.abs(p.north), Math.abs(p.east)) * 1.1),
  ));
};
const squareMission = () => {
  const wing = !MULTIROTOR_AIRFRAMES.includes(config.airframe);
  const side = wing
    ? Math.min(60, (config.geofenceRadius / Math.SQRT2) * 0.9)
    : 8;
  const altitude = wing ? Math.min(30, config.geofenceHeight) : 3;
  return [
    [side, 0],
    [side, side],
    [0, side],
    [0, 0],
  ].map(([north, east]) => ({ north, east, altitude, hold: 1 }));
};
let waypoints = squareMission();
const map = $("mission-map");
let missionBusy = false;
const invalidWaypointInputs = new Set();
const missionEditable = () =>
  !replay &&
  (!externalActive ||
    ["ardupilot", "px4"].includes(hilState?.controller?.kind));
let displayedExternalMission = "";
let surveyEditor;
let missionTerrain = createTerrain({ kind: "flat" });
function missionContext() {
  const selected = worldState?.vehicles.find((v) => v.id === activeVehicleId);
  const vehicleConfig = externalActive ? config : selected?.config || config;
  const home = externalActive ? [0, 0] : selected?.home || [0, 0];
  const limits = externalActive ? hilState?.mission?.limits : null;
  return {
    vehicleId: externalActive
      ? `external:${hilState?.controller?.kind || "pending"}`
      : activeVehicleId,
    readonly: !missionEditable(),
    blocked: invalidWaypointInputs.size > 0 || missionBusy,
    home,
    radius: limits?.radius ?? vehicleConfig.geofenceRadius,
    minAltitude: externalActive
      ? (limits?.minAltitude ?? 1)
      : missionTerrain.data.kind === "dem" ? -1000 : 0,
    maxAltitude: externalActive
      ? (limits?.height ?? vehicleConfig.geofenceHeight)
      : Math.min(
          1500,
          missionTerrain.sample(...home).heightM + vehicleConfig.geofenceHeight,
        ),
    cameraConfig,
    cameraAvailable: automaticCameraContext().available,
    cameraBlocked: surveyApplying || cameraBusy || cameraDatasetBusy ||
      !!automaticCamera?.getState().active || !!automaticCamera?.getState().pending,
  };
}
function drawMission() {
  surveyEditor?.refresh();
  const extent = mapExtent();
  $("map-extent").textContent =
    `座標範圍 ±${extent.toFixed(0)} m；實際航點須在圍欄內。`;
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
  surveyEditor?.draw(ctx, w, h, extent);
  ctx.strokeStyle = "#ffbf78";
  ctx.fillStyle = "#ffbf78";
  ctx.lineWidth = 2;
  ctx.beginPath();
  waypoints.forEach((p, i) => {
    const x = ((p.east + extent) / (2 * extent)) * w,
      y = ((extent - p.north) / (2 * extent)) * h;
    if (i) ctx.lineTo(x, y);
    else ctx.moveTo(x, y);
  });
  ctx.stroke();
  waypoints.forEach((p, i) => {
    const x = ((p.east + extent) / (2 * extent)) * w,
      y = ((extent - p.north) / (2 * extent)) * h;
    ctx.beginPath();
    ctx.arc(x, y, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillText(String(i + 1), x + 7, y - 5);
  });
  for (const vehicle of worldState?.vehicles || []) {
    const [north, east] = vehicle.snapshot.position;
    const x = ((east + extent) / (2 * extent)) * w,
      y = ((extent - north) / (2 * extent)) * h;
    ctx.fillStyle = vehicle.id === activeVehicleId ? "#63e0ce" : "#f4a34f";
    ctx.beginPath();
    ctx.arc(x, y, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillText(vehicle.name, x + 6, y + 5);
  }
  if (!externalActive) scene?.setMission(waypoints);
}
function renderWaypoints() {
  const list = $("waypoint-list");
  const limits = missionContext();
  invalidWaypointInputs.clear();
  list.replaceChildren();
  waypoints.forEach((point, index) => {
    const row = document.createElement("div");
    row.className = "waypoint-row";
    const title = document.createElement("strong");
    title.textContent = `${index + 1}`;
    row.append(title);
    for (const [key, label, min, max] of [
      ["north", "北", -500, 500],
      ["east", "東", -500, 500],
      ["altitude", "高度", limits.minAltitude, limits.maxAltitude],
      ["hold", "停留秒", 0, 60],
    ]) {
      const wrapper = document.createElement("label");
      wrapper.textContent = label;
      const input = document.createElement("input");
      input.type = "number";
      input.min = min;
      input.max = max;
      input.step =
        externalActive &&
        key === "hold" &&
        hilState?.mission?.limits?.holdRequiresInteger === true
          ? "1"
          : "0.1";
      input.value = point[key];
      input.disabled = !missionEditable();
      input.oninput = () => {
        const value = Number(input.value);
        if (
          input.value.trim() !== "" &&
          Number.isFinite(value) &&
          value >= Number(input.min) &&
          value <= Number(input.max) &&
          !(
            externalActive &&
            key === "hold" &&
            hilState?.mission?.limits?.holdRequiresInteger === true &&
            !Number.isInteger(value)
          )
        ) {
          point[key] = value;
          invalidWaypointInputs.delete(input);
          drawMission();
        } else invalidWaypointInputs.add(input);
        updateMissionControls();
      };
      input.dataset.missionField = key;
      input.onblur = input.onchange = () => {
        if (invalidWaypointInputs.delete(input)) {
          input.value = point[key];
          toast(`${label}超出範圍，已還原。`);
        }
        updateMissionControls();
      };
      wrapper.append(input);
      row.append(wrapper);
    }
    const remove = document.createElement("button");
    remove.type = "button";
    remove.textContent = "移除";
    remove.disabled = !missionEditable();
    remove.onclick = () => {
      waypoints.splice(index, 1);
      renderWaypoints();
    };
    row.append(remove);
    list.append(row);
  });
  drawMission();
  updateMissionControls();
}
surveyEditor = createSurveyEditor({
  getContext: missionContext,
  onChange: drawMission,
  onUse: useSurveyPlan,
});
async function useSurveyPlan(points, plan) {
  if (!plan) {
    waypoints = points;
    renderWaypoints();
    return;
  }
  const context = missionContext();
  if (!context.cameraAvailable || context.cameraBlocked || surveyApplying || !latest?.gimbal)
    throw new Error("請先停止拍攝、等待相機完成並回到內建模擬。");
  if (JSON.stringify(plan.cameraConfig) !== JSON.stringify(cameraConfig))
    throw new Error("相機設定已變更，請重新預覽測繪航線。");
  const remaining = Math.min(
    CAMERA_DATASET_MAX_FRAMES - cameraDataset.length,
    Math.floor((CAMERA_DATASET_MAX_BYTES - cameraDatasetBytes) /
      (cameraConfig.width * cameraConfig.height * 9)),
  );
  if (remaining < 1) throw new Error("相機資料集已滿，請先匯出並清除。");
  const vehicleId = activeVehicleId, epoch = cameraEpoch;
  surveyApplying = true;
  updateCameraStatus();
  updateMissionControls();
  renderGimbal();
  try {
    await requestApi({
      type: "gimbal", vehicleId,
      value: {
        mode: "ned",
        target: { yaw: plan.angleDeg / DEG, pitch: Math.PI / 2, roll: 0 },
        maxRate: { ...latest.gimbal.maxRate },
      },
    });
    if (epoch !== cameraEpoch || vehicleId !== activeVehicleId || replay || externalActive)
      throw new Error("載具或實驗已切換，請在目前載具重新預覽並套用。");
    automaticCameraControls.setDistance(plan.triggerDistanceM, remaining);
    waypoints = points;
    renderWaypoints();
  } finally {
    surveyApplying = false;
    updateCameraStatus();
    updateMissionControls();
    renderGimbal();
  }
}
map.onclick = (event) => {
  if (!missionEditable() || invalidWaypointInputs.size || missionBusy || surveyApplying) return;
  const rect = map.getBoundingClientRect();
  const point = {
    north:
      Math.round(
        (0.5 - (event.clientY - rect.top) / rect.height) * 2 * mapExtent() * 10,
      ) / 10,
    east:
      Math.round(
        ((event.clientX - rect.left) / rect.width - 0.5) * 2 * mapExtent() * 10,
      ) / 10,
  };
  if (surveyEditor.mapClick(point)) return;
  if (waypoints.length >= 50) return;
  waypoints.push({
    ...point,
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
$("mission-export").onclick = () => {
  if (invalidWaypointInputs.size) return toast("請先修正航點數值。");
  download(JSON.stringify(waypoints, null, 2), "json", "application/json");
};
function updateMissionControls() {
  if (surveyEditor?.refresh()) drawMission();
  const editable = missionEditable() && !surveyApplying;
  const limits = missionContext();
  document
    .querySelectorAll("#waypoint-list input, #waypoint-list button")
    .forEach((element) => {
      if (element.tagName === "INPUT") {
        const field = element.dataset.missionField;
        if (field === "north" || field === "east") {
          element.min = -500;
          element.max = 500;
        } else if (field === "altitude") {
          element.min = limits.minAltitude;
          element.max = limits.maxAltitude;
        }
        if (field === "hold")
          element.step =
            externalActive &&
            hilState?.mission?.limits?.holdRequiresInteger === true
              ? "1"
              : "0.1";
      }
      element.disabled =
        !editable ||
        (element.tagName === "BUTTON" && invalidWaypointInputs.size > 0);
    });
  $("waypoint-add").disabled = !editable || invalidWaypointInputs.size > 0;
  $("waypoint-square").disabled = !editable || invalidWaypointInputs.size > 0;
  $("mission-export").disabled = invalidWaypointInputs.size > 0;
  const controls = externalMissionControls(
    hilState,
    waypoints,
    invalidWaypointInputs.size > 0,
    missionBusy,
  );
  $("mission-apply").disabled = externalActive
    ? !controls.upload
    : !editable || invalidWaypointInputs.size > 0;
  $("mission-start").disabled = externalActive
    ? !controls.start
    : !editable || invalidWaypointInputs.size > 0;
  $("mission-apply").textContent = externalActive ? "上傳任務" : "套用任務";
  $("mission-help").textContent = externalActive
    ? hilState?.groundStation?.owner === "gcs"
      ? "地面站管理飛行與任務；這裡的草稿僅供編輯與匯出，不能上傳或開始。"
      : controls.editable
        ? `草稿與飛控已確認航線分開；本機飛控準備完成且地面未解鎖後可上傳。上傳後須確認路線一致才能開始。${hilState?.mission?.limits?.holdRequiresInteger === true ? "停留秒數須為整數。" : ""}`
        : "等待支援的本機飛控任務狀態。"
    : config.airframe === "fixedwing"
      ? "固定翼請先完成跑道起飛，再開始航點任務；套用任務會切換至任務模式。"
      : "開始任務會由地面起飛並啟動模擬；套用任務只切換任務模式。";
  const m = hilState?.mission;
  const phases = {
    empty: "尚無任務",
    uploading: "上傳中",
    verifying: "核對中",
    ready: "已核對",
    running: "執行中",
    completed: "已完成",
    interrupted: "已中止",
    error: "錯誤",
  };
  const missionStatus =
    externalActive && hilState?.groundStation?.owner === "gcs"
      ? "地面站管理任務；網頁不宣稱已核對航線或進度。"
      : externalActive && controls.editable && m
        ? `${phases[m.phase] || "等待狀態"}${m.homeReady === true ? "" : " · 等待飛控定位及家點確認"} · 目前航點 ${externalMissionIndex(m) === null ? "—" : externalMissionIndex(m) + 1} / ${m.points?.length ?? 0} · 飛控回報到達 ${externalMissionReachedCount(m) ?? "—"} 點${m.error ? ` · ${m.error}` : ""}`
        : "";
  $("mission-status").textContent = missionStatus;
  $("flight-mission-status").textContent = missionStatus
    ? `飛控任務：${missionStatus}`
    : "";
  $("flight-mission-status").hidden = !missionStatus;
}
async function applyMission(start) {
  if (surveyApplying) return toast("測繪拍攝設定正在套用，請稍候。");
  if (invalidWaypointInputs.size) return toast("請先修正航點數值。");
  if (externalActive) {
    const controls = externalMissionControls(
      hilState,
      waypoints,
      false,
      missionBusy,
    );
    if (!(start ? controls.start : controls.upload))
      return toast("任務尚未符合操作條件。");
    missionBusy = true;
    updateMissionControls();
    try {
      await hilClient.request(
        start ? "command" : "mission",
        start ? "mission" : structuredClone(waypoints),
      );
      toast(
        start
          ? "任務開始命令已送出，請觀察飛控回報。"
          : "任務上傳請求完成，請確認飛控核對狀態。",
      );
    } catch (error) {
      toast(`任務操作失敗：${error.message}`);
    } finally {
      missionBusy = false;
      updateMissionControls();
    }
    return;
  }
  if (!waypoints.length) return toast("請先新增航點。");
  const vehicleId = activeVehicleId;
  const onGround = latest?.runway.weightOnWheels;
  try {
    if (start && onGround && config.airframe === "fixedwing")
      throw new Error("請先完成跑道起飛，再開始航點任務。");
    await requestApi({
      type: "mission",
      vehicleId,
      value: structuredClone(waypoints),
    });
    if (start && onGround) await requestApi({ type: "takeoff", vehicleId });
    await requestApi({ type: "mode", vehicleId, value: "mission" });
    $("flight-mode").value = "mission";
    if (start) {
      focusFlightView();
      await requestApi({ type: "play" });
    }
  } catch (error) {
    toast(`無法${start ? "開始" : "套用"}任務：${error.message}`);
  }
}
$("mission-apply").onclick = () => applyMission(false);
$("mission-start").onclick = () => applyMission(true);
let sceneWriteBusy = false;
async function applyScene(value, restart = false) {
  if (sceneWriteBusy) throw new Error("場景正在套用，請稍候。");
  const vehicleId = activeVehicleId;
  const valid = validateScene(value);
  sceneWriteBusy = true;
  try {
    const current = await requestApi({ type: "inspect", vehicleId });
    const geographyChanged =
      JSON.stringify(valid.geography) !==
      JSON.stringify(current.setup.scene.geography);
    const reset =
      restart ||
      geographyChanged ||
      JSON.stringify(valid.terrain) !==
        JSON.stringify(current.setup.scene.terrain);
    const losesRecording = reset && dirty();
    const losesInspection = current.world.vehicles.some(
      (item) => item.snapshot.inspection?.attempts > 0,
    );
    if (losesRecording || losesInspection) {
      const message =
        losesRecording && losesInspection
          ? "變更共享地形或地理場景會重設所有載具、清除尚未匯出的紀錄及各機巡檢進度；請先匯出紀錄與巡檢報告。確定繼續？"
          : losesRecording
            ? "變更共享地形或地理場景會重設所有載具並清除尚未匯出的紀錄。確定繼續？"
            : "套用場景會清除各機巡檢進度，請先匯出巡檢報告。確定繼續？";
      if (!(await confirmAction(message)))
        throw new Error("已取消套用，原實驗已保留。");
    }
    const origin = valid.geography?.origin;
    const nextSensors = geographyChanged
      ? {
          ...current.setup.sensors,
          ...(origin
            ? {
                originLatitudeDeg: origin.latitudeDeg,
                originLongitudeDeg: origin.longitudeDeg,
                originAltitudeM: origin.altitudeM,
                originVerticalDatum: origin.verticalDatum,
              }
            : vehicleSensorOrigins.get(vehicleId) || localSensorOrigin),
        }
      : current.setup.sensors;
    const state = await requestApi(
      reset
        ? {
            type: "reset",
            vehicleId,
            config: current.config,
            scenario: current.scenario,
            setup: {
              ...current.setup,
              mode: "attitude",
              mission: [],
              sensors: nextSensors,
              scene: valid,
            },
          }
        : { type: "scene", value: valid },
    );
    renderScene(state.setup.scene, false);
    return state.setup.scene;
  } finally {
    sceneWriteBusy = false;
  }
}
async function applySceneSetting(change) {
  try {
    const value = sceneEditor.getApplied();
    change(value);
    await applyScene(value);
  } catch (error) {
    showSceneControls(sceneEditor.getApplied());
    toast(`無法套用場景：${error.message}`);
  }
}
for (const id of [
  "environment-preset",
  "environment-time",
  "environment-fog",
  "environment-rain",
])
  $(id).onchange = () =>
    applySceneSetting((value) => {
      value.environment = {
        preset: $("environment-preset").value,
        timeOfDay: Number($("environment-time").value),
        fog: Number($("environment-fog").value),
        rain: Number($("environment-rain").value),
      };
    });
$("training-select").onchange = () => {
  renderTutorial();
  if ($("training-select").value !== "tutorial")
    sendWorker({ type: "training", value: $("training-select").value });
};
let tutorialBusy = false;
const tutorialFailures = {
  "procedure-order": "步驟順序不符",
  failsafe: "保護模式",
  unsafe: "飛行狀態不安全",
  crashed: "碰撞停機",
};
function renderTutorial() {
  const t = latest?.training;
  const isTutorial =
    t?.course === "tutorial" &&
    (!externalActive || latest.controlSource === "external");
  const readonly = !!replay || externalActive;
  $("tutorial-start-controls").hidden =
    $("training-select").value !== "tutorial";
  $("tutorial-start").disabled =
    readonly || tutorialBusy || config.airframe === "fixedwing";
  $("tutorial-panel").hidden = !isTutorial;
  if (!isTutorial) return;
  const step = TUTORIAL_STEPS[t.passed];
  const failure = tutorialFailures[t.reason];
  $("tutorial-title").textContent = failure
    ? "教學已停止"
    : t.complete
      ? "基礎飛行教學已完成"
      : step.title;
  $("tutorial-instruction").textContent = failure
    ? failure + "；請重新開始教學。"
    : t.complete
      ? "已完成所有步驟。"
      : step.instruction;
  $("tutorial-progress").textContent =
    "已完成 " +
    t.passed +
    "/" +
    t.total +
    " 步" +
    (step && !failure
      ? " · 保持 " +
        Math.min(t.holdSeconds, step.holdSeconds).toFixed(1) +
        "/" +
        step.holdSeconds +
        " 秒"
      : "") +
    (readonly ? " · 唯讀" : "");
  $("tutorial-action").textContent = failure
    ? "教學已停止"
    : t.complete
      ? "教學完成"
      : [
          running ? "檢查中" : "繼續檢查",
          "解鎖",
          "起飛",
          running ? "保持懸停" : "繼續懸停",
          "飛往前方",
          "返回家點",
          "降落",
        ][t.passed] || "執行這一步";
  $("tutorial-action").disabled =
    readonly ||
    tutorialBusy ||
    t.complete ||
    !!failure ||
    ((t.passed === 0 || t.passed === 3) && running);
}
$("tutorial-start").onclick = async () => {
  if (
    replay ||
    externalActive ||
    tutorialBusy ||
    config.airframe === "fixedwing"
  )
    return;
  if (config.geofenceRadius < 10 || config.geofenceHeight < 3)
    return toast(
      "教學需要至少 10 m 圍欄半徑及 3 m 高度，以保留飛行修正空間，請先調整管理參數。",
    );
  const vehicleId = activeVehicleId;
  if (
    dirty() &&
    !(await confirmAction("開始教學會清除尚未匯出的實驗紀錄。確定繼續？"))
  )
    return;
  tutorialBusy = true;
  renderTutorial();
  try {
    const current = await requestApi({ type: "inspect", vehicleId });
    const setup = {
      ...current.setup,
      mode: "attitude",
      mission: [],
      scene: current.setup.scene,
      training: "tutorial",
    };
    await requestApi({
      type: "reset",
      vehicleId,
      config: { ...current.config, startOnGround: true },
      scenario: "hover",
      setup,
    });
    focusFlightView();
    await requestApi({ type: "play" });
  } catch (error) {
    toast("無法開始教學：" + error.message);
  } finally {
    tutorialBusy = false;
    renderTutorial();
  }
};
$("tutorial-action").onclick = async () => {
  const t = latest?.training;
  if (
    replay ||
    externalActive ||
    tutorialBusy ||
    t?.course !== "tutorial" ||
    t.complete ||
    tutorialFailures[t.reason]
  )
    return;
  tutorialBusy = true;
  renderTutorial();
  const vehicleId = activeVehicleId;
  try {
    if (t.passed === 1) await requestApi({ type: "arm", vehicleId });
    else if (t.passed === 2) {
      await launchFlight(vehicleId);
      return;
    } else if (t.passed === 4 || t.passed === 5) {
      const home = worldState.vehicles.find(
        (item) => item.id === vehicleId,
      ).home;
      const altitude =
        createTerrain(worldState.scene.terrain).sample(...home).heightM + 2;
      await requestApi({ type: "mode", vehicleId, value: "position" });
      await requestApi({
        type: "position",
        vehicleId,
        value: {
          north: home[0] + (t.passed === 4 ? 8 : 0),
          east: home[1],
          altitude,
          altitudeReference: "datum",
        },
      });
    } else if (t.passed === 6)
      await requestApi({ type: "mode", vehicleId, value: "land" });
    if (!running) await requestApi({ type: "play" });
  } catch (error) {
    toast("無法執行這一步：" + error.message);
  } finally {
    tutorialBusy = false;
    renderTutorial();
  }
};
$("flight-mode").onchange = (event) =>
  sendWorker({ type: "mode", value: event.target.value });
document.querySelectorAll("[data-command]").forEach(
  (button) =>
    (button.onclick = () => {
      if (externalActive) return sendHilCommand(button.dataset.command);
      return button.dataset.command === "takeoff"
        ? launchFlight()
        : sendWorker({ type: button.dataset.command });
    }),
);
document.querySelectorAll("[data-mode]").forEach(
  (button) =>
    (button.onclick = () => {
      $("flight-mode").value = button.dataset.mode;
      sendWorker({ type: "mode", value: button.dataset.mode });
    }),
);
$("position-apply").onclick = () => {
  const value = {
    north: Number($("position-north").value),
    east: Number($("position-east").value),
    altitude: Number($("position-altitude").value),
    altitudeReference: $("position-reference").value,
  };
  sendWorker({ type: "mode", value: "position" });
  sendWorker({ type: "position", value });
  $("flight-mode").value = "position";
};
$("fault-apply").onclick = () =>
  sendWorker({
    type: "fault",
    fault: "motor",
    value: {
      index: Number($("fault-motor").value),
      efficiency: Number($("fault-efficiency").value),
    },
  });
$("abort-takeoff").onclick = () => sendWorker({ type: "abortTakeoff" });
$("fault-pusher").onclick = () =>
  sendWorker({
    type: "fault",
    fault: "pusher",
    value: { efficiency: Number($("fault-efficiency").value) },
  });
for (const mode of ["hover", "cruise"])
  $(`transition-${mode}`).onclick = () =>
    sendWorker({ type: "transition", value: mode });
$("fault-link").onclick = () =>
  sendWorker({ type: "fault", fault: "link", value: { lost: true } });
$("fault-clear").onclick = () => sendWorker({ type: "clearFaults" });
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
    for (const name of [
      "mission",
      "training",
      "scene",
      "inspection",
      "sensors",
      "hil",
      "catalog",
    ])
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
  if (pilotWasActive && !replay && !externalActive)
    sendWorker({
      type: "targets",
      value: { roll: 0, pitch: 0, yawRate: 0 },
    });
  pilotWasActive = false;
}
$("pilot-enable").onchange = (event) => {
  if (event.target.checked && !replay) {
    event.target.blur();
    $("flight-mode").value = "attitude";
    sendWorker({ type: "mode", value: "attitude" });
  } else releasePilot();
};
window.addEventListener("keydown", (event) => {
  if (
    !$("pilot-enable").checked ||
    replay ||
    externalActive ||
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
    externalActive ||
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
      sceneEditor?.getApplied().terrain.kind === "dem" ? -1000 : 0,
      Math.min(100, latest.target.altitude + input.climb * 2 * dt),
    ),
  };
  sendWorker({ type: "targets", value });
  pilotWasActive = true;
  for (const t of targets) {
    if (!Object.hasOwn(value, t.key)) continue;
    const display = value[t.key] * t.factor;
    $(`target-${t.key}`).value = display;
    $(`output-${t.key}`).textContent = `${display.toFixed(1)} ${t.unit}`;
  }
}
function setReplayControls(disabled) {
  demEditor?.setReadonly(disabled || externalActive);
  renderGimbal();
  renderTutorial();
  document
    .querySelectorAll(
      "#mission-workspace button:not(#mission-export), #training-workspace input, #training-workspace select, #sensor-fault-channel, #sensor-fault-on, #sensor-fault-off, #sensor-export-json, #camera-capture, #camera-add, #camera-live, #flight-mode, [data-command], [data-mode], #position-north, #position-east, #position-altitude, #position-reference, #position-apply, #fault-apply, #fault-link, #fault-clear, #abort-takeoff, #fault-pusher, #transition-hover, #transition-cruise, #pilot-enable",
    )
    .forEach((element) => {
      element.disabled = disabled;
    });
  if (disabled) {
    $("pilot-enable").checked = false;
    releasePilot();
  }
  if (typeof waypoints !== "undefined") renderWaypoints();
  updateLidarExportButtons();
}
renderWaypoints();
renderCatalog();

function showSceneControls(value) {
  $("environment-preset").value = value.environment.preset;
  $("environment-time").value = value.environment.timeOfDay;
  $("environment-fog").value = value.environment.fog;
  $("environment-rain").value = value.environment.rain;
  $("training-select").value = latest?.training?.course || "free";
  $("training-runway-hint").hidden = $("training-select").value !== "runway";
}
function renderScene(value, viewOnly) {
  if (!value) return;
  const key = `${activeVehicleId}:${config.airframe}:${latest?.training?.course}:${JSON.stringify(value)}`;
  if (key === lastRenderedSceneKey && viewOnly === lastRenderedSceneReadonly)
    return;
  lastRenderedSceneKey = key;
  lastRenderedSceneReadonly = viewOnly;
  missionTerrain = createTerrain(value.terrain);
  runwayCompatible = missionTerrain.runwayCompatible;
  sceneEditor?.setScene(value, viewOnly);
  demEditor?.setScene(value);
  demEditor?.setReadonly(viewOnly || externalActive || !!replay);
  showSceneControls(value);
  scene?.setTerrain(value.terrain);
  scene?.setEnvironment(value.environment);
  scene?.setObstacles(value.obstacles, value.inspectionTargets);
  const selected = worldState?.vehicles.find(
    (item) => item.id === activeVehicleId,
  );
  const home = selected?.home || [0, 0];
  scene?.setGates(
    resolveTrainingGates(
      value,
      config.airframe,
      latest?.training?.course || "free",
      home,
      missionTerrain.sample(...home).heightM,
    ),
  );
}
function showReplayEvent(type, value) {
  if (type === "scene") renderScene(value, !!replay || externalActive);
  else if (type === "mission") {
    waypoints = structuredClone(value);
    renderWaypoints();
  } else if (type === "sensorConfig" || type === "sensors")
    sensorConfig = value;
}
function projectReplayWorld(resetHistory = false) {
  worldState = recordingWorldAt(replay, replayIndex);
  worldTermination =
    replayIndex === replay.samples.length - 1 ? replay.termination : null;
  const selected =
    worldState.vehicles.find((item) => item.id === activeVehicleId) ||
    worldState.vehicles[0];
  activeVehicleId = selected.id;
  latest = selected.snapshot;
  sensors = selected.sensors;
  config = selected.config;
  scenario = selected.scenario;
  sensorConfig = selected.setup.sensors;
  $("position-north").value = latest.navigation.targetPosition[0];
  $("position-east").value = latest.navigation.targetPosition[1];
  $("position-altitude").value = latest.navigation.positionAltitude;
  for (const target of targets) {
    const value = latest.target[target.key] * target.factor;
    $(`target-${target.key}`).value = value;
    $(`output-${target.key}`).textContent =
      `${value.toFixed(1)} ${target.unit}`;
  }
  waypoints = structuredClone(selected.setup.mission);
  if (resetHistory)
    history = replay.samples
      .slice(Math.max(0, replayIndex - 399), replayIndex + 1)
      .map(
        (frame) =>
          frame.vehicles.find((item) => item.id === activeVehicleId).snapshot,
      );
  renderScene(worldState.scene, true);
  renderWaypoints();
  renderVehicleRoster();
  renderSensors();
  updateTelemetry();
  setPlaying(running);
  replayControls.update(replayIndex);
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
  originAltitudeM: "原點高程 (m)",
  originVerticalDatum: "高程垂直基準",
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
  lidarRays: "LiDAR 每層方位角射線數",
  lidarChannels: "LiDAR 垂直層數",
  lidarVerticalMinDeg: "LiDAR 最低仰角 (°)",
  lidarVerticalMaxDeg: "LiDAR 最高仰角 (°)",
  lidarMinM: "LiDAR 最小值 (m)",
  lidarMaxM: "LiDAR 最大值 (m)",
  lidarNoiseM: "LiDAR 雜訊 (m)",
  opticalFlowNoiseRadS: "光流雜訊 (rad/s)",
  radarHz: "雷達更新率 (Hz)",
  radarAzimuthRays: "雷達水平方向射線數",
  radarElevationRays: "雷達垂直方向射線數",
  radarHorizontalFovDeg: "雷達水平視角 (°)",
  radarVerticalFovDeg: "雷達垂直視角 (°)",
  radarMinM: "雷達最小距離 (m)",
  radarMaxM: "雷達最大距離 (m)",
  radarNoiseM: "雷達距離雜訊 (m)",
  radarVelocityNoiseMps: "雷達距離變化速度雜訊 (m/s)",
  proximityHz: "六向近接更新率 (Hz)",
  proximityMinM: "六向近接最小距離 (m)",
  proximityMaxM: "六向近接最大距離 (m)",
  proximityNoiseM: "六向近接距離雜訊 (m)",
  proximityWarningM: "六向近接警示距離 (m)",
};
const sensorActions = $("sensor-fault-on").parentElement;
for (const [kind, label] of [
  ["json", "匯出目前感測狀態 JSON"],
  ["csv", "匯出目前 LiDAR 點雲 CSV"],
  ["pcd", "匯出目前 LiDAR 點雲 PCD"],
]) {
  const button = document.createElement("button");
  button.type = "button";
  button.disabled = kind !== "json";
  button.id = `sensor-export-${kind}`;
  button.textContent = label;
  button.onclick = () => {
    if (kind === "json") return sendWorker({ type: "sensorExport", kind });
    try {
      const output = exportVehiclePointCloud(kind, sensors.lidar, {
        sourceVehicleId: activeVehicleId,
        worldTick: externalActive ? latest.metrics.steps : worldState.tick,
      });
      download(
        output,
        kind,
        kind === "csv" ? "text/csv;charset=utf-8" : "text/plain;charset=utf-8",
        `flight-lab-lidar-${activeVehicleId}.${kind}`,
      );
    } catch (error) {
      toast(`無法匯出點雲：${error.message}`);
    }
  };
  sensorActions.append(button);
}
function updateLidarExportButtons() {
  for (const kind of ["csv", "pcd"])
    $(`sensor-export-${kind}`).disabled = !Array.isArray(
      sensors?.lidar?.positionNed,
    );
}

const sensorSettings = $("sensor-settings");
for (const key of Object.keys(DEFAULT_SENSOR_CONFIG)) {
  const field = document.createElement("div");
  field.className = "settings-field";
  const label = document.createElement("label");
  label.htmlFor = `sensor-config-${key}`;
  label.textContent = sensorLabels[key] || key;
  const input = document.createElement(
    key === "originVerticalDatum" ? "select" : "input",
  );
  if (key === "originVerticalDatum") {
    for (const [value, text] of [
      ["unspecified", "未指定（模型高程）"],
      ["ellipsoidal", "橢球高"],
      ["orthometric", "正高／海拔"],
    ])
      input.add(new Option(text, value));
  } else {
    input.type = "number";
    input.step = "any";
  }
  input.required = true;
  input.id = label.htmlFor;
  if (key === "lidarChannels") {
    input.min = "1";
    input.max = "32";
    input.step = "1";
  }
  if (key === "lidarRays") {
    input.min = "12";
    input.max = "180";
    input.step = "1";
  }
  if (["lidarVerticalMinDeg", "lidarVerticalMaxDeg"].includes(key)) {
    input.min = "-90";
    input.max = "90";
  }
  if (Object.hasOwn(PROXIMITY_BOUNDS, key)) {
    input.min = String(PROXIMITY_BOUNDS[key][0]);
    input.max = String(PROXIMITY_BOUNDS[key][1]);
  }
  field.append(label, input);
  sensorSettings.append(field);
}
$("lidar-preset-planar").onclick = () => {
  $("sensor-config-lidarChannels").value = 1;
  $("sensor-config-lidarVerticalMinDeg").value = 0;
  $("sensor-config-lidarVerticalMaxDeg").value = 0;
};
$("lidar-preset-3d").onclick = () => {
  $("sensor-config-lidarChannels").value = 8;
  $("sensor-config-lidarVerticalMinDeg").value = -30;
  $("sensor-config-lidarVerticalMaxDeg").value = 10;
};
function populateSensorConfig(values) {
  const origin = sceneEditor?.getApplied().geography?.origin;
  const activeOrigin = origin
    ? {
        originLatitudeDeg: origin.latitudeDeg,
        originLongitudeDeg: origin.longitudeDeg,
        originAltitudeM: origin.altitudeM,
        originVerticalDatum: origin.verticalDatum,
      }
    : null;
  for (const key of Object.keys(DEFAULT_SENSOR_CONFIG)) {
    const input = $(`sensor-config-${key}`);
    input.value =
      activeOrigin && ORIGIN_KEYS.includes(key)
        ? activeOrigin[key]
        : values[key];
    input.disabled = !!activeOrigin && ORIGIN_KEYS.includes(key);
  }
}
const sensorNames = {
  gps: "GPS",
  barometer: "氣壓計",
  magnetometer: "磁力計",
  rangefinder: "向下測距",
  lidar: "LiDAR",
  radar: "雷達",
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
const proximityPanel = createProximityPanel({
  exportData: (format) => {
    if (externalActive || !sensors?.proximity || !worldState)
      throw new Error("目前未提供近接資料。");
    const text = exportProximityData(format, sensors.proximity, {
      sourceVehicleId: activeVehicleId, worldTick: worldState.tick,
    });
    download(text, format, format === "json" ? "application/json" : "text/csv;charset=utf-8",
      `flight-lab-proximity-${activeVehicleId}.${format}`);
  },
});
function renderSensors() {
  if (!sensors) return;
  const readings = $("sensor-readings");
  readings.replaceChildren();
  const detail = {
    gps: (s) => [
      `緯度 / 經度 ${valueText(s.latitudeDeg, 6)} / ${valueText(s.longitudeDeg, 6)} °`,
      `高程 ${valueText(s.altitudeM)} m（${{ unspecified: "垂直基準未指定", ellipsoidal: "橢球高", orthometric: "正高／海拔" }[s.verticalDatum] || "未指定"}）`,
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
      `${s.channelCount} 層 · 每層 ${s.azimuthCount} 射線`,
      `仰角 ${valueText(s.elevationStartRad * DEG, 1)}° 至 ${valueText((s.elevationStartRad + (s.channelCount - 1) * s.elevationStepRad) * DEG, 1)}°`,
      `量測範圍 ${valueText(s.minM)}–${valueText(s.maxM)} m`,
    ],
    radar: (s) => [
      `回波 ${s.detections.length} 個`,
      `量測範圍 ${valueText(s.minM)}–${valueText(s.maxM)} m`,
    ],
    opticalFlow: (s) => [
      `光流 ${vectorText(s.flowRadS, 3)} rad/s`,
      `地面距離 ${valueText(s.groundDistanceM)} m`,
    ],
  };
  for (const [key, name] of Object.entries(sensorNames)) {
    const item = sensors[key];
    if (key === "radar" && externalActive) {
      const card = document.createElement("article");
      card.className = "sensor-card";
      const title = document.createElement("h3");
      title.textContent = name;
      const state = document.createElement("p");
      state.textContent = "外部飛控模式未提供雷達量測";
      card.append(title, state);
      readings.append(card);
      continue;
    }
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
  updateLidarExportButtons();
  drawLidar(sensors.lidar);
  renderRadar(sensors.radar);
  proximityPanel.render(sensors.proximity, {
    sourceVehicleId: activeVehicleId, worldTick: worldState?.tick,
    time: sensors.time, available: !externalActive && !!worldState,
    maxAgeS: Math.max(0.25, 2 / sensorConfig.proximityHz + sensorConfig.latency),
  });
  scene?.setSensorScan($("sensor-scan-visible").checked ? sensors.lidar : null);
}
function renderRadar(reading) {
  const canvas = $("radar-map"),
    ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const buttons = [$("radar-export-json"), $("radar-export-csv")];
  buttons.forEach((button) => {
    button.disabled = externalActive || !reading;
  });
  if (externalActive || !reading) {
    $("radar-status").textContent = externalActive
      ? "外部飛控模式未提供雷達量測"
      : "尚無雷達資料";
    $("radar-pose").textContent = externalActive
      ? "外部飛控模式未提供雷達量測"
      : "尚無量測姿態";
    $("radar-count").textContent = "";
    $("radar-returns").replaceChildren();
    return;
  }
  const age = Number.isFinite(reading.time)
    ? Math.max(0, sensors.time - reading.time)
    : null;
  $("radar-status").textContent =
    `來源 ${activeVehicleId} · ${reading.valid ? "有效" : `無效（${reasonsSensor[reading.reason] || "無資料"}）`} · 量測 ${valueText(reading.time, 3)} s · 齡期 ${valueText(age, 3)} s${sensors.faults?.radar ? " · 故障啟用" : ""}`;
  $("radar-pose").textContent =
    `NED 位置 ${vectorText(reading.positionNed)} m · 四元數 wxyz ${vectorText(reading.quaternion, 3)} · NED 速度 ${vectorText(reading.velocityNedMps)} m/s`;
  const points = reading.detections;
  const visibleRate = (value) => (Math.abs(value) < 0.005 ? 0 : value);
  $("radar-count").textContent =
    points.length > 100
      ? `共 ${points.length} 個回波，表格顯示最近 100 個；匯出包含全部。`
      : `共 ${points.length} 個回波。`;
  const rows = [...points]
    .sort((a, b) => a.rangeM - b.rangeM || a.beam - b.beam)
    .slice(0, 100)
    .map((point) => {
      const row = document.createElement("tr");
      for (const value of [
        point.rangeM,
        point.azimuthRad * DEG,
        point.elevationRad * DEG,
        visibleRate(point.rangeRateMps),
      ]) {
        const cell = document.createElement("td");
        cell.textContent = valueText(value);
        row.append(cell);
      }
      return row;
    });
  $("radar-returns").replaceChildren(...rows);
  const x0 = canvas.width / 2,
    y0 = canvas.height - 16;
  const half = reading.horizontalFovRad / 2;
  const radius = Math.min(
    canvas.height - 32,
    (x0 - 15) / Math.max(Math.sin(half), 0.01),
  );
  ctx.strokeStyle = "#537385";
  ctx.lineWidth = 1;
  for (const fraction of [0.25, 0.5, 0.75, 1]) {
    ctx.beginPath();
    ctx.arc(
      x0,
      y0,
      radius * fraction,
      -Math.PI / 2 - half,
      -Math.PI / 2 + half,
    );
    ctx.stroke();
  }
  for (const angle of [-half, half]) {
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x0 + Math.sin(angle) * radius, y0 - Math.cos(angle) * radius);
    ctx.stroke();
  }
  for (const point of points) {
    const distance =
      ((point.rangeM * Math.cos(point.elevationRad)) / reading.maxM) * radius;
    ctx.fillStyle = visibleRate(point.rangeRateMps) < 0 ? "#ffad7d" : "#68daca";
    ctx.beginPath();
    ctx.arc(
      x0 + Math.sin(point.azimuthRad) * distance,
      y0 - Math.cos(point.azimuthRad) * distance,
      2.5,
      0,
      Math.PI * 2,
    );
    ctx.fill();
  }
}
for (const kind of ["json", "csv"])
  $(`radar-export-${kind}`).onclick = () => {
    try {
      if (externalActive) throw new Error("外部飛控模式未提供雷達量測");
      if (!sensors?.radar) throw new Error("尚無雷達資料");
      const text = exportRadarData(
        kind,
        sensors.radar,
        { sourceVehicleId: activeVehicleId, worldTick: worldState.tick },
        sensorConfig,
      );
      download(
        text,
        kind,
        kind === "json" ? "application/json" : "text/csv;charset=utf-8",
        `flight-lab-radar-${activeVehicleId}-${Number.isFinite(sensors.radar.time) ? sensors.radar.time.toFixed(3) : "startup"}.${kind}`,
      );
    } catch (error) {
      toast(`無法匯出雷達：${error.message}`);
    }
  };

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
  reading.rangesM.forEach((range, i) => {
    if (!Number.isFinite(range)) return;
    const layer = Math.floor(i / reading.azimuthCount);
    const angle =
      reading.angleStartRad + (i % reading.azimuthCount) * reading.angleStepRad;
    const elevation =
      reading.elevationStartRad + layer * reading.elevationStepRad;
    const radius =
      ((range * Math.cos(elevation)) / reading.maxM) * (center - 15);
    const hue =
      reading.channelCount === 1
        ? 29
        : 234 - (198 * layer) / (reading.channelCount - 1);
    ctx.fillStyle = `hsl(${hue} 90% 60%)`;
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
    sendWorker({
      type: "sensorFault",
      value: { sensor: $("sensor-fault-channel").value, failed },
    });

const cameraLegend = $("camera-legend");
cameraLegend.replaceChildren(document.createTextNode("分類圖例： "));
for (const item of CAMERA_CLASSES) {
  const label = document.createElement("span");
  const swatch = document.createElement("span");
  swatch.className = "camera-swatch";
  swatch.style.backgroundColor = `rgb(${item.color.join(",")})`;
  label.append(swatch, ` ${item.label} `);
  cameraLegend.append(label);
}
let cameraFrame;
let displayedGimbalSettings = "";
function renderGimbal() {
  const g =
    externalActive && latest?.controlSource !== "external"
      ? undefined
      : latest?.gimbal;
  const disabled = !!replay || externalActive || !g || surveyApplying;
  document
    .querySelectorAll(
      "#gimbal-controls input, #gimbal-controls select, #gimbal-controls button",
    )
    .forEach((element) => (element.disabled = disabled));
  if (!g) {
    $("gimbal-status").textContent = "雲台：目前資料來源未提供姿態";
    return;
  }
  const settings = JSON.stringify([g.mode, g.target]);
  if (
    settings !== displayedGimbalSettings &&
    !$("gimbal-controls").contains(document.activeElement)
  ) {
    $("gimbal-mode").value = g.mode;
    for (const axis of ["yaw", "pitch", "roll"])
      $("gimbal-" + axis).value = (g.target[axis] * DEG).toFixed(2);
    displayedGimbalSettings = settings;
  }
  const angles = ["yaw", "pitch", "roll"]
    .map((axis) => (g.actual[axis] * DEG).toFixed(1))
    .join(" / ");
  const limited = ["yaw", "pitch", "roll"].filter((axis) => g.limited[axis]);
  const rateLimited = ["yaw", "pitch", "roll"].filter(
    (axis) => g.rateLimited[axis],
  );
  $("gimbal-status").textContent =
    `實際偏航 / 俯仰 / 橫滾 ${angles}° · 追蹤誤差 ${(g.trackingErrorRad * DEG).toFixed(1)}° · 角度限位 ${limited.join("、") || "無"} · 速度限幅 ${rateLimited.join("、") || "無"}`;
}
$("gimbal-apply").onclick = async () => {
  if (replay || externalActive || !latest?.gimbal) return;
  try {
    const target = Object.fromEntries(
      ["yaw", "pitch", "roll"].map((axis) => [
        axis,
        Number($("gimbal-" + axis).value) / DEG,
      ]),
    );
    await requestApi({
      type: "gimbal",
      value: {
        mode: $("gimbal-mode").value,
        target,
        maxRate: { ...latest.gimbal.maxRate },
      },
    });
    renderGimbal();
    toast("雲台目標已套用。");
  } catch (error) {
    toast(`雲台設定失敗：${error.message}`);
  }
};
let cameraEpoch = 0;
let cameraLastLive = 0;
let cameraDataset = [];
let cameraDatasetBytes = 0;
let cameraDatasetDirty = false;
function populateCameraConfig(value) {
  const resolution = $("camera-resolution"),
    size = `${value.width}x${value.height}`;
  resolution.querySelector("option[data-custom]")?.remove();
  if (![...resolution.options].some((option) => option.value === size)) {
    const option = new Option(`${value.width} × ${value.height}（自訂）`, size);
    option.dataset.custom = "true";
    resolution.add(option);
  }
  resolution.value = size;
  for (const [id, key] of [
    ["camera-fov", "horizontalFovDeg"],
    ["camera-near", "nearM"],
    ["camera-far", "farM"],
  ])
    $(id).value = value[key];
}
function applyCameraConfig(value) {
  if (surveyApplying) throw new Error("測繪拍攝設定正在套用，請稍候。");
  if (
    !value ||
    typeof value !== "object" ||
    Array.isArray(value) ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value))
  )
    throw new Error("相機設定必須是物件。");
  cameraConfig = validateCameraConfig({ ...cameraConfig, ...value });
  try {
    localStorage.setItem("flight-lab-camera-v2", JSON.stringify(cameraConfig));
  } catch {
    toast("此瀏覽器無法儲存相機設定。");
  }
  populateCameraConfig(cameraConfig);
  updateCameraStatus();
  return { ...cameraConfig };
}
function clearCameraPreview() {
  if (automaticCamera?.getState().active)
    automaticCamera.stop("實驗或載具已切換");
  cameraEpoch++;
  $("camera-live").checked = false;
  cameraFrame = undefined;
  $("camera-metadata").textContent = "";
  for (const id of ["camera-rgb", "camera-depth", "camera-segmentation"]) {
    const canvas = $(id);
    canvas.getContext("2d").clearRect(0, 0, canvas.width, canvas.height);
  }
  updateCameraStatus();
}
function updateCameraStatus() {
  const automatic = automaticCamera?.getState();
  const reserved = automatic?.active || automatic?.pending || surveyApplying;
  const full =
    cameraDataset.length >= CAMERA_DATASET_MAX_FRAMES ||
    cameraDatasetBytes + cameraConfig.width * cameraConfig.height * 9 >
      CAMERA_DATASET_MAX_BYTES;
  $("camera-status").textContent =
    `${cameraFrame ? "已顯示影像" : "尚未拍攝"} · 資料集 ${cameraDataset.length}/${CAMERA_DATASET_MAX_FRAMES} 張 · ${(cameraDatasetBytes / 1048576).toFixed(2)}/${(CAMERA_DATASET_MAX_BYTES / 1048576).toFixed(0)} MiB${cameraDatasetDirty ? " · 尚未匯出" : ""}${full ? " · 容量不足，請匯出並清除" : ""}`;
  $("camera-png").disabled = !cameraFrame;
  $("camera-export").disabled = !cameraDataset.length;
  $("camera-clear").disabled = !cameraDataset.length || cameraBusy || reserved || cameraDatasetBusy;
  $("camera-import").disabled = cameraBusy || reserved || cameraDatasetBusy;
  $("camera-add").disabled =
    !!replay ||
    cameraBusy ||
    reserved ||
    cameraDatasetBusy ||
    cameraDataset.length >= CAMERA_DATASET_MAX_FRAMES ||
    cameraDatasetBytes + cameraConfig.width * cameraConfig.height * 9 >
      CAMERA_DATASET_MAX_BYTES;
  $("camera-capture").disabled = !!replay || cameraBusy || reserved || cameraDatasetBusy;
  $("camera-live").disabled = !!replay || reserved || cameraDatasetBusy;
  if (automatic) automaticCameraControls?.refresh(automatic);
  if (surveyEditor?.refresh()) drawMission();
}
function showCameraFrame(frame) {
  cameraFrame = frame;
  for (const type of ["rgb", "depth", "segmentation"]) {
    const canvas = $(`camera-${type}`);
    canvas.width = frame.width;
    canvas.height = frame.height;
    canvas
      .getContext("2d")
      .putImageData(
        new ImageData(cameraPreview(frame, type), frame.width, frame.height),
        0,
        0,
      );
  }
  const pose = frame.pose,
    intrinsics = frame.intrinsics;
  $("camera-metadata").textContent =
    `來源載具 ${frame.sourceVehicleId} · 影格機型 ${{ quadrotor: "四旋翼", hexarotor: "六旋翼", octorotor: "八旋翼", fixedwing: "固定翼", vtol: "VTOL" }[frame.airframe]} · 模擬時間 ${frame.time.toFixed(3)} s · NED 位置 ${pose.positionNed.map((v) => v.toFixed(2)).join(" / ")} m · 機體四元數 ${pose.bodyQuaternionWxyz.map((v) => v.toFixed(3)).join(" / ")} · 拍攝時雲台偏航 / 俯仰 / 橫滾 ${["yaw", "pitch", "roll"].map((axis) => (frame.gimbal.actual[axis] * DEG).toFixed(1)).join(" / ")}° · 焦距 fx/fy ${intrinsics.fx.toFixed(1)} / ${intrinsics.fy.toFixed(1)} px · 主點 cx/cy ${intrinsics.cx.toFixed(1)} / ${intrinsics.cy.toFixed(1)} px · 水平視角 ${intrinsics.horizontalFovDeg.toFixed(1)}° · 景深範圍 ${frame.nearM.toFixed(2)}–${frame.farM.toFixed(1)} m`;
  updateCameraStatus();
}
async function captureCameraFrame(vehicleId = activeVehicleId) {
  if (surveyApplying) throw new Error("測繪拍攝設定正在套用，請稍候。");
  if (cameraDatasetBusy) throw new Error("相機資料集正在處理，請稍候。");
  if (replay) throw new Error("重播時無法拍攝新影像。");
  if (!externalActive && vehicleId !== activeVehicleId)
    throw new Error("請先選擇要拍攝的飛行器。");
  if (
    !scene ||
    !latest ||
    (externalActive && latest.controlSource !== "external")
  )
    throw new Error("相機尚未準備完成。");
  if (cameraBusy) throw new Error("相機正在拍攝，請稍候。");
  cameraBusy = true;
  const epoch = cameraEpoch;
  updateCameraStatus();
  try {
    const frozen = externalActive ? null : structuredClone(worldState);
    const selected = frozen?.vehicles.find((item) => item.id === vehicleId);
    if (!externalActive && !selected) throw new Error("找不到所選飛行器。");
    if (frozen) scene.updateWorld(frozen, vehicleId, 0);
    let pending;
    try {
      pending = scene.captureCamera(
        structuredClone(externalActive ? latest : selected.snapshot),
        { ...cameraConfig },
        externalActive ? null : frozen.scene.geography,
        {
          sourceVehicleId: vehicleId,
          worldTick: externalActive ? latest.metrics.steps : frozen.tick,
          captureId: crypto.randomUUID(),
        },
      );
    } finally {
      if (frozen && worldState)
        scene.updateWorld(worldState, activeVehicleId, 0);
    }
    const frame = await pending;
    if (epoch !== cameraEpoch || replay)
      throw new Error("實驗已切換，本次影像已取消。");
    return frame;
  } finally {
    cameraBusy = false;
    updateCameraStatus();
  }
}
let inspectionBusy = false;
let inspectionMatch = { key: "", status: "影像未核對" };
let inspectionMatchEpoch = 0;
async function checkInspectionImage() {
  const last = latest?.inspection?.latest;
  const key = `${activeVehicleId}:${last?.captureId || ""}:${last?.sha256 || ""}`;
  const epoch = ++inspectionMatchEpoch;
  inspectionMatch = { key, status: "影像未核對" };
  if (!last) return;
  const frame = cameraDataset.find((item) => item.captureId === last.captureId);
  if (!frame) {
    renderInspection();
    return;
  }
  try {
    const hash = await hashCameraFrame(frame);
    if (epoch !== inspectionMatchEpoch) return;
    inspectionMatch = {
      key,
      status: hash === last.sha256 ? "影像雜湊相符" : "影像雜湊不符",
    };
  } catch {
    if (epoch !== inspectionMatchEpoch) return;
    inspectionMatch = { key, status: "影像無法核對" };
  }
  renderInspection();
}
const inspectionReasons = {
  range: "距離不合格",
  angle: "朝向不合格",
  frustum: "目標超出畫面",
  pixels: "影像解析度不足",
  speed: "移動過快",
  occluded: "遮擋",
};
function renderInspection() {
  const targets = worldState?.scene?.inspectionTargets || [];
  const select = $("inspection-target"),
    previous = select.value;
  const targetKey = JSON.stringify(
    targets.map((target) => [target.id, target.name]),
  );
  if (select.dataset.targets !== targetKey) {
    select.replaceChildren(
      ...targets.map(
        (target) => new Option(target.name || target.id, target.id),
      ),
    );
    select.dataset.targets = targetKey;
    if (targets.some((target) => target.id === previous))
      select.value = previous;
  }
  const status = latest?.inspection;
  $("inspection-progress").textContent = targets.length
    ? worldState.vehicles
        .map((vehicle) => {
          const progress = vehicle.snapshot.inspection;
          return `${vehicle.name}（${vehicle.id}）${progress ? `已合格 ${progress.passed}/${progress.total} · 拍攝 ${progress.attempts} 次` : "尚無進度"}`;
        })
        .join(" ｜ ")
    : "尚無巡檢目標。";
  const last = status?.latest;
  const key = `${activeVehicleId}:${last?.captureId || ""}:${last?.sha256 || ""}`;
  if (inspectionMatch.key !== key) void checkInspectionImage();
  $("inspection-result").textContent = last
    ? `${last.targetId}：${last.assessment.passed ? "合格" : last.assessment.reasons.map((reason) => inspectionReasons[reason] || reason).join("、")} · 可見 ${last.assessment.visibleSamples}/${last.assessment.totalSamples} · ${inspectionMatch.status}`
    : "";
  $("inspection-capture").disabled =
    !!replay ||
    externalActive ||
    inspectionBusy ||
    cameraBusy ||
    cameraDatasetBusy ||
    automaticCamera?.getState().active ||
    automaticCamera?.getState().pending ||
    !targets.length;
  $("inspection-export").disabled =
    externalActive || inspectionBusy || !targets.length;
  for (const id of [
    "inspection-template-tower",
    "inspection-template-facade",
    "inspection-template-bridge",
  ])
    $(id).disabled = !!replay || externalActive || inspectionBusy;
}
for (const kind of ["tower", "facade", "bridge"])
  $("inspection-template-" + kind).onclick = () => {
    try {
      sceneEditor.loadInspectionTemplate(kind);
      document.querySelector('[data-workspace="scene"]').click();
      toast("巡檢範本已載入場景草稿，請檢查並套用。");
    } catch (error) {
      toast(`無法載入巡檢範本：${error.message}`);
    }
  };
$("inspection-export").onclick = async () => {
  if (externalActive) return;
  try {
    const report = replay
      ? recordingInspectionReport(replay, replayIndex, activeVehicleId)
      : await requestApi({
          type: "inspectionReport",
          vehicleId: activeVehicleId,
        });
    download(
      JSON.stringify(report),
      "json",
      "application/json",
      `flight-lab-inspection-${activeVehicleId}.json`,
    );
  } catch (error) {
    toast(`無法匯出巡檢報告：${error.message}`);
  }
};
async function captureInspectionFrame(vehicleId, targetId) {
  if (
    replay || externalActive || inspectionBusy || cameraBusy || cameraDatasetBusy || surveyApplying ||
    automaticCamera?.getState().active || automaticCamera?.getState().pending
  )
    throw new Error("目前無法拍攝巡檢影像");
  if (vehicleId !== activeVehicleId) throw new Error("請先選擇要拍攝的飛行器");
  if (
    cameraDataset.length >= CAMERA_DATASET_MAX_FRAMES ||
    cameraDatasetBytes + cameraConfig.width * cameraConfig.height * 9 >
      CAMERA_DATASET_MAX_BYTES
  )
    throw new Error("相機資料集已滿，請先匯出並清除。");
  const captureConfig = { ...cameraConfig };
  inspectionBusy = true;
  cameraBusy = true;
  renderInspection();
  updateCameraStatus();
  let captureId;
  try {
    const ticket = await requestApi({
      type: "inspectionBegin",
      vehicleId,
      value: { targetId, config: captureConfig },
    });
    captureId = ticket.captureId;
    const selected = ticket.world.vehicles.find(
      (item) => item.id === vehicleId,
    );
    if (!selected) throw new Error("找不到巡檢載具");
    scene.updateWorld(ticket.world, vehicleId, 0);
    let pending;
    try {
      pending = scene.captureCamera(
        selected.snapshot,
        captureConfig,
        ticket.world.scene.geography,
        { sourceVehicleId: vehicleId, worldTick: ticket.world.tick, captureId },
      );
    } finally {
      if (worldState) scene.updateWorld(worldState, activeVehicleId, 0);
    }
    const frame = { ...(await pending), ...ticket.metadata };
    appendCameraFrame(frame);
    refreshCameraSelector();
    $("camera-frame-select").value = String(cameraDataset.length - 1);
    showCameraFrame(frame);
    const report = await requestApi({
      type: "inspectionSubmit",
      vehicleId,
      value: { captureId, frame },
    });
    captureId = undefined;
    void checkInspectionImage();
    return report;
  } finally {
    if (captureId)
      try {
        await requestApi({
          type: "inspectionCancel",
          vehicleId,
          value: { captureId },
        });
      } catch {}
    inspectionBusy = false;
    cameraBusy = false;
    renderInspection();
    updateCameraStatus();
  }
}
$("inspection-capture").onclick = async () => {
  try {
    const report = await captureInspectionFrame(
      activeVehicleId,
      $("inspection-target").value,
    );
    toast(
      report.receipts.at(-1).assessment.passed
        ? "巡檢拍攝合格，影格已加入相機資料集。"
        : "拍攝已記錄，請依檢查原因調整位置或相機。",
    );
  } catch (error) {
    toast(
      `巡檢失敗：${error.message}；若已拍攝，影像保留於資料集但未通過核對。`,
    );
  }
};

function refreshCameraSelector() {
  const select = $("camera-frame-select");
  select.replaceChildren(new Option("選擇畫面", ""));
  cameraDataset.forEach((frame, i) =>
    select.add(
      new Option(
        `影像 ${i + 1} · ${frame.sourceVehicleId} · ${Number(frame.time).toFixed(2)} s`,
        String(i),
      ),
    ),
  );
  updateCameraStatus();
  void checkInspectionImage();
}
function appendCameraFrame(frame) {
  if (cameraDatasetBusy) throw new Error("相機資料集正在處理，請稍候。");
  const bytes = cameraFrameBytes(frame);
  if (
    cameraDataset.length >= CAMERA_DATASET_MAX_FRAMES ||
    cameraDatasetBytes + bytes > CAMERA_DATASET_MAX_BYTES
  )
    throw new Error("相機資料集已滿，請匯出並清除後再開始。");
  if (cameraDataset.some((item) => item.captureId === frame.captureId))
    throw new Error("同一張影像不能重複加入資料集。");
  cameraDataset.push(frame);
  cameraDatasetBytes += bytes;
  cameraDatasetDirty = true;
}
$("camera-capture").onclick = async () => {
  try {
    showCameraFrame(await captureCameraFrame());
  } catch (error) {
    toast(`拍攝失敗：${error.message}`);
  }
};
$("camera-add").onclick = async () => {
  if (
    cameraDataset.length >= CAMERA_DATASET_MAX_FRAMES ||
    cameraDatasetBytes + cameraConfig.width * cameraConfig.height * 9 >
      CAMERA_DATASET_MAX_BYTES
  )
    return toast("資料集已達容量上限，請先匯出並清除。");
  try {
    const frame = await captureCameraFrame();
    appendCameraFrame(frame);
    refreshCameraSelector();
    $("camera-frame-select").value = String(cameraDataset.length - 1);
    showCameraFrame(frame);
  } catch (error) {
    toast(`拍攝失敗：${error.message}`);
  }
};
$("camera-frame-select").onchange = (event) => {
  if (event.target.value !== "")
    showCameraFrame(cameraDataset[Number(event.target.value)]);
  void checkInspectionImage();
};
$("camera-png").onclick = () =>
  $("camera-rgb").toBlob((blob) => {
    if (!blob) return toast("無法匯出影像。");
    const url = URL.createObjectURL(blob),
      link = document.createElement("a");
    link.href = url;
    link.download = "flight-camera-rgb.png";
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }, "image/png");
$("camera-export").onclick = () => {
  try {
    download(
      cameraDatasetToJson(cameraDataset),
      "json",
      "application/json",
      "flight-camera-dataset.json",
    );
    cameraDatasetDirty = false;
    updateCameraStatus();
  } catch (error) {
    toast(`資料集匯出失敗：${error.message}`);
  }
};
$("camera-clear").onclick = async () => {
  if (surveyApplying || cameraBusy || cameraDatasetBusy || automaticCamera.getState().active || automaticCamera.getState().pending) return;
  cameraDatasetBusy = true;
  updateCameraStatus();
  try {
    if (
      cameraDatasetDirty &&
      !(await confirmAction("清除會移除尚未匯出的相機資料集。確定清除？"))
    )
      return;
    cameraDataset = [];
    cameraDatasetBytes = 0;
    cameraDatasetDirty = false;
    refreshCameraSelector();
  } finally {
    cameraDatasetBusy = false;
    updateCameraStatus();
  }
};
$("camera-import").onclick = () => {
  if (surveyApplying || cameraBusy || cameraDatasetBusy || automaticCamera.getState().active || automaticCamera.getState().pending) return;
  cameraEpoch++;
  $("camera-live").checked = false;
  $("camera-import-file").click();
};
$("camera-import-file").onchange = async (event) => {
  const file = event.target.files?.[0];
  event.target.value = "";
  if (!file) return;
  if (surveyApplying || cameraBusy || cameraDatasetBusy || automaticCamera.getState().active || automaticCamera.getState().pending)
    return toast("相機正在拍攝或處理資料集，請停止並等待後再匯入。");
  cameraEpoch++;
  $("camera-live").checked = false;
  if (file.size > CAMERA_DATASET_MAX_IMPORT_BYTES)
    return toast("相機資料集超過 40 MiB 匯入上限。");
  cameraDatasetBusy = true;
  updateCameraStatus();
  try {
    const frames = parseCameraDataset(await file.text());
    if (
      cameraDatasetDirty &&
      !(await confirmAction("匯入將取代尚未匯出的相機資料集。確定繼續？"))
    )
      return;
    cameraDataset = frames;
    cameraDatasetBytes = frames.reduce(
      (sum, frame) => sum + cameraFrameBytes(frame),
      0,
    );
    cameraDatasetDirty = false;
    refreshCameraSelector();
    if (frames.length) {
      $("camera-frame-select").value = "0";
      showCameraFrame(frames[0]);
    } else clearCameraPreview();
    toast("相機資料集已匯入。");
  } catch (error) {
    toast(`相機資料集無法讀取：${error.message}`);
  } finally {
    cameraDatasetBusy = false;
    updateCameraStatus();
  }
};
function updateCameraLive(now) {
  if (
    !$("camera-live").checked ||
    $("sensors-workspace").hidden ||
    document.hidden ||
    replay ||
    cameraBusy ||
    cameraDatasetBusy ||
    surveyApplying ||
    automaticCamera?.getState().active ||
    now - cameraLastLive < 500
  )
    return;
  cameraLastLive = now;
  captureCameraFrame().then(showCameraFrame).catch((error) => {
    $("camera-live").checked = false;
    toast(`相機預覽已停止：${error.message}`);
  });
}
function automaticCameraContext() {
  return {
    available: !!scene && !!latest && !!worldState && !replay && !externalActive &&
      !["crashed", "failsafe"].includes(latest.status),
    busy: cameraBusy || cameraDatasetBusy || surveyApplying,
    reason: replay
      ? "重播時不可開始新拍攝。"
      : externalActive
        ? "自動拍攝目前僅支援內建模擬。"
        : !scene ? "相機尚未準備完成。" : "",
    vehicleName: worldState?.vehicles.find((v) => v.id === activeVehicleId)?.name || activeVehicleId,
  };
}
function automaticCameraObservation() {
  if (!worldState || !latest) return null;
  return {
    vehicleId: activeVehicleId,
    epoch: cameraEpoch,
    worldTick: worldState.tick,
    time: latest.time,
    position: [...latest.position],
    running: running && !document.hidden,
    airborne: latest.ground.clearanceM >= 0.5 && latest.status !== "landed",
    available: automaticCameraContext().available,
    busy: cameraBusy,
    nowMs: performance.now(),
  };
}
function observeAutomaticCamera() {
  if (!automaticCamera) return;
  const observation = automaticCameraObservation();
  if (observation) automaticCamera.observe(observation);
  automaticCameraControls?.refresh(automaticCamera.getState());
}
function startAutomaticCamera(settings, vehicleId = activeVehicleId) {
  if (vehicleId !== activeVehicleId)
    throw new Error("請先選擇要拍攝的飛行器。");
  const context = automaticCameraContext();
  if (!context.available || context.busy)
    throw new Error(context.reason || "相機尚未可用或正在拍攝。");
  if (
    cameraDataset.length >= CAMERA_DATASET_MAX_FRAMES ||
    cameraDatasetBytes + cameraConfig.width * cameraConfig.height * 9 > CAMERA_DATASET_MAX_BYTES
  )
    throw new Error("相機資料集已滿，請先匯出並清除。");
  $("camera-live").checked = false;
  automaticCamera.start(settings, automaticCameraObservation());
  updateCameraStatus();
}
let displayedAutomaticFrame;
automaticCamera = createAutomaticCamera({
  capture: (vehicleId) => {
    if (
      cameraDataset.length >= CAMERA_DATASET_MAX_FRAMES ||
      cameraDatasetBytes + cameraConfig.width * cameraConfig.height * 9 > CAMERA_DATASET_MAX_BYTES
    )
      throw new Error("相機資料集已滿，已停止自動拍攝；已存影像保留。");
    return captureCameraFrame(vehicleId);
  },
  save: appendCameraFrame,
  onChange: (status) => {
    if (status.last && status.last.captureId !== displayedAutomaticFrame) {
      displayedAutomaticFrame = status.last.captureId;
      refreshCameraSelector();
      $("camera-frame-select").value = String(cameraDataset.length - 1);
      showCameraFrame(cameraDataset.at(-1));
    }
    updateCameraStatus();
  },
});
automaticCameraControls = createAutomaticCameraControls({
  start: startAutomaticCamera,
  stop: () => automaticCamera.stop(),
  getContext: automaticCameraContext,
});
updateCameraStatus();

const isLocalCompanion = isCompanionOrigin(window.location);
$("hil-remote-notice").hidden = isLocalCompanion;
$("hil-local-controls").hidden = !isLocalCompanion;
let hilOwned = false;
let hilExportedPackets = 0;
let hilPreviousGrounded;
let hilFollowed = false;
let groundStationBusy = false;
let groundStationSettingsKey = "";
function renderGroundStation(state) {
  const station = state?.groundStation;
  const ready =
    externalActive &&
    ["running", "paused"].includes(state?.phase) &&
    state.controller?.verified === true &&
    !!station &&
    !groundStationBusy;
  for (const element of $("ground-station-controls").querySelectorAll(
    "input, select, button",
  ))
    element.disabled = !ready;
  if (!station) {
    $("ground-station-status").textContent = "等待本機飛控橋接狀態。";
    return;
  }
  const key = JSON.stringify([
    station.enabled,
    station.targetPort,
    station.owner,
  ]);
  if (
    key !== groundStationSettingsKey &&
    !$("ground-station-controls").contains(document.activeElement)
  ) {
    $("ground-station-enabled").checked = station.enabled;
    $("ground-station-port").value = station.targetPort;
    $("ground-station-owner").value = station.owner;
    groundStationSettingsKey = key;
  }
  const age = Number.isFinite(station.lastHeartbeatAgeMs)
    ? (station.lastHeartbeatAgeMs / 1000).toFixed(1) + " 秒"
    : "—";
  $("ground-station-status").textContent =
    (station.enabled ? "橋接已啟用" : "橋接未啟用") +
    " · " +
    (station.connected ? "已收到地面站心跳" : "未收到地面站心跳") +
    " · 控制權：" +
    (station.owner === "gcs"
      ? "地面站"
      : station.owner === "web"
        ? "網頁"
        : "未知") +
    " · 本機接收埠 " +
    (station.localPort ?? "—") +
    " · 目的地 127.0.0.1:" +
    station.targetPort +
    " · 心跳齡期 " +
    age +
    (station.stats
      ? " · 接收 " +
        station.stats.receivedPackets +
        "／已轉送 " +
        station.stats.forwardedPackets +
        " 封包／拒絕 " +
        station.stats.rejectedPackets +
        "／遙測 " +
        station.stats.telemetryPackets
      : "") +
    (station.lastRejection
      ? " · 拒絕原因：" +
        station.lastRejection.reason +
        (station.lastRejection.messageId == null
          ? ""
          : "（訊息 " + station.lastRejection.messageId + "）")
      : "");
}
$("ground-station-apply").onclick = async () => {
  if (
    !hilClient ||
    !externalActive ||
    groundStationBusy ||
    !["running", "paused"].includes(hilState?.phase) ||
    hilState.controller?.verified !== true
  )
    return;
  const targetPort = Number($("ground-station-port").value);
  if (!validGroundStationPort(targetPort))
    return toast("地面站 UDP 埠須介於 1024 至 65535，且不可使用飛控保留埠。");
  groundStationBusy = true;
  renderGroundStation(hilState);
  try {
    await hilClient.configureGroundStation({
      enabled: $("ground-station-enabled").checked,
      targetPort,
      owner: $("ground-station-owner").value,
    });
    toast("橋接設定已送出；請以回報狀態確認。");
  } catch (error) {
    toast("無法套用地面站設定：" + error.message);
  } finally {
    groundStationBusy = false;
    renderGroundStation(hilState);
  }
};
function setExternalControls(disabled) {
  demEditor?.setReadonly(disabled || !!replay);
  renderGimbal();
  renderTutorial();
  renderGroundStation(hilState);
  if (disabled && latest?.controlSource !== "external")
    $("ground-status").textContent = "等待外部飛控模型真值。";
  $("hil-type").disabled = disabled;
  const selector =
    "#reset, #settings-open, #speed, #import-open, #target-controls input, #gust, [data-scenario], #training-workspace input, #training-workspace select, #sensor-fault-channel, #sensor-fault-on, #sensor-fault-off, #sensor-export-json, #flight-mode, [data-mode], #position-reference, #position-apply, #fault-apply, #fault-link, #fault-clear, #fault-pusher, #abort-takeoff, #transition-hover, #transition-cruise, #pilot-enable";
  document.querySelectorAll(selector).forEach((element) => {
    element.disabled = disabled;
  });
  if (disabled) {
    $("pilot-enable").checked = false;
    releasePilot();
  }
  const unready = hilFlightReadiness(hilState).length > 0;
  document.querySelectorAll("[data-command]").forEach((button) => {
    button.disabled =
      disabled &&
      (hilState?.phase !== "running" ||
        !hilWebControlAllowed(hilState) ||
        (unready && ["arm", "takeoff"].includes(button.dataset.command)));
  });
  updateMissionControls();
  updateLidarExportButtons();
  $("play").disabled =
    disabled &&
    (!["running", "paused"].includes(hilState?.phase) ||
      (hilState?.phase === "running" &&
        latest?.runway.weightOnWheels &&
        hilWebControlAllowed(hilState) &&
        unready));
}
function presentHilState(state) {
  hilState = state;
  showHilHealth(state, true);
  renderGroundStation(state);
  for (const button of document.querySelectorAll("[data-hil-command]")) {
    const action = button.dataset.hilCommand;
    button.disabled =
      !externalActive ||
      (!["pause", "resume", "stop"].includes(action) &&
        !hilWebControlAllowed(state)) ||
      (["arm", "takeoff"].includes(action) &&
        hilFlightReadiness(state).length > 0) ||
      (action === "stop"
        ? !["starting", "running", "paused", "fault"].includes(state.phase)
        : action === "resume"
          ? state.phase !== "paused"
          : state.phase !== "running");
  }
  $("hil-return").disabled = !externalActive;
  $("hil-export").disabled = !hilClient || !state.snapshot;
  const phases = {
    idle: "待命",
    starting: "啟動中",
    running: "運行中",
    paused: "已暫停",
    fault: "發生錯誤",
    stopped: "已停止",
  };
  const connected =
    state.controller?.verified === true &&
    state.stats?.sensorPackets > 0 &&
    state.stats?.actuatorPackets > 0;
  $("hil-status").textContent =
    `${phases[state.phase] || "未知狀態"}${connected ? ` · ${hilControllerName()} SITL 閉環／植物模型真值` : " · 等待飛控資料"}`;
  const c = state.controller;
  const missing = hilFlightReadiness(state);
  $("hil-readiness").textContent = ["ardupilot", "px4"].includes(c?.kind)
    ? missing.length
      ? `通訊準備未完成：等待${missing.join("、")}確認。`
      : "通訊準備：身分、參數及心跳已確認。"
    : "";
  $("hil-controller").textContent = c
    ? `${c.kind === "ardupilot" ? "ArduPilot" : "PX4"} ${c.version || "—"} · 回報來源 ${c.sourceSha || c.reportedGitHash || "—"} · ${c.kind === "ardupilot" ? "Quad X" : `機型 ${c.sysAutostart ?? "—"}`} · ${c.armed ? "已解鎖" : "未解鎖"} · 飛行模式 ${c.flightMode || "—"} · 估測 ${c.estimated == null ? "未提供" : "已提供"}`
    : "飛控尚未就緒";
  const stats = state.stats;
  $("hil-stats").textContent = stats
    ? `感測封包 ${stats.sensorPackets ?? 0} · 馬達封包 ${stats.actuatorPackets ?? 0} · ${c?.kind === "ardupilot" ? `JSON 格式錯誤 ${stats.invalidActuatorPackets ?? 0} · 序號錯配 ${stats.frameMismatches ?? 0}` : `HIL CRC 錯誤 ${stats.crcErrors ?? 0} · 任務 CRC 錯誤 ${stats.gcsCrcErrors ?? 0}`} · 馬達資料齡期 ${stats.lastActuatorAgeMs == null ? "—" : `${Number(stats.lastActuatorAgeMs).toFixed(0)} ms`}`
    : "";
  $("hil-logs").textContent = Array.isArray(state.logs)
    ? state.logs
        .slice(-40)
        .map((line) => String(line))
        .join("\n")
    : "";
  if (!externalActive) return;
  const confirmed =
    ["ardupilot", "px4"].includes(state.controller?.kind) &&
    hilWebControlAllowed(state) &&
    state.mission?.verified
      ? state.mission.points
      : [];
  const missionKey = JSON.stringify(confirmed);
  if (missionKey !== displayedExternalMission) {
    scene?.setMission(confirmed);
    displayedExternalMission = missionKey;
  }
  if (state.snapshot?.controlSource === "external") {
    const grounded = state.snapshot.runway.weightOnWheels;
    if (shouldFollowTakeoff(hilPreviousGrounded, grounded, hilFollowed)) {
      focusFlightView();
      hilFollowed = true;
    }
    if (hilPreviousGrounded === false && grounded) hilFollowed = false;
    hilPreviousGrounded = grounded;
    latest = state.snapshot;
    if (state.sensors) {
      sensors = state.sensors;
      renderSensors();
    }
    history.push(latest);
    while (history.length > 400) history.shift();
    timing = {};
    setPlaying(state.phase === "running");
    updateTelemetry();
  }
  setExternalControls(true);
}
async function connectHil() {
  if (hilClient) return;
  const client = createHilClient({
    location: window.location,
    fetch: window.fetch.bind(window),
    WebSocket: window.WebSocket,
    onState: presentHilState,
    onDisconnect: () => {
      hilClient = undefined;
      showHilHealth(hilState, false);
      if (externalActive) {
        hilState = { ...hilState, phase: "fault" };
        setPlaying(false);
        setExternalControls(true);
        $("hil-status").textContent = "本機連線已中斷；請返回內建模擬。";
      }
    },
  });
  await client.connect();
  hilClient = client;
}
function showHilHealth(state, connected) {
  const formatted = formatHilHealth(state?.controller?.health, {
    verified: state?.controller?.verified === true,
    connected,
  });
  $("hil-health").textContent = formatted.status;
  $("hil-health-warning").textContent = formatted.warning;
  $("hil-health-bits").textContent = formatted.details;
  $("hil-health-messages").textContent = formatted.messages.join("\n");
}
function focusFlightView() {
  document.querySelector('[data-workspace="flight"]').click();
  if (!rendering) $("render-toggle").click();
  document.querySelector('[data-camera="follow"]').click();
  $("viewport").scrollIntoView({ behavior: "smooth", block: "center" });
}
async function sendHilCommand(command) {
  if (!hilClient || !externalActive) return;
  if (
    !["pause", "resume", "stop"].includes(command) &&
    !hilWebControlAllowed(hilState)
  ) {
    toast("目前由地面站接管，網頁無法發送飛行命令。");
    return;
  }
  if (
    ["arm", "takeoff"].includes(command) &&
    hilFlightReadiness(hilState).length
  ) {
    toast(`飛控尚未準備：等待${hilFlightReadiness(hilState).join("、")}確認。`);
    return;
  }
  try {
    await hilClient.request("command", command);
    if (command === "takeoff" && !hilFollowed) {
      focusFlightView();
      hilFollowed = true;
    }
    toast("命令已送出，請確認飛控狀態與高度回饋。");
  } catch (error) {
    toast(`飛控命令失敗：${error.message}`);
  }
}

$("hil-start").onclick = async () => {
  if (!isLocalCompanion || externalActive) return;
  if (replay) return toast("請先結束重播並返回內建模擬。");
  $("hil-start").disabled = true;
  try {
    if (worldConfig.vehicles.length !== 1)
      throw new Error("外部飛控僅支援單架載具，請先將機隊設為一架。");
    await connectHil();
    const existing = await hilClient.request("inspect");
    const reattach =
      ["starting", "running", "paused", "fault"].includes(existing.phase) &&
      existing.snapshot;
    const binaryPath = $("hil-binary").value.trim();
    if (sceneEditor?.getApplied().geography)
      throw new Error(
        "外部飛控目前使用平坦的本機場景；請先移除地理地形並重設，再啟動或接回。",
      );
    const controller = reattach
      ? existing.controller.kind
      : $("hil-type").value;
    $("hil-type").value = controller;
    if (!reattach) {
      if (!binaryPath) {
        toast("請輸入所選飛控的執行檔完整路徑。");
        return;
      }
      if (config.airframe !== "quadrotor" || !config.startOnGround) {
        toast("請在管理參數選擇四旋翼及地面起始後再啟動。");
        return;
      }
      if (
        (existing.stats?.actuatorPackets ?? 0) > hilExportedPackets &&
        !(await confirmAction(
          "啟動新飛控將取代尚未匯出的 SITL 紀錄。確定繼續？",
        ))
      )
        return;
    }
    const paused = await requestApi({ type: "pause" });
    if (replay) throw new Error("已進入紀錄重播，請先返回內建模擬。");
    internalSaved = {
      state: paused,
      history: [...history],
      sampleCount,
      timing,
      config,
      scenario,
      waypoints: structuredClone(waypoints),
    };
    waypoints =
      reattach && existing.mission?.points
        ? structuredClone(existing.mission.points)
        : structuredClone(waypoints);
    displayedExternalMission = "";
    clearCameraPreview();
    externalActive = true;
    renderRadar(null);
    proximityPanel.render(null, { available: false });
    groundStationSettingsKey = "";
    hilState = reattach
      ? existing
      : { phase: "starting", controller: { kind: controller } };
    renderWaypoints();
    hilPreviousGrounded = undefined;
    hilFollowed = false;
    sceneEditor?.setScene(sceneEditor.getApplied(), true);
    lastRenderedSceneKey = "";
    hilOwned = true;
    history = [];
    sensors = undefined;
    $("sensor-readings").textContent =
      "外部飛控模式未提供此工作區的感測器讀值。";
    drawLidar(null);
    scene?.setSensorScan(null);
    scene?.setTerrain({ kind: "flat" });
    scene?.setEnvironment({ preset: "lab", timeOfDay: 12, fog: 0, rain: 0 });
    scene?.setObstacles([]);
    scene?.setMission([]);
    scene?.setGates([]);
    presentHilState(
      reattach
        ? existing
        : { phase: "starting", controller: { kind: controller } },
    );
    setExternalControls(true);
    if (reattach) toast("已接回飛控；模擬保持目前狀態，請自行選擇是否繼續。");
    else {
      await hilClient.request("start", { controller, binaryPath, config });
      hilExportedPackets = 0;
      toast("飛控啟動請求已送出，請等待飛控資料。");
    }
  } catch (error) {
    toast(`無法啟動或接回飛控：${error.message}`);
    if (externalActive) await returnInternal(false);
  } finally {
    $("hil-start").disabled = externalActive;
  }
};
document
  .querySelectorAll("[data-hil-command]")
  .forEach(
    (button) =>
      (button.onclick = () => sendHilCommand(button.dataset.hilCommand)),
  );
async function returnInternal(stop = true) {
  if (!externalActive) return;
  if (stop && hilOwned) {
    try {
      if (!hilClient) await connectHil();
      await hilClient.request("command", "stop");
    } catch (error) {
      toast(`無法停止飛控：${error.message}`);
      return;
    }
  }
  let current;
  try {
    current = await requestApi({ type: "inspect" }, true);
  } catch (error) {
    toast(`無法恢復內建模擬：${error.message}`);
    return;
  }
  clearCameraPreview();
  externalActive = false;
  renderGroundStation(hilState);
  hilOwned = false;
  config = internalSaved.config;
  scenario = internalSaved.scenario;
  history = internalSaved.history;
  sampleCount = internalSaved.sampleCount;
  timing = internalSaved.timing;
  latest = current.snapshot;
  sensors = current.sensors;
  if (current.setup) {
    showReplayEvent("scene", current.setup.scene);
    waypoints = internalSaved.waypoints;
    renderWaypoints();
  }
  setExternalControls(false);
  setPlaying(false);
  updateTelemetry();
  if (sensors) renderSensors();
  $("hil-start").disabled = false;
  for (const button of document.querySelectorAll("[data-hil-command]"))
    button.disabled = true;
  $("hil-return").disabled = true;
  $("hil-status").textContent = "已返回內建模擬；原實驗紀錄已保留。";
}
$("hil-return").onclick = () => returnInternal();
$("hil-export").onclick = async () => {
  try {
    if (!hilClient) throw new Error("本機服務尚未連線。");
    const exportedPackets = hilState?.stats?.actuatorPackets ?? 0;
    download(
      await hilClient.recording(),
      "json",
      "application/json",
      `flight-lab-${hilState?.controller?.kind === "ardupilot" ? "ardupilot" : "px4"}-sitl.json`,
    );
    hilExportedPackets = exportedPackets;
  } catch (error) {
    toast(`無法匯出 SITL 紀錄：${error.message}`);
  }
};

for (const button of document.querySelectorAll("[data-hil-command]"))
  button.disabled = true;
$("hil-return").disabled = true;
$("hil-export").disabled = true;

sceneEditor = createSceneEditor({
  apply: async (value) => {
    const applied = await applyScene(value);
    sceneEditor.markApplied(applied, value);
  },
  confirm: confirmAction,
  download,
  notify: toast,
});
demEditor = createDemEditor({
  getReadonly: () => !!replay || externalActive,
  onApply: async ({ terrain, geography }) => {
    await applyScene(
      {
        ...sceneEditor.getApplied(),
        terrain,
        geography,
        name: geography.source.name.slice(0, 80),
      },
      true,
    );
    return true;
  },
});
