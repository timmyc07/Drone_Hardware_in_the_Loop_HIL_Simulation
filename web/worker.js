import { validateScene } from "./scene-data.js";
import { createSimulation, validateConfig } from "./sim.js";
import {
  createTrainer,
  isCourseSupported,
  resolveTrainingGates,
} from "./training.js";
import { createSensorSuite, validateSensorConfig } from "./sensors.js";
import { exportLidarCsv, exportLidarPcd } from "./pointcloud.js";
import {
  RECORDING_VERSION,
  MODEL_VERSION,
  compactSnapshot,
  validateSetup,
} from "./recording.js";

const DT = 0.005;
const LIMIT = 1800;
const DATA_LIMIT = 96 * 1024 * 1024;
let config = validateConfig();
let setup = validateSetup(undefined, config);
let initialSetup = structuredClone(setup);
let sim = createSimulation(config);
let trainer = createTrainer(
  setup.scene.training,
  config.airframe,
  setup.scene.customGates,
);
let training = trainer.update(sim.snapshot());
let sensorSuite = createSensorSuite(
  setup.sensors,
  (config.seed ^ 0x9e3779b9) >>> 0,
);
sensorSuite.setObstacles(setup.scene.obstacles);
let sensorState = sensorSuite.update(sim.snapshot());
let scenario = "hover";
let running = false;
let speed = 1;
let renderEnabled = true;
let accumulator = 0;
let last = performance.now();
let lastPublish = last;
let wallSeconds = 0;
let skippedWallSeconds = 0;
let lateness = [];
let lateTicks = 0;
let samples = [compactSnapshot(snapshot())];
let sensorFrames = [sensorState];
let recordBytes =
  JSON.stringify(samples[0]).length + JSON.stringify(sensorState).length;
let storageLimited = false;
let pending = [...samples];
let events = [];
let createdAt = new Date().toISOString();

function snapshot() {
  return { ...sim.snapshot(), training: structuredClone(training) };
}

function runtime() {
  const sorted = [...lateness].sort((a, b) => a - b);
  return {
    speed,
    wallSeconds,
    p95LateMs: sorted.length
      ? sorted[Math.floor((sorted.length - 1) * 0.95)]
      : 0,
    lateTicks,
    skippedWallSeconds,
    renderEnabled,
  };
}

function publish(message) {
  const state = snapshot();
  postMessage({
    type: "state",
    running,
    snapshot: state,
    sensors: sensorState,
    samples: pending,
    sampleCount: samples.length,
    config,
    scenario,
    setup,
    timing: {
      ...runtime(),
      realTimeFactor: wallSeconds ? state.time / wallSeconds : 0,
    },
    storageLimited,
    message: storageLimited ? "storage-limit" : message,
  });
  pending = [];
  lastPublish = performance.now();
}

function reset(nextConfig, nextScenario, nextSetup) {
  const validated = validateConfig(nextConfig);
  if (!["hover", "step", "gust"].includes(nextScenario))
    throw new Error("未知的實驗情境");
  const validatedSetup = validateSetup(
    nextSetup === undefined
      ? {
          ...setup,
          mode: "attitude",
          mission: [],
          scene: {
            ...setup.scene,
            training: isCourseSupported(
              setup.scene.training,
              validated.airframe,
            )
              ? setup.scene.training
              : "free",
          },
        }
      : nextSetup,
    validated,
  );
  const next = createSimulation(validated);
  next.setScenario(nextScenario);
  next.setObstacles(validatedSetup.scene.obstacles);
  if (validatedSetup.mission.length) next.setMission(validatedSetup.mission);
  next.setMode(validatedSetup.mode);
  const nextTrainer = createTrainer(
    validatedSetup.scene.training,
    validated.airframe,
    validatedSetup.scene.customGates,
  );
  const nextTraining = nextTrainer.update(next.snapshot());
  const nextSensors = createSensorSuite(
    validatedSetup.sensors,
    (validated.seed ^ 0x9e3779b9) >>> 0,
  );
  nextSensors.setObstacles(validatedSetup.scene.obstacles);
  const nextSensorState = nextSensors.update(next.snapshot());
  config = validated;
  setup = validatedSetup;
  initialSetup = structuredClone(setup);
  scenario = nextScenario;
  sim = next;
  trainer = nextTrainer;
  training = nextTraining;
  sensorSuite = nextSensors;
  sensorState = nextSensorState;
  running = false;
  accumulator = 0;
  wallSeconds = 0;
  skippedWallSeconds = 0;
  lateness = [];
  lateTicks = 0;
  samples = [compactSnapshot(snapshot())];
  sensorFrames = [sensorState];
  recordBytes =
    JSON.stringify(samples[0]).length + JSON.stringify(sensorState).length;
  storageLimited = false;
  pending = [...samples];
  events = [];
  createdAt = new Date().toISOString();
  last = performance.now();
  publish("reset");
}

function recordEvent(data) {
  const event = { time: sim.snapshot().time, type: data.type };
  if (
    ![
      "gust",
      "clearFaults",
      "takeoff",
      "abortTakeoff",
      "arm",
      "disarm",
    ].includes(data.type)
  )
    event.value = structuredClone(data.value);
  if (data.type === "fault") event.fault = data.fault;
  events.push(event);
  recordBytes += JSON.stringify(event).length + 1;
  const current = compactSnapshot(snapshot());
  if (Math.abs(samples.at(-1).time - current.time) < 1e-9) {
    recordBytes +=
      JSON.stringify(current).length +
      JSON.stringify(sensorState).length -
      JSON.stringify(samples.at(-1)).length -
      JSON.stringify(sensorFrames.at(-1)).length;
    samples[samples.length - 1] = current;
    sensorFrames[sensorFrames.length - 1] = sensorState;
    if (pending.length && Math.abs(pending.at(-1).time - current.time) < 1e-9)
      pending[pending.length - 1] = current;
    else pending.push(current);
  }
  if (recordBytes >= DATA_LIMIT) {
    running = false;
    storageLimited = true;
  }
}

function recording() {
  const final = compactSnapshot(snapshot());
  const exported = [...samples];
  const exportedSensors = [...sensorFrames];
  if (final.time > exported.at(-1).time + 1e-8) {
    exported.push(final);
    exportedSensors.push(sensorState);
  } else {
    exported[exported.length - 1] = final;
    exportedSensors[exportedSensors.length - 1] = sensorState;
  }
  return {
    format: "flight-lab",
    version: RECORDING_VERSION,
    modelVersion: MODEL_VERSION,
    createdAt,
    config,
    scenario,
    dt: DT,
    setup: initialSetup,
    runtime: runtime(),
    events,
    samples: exported,
    sensorFrames: exportedSensors,
  };
}

function inspectedState() {
  return {
    snapshot: snapshot(),
    sensors: sensorState,
    setup,
    running,
    timing: {
      ...runtime(),
      realTimeFactor: wallSeconds ? sim.snapshot().time / wallSeconds : 0,
    },
  };
}

onmessage = ({ data }) => {
  try {
    if (!data || typeof data.type !== "string")
      throw new Error("無效的模擬操作");
    if (
      data.requestId !== undefined &&
      (!Number.isSafeInteger(data.requestId) || data.requestId < 1)
    )
      throw new Error("無效的程式介面請求編號");
    const readCommands = [
      "reset",
      "pause",
      "export",
      "sensorExport",
      "pointCloudExport",
      "inspect",
      "inspectSensors",
      "inspectScene",
      "apiExport",
    ];
    if (storageLimited && !readCommands.includes(data.type))
      throw new Error("紀錄資料已達容量上限，請匯出後重設");
    if (events.length >= 100000 && !readCommands.includes(data.type))
      throw new Error("事件紀錄已達上限，請先匯出再重設");
    switch (data.type) {
      case "reset":
        reset(
          data.config === undefined ? config : data.config,
          data.scenario === undefined ? scenario : data.scenario,
          data.setup,
        );
        break;
      case "play":
        if (sim.snapshot().time >= LIMIT)
          throw new Error("本次實驗已達 30 分鐘，請先匯出再重設");
        running = true;
        last = performance.now();
        publish();
        break;
      case "pause":
        running = false;
        publish();
        break;
      case "speed":
        if (![0.5, 1, 2, 4].includes(data.value))
          throw new Error("無效的模擬速度");
        speed = data.value;
        break;
      case "targets":
        sim.setTargets(data.value);
        recordEvent(data);
        publish();
        break;
      case "gust":
        sim.gust();
        recordEvent(data);
        publish();
        break;
      case "mode":
        sim.setMode(data.value);
        setup.mode = data.value;
        recordEvent(data);
        publish();
        break;
      case "transition":
        sim.setTransition(data.value);
        recordEvent(data);
        publish();
        break;
      case "position":
        sim.setPositionTarget(data.value);
        recordEvent(data);
        publish();
        break;
      case "mission":
        sim.setMission(data.value);
        setup.mission = sim.snapshot().navigation.waypoints;
        recordEvent({ ...data, value: setup.mission });
        publish();
        break;
      case "fault":
        sim.injectFault(data.fault, data.value);
        recordEvent(data);
        publish();
        break;
      case "clearFaults":
        sim.clearFaults();
        recordEvent(data);
        publish();
        break;
      case "takeoff":
        sim.takeoff();
        setup.mode = sim.snapshot().navigation.mode;
        recordEvent(data);
        publish();
        break;
      case "abortTakeoff":
        sim.abortTakeoff();
        setup.mode = sim.snapshot().navigation.mode;
        recordEvent(data);
        publish();
        break;
      case "arm":
        sim.arm();
        recordEvent(data);
        publish();
        break;
      case "disarm":
        sim.disarm();
        recordEvent(data);
        publish();
        break;
      case "scene": {
        const nextScene = validateScene(data.value, config.airframe);
        const changed =
          setup.scene.training !== nextScene.training ||
          JSON.stringify(resolveTrainingGates(setup.scene, config.airframe)) !==
            JSON.stringify(resolveTrainingGates(nextScene, config.airframe));
        const nextTrainer = changed
          ? createTrainer(
              nextScene.training,
              config.airframe,
              nextScene.customGates,
            )
          : trainer;
        const nextTraining = changed
          ? nextTrainer.update(sim.snapshot())
          : training;
        // Both consumers receive one canonical validated geometry at the same simulation boundary.
        sim.setObstacles(nextScene.obstacles);
        sensorSuite.setObstacles(nextScene.obstacles);
        setup.scene = nextScene;
        trainer = nextTrainer;
        training = nextTraining;
        recordEvent({ ...data, value: nextScene });
        publish();
        break;
      }
      case "render":
        if (typeof data.value !== "boolean")
          throw new Error("渲染設定必須為布林值");
        renderEnabled = data.value;
        recordEvent(data);
        break;
      case "sensorConfig": {
        const validated = validateSensorConfig(data.value);
        const next = createSensorSuite(
          validated,
          (config.seed ^ 0x9e3779b9) >>> 0,
        );
        next.setObstacles(setup.scene.obstacles);
        const state = next.update(sim.snapshot());
        sensorSuite = next;
        sensorState = state;
        setup.sensors = validated;
        recordEvent({ ...data, value: validated });
        publish();
        break;
      }
      case "sensorFault":
        if (
          !data.value ||
          typeof data.value !== "object" ||
          Object.keys(data.value).some((k) => !["sensor", "failed"].includes(k))
        )
          throw new Error("無效的感測器故障設定");
        sensorSuite.setFault(data.value.sensor, data.value.failed);
        sensorState = sensorSuite.snapshot();
        recordEvent(data);
        publish();
        break;
      case "sensorExport": {
        if (!["json", "csv", "pcd"].includes(data.kind))
          throw new Error("不支援的感測器匯出格式");
        const text =
          data.kind === "json"
            ? JSON.stringify({
                modelVersion: MODEL_VERSION,
                config: setup.sensors,
                sensors: sensorState,
              })
            : data.kind === "csv"
              ? exportLidarCsv(sensorState.lidar)
              : exportLidarPcd(sensorState.lidar);
        postMessage({ type: "sensorExport", kind: data.kind, text });
        break;
      }
      case "pointCloudExport": {
        if (data.requestId === undefined)
          throw new Error("此操作需要程式介面請求編號");
        if (!["csv", "pcd"].includes(data.value))
          throw new Error("不支援的點雲匯出格式");
        const reading = sensorState.lidar;
        postMessage({
          type: "apiReply",
          requestId: data.requestId,
          ok: true,
          value: {
            format: data.value,
            frame: "NED",
            units: "m",
            time: reading.time,
            pointCount: reading.rangesM.filter((range) => range !== null)
              .length,
            text:
              data.value === "csv"
                ? exportLidarCsv(reading)
                : exportLidarPcd(reading),
          },
        });
        return;
      }
      case "inspect":
      case "inspectSensors":
      case "inspectScene":
      case "apiExport":
        if (data.requestId === undefined)
          throw new Error("此操作需要程式介面請求編號");
        postMessage({
          type: "apiReply",
          requestId: data.requestId,
          ok: true,
          value:
            data.type === "inspect"
              ? inspectedState()
              : data.type === "inspectScene"
                ? setup.scene
                : data.type === "inspectSensors"
                  ? sensorState
                  : recording(),
        });
        return;
      case "export": {
        if (!["csv", "json"].includes(data.kind))
          throw new Error("不支援的紀錄格式");
        postMessage({
          type: "recording",
          kind: data.kind,
          recording: recording(),
        });
        break;
      }
      default:
        throw new Error("未知的模擬操作");
    }
    if (data.requestId !== undefined)
      postMessage({
        type: "apiReply",
        requestId: data.requestId,
        ok: true,
        value: inspectedState(),
      });
  } catch (error) {
    if (data?.requestId !== undefined) {
      postMessage({
        type: "apiReply",
        requestId: data.requestId,
        ok: false,
        error: error.message,
      });
      return;
    }
    running = false;
    postMessage({ type: "error", message: error.message });
    publish();
  }
};

function tick() {
  const now = performance.now();
  const elapsed = Math.max(0, (now - last) / 1000);
  last = now;
  if (running) {
    wallSeconds += elapsed;
    skippedWallSeconds += Math.max(0, elapsed - 0.1);
    const lateMs = Math.max(0, elapsed * 1000 - 10);
    lateness.push(lateMs);
    if (lateness.length > 1000) lateness.shift();
    if (lateMs > 10) lateTicks++;
    accumulator += Math.min(elapsed, 0.1) * speed;
    try {
      while (accumulator >= DT && running) {
        const state = sim.step(DT);
        training = trainer.update(state);
        sensorState = sensorSuite.update(state);
        accumulator -= DT;
        if (state.metrics.steps % 10 === 0) {
          const sample = compactSnapshot({
            ...state,
            training: structuredClone(training),
          });
          samples.push(sample);
          sensorFrames.push(sensorState);
          recordBytes +=
            JSON.stringify(sample).length +
            JSON.stringify(sensorState).length +
            2;
          pending.push(sample);
          if (recordBytes >= DATA_LIMIT) {
            running = false;
            storageLimited = true;
            publish("storage-limit");
          }
        }
        if (state.time >= LIMIT - 1e-8) {
          running = false;
          publish("limit");
        }
      }
      if (now - lastPublish >= 50) publish();
    } catch (error) {
      running = false;
      postMessage({
        type: "error",
        message: `數值模擬已停止：${error.message}`,
      });
      publish();
    }
  }
  setTimeout(tick, 10);
}
publish("reset");
tick();
