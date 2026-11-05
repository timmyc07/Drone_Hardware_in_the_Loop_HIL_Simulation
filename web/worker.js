import { createWorld, defaultWorldConfig } from "./world.js";
import { exportRadarData } from "./radar.js";
import { exportProximityData } from "./proximity.js";
import { exportVehiclePointCloud } from "./pointcloud.js";
import {
  cameraMetadata,
  validateCameraConfig,
  validateCameraFrame,
  validateCameraMetadata,
  hashCameraFrame,
} from "./camera.js";
import {
  RECORDING_VERSION,
  MODEL_VERSION,
  compactWorldSnapshot,
  worldSensorFrame,
  compactSensorFrame,
} from "./recording.js";

const DT = 0.005,
  LIMIT = 1800,
  DATA_LIMIT = 96 * 1024 * 1024;
let world = createWorld(defaultWorldConfig());
let initialWorld = world.configuration;
let activeVehicleId = initialWorld.vehicles[0].id;
let running = false,
  speed = 1,
  renderEnabled = true,
  accumulator = 0;
let inspectionCapture = null;
function releaseInspection(capture = inspectionCapture) {
  if (!capture || capture !== inspectionCapture) return;
  inspectionCapture = null;
  running = capture.resume && !storageLimited;
  last = performance.now();
}
let last = performance.now(),
  lastPublish = last,
  wallSeconds = 0,
  skippedWallSeconds = 0;
let lateness = [],
  lateTicks = 0,
  samples = [],
  sensorFrames = [],
  pending = [],
  events = [];
let recordBytes = 0,
  storageLimited = false,
  termination = null,
  createdAt = new Date().toISOString();
const VEHICLE_COMMANDS = [
  "targets",
  "gust",
  "mode",
  "gimbal",
  "transition",
  "position",
  "mission",
  "fault",
  "clearFaults",
  "takeoff",
  "abortTakeoff",
  "arm",
  "disarm",
  "training",
  "sensorConfig",
  "sensorFault",
];
const VEHICLE_READS = [
  "inspect",
  "inspectSensors",
  "inspectGimbal",
  "pointCloudExport",
  "sensorExport",
  "radarExport",
  "proximityExport",
  "inspectionBegin",
  "inspectionSubmit",
  "inspectionCancel",
  "inspectionReport",
];
const READS = [
  "resetWorld",
  "pause",
  "export",
  "apiExport",
  "inspectWorld",
  "inspectScene",
  "selectVehicle",
  ...VEHICLE_READS.filter(
    (type) => !["inspectionBegin", "inspectionSubmit"].includes(type),
  ),
];
function state() {
  const snapshot = world.snapshot(),
    configuration = world.configuration;
  return { ...snapshot, scene: configuration.scene, wind: configuration.wind };
}
function selected(value, vehicleId = activeVehicleId) {
  if (typeof vehicleId !== "string") throw new Error("請指定載具 vehicleId");
  const vehicle = value.vehicles.find((v) => v.id === vehicleId);
  if (!vehicle) throw new Error("找不到指定載具");
  return vehicle;
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
function inspectedState(vehicleId = activeVehicleId) {
  const value = state(),
    v = selected(value, vehicleId);
  return {
    world: value,
    worldConfig: world.configuration,
    activeVehicleId,
    snapshot: v.snapshot,
    sensors: v.sensors,
    config: v.config,
    scenario: v.scenario,
    setup: { ...v.setup, scene: value.scene },
    running,
    termination,
    timing: {
      ...runtime(),
      realTimeFactor: wallSeconds ? value.time / wallSeconds : 0,
    },
  };
}
function publish(message) {
  const value = inspectedState();
  postMessage({
    type: "state",
    ...value,
    samples: pending.map((frame) => selected(frame).snapshot),
    worldSamples: pending,
    sampleCount: samples.length,
    storageLimited,
    message: storageLimited ? "storage-limit" : message,
  });
  pending = [];
  lastPublish = performance.now();
}
function capture() {
  const value = state(),
    frame = compactWorldSnapshot(value),
    sensors = worldSensorFrame(value);
  sensors.vehicles = sensors.vehicles.map((v) => ({
    id: v.id,
    sensors: compactSensorFrame(v.sensors),
  }));
  const previous = samples.at(-1);
  if (previous?.tick === frame.tick) {
    recordBytes -=
      JSON.stringify(previous).length +
      JSON.stringify(sensorFrames.at(-1)).length +
      2;
    samples[samples.length - 1] = frame;
    sensorFrames[sensorFrames.length - 1] = sensors;
  } else {
    samples.push(frame);
    sensorFrames.push(sensors);
  }
  recordBytes +=
    JSON.stringify(frame).length + JSON.stringify(sensors).length + 2;
  const view = {
    tick: value.tick,
    time: value.time,
    vehicles: value.vehicles.map((v) => ({ id: v.id, snapshot: v.snapshot })),
    collisions: value.collisions,
  };
  if (pending.at(-1)?.tick === view.tick) pending[pending.length - 1] = view;
  else pending.push(view);
  if (recordBytes >= DATA_LIMIT) {
    running = false;
    storageLimited = true;
  }
}
function reset(value, id) {
  const next = createWorld(value),
    configuration = next.configuration;
  const nextId = id ?? configuration.vehicles[0].id;
  if (!configuration.vehicles.some((v) => v.id === nextId))
    throw new Error("找不到指定載具");
  inspectionCapture = null;
  world = next;
  initialWorld = configuration;
  activeVehicleId = nextId;
  running = false;
  accumulator = 0;
  wallSeconds = 0;
  skippedWallSeconds = 0;
  lateness = [];
  lateTicks = 0;
  samples = [];
  sensorFrames = [];
  pending = [];
  events = [];
  recordBytes = 0;
  storageLimited = false;
  termination = null;
  createdAt = new Date().toISOString();
  last = performance.now();
  capture();
  publish("reset");
}
function recordEvent(data) {
  const value = world.snapshot();
  const event = { tick: value.tick, time: value.time, type: data.type };
  if (VEHICLE_COMMANDS.includes(data.type)) event.vehicleId = data.vehicleId;
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
  if (data.type === "inspection") {
    event.vehicleId = data.vehicleId;
    event.receipt = world.inspectionReport(data.vehicleId).receipts.at(-1);
  }
  events.push(event);
  recordBytes += JSON.stringify(event).length + 1;
  capture();
}
function recording() {
  // Every command and export retains one atomic frame for the entire world.
  capture();
  return {
    format: "flight-lab",
    version: RECORDING_VERSION,
    modelVersion: MODEL_VERSION,
    createdAt,
    dt: DT,
    world: initialWorld,
    runtime: runtime(),
    events,
    samples,
    sensorFrames,
    termination,
  };
}
function cloud(kind, v, value) {
  return exportVehiclePointCloud(kind, v.sensors.lidar, {
    sourceVehicleId: v.id,
    worldTick: value.tick,
  });
}

onmessage = async ({ data }) => {
  try {
    if (!data || typeof data.type !== "string")
      throw new Error("無效的模擬操作");
    if (
      data.requestId !== undefined &&
      (!Number.isSafeInteger(data.requestId) || data.requestId < 1)
    )
      throw new Error("無效的程式介面請求編號");
    if (
      (storageLimited || events.length >= 100000) &&
      !READS.includes(data.type)
    )
      throw new Error("紀錄資料已達上限，請匯出後重設");
    if (
      inspectionCapture &&
      ![...READS, "inspectionSubmit", "inspectionCancel"].includes(data.type)
    )
      throw new Error("巡檢拍攝中，請等待影像完成或取消拍攝");
    if (
      VEHICLE_COMMANDS.includes(data.type) ||
      VEHICLE_READS.includes(data.type)
    ) {
      if (typeof data.vehicleId !== "string")
        throw new Error("請指定載具 vehicleId");
      selected(state(), data.vehicleId);
    }
    let replyVehicle;
    if (VEHICLE_COMMANDS.includes(data.type)) {
      world.command(data.vehicleId, data.type, data.value, data.fault);
      replyVehicle = data.vehicleId;
      // Store canonical setup values rather than partially specified input forms.
      if (["mission", "gimbal", "sensorConfig"].includes(data.type)) {
        const v = selected(state(), data.vehicleId);
        data = {
          ...data,
          value: v.setup[data.type === "sensorConfig" ? "sensors" : data.type],
        };
      }
      recordEvent(data);
      publish();
    } else
      switch (data.type) {
        case "resetWorld":
          reset(data.value, data.activeVehicleId);
          break;
        case "selectVehicle":
          selected(state(), data.vehicleId);
          activeVehicleId = data.vehicleId;
          publish("selection");
          break;
        case "play":
          if (world.snapshot().time >= LIMIT - 1e-8)
            throw new Error("本次實驗已達30分鐘，請匯出後重設");
          if (world.snapshot().collisions.length)
            throw new Error("載具碰撞後請匯出並重設世界");
          running = true;
          last = performance.now();
          publish();
          break;
        case "pause":
          if (inspectionCapture) inspectionCapture.resume = false;
          running = false;
          publish();
          break;
        case "inspectionBegin": {
          if (data.requestId === undefined) throw new Error("拍攝需要請求編號");
          if (inspectionCapture) throw new Error("已有巡檢拍攝正在進行");
          const value = state(),
            v = selected(value, data.vehicleId);
          if (
            !data.value ||
            Object.keys(data.value).sort().join(",") !== "config,targetId"
          )
            throw new Error("無效的巡檢拍攝設定");
          if (
            !value.scene.inspectionTargets.some(
              (target) => target.id === data.value.targetId,
            )
          )
            throw new Error("找不到巡檢目標");
          if (world.inspectionReport(v.id).receipts.length >= 100)
            throw new Error("巡檢拍攝已達100次，請匯出後重設");
          const config = validateCameraConfig(data.value.config),
            captureId = crypto.randomUUID();
          const metadata = cameraMetadata(
            v.snapshot,
            config,
            value.scene.geography,
            {
              sourceVehicleId: v.id,
              worldTick: value.tick,
              captureId,
            },
          );
          inspectionCapture = {
            captureId,
            vehicleId: v.id,
            targetId: data.value.targetId,
            metadata,
            config,
            resume: running,
            startedAt: performance.now(),
            submitting: false,
          };
          running = false;
          publish();
          postMessage({
            type: "apiReply",
            requestId: data.requestId,
            ok: true,
            value: { captureId, metadata, world: value },
          });
          return;
        }
        case "inspectionCancel": {
          const lease = inspectionCapture;
          if (
            !lease ||
            lease.vehicleId !== data.vehicleId ||
            data.value?.captureId !== lease.captureId
          )
            throw new Error("巡檢拍攝已失效");
          releaseInspection(lease);
          publish();
          break;
        }
        case "inspectionSubmit": {
          const lease = inspectionCapture;
          if (
            !lease ||
            lease.vehicleId !== data.vehicleId ||
            data.value?.captureId !== lease.captureId ||
            lease.submitting
          )
            throw new Error("巡檢拍攝已失效或重複提交");
          lease.submitting = true;
          try {
            if (
              !data.value ||
              Object.keys(data.value).sort().join(",") !== "captureId,frame"
            )
              throw new Error("無效的巡檢影像欄位");
            const frame = validateCameraFrame(data.value.frame);
            if (
              JSON.stringify(validateCameraMetadata(frame)) !==
              JSON.stringify(lease.metadata)
            )
              throw new Error("影像不屬於目前巡檢拍攝時刻");
            const sha256 = await hashCameraFrame(frame);
            if (inspectionCapture !== lease) throw new Error("巡檢拍攝已失效");
            const value = {
              captureId: lease.captureId,
              targetId: lease.targetId,
              config: lease.config,
              sha256,
            };
            world.command(lease.vehicleId, "inspection", value);
            recordEvent({
              type: "inspection",
              vehicleId: lease.vehicleId,
              value,
            });
            const report = world.inspectionReport(lease.vehicleId);
            releaseInspection(lease);
            publish();
            postMessage({
              type: "apiReply",
              requestId: data.requestId,
              ok: true,
              value: report,
            });
            return;
          } finally {
            if (inspectionCapture === lease) {
              releaseInspection(lease);
              publish();
            }
          }
        }
        case "inspectionReport": {
          if (data.requestId === undefined)
            throw new Error("此操作需要請求編號");
          postMessage({
            type: "apiReply",
            requestId: data.requestId,
            ok: true,
            value: world.inspectionReport(data.vehicleId),
          });
          return;
        }
        case "speed":
          if (![0.5, 1, 2, 4].includes(data.value))
            throw new Error("無效的模擬速度");
          speed = data.value;
          break;
        case "render":
          if (typeof data.value !== "boolean")
            throw new Error("渲染設定必須為布林值");
          renderEnabled = data.value;
          recordEvent(data);
          break;
        case "scene":
          world.setScene(data.value);
          recordEvent({ ...data, value: world.configuration.scene });
          publish();
          break;
        case "wind":
          world.setWind(data.value);
          recordEvent({ ...data, value: world.configuration.wind });
          publish();
          break;
        case "sensorExport": {
          if (!["json", "csv", "pcd"].includes(data.kind))
            throw new Error("不支援的感測器匯出格式");
          const value = state(),
            v = selected(value, data.vehicleId);
          const text =
            data.kind === "json"
              ? JSON.stringify({
                  modelVersion: MODEL_VERSION,
                  sourceVehicleId: v.id,
                  worldTick: value.tick,
                  config: v.setup.sensors,
                  geography: value.scene.geography,
                  sensors: v.sensors,
                })
              : cloud(data.kind, v, value);
          postMessage({
            type: "sensorExport",
            kind: data.kind,
            text,
            vehicleId: v.id,
          });
          break;
        }
        case "radarExport": {
          if (data.requestId === undefined)
            throw new Error("此操作需要程式介面請求編號");
          if (!["json", "csv"].includes(data.value))
            throw new Error("不支援的雷達匯出格式");
          const value = state(),
            v = selected(value, data.vehicleId);
          const text = exportRadarData(
            data.value,
            v.sensors.radar,
            { sourceVehicleId: v.id, worldTick: value.tick },
            v.setup.sensors,
          );
          postMessage({
            type: "apiReply",
            requestId: data.requestId,
            ok: true,
            value: {
              format: data.value,
              sourceVehicleId: v.id,
              exportWorldTick: value.tick,
              time: v.sensors.radar.time,
              detectionCount: v.sensors.radar.detections.length,
              text,
            },
          });
          return;
        }
        case "proximityExport": {
          if (data.requestId === undefined)
            throw new Error("此操作需要程式介面請求編號");
          const value = state(), v = selected(value, data.vehicleId);
          const text = exportProximityData(data.value, v.sensors.proximity, {
            sourceVehicleId: v.id, worldTick: value.tick,
          });
          postMessage({
            type: "apiReply", requestId: data.requestId, ok: true,
            value: {
              format: data.value, sourceVehicleId: v.id,
              exportWorldTick: value.tick, time: v.sensors.proximity.time, text,
            },
          });
          return;
        }
        case "pointCloudExport": {
          if (data.requestId === undefined)
            throw new Error("此操作需要程式介面請求編號");
          if (!["csv", "pcd"].includes(data.value))
            throw new Error("不支援的點雲匯出格式");
          const value = state(),
            v = selected(value, data.vehicleId),
            reading = v.sensors.lidar;
          postMessage({
            type: "apiReply",
            requestId: data.requestId,
            ok: true,
            value: {
              format: data.value,
              frame: "NED",
              units: "m",
              sourceVehicleId: v.id,
              worldTick: value.tick,
              time: reading.time,
              pointCount: reading.rangesM.filter((x) => x !== null).length,
              text: cloud(data.value, v, value),
            },
          });
          return;
        }
        case "inspect":
        case "inspectWorld":
        case "inspectSensors":
        case "inspectScene":
        case "inspectGimbal":
        case "apiExport": {
          if (data.requestId === undefined)
            throw new Error("此操作需要程式介面請求編號");
          const value = state();
          const result =
            data.type === "inspect"
              ? inspectedState(data.vehicleId)
              : data.type === "inspectWorld"
                ? inspectedState()
                : data.type === "inspectScene"
                  ? value.scene
                  : data.type === "inspectSensors"
                    ? selected(value, data.vehicleId).sensors
                    : data.type === "inspectGimbal"
                      ? selected(value, data.vehicleId).snapshot.gimbal
                      : recording();
          postMessage({
            type: "apiReply",
            requestId: data.requestId,
            ok: true,
            value: result,
          });
          return;
        }
        case "export":
          if (!["csv", "json"].includes(data.kind))
            throw new Error("不支援的紀錄格式");
          postMessage({
            type: "recording",
            kind: data.kind,
            recording: recording(),
          });
          break;
        default:
          throw new Error("未知的模擬操作");
      }
    if (data.requestId !== undefined)
      postMessage({
        type: "apiReply",
        requestId: data.requestId,
        ok: true,
        value: inspectedState(replyVehicle ?? activeVehicleId),
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
  const now = performance.now(),
    elapsed = Math.max(0, (now - last) / 1000);
  last = now;
  if (inspectionCapture && now - inspectionCapture.startedAt >= 15000) {
    releaseInspection();
    publish("inspection-timeout");
  }
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
        const value = world.step(DT);
        termination = null;
        accumulator -= DT;
        if (value.tick % 10 === 0 || value.collisions.length) capture();
        if (value.collisions.length) {
          running = false;
          termination = { reason: "vehicle-collision", time: value.time };
          publish("vehicle-collision");
        } else if (storageLimited) publish("storage-limit");
        else if (value.time >= LIMIT - 1e-8) {
          running = false;
          capture();
          publish("limit");
        }
      }
      if (now - lastPublish >= 50) publish();
    } catch (error) {
      running = false;
      if (error.code === "TERRAIN_COVERAGE")
        termination = {
          reason: "terrain-coverage",
          time: world.snapshot().time,
        };
      capture();
      postMessage({
        type: "error",
        message:
          error.code === "TERRAIN_COVERAGE"
            ? "載具已到達高程資料範圍，全體模擬已暫停。請調整航線或載入更大範圍的地形。"
            : `數值模擬已停止：${error.message}`,
      });
      publish(error.code === "TERRAIN_COVERAGE" ? "terrain-limit" : undefined);
    }
  }
  setTimeout(tick, 10);
}
capture();
publish("reset");
tick();
