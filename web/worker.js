import { createSimulation, validateConfig } from "./sim.js";
import { createTrainer } from "./training.js";
import {
  RECORDING_VERSION,
  compactSnapshot,
  validateSetup,
} from "./recording.js";

const DT = 0.005;
const LIMIT = 1800;
let config = validateConfig();
let setup = validateSetup(undefined, config);
let initialSetup = structuredClone(setup);
let sim = createSimulation(config);
let trainer = createTrainer(setup.training);
let training = trainer.update(sim.snapshot());
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
    samples: pending,
    sampleCount: samples.length,
    setup,
    timing: {
      ...runtime(),
      realTimeFactor: wallSeconds ? state.time / wallSeconds : 0,
    },
    message,
  });
  pending = [];
  lastPublish = performance.now();
}

function reset(
  nextConfig,
  nextScenario,
  nextSetup = { ...setup, mode: "attitude", mission: [] },
) {
  const validated = validateConfig(nextConfig);
  if (!["hover", "step", "gust"].includes(nextScenario))
    throw new Error("未知的實驗情境");
  const validatedSetup = validateSetup(nextSetup, validated);
  const next = createSimulation(validated);
  next.setScenario(nextScenario);
  next.setObstacles(validatedSetup.obstacles);
  if (validatedSetup.mission.length) next.setMission(validatedSetup.mission);
  next.setMode(validatedSetup.mode);
  const nextTrainer = createTrainer(validatedSetup.training);
  const nextTraining = nextTrainer.update(next.snapshot());
  config = validated;
  setup = validatedSetup;
  initialSetup = structuredClone(setup);
  scenario = nextScenario;
  sim = next;
  trainer = nextTrainer;
  training = nextTraining;
  running = false;
  accumulator = 0;
  wallSeconds = 0;
  skippedWallSeconds = 0;
  lateness = [];
  lateTicks = 0;
  samples = [compactSnapshot(snapshot())];
  pending = [...samples];
  events = [];
  createdAt = new Date().toISOString();
  last = performance.now();
  publish("reset");
}

function recordEvent(data) {
  const event = { time: sim.snapshot().time, type: data.type };
  if (Object.hasOwn(data, "value")) event.value = structuredClone(data.value);
  if (Object.hasOwn(data, "fault")) event.fault = data.fault;
  events.push(event);
  const current = compactSnapshot(snapshot());
  if (Math.abs(samples.at(-1).time - current.time) < 1e-9) {
    samples[samples.length - 1] = current;
    if (pending.length && Math.abs(pending.at(-1).time - current.time) < 1e-9)
      pending[pending.length - 1] = current;
    else pending.push(current);
  }
}

onmessage = ({ data }) => {
  try {
    if (!data || typeof data.type !== "string")
      throw new Error("無效的模擬操作");
    if (
      events.length >= 100000 &&
      !["reset", "pause", "export"].includes(data.type)
    )
      throw new Error("事件紀錄已達上限，請先匯出再重設");
    switch (data.type) {
      case "reset":
        reset(data.config, data.scenario, data.setup);
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
      case "obstacles":
        sim.setObstacles(data.value);
        setup.obstacles = structuredClone(data.value);
        recordEvent(data);
        publish();
        break;
      case "training": {
        const next = createTrainer(data.value);
        const nextTraining = next.update(sim.snapshot());
        trainer = next;
        training = nextTraining;
        setup.training = data.value;
        recordEvent(data);
        publish();
        break;
      }
      case "environment":
        setup.environment = validateSetup(
          { ...setup, environment: data.value },
          config,
        ).environment;
        recordEvent({ ...data, value: setup.environment });
        publish();
        break;
      case "render":
        if (typeof data.value !== "boolean")
          throw new Error("渲染設定必須為布林值");
        renderEnabled = data.value;
        recordEvent(data);
        break;
      case "export": {
        if (!["csv", "json"].includes(data.kind))
          throw new Error("不支援的紀錄格式");
        const final = compactSnapshot(snapshot());
        const exported = [...samples];
        if (final.time > exported.at(-1).time + 1e-8) exported.push(final);
        else exported[exported.length - 1] = final;
        postMessage({
          type: "recording",
          kind: data.kind,
          recording: {
            format: "flight-lab",
            version: RECORDING_VERSION,
            modelVersion: "0.2.0",
            createdAt,
            config,
            scenario,
            dt: DT,
            setup: initialSetup,
            runtime: runtime(),
            events,
            samples: exported,
          },
        });
        break;
      }
      default:
        throw new Error("未知的模擬操作");
    }
  } catch (error) {
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
        accumulator -= DT;
        if (state.metrics.steps % 10 === 0) {
          const sample = compactSnapshot({
            ...state,
            training: structuredClone(training),
          });
          samples.push(sample);
          pending.push(sample);
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
