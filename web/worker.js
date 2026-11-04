import { createSimulation, validateConfig } from "./sim.js";

const DT = 0.005;
const LIMIT = 1800;
let config = validateConfig();
let sim = createSimulation(config);
let scenario = "hover";
let running = false;
let speed = 1;
let accumulator = 0;
let last = performance.now();
let lastPublish = last;
let wallSeconds = 0;
let skippedWallSeconds = 0;
let lateness = [];
let lateTicks = 0;
let samples = [sim.snapshot()];
let pending = [...samples];
let events = [];
let createdAt = new Date().toISOString();

function publish(message) {
  const sorted = [...lateness].sort((a, b) => a - b);
  postMessage({
    type: "state",
    running,
    snapshot: sim.snapshot(),
    samples: pending,
    sampleCount: samples.length,
    timing: {
      realTimeFactor: wallSeconds ? sim.snapshot().time / wallSeconds : 0,
      p95LateMs: sorted.length
        ? sorted[Math.floor((sorted.length - 1) * 0.95)]
        : 0,
      lateTicks,
      skippedWallSeconds,
    },
    message,
  });
  pending = [];
  lastPublish = performance.now();
}

function reset(nextConfig, nextScenario) {
  const validated = validateConfig(nextConfig);
  if (!["hover", "step", "gust"].includes(nextScenario))
    throw new Error("未知的實驗情境");
  config = validated;
  scenario = nextScenario;
  sim = createSimulation(config);
  sim.setScenario(scenario);
  running = false;
  accumulator = 0;
  wallSeconds = 0;
  skippedWallSeconds = 0;
  lateness = [];
  lateTicks = 0;
  samples = [sim.snapshot()];
  pending = [...samples];
  events = [];
  createdAt = new Date().toISOString();
  last = performance.now();
  publish("reset");
}

onmessage = ({ data }) => {
  try {
    switch (data.type) {
      case "reset":
        reset(data.config, data.scenario);
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
        events.push({
          time: sim.snapshot().time,
          type: "targets",
          value: { ...data.value },
        });
        publish();
        break;
      case "gust":
        sim.gust();
        events.push({ time: sim.snapshot().time, type: "gust" });
        break;
      case "export": {
        const final = sim.snapshot();
        const exported = [...samples];
        if (final.time > exported.at(-1).time + 1e-8) exported.push(final);
        else exported[exported.length - 1] = final;
        postMessage({
          type: "recording",
          kind: data.kind,
          recording: {
            format: "flight-lab",
            version: 1,
            createdAt,
            config,
            scenario,
            dt: DT,
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
        const snapshot = sim.step(DT);
        accumulator -= DT;
        if (snapshot.metrics.steps % 10 === 0) {
          samples.push(snapshot);
          pending.push(snapshot);
        }
        if (snapshot.time >= LIMIT - 1e-8) {
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
