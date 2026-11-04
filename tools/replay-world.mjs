import { readFile } from "node:fs/promises";
import { pathToFileURL } from "node:url";
import { createWorld } from "../web/world.js";
import { validateRecording } from "../web/recording.js";

export function replayWorldRecording(input) {
  const record = validateRecording(input),
    world = createWorld(record.world);
  let state = world.snapshot(),
    eventIndex = 0;
  const maximum = {
    positionM: 0,
    quaternion: 0,
    motors: 0,
    allState: 0,
    sensors: 0,
  };
  function difference(actual, expected, path = "") {
    if (typeof expected === "number") {
      if (!Number.isFinite(actual))
        throw new Error(`Non-numeric replay value: ${path}`);
      return Math.abs(actual - expected);
    }
    if (expected === null || typeof expected !== "object") {
      if (actual !== expected)
        throw new Error(`Replay value disagrees: ${path}`);
      return 0;
    }
    if (
      !actual ||
      typeof actual !== "object" ||
      Object.keys(actual).sort().join("/") !==
        Object.keys(expected).sort().join("/")
    )
      throw new Error(`Replay fields disagree: ${path}`);
    return Object.keys(expected).reduce(
      (max, key) =>
        Math.max(max, difference(actual[key], expected[key], `${path}/${key}`)),
      0,
    );
  }
  function applyEvents() {
    while (
      eventIndex < record.events.length &&
      record.events[eventIndex].tick === state.tick
    ) {
      const e = record.events[eventIndex++];
      if (e.type === "scene") state = world.setScene(e.value);
      else if (e.type === "wind") state = world.setWind(e.value);
      else if (e.type !== "render")
        state = world.command(e.vehicleId, e.type, e.value, e.fault);
    }
  }
  for (const [index, frame] of record.samples.entries()) {
    while (state.tick < frame.tick) {
      applyEvents();
      state = world.step();
    }
    applyEvents();
    for (const [i, v] of state.vehicles.entries()) {
      const expected = frame.vehicles[i].snapshot;
      maximum.positionM = Math.max(
        maximum.positionM,
        difference(v.snapshot.position, expected.position),
      );
      maximum.quaternion = Math.max(
        maximum.quaternion,
        difference(v.snapshot.quaternion, expected.quaternion),
      );
      maximum.motors = Math.max(
        maximum.motors,
        difference(v.snapshot.motors, expected.motors),
      );
      maximum.allState = Math.max(
        maximum.allState,
        difference(v.snapshot, expected),
      );
      maximum.sensors = Math.max(
        maximum.sensors,
        difference(v.sensors, record.sensorFrames[index].vehicles[i].sensors),
      );
    }
    difference(state.collisions, frame.collisions);
  }
  return {
    passed: Math.max(...Object.values(maximum)) < 1e-9,
    vehicles: record.world.vehicles.length,
    frames: record.samples.length,
    ticks: state.tick,
    simSeconds: state.time,
    maximum,
  };
}
if (
  process.argv[1] &&
  import.meta.url === pathToFileURL(process.argv[1]).href
) {
  if (process.argv.length !== 3)
    throw new Error(
      "Usage: node tools/replay-world.mjs /absolute/path/to/world-recording.json",
    );
  const summary = replayWorldRecording(
    JSON.parse(await readFile(process.argv[2], "utf8")),
  );
  console.log(JSON.stringify(summary, null, 2));
  if (!summary.passed) process.exitCode = 1;
}
