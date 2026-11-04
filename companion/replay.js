import { readFile, stat } from "node:fs/promises";
import { resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { createSimulation, validateConfig } from "../web/sim.js";
import { mapActuators, OUTPUT_MAP } from "./px4-protocol.js";

const MAX_FILE_BYTES = 128 * 1024 * 1024;
const object = (value) => {
  if (
    !value ||
    typeof value !== "object" ||
    Array.isArray(value) ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value))
  )
    throw new Error("Invalid SITL recording object");
  return value;
};
const integer = (value, min, max) => {
  if (!Number.isSafeInteger(value) || value < min || value > max)
    throw new Error("Invalid SITL recording integer");
  return value;
};
function vector(value, length) {
  if (
    !Array.isArray(value) ||
    value.length !== length ||
    !Array.from(value).every((v) => Number.isFinite(v))
  )
    throw new Error("Invalid SITL recording vector");
  return value;
}
function maximumError(actual, expected) {
  vector(expected, actual.length);
  return Math.max(...actual.map((v, i) => Math.abs(v - expected[i])));
}

// Replays accepted actuator inputs through the plant; it does not run or replace PX4.
export function replayPx4Recording(value) {
  const record = object(value),
    firmware = object(record.firmware);
  if (
    record.format !== "flight-lab-px4" ||
    record.version !== 1 ||
    record.dt !== 0.004 ||
    firmware.verified !== true ||
    firmware.version !== "1.17.0" ||
    firmware.sourceSha !== "d6f12ad1c4f70ad3230afd7d86e971421e02fef4" ||
    !Array.isArray(record.outputMap) ||
    record.outputMap.length !== 4 ||
    record.outputMap.some((v, i) => v !== OUTPUT_MAP[i])
  )
    throw new Error("Unsupported SITL recording format or firmware");
  const config = validateConfig(object(record.config));
  if (config.airframe !== "quadrotor" || config.startOnGround !== true)
    throw new Error("SITL replay requires the external ground-start quadrotor");
  const bootstrap = integer(record.bootstrapTicks ?? 0, 0, 450000);
  if (
    !Array.isArray(record.actuators) ||
    record.actuators.length + bootstrap > 450001 ||
    !Array.isArray(record.samples) ||
    record.samples.length > 450002 ||
    !Array.isArray(record.events) ||
    record.events.length > 100000
  )
    throw new Error("SITL recording exceeds limits");
  let previousEvent = -1;
  const events = Array.from(record.events, (event) => {
    object(event);
    const timeUs = Math.round(event.time * 1e6);
    integer(timeUs, 0, 1800010000);
    if (
      !Number.isFinite(event.time) ||
      timeUs < previousEvent ||
      ![
        "arm",
        "takeoff",
        "land",
        "disarm",
        "pause",
        "resume",
        "stop",
        "gust",
      ].includes(event.action)
    )
      throw new Error("Invalid SITL recording event");
    previousEvent = timeUs;
    return { ...event, timeUs };
  });
  let previousSample = -1;
  for (const sample of record.samples) {
    object(sample);
    const tick = Math.round(sample.time * 1e6);
    integer(tick, 0, 1800010000);
    if (!Number.isFinite(sample.time) || tick < previousSample)
      throw new Error("Invalid SITL sample ordering");
    previousSample = tick;
  }
  const sim = createSimulation(config, { controlSource: "external" });
  let sequence = 0,
    sampleIndex = 0,
    eventIndex = 0,
    maxPositionErrorM = 0,
    maxQuaternionError = 0,
    maxMotorError = 0;
  function checkSamples() {
    const actual = sim.snapshot(),
      timeUs = sim.readSensors().timeUs;
    while (
      sampleIndex < record.samples.length &&
      Math.round(record.samples[sampleIndex].time * 1e6) === timeUs
    ) {
      const sample = record.samples[sampleIndex++];
      if (sample.controlSource !== "external" || sample.armed !== actual.armed)
        throw new Error("SITL sample source or armed state mismatch");
      maxPositionErrorM = Math.max(
        maxPositionErrorM,
        maximumError(actual.position, vector(sample.position, 3)),
      );
      maxQuaternionError = Math.max(
        maxQuaternionError,
        maximumError(actual.quaternion, vector(sample.quaternion, 4)),
      );
      maxMotorError = Math.max(
        maxMotorError,
        maximumError(actual.motors, vector(sample.motors, 4)),
      );
    }
    if (
      sampleIndex < record.samples.length &&
      Math.round(record.samples[sampleIndex].time * 1e6) < timeUs
    )
      throw new Error("SITL sample is not on an integrated tick");
  }
  function applyEvents() {
    const timeUs = sim.readSensors().timeUs;
    while (eventIndex < events.length && events[eventIndex].timeUs <= timeUs) {
      const event = events[eventIndex++];
      if (event.timeUs !== timeUs)
        throw new Error("SITL event is not on an integrated tick");
      if (event.action === "gust") sim.gust();
    }
  }
  checkSamples();
  for (let i = 0; i < bootstrap; i++) {
    applyEvents();
    sim.applyExternalActuators({
      sequence: ++sequence,
      timeUs: sim.readSensors().timeUs,
      armed: false,
      rotorSpeedCommands: [0, 0, 0, 0],
    });
    sim.stepExternal(record.dt);
    checkSamples();
  }
  for (const input of record.actuators) {
    object(input);
    applyEvents();
    const timeUs = sim.readSensors().timeUs;
    if (
      integer(input.sequence, 1, 450001) !== ++sequence ||
      integer(input.timeUs, 0, 1800010000) !== timeUs ||
      integer(input.rawTimeUs, 0, Number.MAX_SAFE_INTEGER) !== timeUs + 1000000
    )
      throw new Error("SITL actuator sequence or timestamp mismatch");
    integer(input.mode, 0, 255);
    integer(input.flags, 0, Number.MAX_SAFE_INTEGER);
    const mapped = mapActuators({
      timeUsec: BigInt(input.rawTimeUs),
      mode: input.mode,
      flags: BigInt(input.flags),
      controls: vector(input.rawControls, 16),
    });
    if (
      input.armed !== mapped.armed ||
      maximumError(
        mapped.rotorSpeedCommands,
        vector(input.rotorSpeedCommands, 4),
      ) > 1e-12
    )
      throw new Error("SITL motor mapping mismatch");
    sim.applyExternalActuators({
      sequence,
      timeUs,
      armed: input.armed,
      rotorSpeedCommands: input.rotorSpeedCommands,
    });
    sim.stepExternal(record.dt);
    checkSamples();
  }
  applyEvents();
  if (eventIndex !== events.length || sampleIndex !== record.samples.length)
    throw new Error("SITL recording has future samples or events");
  const final = object(record.finalSnapshot),
    actual = sim.snapshot();
  if (
    Math.round(final.time * 1e6) !== sim.readSensors().timeUs ||
    final.controlSource !== "external" ||
    final.armed !== actual.armed
  )
    throw new Error("SITL final state mismatch");
  maxPositionErrorM = Math.max(
    maxPositionErrorM,
    maximumError(actual.position, vector(final.position, 3)),
  );
  maxQuaternionError = Math.max(
    maxQuaternionError,
    maximumError(actual.quaternion, vector(final.quaternion, 4)),
  );
  maxMotorError = Math.max(
    maxMotorError,
    maximumError(actual.motors, vector(final.motors, 4)),
  );
  if (Math.max(maxPositionErrorM, maxQuaternionError, maxMotorError) > 1e-8)
    throw new Error("SITL plant replay differs from recorded trajectory");
  return {
    passed: true,
    bootstrapTicks: bootstrap,
    acceptedActuators: record.actuators.length,
    checkedSamples: sampleIndex,
    simSeconds: actual.time,
    maxPositionErrorM,
    maxQuaternionError,
    maxMotorError,
  };
}

if (
  process.argv[1] &&
  resolve(process.argv[1]) === fileURLToPath(import.meta.url)
) {
  const filename = process.argv[2];
  if (!filename) throw new Error("Pass a PX4 SITL recording JSON path");
  if ((await stat(filename)).size > MAX_FILE_BYTES)
    throw new Error("SITL import exceeds 128 MiB");
  console.log(
    JSON.stringify(
      replayPx4Recording(JSON.parse(await readFile(filename, "utf8"))),
      null,
      2,
    ),
  );
}
