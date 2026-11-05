import { MODEL_VERSION } from "../web/recording.js";
import { readFile, stat } from "node:fs/promises";
import { resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { createSimulation, validateConfig } from "../web/sim.js";
import { mapActuators, OUTPUT_MAP } from "./px4-protocol.js";
import {
  ARDUPILOT_DT,
  ARDUPILOT_VERSION,
  ARDUPILOT_SOURCE_SHA,
  ARDUPILOT_OUTPUT_MAP,
  mapServoSpeeds,
} from "./ardupilot-protocol.js";

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

// Replays accepted actuator inputs through the plant; it does not rerun a flight controller.
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
    !Array.from(record.outputMap).every((v, i) => v === OUTPUT_MAP[i])
  )
    throw new Error("Unsupported SITL recording format or firmware");
  const bootstrap = integer(record.bootstrapTicks ?? 0, 0, 450000);
  return replayPlant(record, bootstrap, (input, timeUs) => {
    if (
      integer(input.rawTimeUs, 0, Number.MAX_SAFE_INTEGER) !==
      timeUs + 1000000
    )
      throw new Error("SITL actuator timestamp mismatch");
    integer(input.mode, 0, 255);
    integer(input.flags, 0, Number.MAX_SAFE_INTEGER);
    return mapActuators({
      timeUsec: BigInt(input.rawTimeUs),
      mode: input.mode,
      flags: BigInt(input.flags),
      controls: vector(input.rawControls, 16),
    });
  });
}

export function replayArduPilotRecording(value) {
  const record = object(value),
    firmware = object(record.firmware);
  if (
    record.format !== "flight-lab-ardupilot" ||
    record.version !== 1 ||
    record.dt !== ARDUPILOT_DT ||
    object(record.clock).stepUs !== 1000 ||
    firmware.verified !== true ||
    firmware.version !== ARDUPILOT_VERSION ||
    firmware.reportedGitHash !== ARDUPILOT_SOURCE_SHA.slice(0, 8) ||
    firmware.expectedSourceSha !== ARDUPILOT_SOURCE_SHA ||
    firmware.sourceSha !== null ||
    record.bootstrapIncludedInActuators !== true ||
    !Array.isArray(record.outputMap) ||
    record.outputMap.length !== 4 ||
    !Array.from(record.outputMap).every(
      (value, i) => value === ARDUPILOT_OUTPUT_MAP[i],
    )
  )
    throw new Error(
      "Unsupported ArduPilot recording format or firmware self-report",
    );
  integer(record.bootstrapTicks, 0, 1800000);
  let bootstrapTicks = 0;
  const result = replayPlant(record, 0, (input) => {
    if (
      integer(input.frameCount, 0, 0xffffffff) !== input.sequence - 1 ||
      !(
        input.frameRate === 1000 ||
        (input.sequence === 1 && input.frameRate === 1200)
      ) ||
      typeof input.armedKnown !== "boolean" ||
      typeof input.heartbeatArmed !== "boolean" ||
      (input.heartbeatAgeMs !== null &&
        (!Number.isFinite(input.heartbeatAgeMs) || input.heartbeatAgeMs < 0))
    )
      throw new Error("Invalid ArduPilot frame or heartbeat metadata");
    const pwm = vector(input.pwm, 16);
    if (
      !pwm.every(
        (value) =>
          Number.isInteger(value) &&
          (value === 0 || (value >= 800 && value <= 2200)),
      )
    )
      throw new Error("Invalid ArduPilot raw PWM");
    if (!input.armedKnown) bootstrapTicks++;
    const armed = input.armedKnown && input.heartbeatArmed;
    return { armed, rotorSpeedCommands: mapServoSpeeds({ pwm }, armed) };
  });
  if (bootstrapTicks !== record.bootstrapTicks)
    throw new Error("ArduPilot bootstrap count mismatch");
  return { ...result, bootstrapTicks };
}

function replayPlant(record, bootstrap, mapInput) {
  if (record.modelVersion !== MODEL_VERSION)
    throw new Error(`Unsupported SITL simulation model: ${MODEL_VERSION} required`);
  const config = validateConfig(object(record.config));
  if (config.airframe !== "quadrotor" || config.startOnGround !== true)
    throw new Error("SITL replay requires the external ground-start quadrotor");
  const maxTicks = Math.round(1800 / record.dt) + 1;
  if (
    !Array.isArray(record.actuators) ||
    record.actuators.length + bootstrap > maxTicks ||
    !Array.isArray(record.samples) ||
    record.samples.length > maxTicks + 1 ||
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
        "mission",
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
    maxMotorError = 0,
    maxBatteryError = 0,
    maxPropulsionError = 0;
  function checkPower(actual, sample) {
    if (sample.collisionPrevention !== null)
      throw new Error("Native plant requires disabled collision prevention");
    const batteryRanges = {
      soc: [0, 1], voltage: [0, 100], openCircuitVoltage: [0, 100], motorSpeedScale: [0, 1],
      current: [0, 1e6], power: [0, 1e6], usedWh: [0, 1000], remainingSeconds: [0, 1e9],
      low: null, critical: null, powerLimited: null,
    };
    for (const [name, ranges] of [['battery', batteryRanges], ['propulsion', { command: [0, 1], motor: [0, 1], thrustN: [0, Infinity] }]]) {
      const value = object(sample[name]);
      if (Object.keys(value).sort().join() !== Object.keys(ranges).sort().join())
        throw new Error(`Invalid SITL ${name} fields`);
      for (const [key, range] of Object.entries(ranges)) {
        if (range === null) {
          if (typeof value[key] !== 'boolean' || value[key] !== actual[name][key])
            throw new Error(`SITL ${name}.${key} mismatch`);
        } else {
          if (!Number.isFinite(value[key]) || value[key] < range[0] || value[key] > range[1])
            throw new Error(`Invalid SITL ${name}.${key}`);
          const error = Math.abs(actual[name][key] - value[key]);
          if (name === 'battery') maxBatteryError = Math.max(maxBatteryError, error);
          else maxPropulsionError = Math.max(maxPropulsionError, error);
        }
      }
    }
  }
  function checkSamples() {
    const actual = sim.snapshot(),
      timeUs = sim.readSensors().timeUs;
    while (
      sampleIndex < record.samples.length &&
      Math.round(record.samples[sampleIndex].time * 1e6) === timeUs
    ) {
      const sample = record.samples[sampleIndex++];
      checkPower(actual, sample);
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
      integer(input.sequence, 1, maxTicks) !== ++sequence ||
      integer(input.timeUs, 0, 1800010000) !== timeUs
    )
      throw new Error("SITL actuator sequence or timestamp mismatch");
    const mapped = mapInput(input, timeUs);
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
  checkPower(actual, final);
  if (Math.max(maxPositionErrorM, maxQuaternionError, maxMotorError, maxBatteryError, maxPropulsionError) > 1e-8)
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
    maxBatteryError,
    maxPropulsionError,
  };
}

if (
  process.argv[1] &&
  resolve(process.argv[1]) === fileURLToPath(import.meta.url)
) {
  const filename = process.argv[2];
  if (!filename)
    throw new Error("Pass a PX4 or ArduPilot SITL recording JSON path");
  if ((await stat(filename)).size > MAX_FILE_BYTES)
    throw new Error("SITL import exceeds 128 MiB");
  const record = JSON.parse(await readFile(filename, "utf8"));
  const replay =
    record?.format === "flight-lab-ardupilot"
      ? replayArduPilotRecording
      : replayPx4Recording;
  console.log(JSON.stringify(replay(record), null, 2));
}
