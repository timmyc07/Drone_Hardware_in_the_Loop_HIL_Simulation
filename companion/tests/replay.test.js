import { MODEL_VERSION } from "../../web/recording.js";
import test from "node:test";
import assert from "node:assert/strict";
import { createSimulation, validateConfig } from "../../web/sim.js";
import { OUTPUT_MAP } from "../px4-protocol.js";
import { replayPx4Recording, replayArduPilotRecording } from "../replay.js";
import {
  ARDUPILOT_OUTPUT_MAP,
  ARDUPILOT_SOURCE_SHA,
  mapServoSpeeds,
} from "../ardupilot-protocol.js";

function fixture(configOverrides = {}) {
  const config = validateConfig({ startOnGround: true, ...configOverrides }),
    sim = createSimulation(config, { controlSource: "external" });
  const record = {
    format: "flight-lab-px4",
    version: 1,
    modelVersion: MODEL_VERSION,
    firmware: {
      verified: true,
      version: "1.17.0",
      sourceSha: "d6f12ad1c4f70ad3230afd7d86e971421e02fef4",
    },
    config,
    dt: 0.004,
    outputMap: OUTPUT_MAP,
    bootstrapTicks: 2,
    actuators: [],
    samples: [sim.snapshot()],
    events: [{ time: 0, action: "mission" }],
  };
  let sequence = 0;
  for (let i = 0; i < 2; i++) {
    sim.applyExternalActuators({
      sequence: ++sequence,
      timeUs: sim.readSensors().timeUs,
      armed: false,
      rotorSpeedCommands: [0, 0, 0, 0],
    });
    sim.stepExternal(0.004);
  }
  for (let i = 0; i < 200; i++) {
    const timeUs = sim.readSensors().timeUs;
    if (i === 50) {
      sim.gust();
      record.events.push({ time: sim.snapshot().time, action: "gust" });
    }
    const armed = i < 160,
      rawControls = [0.65, 0.65, 0.65, 0.65, ...Array(12).fill(0)],
      rotorSpeedCommands = armed ? Array(4).fill(0.65) : Array(4).fill(0);
    const input = {
      sequence: ++sequence,
      timeUs,
      armed,
      rotorSpeedCommands,
      rawTimeUs: timeUs + 1000000,
      rawControls,
      mode: armed ? 128 : 0,
      flags: 1,
    };
    sim.applyExternalActuators({ sequence, timeUs, armed, rotorSpeedCommands });
    sim.stepExternal(0.004);
    record.actuators.push(input);
    if (i % 10 === 0) record.samples.push(sim.snapshot());
  }
  record.finalSnapshot = sim.snapshot();
  return record;
}
test("accepted PX4 actuator replay reconstructs every recorded plant sample including a gust", () => {
  const result = replayPx4Recording(JSON.parse(JSON.stringify(fixture())));
  assert.equal(result.passed, true);
  assert.equal(result.acceptedActuators, 200);
  assert.equal(result.bootstrapTicks, 2);
  assert.equal(result.maxPositionErrorM, 0);
  assert.equal(result.maxQuaternionError, 0);
  assert.equal(result.maxMotorError, 0);
  assert.equal(result.maxBatteryError, 0);
  assert.equal(result.maxPropulsionError, 0);
});
test("SITL replay rejects corrupted provenance, clocks, motor maps and trajectories", () => {
  const mutations = [
    (r) => (r.version = 2),
    (r) => (r.firmware.verified = false),
    (r) => (r.firmware.sourceSha = "unknown"),
    (r) => (r.dt = 0.005),
    (r) => (r.outputMap = [0, 1, 2, 3]),
    (r) => r.actuators[0].sequence++,
    (r) => r.actuators[0].timeUs++,
    (r) => r.actuators[0].rawTimeUs++,
    (r) => (r.actuators[0].rotorSpeedCommands[0] = 0.5),
    (r) => (r.actuators[0].rawControls[0] = NaN),
    (r) => (r.actuators[0].flags = 0),
    (r) => (r.samples[0].position[0] = 1),
    (r) => (r.samples[1].time += 0.001),
    (r) => (r.events[0].time += 0.001),
    (r) => (r.events[0].action = "unsupported"),
    (r) => (r.finalSnapshot.motors[0] = 0.5),
    (r) => (r.finalSnapshot.armed = !r.finalSnapshot.armed),
  ];
  for (const change of mutations) {
    const record = fixture();
    change(record);
    assert.throws(() => replayPx4Recording(record));
  }
});

function ardupilotFixture(configOverrides = {}) {
  const config = validateConfig({ startOnGround: true, ...configOverrides }),
    sim = createSimulation(config, { controlSource: "external" });
  const record = {
    format: "flight-lab-ardupilot",
    version: 1,
    modelVersion: MODEL_VERSION,
    firmware: {
      verified: true,
      version: "4.7.2",
      reportedGitHash: "fb773567",
      expectedSourceSha: ARDUPILOT_SOURCE_SHA,
      sourceSha: null,
    },
    config,
    dt: 0.001,
    clock: { stepUs: 1000 },
    outputMap: ARDUPILOT_OUTPUT_MAP,
    bootstrapIncludedInActuators: true,
    bootstrapTicks: 12,
    actuators: [],
    samples: [sim.snapshot()],
    events: [{ time: 0, action: "mission" }],
  };
  for (let i = 0; i < 800; i++) {
    if (i === 200) {
      sim.gust();
      record.events.push({ time: sim.snapshot().time, action: "gust" });
    }
    const armedKnown = i >= record.bootstrapTicks,
      heartbeatArmed = armedKnown && i < 650,
      pwm = [1651, 1649, 1650, 1652, ...Array(12).fill(0)],
      armed = armedKnown && heartbeatArmed,
      input = {
        sequence: i + 1,
        timeUs: sim.readSensors().timeUs,
        frameCount: i,
        frameRate: i === 0 ? 1200 : 1000,
        pwm,
        armedKnown,
        heartbeatArmed,
        heartbeatAgeMs: armedKnown ? 10 : null,
        armed,
        rotorSpeedCommands: mapServoSpeeds({ pwm }, armed),
      };
    sim.applyExternalActuators({
      sequence: input.sequence,
      timeUs: input.timeUs,
      armed: input.armed,
      rotorSpeedCommands: input.rotorSpeedCommands,
    });
    sim.stepExternal(record.dt);
    record.actuators.push(input);
    if (i % 50 === 49) record.samples.push(sim.snapshot());
  }
  record.finalSnapshot = sim.snapshot();
  return record;
}

test("ArduPilot replay integrates bootstrap once and checks raw asymmetric PWM through gust and disarm", () => {
  const result = replayArduPilotRecording(
    JSON.parse(JSON.stringify(ardupilotFixture())),
  );
  assert.equal(result.acceptedActuators, 800);
  assert.equal(result.bootstrapTicks, 12);
  assert.equal(Math.round(result.simSeconds * 1e6), 800000);
  assert.equal(result.maxPositionErrorM, 0);
  assert.equal(result.maxQuaternionError, 0);
  assert.equal(result.maxMotorError, 0);
  assert.equal(result.maxBatteryError, 0);
  assert.equal(result.maxPropulsionError, 0);
});

test("ArduPilot replay rejects invalid clocks, self-reports, frames, PWM, heartbeats and trajectory", () => {
  const mutations = [
    (r) => (r.firmware.reportedGitHash = "unknown"),
    (r) => (r.firmware.sourceSha = ARDUPILOT_SOURCE_SHA),
    (r) => (r.firmware.verified = false),
    (r) => (r.clock.stepUs = 4000),
    (r) => (r.dt = 0.004),
    (r) => (r.bootstrapIncludedInActuators = false),
    (r) => r.bootstrapTicks++,
    (r) => (r.actuators[1].frameRate = 1200),
    (r) => (r.actuators[0].frameRate = 400),
    (r) => r.actuators[1].frameCount++,
    (r) => r.actuators[1].sequence++,
    (r) => r.actuators[1].timeUs++,
    (r) => (r.actuators[20].pwm[0] = 999),
    (r) => (r.actuators[20].pwm[5] = 500),
    (r) => delete r.actuators[20].pwm[5],
    (r) => r.actuators[20].rotorSpeedCommands.reverse(),
    (r) => (r.actuators[20].heartbeatArmed = false),
    (r) => (r.actuators[0].armed = true),
    (r) => (r.actuators[20].heartbeatAgeMs = -1),
    (r) => r.samples[1].position[0]++,
    (r) => (r.finalSnapshot.quaternion[0] = 0.5),
  ];
  for (const mutate of mutations) {
    const record = ardupilotFixture();
    mutate(record);
    assert.throws(() => replayArduPilotRecording(record));
  }
});


for (const [kind, makeRecord, replay] of [
  ['PX4', fixture, replayPx4Recording], ['ArduPilot', ardupilotFixture, replayArduPilotRecording],
]) {
  test(`${kind} rejects unsupported plant model and modified battery or propulsion evidence`, () => {
    for (const mutate of [
      r => { delete r.modelVersion; }, r => { r.modelVersion = '0.12.0'; },
      ...['samples', 'finalSnapshot'].flatMap(location => [
        r => { const s = location === 'samples' ? r.samples[0] : r.finalSnapshot; delete s.battery.openCircuitVoltage; },
        r => { const s = location === 'samples' ? r.samples[0] : r.finalSnapshot; s.battery.motorSpeedScale = 2; },
        r => { const s = location === 'samples' ? r.samples[0] : r.finalSnapshot; s.battery.openCircuitVoltage += 0.1; },
        r => { const s = location === 'samples' ? r.samples[0] : r.finalSnapshot; s.battery.motorSpeedScale *= 0.9; },
        r => { const s = location === 'samples' ? r.samples[0] : r.finalSnapshot; s.battery.voltage += 0.1; },
        r => { const s = location === 'samples' ? r.samples[0] : r.finalSnapshot; s.battery.powerLimited = !s.battery.powerLimited; },
        r => { const s = location === 'samples' ? r.samples[0] : r.finalSnapshot; s.propulsion.motor += 0.1; },
        r => { const s = location === 'samples' ? r.samples[0] : r.finalSnapshot; s.propulsion.thrustN = NaN; },
      ]),
    ]) {
      const record = makeRecord(); mutate(record); assert.throws(() => replay(record));
    }
  });
}


test('both native external plant replays preserve low-SOC voltage-limited propulsion without changing raw commands', () => {
  for (const [makeRecord, replay] of [[fixture, replayPx4Recording], [ardupilotFixture, replayArduPilotRecording]]) {
    const record = makeRecord({ batteryInitialSoc: 0.25, batteryInternalResistance: 0.5 });
    assert.ok(record.samples.some(s => s.armed && s.battery.motorSpeedScale < 1 && s.battery.voltage < s.battery.openCircuitVoltage));
    assert.ok(record.finalSnapshot.battery.usedWh > 0);
    const result = replay(JSON.parse(JSON.stringify(record)));
    assert.equal(result.maxBatteryError, 0); assert.equal(result.maxPropulsionError, 0); assert.equal(result.passed, true);
  }
});
