import test from "node:test";
import assert from "node:assert/strict";
import { createSimulation, validateConfig } from "../../web/sim.js";
import { OUTPUT_MAP } from "../px4-protocol.js";
import { replayPx4Recording } from "../replay.js";

function fixture() {
  const config = validateConfig({ startOnGround: true }),
    sim = createSimulation(config, { controlSource: "external" });
  const record = {
    format: "flight-lab-px4",
    version: 1,
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
    events: [],
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
    (r) => (r.finalSnapshot.motors[0] = 0.5),
    (r) => (r.finalSnapshot.armed = !r.finalSnapshot.armed),
  ];
  for (const change of mutations) {
    const record = fixture();
    change(record);
    assert.throws(() => replayPx4Recording(record));
  }
});
