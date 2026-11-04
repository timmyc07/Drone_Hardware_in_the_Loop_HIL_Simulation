import test from "node:test";
import assert from "node:assert/strict";
import { PassThrough } from "node:stream";
import { standard } from "node-mavlink";
import {
  decodeServoPacket,
  mapServoSpeeds,
  physicsJson,
  commandMessage,
  mavlinkEncoder,
  autopilotIdentity,
} from "../ardupilot-protocol.js";
import { packetReader } from "../px4-protocol.js";
import { ArduPilotSession } from "../ardupilot-session.js";
import { createSimulation } from "../../web/sim.js";
function wire(frame = 0, rate = 1000) {
  const bytes = Buffer.alloc(40);
  bytes.writeUInt16LE(18458);
  bytes.writeUInt16LE(rate, 2);
  bytes.writeUInt32LE(frame, 4);
  for (let i = 0; i < 16; i++)
    bytes.writeUInt16LE(i < 4 ? 1100 + i * 100 : 0, 8 + i * 2);
  return bytes;
}
test("ArduPilot native servo layout, PWM speed curve and QuadX mapping are explicit", () => {
  const packet = decodeServoPacket(wire(42));
  assert.equal(packet.frameCount, 42);
  assert.equal(packet.frameRate, 1000);
  assert.deepEqual(mapServoSpeeds(packet, true), [0.1, 0.4, 0.2, 0.3]);
  assert.deepEqual(mapServoSpeeds(packet, false), [0, 0, 0, 0]);
  for (const invalid of [
    Buffer.alloc(72),
    wire().subarray(0, 39),
    wire(0, 250),
  ])
    assert.throws(() => decodeServoPacket(invalid));
  const bad = wire();
  bad.writeUInt16LE(0);
  assert.throws(() => decodeServoPacket(bad));
  assert.equal(decodeServoPacket(wire(0, 1200)).frameRate, 1200);
  assert.throws(() => mapServoSpeeds({ pwm: Array(16) }, true));
  const zero = decodeServoPacket(Buffer.from(wire()));
  zero.pwm[0] = 0;
  assert.throws(() => mapServoSpeeds(zero, true));
});
test("JSON preserves exact current NED/FRD acquisition truth, specific force and clock units", () => {
  const sim = createSimulation(
      { startOnGround: true },
      { controlSource: "external" },
    ),
    truth = sim.readSensors();
  const encoded = physicsJson(truth, sim.snapshot().battery);
  assert.equal(encoded[0], 10);
  assert.equal(encoded.at(-1), 10);
  const json = JSON.parse(encoded);
  assert.equal(json.timestamp, 0);
  assert.deepEqual(json.imu.accel_body, [0, 0, -9.80665]);
  assert.deepEqual(json.quaternion, [1, 0, 0, 0]);
  assert.equal(json.no_time_sync, false);
  assert.equal(json.no_lockstep, false);
  assert.equal(
    JSON.parse(physicsJson({ ...truth, timeUs: 2500 })).timestamp,
    0.0025,
  );
  for (const change of [
    { quaternionWxyz: [0, 0, 0, 0] },
    { positionNed: Array(3) },
    { ratesFrd: [NaN, 0, 0] },
    { timeUs: -1 },
  ])
    assert.throws(() => physicsJson({ ...truth, ...change }));
  assert.throws(() => physicsJson(truth, { voltage: NaN, current: 0 }));
});
test("standard AUTOPILOT_VERSION is decoded and only claims matching version and short git-prefix", async () => {
  const source = new PassThrough();
  const received = [];
  const streams = packetReader(
    source,
    (_, message) => received.push(message),
    () => {},
  );
  const message = Object.assign(new standard.AutopilotVersion(), {
    capabilities: 0n,
    uid: 0n,
    flightSwVersion: 0x040702ff,
    flightCustomVersion: [...Buffer.from("fb773567")],
  });
  source.write(mavlinkEncoder()(message));
  await new Promise((resolve) => setImmediate(resolve));
  assert.equal(received.length, 1);
  assert.deepEqual(autopilotIdentity(received[0]), {
    version: "4.7.2",
    reportedGitHash: "fb773567",
    verified: true,
  });
  assert.equal(
    autopilotIdentity({
      ...message,
      flightCustomVersion: [...Buffer.from("00000000")],
    }).verified,
    false,
  );
  assert.throws(() =>
    autopilotIdentity({ ...message, flightCustomVersion: Array(8) }),
  );
  assert.throws(() => commandMessage(400, Array(1)));
  source.destroy();
  streams.forEach((stream) => stream.destroy());
});
function session() {
  const s = new ArduPilotSession();
  s.phase = "running";
  s.sim = createSimulation(
    { startOnGround: true },
    { controlSource: "external" },
  );
  s.record = {
    actuators: [],
    samples: [s.sim.snapshot()],
    events: [],
    bootstrapTicks: 0,
  };
  s.sendReply = () => {};
  return s;
}
test("frame duplicates reuse cached sensor state, frame gaps/reset freeze rather than inventing physics ticks", () => {
  const s = session(),
    remote = { address: "127.0.0.1", port: 12345 };
  s.receiveServo(wire(0, 1200), remote);
  clearTimeout(s.nextTick);
  assert.equal(s.sim.readSensors().timeUs, 1000);
  assert.equal(s.record.actuators.length, 1);
  s.receiveServo(wire(0, 1200), remote);
  assert.equal(s.sim.readSensors().timeUs, 1000);
  assert.equal(s.stats.duplicateFrames, 1);
  const changed = wire(0, 1200);
  changed.writeUInt16LE(1900, 8);
  assert.throws(() => s.receiveServo(changed, remote), /retransmitted/);
  assert.equal(s.sim.readSensors().timeUs, 1000);
  assert.throws(() => s.receiveServo(wire(2), remote), /gap|reset/);
  assert.equal(s.sim.readSensors().timeUs, 1000);
  s.receiveServo(wire(1), remote);
  clearTimeout(s.nextTick);
  assert.equal(s.sim.readSensors().timeUs, 2000);
  assert.throws(() => s.receiveServo(wire(0), remote));
  assert.throws(() => s.receiveServo(wire(2), { ...remote, port: 12346 }));
});
test("pause holds fresh frame until resume, export includes final accepted state and no actuator CRC is claimed", async () => {
  const s = session(),
    remote = { address: "127.0.0.1", port: 12345 };
  s.phase = "paused";
  s.receiveServo(wire(0, 1200), remote);
  assert.equal(s.sim.readSensors().timeUs, 0);
  await s.command("resume");
  clearTimeout(s.nextTick);
  assert.equal(s.sim.readSensors().timeUs, 1000);
  assert.equal(s.record.actuators.length, 1);
  const record = s.exportRecording();
  assert.equal(record.samples.at(-1).time, record.finalSnapshot.time);
  assert.equal(s.state().stats.crcErrors, null);
  assert.equal(s.controller.sourceSha, null);
  assert.equal(s.state().clock.stepUs, 1000);
  const limited = session();
  limited.bytes = 96 * 1024 * 1024;
  limited.receiveServo(wire(0, 1200), remote);
  assert.equal(limited.phase, "paused");
  assert.equal(limited.record.limitReached, true);
  assert.equal(limited.record.actuators.length, 1);
  assert.equal(limited.exportRecording().finalSnapshot.time, 0.001);
  await assert.rejects(() => limited.command("resume"));
});
