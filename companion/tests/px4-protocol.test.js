import test from "node:test";
import assert from "node:assert/strict";
import { PassThrough } from "node:stream";
import { common } from "node-mavlink";
import { mapActuators, sensorMessages, encoder } from "../px4-protocol.js";
import { packetReader } from "../mavlink.js";
import { Px4Session } from "../px4-session.js";
import { createSimulation } from "../../web/sim.js";

test("pinned Iris maps outputs to FR RR RL FL and rejects unsafe/missing lockstep values", () => {
  const message = Object.assign(new common.HilActuatorControls(), {
    timeUsec: 1000000n,
    flags: 1n,
    mode: 128,
    controls: [0.1, 0.2, 0.3, 0.4, ...Array(12).fill(0)],
  });
  assert.deepEqual(
    mapActuators(message).rotorSpeedCommands,
    [0.1, 0.4, 0.2, 0.3],
  );
  for (const patch of [
    { flags: 0n },
    { flags: 1 },
    { mode: 256 },
    { timeUsec: -1n },
    { controls: Array(16).fill(NaN) },
  ])
    assert.throws(() => mapActuators({ ...message, ...patch }));
  assert.deepEqual(
    mapActuators({ ...message, mode: 0 }).rotorSpeedCommands,
    [0, 0, 0, 0],
  );
});
test("HIL sensor units, WMM field, acquisition timestamps and GPS signs remain consistent", () => {
  const sim = createSimulation(
      { startOnGround: true },
      { controlSource: "external" },
    ),
    truth = sim.readSensors();
  const { sensor, gps } = sensorMessages(truth);
  assert.equal(sensor.timeUsec, 1000000n);
  assert.equal(gps.timeUsec, 1000000n);
  assert.ok(Math.abs(sensor.zacc + 9.80665) < 0.02);
  assert.equal(sensor.fieldsUpdated & 1024, 0);
  assert.ok(sensor.xmag > 0.36 && sensor.xmag < 0.37);
  assert.ok(sensor.ymag < 0);
  assert.ok(sensor.zmag > 0.27 && sensor.zmag < 0.28);
  assert.equal(gps.lat, 250330000);
  assert.equal(gps.lon, 1215654000);
  assert.equal(gps.alt, 20000);
  const moved = sensorMessages({
    ...truth,
    positionNed: [100, 100, -10],
    timeUs: 4000,
  });
  assert.ok(moved.gps.lat > gps.lat);
  assert.ok(moved.gps.lon > gps.lon);
  assert.equal(moved.gps.alt, 30000);
  assert.notEqual(moved.sensor.xmag, sensor.xmag);
  assert.deepEqual(sensorMessages(truth), { sensor, gps });
});
test("MAVLink 2 fragmented packets validate CRC and corrupted data is rejected", async () => {
  const source = new PassThrough();
  let errors = 0;
  const received = [];
  const streams = packetReader(
    source,
    (header, message) => received.push({ header, message }),
    () => errors++,
  );
  const wire = encoder()(
    sensorMessages(
      createSimulation(
        { startOnGround: true },
        { controlSource: "external" },
      ).readSensors(),
    ).sensor,
  );
  source.write(wire.subarray(0, 7));
  source.write(wire.subarray(7));
  const corrupt = Buffer.from(wire);
  corrupt[15] ^= 1;
  source.write(corrupt);
  await new Promise((resolve) => setImmediate(resolve));
  assert.equal(received.length, 1);
  assert.equal(received[0].header.msgid, 107);
  assert.equal(errors, 1);
  source.destroy();
  streams.forEach((stream) => stream.destroy());
});
test("session firmware verification and command allowlist are explicit", async () => {
  const session = new Px4Session();
  assert.equal(session.state().controller.verified, false);
  await assert.rejects(session.command("arm"), /verified/);
  await assert.rejects(session.command("shell"), /Unknown/);
  session.record = { firmware: {} };
  session.log("PX4 git-hash: d6f12ad1c4f70ad3230afd7d86e971421e02fef4");
  assert.equal(session.state().controller.verified, true);
  const wrong = new Px4Session();
  wrong.log("PX4 git-hash: 0000000000000000000000000000000000000000");
  assert.equal(wrong.phase, "fault");
});

test("paused in-flight control is consumed exactly once on resume", async () => {
  const session = new Px4Session();
  session.sim = createSimulation(
    { startOnGround: true },
    { controlSource: "external" },
  );
  session.phase = "paused";
  session.lastSentUs = 1000000;
  session.record = { actuators: [], samples: [], events: [] };
  session.controller.verified = true;
  session.child = { stdin: { write() {} } };
  session.sendSensors = () => {};
  const message = Object.assign(new common.HilActuatorControls(), {
    timeUsec: 1000000n,
    flags: 1n,
    mode: 0,
    controls: Array(16).fill(0),
  });
  session.onActuator(message);
  assert.equal(session.sim.readSensors().timeUs, 0);
  session.command("resume");
  assert.equal(session.sim.readSensors().timeUs, 4000);
  assert.equal(session.record.actuators.length, 1);
  session.onActuator(message);
  assert.equal(session.sim.readSensors().timeUs, 4000);
  clearTimeout(session.nextTick);
});

test("external protocol rejects sparse controls, bad truth and out-of-range GPS", () => {
  const message = Object.assign(new common.HilActuatorControls(), {
    timeUsec: 1000000n,
    flags: 1n,
    mode: 128,
    controls: Array(16),
  });
  assert.throws(() => mapActuators(message));
  const truth = createSimulation(
    { startOnGround: true },
    { controlSource: "external" },
  ).readSensors();
  assert.throws(() =>
    sensorMessages({ ...truth, quaternionWxyz: [0, 0, 0, 0] }),
  );
  assert.throws(() => sensorMessages({ ...truth, velocityNed: [400, 0, 0] }));
  assert.throws(() => sensorMessages({ ...truth, timeUs: -1 }));
});
