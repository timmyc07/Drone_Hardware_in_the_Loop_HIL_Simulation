import test from "node:test";
import assert from "node:assert/strict";
import {
  Px4Mission,
  compileMission,
  validateMissionPoints,
  verifyMissionItem,
} from "../px4-mission.js";
import { projectNed, sensorMessages, DEFAULT_ORIGIN } from "../px4-protocol.js";
import { createSimulation, validateConfig } from "../../web/sim.js";
const points = [{ north: 8, east: -8, altitude: 3, hold: 0.5 }];
function fixture() {
  const s = {
    config: validateConfig({ startOnGround: true }),
    origin: DEFAULT_ORIGIN,
    phase: "running",
    heartbeatEpoch: 2,
    lastHeartbeat: Date.now(),
    controller: {
      verified: true,
      parametersVerified: true,
      armedKnown: true,
      armed: false,
      telemetryArmed: false,
      flightMode: "AUTO_MISSION",
    },
    record: { missionEvents: [] },
    bytes: 0,
    sim: createSimulation(
      { startOnGround: true },
      { controlSource: "external" },
    ),
    gcs: { send() {}, command: async () => {} },
  };
  s.missionManager = new Px4Mission(s);
  s.missionManager.handle(242, {
    latitude: 250330000,
    longitude: 1215654000,
    altitude: 20000,
  });
  s.missionManager.handle(24, { fixType: 3 });
  return s;
}
test("PX4 mission uses seq0 takeoff, float hold, explicit capture radius and exact HIL GPS projection", () => {
  const c = validateConfig();
  const items = compileMission(validateMissionPoints(points, c));
  assert.deepEqual(
    items.map((x) => x.command),
    [22, 16, 21],
  );
  assert.equal(items[1].seq, 1);
  assert.equal(items[1].param2, 0.5);
  assert.equal(items[1].param1, 0.5);
  for (const [n, e] of [
    [500, 500],
    [-500, -500],
    [0, 0],
  ]) {
    const p = projectNed(n, e),
      truth = createSimulation(
        { startOnGround: true },
        { controlSource: "external" },
      ).readSensors();
    truth.positionNed = [n, e, -3];
    const gps = sensorMessages(truth).gps;
    assert.equal(Math.round(p.latitude * 1e7), gps.lat);
    assert.equal(Math.round(p.longitude * 1e7), gps.lon);
  }
  assert.throws(() => validateMissionPoints(Array(1), c));
  assert.throws(() => validateMissionPoints([{ ...points[0], hold: NaN }], c));
  assert.throws(() =>
    validateMissionPoints([{ ...points[0], altitude: 0 }], c),
  );
  assert.throws(() => validateMissionPoints([{ ...points[0], extra: 1 }], c));
  verifyMissionItem({ ...items[1], frame: 3 }, items[1]);
  assert.throws(() =>
    verifyMissionItem({ ...items[1], x: items[1].x + 1 }, items[1]),
  );
  assert.throws(() => verifyMissionItem({ ...items[1], param2: 10 }, items[1]));
});
test("PX4 mission readiness requires fresh home/GPS, independent HIL/HB disarm and post-command heartbeat", () => {
  const s = fixture(),
    m = s.missionManager;
  assert.equal(m.snapshot().readyToUpload, true);
  s.armingAckEpoch = 2;
  assert.equal(m.snapshot().readyToUpload, false);
  s.heartbeatEpoch = 3;
  assert.equal(m.snapshot().readyToUpload, true);
  s.controller.telemetryArmed = true;
  assert.equal(m.snapshot().readyToUpload, false);
  s.controller.telemetryArmed = false;
  s.controller.armed = true;
  assert.equal(m.snapshot().readyToUpload, false);
  s.controller.armed = false;
  s.lastHeartbeat = Date.now() - 3000;
  assert.equal(m.snapshot().readyToUpload, false);
  s.lastHeartbeat = Date.now();
  m.handle(242, {
    latitude: 250350000,
    longitude: 1215654000,
    altitude: 20000,
  });
  assert.equal(m.snapshot().homeReady, false);
});
test("PX4 native COMPLETE cannot fabricate waypoint reaches or plant landing, and manual interruption stays interrupted", () => {
  const s = fixture(),
    m = s.missionManager;
  m.items = compileMission(points);
  m.state.points = points;
  m.state.verified = true;
  m.state.phase = "running";
  m.evidence = {
    armed: { time: 1 },
    airborne: { time: 2 },
    active: { time: 1 },
  };
  m.handle(42, { seq: 2, missionState: 5, total: 3 });
  assert.equal(m.state.phase, "running");
  assert.equal(m.evidence.complete, undefined);
  m.handle(46, { seq: 1 });
  assert.equal(m.state.phase, "running");
  m.handle(245, { landedState: 1 });
  assert.equal(m.state.phase, "running");
  m.handle(42, { seq: 2, missionState: 5, total: 3 });
  assert.equal(m.state.phase, "completed");
  assert.equal(m.state.completionEvidence.complete.missionState, 5);
  m.expectedMissionId = 123;
  assert.throws(
    () => m.handle(42, { seq: 2, missionState: 5, total: 3, missionId: 124 }),
    /identity/,
  );
  m.state.phase = "running";
  m.interrupt("manual land");
  m.update();
  assert.equal(m.state.phase, "interrupted");
  s.record.missionEvents = Array(10000);
  m.state.phase = "running";
  assert.doesNotThrow(() => m.abort("close"));
  assert.equal(m.state.phase, "error");
});

test("PX4 GCS preserves HIL arming, decodes bytewise parameters and always rejects pending ACK on diagnostic failure", async () => {
  const { Px4Gcs } = await import("../px4-gcs.js");
  const s = fixture();
  s.controller.armed = true;
  const g = new Px4Gcs(s);
  g.send = () => {};
  g.handle(0, {
    autopilot: 12,
    baseMode: 0,
    customMode: (4 << 16) | (4 << 24),
  });
  assert.equal(s.controller.armed, true);
  assert.equal(s.controller.telemetryArmed, false);
  assert.equal(s.controller.flightMode, "AUTO_MISSION");
  const bits = Buffer.alloc(4);
  bits.writeInt32LE(4);
  g.handle(22, {
    paramId: "COM_RC_IN_MODE",
    paramType: 6,
    paramValue: bits.readFloatLE(),
  });
  g.handle(22, { paramId: "THR_MDL_FAC", paramType: 9, paramValue: 1 });
  assert.equal(s.controller.parametersVerified, true);
  assert.equal(s.controller.confirmedParameters.COM_RC_IN_MODE, 4);
  assert.throws(
    () =>
      g.handle(22, { paramId: "COM_RC_IN_MODE", paramType: 9, paramValue: 4 }),
    /type/,
  );
  s.missionManager.diagnostic = () => {
    throw Error("diagnostic cap");
  };
  const pending = g.command(300, [0]);
  g.handle(77, {
    command: 300,
    result: 0,
    targetSystem: 255,
    targetComponent: 190,
  });
  await assert.rejects(pending, /diagnostic cap/);
  assert.equal(g.pending.size, 0);
  await g.close();
});

test("PX4 upload rewinds the actual navigator after readback, preserves float holds and waits for a fresh heartbeat", async () => {
  const s = fixture(),
    m = s.missionManager;
  const sent = [];
  m.transfer.upload = async () => {
    m.state.phase = "verifying";
    m.expectedMissionId = 123;
    return compileMission(points);
  };
  s.gcs.send = (message) => {
    sent.push(message);
    m.handle(42, { seq: 0, total: 3, missionState: 4, missionId: 123 });
    setTimeout(() => s.heartbeatEpoch++, 5);
  };
  m.state.phase = "completed";
  m.state.currentSeq = 2;
  m.state.reachedSeqs = [0, 1, 2];
  const uploaded = await m.upload(points);
  assert.equal(uploaded.phase, "ready");
  assert.equal(uploaded.currentSeq, 0);
  assert.deepEqual(uploaded.reachedSeqs, []);
  assert.equal(uploaded.points[0].hold, 0.5);
  assert.equal(sent[0].seq, 0);
  assert.equal(sent[0].targetSystem, 1);
  assert.equal(s.heartbeatEpoch, 3);
  await assert.rejects(m.upload([{ ...points[0], hold: 61 }]), /limits/);
  assert.equal(m.state.verified, false);
  assert.equal(m.state.phase, "error");
});

test("PX4 upload mutex spans navigator confirmation and cancellation leaves an error without leaking a pending transfer", async () => {
  const s = fixture(),
    m = s.missionManager;
  m.transfer.upload = async () => {
    m.state.phase = "verifying";
    return compileMission(points);
  };
  s.gcs.send = () => {};
  const waiting = m.upload(points);
  await new Promise((resolve) => setTimeout(resolve, 0));
  assert.equal(m.uploading, true);
  assert.equal(m.snapshot().readyToUpload, false);
  await assert.rejects(m.upload(points), /exclusive/);
  m.abort("Session stopped");
  await assert.rejects(waiting);
  assert.equal(m.uploading, false);
  assert.equal(m.state.phase, "error");
  assert.equal(m.state.verified, false);
  s.controller.armed = true;
  m.state.phase = "running";
  await assert.rejects(m.upload(points), /readiness/);
  assert.equal(m.state.phase, "running");
});

test("PX4 mission stop during ACK or armed confirmation cannot become a successful start", async () => {
  for (const afterAck of [false, true]) {
    const s = fixture(),
      m = s.missionManager;
    m.state.phase = "ready";
    m.state.verified = true;
    m.items = compileMission(points);
    const cancel = () => {
      s.controller.armed = true;
      s.phase = "stopped";
      m.abort("Session stopped");
    };
    s.gcs.command = async () => {
      if (afterAck) setTimeout(cancel, 5);
      else cancel();
    };
    await assert.rejects(m.start(), /cancelled/);
    assert.equal(m.state.phase, "error");
    assert.equal(m.state.verified, false);
    assert.equal(m.starting, false);
    assert.equal(
      s.record.missionEvents.some((event) => event.type === "start"),
      false,
    );
    assert.equal(
      s.record.missionEvents.some((event) => event.type === "interrupted"),
      true,
    );
  }
});
