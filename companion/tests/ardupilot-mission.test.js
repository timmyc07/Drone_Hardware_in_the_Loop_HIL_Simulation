import { ArduPilotSession } from "../ardupilot-session.js";
import test from "node:test";
import assert from "node:assert/strict";
import { common } from "node-mavlink";
import { createSimulation, validateConfig } from "../../web/sim.js";
import {
  ArduPilotMission,
  validateMissionPoints,
  projectWaypoint,
  compileMission,
  verifyMissionItem,
  MISSION_ORIGIN,
} from "../ardupilot-mission.js";
const points = [
  { north: 8, east: 0, altitude: 3, hold: 1 },
  { north: 8, east: 8, altitude: 3, hold: 1 },
];
const home = { latitude: 250330000, longitude: 1215654000, altitude: 20000 };
function fixture(mismatch = false) {
  const s = {
    phase: "running",
    config: validateConfig(),
    controller: {
      verified: true,
      parametersVerified: true,
      armedKnown: true,
      armed: false,
      flightMode: "STABILIZE",
    },
    sim: createSimulation(
      { startOnGround: true },
      { controlSource: "external" },
    ),
    record: { missionEvents: [] },
    bytes: 0,
  };
  const mission = new ArduPilotMission(s);
  mission.home = { ...home };
  mission.gpsFix = 3;
  const uploaded = [];
  s.writeMavlink = (message) =>
    queueMicrotask(() => {
      const id = message.constructor.MSG_ID,
        base = { targetSystem: 255, targetComponent: 191, missionType: 0 };
      if (id === 44) mission.handle(40, { ...base, seq: 0 });
      else if (id === 73) {
        uploaded[message.seq] = { ...message };
        if (message.seq + 1 < messageCount())
          mission.handle(51, { ...base, seq: message.seq + 1 });
        else mission.handle(47, { ...base, type: 0 });
      } else if (id === 43)
        mission.handle(44, { ...base, count: uploaded.length });
      else if (id === 51) {
        const item = {
          ...uploaded[message.seq],
          ...base,
          frame: uploaded[message.seq].frame === 6 ? 3 : 0,
        };
        if (mismatch && message.seq === 2) item.x++;
        mission.handle(73, item);
      }
    });
  const messageCount = () => mission.items.length;
  return { s, mission, uploaded };
}
test("AP projection follows tagged spherical midpoint longitude and retains negative offsets within centimeter quantization", () => {
  assert.deepEqual(projectWaypoint(0, 0), { x: 250330000, y: 1215654000 });
  for (const [north, east] of [
    [500, 500],
    [-500, -500],
    [-8, 8],
    [8, -8],
  ]) {
    const p = projectWaypoint(north, east),
      dn = (p.x - home.latitude) * 0.011131884502145034,
      de =
        (p.y - home.longitude) *
        0.011131884502145034 *
        Math.cos((((p.x + home.latitude) / 2) * 1e-7 * Math.PI) / 180);
    assert.ok(Math.abs(dn - north) < 0.012);
    assert.ok(Math.abs(de - east) < 0.012);
  }
  assert.throws(() => projectWaypoint(501, 0));
  const config = validateConfig();
  for (const input of [
    [],
    Array(1),
    [{ ...points[0], hold: 0.5 }],
    [{ ...points[0], altitude: 0 }],
    [{ ...points[0], extra: 1 }],
    [{ ...points[0], hold: 61 }],
  ])
    assert.throws(() => validateMissionPoints(input, config));
  assert.equal(common.MavCmd.MISSION_START, 300);
  assert.equal(common.MavCmd.DO_PAUSE_CONTINUE, 193);
});
test("mission upload answers40/51 with INT payload and requires verified download", async () => {
  const { s, mission } = fixture();
  const result = await mission.upload(points);
  assert.equal(result.phase, "ready");
  assert.equal(result.verified, true);
  assert.equal(mission.items.length, 5);
  assert.equal(mission.items[0].frame, 5);
  assert.equal(mission.items[1].command, common.MavCmd.NAV_TAKEOFF);
  assert.equal(mission.items.at(-1).command, common.MavCmd.NAV_LAND);
  assert.equal(result.currentSeq, null);
  assert.ok(
    s.record.missionEvents.some(
      (e) => e.type === "uploadRequest" && e.messageId === 40 && e.seq === 0,
    ),
  );
  assert.equal(mission.items[2].x, 250330718);
  assert.ok(Number.isInteger(mission.items[2].y));
  assert.ok(s.record.missionEvents.some((e) => e.type === "verified"));
  const bad = fixture(true);
  await assert.rejects(() => bad.mission.upload(points), /differs/);
  assert.equal(bad.mission.state.verified, false);
  assert.equal(bad.mission.state.phase, "error");
  const duplicate = fixture();
  await duplicate.mission.upload(points);
  duplicate.mission.state.phase = "verifying";
  duplicate.mission.transfer.downloaded =
    duplicate.mission.transfer.downloaded.slice(0, 1);
  duplicate.mission.transfer.phase = "verifying";
  duplicate.mission.transfer.active = true;
  let requested;
  duplicate.s.writeMavlink = (message) => {
    requested = message.seq;
  };
  let rejected = false;
  duplicate.mission.transfer.reject = () => {
    rejected = true;
  };
  duplicate.mission.handle(73, { ...duplicate.mission.transfer.downloaded[0] });
  assert.equal(rejected, false);
  assert.equal(requested, 1);
  duplicate.mission.handle(73, {
    ...duplicate.mission.transfer.downloaded[0],
    x: 0,
  });
  assert.equal(rejected, true);
  assert.equal(duplicate.mission.state.verified, false);
  const float = fixture();
  float.s.writeMavlink = () => {};
  const floatPending = float.mission.upload(points);
  float.mission.handle(39, {
    targetSystem: 255,
    targetComponent: 191,
    missionType: 0,
  });
  await assert.rejects(floatPending, /Float MISSION_ITEM/);
  const early = fixture();
  early.s.writeMavlink = () => {};
  const promise = early.mission.upload(points);
  early.mission.handle(47, {
    targetSystem: 255,
    targetComponent: 191,
    missionType: 0,
    type: 0,
  });
  await assert.rejects(promise, /incomplete/);
});
test("mission transfer rejects target/type/sequence mismatch, timeout and cancellation without retaining readiness", async (t) => {
  for (const message of [
    { targetSystem: 2, targetComponent: 191, missionType: 0, seq: 0 },
    { targetSystem: 255, targetComponent: 191, missionType: 1, seq: 0 },
    { targetSystem: 255, targetComponent: 191, missionType: 0, seq: 2 },
  ]) {
    const f = fixture();
    f.s.writeMavlink = () => {};
    const pending = f.mission.upload(points);
    f.mission.handle(51, message);
    await assert.rejects(pending);
    assert.equal(f.mission.state.verified, false);
  }
  t.mock.timers.enable({ apis: ["setTimeout", "setInterval"] });
  const f = fixture();
  f.s.writeMavlink = () => {};
  const pending = f.mission.upload(points);
  t.mock.timers.tick(15001);
  await assert.rejects(pending, /deadline/);
  assert.equal(f.mission.state.phase, "error");
  const race = fixture();
  race.s.heartbeatEpoch = 2;
  race.s.armingAckEpoch = 2;
  assert.equal(race.mission.readiness(), false);
  race.s.heartbeatEpoch = 3;
  assert.equal(race.mission.readiness(), true);
  race.s.controller.armed = true;
  assert.equal(race.mission.readiness(), false);
  const cap = fixture();
  cap.s.writeMavlink = () => {};
  const capped = cap.mission.upload(points);
  cap.s.record.missionEvents = Array(10000).fill({});
  cap.mission.abort("Session stopped");
  await assert.rejects(capped, /stopped/);
  const broken = fixture();
  broken.s.writeMavlink = () => {};
  const retry = broken.mission.upload(points);
  broken.s.writeMavlink = () => {
    throw new Error("socket closed");
  };
  t.mock.timers.tick(1000);
  await assert.rejects(retry, /socket closed/);
  const closing = new ArduPilotSession();
  closing.phase = "running";
  closing.sim = createSimulation(
    { startOnGround: true },
    { controlSource: "external" },
  );
  closing.record = { missionEvents: Array(10000).fill({}) };
  closing.missionManager.state.phase = "running";
  let killed = false,
    serverClosed = false,
    udpClosed = false,
    ticks = 0;
  closing.child = {
    exitCode: null,
    signalCode: null,
    kill() {
      killed = true;
      this.exitCode = 0;
    },
  };
  closing.exitPromise = Promise.resolve();
  closing.server = {
    listening: true,
    close(callback) {
      serverClosed = true;
      callback();
    },
  };
  closing.udp = {
    close(callback) {
      udpClosed = true;
      callback();
    },
  };
  closing.publisher = setInterval(() => ticks++, 10);
  await closing.close();
  t.mock.timers.tick(100);
  assert.equal(closing.phase, "stopped");
  assert.equal(closing.missionManager.state.phase, "error");
  assert.ok(killed && serverClosed && udpClosed);
  assert.equal(ticks, 0);
});
test("only actual reached messages plus land and disarm complete a running mission; manual mode interruption latches", async () => {
  const { s, mission } = fixture();
  await mission.upload(points);
  mission.state.phase = "running";
  mission.handle(46, { seq: 2 });
  mission.update();
  assert.equal(mission.state.phase, "running");
  mission.handle(46, { seq: 3 });
  mission.update();
  assert.equal(mission.state.phase, "running");
  mission.handle(46, { seq: 4 });
  mission.update();
  assert.equal(mission.state.phase, "running");
  let truth = {
    time: 5,
    position: [0, 0, -2],
    velocity: [0, 0, 0],
    status: "flying",
  };
  s.sim = { snapshot: () => truth };
  s.controller.armed = true;
  s.controller.flightMode = "AUTO";
  mission.update();
  mission.handle(42, { seq: 4, missionState: 3 });
  mission.handle(245, { landedState: 2 });
  truth = { ...truth, time: 10, position: [0, 0, 0], status: "landed" };
  s.controller.armed = false;
  mission.update();
  assert.equal(mission.state.phase, "running");
  mission.handle(245, { landedState: 1 });
  assert.equal(mission.state.phase, "completed");
  mission.handle(42, { seq: 1, missionState: 2 });
  assert.equal(mission.state.phase, "completed");
  assert.equal(mission.state.completionEvidence.landSelected.seq, 4);
  mission.state.phase = "running";
  mission.interrupt("User requested land");
  mission.update();
  assert.equal(mission.state.phase, "interrupted");
  const batch = fixture();
  await batch.mission.upload(points);
  const commands = [];
  batch.s.sendCommand = async (command) => {
    commands.push(command);
    if (command === common.MavCmd.MISSION_START) {
      batch.mission.handle(46, { seq: 1 });
      batch.s.controller.flightMode = "AUTO";
    }
  };
  await batch.mission.start();
  assert.deepEqual(batch.mission.state.reachedSeqs, [1]);
  assert.equal(batch.mission.state.currentSeq, null);
  assert.deepEqual(commands, [
    common.MavCmd.DO_SET_MODE,
    common.MavCmd.COMPONENT_ARM_DISARM,
    common.MavCmd.DO_SET_MISSION_CURRENT,
    common.MavCmd.MISSION_START,
  ]);
  const override = new ArduPilotSession();
  override.phase = "running";
  override.config = validateConfig();
  override.sim = createSimulation(
    { startOnGround: true },
    { controlSource: "external" },
  );
  Object.assign(override.controller, {
    verified: true,
    parametersVerified: true,
    armedKnown: true,
    armed: true,
  });
  override.record = { events: [], commands: [], missionEvents: [] };
  override.sendCommand = async () => {};
  override.missionManager.state.phase = "running";
  await override.command("arm");
  assert.equal(override.missionManager.state.phase, "interrupted");
  const compiled = compileMission(points);
  assert.throws(() =>
    verifyMissionItem({ ...compiled[0], x: 0 }, compiled[0], home),
  );
  assert.deepEqual(MISSION_ORIGIN, {
    latitude: 25.033,
    longitude: 121.5654,
    altitudeMsl: 20,
  });
});
