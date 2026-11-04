import test from "node:test";
import assert from "node:assert/strict";
import { common, minimal, MavLinkProtocolV2 } from "node-mavlink";
import { Px4Session } from "../px4-session.js";
import { Px4Gcs } from "../px4-gcs.js";
import { Px4Mission } from "../px4-mission.js";
import { ArduPilotSession } from "../ardupilot-session.js";
import { decodeDatagram } from "../mavlink.js";
import { createSimulation } from "../../web/sim.js";

const PREARM = 0x10000000;
const RC_RECEIVER = 0x10000;
const HIGH_BIT = 0x80000000;

// CRC-validated library encoding/decoding followed by direct handlers, not a network transport test.
function nativeMessage(message) {
  const frame = decodeDatagram(
    new MavLinkProtocolV2(1, 1).serialize(message, 17),
  )[0];
  assert.equal(frame.header.sysid, 1);
  assert.equal(frame.header.compid, 1);
  return frame;
}

for (const kind of ["px4", "ardupilot"]) {
  test(`${kind} real Session passively exposes native health during external GCS ownership without control or plant changes`, () => {
    const session = kind === "px4" ? new Px4Session() : new ArduPilotSession();
    assert.equal(session.state().controller.health.prearm, "unknown");
    assert.equal(session.state().controller.health.sysStatus, null);
    session.phase = "running";
    session.controller.verified = true;
    session.sim = createSimulation(
      { startOnGround: true },
      { controlSource: "external" },
    );
    session.groundStation.settings = {
      enabled: true,
      targetPort: 14550,
      owner: "gcs",
    };
    assert.equal(session.groundStation.externalOwner, true);
    if (kind === "px4") {
      session.gcs = new Px4Gcs(session);
      session.missionManager = new Px4Mission(session);
    }
    const missionBefore = session.missionManager.snapshot();
    const physicalBefore = session.sim.snapshot();
    let controlCalls = 0;
    const forbid = () => {
      controlCalls++;
      throw new Error(
        "Passive telemetry cannot issue controls or advance mission state",
      );
    };
    session.command = forbid;
    session.sim.stepExternal = forbid;
    session.missionManager.handle = forbid;
    session.missionManager.update = forbid;
    session.missionManager.interrupt = forbid;
    if (kind === "px4") {
      session.gcs.send = forbid;
      session.gcs.command = forbid;
      session.phase = "paused";
      session.groundStation.settings.owner = "web";
      session.gcs.poll();
      session.phase = "running";
      session.groundStation.settings.owner = "gcs";
    } else {
      const requested = [];
      session.telemetry = { destroyed: false };
      session.writeMavlink = (message) =>
        requested.push(nativeMessage(message).message);
      session.pollTelemetry();
      assert.equal(requested.length, 1);
      assert.equal(requested[0].constructor.MSG_ID, 76);
      assert.equal(requested[0].command, 512);
      assert.equal(requested[0]._param1, 1);
      session.phase = "paused";
      session.groundStation.settings.owner = "web";
      session.pollTelemetry();
      assert.equal(
        requested.length,
        1,
        "Paused telemetry must not queue new requests",
      );
      session.phase = "running";
      session.groundStation.settings.owner = "gcs";
      session.telemetry = undefined;
      session.writeMavlink = forbid;
    }
    const deliver = (message) => {
      const { header, message: decoded } = nativeMessage(message);
      if (kind === "px4") session.gcs.handle(header.msgid, decoded);
      else session.onTelemetry(header.msgid, decoded);
    };
    const heartbeat = (armed) =>
      Object.assign(new minimal.Heartbeat(), {
        type: 2,
        autopilot: kind === "px4" ? 12 : 3,
        baseMode: armed ? 128 : 0,
        customMode: kind === "px4" ? (4 << 16) | (3 << 24) : 4,
        systemStatus: 4,
        mavlinkVersion: 3,
      });
    deliver(heartbeat(false));
    const enabled =
      (HIGH_BIT | RC_RECEIVER | (kind === "ardupilot" ? PREARM : 0)) >>> 0;
    deliver(
      Object.assign(new common.SysStatus(), {
        onboardControlSensorsPresent: HIGH_BIT,
        onboardControlSensorsEnabled: enabled,
        onboardControlSensorsHealth: PREARM,
      }),
    );
    deliver(
      Object.assign(new common.StatusText(), {
        severity: 4,
        text: "Passive native diagnostic",
        id: 0,
        chunkSeq: 0,
      }),
    );
    let state = session.state();
    assert.equal(state.controller.health.prearm, "reported-ok");
    assert.equal(state.controller.health.sysStatus.present, HIGH_BIT);
    assert.equal(state.controller.health.sysStatus.enabled, enabled);
    assert.equal(
      state.controller.health.sysStatus.unhealthyMask,
      (HIGH_BIT | RC_RECEIVER) >>> 0,
    );
    assert.ok(state.controller.health.sysStatus.unhealthyMask > 0x7fffffff);
    assert.ok(
      state.controller.health.sysStatus.ageMs >= 0 &&
        state.controller.health.sysStatus.ageMs < 1000,
    );
    assert.ok(
      state.controller.health.heartbeatAgeMs >= 0 &&
        state.controller.health.heartbeatAgeMs < 1000,
    );
    assert.equal(
      state.controller.health.messages.at(-1).text,
      "Passive native diagnostic",
    );
    assert.equal(state.controller.health.messages.at(-1).severity, 4);
    deliver(
      Object.assign(new common.StatusText(), {
        severity: 6,
        text: Buffer.from("需要定位", "utf8").toString("latin1"),
        id: 0,
        chunkSeq: 0,
      }),
    );
    assert.equal(
      session.state().controller.health.messages.at(-1).text,
      "需要定位",
    );
    if (kind === "ardupilot")
      assert.equal(session.logs.at(-1), "MAVLink: 需要定位");
    deliver(heartbeat(true));
    state = session.state();
    assert.equal(state.controller.health.prearm, "armed");
    if (kind === "px4") {
      assert.equal(
        state.controller.armed,
        false,
        "GCS heartbeat cannot overwrite HIL actuator armed authority",
      );
      assert.equal(state.controller.telemetryArmed, true);
    } else assert.equal(state.controller.armed, true);
    assert.equal(controlCalls, 0);
    assert.deepEqual(session.sim.snapshot(), physicalBefore);
    assert.deepEqual(session.missionManager.snapshot(), missionBefore);
    assert.equal(state.snapshot.time, 0);
    assert.equal(state.stats.sensorPackets, 0);
    assert.equal(state.stats.actuatorPackets, 0);
    assert.equal(session.gcs?.socket, undefined);
    assert.equal(session.telemetry, undefined);
  });
}
