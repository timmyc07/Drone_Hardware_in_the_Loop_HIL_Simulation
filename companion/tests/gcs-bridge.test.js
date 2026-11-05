import test from "node:test";
import assert from "node:assert/strict";
import dgram from "node:dgram";
import { once } from "node:events";
import {
  common,
  minimal,
  MavLinkProtocolV2,
  MavLinkProtocolV1,
} from "node-mavlink";
import { GroundStationBridge } from "../gcs-bridge.js";
import { decodeDatagram, mavlinkEncoder } from "../mavlink.js";
import { createSimulation, validateConfig } from "../../web/sim.js";
const heartbeat = () =>
  Object.assign(new minimal.Heartbeat(), {
    type: 6,
    autopilot: 8,
    baseMode: 0,
    customMode: 0,
    systemStatus: 4,
    mavlinkVersion: 3,
  });
const command = (id, params = []) => {
  const m = Object.assign(new common.CommandLong(), {
    targetSystem: 1,
    targetComponent: 1,
    command: id,
  });
  for (let i = 1; i <= 7; i++) m[`_param${i}`] = params[i - 1] ?? 0;
  return m;
};
function fixture(kind = "px4") {
  const sent = [],
    s = {
      phase: "running",
      controller: {
        kind,
        verified: true,
        parametersVerified: true,
        armedKnown: true,
        armed: false,
      },
      heartbeatEpoch: 3,
      armingAckEpoch: 1,
      lastHeartbeat: Date.now(),
      config: validateConfig({ startOnGround: true }),
      sim: createSimulation(
        { startOnGround: true },
        { controlSource: "external" },
      ),
      record: { events: [], actuators: [{ retained: true }] },
      bytes: 0,
      pendingAcks: new Map(),
      gcs: { pending: new Map() },
      missionManager: {
        state: { verified: true },
        transfer: { active: false },
        interrupt() {},
      },
      publish() {},
      state() {
        return {
          phase: this.phase,
          groundStation: this.groundStation.snapshot(),
        };
      },
      fail(error) {
        this.phase = "fault";
        this.error = error.message;
      },
    };
  const bridge = new GroundStationBridge(s, {
    sendRaw: (raw) => sent.push(Buffer.from(raw)),
    nativeReady: () => true,
  });
  s.groundStation = bridge;
  let seq = 0;
  const wire = (m) => new MavLinkProtocolV2(255, 190).serialize(m, seq++ % 256);
  return { s, bridge, sent, wire };
}
async function monitored(t, kind) {
  const f = fixture(kind),
    peer = dgram.createSocket("udp4");
  await new Promise((resolve) => peer.bind(0, "127.0.0.1", resolve));
  const port = peer.address().port;
  await f.bridge.configure({ enabled: true, targetPort: port, owner: "web" });
  t.after(async () => {
    await f.bridge.close();
    await new Promise((resolve) => peer.close(resolve));
  });
  f.receive = (m) =>
    f.bridge.receive(f.wire(m), { address: "127.0.0.1", port });
  f.peer = peer;
  f.port = port;
  return f;
}
test("raw datagram parser validates CRC without joining partial datagrams and preserves sender sequences", () => {
  const raw = new MavLinkProtocolV2(255, 190).serialize(heartbeat(), 42);
  const frames = decodeDatagram(Buffer.concat([raw, raw]));
  assert.equal(frames.length, 2);
  assert.deepEqual(frames[0].buffer, raw);
  assert.equal(frames[0].header.seq, 42);
  assert.equal(
    decodeDatagram(mavlinkEncoder()(heartbeat()))[0].header.compid,
    191,
  );
  assert.throws(() => decodeDatagram(raw.subarray(0, 5)), /Incomplete/);
  assert.throws(() => decodeDatagram(raw.subarray(5)), /framing/);
  const bad = Buffer.from(raw);
  bad[bad.length - 1] ^= 1;
  assert.throws(() => decodeDatagram(bad), /CRC/);
  const signed = Buffer.from(raw);
  signed[2] = 1;
  assert.throws(() => decodeDatagram(signed), /Signed/);
  assert.throws(() => decodeDatagram(Buffer.alloc(8193)), /size/);
});
test("real loopback UDP monitor preserves complete telemetry and rejects control until fresh explicit takeover", async (t) => {
  const f = await monitored(t);
  const { bridge, peer, port, s, sent } = f;
  const raw = f.wire(heartbeat());
  await Promise.all([
    once(bridge.socket, "message", { signal: AbortSignal.timeout(2000) }),
    new Promise((resolve, reject) =>
      peer.send(raw, bridge.snapshot().localPort, "127.0.0.1", (error) =>
        error ? reject(error) : resolve(),
      ),
    ),
  ]);
  assert.equal(bridge.snapshot().connected, true);
  assert.deepEqual(sent[0], raw);
  f.receive(command(400, [1, 0]));
  assert.match(bridge.snapshot().lastRejection.reason, /Web owns/);
  assert.equal(sent.length, 1);
  await bridge.configure({ enabled: true, targetPort: port, owner: "gcs" });
  assert.equal(s.missionManager.state.verified, false);
  f.receive(command(400, [1, 0]));
  assert.equal(sent.length, 2);
  assert.ok(
    s.record.groundStationEvents.some(
      (e) =>
        e.type === "outboundControl" &&
        e.forwarded &&
        e.rawBase64 === sent[1].toString("base64"),
    ),
  );
  const nativeRaw = new MavLinkProtocolV2(1, 1).serialize(heartbeat(), 12),
    packet = decodeDatagram(nativeRaw)[0];
  const received = new Promise((resolve) => peer.once("message", resolve));
  assert.equal(
    bridge.nativePacket(packet.header, packet.message, packet),
    true,
  );
  assert.deepEqual(await received, nativeRaw);
  assert.equal(bridge.snapshot().localPort > 0, true);
});
test("routing isolates internal and external ACKs, while flight ownership suppresses internal mission packets", async (t) => {
  const { bridge, receive, port } = await monitored(t);
  receive(heartbeat());
  await bridge.configure({ enabled: true, targetPort: port, owner: "gcs" });
  const packet = (target) => {
    const m = Object.assign(new common.CommandAck(), {
      command: 400,
      result: 0,
      targetSystem: 255,
      targetComponent: target,
    });
    return decodeDatagram(new MavLinkProtocolV2(1, 1).serialize(m, 0))[0];
  };
  for (const target of [190, 191]) {
    const p = packet(target);
    assert.equal(bridge.nativePacket(p.header, p.message, p), false);
  }
  const current = Object.assign(new common.MissionCurrent(), {
    seq: 0,
    total: 3,
    missionState: 2,
  });
  const p = decodeDatagram(
    new MavLinkProtocolV2(1, 1).serialize(current, 0),
  )[0];
  assert.equal(bridge.nativePacket(p.header, p.message, p), false);
});
test("GCS filters protect plant parameters, normal arm gates and official PX4 NaN takeoff sentinels", async (t) => {
  const { bridge, receive, port, sent } = await monitored(t);
  receive(heartbeat());
  await bridge.configure({ enabled: true, targetPort: port, owner: "gcs" });
  receive(command(400, [1, 21196]));
  assert.match(bridge.snapshot().lastRejection.reason, /normal/);
  const before = sent.length;
  receive(
    Object.assign(new common.ParamSet(), {
      targetSystem: 1,
      targetComponent: 1,
      paramId: "THR_MDL_FAC",
      paramValue: 0,
      paramType: 9,
    }),
  );
  assert.equal(sent.length, before);
  receive(command(22, [NaN, NaN, 0, NaN, NaN, NaN, 22.5]));
  assert.equal(sent.length, before + 1);
  const decoded = decodeDatagram(sent.at(-1))[0].message;
  assert.ok(Number.isNaN(decoded._param1));
  assert.ok(Number.isNaN(decoded._param5));
  receive(command(512, [148]));
  assert.equal(sent.length, before + 2);
  receive(command(511, [30, 1]));
  assert.match(bridge.snapshot().lastRejection.reason, /interval/);
  const v1 = new MavLinkProtocolV1(255, 190).serialize(command(400, [1, 0]), 0);
  bridge.receive(v1, { address: "127.0.0.1", port });
  assert.match(bridge.snapshot().lastRejection.reason, /MAVLink2/);
});
test("external mission transactions are bounded, preserve safe item types and block handoff until complete", async (t) => {
  const { bridge, receive, port, s } = await monitored(t, "ardupilot");
  receive(heartbeat());
  await bridge.configure({ enabled: true, targetPort: port, owner: "gcs" });
  const count = Object.assign(new common.MissionCount(), {
    targetSystem: 1,
    targetComponent: 1,
    count: 3,
    missionType: 0,
  });
  receive(count);
  assert.equal(bridge.snapshot().mission.active, true);
  await assert.rejects(
    bridge.configure({ enabled: true, targetPort: port, owner: "web" }),
    /idle/,
  );
  const request = Object.assign(new common.MissionRequest(), {
      targetSystem: 255,
      targetComponent: 190,
      seq: 0,
      missionType: 0,
    }),
    req = decodeDatagram(new MavLinkProtocolV2(1, 1).serialize(request, 0))[0];
  bridge.nativePacket(req.header, req.message, req);
  const item = Object.assign(new common.MissionItemInt(), {
    targetSystem: 1,
    targetComponent: 1,
    seq: 0,
    frame: 0,
    command: 16,
    current: 0,
    autocontinue: 1,
    param1: 0,
    param2: 0,
    param3: 0,
    param4: NaN,
    x: 250330000,
    y: 1215654000,
    z: 20,
    missionType: 0,
  });
  receive(item);
  assert.equal(bridge.mission.sent.size, 1);
  receive(Object.assign(new common.MissionItemInt(), item, { command: 183 }));
  assert.match(bridge.snapshot().lastRejection.reason, /command/);
  const ack = Object.assign(new common.MissionAck(), {
      targetSystem: 255,
      targetComponent: 190,
      type: 0,
      missionType: 0,
    }),
    a = decodeDatagram(new MavLinkProtocolV2(1, 1).serialize(ack, 0))[0];
  bridge.nativePacket(a.header, a.message, a);
  await new Promise((resolve) => setTimeout(resolve, 10));
  assert.equal(bridge.mission.active, false);
  await bridge.configure({ enabled: true, targetPort: port, owner: "web" });
  assert.equal(s.missionManager.state.verified, false);
});
test("record cap freezes before sending, watchdog freezes on lost GCS and close releases every endpoint", async (t) => {
  const f = await monitored(t);
  f.receive(heartbeat());
  await f.bridge.configure({ enabled: true, targetPort: f.port, owner: "gcs" });
  const before = f.sent.length;
  f.s.bytes = 96 * 1024 * 1024 - 10001;
  f.receive(command(400, [1, 0]));
  assert.equal(f.s.phase, "fault");
  assert.equal(f.sent.length, before);
  assert.deepEqual(f.s.record.actuators, [{ retained: true }]);
  const g = await monitored(t);
  g.receive(heartbeat());
  await g.bridge.configure({ enabled: true, targetPort: g.port, owner: "gcs" });
  g.bridge.lastHeartbeat = Date.now() - 6000;
  g.bridge.checkWatchdog();
  assert.equal(g.s.phase, "fault");
  assert.match(g.s.error, /heartbeat/);
  await g.bridge.close();
  assert.equal(g.bridge.snapshot().localPort, null);
  assert.equal(g.bridge.snapshot().enabled, false);
});

test("mission download lease ends only after every unique wire item, without inventing an ACK or download verification", async (t) => {
  const { bridge, receive, port } = await monitored(t);
  receive(heartbeat());
  await bridge.configure({ enabled: true, targetPort: port, owner: "gcs" });
  receive(
    Object.assign(new common.MissionRequestList(), {
      targetSystem: 1,
      targetComponent: 1,
      missionType: 0,
    }),
  );
  const fromNative = (m) => {
    const p = decodeDatagram(new MavLinkProtocolV2(1, 1).serialize(m, 0))[0];
    bridge.nativePacket(p.header, p.message, p);
  };
  fromNative(
    Object.assign(new common.MissionCount(), {
      targetSystem: 255,
      targetComponent: 190,
      count: 2,
      missionType: 0,
    }),
  );
  const item = (seq) =>
    Object.assign(new common.MissionItemInt(), {
      targetSystem: 255,
      targetComponent: 190,
      seq,
      frame: 6,
      command: 16,
      missionType: 0,
    });
  fromNative(item(0));
  fromNative(item(0));
  assert.equal(bridge.mission.active, true);
  await assert.rejects(
    bridge.configure({ enabled: true, targetPort: port, owner: "web" }),
    /idle/,
  );
  fromNative(item(1));
  await new Promise((resolve) => setTimeout(resolve, 10));
  assert.equal(bridge.mission.active, false);
  assert.equal(bridge.mission.received.size, 2);
  await bridge.configure({ enabled: true, targetPort: port, owner: "web" });
});

test("cancelled endpoint setup cannot reopen a socket, and external arming ACK requires a subsequent native heartbeat before handback", async (t) => {
  const f = fixture(),
    opening = f.bridge.configure({
      enabled: true,
      targetPort: 14550,
      owner: "web",
    });
  await f.bridge.close();
  await assert.rejects(opening, /stopped/);
  assert.equal(f.bridge.snapshot().localPort, null);
  const { bridge, receive, port, s } = await monitored(t);
  receive(heartbeat());
  await bridge.configure({ enabled: true, targetPort: port, owner: "gcs" });
  receive(command(400, [1, 0]));
  const ack = Object.assign(new common.CommandAck(), {
      targetSystem: 255,
      targetComponent: 190,
      command: 400,
      result: 0,
    }),
    p = decodeDatagram(new MavLinkProtocolV2(1, 1).serialize(ack, 0))[0];
  bridge.nativePacket(p.header, p.message, p);
  await assert.rejects(
    bridge.configure({ enabled: true, targetPort: port, owner: "web" }),
    /fresh/,
  );
  await new Promise((resolve) => setTimeout(resolve, 10));
  s.heartbeatEpoch++;
  s.lastHeartbeat = Date.now();
  await bridge.configure({ enabled: true, targetPort: port, owner: "web" });
});

test("both session classes reject web flight/upload when external control owns the channel and AP ignores foreign ACKs", async () => {
  const { Px4Session } = await import("../px4-session.js"),
    { ArduPilotSession } = await import("../ardupilot-session.js");
  for (const Type of [Px4Session, ArduPilotSession]) {
    const s = new Type();
    s.groundStation.settings = {
      enabled: true,
      targetPort: 14550,
      owner: "gcs",
    };
    await assert.rejects(s.command("arm"), /owns/);
    await assert.rejects(async () => s.uploadMission([]), /owns/);
    await s.close();
  }
  const s = new ArduPilotSession();
  let resolved = 0;
  s.pendingAcks.set(400, {
    log: {},
    resolve() {
      resolved++;
    },
  });
  s.onTelemetry(77, {
    targetSystem: 255,
    targetComponent: 190,
    command: 400,
    result: 0,
  });
  assert.equal(resolved, 0);
  s.onTelemetry(77, {
    targetSystem: 255,
    targetComponent: 191,
    command: 400,
    result: 0,
  });
  assert.equal(resolved, 1);
  await s.close();
});

test("read-only GET_HOME_POSITION is passed, while guided mission flags, HIL mode bits and wrong altitude frames are rejected", async (t) => {
  const { bridge, receive, port, s, sent } = await monitored(t);
  receive(heartbeat());
  const reads = sent.length;
  receive(command(410));
  assert.equal(sent.length, reads + 1);
  await bridge.configure({ enabled: true, targetPort: port, owner: "gcs" });
  for (const base of [33, NaN, 1.5, 256]) {
    const before = sent.length;
    receive(command(176, [base, 4, 3]));
    assert.equal(sent.length, before);
    assert.match(bridge.snapshot().lastRejection.reason, /HIL/);
  }
  s.config = { ...s.config, geofenceHeight: 10 };
  const item = Object.assign(new common.CommandInt(), {
    targetSystem: 1,
    targetComponent: 1,
    command: 22,
    frame: 6,
    _param5: 250330000,
    _param6: 1215654000,
    _param7: 20,
  });
  let before = sent.length;
  receive(item);
  assert.equal(sent.length, before);
  assert.match(bridge.snapshot().lastRejection.reason, /altitude/);
  item.frame = 0;
  item._param7 = 25;
  receive(item);
  assert.equal(sent.length, before + 1);
  item.frame = 6;
  item._param7 = 5;
  receive(item);
  assert.equal(sent.length, before + 2);
  receive(command(22, [NaN, NaN, 0, NaN, NaN, NaN, 35]));
  assert.equal(sent.length, before + 2);
  receive(command(21));
  assert.equal(sent.length, before + 3);
  const ap = await monitored(t, "ardupilot");
  ap.s.config = { ...ap.s.config, geofenceHeight: 10 };
  ap.receive(heartbeat());
  await ap.bridge.configure({
    enabled: true,
    targetPort: ap.port,
    owner: "gcs",
  });
  const apReads = ap.sent.length;
  ap.receive(command(22, [0, 0, 0, 0, 0, 0, 20]));
  assert.equal(ap.sent.length, apReads);
  assert.match(ap.bridge.snapshot().lastRejection.reason, /altitude/);
  ap.receive(
    Object.assign(new common.MissionCount(), {
      targetSystem: 1,
      targetComponent: 1,
      count: 3,
      missionType: 0,
    }),
  );
  const request = decodeDatagram(
    new MavLinkProtocolV2(1, 1).serialize(
      Object.assign(new common.MissionRequestInt(), {
        targetSystem: 255,
        targetComponent: 190,
        seq: 0,
        missionType: 0,
      }),
      0,
    ),
  )[0];
  ap.bridge.nativePacket(request.header, request.message, request);
  const home = Object.assign(new common.MissionItemInt(), {
    targetSystem: 1,
    targetComponent: 1,
    seq: 0,
    current: 2,
    autocontinue: 1,
    command: 16,
    frame: 6,
    x: 260000000,
    y: 1220000000,
    z: 9000,
    missionType: 0,
  });
  const apBefore = ap.sent.length;
  for (const current of [2, 3]) {
    home.current = current;
    ap.receive(home);
    assert.equal(ap.sent.length, apBefore);
  }
  assert.equal(ap.sent.length, apBefore);
  assert.match(ap.bridge.snapshot().lastRejection.reason, /command/);
  Object.assign(home, {
    current: 1,
    frame: 0,
    x: 250330000,
    y: 1215654000,
    z: 20,
  });
  ap.receive(home);
  assert.equal(ap.sent.length, apBefore + 1);
});

test("critical mission replies bypass disposable telemetry pressure and complete only after successful send callbacks", async (t) => {
  const { bridge, receive, port } = await monitored(t);
  receive(heartbeat());
  await bridge.configure({ enabled: true, targetPort: port, owner: "gcs" });
  receive(
    Object.assign(new common.MissionRequestList(), {
      targetSystem: 1,
      targetComponent: 1,
      missionType: 0,
    }),
  );
  const callbacks = [];
  bridge.socket.getSendQueueSize = () => 300000;
  bridge.socket.send = (...args) => callbacks.push(args.at(-1));
  const fromNative = (m) => {
    const p = decodeDatagram(new MavLinkProtocolV2(1, 1).serialize(m, 0))[0];
    bridge.nativePacket(p.header, p.message, p);
  };
  fromNative(
    Object.assign(new common.MissionCount(), {
      targetSystem: 255,
      targetComponent: 190,
      count: 1,
      missionType: 0,
    }),
  );
  assert.equal(callbacks.length, 1);
  callbacks.shift()(null);
  const item = Object.assign(new common.MissionItemInt(), {
    targetSystem: 255,
    targetComponent: 190,
    seq: 0,
    frame: 6,
    command: 16,
    missionType: 0,
  });
  fromNative(item);
  assert.equal(bridge.mission.active, true);
  assert.equal(bridge.mission.received.size, 0);
  callbacks.shift()(Error("send failed"));
  assert.equal(bridge.mission.active, true);
  assert.equal(bridge.mission.received.size, 0);
  fromNative(item);
  receive(
    Object.assign(new common.MissionAck(), {
      targetSystem: 1,
      targetComponent: 1,
      missionType: 0,
      type: 0,
    }),
  );
  assert.equal(bridge.mission.active, true);
  callbacks.shift()(null);
  assert.equal(bridge.mission.active, false);
  assert.equal(bridge.mission.received.size, 1);
  const sends = callbacks.length;
  fromNative(heartbeat());
  assert.equal(callbacks.length, sends);
  assert.ok(bridge.stats.droppedTelemetryPackets > 0);
});
