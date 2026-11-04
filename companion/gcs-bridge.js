import dgram from "node:dgram";
import {
  decodeDatagram,
  INTERNAL_COMPONENT_ID,
  EXTERNAL_COMPONENT_ID,
} from "./mavlink.js";
import { DEFAULT_ORIGIN } from "./px4-protocol.js";
const MAX_BYTES = 96 * 1024 * 1024,
  MAX_EVENTS = 100000;
const MISSIONS = new Set([39, 40, 41, 43, 44, 45, 47, 51, 73]);
const READ_MESSAGES = new Set([0, 4, 20, 21, 66, 111, 183]);
const READ_COMMANDS = new Set([410, 511, 512, 519, 520]);
const FLIGHT_COMMANDS = new Set([21, 22, 176, 193, 224, 300, 400]);
const finite = (v, min, max) => Number.isFinite(v) && v >= min && v <= max;
const integer = (v, min, max) => Number.isInteger(v) && v >= min && v <= max;
const requireValue = (ok, reason) => {
  if (!ok) throw Error(reason);
};
export class GroundStationBridge {
  constructor(session, { sendRaw, nativeReady }) {
    this.session = session;
    this.sendRaw = sendRaw;
    this.nativeReady = nativeReady;
    this.settings = { enabled: false, targetPort: 14550, owner: "web" };
    this.pending = new Map();
    this.mission = null;
    this.stats = {
      receivedPackets: 0,
      forwardedPackets: 0,
      telemetryPackets: 0,
      rejectedPackets: 0,
      receivedBytes: 0,
      telemetryBytes: 0,
      droppedTelemetryPackets: 0,
    };
    this.lastRejection = null;
    this.lastHeartbeat = 0;
    this.configureBusy = false;
    this.tokens = 200;
    this.refill = Date.now();
  }
  snapshot() {
    return {
      ...this.settings,
      localPort: this.socket?.address?.().port ?? null,
      connected:
        this.settings.enabled &&
        this.lastHeartbeat > 0 &&
        Date.now() - this.lastHeartbeat <= 5000,
      lastHeartbeatAgeMs: this.lastHeartbeat
        ? Date.now() - this.lastHeartbeat
        : null,
      stats: { ...this.stats },
      lastRejection: this.lastRejection ? { ...this.lastRejection } : null,
      internalIdentity: { system: 255, component: INTERNAL_COMPONENT_ID },
      externalIdentity: { system: 255, component: EXTERNAL_COMPONENT_ID },
      mission: this.mission
        ? {
            kind: this.mission.kind,
            count: this.mission.count,
            active: this.mission.active,
          }
        : null,
    };
  }
  get externalOwner() {
    return this.settings.enabled && this.settings.owner === "gcs";
  }
  internalIdle() {
    const s = this.session,
      m = s.missionManager;
    return (
      !s.commandBusy &&
      !s.gcs?.pending?.size &&
      !s.pendingAcks?.size &&
      !m?.transfer?.active &&
      !m?.uploading &&
      !m?.starting &&
      !m?.startingFlight
    );
  }
  groundReady() {
    const s = this.session,
      c = s.controller,
      p = s.sim?.snapshot();
    return (
      ["running", "paused"].includes(s.phase) &&
      c.verified &&
      c.parametersVerified &&
      c.armedKnown &&
      !c.armed &&
      !c.telemetryArmed &&
      Date.now() - (s.lastHeartbeat ?? 0) < 2500 &&
      (s.heartbeatEpoch ?? 0) > (s.armingAckEpoch ?? -1) &&
      p?.status === "landed" &&
      !p.armed &&
      Math.hypot(...p.velocity) < 0.2
    );
  }
  append(type, value = {}) {
    const s = this.session;
    requireValue(s.record, "No session recording");
    const event = { timeUs: s.sim.readSensors().timeUs, type, ...value },
      size = Buffer.byteLength(JSON.stringify(event)) + 1;
    s.record.groundStationEvents ??= [];
    if (
      s.record.groundStationEvents.length >= MAX_EVENTS ||
      s.bytes + size >= MAX_BYTES - 10000
    ) {
      s.record.limitReached = true;
      s.fail(
        Error(
          "Ground station recording limit reached; plant frozen before forwarding",
        ),
      );
      throw Error("Ground station recording limit reached");
    }
    s.record.groundStationEvents.push(event);
    s.bytes += size;
    return event;
  }
  async configure(value) {
    requireValue(
      !this.closed && !this.configureBusy,
      "Ground station configuration is busy or closed",
    );
    requireValue(
      value &&
        Object.getPrototypeOf(value) === Object.prototype &&
        Object.keys(value).sort().join(",") === "enabled,owner,targetPort" &&
        typeof value.enabled === "boolean" &&
        ["web", "gcs"].includes(value.owner) &&
        integer(value.targetPort, 1024, 65535) &&
        ![4560, 5760, 9002, 14560, 14561].includes(value.targetPort) &&
        (value.enabled || value.owner === "web"),
      "Invalid ground station settings",
    );
    const s = this.session;
    requireValue(
      ["running", "paused"].includes(s.phase) &&
        s.controller.verified &&
        this.nativeReady(),
      "Ground station requires a verified connected native session",
    );
    const ownershipChange = value.owner !== this.settings.owner,
      disabling = this.settings.enabled && !value.enabled;
    if (ownershipChange || disabling)
      requireValue(
        this.groundReady() &&
          this.internalIdle() &&
          !this.pending.size &&
          !this.mission?.active,
        "Control handoff requires fresh disarmed ground state and idle transactions",
      );
    if (value.owner === "gcs")
      requireValue(
        this.settings.enabled &&
          this.settings.targetPort === value.targetPort &&
          this.lastHeartbeat > 0 &&
          Date.now() - this.lastHeartbeat <= 2500,
        "Enable monitoring and wait for a fresh ground station heartbeat before takeover",
      );
    if (this.externalOwner && value.targetPort !== this.settings.targetPort)
      throw Error("Return control to web before changing the endpoint");
    this.configureBusy = true;
    let socket;
    try {
      if (
        value.enabled &&
        (!this.socket || value.targetPort !== this.settings.targetPort)
      ) {
        socket = dgram.createSocket("udp4");
        socket.on("error", (error) => {
          if (!this.closed) s.fail(error);
        });
        await new Promise((resolve, reject) => {
          socket.once("error", reject);
          socket.bind(0, "127.0.0.1", resolve);
        });
      }
      requireValue(
        !this.closed && ["running", "paused"].includes(s.phase),
        "Session stopped during ground station configuration",
      );
      if (ownershipChange || disabling)
        requireValue(
          this.groundReady() &&
            this.internalIdle() &&
            !this.pending.size &&
            !this.mission?.active,
          "Control handoff state changed during configuration",
        );
      this.append("configuration", { settings: { ...value } });
      const old = this.socket;
      if (socket) {
        this.socket = socket;
        this.lastHeartbeat = 0;
        socket.on("message", (data, peer) => this.receive(data, peer));
      }
      if (!value.enabled) this.socket = null;
      this.settings = { ...value };
      s.record.groundStation = {
        ...value,
        internalIdentity: { system: 255, component: INTERNAL_COMPONENT_ID },
        externalIdentity: { system: 255, component: EXTERNAL_COMPONENT_ID },
      };
      if (ownershipChange) {
        s.missionManager?.interrupt("Ground station control ownership changed");
        if (s.missionManager?.state) s.missionManager.state.verified = false;
      }
      if (old && old !== this.socket)
        await new Promise((resolve) => old.close(resolve));
      clearInterval(this.timer);
      if (this.closed) return s.state();
      if (value.enabled)
        this.timer = setInterval(() => this.checkWatchdog(), 250);
      s.publish();
      return s.state();
    } catch (error) {
      if (socket && socket !== this.socket)
        await new Promise((resolve) => {
          try {
            socket.close(resolve);
          } catch {
            resolve();
          }
        });
      throw error;
    } finally {
      this.configureBusy = false;
    }
  }
  reject(reason, id) {
    this.stats.rejectedPackets++;
    this.lastRejection = {
      reason: String(reason),
      messageId: id ?? null,
      timeUs: this.session.sim?.readSensors().timeUs ?? null,
    };
  }
  receive(data, peer) {
    if (
      !this.settings.enabled ||
      peer.address !== "127.0.0.1" ||
      peer.port !== this.settings.targetPort
    )
      return;
    this.tokens = Math.min(200, this.tokens + (Date.now() - this.refill) * 0.1);
    this.refill = Date.now();
    let frames;
    try {
      frames = decodeDatagram(data);
    } catch (error) {
      this.reject(error.message);
      return;
    }
    for (const packet of frames) {
      const { header, message, buffer } = packet;
      this.stats.receivedPackets++;
      this.stats.receivedBytes += buffer.length;
      try {
        requireValue(this.tokens >= 1, "Ground station packet rate exceeded");
        this.tokens--;
        requireValue(
          header.sysid === 255 && header.compid === EXTERNAL_COMPONENT_ID,
          "Ground station must use system255/component190",
        );
        this.validate(header.msgid, message, header);
        if (header.msgid === 0) this.lastHeartbeat = Date.now();
        const control =
          MISSIONS.has(header.msgid) ||
          header.msgid === 11 ||
          ([75, 76].includes(header.msgid) &&
            !READ_COMMANDS.has(message.command));
        const logged = control
          ? this.append("outboundControl", {
              messageId: header.msgid,
              rawBase64: buffer.toString("base64"),
              owner: this.settings.owner,
              forwarded: false,
            })
          : null;
        this.trackOutgoing(header.msgid, message);
        this.sendRaw(buffer);
        if (logged) logged.forwarded = true;
        this.stats.forwardedPackets++;
      } catch (error) {
        this.reject(error.message, header.msgid);
      }
    }
  }
  validate(id, m, header) {
    requireValue(
      ["running", "paused"].includes(this.session.phase),
      "Session is not accepting GCS packets",
    );
    if (id === 0) {
      requireValue(
        m.type === 6 && m.autopilot === 8,
        "Expected a ground-station heartbeat",
      );
      return;
    }
    if ("targetSystem" in m)
      requireValue(
        m.targetSystem === 1,
        "Ground station target must be vehicle1",
      );
    if ("targetComponent" in m)
      requireValue(
        [0, 1].includes(m.targetComponent),
        "Unsupported vehicle component",
      );
    if (READ_MESSAGES.has(id)) {
      if (id === 66)
        requireValue(
          integer(m.reqStreamId, 0, 12) &&
            integer(m.reqMessageRate, 0, 50) &&
            [0, 1].includes(m.startStop),
          "Invalid telemetry stream request",
        );
      return;
    }
    if ([75, 76].includes(id) && READ_COMMANDS.has(m.command)) {
      requireValue(
        !this.session.gcs?.pending?.has(m.command) &&
          !this.session.pendingAcks?.has(m.command),
        "Internal command owns this ACK",
      );
      if (m.command === 512)
        requireValue(
          integer(m._param1, 0, 16777215),
          "Invalid requested message",
        );
      if (m.command === 511)
        requireValue(
          integer(m._param1, 0, 16777215) &&
            (m._param2 === -1 ||
              m._param2 === 0 ||
              finite(m._param2, 10000, 10000000)),
          "Invalid telemetry interval",
        );
      return;
    }
    requireValue(this.externalOwner, "Web owns flight and mission control");
    requireValue(
      header.magic !== 254,
      "Control requires MAVLink2 target-aware acknowledgements",
    );
    if (MISSIONS.has(id)) {
      this.validateMission(id, m);
      return;
    }
    if (id === 11) {
      requireValue(
        (m.baseMode & 32) === 0,
        "Ground station cannot toggle HIL mode",
      );
      this.validateMode(m.customMode);
      return;
    }
    requireValue(
      [75, 76].includes(id) && FLIGHT_COMMANDS.has(m.command),
      "Unsupported ground station control message",
    );
    const p = (i) => m[`_param${i}`];
    if (m.command === 400)
      requireValue(
        [0, 1].includes(p(1)) && p(2) === 0,
        "Only normal arm/disarm is permitted",
      );
    if ([21, 22].includes(m.command)) {
      requireValue(
        Number.isNaN(p(4)) || finite(p(4), -360, 360),
        "Invalid yaw",
      );
      const latitude = id === 75 ? p(5) / 1e7 : p(5),
        longitude = id === 75 ? p(6) / 1e7 : p(6),
        altitude = p(7);
      if (
        !(Number.isNaN(latitude) && Number.isNaN(longitude)) &&
        !(latitude === 0 && longitude === 0)
      )
        this.validatePosition(latitude, longitude);
      if (id === 75)
        requireValue(
          [0, 3, 5, 6].includes(m.frame),
          "Unsupported command coordinate frame",
        );
      const relative =
        id === 75
          ? [0, 5].includes(m.frame)
            ? altitude - this.origin().altitudeMsl
            : altitude
          : this.session.controller.kind === "px4"
            ? altitude - this.origin().altitudeMsl
            : altitude;
      requireValue(
        (m.command === 21 && altitude === 0) ||
          finite(relative, 0, this.session.config.geofenceHeight),
        "Invalid takeoff/land altitude",
      );
    }
    if (m.command === 176) {
      requireValue(
        integer(p(1), 0, 255) && (p(1) & 32) === 0,
        "Ground station cannot toggle HIL mode",
      );
      if (this.session.controller.kind === "ardupilot") this.validateMode(p(2));
      else this.validateMode((p(2) << 16) | (p(3) << 24));
    }
    if ([193, 224, 300].includes(m.command))
      requireValue(
        [p(1), p(2)].every((v) => finite(v, 0, 53)),
        "Invalid mission command arguments",
      );
  }
  origin() {
    return this.session.origin ?? this.session.record?.origin ?? DEFAULT_ORIGIN;
  }
  validatePosition(latitude, longitude) {
    requireValue(
      finite(latitude, -85, 85) && finite(longitude, -180, 180),
      "Invalid geographic coordinate",
    );
    const o = this.origin(),
      north = (((latitude - o.latitude) * Math.PI) / 180) * 6378137,
      east =
        (((longitude - o.longitude) * Math.PI) / 180) *
        6378137 *
        Math.cos((o.latitude * Math.PI) / 180);
    requireValue(
      Math.hypot(north, east) <= this.session.config.geofenceRadius + 1,
      "Ground station target is outside the local fence",
    );
  }
  validateMode(mode) {
    requireValue(integer(mode, 0, 0xffffffff), "Invalid flight mode");
    if (this.session.controller.kind === "ardupilot")
      requireValue(
        [0, 3, 4, 5, 6, 9, 16].includes(mode),
        "Unsupported ArduPilot mode",
      );
    else {
      const main = (mode >>> 16) & 255,
        sub = (mode >>> 24) & 255;
      requireValue(
        [2, 3].includes(main) || (main === 4 && [2, 3, 4, 5, 6].includes(sub)),
        "Unsupported PX4 mode",
      );
    }
  }
  validateMission(id, m) {
    if ("missionType" in m)
      requireValue(m.missionType === 0, "Only flight missions are supported");
    if ([44, 45].includes(id))
      requireValue(
        this.groundReady(),
        "Mission writes require a disarmed ground aircraft",
      );
    if ([43, 44, 45].includes(id))
      requireValue(
        !this.mission?.active ||
          (id === 44 &&
            this.mission.kind === "upload" &&
            this.mission.count === m.count),
        "A mission transaction is already active",
      );
    if (id === 44)
      requireValue(
        integer(
          m.count,
          0,
          this.session.controller.kind === "ardupilot" ? 53 : 52,
        ),
        "Mission count exceeds route limits",
      );
    if ([40, 41, 51].includes(id))
      requireValue(
        integer(m.seq, 0, (this.mission?.count ?? 0) - 1),
        "Invalid mission sequence",
      );
    if ([39, 73].includes(id)) {
      requireValue(
        this.mission?.active &&
          this.mission.kind === "upload" &&
          integer(m.seq, 0, this.mission.count - 1),
        "No matching mission upload",
      );
      requireValue(
        this.mission.requested.has(m.seq),
        "Mission item was not requested by the vehicle",
      );
      requireValue(
        [16, 21, 22].includes(m.command) &&
          [0, 3, 5, 6].includes(m.frame) &&
          [0, 1].includes(m.current) &&
          m.autocontinue === 1,
        "Unsupported mission command/frame",
      );
      requireValue(
        finite(m.param1, 0, 60) &&
          finite(m.param2, 0, 100) &&
          finite(m.param3, 0, 100) &&
          (Number.isNaN(m.param4) || finite(m.param4, -360, 360)),
        "Invalid mission parameters",
      );
      const home = this.session.controller.kind === "ardupilot" && m.seq === 0;
      if (home)
        requireValue(
          finite(m.x, -180e7, 180e7) &&
            finite(m.y, -180e7, 180e7) &&
            finite(m.z, -500, 9000),
          "Invalid ArduPilot home placeholder",
        );
      if (!home)
        this.validatePosition(
          id === 73 ? m.x / 1e7 : m.x,
          id === 73 ? m.y / 1e7 : m.y,
        );
      else requireValue(m.command === 16, "ArduPilot first item must be home");
      const altitude =
        m.z - ([0, 5].includes(m.frame) ? this.origin().altitudeMsl : 0);
      requireValue(
        home ||
          finite(
            altitude,
            m.command === 21 ? 0 : 1,
            this.session.config.geofenceHeight,
          ),
        "Mission altitude exceeds limits",
      );
      if (this.session.controller.kind === "ardupilot")
        requireValue(
          Number.isInteger(m.param1),
          "ArduPilot hold time requires integer seconds",
        );
      const shape = JSON.stringify(m),
        prior = this.mission.sent.get(m.seq);
      requireValue(!prior || prior === shape, "Mission retry changed an item");
    }
  }
  trackOutgoing(id, m) {
    if ([43, 44, 45].includes(id)) {
      this.mission = {
        kind: id === 43 ? "download" : id === 45 ? "clear" : "upload",
        count: m.count ?? 0,
        active: true,
        deadline: Date.now() + 15000,
        requested: new Set(),
        sent: new Map(),
        received: new Set(),
      };
    }
    if ([39, 73].includes(id)) this.mission.sent.set(m.seq, JSON.stringify(m));
    if ([75, 76].includes(id) && !READ_COMMANDS.has(m.command))
      this.pending.set(m.command, Date.now() + 5000);
  }
  nativePacket(header, message, packet) {
    if (header.sysid !== 1 || header.compid !== 1) return false;
    const targetSystem = message.targetSystem,
      targetComponent = message.targetComponent;
    const external =
      targetSystem !== undefined &&
      [0, 255].includes(targetSystem) &&
      targetComponent === EXTERNAL_COMPONENT_ID;
    const internal =
      targetSystem !== undefined &&
      [0, 255].includes(targetSystem) &&
      targetComponent === INTERNAL_COMPONENT_ID;
    if (this.settings.enabled && !internal) {
      try {
        const ownedReply =
          this.externalOwner &&
          (external || targetComponent === 0 || targetComponent === undefined);
        const critical =
          ownedReply &&
          (MISSIONS.has(header.msgid) ||
            (header.msgid === 77 && !READ_COMMANDS.has(message.command)));
        const operation = this.mission;
        let receipt;
        if (critical)
          receipt = this.append(
            header.msgid === 77 ? "controlAck" : "missionReply",
            {
              messageId: header.msgid,
              rawBase64: packet.buffer.toString("base64"),
              forwarded: false,
            },
          );
        if (
          ownedReply &&
          header.msgid === 77 &&
          message.result === 0 &&
          [176, 300, 400].includes(message.command)
        )
          this.session.armingAckEpoch = this.session.heartbeatEpoch ?? 0;
        if (ownedReply && operation?.active) {
          if ([40, 51].includes(header.msgid))
            operation.requested.add(message.seq);
          if (header.msgid === 44) {
            requireValue(
              integer(
                message.count,
                0,
                this.session.controller.kind === "ardupilot" ? 53 : 52,
              ),
              "Native mission count exceeds limits",
            );
            operation.count = message.count;
          }
          if ([39, 73].includes(header.msgid) && operation.kind === "download")
            requireValue(
              integer(message.seq, 0, operation.count - 1),
              "Native mission sequence exceeds count",
            );
        }
        // Discard ordinary telemetry under pressure; bounded transaction replies keep their delivery callback.
        if (!critical && this.socket.getSendQueueSize() >= 256 * 1024)
          this.stats.droppedTelemetryPackets++;
        else
          this.socket.send(
            packet.buffer,
            this.settings.targetPort,
            "127.0.0.1",
            (error) => {
              if (error) {
                if (!this.closed) this.reject(error.message, header.msgid);
                return;
              }
              if (this.closed) return;
              if (receipt) receipt.forwarded = true;
              this.stats.telemetryPackets++;
              this.stats.telemetryBytes += packet.buffer.length;
              if (ownedReply && header.msgid === 77 && message.result !== 5)
                this.pending.delete(message.command);
              if (
                ownedReply &&
                operation?.active &&
                this.mission === operation
              ) {
                if (
                  header.msgid === 44 &&
                  operation.kind === "download" &&
                  message.count === 0
                )
                  operation.active = false;
                if (
                  [39, 73].includes(header.msgid) &&
                  operation.kind === "download"
                ) {
                  operation.received.add(message.seq);
                  // Full wire delivery ends the lease; it does not prove that the GCS consumed or verified it.
                  if (operation.received.size === operation.count)
                    operation.active = false;
                }
                if (header.msgid === 47) operation.active = false;
              }
            },
          );
      } catch (error) {
        this.session.fail(error);
      }
    }
    return (
      !external &&
      !(
        this.externalOwner &&
        (MISSIONS.has(header.msgid) ||
          header.msgid === 77 ||
          [42, 46].includes(header.msgid))
      ) &&
      (targetSystem === undefined || [0, 255].includes(targetSystem))
    );
  }
  checkWatchdog() {
    const now = Date.now();
    for (const [command, until] of this.pending)
      if (now > until) {
        this.pending.delete(command);
        this.reject("Ground station command ACK timeout", command);
      }
    if (this.mission?.active && now > this.mission.deadline) {
      this.mission.active = false;
      this.reject("Ground station mission timeout");
    }
    if (this.externalOwner && now - this.lastHeartbeat > 5000)
      this.session.fail(
        Error("Ground station heartbeat lost; plant clock frozen"),
      );
  }
  async close() {
    if (this.closed) return;
    this.closed = true;
    clearInterval(this.timer);
    this.pending.clear();
    if (this.mission) this.mission.active = false;
    const socket = this.socket;
    this.socket = null;
    this.settings = { ...this.settings, enabled: false, owner: "web" };
    if (socket)
      await new Promise((resolve) => {
        try {
          socket.close(resolve);
        } catch {
          resolve();
        }
      });
  }
}
