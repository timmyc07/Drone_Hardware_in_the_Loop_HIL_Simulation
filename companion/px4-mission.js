import { common } from "node-mavlink";
import { INTERNAL_COMPONENT_ID } from "./mavlink.js";
import { MissionTransfer } from "./mission-transfer.js";
import { DEFAULT_ORIGIN, projectNed } from "./px4-protocol.js";
export function validateMissionPoints(points, config) {
  if (!Array.isArray(points) || points.length < 1 || points.length > 50)
    throw Error("Mission requires 1..50 waypoints");
  return Array.from(points, (p) => {
    if (
      !p ||
      ![Object.prototype, null].includes(Object.getPrototypeOf(p)) ||
      Object.keys(p).sort().join(",") !== "altitude,east,hold,north" ||
      !Object.values(p).every(Number.isFinite) ||
      Math.abs(p.north) > 500 ||
      Math.abs(p.east) > 500 ||
      Math.hypot(p.north, p.east) > config.geofenceRadius ||
      p.altitude < 1 ||
      p.altitude > Math.min(500, config.geofenceHeight) ||
      p.hold < 0 ||
      p.hold > 60
    )
      throw Error("Mission waypoint exceeds position, altitude or hold limits");
    return { ...p };
  });
}
export function compileMission(points, origin = DEFAULT_ORIGIN) {
  const item = (seq, command, north, east, z, hold = 0) => {
    const { latitude, longitude } = projectNed(north, east, origin);
    return {
      seq,
      command,
      frame: 6,
      current: 0,
      autocontinue: 1,
      param1: hold,
      param2: command === 16 ? 0.5 : 0,
      param3: 0,
      param4: 0,
      x: Math.round(latitude * 1e7),
      y: Math.round(longitude * 1e7),
      z,
      missionType: 0,
    };
  };
  return [
    { ...item(0, 22, 0, 0, points[0].altitude), current: 1 },
    ...points.map((p, i) =>
      item(i + 1, 16, p.north, p.east, p.altitude, p.hold),
    ),
    item(points.length + 1, 21, points.at(-1).north, points.at(-1).east, 0),
  ];
}
export function verifyMissionItem(actual, expected) {
  if (
    !actual ||
    actual.seq !== expected.seq ||
    actual.command !== expected.command ||
    actual.missionType !== 0 ||
    ![3, 6].includes(actual.frame) ||
    actual.autocontinue !== 1
  )
    throw Error("PX4 mission item header mismatch");
  for (const key of ["x", "y", "z", "param1", "param2", "param3", "param4"]) {
    const tolerance = ["x", "y"].includes(key) ? 0 : 1e-5;
    if (
      !Number.isFinite(actual[key]) ||
      Math.abs(actual[key] - expected[key]) > tolerance
    )
      throw Error(`PX4 downloaded mission ${expected.seq} differs: ${key}`);
  }
}
export class Px4Mission {
  constructor(session) {
    this.session = session;
    this.home = null;
    this.gpsFix = 0;
    this.evidence = {};
    this.state = {
      phase: "empty",
      waypointStartSeq: 1,
      points: [],
      origin: { ...(session.origin ?? DEFAULT_ORIGIN) },
      limits: {
        radius: session.config?.geofenceRadius ?? 500,
        height: session.config?.geofenceHeight ?? 100,
        minAltitude: 1,
        holdRequiresInteger: false,
      },
      currentSeq: null,
      reachedSeqs: [],
      error: null,
      verified: false,
    };
    this.transfer = new MissionTransfer({
      send: (m) => session.gcs.send(m),
      verifyItem: verifyMissionItem,
      diagnostic: (type, value) => this.diagnostic(type, value),
      onPhase: (phase, error) => {
        this.state.phase = phase === "ready" ? "verifying" : phase;
        this.state.error = error ?? null;
        this.state.verified = false;
      },
    });
  }
  readiness() {
    const s = this.session,
      c = s.controller;
    return Boolean(
      s.phase === "running" &&
      c.verified &&
      c.parametersVerified &&
      c.armedKnown &&
      !c.armed &&
      !c.telemetryArmed &&
      (s.heartbeatEpoch ?? 0) > (s.armingAckEpoch ?? -1) &&
      Date.now() - (s.lastHeartbeat ?? 0) < 2500 &&
      this.home &&
      this.gpsFix >= 3 &&
      s.sim?.snapshot().status === "landed",
    );
  }
  snapshot() {
    return structuredClone({
      ...this.state,
      homeReady: Boolean(this.home) && this.gpsFix >= 3,
      readyToUpload:
        this.readiness() &&
        !this.uploading &&
        !this.transfer.active &&
        !this.session.commandBusy,
    });
  }
  diagnostic(type, value = {}) {
    const s = this.session;
    s.record.missionEvents ??= [];
    if (s.record.missionEvents.length >= 10000) {
      this.state.phase = "error";
      this.state.verified = false;
      this.state.error = "Mission diagnostic limit reached";
      throw Error(this.state.error);
    }
    const event = { time: s.sim.snapshot().time, type, ...value };
    s.record.missionEvents.push(event);
    s.bytes += Buffer.byteLength(JSON.stringify(event)) + 1;
  }
  interrupt(reason) {
    if (this.state.phase === "running" || this.starting) {
      this.starting = false;
      this.state.phase = "interrupted";
      this.state.error = reason;
      try {
        this.diagnostic("interrupted", { reason });
      } catch {}
    }
  }
  abort(reason) {
    this.cancelled = true;
    if (this.uploading) {
      this.state.phase = "error";
      this.state.error = reason;
      this.state.verified = false;
    }
    this.transfer.abort(reason);
    this.interrupt(reason);
  }
  async upload(points) {
    if (
      this.uploading ||
      !this.readiness() ||
      this.transfer.active ||
      this.session.commandBusy
    )
      throw new Error(
        "Mission upload requires exclusive, verified disarmed-ground readiness",
      );
    this.uploading = true;
    this.cancelled = false;
    this.uploadDeadlineTimer = setTimeout(() => {
      this.cancelled = true;
      this.transfer.abort("Mission upload total wall deadline exceeded");
    }, 15000);
    try {
      return await this.uploadRoute(points);
    } catch (error) {
      this.state.phase = "error";
      this.state.verified = false;
      this.state.error = error.message;
      throw error;
    } finally {
      clearTimeout(this.uploadDeadlineTimer);
      this.uploading = false;
    }
  }
  async uploadRoute(points) {
    if (!this.readiness() || this.transfer.active || this.session.commandBusy)
      throw Error(
        "Mission upload requires verified, home-ready, disarmed ground session",
      );
    Object.assign(this.state, {
      phase: "error",
      verified: false,
      error: "Mission upload not verified",
    });
    const canonical = validateMissionPoints(points, this.session.config);
    this.items = compileMission(canonical, this.session.origin);
    Object.assign(this.state, {
      phase: "uploading",
      points: canonical,
      currentSeq: null,
      reachedSeqs: [],
      error: null,
    });
    delete this.state.completionEvidence;
    this.evidence = {};
    this.diagnostic("upload", {
      points: canonical,
      items: this.items,
      home: this.home,
    });
    const deadline = Date.now() + 15000;
    if (this.session.controller.flightMode === "AUTO_MISSION") {
      const epoch = this.session.heartbeatEpoch ?? 0;
      await this.session.gcs.command(176, [1, 4, 3]);
      const until = Math.min(deadline, Date.now() + 5000);
      while (
        this.session.controller.flightMode !== "PX4_4_3" ||
        (this.session.heartbeatEpoch ?? 0) <= epoch
      ) {
        if (
          this.cancelled ||
          this.session.phase !== "running" ||
          Date.now() > until
        )
          throw new Error(
            "PX4 HOLD heartbeat was not confirmed before mission upload",
          );
        await new Promise((resolve) => setTimeout(resolve, 25));
      }
      if (this.cancelled || !this.readiness())
        throw new Error(
          "Mission upload readiness changed while selecting HOLD",
        );
      this.diagnostic("holdBeforeUpload", {
        mode: this.session.controller.flightMode,
      });
    }
    // Same-CRC uploads are not a restart in this PX4 tag. Clear through the mission service first.
    const clear = new common.MissionClearAll();
    Object.assign(clear, {
      targetSystem: 1,
      targetComponent: 1,
      missionType: 0,
    });
    this.clearing = { acked: false, empty: false, error: null, epoch: null };
    this.diagnostic("clearBeforeUpload", {});
    let lastClear = 0;
    try {
      while (
        !this.clearing.acked ||
        !this.clearing.empty ||
        this.clearing.epoch === null ||
        (this.session.heartbeatEpoch ?? 0) <= this.clearing.epoch
      ) {
        if (
          this.cancelled ||
          this.session.phase !== "running" ||
          Date.now() > deadline
        )
          throw new Error("PX4 mission clear was not confirmed before upload");
        if (this.clearing.error) throw new Error(this.clearing.error);
        if (!this.clearing.acked && Date.now() - lastClear >= 1000) {
          lastClear = Date.now();
          this.session.gcs.send(clear);
        }
        await new Promise((resolve) => setTimeout(resolve, 25));
      }
    } finally {
      this.clearing = null;
    }
    if (this.cancelled || !this.readiness())
      throw new Error(
        "Mission upload readiness changed after clearing mission",
      );
    this.state.missionState = null;
    this.state.total = null;
    this.expectedMissionId = null;
    await this.transfer.upload(this.items);
    while (
      this.state.missionState !== 2 ||
      this.state.currentSeq !== 0 ||
      this.state.total !== this.items.length
    ) {
      if (
        Date.now() > deadline ||
        this.session.phase !== "running" ||
        this.cancelled
      ) {
        this.state.phase = "error";
        this.state.error = "PX4 navigator did not confirm valid mission";
        throw new Error(this.state.error);
      }
      await new Promise((resolve) => setTimeout(resolve, 25));
    }
    const navigatorEpoch = this.session.heartbeatEpoch ?? 0;
    while ((this.session.heartbeatEpoch ?? 0) <= navigatorEpoch) {
      if (
        Date.now() > deadline ||
        this.session.phase !== "running" ||
        this.cancelled
      )
        throw new Error("PX4 post-mission heartbeat timeout");
      await new Promise((resolve) => setTimeout(resolve, 25));
    }
    this.state.phase = "ready";
    this.state.verified = true;
    this.diagnostic("navigatorReady", {
      missionState: this.state.missionState,
      total: this.state.total,
    });
    return this.snapshot();
  }
  handle(id, m) {
    if (this.clearing && id === 47) {
      if (
        m.targetSystem !== 255 ||
        ![0, INTERNAL_COMPONENT_ID].includes(m.targetComponent) ||
        m.missionType !== 0
      ) {
        this.clearing.error = "Unexpected PX4 mission-clear ACK target/type";
        return;
      }
      this.diagnostic("clearAck", { result: m.type });
      if (m.type === 0) this.clearing.acked = true;
      else this.clearing.error = `PX4 mission clear rejected (${m.type})`;
      if (
        this.clearing.acked &&
        this.clearing.empty &&
        this.clearing.epoch === null
      )
        this.clearing.epoch = this.session.heartbeatEpoch ?? 0;
      return;
    }
    if (this.clearing && id === 42) {
      this.clearing.empty =
        m.seq === 0 &&
        m.total === 65535 &&
        m.missionId === 0 &&
        m.missionState === 1;
      if (!this.clearing.empty) this.clearing.epoch = null;
      else if (this.clearing.acked && this.clearing.epoch === null)
        this.clearing.epoch = this.session.heartbeatEpoch ?? 0;
      return;
    }
    if (id === 242) {
      const o = this.session.origin ?? DEFAULT_ORIGIN;
      if (![m.latitude, m.longitude, m.altitude].every(Number.isInteger))
        throw Error("Invalid PX4 HOME_POSITION");
      const distance = Math.hypot(
        (((m.latitude / 1e7 - o.latitude) * Math.PI) / 180) * 6378137,
        (((m.longitude / 1e7 - o.longitude) * Math.PI) / 180) *
          6378137 *
          Math.cos((o.latitude * Math.PI) / 180),
      );
      if (distance > 2 || Math.abs(m.altitude / 1000 - o.altitudeMsl) > 2) {
        this.home = null;
        this.abort("PX4 home differs from simulator origin");
        return;
      }
      this.home = {
        latitude: m.latitude,
        longitude: m.longitude,
        altitude: m.altitude,
      };
      return;
    }
    if (id === 24) {
      if (!Number.isInteger(m.fixType) || m.fixType < 0 || m.fixType > 8)
        throw new Error("Invalid PX4 GPS fix type");
      this.gpsFix = m.fixType;
      return;
    }
    if (id === 245) {
      if (
        !Number.isInteger(m.landedState) ||
        m.landedState < 0 ||
        m.landedState > 4
      )
        throw Error("Invalid PX4 landed state");
      this.state.landedState = m.landedState;
      this.update();
      return;
    }
    if (
      id === 44 &&
      this.transfer.active &&
      Number.isInteger(m.opaqueId) &&
      m.opaqueId > 0
    )
      this.expectedMissionId = m.opaqueId;
    if (this.transfer.handle(id, m)) return;
    if (
      id === 42 &&
      this.state.phase === "verifying" &&
      (this.expectedMissionId === null ||
        m.missionId === this.expectedMissionId)
    ) {
      this.state.missionState = m.missionState;
      this.state.total = m.total;
      this.state.currentSeq = m.seq;
      this.state.missionId = m.missionId ?? null;
    }
    if (
      !this.state.verified ||
      !["ready", "running", "completed"].includes(this.state.phase)
    )
      return;
    if (
      id === 42 &&
      Number.isInteger(m.seq) &&
      m.seq >= 0 &&
      m.seq < this.items.length
    ) {
      if (
        m.total !== this.items.length ||
        !Number.isInteger(m.missionState) ||
        m.missionState < 0 ||
        m.missionState > 5
      )
        throw new Error("PX4 mission telemetry count/state mismatch");
      if (
        this.expectedMissionId !== null &&
        this.expectedMissionId !== undefined &&
        m.missionId !== this.expectedMissionId
      )
        throw new Error("PX4 active mission identity changed");
      if (
        (this.state.phase === "running" || this.starting) &&
        m.missionState === 3 &&
        !this.evidence.active
      )
        this.evidence.active = { time: this.session.sim.snapshot().time };
      const changed =
        this.state.currentSeq !== m.seq ||
        this.state.missionState !== m.missionState;
      this.state.currentSeq = m.seq;
      this.state.missionState = m.missionState;
      if ((this.state.phase === "running" || this.starting) && changed)
        this.diagnostic("current", {
          seq: m.seq,
          missionState: m.missionState,
          total: m.total,
        });
      if (
        (this.state.phase === "running" || this.starting) &&
        m.missionState === 5 &&
        m.seq === this.items.length - 1 &&
        this.evidence.active &&
        this.evidence.airborne &&
        this.items
          .slice(1, -1)
          .every((item) => this.state.reachedSeqs.includes(item.seq)) &&
        !this.evidence.complete
      )
        this.evidence.complete = {
          time: this.session.sim.snapshot().time,
          seq: m.seq,
          missionState: 5,
        };
    }
    if (
      id === 46 &&
      (this.state.phase === "running" || this.starting) &&
      Number.isInteger(m.seq) &&
      m.seq >= 0 &&
      m.seq < this.items.length &&
      !this.state.reachedSeqs.includes(m.seq)
    ) {
      this.state.reachedSeqs.push(m.seq);
      this.diagnostic("reached", {
        seq: m.seq,
        truth: this.session.sim.snapshot().position,
      });
    }
    this.update();
  }
  async start() {
    if (
      !this.readiness() ||
      this.state.phase !== "ready" ||
      !this.state.verified
    )
      throw Error(
        "A verified mission and disarmed ground readiness are required",
      );
    this.starting = true;
    this.cancelled = false;
    this.evidence = {};
    this.state.reachedSeqs = [];
    delete this.state.completionEvidence;
    const checkStarting = () => {
      if (this.session.phase !== "running" || !this.starting || this.cancelled)
        throw new Error("PX4 mission start cancelled");
    };
    try {
      await this.session.gcs.command(300, [0, 0]);
      checkStarting();
      const until = Date.now() + 5000;
      while (
        this.session.controller.flightMode !== "AUTO_MISSION" ||
        !this.session.controller.armed
      ) {
        if (Date.now() > until || this.session.phase !== "running")
          throw Error("PX4 AUTO_MISSION / HIL armed confirmation timeout");
        await new Promise((resolve) => setTimeout(resolve, 20));
        checkStarting();
      }
      checkStarting();
      this.state.phase = "running";
      this.starting = false;
      this.diagnostic("start");
      this.update();
    } catch (error) {
      this.starting = false;
      this.state.phase = "error";
      this.state.error = error.message;
      this.state.verified = false;
      try {
        this.diagnostic("error", { reason: error.message });
      } catch {}
      throw error;
    }
  }
  update() {
    if (this.state.phase !== "running" && !this.starting) return;
    const s = this.session,
      p = s.sim.snapshot(),
      e = this.evidence;
    if (s.controller.armed && !e.armed) e.armed = { time: p.time };
    if (-p.position[2] > 1 && !e.airborne) e.airborne = { time: p.time };
    if (
      this.state.phase === "running" &&
      s.controller.telemetryArmed &&
      s.controller.flightMode !== "AUTO_MISSION"
    ) {
      this.interrupt("PX4 left AUTO_MISSION");
      return;
    }
    if (
      e.complete &&
      e.armed &&
      e.airborne &&
      this.items
        .slice(1, -1)
        .every((item) => this.state.reachedSeqs.includes(item.seq)) &&
      this.state.landedState === 1 &&
      !s.controller.armed &&
      !s.controller.telemetryArmed &&
      s.controller.armedKnown &&
      Date.now() - (s.lastHeartbeat ?? 0) < 2500 &&
      p.status === "landed" &&
      Math.hypot(...p.velocity) < 0.15
    ) {
      this.state.phase = "completed";
      this.state.completionEvidence = {
        ...e,
        ground: {
          time: p.time,
          landedState: 1,
          heartbeatDisarmed: true,
          hilDisarmed: true,
          plantSpeed: Math.hypot(...p.velocity),
        },
      };
      this.diagnostic("completed", this.state.completionEvidence);
    }
  }
}
