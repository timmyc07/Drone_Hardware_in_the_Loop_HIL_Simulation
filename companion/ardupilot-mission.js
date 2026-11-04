import { common } from "node-mavlink";
import { commandMessage } from "./ardupilot-protocol.js";
const { MavCmd, MavFrame, MavMissionType, MavLandedState } = common;
export const MISSION_ORIGIN = Object.freeze({
  latitude: 25.033,
  longitude: 121.5654,
  altitudeMsl: 20,
});
const SCALE = 0.011131884502145034;
export function projectWaypoint(north, east) {
  if (
    !Number.isFinite(north) ||
    !Number.isFinite(east) ||
    Math.abs(north) > 500 ||
    Math.abs(east) > 500
  )
    throw new Error("Invalid local mission coordinates");
  // Tagged AP Location::offset_latlng: truncate degE7 offsets; longitude uses midpoint latitude.
  const lat = Math.round(MISSION_ORIGIN.latitude * 1e7),
    lon = Math.round(MISSION_ORIGIN.longitude * 1e7);
  const dlat = Math.trunc(north / SCALE),
    dlon = Math.trunc(
      east /
        SCALE /
        Math.max(
          0.01,
          Math.cos(((lat + Math.trunc(dlat / 2)) * 1e-7 * Math.PI) / 180),
        ),
    );
  return { x: lat + dlat, y: lon + dlon };
}
export function validateMissionPoints(points, config) {
  if (!Array.isArray(points) || points.length < 1 || points.length > 50)
    throw new Error("Mission requires1..50 waypoints");
  return Array.from(points, (point) => {
    if (
      !point ||
      ![Object.prototype, null].includes(Object.getPrototypeOf(point)) ||
      Object.keys(point).length !== 4 ||
      Object.keys(point).some(
        (k) => !["north", "east", "altitude", "hold"].includes(k),
      )
    )
      throw new Error("Each mission point requires north,east,altitude,hold");
    if (
      !Object.values(point).every(Number.isFinite) ||
      Math.abs(point.north) > 500 ||
      Math.abs(point.east) > 500 ||
      Math.hypot(point.north, point.east) > config.geofenceRadius ||
      point.altitude < 1 ||
      point.altitude > Math.min(500, config.geofenceHeight) ||
      !Number.isInteger(point.hold) ||
      point.hold < 0 ||
      point.hold > 60
    )
      throw new Error(
        "Mission point exceeds limits: altitude1..height, hold integer0..60 seconds",
      );
    return { ...point };
  });
}
export function compileMission(points) {
  const item = (
    seq,
    command,
    position,
    z,
    hold = 0,
    frame = MavFrame.GLOBAL_RELATIVE_ALT_INT,
  ) => ({
    seq,
    command,
    frame,
    current: 0,
    autocontinue: 1,
    param1: hold,
    param2: 0,
    param3: 0,
    param4: 0,
    x: position.x,
    y: position.y,
    z,
    missionType: MavMissionType.MISSION,
  });
  const home = projectWaypoint(0, 0),
    items = [
      item(
        0,
        MavCmd.NAV_WAYPOINT,
        home,
        MISSION_ORIGIN.altitudeMsl,
        0,
        MavFrame.GLOBAL_INT,
      ),
      item(1, MavCmd.NAV_TAKEOFF, home, points[0].altitude),
    ];
  points.forEach((point, i) =>
    items.push(
      item(
        i + 2,
        MavCmd.NAV_WAYPOINT,
        projectWaypoint(point.north, point.east),
        point.altitude,
        point.hold,
      ),
    ),
  );
  items.push(
    item(
      items.length,
      MavCmd.NAV_LAND,
      projectWaypoint(points.at(-1).north, points.at(-1).east),
      0,
    ),
  );
  // AP shared mission storage canonicalizes LAND yaw-direction sign to +1; Copter ignores this Plane-only field.
  items.at(-1).param4 = 1;
  return items;
}
const family = (frame) =>
  [0, 5].includes(frame)
    ? "global"
    : [3, 6].includes(frame)
      ? "relative"
      : null;
export function verifyMissionItem(actual, expected, home) {
  if (
    !actual ||
    actual.seq !== expected.seq ||
    actual.command !== expected.command ||
    actual.missionType !== 0 ||
    family(actual.frame) !== family(expected.frame) ||
    actual.autocontinue !== 1
  )
    throw new Error("Mission item header mismatch");
  for (const field of ["param1", "param2", "param3", "param4", "x", "y", "z"])
    if (!Number.isFinite(actual[field]))
      throw new Error("Mission contains nonfinite values");
  if (expected.seq === 0) {
    if (
      !home ||
      Math.abs(actual.x - home.latitude) > 1 ||
      Math.abs(actual.y - home.longitude) > 1 ||
      Math.abs(actual.z - home.altitude / 1000) > 0.011
    )
      throw new Error(
        "Downloaded mission home does not match actual HOME_POSITION",
      );
  } else if (
    actual.x !== expected.x ||
    actual.y !== expected.y ||
    Math.abs(actual.z - expected.z) > 0.011 ||
    actual.param1 !== expected.param1 ||
    actual.param2 !== 0 ||
    actual.param3 !== 0 ||
    actual.param4 !== expected.param4
  )
    throw new Error(
      `Downloaded mission item differs from upload: ${JSON.stringify({ actual, expected })}`,
    );
}
export class ArduPilotMission {
  constructor(session) {
    this.session = session;
    this.state = {
      phase: "empty",
      points: [],
      origin: { ...MISSION_ORIGIN },
      limits: {
        radius: session.config?.geofenceRadius ?? 500,
        height: session.config?.geofenceHeight ?? 100,
        minAltitude: 1,
      },
      currentSeq: null,
      reachedSeqs: [],
      error: null,
      verified: false,
    };
    this.home = null;
    this.gpsFix = 0;
    this.flightEvidence = {};
  }
  snapshot() {
    return structuredClone({
      ...this.state,
      homeReady: Boolean(this.home) && this.gpsFix >= 3,
      readyToUpload:
        this.readiness() && !this.transfer && !this.session.commandBusy,
    });
  }
  readiness() {
    const s = this.session;
    return (
      s.phase === "running" &&
      s.controller.verified &&
      s.controller.parametersVerified &&
      s.controller.armedKnown &&
      (s.heartbeatEpoch ?? 0) > (s.armingAckEpoch ?? -1) &&
      !s.controller.armed &&
      this.gpsFix >= 3 &&
      Boolean(this.home) &&
      s.sim?.snapshot().status === "landed"
    );
  }
  diagnostic(type, value) {
    const s = this.session;
    if (!s.record) return;
    s.record.missionEvents ??= [];
    if (s.record.missionEvents.length >= 10000) {
      this.state.phase = "error";
      this.state.error = "Mission diagnostic limit reached";
      this.state.verified = false;
      throw new Error(this.state.error);
    }
    const entry = { time: s.sim.snapshot().time, type, ...value };
    s.record.missionEvents.push(entry);
    s.bytes += Buffer.byteLength(JSON.stringify(entry)) + 1;
  }
  poll() {
    if (!this.session.telemetry || this.session.telemetry.destroyed) return;
    this.session.writeMavlink(commandMessage(MavCmd.REQUEST_MESSAGE, [242]));
    this.session.writeMavlink(commandMessage(MavCmd.REQUEST_MESSAGE, [24]));
    this.session.writeMavlink(commandMessage(MavCmd.REQUEST_MESSAGE, [42]));
    this.session.writeMavlink(commandMessage(MavCmd.REQUEST_MESSAGE, [245]));
  }
  interrupt(reason) {
    if (this.state.phase === "running") {
      this.state.phase = "interrupted";
      this.state.error = reason;
      try {
        this.diagnostic("interrupted", { reason });
      } catch {
        /* Diagnostic exhaustion must never prevent lifecycle cleanup. */
      }
    }
  }
  abort(reason) {
    if (this.transfer) {
      const transfer = this.transfer;
      this.transfer = null;
      clearTimeout(this.deadline);
      clearInterval(this.retry);
      this.state.phase = "error";
      this.state.error = reason;
      this.state.verified = false;
      try {
        this.diagnostic("error", { reason });
      } catch {
      } finally {
        transfer.reject(new Error(reason));
      }
    } else this.interrupt(reason);
  }
  send(message) {
    this.lastMessage = message;
    this.session.writeMavlink(message);
  }
  async upload(points) {
    if (!this.readiness() || this.transfer || this.session.commandBusy)
      throw new Error(
        "Mission upload requires a running, verified, GPS/home-ready, disarmed ground session",
      );
    this.state.verified = false;
    this.state.phase = "error";
    this.state.error = "Mission upload not verified";
    const canonical = validateMissionPoints(points, this.session.config);
    this.items = compileMission(canonical);
    Object.assign(this.state, {
      phase: "uploading",
      points: canonical,
      limits: {
        radius: this.session.config.geofenceRadius,
        height: this.session.config.geofenceHeight,
        minAltitude: 1,
      },
      currentSeq: null,
      reachedSeqs: [],
      error: null,
      home: { ...this.home },
    });
    delete this.state.completionEvidence;
    this.flightEvidence = {};
    this.diagnostic("upload", {
      points: canonical,
      items: this.items,
      home: this.home,
    });
    this.requested = new Set();
    this.nextRequest = 0;
    this.downloaded = [];
    return new Promise((resolve, reject) => {
      this.transfer = { resolve, reject };
      this.deadline = setTimeout(
        () => this.abort("Mission upload/download wall deadline exceeded"),
        15000,
      );
      this.retry = setInterval(() => {
        if (this.transfer && this.lastMessage)
          try {
            this.session.writeMavlink(this.lastMessage);
          } catch (error) {
            this.abort(error.message);
          }
      }, 1000);
      try {
        this.send(
          Object.assign(new common.MissionCount(), {
            targetSystem: 1,
            targetComponent: 1,
            count: this.items.length,
            missionType: 0,
          }),
        );
      } catch (error) {
        this.abort(error.message);
      }
    });
  }
  handle(id, message) {
    if (id === 245) {
      if (
        !Number.isInteger(message.landedState) ||
        message.landedState < 0 ||
        message.landedState > 4
      )
        throw new Error("Invalid EXTENDED_SYS_STATE landed state");
      this.state.landedState = message.landedState;
      if (
        this.state.phase === "running" &&
        this.flightEvidence.landSelected &&
        message.landedState === MavLandedState.ON_GROUND
      ) {
        this.flightEvidence.ground = {
          landedState: message.landedState,
          time: this.session.sim.snapshot().time,
        };
      }
      this.update();
      return;
    }
    if (id === 242) {
      if (
        !Number.isInteger(message.latitude) ||
        !Number.isInteger(message.longitude) ||
        !Number.isInteger(message.altitude)
      )
        throw new Error("Invalid HOME_POSITION");
      const delta = projectWaypoint(0, 0),
        distance = Math.hypot(
          (message.latitude - delta.x) * SCALE,
          (message.longitude - delta.y) *
            SCALE *
            Math.cos((MISSION_ORIGIN.latitude * Math.PI) / 180),
        );
      if (
        distance > 2 ||
        Math.abs(message.altitude / 1000 - MISSION_ORIGIN.altitudeMsl) > 1
      ) {
        this.home = null;
        this.abort("Autopilot home does not match simulator origin");
        return;
      }
      this.home = {
        latitude: message.latitude,
        longitude: message.longitude,
        altitude: message.altitude,
      };
      return;
    }
    if (id === 24) {
      this.gpsFix = message.fixType;
      return;
    }
    if (this.transfer && [39, 40, 44, 47, 51, 73].includes(id)) {
      try {
        if (
          message.missionType !== 0 ||
          message.targetSystem !== 255 ||
          ![0, 190].includes(message.targetComponent)
        )
          throw new Error("Unexpected mission transfer target/type");
        if (id === 39)
          throw new Error("Float MISSION_ITEM download is not accepted");
        if (this.state.phase === "uploading") {
          if (id === 40 || id === 51) {
            const seq = message.seq;
            if (
              !Number.isInteger(seq) ||
              seq < 0 ||
              seq >= this.items.length ||
              seq > this.nextRequest ||
              seq < this.nextRequest - 1
            )
              throw new Error("Unexpected mission request sequence");
            this.diagnostic("uploadRequest", { messageId: id, seq });
            this.requested.add(seq);
            this.nextRequest = Math.max(this.nextRequest, seq + 1);
            this.send(
              Object.assign(new common.MissionItemInt(), {
                ...this.items[seq],
                targetSystem: 1,
                targetComponent: 1,
              }),
            );
          } else if (id === 47) {
            this.diagnostic("uploadAck", { result: message.type });
            if (message.type !== 0 || this.requested.size !== this.items.length)
              throw new Error("Mission upload ACK failed or incomplete");
            this.state.phase = "verifying";
            this.send(
              Object.assign(new common.MissionRequestList(), {
                targetSystem: 1,
                targetComponent: 1,
                missionType: 0,
              }),
            );
          }
        } else if (this.state.phase === "verifying") {
          if (id === 44) {
            if (message.count !== this.items.length)
              throw new Error("Downloaded mission count differs");
            if (!this.downloaded.length) this.requestItem(0);
          } else if (id === 73) {
            if (
              Number.isInteger(message.seq) &&
              message.seq >= 0 &&
              message.seq < this.downloaded.length
            ) {
              const prior = this.downloaded[message.seq];
              if (
                family(message.frame) !== family(prior.frame) ||
                [
                  "command",
                  "autocontinue",
                  "x",
                  "y",
                  "z",
                  "param1",
                  "param2",
                  "param3",
                  "param4",
                ].some((key) => message[key] !== prior[key])
              )
                throw new Error("Changed duplicate mission item");
              this.requestItem(this.downloaded.length);
              return;
            }
            if (message.seq !== this.downloaded.length)
              throw new Error("Unexpected downloaded mission sequence");
            this.diagnostic("downloadItem", {
              item: { ...message },
              expected: this.items[message.seq],
            });
            verifyMissionItem(message, this.items[message.seq], this.home);
            this.downloaded.push({ ...message });
            if (this.downloaded.length === this.items.length) {
              this.session.writeMavlink(
                Object.assign(new common.MissionAck(), {
                  targetSystem: 1,
                  targetComponent: 1,
                  type: 0,
                  missionType: 0,
                }),
              );
              this.diagnostic("verified", { items: this.downloaded });
              const transfer = this.transfer;
              this.transfer = null;
              clearTimeout(this.deadline);
              clearInterval(this.retry);
              this.state.phase = "ready";
              this.state.verified = true;
              transfer.resolve(this.snapshot());
            } else this.requestItem(this.downloaded.length);
          }
        }
      } catch (error) {
        this.abort(error.message);
      }
      return;
    }
    if (
      this.state.verified &&
      ["ready", "running", "completed"].includes(this.state.phase)
    ) {
      if (id === 42) {
        if (
          !Number.isInteger(message.seq) ||
          message.seq < 0 ||
          message.seq >= this.items.length
        )
          return;
        const changed =
          this.state.currentSeq !== message.seq ||
          this.state.missionState !== message.missionState;
        if (
          this.state.phase === "running" &&
          message.seq === this.items.length - 1 &&
          this.session.controller.flightMode === "AUTO" &&
          !this.flightEvidence.landSelected
        ) {
          this.flightEvidence.landSelected = {
            seq: message.seq,
            time: this.session.sim.snapshot().time,
          };
          this.diagnostic("landSelected", this.flightEvidence.landSelected);
        }
        this.state.currentSeq = message.seq;
        this.state.missionState = message.missionState;
        if (changed && this.state.phase === "running")
          this.diagnostic("current", {
            seq: message.seq,
            missionState: message.missionState,
          });
      }
      if (
        id === 46 &&
        (this.state.phase === "running" || this.startingFlight)
      ) {
        if (
          !Number.isInteger(message.seq) ||
          message.seq < 1 ||
          message.seq >= this.items.length
        )
          return;
        if (!this.state.reachedSeqs.includes(message.seq)) {
          this.state.reachedSeqs.push(message.seq);
          this.diagnostic("reached", { seq: message.seq });
        }
      }
    }
  }
  requestItem(seq) {
    this.send(
      Object.assign(new common.MissionRequestInt(), {
        targetSystem: 1,
        targetComponent: 1,
        seq,
        missionType: 0,
      }),
    );
  }
  async start() {
    if (
      !this.readiness() ||
      this.state.phase !== "ready" ||
      !this.state.verified
    )
      throw new Error(
        "A read-back verified mission and disarmed ground readiness are required",
      );
    this.startingFlight = true;
    this.flightEvidence = {};
    delete this.state.completionEvidence;
    this.state.reachedSeqs = [];
    try {
      await this.session.sendCommand(MavCmd.DO_SET_MODE, [1, 4]);
      await this.session.sendCommand(MavCmd.COMPONENT_ARM_DISARM, [1, 0]);
      await this.session.sendCommand(MavCmd.DO_SET_MISSION_CURRENT, [1, 1]);
      await this.session.sendCommand(MavCmd.MISSION_START, [0, 0]);
      this.state.phase = "running";
      const deadline = Date.now() + 3000;
      while (this.session.controller.flightMode !== "AUTO") {
        if (Date.now() > deadline || this.session.phase !== "running")
          throw new Error("AUTO mode not confirmed by heartbeat");
        await new Promise((resolve) => setTimeout(resolve, 25));
      }
      this.startingFlight = false;
      this.diagnostic("start", {});
    } catch (error) {
      this.startingFlight = false;
      this.state.phase = "error";
      this.state.verified = false;
      this.state.error = error.message;
      this.diagnostic("error", { reason: error.message });
      throw error;
    }
  }
  update() {
    if (this.state.phase !== "running") return;
    const last = this.items.length - 1,
      all = this.items
        .slice(2, -1)
        .every((item) => this.state.reachedSeqs.includes(item.seq));
    const snapshot = this.session.sim.snapshot(),
      evidence = this.flightEvidence;
    if (this.session.controller.armed && evidence.armedAt === undefined)
      evidence.armedAt = snapshot.time;
    if (
      this.session.controller.armed &&
      snapshot.position[2] < -0.5 &&
      evidence.airborneAt === undefined
    )
      evidence.airborneAt = snapshot.time;
    if (
      !this.session.controller.armed &&
      evidence.airborneAt !== undefined &&
      evidence.disarmedAt === undefined
    )
      evidence.disarmedAt = snapshot.time;
    if (
      all &&
      evidence.armedAt !== undefined &&
      evidence.airborneAt !== undefined &&
      evidence.landSelected &&
      evidence.ground &&
      evidence.disarmedAt !== undefined &&
      !this.session.controller.armed &&
      snapshot.status === "landed" &&
      Math.hypot(...snapshot.velocity) < 0.25
    ) {
      this.state.phase = "completed";
      this.state.completionEvidence = {
        policy:
          "all-user-reached+final-land-selected+auto-flight+source-ground+disarmed",
        ...structuredClone(evidence),
      };
      this.diagnostic("completed", { evidence: this.state.completionEvidence });
    }
  }
}
