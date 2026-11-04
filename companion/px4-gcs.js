import dgram from "node:dgram";
import { PassThrough } from "node:stream";
import { common, minimal } from "node-mavlink";
import {
  packetReader,
  mavlinkEncoder,
  commandMessage,
  INTERNAL_COMPONENT_ID,
} from "./mavlink.js";
const PARAMETERS = Object.freeze({ THR_MDL_FAC: 1, COM_RC_IN_MODE: 4 });
export class Px4Gcs {
  constructor(session) {
    this.session = session;
    this.encode = mavlinkEncoder();
    this.pending = new Map();
    this.confirmed = {};
  }
  async start() {
    this.socket = dgram.createSocket("udp4");
    this.input = new PassThrough();
    this.streams = packetReader(
      this.input,
      (header, message, packet) => {
        if (header.sysid !== 1 || header.compid !== 1) return;
        try {
          if (
            this.session.groundStation?.nativePacket(
              header,
              message,
              packet,
            ) === false
          )
            return;
          this.handle(header.msgid, message);
        } catch (error) {
          this.session.fail(error);
        }
      },
      () => this.session.stats.gcsCrcErrors++,
    );
    for (const stream of [this.input, ...this.streams])
      stream.on("error", (error) => this.session.fail(error));
    this.socket.on("message", (data, peer) => {
      if (peer.address === "127.0.0.1" && peer.port === 14560)
        this.input.write(data);
    });
    this.socket.on("error", (error) => this.session.fail(error));
    await new Promise((resolve, reject) => {
      this.socket.once("error", reject);
      this.socket.bind(14561, "127.0.0.1", resolve);
    });
    this.timer = setInterval(() => this.poll(), 1000);
  }
  send(message) {
    if (!this.socket || this.closed) throw Error("PX4 GCS transport closed");
    this.socket.send(this.encode(message), 14560, "127.0.0.1", (error) => {
      if (error && !this.closed) this.session.fail(error);
    });
  }
  sendRaw(buffer) {
    if (!this.socket || this.closed) throw Error("PX4 GCS transport closed");
    this.socket.send(buffer, 14560, "127.0.0.1", (error) => {
      if (error && !this.closed) this.session.fail(error);
    });
  }
  poll() {
    if (
      this.closed ||
      !["starting", "running"].includes(this.session.phase) ||
      this.session.groundStation?.externalOwner
    )
      return;
    try {
      this.send(
        Object.assign(new minimal.Heartbeat(), {
          type: 6,
          autopilot: 8,
          baseMode: 0,
          customMode: 0,
          systemStatus: 4,
          mavlinkVersion: 3,
        }),
      );
      for (const id of [242, 24, 42, 245]) this.send(commandMessage(512, [id]));
      for (const [paramId] of Object.entries(PARAMETERS))
        if (this.confirmed[paramId] === undefined)
          this.send(
            Object.assign(new common.ParamRequestRead(), {
              targetSystem: 1,
              targetComponent: 1,
              paramId,
              paramIndex: -1,
            }),
          );
    } catch (error) {
      this.session.fail(error);
    }
  }
  handle(id, m) {
    const s = this.session;
    s.health.update(id, m);
    if (id === 0) {
      if (
        m.autopilot !== 12 ||
        !Number.isInteger(m.baseMode) ||
        !Number.isInteger(m.customMode)
      )
        throw Error("Invalid PX4 heartbeat");
      s.heartbeatEpoch = (s.heartbeatEpoch ?? 0) + 1;
      s.lastHeartbeat = Date.now();
      s.controller.telemetryArmed = Boolean(m.baseMode & 128);
      s.controller.armedKnown = true;
      const main = (m.customMode >>> 16) & 255,
        sub = (m.customMode >>> 24) & 255;
      s.controller.flightMode =
        main === 4 && sub === 4 ? "AUTO_MISSION" : `PX4_${main}_${sub}`;
      if (!s.groundStation?.externalOwner) s.missionManager?.update();
    } else if (id === 22) {
      const key = String(m.paramId).replace(/\0.*$/, "");
      if (Object.hasOwn(PARAMETERS, key)) {
        if (m.paramType !== (key === "COM_RC_IN_MODE" ? 6 : 9))
          throw Error(`PX4 parameter ${key} has unexpected type`);
        // PX4 v1.17 bytewise PARAM_VALUE encoding preserves int32 bits in float storage.
        let value = m.paramValue;
        if (m.paramType === 6) {
          const bits = Buffer.alloc(4);
          bits.writeFloatLE(value);
          value = bits.readInt32LE();
        }
        if (!Number.isFinite(value) || Math.abs(value - PARAMETERS[key]) > 1e-6)
          throw Error(`PX4 parameter ${key} differs from plant contract`);
        this.confirmed[key] = value;
        s.controller.confirmedParameters = { ...this.confirmed };
        s.controller.parametersVerified = Object.keys(PARAMETERS).every(
          (k) => this.confirmed[k] !== undefined,
        );
      }
    } else if (id === 77) {
      if (
        ![0, 255].includes(m.targetSystem) ||
        ![0, INTERNAL_COMPONENT_ID].includes(m.targetComponent)
      )
        return;
      const waiter = this.pending.get(m.command);
      if (waiter) {
        if (m.result === 5) return;
        this.pending.delete(m.command);
        clearTimeout(waiter.timer);
        try {
          s.missionManager?.diagnostic("commandAck", {
            command: m.command,
            result: m.result,
          });
        } catch (error) {
          waiter.reject(error);
          return;
        }
        if (m.result === 0) {
          if ([300, 400].includes(m.command))
            s.armingAckEpoch = s.heartbeatEpoch ?? 0;
          waiter.resolve(m);
        } else
          waiter.reject(
            Error(`PX4 command ${m.command} rejected (${m.result})`),
          );
      }
    }
    // Source observations remain fresh; external ownership still excludes internal mission transactions/progress.
    if (!s.groundStation?.externalOwner || id === 242 || id === 24)
      s.missionManager?.handle(id, m);
  }
  command(command, params = []) {
    if (this.pending.has(command))
      return Promise.reject(Error("PX4 command already pending"));
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(command);
        reject(Error(`PX4 command ${command} ACK timeout`));
      }, 5000);
      this.pending.set(command, { resolve, reject, timer });
      try {
        this.send(commandMessage(command, params));
      } catch (error) {
        clearTimeout(timer);
        this.pending.delete(command);
        reject(error);
      }
    });
  }
  async close() {
    if (this.closed) return;
    this.closed = true;
    clearInterval(this.timer);
    for (const waiter of this.pending.values()) {
      clearTimeout(waiter.timer);
      waiter.reject(Error("PX4 GCS closed"));
    }
    this.pending.clear();
    for (const stream of [this.input, ...(this.streams ?? [])])
      stream?.destroy();
    if (this.socket)
      await new Promise((resolve) => {
        try {
          this.socket.close(resolve);
        } catch {
          resolve();
        }
      });
  }
}
