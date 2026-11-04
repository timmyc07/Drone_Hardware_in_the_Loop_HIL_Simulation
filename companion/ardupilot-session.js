import { EventEmitter } from "node:events";
import dgram from "node:dgram";
import net from "node:net";
import { spawn } from "node:child_process";
import { mkdtemp, writeFile, realpath, stat, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import path from "node:path";
import { createHash } from "node:crypto";
import { createReadStream } from "node:fs";
import { common, minimal } from "node-mavlink";
import { packetReader } from "./px4-protocol.js";
import { createSimulation, validateConfig } from "../web/sim.js";
import {
  ARDUPILOT_SOURCE_SHA,
  ARDUPILOT_DT as DT,
  ARDUPILOT_OUTPUT_MAP,
  decodeServoPacket,
  mapServoSpeeds,
  physicsJson,
  mavlinkEncoder,
  commandMessage,
  autopilotIdentity,
} from "./ardupilot-protocol.js";
import { ArduPilotMission } from "./ardupilot-mission.js";
const MAX_BYTES = 96 * 1024 * 1024;
export class ArduPilotSession extends EventEmitter {
  constructor() {
    super();
    this.phase = "idle";
    this.logs = [];
    this.sequence = 0;
    this.bytes = 0;
    this.pendingAcks = new Map();
    this.telemetryEpoch = 0;
    this.heartbeatEpoch = -1;
    this.armingAckEpoch = -1;
    this.missionManager = new ArduPilotMission(this);
    this.controller = {
      kind: "ardupilot",
      version: null,
      reportedGitHash: null,
      sourceSha: null,
      expectedLocalPatchSha256:
        "892be52e36b8e6bdbc7dd6f4208a99f250b1aae116abd2e8080b9b3b068c0e42",
      expectedSourceSha: ARDUPILOT_SOURCE_SHA,
      verified: false,
      verification:
        "MAVLink version and 8-character git-prefix self-report; not full-SHA or cryptographic attestation",
      armed: false,
      armedKnown: false,
      parametersVerified: false,
      flightMode: null,
      estimated: null,
    };
    this.stats = {
      sensorPackets: 0,
      actuatorPackets: 0,
      crcErrors: null,
      mavlinkCrcErrors: 0,
      invalidActuatorPackets: 0,
      frameMismatches: 0,
      duplicateFrames: 0,
      staleFrames: 0,
      lastActuatorAgeMs: null,
    };
  }
  state() {
    return {
      phase: this.phase,
      clock: { stepUs: 1000 },
      snapshot: this.sim?.snapshot() ?? null,
      mission: this.missionManager.snapshot(),
      controller: { ...this.controller },
      stats: {
        ...this.stats,
        lastActuatorAgeMs: this.lastActuator
          ? Date.now() - this.lastActuator
          : null,
      },
      logs: [...this.logs],
    };
  }
  log(line) {
    this.logs.push(String(line).slice(0, 1000));
    this.logs = this.logs.slice(-40);
  }
  publish() {
    this.emit("state", this.state());
  }
  fail(error) {
    if (["fault", "stopped"].includes(this.phase)) return;
    this.missionManager.abort(String(error.message ?? error));
    this.phase = "fault";
    this.log(error.message ?? error);
    clearTimeout(this.nextTick);
    for (const pending of this.pendingAcks.values())
      pending.reject(new Error(String(error.message ?? error)));
    this.pendingAcks.clear();
    this.publish();
  }
  async start({ binaryPath, config = {} } = {}) {
    if (this.phase !== "idle")
      throw new Error("Create a new session to start a new ArduPilot clock");
    if (config.airframe && config.airframe !== "quadrotor")
      throw new Error("ArduPilot external plant supports quadrotor only");
    const validated = validateConfig({
      ...config,
      airframe: "quadrotor",
      startOnGround: true,
    });
    const binary = await realpath(binaryPath);
    if (
      path.basename(binary) !== "arducopter" ||
      !(await stat(binary)).isFile()
    )
      throw new Error("Expected native arducopter binary");
    const hash = createHash("sha256");
    for await (const chunk of createReadStream(binary)) hash.update(chunk);
    this.controller.binarySha256 = hash.digest("hex");
    const sourceRoot = path.resolve(path.dirname(binary), "../../.."),
      defaults = path.join(
        sourceRoot,
        "Tools/autotest/default_params/copter.parm",
      );
    await stat(defaults);
    this.config = validated;
    this.missionManager.state.limits = {
      radius: validated.geofenceRadius,
      height: validated.geofenceHeight,
      minAltitude: 1,
    };
    this.sim = createSimulation(validated, { controlSource: "external" });
    this.send = mavlinkEncoder();
    this.directory = await mkdtemp(
      path.join(tmpdir(), "flight-lab-ardupilot-"),
    );
    try {
      const params = {
        FRAME_CLASS: 1,
        FRAME_TYPE: 1,
        RC_PROTOCOLS: 524288,
        ARMING_SKIPCHK: 0,
        AHRS_EKF_TYPE: 3,
        SCHED_LOOP_RATE: 400,
        SIM_RATE_HZ: 1000,
        MOT_PWM_MIN: 1000,
        MOT_PWM_MAX: 2000,
        MOT_THST_EXPO: 1,
        MOT_SPIN_ARM: 0,
        MOT_SPIN_MIN: 0,
        MOT_SPIN_MAX: 1,
        MOT_BAT_VOLT_MIN: 0,
        MOT_BAT_VOLT_MAX: 0,
        MOT_THST_HOVER:
          (validated.mass * 9.80665) / (4 * validated.maxMotorThrust),
        MOT_HOVER_LEARN: 0,
        SERIAL0_PROTOCOL: 2,
        SERIAL1_PROTOCOL: 0,
        SERIAL2_PROTOCOL: 0,
        SERIAL3_PROTOCOL: 5,
        SERIAL4_PROTOCOL: 5,
        SERIAL5_PROTOCOL: 0,
        SERIAL6_PROTOCOL: 0,
        SERIAL7_PROTOCOL: 0,
        SERIAL8_PROTOCOL: 0,
        SERVO1_FUNCTION: 33,
        SERVO2_FUNCTION: 34,
        SERVO3_FUNCTION: 35,
        SERVO4_FUNCTION: 36,
      };
      const paramPath = path.join(this.directory, "flight-lab.parm");
      await writeFile(
        paramPath,
        Object.entries(params)
          .map(([key, value]) => `${key} ${value}`)
          .join("\n") + "\n",
      );
      this.record = {
        format: "flight-lab-ardupilot",
        version: 1,
        createdAt: new Date().toISOString(),
        firmware: { ...this.controller },
        config: validated,
        origin: { latitude: 25.033, longitude: 121.5654, altitudeMsl: 20 },
        dt: DT,
        clock: { stepUs: 1000 },
        bootstrapIncludedInActuators: true,
        outputMap: ARDUPILOT_OUTPUT_MAP,
        parameters: params,
        assumptions: {
          motorCommand:
            "normalized rotor speed=(PWM-1000)/1000; MOT_THST_EXPO=1, spin offsets0, spin max1, voltage compensation off",
          sensors:
            "JSON supplies exact plant FRD specific force/body rates and NED pose; ArduPilot owns its simulated sensors and estimator",
          rc: "explicit synthetic centered RC with throttle low; not hardware input",
          identity: this.controller.verification,
          transport:
            "JSON UDP and MAVLink TCP intended loopback; socket exposure requires native validation",
        },
        actuators: [],
        samples: [this.sim.snapshot()],
        events: [],
        commands: [],
        bootstrapTicks: 0,
      };
      this.phase = "starting";
      this.udp = dgram.createSocket("udp4");
      this.udp.on("error", (error) => this.fail(error));
      this.udp.on("message", (data, remote) => {
        try {
          this.receiveServo(data, remote);
        } catch (error) {
          this.stats.invalidActuatorPackets++;
          this.fail(error);
        }
      });
      await new Promise((resolve, reject) => {
        this.udp.once("error", reject);
        this.udp.bind(9002, "127.0.0.1", resolve);
      });
      this.server = net.createServer((socket) => this.connectTelemetry(socket));
      await new Promise((resolve, reject) => {
        this.server.once("error", reject);
        this.server.listen(5760, "127.0.0.1", resolve);
      });
      const args = [
        "--model",
        "JSON:127.0.0.1",
        "--rate",
        "1000",
        "--speedup",
        "1",
        "--home",
        "25.033,121.5654,20,0",
        "--defaults",
        `${defaults},${paramPath}`,
        "--serial0",
        "tcpclient:127.0.0.1:5760",
        "--rc-in-port",
        "0",
        "--irlock-port",
        "0",
      ];
      for (let i = 1; i < 9; i++)
        if (i !== 3 && i !== 4) args.push(`--serial${i}`, "none");
      this.expectedParameters = Object.fromEntries(
        Object.entries(params).filter(([key]) =>
          [
            "FRAME_CLASS",
            "FRAME_TYPE",
            "RC_PROTOCOLS",
            "ARMING_SKIPCHK",
            "AHRS_EKF_TYPE",
            "SCHED_LOOP_RATE",
            "SIM_RATE_HZ",
            "MOT_PWM_MIN",
            "MOT_PWM_MAX",
            "MOT_THST_EXPO",
            "MOT_SPIN_ARM",
            "MOT_SPIN_MIN",
            "MOT_SPIN_MAX",
            "MOT_BAT_VOLT_MIN",
            "MOT_BAT_VOLT_MAX",
            "SERVO1_FUNCTION",
            "SERVO2_FUNCTION",
            "SERVO3_FUNCTION",
            "SERVO4_FUNCTION",
          ].includes(key),
        ),
      );
      this.confirmedParameters = {};
      this.child = spawn(binary, args, {
        cwd: this.directory,
        env: process.env,
        stdio: ["ignore", "pipe", "pipe"],
      });
      this.exitPromise = new Promise((resolve) =>
        this.child.once("close", resolve),
      );
      this.child.on("error", (error) => this.fail(error));
      this.child.on("exit", (code, signal) => {
        if (this.phase !== "stopped")
          this.fail(new Error(`ArduCopter exited ${code ?? signal}`));
      });
      for (const output of [this.child.stdout, this.child.stderr]) {
        let pending = "";
        output.on("data", (chunk) => {
          pending += String(chunk);
          const lines = pending.split(/\r?\n/);
          pending = lines.pop();
          for (const line of lines) if (line) this.log(line);
        });
      }

      await new Promise((resolve, reject) => {
        this.child.once("spawn", resolve);
        this.child.once("error", reject);
      });
      this.startedAt = Date.now();
      this.publisher = setInterval(() => this.publish(), 50);
      this.heartbeatTimer = setInterval(() => {
        if (this.telemetry && !this.telemetry.destroyed) {
          this.writeMavlink(
            Object.assign(new minimal.Heartbeat(), {
              type: 6,
              autopilot: 8,
              baseMode: 0,
              customMode: 0,
              systemStatus: 4,
              mavlinkVersion: 3,
            }),
          );
          this.missionManager.poll();
          if (!this.controller.verified)
            this.writeMavlink(commandMessage(512, [148]));
          if (!this.controller.parametersVerified)
            for (const key of Object.keys(this.expectedParameters))
              if (!Object.hasOwn(this.confirmedParameters, key))
                this.writeMavlink(
                  Object.assign(new common.ParamRequestRead(), {
                    targetSystem: 1,
                    targetComponent: 1,
                    paramId: key,
                    paramIndex: -1,
                  }),
                );
        }
      }, 1000);
      this.watchdog = setInterval(() => {
        if (
          ["starting", "running"].includes(this.phase) &&
          Date.now() - (this.lastActuator ?? this.startedAt) > 5000
        )
          this.fail(
            new Error(
              "ArduPilot actuator watchdog expired; plant clock frozen",
            ),
          );
      }, 250);
      this.publish();
      return this.state();
    } catch (error) {
      this.fail(error);
      await this.close();
      this.phase = "fault";
      this.publish();
      throw error;
    }
  }
  connectTelemetry(socket) {
    if (this.telemetry || socket.remoteAddress !== "127.0.0.1") {
      socket.destroy();
      return;
    }
    this.telemetry = socket;
    socket.setNoDelay(true);
    this.streams = packetReader(
      socket,
      (header, message) => {
        if (header.sysid !== 1 || header.compid !== 1) return;
        try {
          this.onTelemetry(header.msgid, message);
        } catch (error) {
          this.fail(error);
        }
      },
      () => this.stats.mavlinkCrcErrors++,
    );
    for (const stream of this.streams)
      stream.on("error", (error) => this.fail(error));
    socket.on("error", (error) => this.fail(error));
    socket.on("close", () => {
      if (this.phase !== "stopped")
        this.fail(new Error("ArduPilot MAVLink disconnected"));
    });
  }
  writeMavlink(message) {
    if (!this.telemetry || this.telemetry.destroyed)
      throw new Error("ArduPilot MAVLink is not connected");
    this.telemetry.write(this.send(message));
  }
  onTelemetry(id, message) {
    this.telemetryEpoch++;
    this.missionManager.handle(id, message);
    if (id === 0) {
      this.controller.armed = Boolean(message.baseMode & 128);
      this.controller.armedKnown = true;
      this.controller.flightMode =
        { 0: "STABILIZE", 3: "AUTO", 4: "GUIDED", 9: "LAND" }[
          message.customMode
        ] ?? String(message.customMode);
      this.lastHeartbeat = Date.now();
      this.heartbeatEpoch = this.telemetryEpoch;
      if (
        !this.missionManager.startingFlight &&
        this.controller.flightMode !== "AUTO"
      )
        this.missionManager.interrupt(
          `Autopilot mode changed to ${this.controller.flightMode}`,
        );
      this.missionManager.update();
    } else if (id === 148) {
      const identity = autopilotIdentity(message);
      Object.assign(this.controller, identity);
      if (!identity.verified)
        throw new Error("Unsupported ArduCopter version or git-prefix");
      this.record.firmware = { ...this.controller };
    } else if (id === 22) {
      const key = message.paramId.replace(/\0+$/, "");
      if (Object.hasOwn(this.expectedParameters, key)) {
        if (
          !Number.isFinite(message.paramValue) ||
          Math.abs(message.paramValue - this.expectedParameters[key]) > 1e-5
        )
          throw new Error(`ArduPilot parameter mismatch ${key}`);
        this.confirmedParameters[key] = message.paramValue;
        this.controller.parametersVerified =
          Object.keys(this.confirmedParameters).length ===
          Object.keys(this.expectedParameters).length;
        this.record.confirmedParameters = { ...this.confirmedParameters };
      }
    } else if (id === 77) {
      const pending = this.pendingAcks.get(message.command);
      if (!pending) return;
      const previousSize = JSON.stringify(pending.log).length;
      pending.log.ackResult = message.result;
      if (
        message.command === common.MavCmd.COMPONENT_ARM_DISARM &&
        message.result === 0
      )
        this.armingAckEpoch = this.telemetryEpoch;
      this.bytes += JSON.stringify(pending.log).length - previousSize;
      if (message.result === 5) return;
      this.pendingAcks.delete(message.command);
      if (message.result === 0) pending.resolve();
      else
        pending.reject(
          new Error(
            `ArduPilot command ${message.command} rejected (${message.result})`,
          ),
        );
    } else if (id === 253) this.log(`MAVLink: ${message.text}`);
  }
  receiveServo(data, remote) {
    if (
      !["starting", "running", "paused"].includes(this.phase) ||
      remote.address !== "127.0.0.1"
    )
      return;
    if (
      this.peer &&
      (this.peer.port !== remote.port || this.peer.address !== remote.address)
    )
      throw new Error("Unexpected JSON actuator peer");
    const packet = decodeServoPacket(data);
    if (
      packet.frameRate !== 1000 &&
      !(
        (this.lastFrame === undefined || this.lastFrame === 0) &&
        packet.frameCount === 0 &&
        packet.frameRate === 1200
      )
    )
      throw new Error("Unexpected JSON frame rate after bootstrap");
    this.peer = { address: remote.address, port: remote.port };
    if (this.lastFrame !== undefined) {
      if (packet.frameCount === this.lastFrame) {
        if (
          packet.frameRate !== this.lastPacket.frameRate ||
          packet.pwm.some((value, i) => value !== this.lastPacket.pwm[i])
        )
          throw new Error("Inconsistent retransmitted actuator frame");
        this.stats.duplicateFrames++;
        if (this.phase !== "paused" && this.cachedReply)
          this.sendReply(this.cachedReply);
        return;
      }
      if (packet.frameCount !== (this.lastFrame + 1) >>> 0) {
        this.stats.frameMismatches++;
        throw new Error(
          "ArduPilot frame gap, reorder or reset; start a new session",
        );
      }
    } else if (packet.frameCount !== 0) {
      this.stats.frameMismatches++;
      throw new Error("ArduPilot initial frame must be zero");
    }
    if (this.pendingFrame) {
      if (this.pendingFrame.frameCount !== packet.frameCount)
        throw new Error("Multiple unconsumed JSON actuator frames");
      this.stats.duplicateFrames++;
      return;
    }
    this.pendingFrame = packet;
    if (this.phase !== "paused") this.consumeFrame();
  }
  consumeFrame() {
    const packet = this.pendingFrame;
    if (!packet) return;
    this.pendingFrame = null;
    const armedKnown = this.controller.armedKnown,
      armed = armedKnown && this.controller.armed;
    const rotorSpeedCommands = mapServoSpeeds(packet, armed),
      timeUs = this.sim.readSensors().timeUs;
    const accepted = {
      sequence: ++this.sequence,
      timeUs,
      armed,
      rotorSpeedCommands,
    };
    this.sim.applyExternalActuators(accepted);
    const snapshot = this.sim.stepExternal(DT);
    const entry = {
      ...accepted,
      frameCount: packet.frameCount,
      frameRate: packet.frameRate,
      pwm: [...packet.pwm],
      armedKnown,
      heartbeatArmed: this.controller.armed,
      heartbeatAgeMs: this.lastHeartbeat
        ? Date.now() - this.lastHeartbeat
        : null,
    };
    this.lastFrame = packet.frameCount;
    this.lastPacket = packet;
    this.lastActuator = Date.now();
    this.stats.actuatorPackets++;
    this.missionManager.update();
    this.cachedReply = physicsJson(this.sim.readSensors(), snapshot.battery);
    this.record.actuators.push(entry);
    this.bytes += Buffer.byteLength(JSON.stringify(entry)) + 1;
    if (!armedKnown) this.record.bootstrapTicks++;
    if (this.sequence % 50 === 0) {
      this.record.samples.push(snapshot);
      this.bytes += Buffer.byteLength(JSON.stringify(snapshot)) + 1;
    }
    if (this.bytes >= MAX_BYTES - 10000 || snapshot.time >= 1800) {
      this.phase = "paused";
      this.record.limitReached = true;
      this.log("Recording limit reached; plant clock paused");
      this.publish();
      return;
    }
    this.phase = "running";
    this.nextTick = setTimeout(() => {
      if (this.phase === "running") this.sendReply(this.cachedReply);
    }, DT * 1000);
  }
  sendReply(buffer) {
    if (!this.peer) return;
    this.udp.send(buffer, this.peer.port, this.peer.address, (error) => {
      if (error) this.fail(error);
    });
    this.stats.sensorPackets++;
  }
  async sendCommand(command, params = []) {
    if (this.pendingAcks.has(command))
      throw new Error("Command already pending");
    if (this.record.commands.length >= 100000)
      throw new Error("Command diagnostic limit reached");
    const log = {
      time: this.sim.snapshot().time,
      command,
      params: [...params],
      ackResult: null,
    };
    this.record.commands.push(log);
    this.bytes += Buffer.byteLength(JSON.stringify(log)) + 1;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pendingAcks.delete(command);
        reject(
          new Error(`ArduPilot command ${command} acknowledgement timeout`),
        );
      }, 5000);
      this.pendingAcks.set(command, {
        log,
        resolve: () => {
          clearTimeout(timer);
          resolve();
        },
        reject: (error) => {
          clearTimeout(timer);
          const previousSize = JSON.stringify(log).length;
          log.error = error.message;
          this.bytes += JSON.stringify(log).length - previousSize;
          reject(error);
        },
      });
      try {
        this.writeMavlink(commandMessage(command, params));
      } catch (error) {
        clearTimeout(timer);
        this.pendingAcks.delete(command);
        reject(error);
      }
    });
  }
  async uploadMission(points) {
    return this.missionManager.upload(points);
  }
  async command(action) {
    if (
      ![
        "arm",
        "takeoff",
        "land",
        "disarm",
        "pause",
        "resume",
        "stop",
        "mission",
      ].includes(action)
    )
      throw new Error("Unknown ArduPilot action");
    if (action === "stop") {
      await this.close();
      return { sent: true };
    }
    if (this.record?.events.length >= 100000)
      throw new Error("Session event limit reached");
    if (this.missionManager.transfer && !["stop", "pause"].includes(action))
      throw new Error("Mission transfer is in progress");
    if (action === "pause") {
      if (this.missionManager.transfer)
        this.missionManager.abort("Mission transfer interrupted by pause");
      if (this.phase !== "running") throw new Error("Session is not running");
      this.phase = "paused";
      clearTimeout(this.nextTick);
    } else if (action === "resume") {
      if (this.phase !== "paused" || this.record.limitReached)
        throw new Error("Session cannot resume");
      this.phase = "running";
      this.lastActuator = Date.now();
      if (this.pendingFrame) this.consumeFrame();
      else if (this.cachedReply) this.sendReply(this.cachedReply);
    } else {
      if (
        this.phase !== "running" ||
        !this.controller.verified ||
        !this.controller.parametersVerified ||
        !this.controller.armedKnown
      )
        throw new Error(
          "ArduPilot identity and heartbeat must be verified before flight",
        );
      if (this.commandBusy)
        throw new Error("A flight command is already pending");
      this.commandBusy = true;
      if (["arm", "land", "disarm", "takeoff"].includes(action))
        this.missionManager.interrupt(`User requested ${action}`);
      try {
        if (action === "mission") await this.missionManager.start();
        if (action === "arm" || action === "takeoff") {
          await this.sendCommand(176, [1, 4]);
          if (!this.controller.armed) await this.sendCommand(400, [1, 0]);
        }
        if (action === "takeoff")
          await this.sendCommand(22, [0, 0, 0, 0, 0, 0, 2.5]);
        else if (action === "land") await this.sendCommand(21);
        else if (action === "disarm") await this.sendCommand(400, [0, 0]);
      } finally {
        this.commandBusy = false;
      }
    }
    const event = { time: this.sim.snapshot().time, action };
    this.record.events.push(event);
    this.bytes += Buffer.byteLength(JSON.stringify(event)) + 1;
    this.publish();
    return {
      sent: true,
      confirmation:
        "MAVLink command ACK accepted; flight completion is observed asynchronously",
    };
  }
  exportRecording() {
    if (!this.record) throw new Error("No ArduPilot recording");
    const finalSnapshot = this.sim.snapshot(),
      samples = [...this.record.samples];
    if (samples.at(-1).time === finalSnapshot.time)
      samples[samples.length - 1] = finalSnapshot;
    else samples.push(finalSnapshot);
    return structuredClone({
      ...this.record,
      firmware: { ...this.controller },
      samples,
      stats: this.stats,
      phase: this.phase,
      finalSnapshot,
    });
  }
  async close() {
    if (this.closePromise) return this.closePromise;
    this.closePromise = (async () => {
      this.missionManager.abort("Session stopped");
      this.phase = "stopped";
      for (const timer of [this.publisher, this.heartbeatTimer, this.watchdog])
        clearInterval(timer);
      clearTimeout(this.nextTick);
      for (const pending of this.pendingAcks.values())
        pending.reject(new Error("Session stopped"));
      this.pendingAcks.clear();
      this.telemetry?.destroy();
      for (const stream of this.streams ?? []) stream.destroy();
      const serverClosed = this.server?.listening
        ? new Promise((resolve) => this.server.close(resolve))
        : Promise.resolve();
      if (
        this.child &&
        this.child.exitCode === null &&
        this.child.signalCode === null
      ) {
        this.child.kill("SIGTERM");
        const killer = setTimeout(() => this.child.kill("SIGKILL"), 1500);
        await this.exitPromise;
        clearTimeout(killer);
      }
      await serverClosed;
      if (this.udp) {
        await new Promise((resolve) => {
          try {
            this.udp.close(resolve);
          } catch {
            resolve();
          }
        });
      }
      if (this.directory)
        await rm(this.directory, { recursive: true, force: true });
      this.publish();
    })();
    return this.closePromise;
  }
}
