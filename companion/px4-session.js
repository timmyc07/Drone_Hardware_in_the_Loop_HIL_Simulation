import { EventEmitter } from "node:events";
import net from "node:net";
import { spawn } from "node:child_process";
import {
  mkdtemp,
  mkdir,
  writeFile,
  stat,
  realpath,
  rm,
} from "node:fs/promises";
import { tmpdir } from "node:os";
import path from "node:path";
import { createSimulation, validateConfig } from "../web/sim.js";
import {
  packetReader,
  mapActuators,
  sensorMessages,
  encoder,
  DEFAULT_ORIGIN,
  OUTPUT_MAP,
} from "./px4-protocol.js";
const DT = 0.004,
  MAX_BYTES = 96 * 1024 * 1024,
  SHA = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4";
export class Px4Session extends EventEmitter {
  constructor() {
    super();
    this.phase = "idle";
    this.logs = [];
    this.stats = {
      sensorPackets: 0,
      actuatorPackets: 0,
      crcErrors: 0,
      timestampMismatches: 0,
      lastTimestampDeltaUs: 0,
      lastActuatorAgeMs: null,
    };
    this.controller = {
      kind: "px4",
      version: null,
      sourceSha: null,
      expectedVersion: "1.17.0",
      expectedSourceSha: SHA,
      verified: false,
      sysAutostart: 10016,
      armed: false,
      estimated: null,
    };
    this.sequence = 0;
    this.bytes = 0;
  }
  state() {
    return {
      phase: this.phase,
      clock: { stepUs: Math.round(DT * 1e6) },
      snapshot: this.sim?.snapshot() ?? null,
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
    const hash = line.match(/(?:FW|PX4) git-hash:\s*(\S+)/i)?.[1];
    if (hash) {
      this.controller.sourceSha = hash;
      if (hash !== SHA)
        this.fail(
          new Error("Unsupported PX4 source SHA; expected pinned v1.17.0"),
        );
      else {
        this.controller.verified = true;
        this.controller.version = "1.17.0";
        this.record.firmware = { ...this.controller };
      }
    }
    this.logs.push(
      line
        .replace(/\x1b\[[0-9;]*[A-Za-z]/g, "")
        .split("\r")
        .at(-1)
        .slice(0, 1000),
    );
    this.logs = this.logs.slice(-40);
  }
  publish() {
    this.emit("state", this.state());
  }
  fail(error) {
    if (["stopped", "fault"].includes(this.phase)) return;
    this.phase = "fault";
    this.log(String(error.message ?? error));
    this.publish();
  }
  async start({ binaryPath, config = {}, origin = DEFAULT_ORIGIN } = {}) {
    if (this.phase !== "idle")
      throw new Error("Create a new session to start a new PX4 clock");
    const validated = validateConfig({
      ...config,
      airframe: "quadrotor",
      startOnGround: true,
    });
    if (config.airframe && config.airframe !== "quadrotor")
      throw new Error("PX4 session supports quadrotor only");
    if (
      !origin ||
      Object.keys(origin).sort().join(",") !==
        "altitudeMsl,latitude,longitude" ||
      !Number.isFinite(origin.latitude) ||
      Math.abs(origin.latitude) > 85 ||
      !Number.isFinite(origin.longitude) ||
      Math.abs(origin.longitude) > 180 ||
      !Number.isFinite(origin.altitudeMsl) ||
      origin.altitudeMsl < -500 ||
      origin.altitudeMsl > 9000
    )
      throw new Error("Invalid simulator origin");
    const binary = await realpath(binaryPath);
    if (path.basename(binary) !== "px4" || !(await stat(binary)).isFile())
      throw new Error("Expected PX4 executable");
    const rootfs = path.dirname(path.dirname(binary));
    await stat(path.join(rootfs, "etc/init.d-posix/rcS"));
    this.config = validated;
    this.origin = { ...origin };
    this.sim = createSimulation(validated, { controlSource: "external" });
    this.record = {
      format: "flight-lab-px4",
      version: 1,
      createdAt: new Date().toISOString(),
      firmware: { ...this.controller },
      config: validated,
      origin: this.origin,
      dt: DT,
      clock: { stepUs: 4000, sensorBootOffsetUs: 1000000 },
      outputMap: OUTPUT_MAP,
      assumptions: {
        gps: "ideal synthetic 3D fix, 10 satellites, eph=1m epv=1.5m",
        magnetometer: "PX4 v1.17.0 WMM-2020 lookup field at declared origin",
        motorCommand: "normalized speed; THR_MDL_FAC=1",
        manualControl: "COM_RC_IN_MODE=4 (no RC, console autonomous commands)",
        sensorNoise: {
          algorithm:
            "xorshift32(timeUs+1+seed+axis*2654435761), uniform variance normalized",
          seed: validated.seed,
          accelSigmaMps2: 0.01,
          gyroSigmaRadS: 0.001,
          magSigmaGauss: 0.0001,
          baroSigmaPa: 2.7,
          configNoiseUsed: false,
          purpose:
            "minimal sensor variation floor avoids constant-value stuck detection; regenerate from exact tick including bootstrap",
        },
        transport: "loopback TCP; console-only commands",
      },
      actuators: [],
      samples: [this.sim.snapshot()],
      events: [],
    };
    this.directory = await mkdtemp(path.join(tmpdir(), "flight-lab-px4-"));
    const wrappers = path.join(this.directory, "wrappers");
    await mkdir(wrappers);
    await writeFile(
      path.join(wrappers, "px4-rc.mavlink"),
      "# GCS UDP deliberately disabled; stop optional loopback DDS client.\nuxrce_dds_client stop\n",
    );
    this.phase = "starting";
    this.send = encoder();
    this.server = net.createServer((socket) => this.connect(socket));
    await new Promise((resolve, reject) => {
      this.server.once("error", (error) => {
        this.fail(error);
        reject(error);
      });
      this.server.listen(4560, "127.0.0.1", resolve);
    });
    const environment = Object.fromEntries(
      Object.entries(process.env).filter(([key]) => !key.startsWith("PX4_")),
    );
    this.child = spawn(
      binary,
      ["-w", this.directory, path.join(rootfs, "etc")],
      {
        cwd: this.directory,
        env: {
          ...environment,
          PATH: `${wrappers}:${process.env.PATH}`,
          PX4_SIM_MODEL: "none_iris",
          PX4_SIM_HOSTNAME: "127.0.0.1",
          PX4_SYS_AUTOSTART: "10016",
          PX4_PARAM_THR_MDL_FAC: "1",
          PX4_PARAM_COM_RC_IN_MODE: "4",
        },
        stdio: ["pipe", "pipe", "pipe"],
      },
    );
    for (const output of [this.child.stdout, this.child.stderr]) {
      let pending = "";
      output.on("data", (chunk) => {
        pending += String(chunk);
        const lines = pending.split(/\r?\n/);
        pending = lines.pop();
        for (const line of lines) if (line) this.log(line);
      });
      output.on("end", () => {
        if (pending) this.log(pending);
      });
    }
    this.child.stdin.on("error", (error) => {
      if (this.phase !== "stopped") this.fail(error);
    });
    this.exitPromise = new Promise((resolve) =>
      this.child.once("close", resolve),
    );
    this.child.on("error", (error) => this.fail(error));
    this.child.on("exit", (code, signal) => {
      if (this.phase !== "stopped")
        this.fail(new Error(`PX4 exited ${code ?? signal}`));
    });
    this.watchdog = setInterval(() => {
      if (
        ["starting", "running"].includes(this.phase) &&
        Date.now() - (this.lastActuator ?? this.startedAt) > 5000
      )
        this.fail(
          new Error("PX4 actuator watchdog expired; plant clock paused"),
        );
    }, 250);
    this.startedAt = Date.now();
    this.publisher = setInterval(() => this.publish(), 50);
    this.publish();
    return this.state();
  }
  connect(socket) {
    if (this.socket || socket.remoteAddress !== "127.0.0.1") {
      socket.destroy();
      return;
    }
    this.socket = socket;
    socket.setNoDelay(true);
    this.streams = packetReader(
      socket,
      (header, message) => {
        if (header.msgid !== 93 || header.sysid !== 1 || header.compid !== 1)
          return;
        try {
          this.onActuator(message);
        } catch (error) {
          this.fail(error);
        }
      },
      () => this.stats.crcErrors++,
    );
    for (const stream of this.streams)
      stream.on("error", (error) => this.fail(error));
    socket.on("error", (error) => this.fail(error));
    socket.on("close", () => {
      if (this.phase !== "stopped")
        this.fail(new Error("PX4 simulator disconnected"));
    });
    this.bootstrap = setInterval(() => {
      if (!this.lastActuator && this.phase === "starting") {
        this.sim.applyExternalActuators({
          sequence: ++this.sequence,
          timeUs: this.sim.readSensors().timeUs,
          armed: false,
          rotorSpeedCommands: [0, 0, 0, 0],
        });
        this.sim.stepExternal(DT);
        this.record.bootstrapTicks = (this.record.bootstrapTicks ?? 0) + 1;
        this.sendSensors();
      }
    }, 4);
    this.sendSensors();
  }
  sendSensors() {
    if (!this.socket || this.socket.destroyed) return;
    const truth = this.sim.readSensors(),
      messages = sensorMessages(truth, this.origin, this.config.seed);
    this.lastSentUs = truth.timeUs + 1_000_000;
    this.socket.write(this.send(messages.sensor));
    this.stats.sensorPackets++;
    if (truth.timeUs % 200000 === 0) this.socket.write(this.send(messages.gps));
  }
  onActuator(message) {
    if (!["starting", "running", "paused"].includes(this.phase)) return;
    const input = mapActuators(message);
    if (this.lastRawTime !== undefined && input.rawTimeUs <= this.lastRawTime)
      return;
    if (this.phase === "paused") {
      if (input.rawTimeUs !== this.lastSentUs)
        throw new Error(
          "Paused actuator timestamp does not match current sensor tick",
        );
      this.pausedActuator = message;
      return;
    }
    this.stats.lastTimestampDeltaUs = input.rawTimeUs - this.lastSentUs;
    if (this.stats.lastTimestampDeltaUs !== 0) {
      this.stats.timestampMismatches++;
      throw new Error(
        "PX4 actuator timestamp does not match the current sensor tick",
      );
    }
    this.lastRawTime = input.rawTimeUs;
    this.lastActuator = Date.now();
    this.stats.actuatorPackets++;
    if (this.bootstrap) {
      clearInterval(this.bootstrap);
      this.bootstrap = null;
      this.child.stdin.write("ver all\n");
    }
    const timeUs = this.sim.readSensors().timeUs;
    if (this.nextSampleUs === undefined)
      this.nextSampleUs = (Math.floor(timeUs / 50000) + 1) * 50000;
    const accepted = {
      sequence: ++this.sequence,
      timeUs,
      armed: input.armed,
      rotorSpeedCommands: input.rotorSpeedCommands,
    };
    this.sim.applyExternalActuators(accepted);
    const snapshot = this.sim.stepExternal(DT);
    this.controller.armed = input.armed;
    this.phase = "running";
    const entry = { ...accepted, ...input };
    const size = Buffer.byteLength(JSON.stringify(entry)) + 1;
    let sampleBytes = 0;
    if (this.sim.readSensors().timeUs >= (this.nextSampleUs ?? 50000))
      sampleBytes = Buffer.byteLength(JSON.stringify(snapshot)) + 1;
    this.bytes += size + sampleBytes;
    this.record.actuators.push(entry);
    if (sampleBytes) {
      this.record.samples.push(snapshot);
      this.nextSampleUs = (this.nextSampleUs ?? 50000) + 50000;
    }
    if (this.bytes >= MAX_BYTES - 10000 || snapshot.time >= 1800) {
      this.phase = "paused";
      this.record.limitReached = true;
      this.log("Recording limit reached; plant paused");
      this.publish();
      return;
    }
    // Yield to socket I/O; only the matching actuator advances the fixed plant clock.
    this.nextTick = setTimeout(() => {
      if (this.phase === "running") this.sendSensors();
    }, 4);
  }
  command(action) {
    if (
      !["arm", "takeoff", "land", "disarm", "pause", "resume", "stop"].includes(
        action,
      )
    )
      throw new Error("Unknown PX4 session action");
    if (action !== "stop" && this.record?.events.length >= 100000)
      throw new Error("PX4 session event limit reached");
    if (action === "stop") return this.close().then(() => ({ sent: true }));
    if (action === "pause") {
      if (this.phase !== "running") throw new Error("Session is not running");
      this.phase = "paused";
      clearTimeout(this.nextTick);
    } else if (action === "resume") {
      if (this.record.limitReached)
        throw new Error("Recording limit requires a new session");
      if (this.phase !== "paused") throw new Error("Session is not paused");
      this.phase = "running";
      this.lastActuator = Date.now();
      if (this.pausedActuator) {
        const message = this.pausedActuator;
        this.pausedActuator = null;
        this.onActuator(message);
      } else if (this.lastSentUs !== this.sim.readSensors().timeUs + 1_000_000)
        this.sendSensors();
    } else {
      if (!this.controller.verified)
        throw new Error("PX4 firmware identity is not verified");
      if (this.phase !== "running")
        throw new Error("PX4 session is not running");
      this.child.stdin.write(`commander ${action}\n`);
    }
    const event = { time: this.sim.snapshot().time, action };
    this.record.events.push(event);
    this.bytes += Buffer.byteLength(JSON.stringify(event)) + 1;
    this.publish();
    return {
      sent: true,
      confirmation:
        "Observe PX4 armed bit and physical flight; console write is not an acknowledgement",
    };
  }
  exportRecording() {
    if (!this.record) throw new Error("No PX4 session recording");
    const finalSnapshot = this.sim.snapshot(),
      samples = [...this.record.samples];
    if (samples.at(-1)?.time === finalSnapshot.time)
      samples[samples.length - 1] = finalSnapshot;
    else samples.push(finalSnapshot);
    return structuredClone({
      ...this.record,
      samples,
      stats: this.stats,
      phase: this.phase,
      finalSnapshot,
    });
  }
  async close() {
    if (this.closePromise) return this.closePromise;
    this.closePromise = (async () => {
      this.phase = "stopped";
      for (const timer of [this.watchdog, this.publisher, this.bootstrap])
        clearInterval(timer);
      clearTimeout(this.nextTick);
      this.socket?.destroy();
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
        const killer = setTimeout(() => this.child.kill("SIGKILL"), 1000);
        await this.exitPromise;
        clearTimeout(killer);
      }
      await serverClosed;
      if (this.directory)
        await rm(this.directory, { recursive: true, force: true });
      this.publish();
    })();
    return this.closePromise;
  }
}
