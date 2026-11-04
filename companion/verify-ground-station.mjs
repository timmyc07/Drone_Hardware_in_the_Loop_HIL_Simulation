// Opt-in native proof using the stock MAVProxy CLI, not a synthetic packet sender.
import assert from "node:assert/strict";
import { spawn, execFileSync } from "node:child_process";
import { mkdir, readFile, writeFile, cp, stat } from "node:fs/promises";
import path from "node:path";
import { Px4Session } from "./px4-session.js";
import { ArduPilotSession } from "./ardupilot-session.js";
import { compileMission as px4Mission } from "./px4-mission.js";
import { compileMission as apMission } from "./ardupilot-mission.js";
import { decodeDatagram } from "./mavlink.js";
import { replayPx4Recording, replayArduPilotRecording } from "./replay.js";

const [kind, binaryPath, mavproxyPath, directory] = process.argv.slice(2);
if (
  !["px4", "ardupilot"].includes(kind) ||
  !binaryPath ||
  !mavproxyPath ||
  !directory
)
  throw Error(
    "Usage: node companion/verify-ground-station.mjs px4|ardupilot /native/binary /mavproxy.py /artifact-directory",
  );
const output = path.resolve(directory);
if (/[\s;\r\n]/.test(output))
  throw Error(
    "MAVProxy artifact path must not contain whitespace or command separators",
  );
await mkdir(output, { recursive: true });
const points = [
  [8, 0],
  [8, 8],
  [0, 8],
  [0, 0],
].map(([north, east]) => ({ north, east, altitude: 3, hold: 1 }));
const items = kind === "px4" ? px4Mission(points) : apMission(points);
const routeFile = path.join(output, "mission.waypoints"),
  downloadedFile = path.join(output, "downloaded.waypoints");
const rows = items.map((m) => [
  m.seq,
  m.current,
  m.frame === 6 ? 3 : m.frame === 5 ? 0 : m.frame,
  m.command,
  m.param1,
  m.param2,
  m.param3,
  m.param4,
  m.x / 1e7,
  m.y / 1e7,
  m.z,
  m.autocontinue,
]);
await writeFile(
  routeFile,
  "QGC WPL 110\n" + rows.map((row) => row.join("\t")).join("\n") + "\n",
);
const session = kind === "px4" ? new Px4Session() : new ArduPilotSession();
const settings = { enabled: true, targetPort: 14650, owner: "web" };
const began = Date.now(),
  samples = [],
  gcsLog = [],
  hostCommands = [];
let gcs,
  gcsExit,
  lastProgress = 0,
  stage = "starting",
  flightStart,
  armedObserved = false;
const progress = () => {
  const s = session.state(),
    p = s.snapshot;
  if (Date.now() - lastProgress > 5000) {
    lastProgress = Date.now();
    console.log(
      JSON.stringify({
        stage,
        phase: s.phase,
        time: p?.time,
        position: p?.position,
        mode: s.controller.flightMode,
        armed: p?.armed,
        bridge: s.groundStation,
        logs: s.logs.slice(-2),
      }),
    );
  }
};
const waitFor = async (predicate, label, milliseconds = 90000) => {
  const deadline = Date.now() + milliseconds;
  while (true) {
    const s = session.state();
    if (s.phase === "fault") throw Error(s.logs.slice(-5).join("\n"));
    if (gcs && (gcs.exitCode !== null || gcs.signalCode !== null))
      throw Error("MAVProxy exited before proof completed");
    if (predicate(s)) return s;
    if (Date.now() > deadline || Date.now() - began > 480000)
      throw Error("Timed out: " + label);
    progress();
    await new Promise((resolve) => setTimeout(resolve, 100));
  }
};
const stopGcs = async () => {
  if (!gcs) return;
  if (gcs.exitCode === null && gcs.signalCode === null) gcs.kill("SIGTERM");
  const timer = setTimeout(() => gcs.kill("SIGKILL"), 2000);
  try {
    await gcsExit;
  } finally {
    clearTimeout(timer);
  }
};
const send = (command) => {
  hostCommands.push({ time: session.sim.snapshot().time, command });
  gcs.stdin.write(command + "\n");
  console.log("GCS", command);
};
const newAck = (start, command) =>
  session.record.groundStationEvents
    .slice(start)
    .filter((e) => e.type === "controlAck")
    .flatMap((e) => decodeDatagram(Buffer.from(e.rawBase64, "base64")))
    .map((p) => p.message)
    .find((m) => m.command === command && m.result !== 5);
const command = async (text, id) => {
  for (let attempt = 0; attempt < 8; attempt++) {
    const start = session.record.groundStationEvents.length;
    send(text);
    await waitFor(
      () => !!newAck(start, id),
      "GCS command acknowledgement " + id,
      10000,
    );
    const ack = newAck(start, id);
    if (ack.result === 0) return;
    if (
      kind !== "ardupilot" ||
      text !== "arm throttle" ||
      ![1, 2, 4].includes(ack.result) ||
      attempt === 7
    )
      assert.equal(
        ack.result,
        0,
        "Native GCS command " + id + " was not accepted",
      );
    assert.equal(session.state().controller.armed, false);
    assert.equal(session.sim.snapshot().armed, false);
    console.log(
      "Native arming health gate; waiting before normal retry",
      ack.result,
    );
    const retryAt = session.sim.snapshot().time + 12;
    await waitFor(
      (s) => s.snapshot.time >= retryAt,
      "native arming health warmup",
      30000,
    );
  }
};
session.on("state", (s) => {
  if (s.snapshot)
    samples.push({
      time: s.snapshot.time,
      position: s.snapshot.position,
      armed: s.snapshot.armed,
      status: s.snapshot.status,
      owner: s.groundStation.owner,
      mode: s.controller.flightMode,
    });
});
try {
  await session.start({ binaryPath });
  gcs = spawn(
    mavproxyPath,
    [
      "--master=udpin:127.0.0.1:14650",
      "--source-system=255",
      "--source-component=190",
      "--target-system=1",
      "--target-component=1",
      "--mav20",
      "--streamrate=4",
      "--default-modules=log,wp,param,arm,mode,cmdlong",
      "--logfile=" + path.join(output, "mavproxy.tlog"),
      "--state-basedir=" + output,
    ],
    {
      cwd: output,
      env: { ...process.env, PYTHONUNBUFFERED: "1" },
      stdio: ["pipe", "pipe", "pipe"],
    },
  );
  gcs.on("error", (error) => session.fail(error));
  gcsExit = new Promise((resolve) => gcs.once("close", resolve));
  for (const stream of [gcs.stdout, gcs.stderr])
    stream.on("data", (data) => {
      gcsLog.push(data.toString());
    });
  stage = "native-warmup";
  await waitFor(
    (s) => s.mission?.readyToUpload && s.snapshot.time > 8,
    stage,
    180000,
  );
  await session.configureGroundStation(settings);
  await waitFor((s) => s.groundStation.connected, "MAVProxy heartbeat", 20000);
  await waitFor(
    () => gcsLog.join("").includes("Detected vehicle 1:1 on link 0"),
    "MAVProxy native vehicle heartbeat",
    20000,
  );
  if (kind === "ardupilot")
    await waitFor(
      () => gcsLog.join("").includes("Mode AUTO"),
      "MAVProxy ArduCopter mode mapping",
      20000,
    );
  const rejected = session.state().groundStation.stats.rejectedPackets;
  send("arm throttle");
  await waitFor(
    (s) =>
      s.groundStation.stats.rejectedPackets > rejected &&
      s.groundStation.lastRejection?.messageId === 76 &&
      s.groundStation.lastRejection.reason ===
        "Web owns flight and mission control",
    "read-only command rejection",
    10000,
  );
  assert.equal(session.sim.snapshot().armed, false);
  const sourceHome = structuredClone(session.missionManager.home);
  await session.configureGroundStation({ ...settings, owner: "gcs" });
  await assert.rejects(
    () => session.command("takeoff"),
    /ground station.*owns/i,
  );
  await assert.rejects(
    async () => session.uploadMission(points),
    /ground station.*owns/i,
  );
  stage = "gcs-mission-upload";
  const uploadStart = session.record.groundStationEvents.length;
  send("wp load " + routeFile);
  await waitFor(
    () =>
      session.record.groundStationEvents
        .slice(uploadStart)
        .some(
          (e) =>
            e.type === "missionReply" &&
            e.messageId === 47 &&
            decodeDatagram(Buffer.from(e.rawBase64, "base64"))[0].message
              .type === 0,
        ),
    "external mission accepted",
    30000,
  );
  stage = "gcs-mission-download";
  send("wp save " + downloadedFile);
  await waitFor(
    () =>
      gcsLog
        .join("")
        .includes("Saved " + rows.length + " waypoints to " + downloadedFile),
    "MAVProxy downloaded mission",
    30000,
  );
  const actual = (await readFile(downloadedFile, "utf8"))
    .trim()
    .split(/\r?\n/)
    .slice(1)
    .map((line) => line.trim().split(/\s+/).map(Number));
  assert.equal(actual.length, rows.length);
  for (let i = 0; i < rows.length; i++)
    for (let c = 0; c < rows[i].length; c++) {
      // Current is live navigation state, not uploaded route content.
      if (c === 1) {
        assert.ok([0, 1].includes(actual[i][c]));
        continue;
      }
      if (c === 2) {
        const frame = actual[i][c];
        assert.equal(frame === 6 ? 3 : frame === 5 ? 0 : frame, rows[i][c]);
        continue;
      }
      if (kind === "ardupilot" && i === 0 && [8, 9, 10].includes(c)) {
        const home = [
          sourceHome.latitude / 1e7,
          sourceHome.longitude / 1e7,
          sourceHome.altitude / 1000,
        ];
        assert.ok(
          Math.abs(actual[i][c] - home[c - 8]) < (c === 10 ? 1e-4 : 1e-6),
        );
        continue;
      }
      assert.ok(
        Math.abs(actual[i][c] - rows[i][c]) <
          ([8, 9].includes(c) ? 1e-6 : 1e-4),
        "Downloaded mission differs at item/field " + i + "/" + c,
      );
    }
  stage = "gcs-mission-flight";
  if (kind === "ardupilot") {
    send("mode GUIDED");
    await waitFor((s) => s.controller.flightMode === "GUIDED", "GUIDED mode");
    await command("arm throttle", 400);
    await waitFor((s) => s.controller.armed && s.snapshot.armed, "GCS arm");
    await command("long 224 1 1", 224);
    await command("long 300 0 0", 300);
  } else {
    await command("long 176 1 4 4", 176);
    await waitFor(
      (s) => s.controller.flightMode === "AUTO_MISSION",
      "AUTO_MISSION mode",
    );
    await command("arm throttle", 400);
  }
  await waitFor(
    (s) => s.snapshot.armed && -s.snapshot.position[2] > 2,
    "external mission takeoff",
  );
  flightStart = session.sim.snapshot().time;
  armedObserved = true;
  await assert.rejects(
    () => session.configureGroundStation(settings),
    /ground|disarmed/i,
  );
  await session.command("pause");
  const paused = session.sim.snapshot().time;
  await new Promise((resolve) => setTimeout(resolve, 500));
  assert.equal(session.sim.snapshot().time, paused);
  await session.command("resume");
  await waitFor(
    (s) =>
      s.snapshot.status === "landed" &&
      !s.snapshot.armed &&
      !s.controller.armed &&
      s.snapshot.time > flightStart + 10,
    "external mission landing",
    180000,
  );
  const flown = samples.filter((s) => s.owner === "gcs" && s.armed);
  for (const point of points)
    assert.ok(
      flown.some(
        (s) =>
          Math.hypot(
            s.position[0] - point.north,
            s.position[1] - point.east,
            -s.position[2] - point.altitude,
          ) < 2,
      ),
      "External GCS flight did not visit each waypoint",
    );
  await waitFor(
    (s) =>
      !s.groundStation.mission?.active && session.groundStation.groundReady(),
    "external transaction completion",
  );
  await session.configureGroundStation(settings);
  stage = "web-owner-regression";
  await waitFor((s) => s.mission.readyToUpload, "web mission readiness");
  await session.uploadMission(points);
  assert.equal(session.state().mission.verified, true);
  await session.command("mission");
  await waitFor(
    (s) => s.mission.phase === "completed",
    "web-owner mission completion",
    180000,
  );
  stage = "gcs-disconnect-watchdog";
  await waitFor(
    () =>
      session.groundStation.groundReady() &&
      session.groundStation.internalIdle(),
    "fresh disarmed native heartbeat",
  );
  await session.configureGroundStation({ ...settings, owner: "gcs" });
  await stopGcs();
  const deadline = Date.now() + 10000;
  while (session.phase !== "fault" && Date.now() < deadline)
    await new Promise((resolve) => setTimeout(resolve, 100));
  assert.equal(session.phase, "fault");
  assert.ok(
    session.logs.some((line) => line.includes("Ground station heartbeat lost")),
  );
  assert.ok(session.state().groundStation.lastHeartbeatAgeMs > 5000);
  const frozen = session.sim.snapshot().time;
  await new Promise((resolve) => setTimeout(resolve, 250));
  assert.equal(session.sim.snapshot().time, frozen);
  assert.equal(session.state().groundStation.owner, "gcs");
  const record = session.exportRecording();
  const replay =
    kind === "px4"
      ? replayPx4Recording(record)
      : replayArduPilotRecording(record);
  await writeFile(
    path.join(output, "summary.json"),
    JSON.stringify(
      {
        kind,
        armedObserved,
        gcsMissionItems: rows.length,
        flightStart,
        final: session.state(),
        replay,
        hostCommands,
        truth: samples,
      },
      null,
      2,
    ),
  );
  console.log(
    "VERIFIED",
    JSON.stringify({ kind, replay, samples: record.samples.length }),
  );
} finally {
  try {
    if (session.record)
      await writeFile(
        path.join(output, "recording.json"),
        JSON.stringify(session.exportRecording()),
      );
    await writeFile(path.join(output, "mavproxy.log"), gcsLog.join(""));
    await writeFile(
      path.join(output, "final-state.json"),
      JSON.stringify(session.state(), null, 2),
    );
    if (session.child?.pid) {
      try {
        await writeFile(
          path.join(output, "native-sockets.txt"),
          execFileSync("lsof", ["-Pan", "-p", String(session.child.pid), "-i"]),
        );
      } catch {}
    }
    if (session.directory)
      for (const name of ["log", "logs"]) {
        const source = path.join(session.directory, name);
        try {
          await stat(source);
        } catch (error) {
          if (error.code === "ENOENT") continue;
          throw error;
        }
        await cp(source, path.join(output, "native-" + name), {
          recursive: true,
        });
      }
  } finally {
    try {
      await stopGcs();
    } finally {
      await session.close();
    }
  }
}
