// Opt-in real pinned PX4 mission proof; requires the reviewed loopback-native build.
import assert from "node:assert/strict";
import { mkdtemp, mkdir, writeFile, cp } from "node:fs/promises";
import { execFileSync } from "node:child_process";
import { tmpdir } from "node:os";
import path from "node:path";
import { Px4Session } from "./px4-session.js";
const binaryPath = process.argv[2];
if (!binaryPath) throw Error("Pass pinned native PX4 binary");
const output =
  process.argv[3] ??
  (await mkdtemp(path.join(tmpdir(), "flight-lab-px4-mission-proof-")));
await mkdir(output, { recursive: true });
const session = new Px4Session(),
  logs = [],
  started = Date.now();
let stage = "warming",
  uploading = false;
const truth = [];
const originalLog = session.log.bind(session);
session.log = (line) => {
  logs.push(line);
  originalLog(line);
};
console.log("ARTIFACT", output);
try {
  await session.start({ binaryPath });
  console.log("OWNED_PID", session.child.pid);
  await new Promise((resolve, reject) => {
    const timer = setInterval(async () => {
      try {
        const state = session.state(),
          p = state.snapshot;
        if (state.phase === "fault") throw Error(state.logs.join("\n"));
        if (Date.now() - started > 240000)
          throw Error("Native mission proof wall deadline exceeded");
        if (!p) return;
        if (
          stage === "warming" &&
          state.mission.readyToUpload &&
          !uploading &&
          p.time > 8
        ) {
          uploading = true;
          stage = "upload";
          console.log("READY", p.time);
          await writeFile(
            path.join(output, "sockets.txt"),
            execFileSync("lsof", [
              "-Pan",
              "-p",
              String(session.child.pid),
              "-i",
            ]),
          );
          const points = [
            { north: 8, east: 0, altitude: 3, hold: 0.5 },
            { north: 8, east: 8, altitude: 3, hold: 1 },
            { north: 0, east: 8, altitude: 3, hold: 1 },
            { north: 0, east: 0, altitude: 3, hold: 1 },
          ];
          await session.uploadMission(points);
          console.log("ROUNDTRIP", JSON.stringify(session.state().mission));
          await session.command("mission");
          stage = "flight";
          console.log("MISSION_STARTED");
        }
        if (stage === "flight")
          truth.push({
            time: p.time,
            position: p.position,
            current: state.mission.currentSeq,
          });
        if (state.mission.phase === "completed") {
          clearInterval(timer);
          resolve();
        }
        if (["error", "interrupted"].includes(state.mission.phase))
          throw Error(JSON.stringify(state.mission));
      } catch (error) {
        clearInterval(timer);
        reject(error);
      }
    }, 50);
  });
  const record = session.exportRecording(),
    state = session.state();
  assert.equal(state.mission.phase, "completed");
  assert.deepEqual(
    state.mission.reachedSeqs.filter((x) => x >= 1 && x <= 4).sort(),
    [1, 2, 3, 4],
  );
  const reached = record.missionEvents.filter(
    (e) => e.type === "reached" && e.seq >= 1 && e.seq <= 4,
  );
  for (const e of reached) {
    const p = state.mission.points[e.seq - 1];
    assert.ok(
      Math.hypot(e.truth[0] - p.north, e.truth[1] - p.east) < 1.5,
      `Waypoint${e.seq} truth capture`,
    );
  }
  await writeFile(path.join(output, "recording.json"), JSON.stringify(record));
  const summary = {
    passed: true,
    wallSeconds: (Date.now() - started) / 1000,
    time: state.snapshot.time,
    mission: state.mission,
    controller: state.controller,
    stats: state.stats,
    actuators: record.actuators.length,
    samples: record.samples.length,
  };
  await writeFile(
    path.join(output, "summary.json"),
    JSON.stringify(summary, null, 2),
  );
  console.log(JSON.stringify(summary));
  await session.uploadMission(state.mission.points);
  assert.equal(session.state().mission.phase, "ready");
  assert.equal(session.state().mission.currentSeq, 0);
  await writeFile(
    path.join(output, "repeat-upload.json"),
    JSON.stringify(session.state().mission, null, 2),
  );
  await writeFile(
    path.join(output, "after-repeat-recording.json"),
    JSON.stringify(session.exportRecording()),
  );
  console.log("SAME_ROUTE_REUPLOAD_READY", session.state().mission.currentSeq);
  await session.command("mission");
  await new Promise((resolve, reject) => {
    const timer = setInterval(() => {
      try {
        const x = session.state();
        if (
          x.phase === "fault" ||
          ["error", "interrupted"].includes(x.mission.phase)
        )
          throw new Error(JSON.stringify(x));
        if (Date.now() - started > 240000)
          throw new Error("Repeat mission deadline");
        if (x.mission.phase === "completed") {
          clearInterval(timer);
          resolve();
        }
      } catch (error) {
        clearInterval(timer);
        reject(error);
      }
    }, 50);
  });
  const repeatRecord = session.exportRecording(),
    repeatState = session.state();
  await writeFile(
    path.join(output, "repeat-flight-recording.json"),
    JSON.stringify(repeatRecord),
  );
  await writeFile(
    path.join(output, "repeat-flight-summary.json"),
    JSON.stringify(repeatState, null, 2),
  );
  console.log(
    "SAME_ROUTE_SECOND_FLIGHT_COMPLETED",
    session.state().snapshot.time,
  );
} catch (error) {
  await writeFile(
    path.join(output, "partial-recording.json"),
    JSON.stringify(session.exportRecording()),
  );
  console.error(error);
  process.exitCode = 1;
} finally {
  await writeFile(path.join(output, "native.log"), logs.join("\n"));
  if (session.directory)
    await cp(path.join(session.directory, "log"), path.join(output, "log"), {
      recursive: true,
    }).catch(() => {});
  await session.close();
  console.log("CLOSED", output);
}
