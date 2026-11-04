// Real, opt-in native AUTO mission: node companion/verify-ardupilot-mission.mjs /path/to/arducopter [artifact-directory]
import assert from "node:assert/strict";
import { mkdtemp, mkdir, writeFile, cp } from "node:fs/promises";
import { tmpdir } from "node:os";
import path from "node:path";
import { ArduPilotSession } from "./ardupilot-session.js";
const binaryPath = process.argv[2];
if (!binaryPath) throw new Error("Pass native arducopter path");
const output =
  process.argv[3] ??
  (await mkdtemp(path.join(tmpdir(), "flight-lab-auto-mission-proof-")));
await mkdir(output, { recursive: true });
const points = [
  [8, 0],
  [8, 8],
  [0, 8],
  [0, 0],
].map(([north, east]) => ({ north, east, altitude: 3, hold: 1 }));
const session = new ArduPilotSession();
let uploaded = false,
  uploadBusy = false,
  startBusy = false,
  startedMission = false,
  lastAttempt = -100,
  lastProgress = -10;
const reachedTruth = new Map();
const began = Date.now();
try {
  await session.start({ binaryPath });
  await new Promise((resolve, reject) => {
    const timer = setInterval(async () => {
      try {
        const state = session.state(),
          s = state.snapshot,
          m = state.mission;
        if (state.phase === "fault") throw new Error(state.logs.join("\n"));
        if (Date.now() - began > 300000)
          throw new Error(
            "Native AUTO mission proof exceeded five wall minutes",
          );
        if (s.time - lastProgress > 10) {
          lastProgress = s.time;
          console.log(
            "PROGRESS",
            s.time.toFixed(1),
            m.phase,
            m.currentSeq,
            m.reachedSeqs,
            s.position.map((v) => v.toFixed(2)),
            state.logs.slice(-2),
          );
        }
        if (m.readyToUpload && !uploaded && !uploadBusy) {
          uploadBusy = true;
          await session.uploadMission(points);
          uploaded = true;
          console.log("UPLOAD+DOWNLOAD VERIFIED", session.sim.snapshot().time);
        }
        if (
          uploaded &&
          !startedMission &&
          !startBusy &&
          s.time - lastAttempt > 12
        ) {
          startBusy = true;
          lastAttempt = s.time;
          try {
            await session.command("mission");
            startedMission = true;
            console.log("AUTO CONFIRMED", session.sim.snapshot().time);
          } catch (error) {
            console.log(
              "MISSION START REJECTED",
              error.message,
              state.logs.slice(-3),
            );
            if (session.state().mission.phase === "error") {
              uploaded = false;
              uploadBusy = false;
            }
          } finally {
            startBusy = false;
          }
        }
        for (const seq of m.reachedSeqs)
          if (seq >= 2 && seq <= points.length + 1 && !reachedTruth.has(seq))
            reachedTruth.set(seq, { time: s.time, position: s.position });
        if (m.phase === "interrupted") throw new Error(m.error);
        if (m.phase === "completed") {
          clearInterval(timer);
          resolve();
        }
      } catch (error) {
        clearInterval(timer);
        reject(error);
      }
    }, 50);
  });
  const state = session.state(),
    record = session.exportRecording();
  assert.equal(state.controller.armed, false);
  assert.equal(state.snapshot.position[2], 0);
  assert.equal(state.controller.flightMode, "AUTO");
  for (let index = 0; index < points.length; index++) {
    const actual = reachedTruth.get(index + 2),
      target = points[index];
    assert.ok(actual, `Waypoint ${index + 2} lacks actual reached event`);
    assert.ok(
      Math.hypot(
        actual.position[0] - target.north,
        actual.position[1] - target.east,
      ) < 1.5,
      `Waypoint ${index + 2} truth outside1.5m acceptance`,
    );
    assert.ok(Math.abs(actual.position[2] + target.altitude) < 0.5);
  }
  assert.equal(state.stats.frameMismatches, 0);
  assert.equal(state.stats.invalidActuatorPackets, 0);
  const holdEvidence = points.map((target, index) => {
    const event = record.missionEvents.find(
      (entry) => entry.type === "reached" && entry.seq === index + 2,
    );
    assert.ok(event, "Missing source reached event");
    const window = record.samples.filter(
      (sample) => sample.time >= event.time - 1.05 && sample.time <= event.time,
    );
    assert.ok(window.length >= 19, "Insufficient truth samples for hold");
    const tail = window.slice(-19);
    const maxDistance = Math.max(
      ...tail.map((sample) =>
        Math.hypot(
          sample.position[0] - target.north,
          sample.position[1] - target.east,
        ),
      ),
    );
    assert.ok(maxDistance < 0.5, "Physical hold left half-meter window");
    return {
      seq: index + 2,
      sourceReachedTime: event.time,
      truthSampleCount: tail.length,
      truthWindowSeconds: tail.at(-1).time - tail[0].time,
      maxHorizontalErrorM: maxDistance,
    };
  });
  const summary = {
    passed: true,
    mission: state.mission,
    firmware: state.controller,
    confirmedParameters: session.confirmedParameters,
    simSeconds: state.snapshot.time,
    wallSeconds: (Date.now() - began) / 1000,
    reachedTruth: [...reachedTruth],
    holdEvidence,
    truthAcceptanceM: 1.5,
    altitudeAcceptanceM: 0.5,
    stats: state.stats,
    finalSnapshot: state.snapshot,
    artifacts: output,
  };
  try {
    await cp(
      path.join(session.directory, "logs"),
      path.join(output, "native-logs"),
      { recursive: true },
    );
  } catch {}
  await writeFile(path.join(output, "recording.json"), JSON.stringify(record));
  await writeFile(
    path.join(output, "summary.json"),
    JSON.stringify(summary, null, 2),
  );
  console.log(JSON.stringify(summary, null, 2));
} catch (error) {
  if (session.record)
    await writeFile(
      path.join(output, "partial-recording.json"),
      JSON.stringify(session.exportRecording()),
    );
  if (session.directory)
    try {
      await cp(
        path.join(session.directory, "logs"),
        path.join(output, "native-logs"),
        { recursive: true },
      );
    } catch {}
  await writeFile(
    path.join(output, "failure.json"),
    JSON.stringify({ error: error.message, state: session.state() }, null, 2),
  );
  throw error;
} finally {
  await session.close();
}
