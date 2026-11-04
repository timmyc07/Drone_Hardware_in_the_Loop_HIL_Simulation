// Opt-in real ArduCopter JSON-physics proof; requires the pinned locally patched native build.
import assert from "node:assert/strict";
import { mkdir, mkdtemp, writeFile, cp } from "node:fs/promises";
import { tmpdir } from "node:os";
import path from "node:path";
import { execFileSync } from "node:child_process";
import { ArduPilotSession } from "./ardupilot-session.js";
const binaryPath = process.argv[2];
if (!binaryPath) throw new Error("Pass native arducopter path");
const output =
  process.argv[3] ??
  (await mkdtemp(path.join(tmpdir(), "flight-lab-ardupilot-proof-")));
await mkdir(output, { recursive: true });
const session = new ArduPilotSession();
const hover = [];
let lastProgress = -10;
let hoverStart = null,
  takeoffAccepted = false,
  lastAttempt = -100,
  attempting = false,
  landSent = false,
  gustSent = false,
  sockets = "";
const started = Date.now();
try {
  await session.start({ binaryPath });
  await new Promise((resolve, reject) => {
    const timer = setInterval(async () => {
      try {
        const state = session.state(),
          s = state.snapshot;
        if (state.phase === "fault") throw new Error(state.logs.join("\n"));
        if (Date.now() - started > 300000)
          throw new Error("Native proof exceeded five wall minutes");
        if (s.time - lastProgress >= 10) {
          lastProgress = s.time;
          console.log(
            "PROGRESS",
            s.time.toFixed(1),
            state.phase,
            state.controller.armed,
            s.position.map((v) => v.toFixed(2)),
            state.logs.slice(-2),
          );
        }
        if (
          s.time > 18 &&
          state.controller.verified &&
          state.controller.parametersVerified &&
          !takeoffAccepted &&
          !attempting &&
          s.time - lastAttempt > 12
        ) {
          attempting = true;
          lastAttempt = s.time;
          try {
            await session.command("takeoff");
            takeoffAccepted = true;
            console.log("TAKEOFF ACK", s.time);
          } catch (error) {
            console.log(
              "Health-gated takeoff retry:",
              error.message,
              session.state().logs.slice(-4),
            );
          } finally {
            attempting = false;
          }
        }
        if (
          takeoffAccepted &&
          state.controller.armed &&
          s.time > lastAttempt + 15 &&
          -s.position[2] > 2.2 &&
          hoverStart === null
        ) {
          hoverStart = s.time;
          sockets = execFileSync(
            "lsof",
            ["-nP", "-a", "-p", String(session.child.pid), "-i"],
            { encoding: "utf8" },
          );
          console.log("HOVER START", hoverStart);
        }
        if (hoverStart !== null && !landSent) {
          hover.push({ time: s.time, position: s.position, motors: s.motors });
          if (!gustSent && s.time - hoverStart > 20) {
            session.sim.gust();
            session.record.events.push({ time: s.time, action: "gust" });
            gustSent = true;
          }
          if (s.time - hoverStart >= 60) {
            landSent = true;
            await session.command("land");
            console.log("LAND ACK", s.time);
          }
        }
        if (landSent && !state.controller.armed && s.position[2] === 0) {
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
  assert.ok(hover.at(-1).time - hover[0].time >= 60);
  assert.ok(record.actuators.some((input) => input.armed));
  const altitudeRmseM = Math.sqrt(
      hover.reduce((sum, s) => sum + (s.position[2] + 2.5) ** 2, 0) /
        hover.length,
    ),
    maxHorizontalM = Math.max(
      ...hover.map((s) => Math.hypot(s.position[0], s.position[1])),
    );
  assert.ok(altitudeRmseM < 0.5, `height RMSE ${altitudeRmseM}`);
  assert.ok(maxHorizontalM < 3, `horizontal excursion ${maxHorizontalM}`);
  assert.equal(state.stats.invalidActuatorPackets, 0);
  assert.equal(state.stats.frameMismatches, 0);
  assert.ok(
    !sockets.split("\n").some((line) => /UDP \*:\d|TCP \*:\d/.test(line)),
    "Native process exposes a wildcard port",
  );
  const summary = {
    passed: true,
    firmware: state.controller,
    parameters: session.confirmedParameters,
    clock: state.clock,
    simSeconds: state.snapshot.time,
    wallSeconds: (Date.now() - started) / 1000,
    hoverSeconds: hover.at(-1).time - hover[0].time,
    altitudeRmseM,
    maxHorizontalM,
    stats: state.stats,
    sockets,
    finalSnapshot: state.snapshot,
    logs: state.logs,
    recordingBytes: Buffer.byteLength(JSON.stringify(record)),
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
  await writeFile(
    path.join(output, "failure.json"),
    JSON.stringify({ error: error.message, state: session.state() }, null, 2),
  );
  throw error;
} finally {
  await session.close();
}
