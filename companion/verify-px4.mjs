import { createLocalDirectory } from "./local-directory.js";
// Opt-in real native PX4 proof, not a mock test: node companion/verify-px4.mjs /absolute/path/to/px4 [artifact-directory]
import assert from "node:assert/strict";
import { mkdir, writeFile, cp } from "node:fs/promises";
import path from "node:path";
import { Px4Session } from "./px4-session.js";
const binaryPath = process.argv[2];
if (!binaryPath)
  throw new Error("Pass the pinned native PX4 v1.17.0 binary path");
const output =
  process.argv[3] ??
  (await createLocalDirectory("flight-lab-px4-proof-"));
await mkdir(output, { recursive: true });
const session = new Px4Session();
let sent = false,
  hoverStart = null,
  landSent = false,
  gustSent = false,
  settled = false;
const hover = [];
const started = Date.now();
try {
  await session.start({ binaryPath });
  await new Promise((resolve, reject) => {
    const timer = setInterval(() => {
      try {
        const state = session.state(),
          s = state.snapshot;
        if (state.phase === "fault") throw new Error(state.logs.join("\n"));
        if (Date.now() - started > 240000)
          throw new Error("Real PX4 flight proof exceeded four wall minutes");
        if (!s) return;
        if (s.time > 8 && state.controller.verified && !sent) {
          session.command("takeoff");
          sent = true;
        }
        if (
          sent &&
          state.controller.armed &&
          s.time > 22 &&
          -s.position[2] > 2.2 &&
          hoverStart === null
        )
          hoverStart = s.time;
        if (hoverStart !== null && !landSent) {
          hover.push({
            time: s.time,
            position: s.position,
            rates: s.rates,
            motors: s.motors,
          });
          if (!gustSent && s.time - hoverStart > 20) {
            session.sim.gust();
            session.record.events.push({ time: s.time, action: "gust" });
            gustSent = true;
          }
          if (s.time - hoverStart >= 60) {
            session.command("land");
            landSent = true;
          }
        }
        if (landSent && !state.controller.armed && s.position[2] === 0) {
          settled = true;
          clearInterval(timer);
          resolve();
        }
      } catch (error) {
        clearInterval(timer);
        reject(error);
      }
    }, 50);
  });
  const record = session.exportRecording(),
    state = session.state();
  assert.ok(settled);
  assert.ok(record.actuators.some((input) => input.armed));
  assert.ok(hover.at(-1).time - hover[0].time >= 60);
  const rms = Math.sqrt(
    hover.reduce((sum, x) => sum + (x.position[2] + 2.5) ** 2, 0) /
      hover.length,
  );
  const maxHorizontal = Math.max(
    ...hover.map((x) => Math.hypot(x.position[0], x.position[1])),
  );
  assert.ok(rms < 0.5, `hover altitude RMSE ${rms}`);
  assert.ok(maxHorizontal < 3, `horizontal excursion ${maxHorizontal}`);
  assert.equal(state.stats.crcErrors, 0);
  assert.equal(state.stats.timestampMismatches, 0);
  const summary = {
    passed: true,
    firmware: state.controller,
    simSeconds: state.snapshot.time,
    wallSeconds: (Date.now() - started) / 1000,
    hoverSeconds: hover.at(-1).time - hover[0].time,
    altitudeRmseM: rms,
    maxHorizontalM: maxHorizontal,
    finalSnapshot: state.snapshot,
    stats: state.stats,
    logs: state.logs,
    recordingBytes: Buffer.byteLength(JSON.stringify(record)),
    artifacts: output,
  };
  await cp(path.join(session.directory, "log"), path.join(output, "px4-log"), {
    recursive: true,
  });
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
