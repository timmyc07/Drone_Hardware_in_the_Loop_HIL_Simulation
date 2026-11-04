import test from "node:test";
import assert from "node:assert/strict";
import { createSimulation, validateConfig } from "../sim.js";
import { validateRecording, recordingToCsv } from "../recording.js";

function record() {
  const sim = createSimulation();
  const first = sim.snapshot();
  for (let i = 0; i < 10; i++) sim.step();
  return {
    format: "flight-lab",
    version: 1,
    createdAt: new Date().toISOString(),
    config: validateConfig(),
    scenario: "hover",
    dt: 0.005,
    events: [{ time: 0, type: "targets", value: { altitude: 2 } }],
    samples: [first, sim.snapshot()],
  };
}
test("round trips and emits numeric CSV with units", () => {
  const input = record();
  const parsed = JSON.parse(JSON.stringify(input));
  assert.deepEqual(validateRecording(parsed), parsed);
  const rows = recordingToCsv(input)
    .split("\n")
    .map((row) => row.split(","));
  assert.match(rows[0][1], /north_m/);
  assert.match(rows[0].join(","), /accel_x_m_s2/);
  assert.equal(rows.length, 3);
  assert.equal(rows[0].length, rows[1].length);
  assert.ok(
    rows
      .slice(1)
      .flat()
      .every((value) => value !== "" && Number.isFinite(Number(value))),
  );
});
test("rejects malformed data and strips extra fields", () => {
  const input = record();
  input.samples[0].timing = { skippedWallSeconds: 1 };
  assert.equal(validateRecording(input).samples[0].timing, undefined);
  const bad = record();
  bad.samples[0].quaternion = [0, 0, 0, 0];
  assert.throws(() => validateRecording(bad));
  const order = record();
  order.samples.reverse();
  assert.throws(() => validateRecording(order));
  const config = record();
  config.config.extra = 1;
  assert.throws(() => validateRecording(config));
  const proto = record();
  Object.setPrototypeOf(proto.samples[0], { status: "ready" });
  assert.throws(() => validateRecording(proto));
});
