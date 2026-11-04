import test from "node:test";
import assert from "node:assert/strict";
import { createSimulation, validateConfig } from "../sim.js";
import { createTrainer } from "../training.js";
import {
  RECORDING_VERSION,
  validateSetup,
  validateRecording,
  recordingToCsv,
  recordingToJson,
  compactSnapshot,
} from "../recording.js";

function record() {
  const sim = createSimulation(),
    trainer = createTrainer();
  const capture = () => {
    const snapshot = sim.snapshot();
    return { ...snapshot, training: trainer.update(snapshot) };
  };
  const first = capture();
  for (let i = 0; i < 10; i++) sim.step();
  return {
    format: "flight-lab",
    version: RECORDING_VERSION,
    modelVersion: "0.2.0",
    createdAt: "2026-11-04T00:00:00.000Z",
    config: validateConfig(),
    scenario: "hover",
    dt: 0.005,
    setup: validateSetup(undefined),
    runtime: {
      speed: 1,
      wallSeconds: 0.05,
      p95LateMs: 0,
      lateTicks: 0,
      skippedWallSeconds: 0,
      renderEnabled: true,
    },
    events: [{ time: 0, type: "targets", value: { altitude: 2 } }],
    samples: [first, capture()],
  };
}
test("v2 round trip validates every new field; CSV preserves45columns and all body cells are numeric", () => {
  const input = record(),
    result = validateRecording(JSON.parse(JSON.stringify(input)));
  assert.deepEqual(result, JSON.parse(JSON.stringify(input)));
  const rows = recordingToCsv(input)
    .split("\n")
    .map((row) => row.split(","));
  assert.equal(rows.length, 3);
  assert.equal(rows[0].length, rows[1].length);
  assert.ok(rows[0].length > 65);
  assert.equal(rows[0][44], "steps_count");
  for (const name of [
    "north_m",
    "accel_x_m_s2",
    "battery_voltage_V",
    "battery_used_Wh",
    "training_hold_s",
  ])
    assert.ok(rows[0].includes(name));
  assert.ok(
    rows
      .slice(1)
      .flat()
      .every((value) => value !== "" && Number.isFinite(Number(value))),
  );
  const json = recordingToJson(input);
  assert.equal(json.includes('"waypoints"'), false);
  assert.deepEqual(
    validateRecording(JSON.parse(json)),
    JSON.parse(JSON.stringify(result)),
  );
});
test("compact snapshots preserve shared mission routes through mission history without per-frame copies", () => {
  const input = record();
  const route = Array.from({ length: 50 }, (_, i) => ({
    north: i,
    east: 0,
    altitude: 2,
    hold: 0,
  }));
  input.setup.mission = route;
  input.samples = input.samples.map((s) =>
    compactSnapshot({
      ...s,
      navigation: { ...s.navigation, waypoints: route },
    }),
  );
  const firstRoute = validateRecording(input).samples;
  assert.equal(
    firstRoute[0].navigation.waypoints,
    firstRoute[1].navigation.waypoints,
  );
  const later = [{ north: 1, east: 1, altitude: 2, hold: 1 }];
  input.events.push({
    time: input.samples[1].time,
    type: "mission",
    value: later,
  });
  input.samples[1].navigation.waypointCount = 1;
  const result = validateRecording(input);
  assert.equal(result.samples[0].navigation.waypoints.length, 50);
  assert.deepEqual(result.samples[1].navigation.waypoints, later);
  const json = recordingToJson(input);
  assert.equal(json.includes('"waypoints"'), false);
  assert.equal((json.match(/"hold"/g) || []).length, 51);
  assert.deepEqual(
    validateRecording(JSON.parse(json)),
    JSON.parse(JSON.stringify(result)),
  );
});
test("rejects legacy headers, malformed snapshots, unsafe navigation/faults and training", () => {
  const mutations = [
    (r) => (r.version = 1),
    (r) => (r.modelVersion = "fake"),
    (r) => delete r.setup,
    (r) => (r.samples[0].quaternion = [0, 0, 0, 0]),
    (r) => delete r.samples[0].battery,
    (r) => delete r.samples[0].training,
    (r) => (r.samples[0].armed = 1),
    (r) => (r.samples[0].navigation.mode = "javascript:bad"),
    (r) => (r.samples[0].navigation.index = 2),
    (r) => (r.samples[0].navigation.home = [1, 0]),
    (r) => (r.samples[0].navigation.waypoints = [{ north: Infinity }]),
    (r) => (r.samples[0].faults.motorEfficiency[0] = 1.2),
    (r) => (r.samples[0].faults.linkLost = "yes"),
    (r) => (r.samples[0].training.passed = 1),
    (r) => (r.samples[0].training.course = "bad"),
    (r) => (r.samples[0].training.nextGate = {}),
    (r) => (r.samples[0].battery.soc = NaN),
    (r) => (r.samples[0].metrics.altitudeRmse = undefined),
    (r) => Object.setPrototypeOf(r.samples[0], { bad: true }),
  ];
  for (const mutate of mutations) {
    const input = record();
    mutate(input);
    assert.throws(() => validateRecording(input));
  }
});
test("rejects unordered/future/sparse events and validates every new event type", () => {
  let input = record();
  input.samples.reverse();
  assert.throws(() => validateRecording(input), /order/);
  input = record();
  input.events = [{ time: 1, type: "gust" }];
  assert.throws(() => validateRecording(input), /last sample/);
  input = record();
  input.events = [
    { time: 0.04, type: "gust" },
    { time: 0.03, type: "gust" },
  ];
  assert.throws(() => validateRecording(input), /order/);
  input = record();
  input.events = Array(1);
  assert.throws(() => validateRecording(input));
  input = record();
  input.events = ["gust", "clearFaults", "takeoff", "arm", "disarm"].map(
    (type) => ({ time: 0, type }),
  );
  for (const [type, value] of [
    ["mode", "position"],
    ["position", { north: 1, east: 2, altitude: 2 }],
    ["obstacles", []],
    ["training", "hover"],
    ["environment", { preset: "field", timeOfDay: 18, fog: 0.2, rain: 0.5 }],
    ["render", false],
  ])
    input.events.push({ time: 0, type, value });
  input.events.push(
    {
      time: 0,
      type: "fault",
      fault: "motor",
      value: { index: 0, efficiency: 0.5 },
    },
    { time: 0, type: "fault", fault: "link", value: { lost: true } },
  );
  assert.equal(validateRecording(input).events.length, input.events.length);
  input.events.push({
    time: 0,
    type: "fault",
    fault: "motor",
    value: { index: 4, efficiency: 0 },
  });
  assert.throws(() => validateRecording(input));
});
test("setup defaults only missing fields and validates nested trust boundaries", () => {
  assert.equal(validateSetup(undefined).mode, "attitude");
  assert.equal(validateSetup({ training: "hover" }).training, "hover");
  for (const setup of [
    null,
    [],
    { mode: "mission" },
    { mission: Array(1) },
    { environment: { preset: "field", timeOfDay: 25, fog: 0, rain: 0 } },
    { obstacles: [{ north: 0, east: 0, width: -1, depth: 1, height: 1 }] },
    { training: "bad" },
  ])
    assert.throws(() => validateSetup(setup));
  const input = record();
  input.samples[0].extra = { foo: 1 };
  assert.equal(validateRecording(input).samples[0].extra, undefined);
});
test("large fifty-point routes are stored once and rehydrated by shared reference", () => {
  const input = record(),
    route = Array.from({ length: 50 }, (_, i) => ({
      north: i,
      east: 0,
      altitude: 2,
      hold: 0,
    }));
  input.setup.mission = route;
  const base = input.samples[0];
  input.samples = Array.from({ length: 1000 }, (_, i) =>
    compactSnapshot({
      ...base,
      time: i * 0.05,
      navigation: { ...base.navigation, waypoints: route },
    }),
  );
  const json = recordingToJson(input);
  assert.equal((json.match(/"hold"/g) || []).length, 50);
  const validated = validateRecording(JSON.parse(json));
  assert.equal(
    validated.samples[0].navigation.waypoints,
    validated.samples[999].navigation.waypoints,
  );
  assert.equal(validated.samples[999].navigation.waypoints.length, 50);
});
test("gate training nextGate is validated against the selected course", () => {
  const input = record(),
    trainer = createTrainer("gates");
  input.setup.training = "gates";
  input.samples = input.samples.map((snapshot) => ({
    ...snapshot,
    training: trainer.update(snapshot),
  }));
  assert.equal(validateRecording(input).samples[0].training.nextGate.north, 8);
  input.samples[0].training.nextGate = {
    ...input.samples[0].training.nextGate,
    radius: 10,
  };
  assert.throws(() => validateRecording(input), /training gate/);
});
