import { AIRFRAMES, rotorCount } from "../rotor-geometry.js";
import { DEFAULT_SCENE } from "../scene-data.js";
import test from "node:test";
import assert from "node:assert/strict";
import { createSimulation, validateConfig } from "../sim.js";
import { createSensorSuite } from "../sensors.js";
import { createTrainer } from "../training.js";
import { DEFAULT_GIMBAL_SETTINGS } from "../gimbal.js";
import {
  RECORDING_VERSION,
  MODEL_VERSION,
  validateSetup,
  validateRecording,
  recordingToCsv,
  recordingToJson,
  compactSnapshot,
  validateSensorFrame,
} from "../recording.js";

function record(airframe = "quadrotor", overrides = {}) {
  const config = validateConfig({ airframe, ...overrides });
  const sim = createSimulation(config),
    trainer = createTrainer("free", airframe);
  const capture = () => {
    const snapshot = sim.snapshot();
    return { ...snapshot, training: trainer.update(snapshot) };
  };
  const first = capture();
  for (let i = 0; i < 10; i++) sim.step();
  const last = capture(),
    sensors = createSensorSuite();
  return {
    format: "flight-lab",
    version: RECORDING_VERSION,
    modelVersion: MODEL_VERSION,
    createdAt: "2026-11-04T00:00:00.000Z",
    config,
    scenario: "hover",
    dt: 0.005,
    setup: validateSetup(undefined, config),
    runtime: {
      speed: 1,
      wallSeconds: 0.05,
      p95LateMs: 0,
      lateTicks: 0,
      skippedWallSeconds: 0,
      renderEnabled: true,
    },
    events: [{ time: 0, type: "targets", value: { altitude: 2 } }],
    samples: [first, last],
    sensorFrames: [sensors.update(first), sensors.update(last)],
  };
}
test("v10 round trip preserves multi-layer scans and exports 164 telemetry CSV columns", () => {
  const input = record(),
    result = validateRecording(JSON.parse(JSON.stringify(input)));
  assert.deepEqual(result, JSON.parse(JSON.stringify(input)));
  const rows = recordingToCsv(input)
    .split("\n")
    .map((row) => row.split(","));
  assert.equal(rows.length, 3);
  assert.equal(rows[0].length, rows[1].length);
  assert.equal(rows[0].length, 164);
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
      .flatMap((row) => row.slice(0, 68))
      .every((value) => value !== "" && Number.isFinite(Number(value))),
  );
  const json = recordingToJson(input);
  assert.equal(json.includes('"waypoints"'), false);
  assert.deepEqual(
    validateRecording(JSON.parse(json)),
    JSON.parse(JSON.stringify(result)),
  );
});
test("gimbal history restores exact acquired poses from compact recording samples and rejects forged settings", () => {
  const input = record();
  const sim = createSimulation(input.config, { gimbal: input.setup.gimbal });
  const trainer = createTrainer("free"),
    sensors = createSensorSuite();
  const settings = {
    ...DEFAULT_GIMBAL_SETTINGS,
    target: { yaw: 1, pitch: 0, roll: 0 },
  };
  sim.setGimbal(settings);
  const capture = () => {
    const s = sim.snapshot();
    return { ...s, training: trainer.update(s) };
  };
  const first = capture();
  for (let i = 0; i < 10; i++) sim.step();
  const last = capture();
  input.events = [{ time: 0, type: "gimbal", value: settings }];
  input.samples = [first, last];
  input.sensorFrames = input.samples.map((s) => sensors.update(s));
  const json = recordingToJson(input),
    compact = JSON.parse(json);
  assert.equal(compact.samples[1].gimbal.length, 6);
  assert.ok(JSON.stringify(compact.samples[1].gimbal).length < 100);
  const restored = validateRecording(compact);
  assert.deepEqual(restored.samples[1].gimbal, last.gimbal);
  assert.notEqual(last.gimbal.actual.yaw, settings.target.yaw);
  for (const mutate of [
    (r) => delete r.setup.gimbal,
    (r) => delete r.samples[0].gimbal,
    (r) => r.samples[0].gimbal.push(0),
    (r) => (r.samples[0].gimbal[0] = NaN),
    (r) => (r.samples[0].gimbal[3] = 8),
    (r) => (r.samples[0].gimbal[4] = -1),
    (r) => (r.samples[0].gimbal[5] = 0),
    (r) => (r.events[0].value.maxRate.yaw = 0),
  ]) {
    const bad = structuredClone(compact);
    mutate(bad);
    assert.throws(() => validateRecording(bad));
  }
  const bad = structuredClone(input);
  bad.samples[1].gimbal.target.yaw = 2;
  assert.throws(() => validateRecording(bad));
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
    ["scene", DEFAULT_SCENE],
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
  assert.equal(
    validateSetup({ scene: { ...DEFAULT_SCENE, training: "hover" } }).scene
      .training,
    "hover",
  );
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
  const suite = createSensorSuite(input.setup.sensors);
  input.sensorFrames = input.samples.map((snapshot) => suite.update(snapshot));
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
  input.setup.scene.training = "gates";
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

test("sensor frames require exact parallel times and reject future/regressing acquisition times", () => {
  const mutations = [
    (r) => r.sensorFrames.pop(),
    (r) => (r.sensorFrames[1].time = 0.04),
    (r) => (r.sensorFrames[0].gps.time = 1),
    (r) => {
      r.sensorFrames[0].gps.time = 0;
      r.sensorFrames[1].gps.time = -1;
    },
    (r) => delete r.sensorFrames[0].faults.gps,
    (r) => (r.sensorFrames[0].gps.latitudeDeg = Infinity),
    (r) => (r.sensorFrames[0].gps.latitudeDeg = 91),
    (r) => (r.sensorFrames[0].barometer.temperatureK = 0),
    (r) => (r.sensorFrames[0].rangefinder.distanceM = 0.01),
    (r) => r.sensorFrames[0].lidar.rangesM.pop(),
    (r) => (r.sensorFrames[0].lidar.rangesM[0] = 51),
    (r) => (r.sensorFrames[0].lidar.channelCount = 7),
    (r) => (r.sensorFrames[0].lidar.azimuthCount = 36),
    (r) => (r.sensorFrames[0].lidar.elevationStartRad = 0),
    (r) => (r.sensorFrames[0].lidar.elevationStepRad = 0),
    (r) => delete r.sensorFrames[0].lidar.rangesM[0],
    (r) => (r.sensorFrames[0].lidar.quaternion = [0, 0, 0, 0]),
    (r) => {
      r.sensorFrames[0].gps.valid = false;
      r.sensorFrames[0].gps.reason = "dropout";
    },
    (r) => (r.sensorFrames[0].gps.reason = "fault"),
    (r) => (r.sensorFrames[0].faults.gps = 1),
  ];
  for (const mutate of mutations) {
    const input = record();
    mutate(input);
    assert.throws(() => validateRecording(input));
  }
  const input = record();
  input.samples[0].time = 0.1;
  input.samples[1].time = 0.2;
  input.sensorFrames[0].time = 0.1;
  input.sensorFrames[1].time = 0.2;
  input.sensorFrames[0].gps.time = 0.1;
  input.sensorFrames[1].gps.time = 0.05;
  assert.throws(() => validateRecording(input), /backwards/);
  for (const version of [1, 2, 3, 4, 5]) {
    const legacy = record();
    legacy.version = version;
    assert.throws(() => validateRecording(legacy), /version 10 required/);
  }
});
test("no-return and startup measurements retain explicit nulls and produce empty CSV sensor cells", () => {
  const input = record();
  input.setup.sensors = {
    ...input.setup.sensors,
    lidarChannels: 1,
    lidarVerticalMinDeg: 0,
    lidarVerticalMaxDeg: 0,
  };
  const planar = createSensorSuite(input.setup.sensors);
  input.sensorFrames = input.samples.map((sample) => planar.update(sample));
  assert.equal(input.sensorFrames[0].lidar.reason, "no-return");
  assert.ok(
    input.sensorFrames[0].lidar.rangesM.every((value) => value === null),
  );
  assert.deepEqual(
    validateRecording(JSON.parse(recordingToJson(input))).sensorFrames,
    input.sensorFrames,
  );
  const rows = recordingToCsv(input)
    .split("\n")
    .map((row) => row.split(","));
  assert.equal(rows[1][rows[0].indexOf("lidar_nearest_m")], "");
  assert.equal(rows[1][rows[0].indexOf("lidar_valid_boolean")], "0");
  const suite = createSensorSuite({ ...input.setup.sensors, latency: 1 });
  const startup = suite.update(input.samples[0]);
  assert.deepEqual(
    validateSensorFrame(startup, { ...input.setup.sensors, latency: 1 }),
    startup,
  );
  startup.gps.latitudeDeg = 0;
  assert.throws(() => validateSensorFrame(startup));
});
test("sensor config/fault history validates different lidar dimensions and preserves exact raw replay", () => {
  const input = record(),
    changed = {
      ...input.setup.sensors,
      lidarRays: 36,
      lidarChannels: 4,
      latency: 0.1,
    };
  input.events.push(
    { time: input.samples[1].time, type: "sensorConfig", value: changed },
    {
      time: input.samples[1].time,
      type: "sensorFault",
      value: { sensor: "gps", failed: true },
    },
  );
  const suite = createSensorSuite(changed);
  suite.update(input.samples[1]);
  suite.setFault("gps", true);
  input.sensorFrames[1] = suite.snapshot();
  const validated = validateRecording(input);
  assert.equal(validated.sensorFrames[0].lidar.rangesM.length, 72 * 8);
  assert.equal(validated.sensorFrames[1].lidar.rangesM.length, 36 * 4);
  assert.equal(validated.sensorFrames[1].gps.reason, "fault");
  assert.deepEqual(
    validateRecording(JSON.parse(recordingToJson(input))).sensorFrames,
    input.sensorFrames,
  );
  const bad = structuredClone(input);
  bad.events[1].value.lidarRays = 37;
  assert.throws(() => validateRecording(bad), /range count/);
  const fault = record();
  fault.events.push({
    time: 0,
    type: "sensorFault",
    value: { sensor: "unknown", failed: true },
  });
  assert.throws(() => validateRecording(fault));
  const config = record();
  config.events.push({
    time: 0,
    type: "sensorConfig",
    value: { lidarRays: 181 },
  });
  assert.throws(() => validateRecording(config));
});

test("terrain scene history restores ground and AGL targets without repeating ground metadata", () => {
  const input = record(), heights = Array(25).fill(0); heights[12] = 4;
  const terrain = { kind: "heightfield", north: 40, east: 40, cellSize: 10, rows: 5, cols: 5, heights };
  input.setup.scene = { ...input.setup.scene, terrain }; input.setup.mode = "position";
  const sim = createSimulation(input.config, { terrain, gimbal: input.setup.gimbal });
  const trainer = createTrainer("free"), sensors = createSensorSuite(); sensors.setTerrain(terrain);
  sim.setMode("position");
  const command = { north: 60, east: 60, altitude: 6, altitudeReference: "agl" };
  sim.setPositionTarget(command);
  input.samples = []; input.sensorFrames = []; input.events = [{ time: 0, type: "position", value: command }];
  const capture = () => { const s = sim.snapshot(); input.samples.push({ ...s, training: trainer.update(s) }); input.sensorFrames.push(sensors.update(s)); };
  capture(); for (let i = 0; i < 8000; i++) sim.step(); capture();
  assert.ok(Math.abs(input.samples[1].ground.clearanceM - 6) < .1);
  const nextScene = structuredClone(input.setup.scene); nextScene.terrain.heights[12] = -4;
  for (let i = 0; i < 10; i++) sim.step(); sim.setTerrain(nextScene.terrain); sensors.setTerrain(nextScene.terrain);
  input.events.push({ time: sim.snapshot().time, type: "scene", value: nextScene }); capture();
  const serialized = JSON.parse(recordingToJson(input));
  assert.equal(Object.hasOwn(serialized.samples[1], "ground"), false);
  assert.deepEqual(serialized.samples[1].navigation.altitude, [1, 6]);
  const restored = validateRecording(serialized);
  assert.ok(Math.abs(restored.samples[1].ground.heightM - 4) < .1);
  assert.ok(Math.abs(restored.samples[2].ground.heightM + 4) < .1);
  assert.ok(restored.samples[2].ground.clearanceM > 13);
  for (const mutate of [
    (r) => (r.samples[1].navigation.altitude[0] = 2),
    (r) => (r.samples[1].navigation.altitude[1] = -1),
    (r) => (r.samples[1].ground = { heightM: 0, clearanceM: 0, normalNed: [0, 0, -1] }),
    (r) => (r.events[0].value.altitude = 98),
    (r) => delete r.setup.scene.terrain,
  ]) { const bad = structuredClone(serialized); mutate(bad); assert.throws(() => validateRecording(bad)); }
  assert.equal(recordingToCsv(restored).split("\n")[0].split(",").length, 164);
});

test("rotor arrays follow the selected frame and inactive CSV channels remain empty", () => {
  for (const airframe of AIRFRAMES) {
    const input = record(airframe),
      n = rotorCount(airframe);
    const result = validateRecording(JSON.parse(recordingToJson(input)));
    assert.equal(result.samples[1].motors.length, n);
    assert.deepEqual(result.samples[1].motors, input.samples[1].motors);
    const [header, row] = recordingToCsv(input)
      .split("\n")
      .map((s) => s.split(","));
    assert.equal(row[header.indexOf("lift_motor_channel_count")], String(n));
    for (let i = 5; i <= 8; i++) {
      for (const [key, array] of [
        [`motor_${i}_normalized`, input.samples[0].motors],
        [`pwm_${i}_us`, input.samples[0].pwm],
        [
          `motor_${i}_efficiency_fraction`,
          input.samples[0].faults.motorEfficiency,
        ],
      ])
        assert.equal(
          row[header.indexOf(key)],
          i <= n ? String(array[i - 1]) : "",
        );
    }
    for (const mutate of [
      (s) => s.motors.pop(),
      (s) => s.pwm.push(1000),
      (s) => s.faults.motorEfficiency.pop(),
      (s) => (s.motors[n - 1] = NaN),
    ]) {
      const bad = structuredClone(input);
      mutate(bad.samples[0]);
      assert.throws(() => validateRecording(bad));
    }
    input.events.push({
      time: 0,
      type: "fault",
      fault: "motor",
      value: { index: n, efficiency: 0 },
    });
    assert.throws(() => validateRecording(input));
  }
});

test("all airframes preserve propulsion, surfaces and flight data in schema 10 JSON and CSV", () => {
  for (const airframe of AIRFRAMES) {
    const input = record(airframe);
    if (["fixedwing", "vtol"].includes(airframe))
      input.events.push({
        time: 0,
        type: "fault",
        fault: "pusher",
        value: { efficiency: 0.5 },
      });
    if (airframe === "vtol")
      input.events.push({ time: 0, type: "transition", value: "cruise" });
    const roundtrip = validateRecording(JSON.parse(recordingToJson(input)));
    for (let i = 0; i < input.samples.length; i++) {
      for (const key of [
        "airframe",
        "propulsion",
        "controlSurfaces",
        "flight",
        "faults",
        "target",
        "runway",
      ])
        assert.deepEqual(roundtrip.samples[i][key], input.samples[i][key]);
    }
    const [header, row] = recordingToCsv(input)
      .split("\n")
      .map((line) => line.split(","));
    assert.equal(header.length, 164);
    assert.equal(
      row[header.indexOf("airframe_code")],
      String(AIRFRAMES.indexOf(airframe)),
    );
    assert.equal(
      Number(row[header.indexOf("airspeed_m_s")]),
      input.samples[0].flight.airspeedMps,
    );
    assert.ok(
      row
        .slice(98, 144)
        .every((cell) => cell !== "" && Number.isFinite(Number(cell))),
    );
  }
});

test("schema 10 rejects mismatched airframes, malformed flight state and invalid transition or propulsion faults", () => {
  const mutations = [
    (r) => (r.samples[0].airframe = "fixedwing"),
    (r) => delete r.samples[0].flight,
    (r) => (r.samples[0].flight.airspeedMps = Infinity),
    (r) => (r.samples[0].flight.transitionPhase = "unknown"),
    (r) => (r.samples[0].flight.transitionProgress = 2),
    (r) => (r.samples[0].flight.stalled = 1),
    (r) => (r.samples[0].propulsion.motor = -0.1),
    (r) => (r.samples[0].propulsion.thrustN = -1),
    (r) => (r.samples[0].controlSurfaces.elevator = Math.PI),
    (r) => (r.samples[0].faults.pusherEfficiency = 1.1),
    (r) => (r.samples[0].target.airspeed = 41),
    (r) => r.events.push({ time: 0, type: "transition", value: "cruise" }),
    (r) =>
      r.events.push({
        time: 0,
        type: "fault",
        fault: "pusher",
        value: { efficiency: 0.5 },
      }),
  ];
  for (const mutate of mutations) {
    const input = record();
    mutate(input);
    assert.throws(() => validateRecording(input));
  }
  const wing = record("fixedwing");
  wing.setup.scene.training = "hover";
  assert.throws(() => validateRecording(wing), /not supported/);
  const vtol = record("vtol");
  vtol.events.push({ time: 0, type: "transition", value: "invalid" });
  assert.throws(() => validateRecording(vtol));
});

test("ground-start runway takeoff and abort history round trip with exact phase and brake data", () => {
  const input = record("fixedwing", { startOnGround: true });
  const sim = createSimulation(input.config),
    trainer = createTrainer("runway", "fixedwing"),
    sensors = createSensorSuite();
  input.setup.scene.training = "runway";
  input.samples = [];
  input.sensorFrames = [];
  const capture = () => {
    const state = sim.snapshot();
    input.samples.push({ ...state, training: trainer.update(state) });
    input.sensorFrames.push(sensors.update(state));
  };
  sim.takeoff();
  input.events = [{ time: 0, type: "takeoff" }];
  capture();
  for (let i = 1; i <= 200; i++) {
    sim.step();
    if (i % 10 === 0) capture();
  }
  sim.abortTakeoff();
  input.events.push({ time: sim.snapshot().time, type: "abortTakeoff" });
  input.samples.pop();
  input.sensorFrames.pop();
  capture();
  for (let i = 1; i <= 400; i++) {
    sim.step();
    if (i % 10 === 0) capture();
  }
  const roundtrip = validateRecording(JSON.parse(recordingToJson(input)));
  assert.equal(roundtrip.config.startOnGround, true);
  assert.ok(
    roundtrip.samples.some((s) =>
      ["runup", "roll"].includes(s.runway.takeoffPhase),
    ),
  );
  assert.ok(roundtrip.samples.some((s) => s.runway.takeoffPhase === "aborted"));
  assert.deepEqual(
    roundtrip.samples.map((s) => s.runway),
    input.samples.map((s) => s.runway),
  );
  const [header, ...rows] = recordingToCsv(input)
    .split("\n")
    .map((line) => line.split(","));
  assert.equal(header.length, 164);
  const phase = header.indexOf("takeoff_phase_code"),
    brake = header.indexOf("wheel_brake_fraction");
  assert.ok(rows.some((row) => row[phase] === "6"));
  assert.ok(rows.some((row) => Number(row[brake]) > 0));
});

test("runway schema rejects missing fields, invalid booleans, phase enums and aborts on other airframes", () => {
  const mutations = [
    (r) => delete r.samples[0].runway,
    (r) => (r.samples[0].runway.weightOnWheels = 1),
    (r) => (r.samples[0].runway.onRunway = "true"),
    (r) => (r.samples[0].runway.distanceRemainingM = -1),
    (r) => (r.samples[0].runway.crossTrackM = Infinity),
    (r) => (r.samples[0].runway.rotationSpeedMps = -1),
    (r) => (r.samples[0].runway.takeoffPhase = "unknown"),
    (r) => (r.samples[0].runway.abortReason = "unknown"),
    (r) => (r.samples[0].runway.brake = 1.1),
    (r) => r.events.push({ time: 0, type: "abortTakeoff" }),
    (r) => r.events.push({ time: 0, type: "mode", value: "takeoff" }),
    (r) => (r.samples[0].navigation.mode = "takeoff"),
    (r) => (r.config.startOnGround = 1),
  ];
  for (const mutate of mutations) {
    const input = record();
    mutate(input);
    assert.throws(() => validateRecording(input));
  }
});

test("scene epochs select immutable custom gate sources and reject forged totals, next gates and completion", () => {
  const input = record();
  const first = { north: 3, east: 0, altitude: 2, yaw: 0, radius: 1 };
  const second = { ...first, north: 7 };
  input.setup.scene = {
    ...DEFAULT_SCENE,
    training: "custom",
    customGates: [first],
  };
  const changed = {
    ...DEFAULT_SCENE,
    name: "第二課程",
    training: "custom",
    customGates: [second],
  };
  input.events = [
    { time: input.samples[1].time, type: "scene", value: changed },
  ];
  input.samples[0].training = createTrainer("custom", "quadrotor", [
    first,
  ]).update(input.samples[0]);
  input.samples[1].training = createTrainer("custom", "quadrotor", [
    second,
  ]).update(input.samples[1]);
  const valid = validateRecording(input);
  assert.equal(valid.samples[0].training.nextGate.north, 3);
  assert.equal(valid.samples[1].training.nextGate.north, 7);
  for (const mutate of [
    (r) => (r.samples[1].training.total = 2),
    (r) => (r.samples[1].training.nextGate.north = 3),
    (r) => (r.samples[1].training.complete = true),
    (r) => (r.events[0].type = "training"),
    (r) => (r.setup.obstacles = []),
    (r) => (r.setup.scene.training = "free"),
  ]) {
    const bad = structuredClone(input);
    mutate(bad);
    assert.throws(() => validateRecording(bad));
  }
  for (const version of [1, 2, 3, 4, 5, 6, 7, 8, 9])
    assert.throws(() => validateRecording({ ...input, version }), /version 10/);
});
