import test from "node:test";
import assert from "node:assert/strict";
import { createTrainer } from "../training.js";
import { createSimulation } from "../sim.js";
import { AIRFRAMES } from "../rotor-geometry.js";

test("six and eight rotor aircraft complete hover training and preserve course restrictions", async () => {
  const { isCourseSupported } = await import("../training.js");
  for (const airframe of ["hexarotor", "octorotor"]) {
    for (const course of [
      "free",
      "hover",
      "gates",
      "landing",
      "circuit",
      "custom",
    ])
      assert.equal(isCourseSupported(course, airframe), true);
    assert.equal(isCourseSupported("runway", airframe), false);
    const sim = createSimulation({ airframe }),
      trainer = createTrainer("hover", airframe);
    trainer.update(sim.snapshot());
    let state;
    for (let i = 0; i < 2200; i++) state = trainer.update(sim.step());
    assert.equal(state.complete, true);
    assert.equal(state.reason, "");
  }
  assert.equal(AIRFRAMES.length, 5);
});

const sample = (time, position = [0, 0, -2], extra = {}) => ({
  time,
  position,
  velocity: [0, 0, 0],
  status: "flying",
  armed: true,
  ...extra,
});

test("gate crossing uses the ordered plane and interpolated position", () => {
  const t = createTrainer("gates");
  t.update(sample(0, [10, 0, -2]));
  assert.equal(t.update(sample(1, [7, 0, -2])).passed, 0); // backward
  assert.equal(t.update(sample(2, [9, 3, -2])).misses, 1); // outside
  t.update(sample(3, [0, 0, -2]));
  assert.equal(t.update(sample(4, [16, 0, -2])).passed, 1); // large jump
  assert.equal(t.update(sample(5, [0, 9, -2])).passed, 1); // misses second gate
  t.update(sample(6, [8, 7, -2]));
  assert.equal(t.update(sample(7, [8, 9, -2])).passed, 2);
  t.update(sample(8, [1, 8, -2]));
  assert.equal(t.update(sample(9, [-1, 8, -2])).passed, 3);
  t.update(sample(10, [0, 1, -2]));
  const done = t.update(sample(11, [0, -1, -2]));
  assert.equal(done.complete, true);
  assert.deepEqual(t.update(sample(20)), done);
});

test("hover must be continuous and reset when time goes backwards", () => {
  const t = createTrainer("hover");
  t.update(sample(0));
  assert.equal(t.update(sample(9)).complete, false);
  t.update(sample(10, [2, 0, -2]));
  t.update(sample(11));
  assert.equal(t.update(sample(20)).complete, false);
  assert.equal(t.update(sample(21)).complete, true);
  const restarted = t.update(sample(1));
  assert.equal(restarted.elapsed, 0);
  assert.equal(restarted.complete, false);
});

test("landing requires descent and two continuous seconds on the ground", () => {
  const t = createTrainer("landing");
  t.update(sample(0, [0, 0, 0]));
  assert.equal(t.update(sample(10, [0, 0, 0])).complete, false);
  t.update(sample(11, [0, 0, -1]));
  t.update(sample(12, [0.5, 0, 0]));
  assert.equal(t.update(sample(14, [0.5, 0, 0])).score, 950);
});

test("crash latches, fence penalty occurs once, and invalid input is rejected", () => {
  assert.throws(() => createTrainer("nope"));
  const t = createTrainer("hover");
  assert.throws(() => t.update(sample(0, [NaN, 0, 0])));
  t.update(sample(0));
  t.update(sample(1, [0, 0, -2], { navigation: { fenceBreached: true } }));
  assert.equal(
    t.update(sample(10, [0, 0, -2], { navigation: { fenceBreached: true } }))
      .score,
    900,
  );
  t.reset();
  assert.equal(
    t.update(sample(1, [0, 0, -2], { status: "crashed" })).reason,
    "crashed",
  );
  assert.equal(t.update(sample(2)).reason, "crashed");
  t.reset();
  assert.equal(t.update(sample(0)).reason, "");
});

test("free flight never completes or scores", () => {
  const t = createTrainer();
  t.update(sample(0));
  const result = t.update(sample(100));
  assert.equal(result.score, 0);
  assert.equal(result.complete, false);
});

test("course compatibility rejects unknown and unsuitable aircraft", async () => {
  const { isCourseSupported } = await import("../training.js");
  assert.equal(isCourseSupported("circuit", "fixedwing"), true);
  assert.equal(isCourseSupported("hover", "fixedwing"), false);
  assert.equal(isCourseSupported("free", "vtol"), true);
  assert.equal(isCourseSupported("unknown", "vtol"), false);
  assert.equal(isCourseSupported("circuit", "unknown"), false);
  assert.throws(() => createTrainer("hover", "fixedwing"));
  assert.throws(() => createTrainer("free", "unknown"));
});

test("circuit uses ordered positive plane crossings with interpolated positions", () => {
  const t = createTrainer("circuit", "fixedwing");
  t.update(sample(0, [110, 0, -30]));
  assert.equal(t.update(sample(1, [90, 0, -30])).passed, 0);
  assert.equal(t.update(sample(2, [110, 50, -30])).misses, 1);
  t.update(sample(3, [80, 0, -30]));
  assert.equal(t.update(sample(4, [120, 0, -30])).passed, 1);
  t.update(sample(5, [100, 100, -30]));
  assert.equal(t.update(sample(6, [100, 140, -30])).passed, 2);
  t.update(sample(7, [20, 120, -30]));
  assert.equal(t.update(sample(8, [-20, 120, -30])).passed, 3);
  t.update(sample(9, [0, 20, -30]));
  assert.equal(t.update(sample(10, [0, -20, -30])).complete, true);
});

const runwaySample = (time, phase, extra = {}) =>
  sample(time, extra.position ?? [0, 0, 0], {
    status: "landed",
    armed: false,
    flight: { airspeedMps: 0 },
    runway: {
      onRunway: true,
      weightOnWheels: true,
      rotationSpeedMps: 10,
      takeoffPhase: phase,
    },
    ...extra,
  });

test("runway practice requires ground start, takeoff and stable disarmed runway landing", () => {
  const t = createTrainer("runway", "fixedwing");
  assert.throws(() => createTrainer("runway", "vtol"));
  const ground = t.update(runwaySample(0, "idle"));
  assert.equal(ground.total, 0);
  assert.equal(ground.passed, 0);
  t.update(runwaySample(1, "runup", { armed: true }));
  t.update(runwaySample(2, "rotate", { armed: true }));
  t.update(
    runwaySample(3, "complete", {
      position: [20, 0, -25],
      status: "flying",
      armed: true,
      flight: { airspeedMps: 10 },
      runway: {
        onRunway: true,
        weightOnWheels: false,
        rotationSpeedMps: 10,
        takeoffPhase: "complete",
      },
    }),
  );
  assert.equal(
    t.update(runwaySample(4, "complete", { armed: true })).complete,
    false,
  );
  t.update(runwaySample(5, "complete"));
  assert.equal(
    t.update(
      runwaySample(6, "complete", {
        runway: {
          onRunway: false,
          weightOnWheels: true,
          rotationSpeedMps: 10,
          takeoffPhase: "complete",
        },
      }),
    ).holdSeconds,
    0,
  );
  t.update(runwaySample(7, "complete"));
  const done = t.update(runwaySample(9, "complete"));
  assert.equal(done.complete, true);
  assert.equal(done.score, 1000);
});

test("air start cannot earn runway score and aborted or crashed runs latch until reset", () => {
  const t = createTrainer("runway", "fixedwing");
  t.update(
    runwaySample(0, "complete", {
      position: [0, 0, -30],
      status: "flying",
      armed: true,
      flight: { airspeedMps: 15 },
      runway: {
        onRunway: true,
        weightOnWheels: false,
        rotationSpeedMps: 10,
        takeoffPhase: "complete",
      },
    }),
  );
  t.update(runwaySample(1, "complete"));
  assert.equal(t.update(runwaySample(4, "complete")).complete, false);
  t.reset();
  t.update(runwaySample(0, "idle"));
  assert.equal(t.update(runwaySample(1, "aborted")).reason, "takeoff-aborted");
  assert.equal(t.update(runwaySample(2, "idle")).reason, "takeoff-aborted");
  t.reset();
  assert.equal(
    t.update(runwaySample(0, "idle", { status: "crashed" })).reason,
    "crashed",
  );
  assert.equal(t.update(runwaySample(1, "idle")).reason, "crashed");
});

test("custom gates preserve ordered continuous segment intersections and immutable scoring sources", async () => {
  const { TRAINING_COURSES, resolveTrainingGates } =
    await import("../training.js");
  const gates = [2, 4, 6].map((north) => ({
    north,
    east: 0,
    altitude: 2,
    yaw: 0,
    radius: 1,
  }));
  const trainer = createTrainer("custom", "fixedwing", gates);
  gates[0].north = 100;
  let result = trainer.update(sample(0, [0, 0, -2]));
  result.nextGate.north = 999;
  result = trainer.update(sample(1, [8, 0, -2]));
  assert.equal(result.passed, 3);
  assert.equal(result.complete, true);
  const reversed = createTrainer("custom", "quadrotor", [
    { ...gates[1], north: 6 },
    { ...gates[2], north: 2 },
  ]);
  reversed.update(sample(0, [0, 0, -2]));
  assert.equal(reversed.update(sample(1, [8, 0, -2])).passed, 1);
  const backward = createTrainer("custom", "vtol", [{ ...gates[1], north: 2 }]);
  backward.update(sample(0, [4, 0, -2]));
  assert.equal(backward.update(sample(1, [0, 0, -2])).passed, 0);
  const edge = createTrainer("custom", "quadrotor", [
    { ...gates[1], north: 2 },
  ]);
  edge.update(sample(0, [0, 0.8, -2]));
  assert.equal(edge.update(sample(1, [4, 0.8, -2])).passed, 1);
  const miss = createTrainer("custom", "quadrotor", [
    { ...gates[1], north: 2 },
  ]);
  miss.update(sample(0, [0, 0.8001, -2]));
  assert.equal(miss.update(sample(1, [4, 0.8001, -2])).misses, 1);
  assert.ok(Object.isFrozen(TRAINING_COURSES.gates.gates[0]));
  const resolved = resolveTrainingGates({ training: "gates", customGates: [] });
  resolved[0].north = 99;
  assert.equal(TRAINING_COURSES.gates.gates[0].north, 8);
  assert.throws(() => createTrainer("custom"));
  assert.throws(() => createTrainer("custom", "quadrotor", Array(1)));
});

test("tutorial completes all seven real physical stages on quad, hexa, octo and hovering VTOL", async () => {
  const { TUTORIAL_STEPS, TRAINING_COURSES } = await import("../training.js");
  assert.deepEqual(Object.keys(TRAINING_COURSES), [
    "free",
    "hover",
    "gates",
    "landing",
    "runway",
    "circuit",
    "custom",
    "tutorial",
  ]);
  assert.deepEqual(
    TUTORIAL_STEPS.map((step) => step.holdSeconds),
    [2, 1, 1, 5, 3, 3, 2],
  );
  assert.equal(
    TUTORIAL_STEPS.reduce((sum, step) => sum + step.holdSeconds, 0),
    17,
  );
  assert.ok(Object.isFrozen(TUTORIAL_STEPS[0]));
  for (const airframe of ["quadrotor", "hexarotor", "octorotor", "vtol"]) {
    const sim = createSimulation({
        airframe,
        startOnGround: true,
        geofenceRadius: 10,
        geofenceHeight: 3,
      }),
      trainer = createTrainer("tutorial", airframe);
    let result = trainer.update(sim.snapshot()),
      handled = -1;
    const transitions = [];
    assert.equal(result.passed, 0);
    assert.equal(result.total, 7);
    assert.equal(result.nextGate, null);
    for (let i = 0; i < 30000 && !result.complete; i++) {
      if (result.passed !== handled) {
        handled = result.passed;
        transitions.push(handled);
        if (handled === 1) sim.arm();
        if (handled === 2) sim.takeoff();
        if (handled === 4)
          sim.setPositionTarget({ north: 8, east: 0, altitude: 2 });
        if (handled === 5)
          sim.setPositionTarget({ north: 0, east: 0, altitude: 2 });
        if (handled === 6) sim.setMode("land");
      }
      const before = result.passed;
      result = trainer.update(sim.step());
      assert.ok(result.passed - before <= 1);
      assert.equal(result.reason, "", `${airframe} failed stage${before}`);
    }
    assert.equal(result.complete, true, `${airframe} did not finish`);
    assert.equal(result.passed, 7);
    assert.equal(result.holdSeconds, 0);
    assert.equal(result.score, 1000);
    assert.ok(result.elapsed >= 17);
    assert.deepEqual(transitions, [0, 1, 2, 3, 4, 5, 6]);
    assert.equal(sim.snapshot().armed, false);
    assert.equal(sim.snapshot().status, "landed");
    const terminal = structuredClone(result);
    assert.deepEqual(trainer.update(sim.step()), terminal);
  }
});

test("tutorial rejects bad starts and freezes failed progress after premature commands, faults or a VTOL transition", () => {
  assert.throws(() => createTrainer("tutorial", "fixedwing"), /unsupported/);
  const airborne = createSimulation();
  assert.throws(
    () => createTrainer("tutorial").update(airborne.snapshot()),
    /ground start/,
  );
  const external = createSimulation(
    { startOnGround: true },
    { controlSource: "external" },
  );
  assert.throws(
    () => createTrainer("tutorial").update(external.snapshot()),
    /internal/,
  );
  const sim = createSimulation({ startOnGround: true }),
    trainer = createTrainer("tutorial");
  trainer.update(sim.snapshot());
  sim.arm();
  const failed = trainer.update(sim.step());
  assert.equal(failed.reason, "procedure-order");
  assert.equal(failed.score, 0);
  assert.equal(failed.holdSeconds, 0);
  sim.disarm();
  assert.deepEqual(trainer.update(sim.step()), failed);
  const scenarios = [
    ["unsafe", (s) => s.injectFault("motor", { index: 0, efficiency: 0.5 })],
    ["unsafe", (s) => s.injectFault("link", { lost: true })],
    ["crashed", null],
    ["failsafe", null],
  ];
  for (const [expected, action] of scenarios) {
    const s = createSimulation({ startOnGround: true }),
      t = createTrainer("tutorial");
    t.update(s.snapshot());
    if (action) {
      action(s);
      assert.equal(t.update(s.step()).reason, expected);
    } else {
      const value = s.snapshot();
      value.time = 0.005;
      value.status = expected;
      assert.equal(t.update(value).reason, expected);
    }
  }
  const v = createSimulation({ airframe: "vtol", startOnGround: true }),
    t = createTrainer("tutorial", "vtol");
  t.update(v.snapshot());
  const sample = v.snapshot();
  sample.time = 0.005;
  sample.flight.transitionProgress = 0.0001;
  assert.equal(t.update(sample).reason, "unsafe");
});

test("tutorial dwell cannot be earned while paused or through sample gaps, and an airborne jump cannot skip armed procedure", () => {
  const sim = createSimulation({ startOnGround: true }),
    trainer = createTrainer("tutorial");
  let snapshot = sim.snapshot(),
    result = trainer.update(snapshot);
  for (let i = 0; i < 100; i++) result = trainer.update(snapshot);
  assert.equal(result.holdSeconds, 0);
  assert.equal(result.passed, 0);
  result.passed = 7;
  assert.equal(trainer.update(snapshot).passed, 0);
  snapshot = { ...snapshot, time: 10 };
  result = trainer.update(snapshot);
  assert.equal(result.passed, 0);
  assert.equal(result.holdSeconds, 0);
  for (let i = 1; i <= 400; i++) {
    snapshot = { ...snapshot, time: 10 + i * 0.005 };
    result = trainer.update(snapshot);
  }
  assert.equal(result.passed, 1);
  assert.equal(result.holdSeconds, 0);
  const jump = {
    ...snapshot,
    time: snapshot.time + 0.005,
    armed: true,
    status: "flying",
    position: [0, 0, -2],
    ground: { heightM: 0, clearanceM: 2, normalNed: [0, 0, -1] },
  };
  result = trainer.update(jump);
  assert.equal(result.passed, 1);
  assert.equal(result.reason, "procedure-order");
  assert.equal(result.score, 0);
  const landingSim = createSimulation({ startOnGround: true }),
    landingTrainer = createTrainer("tutorial");
  let progress = landingTrainer.update(landingSim.snapshot()),
    handled = -1;
  for (let i = 0; i < 30000 && progress.passed < 6; i++) {
    if (handled !== progress.passed) {
      handled = progress.passed;
      if (handled === 1) landingSim.arm();
      if (handled === 2) landingSim.takeoff();
      if (handled === 4)
        landingSim.setPositionTarget({ north: 8, east: 0, altitude: 2 });
      if (handled === 5)
        landingSim.setPositionTarget({ north: 0, east: 0, altitude: 2 });
    }
    progress = landingTrainer.update(landingSim.step());
  }
  assert.equal(progress.passed, 6);
  const jumped = landingSim.snapshot();
  jumped.position = [0, 0, 0];
  jumped.velocity = [0, 0, 0];
  jumped.ground = { heightM: 0, clearanceM: 0, normalNed: [0, 0, -1] };
  jumped.armed = false;
  jumped.status = "landed";
  jumped.navigation.mode = "land";
  for (let i = 0; i < 500; i++) {
    jumped.time += 0.005;
    progress = landingTrainer.update(jumped);
  }
  assert.equal(progress.passed, 6);
  assert.equal(progress.complete, false);
  assert.equal(progress.holdSeconds, 0);
});
