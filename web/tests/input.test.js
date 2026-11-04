import test from "node:test";
import assert from "node:assert/strict";
import { readPilotInput, takeoffPlayAction } from "../input.js";

test("keyboard controls map to body and climb commands", () => {
  assert.deepEqual(
    readPilotInput(new Set(["KeyW", "KeyD", "ArrowLeft", "ArrowUp"])),
    { roll: 1, pitch: -1, yaw: -1, climb: 1 },
  );
});
test("gamepad axes are clamped, dead-zoned and sanitized", () => {
  assert.deepEqual(readPilotInput(new Set(), { axes: [2, 0.08, NaN, -0.5] }), {
    roll: 1,
    pitch: 0,
    yaw: 0,
    climb: 0.5,
  });
});
test("keyboard overrides each active axis including opposing keys", () => {
  assert.deepEqual(
    readPilotInput(new Set(["KeyA", "KeyD", "ArrowDown"]), {
      axes: [0.5, -0.5, 0.4, -0.5],
    }),
    { roll: 0, pitch: -0.5, yaw: 0.4, climb: -1 },
  );
});

test("ground launch and unfinished takeoff select the correct play action", () => {
  const grounded = {
    status: "landed",
    armed: false,
    velocity: [0, 0, 0],
    runway: { weightOnWheels: true, takeoffPhase: "idle" },
    navigation: { mode: "attitude" },
  };
  assert.equal(takeoffPlayAction(grounded), "launch");
  for (const passed of [0, 1, 2, 7])
    assert.equal(
      takeoffPlayAction({
        ...grounded,
        training: { course: "tutorial", passed },
      }),
      "play",
    );
  assert.equal(
    takeoffPlayAction({
      ...grounded,
      velocity: [3, 0, 0],
      runway: { weightOnWheels: true, takeoffPhase: "aborted" },
    }),
    "play",
  );
  assert.equal(
    takeoffPlayAction({
      ...grounded,
      armed: true,
      navigation: { mode: "land" },
    }),
    "play",
  );
  assert.equal(
    takeoffPlayAction({
      ...grounded,
      velocity: [3, 0, 0],
      runway: { weightOnWheels: true, takeoffPhase: "roll" },
    }),
    "resume",
  );
  assert.equal(takeoffPlayAction({ ...grounded, status: "crashed" }), "play");
  assert.equal(takeoffPlayAction(undefined), "play");
});
