import test from "node:test";
import assert from "node:assert/strict";
import { readPilotInput } from "../input.js";

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
