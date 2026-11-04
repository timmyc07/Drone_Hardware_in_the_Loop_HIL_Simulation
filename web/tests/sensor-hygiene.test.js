import test from "node:test";
import assert from "node:assert/strict";
import { createSensorSuite, SENSOR_NAMES } from "../sensors.js";
import { compactSensorFrame, validateSensorFrame } from "../recording.js";

const minimal = {
  lidarRays: 12,
  lidarChannels: 1,
  lidarVerticalMinDeg: 0,
  lidarVerticalMaxDeg: 0,
  radarAzimuthRays: 1,
  radarElevationRays: 1,
};
const state = (time) => ({
  time,
  position: [0, 0, -2],
  velocity: [0, 0, 0],
  quaternion: [1, 0, 0, 0],
  rates: [0, 0, 0],
});
function assertHygiene(frame, config) {
  for (const name of SENSOR_NAMES)
    assert.equal(
      frame.faults[name],
      frame[name].reason === "fault",
      `${name} fault flag must describe its delivered reading`,
    );
  assert.deepEqual(validateSensorFrame(frame, config), frame);
  assert.deepEqual(
    validateSensorFrame(compactSensorFrame(frame), config),
    frame,
  );
}

test("all seven fault flags stay equivalent to delivered fault reasons through immediate changes, latency and recovery", () => {
  for (const latency of [0, 1])
    for (const dropout of [0, 1])
      for (const name of SENSOR_NAMES) {
        const config = { ...minimal, latency, dropout },
          suite = createSensorSuite(config, 42);
        assertHygiene(suite.snapshot(), config);
        assertHygiene(suite.update(state(0)), config);
        suite.setFault(name, true);
        assertHygiene(suite.snapshot(), config);
        for (let i = 1; i <= 220; i++)
          assertHygiene(suite.update(state(i * 0.005)), config);
        suite.setFault(name, false);
        const recovered = suite.snapshot();
        assert.equal(recovered[name].reason, "startup");
        assertHygiene(recovered, config);
        for (let i = 221; i <= 440; i++)
          assertHygiene(suite.update(state(i * 0.005)), config);
        assert.notEqual(suite.snapshot()[name].reason, "fault");
      }
});

test("recording sensor validation rejects contradictory flags and reasons in full and compact carriers", () => {
  for (const name of SENSOR_NAMES) {
    const suite = createSensorSuite(minimal),
      normal = suite.update(state(0));
    for (const compact of [false, true]) {
      const forged = structuredClone(normal);
      forged.faults[name] = true;
      assert.throws(
        () =>
          validateSensorFrame(
            compact ? compactSensorFrame(forged) : forged,
            minimal,
          ),
        undefined,
        `${name}: enabled fault cannot carry ${normal[name].reason || "valid"}`,
      );
    }
    suite.setFault(name, true);
    for (const compact of [false, true]) {
      const forged = suite.snapshot();
      forged.faults[name] = false;
      assert.throws(
        () =>
          validateSensorFrame(
            compact ? compactSensorFrame(forged) : forged,
            minimal,
          ),
        undefined,
        `${name}: fault reason requires its flag`,
      );
    }
  }
});
