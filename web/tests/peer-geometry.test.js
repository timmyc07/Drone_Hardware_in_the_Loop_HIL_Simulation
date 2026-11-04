import test from "node:test";
import assert from "node:assert/strict";
import {
  getVehicleEnvelope,
  relativeSweep,
  rayPeerSphere,
} from "../peer-geometry.js";
import { AIRFRAME_PRESETS } from "../sim.js";

test("configuration-derived peer envelopes cover rotor arms and wing tips without changing motor mappings", () => {
  for (const [airframe, config] of Object.entries(AIRFRAME_PRESETS)) {
    const envelope = getVehicleEnvelope(config);
    assert.ok(envelope.radius >= config.arm + 0.15);
    if (["fixedwing", "vtol"].includes(airframe))
      assert.ok(envelope.radius >= config.wingSpan / 2);
    assert.ok(Object.isFrozen(envelope));
  }
  assert.ok(
    getVehicleEnvelope({ ...AIRFRAME_PRESETS.fixedwing, wingSpan: 3 }).radius >
      getVehicleEnvelope(AIRFRAME_PRESETS.fixedwing).radius,
  );
});
test("relative swept spheres catch crossing paths, tangent contact and initial overlap, but not separated flybys", () => {
  assert.equal(
    relativeSweep([-10, 0, 0], [10, 0, 0], 0.5, [10, 0, 0], [-10, 0, 0], 0.5),
    0.475,
  );
  assert.equal(
    relativeSweep([-2, 1, 0], [2, 1, 0], 0.5, [0, 0, 0], [0, 0, 0], 0.5),
    0.5,
  );
  assert.equal(
    relativeSweep([0, 0, 0], [1, 0, 0], 0.5, [0.5, 0, 0], [2, 0, 0], 0.5),
    0,
  );
  assert.equal(
    relativeSweep([-10, 0, 2], [10, 0, 2], 0.5, [10, 0, 0], [-10, 0, 0], 0.5),
    null,
  );
  assert.equal(
    relativeSweep([0, 0, 0], [0, 0, 0], 0.5, [3, 0, 0], [3, 0, 0], 0.5),
    null,
  );
});
test("peer ray uses nearest front surface and explicit inside/blind-zone semantics", () => {
  const peer = { position: [5, 0, 0], radius: 1 };
  assert.equal(rayPeerSphere([0, 0, 0], [1, 0, 0], peer, 0, 20), 4);
  assert.equal(rayPeerSphere([0, 0, 0], [-1, 0, 0], peer, 0, 20), null);
  assert.equal(rayPeerSphere([0, 0, 0], [1, 0, 0], peer, 0, 3), null);
  assert.equal(rayPeerSphere([5, 0, 0], [1, 0, 0], peer, 0, 20), 0);
  assert.throws(() => rayPeerSphere([0, 0, 0], [2, 0, 0], peer, 0, 20), /ray/);
});
