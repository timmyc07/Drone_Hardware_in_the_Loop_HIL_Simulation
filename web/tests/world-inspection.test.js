import test from "node:test";
import assert from "node:assert/strict";
import {
  createWorld,
  defaultWorldConfig,
  inspectionSummary,
} from "../world.js";
import { DEFAULT_CAMERA_CONFIG, cameraMetadata } from "../camera.js";
import { DEFAULT_GIMBAL_SETTINGS } from "../gimbal.js";
import { DEFAULT_SCENE } from "../scene-data.js";
import { AIRFRAME_PRESETS } from "../sim.js";

const uuid = (n) => `00000000-0000-4000-8000-${String(n).padStart(12, "0")}`;
const value = (n = 1, overrides = {}) => ({
  captureId: uuid(n),
  targetId: "panel",
  config: DEFAULT_CAMERA_CONFIG,
  sha256: "a".repeat(64),
  ...overrides,
});
function fixture(peer = null) {
  const scene = structuredClone(DEFAULT_SCENE);
  scene.obstacles = [{ north: 10, east: 0, width: 2, depth: 4, height: 4 }];
  scene.inspectionTargets = [
    {
      id: "panel",
      name: "Panel",
      obstacleIndex: 0,
      face: "south",
      u: 0,
      v: 0,
      width: 1,
      height: 1,
      minRangeM: 1,
      maxRangeM: 30,
      maxAngleDeg: 60,
      minPixels: 10,
      maxSpeedMps: 2,
    },
  ];
  const input = structuredClone(
    defaultWorldConfig(
      {},
      {
        gimbal: {
          ...DEFAULT_GIMBAL_SETTINGS,
          target: { yaw: 0, pitch: 0, roll: 0 },
        },
        sensors: {
          lidarRays: 12,
          lidarChannels: 1,
          lidarVerticalMinDeg: 0,
          lidarVerticalMaxDeg: 0,
          radarAzimuthRays: 1,
          radarElevationRays: 1,
        },
      },
      scene,
    ),
  );
  if (peer) {
    const other = structuredClone(input.vehicles[0]);
    other.id = "drone-2";
    other.name = "Peer";
    other.spawn.north = peer[0];
    other.spawn.east = peer[1];
    other.home = [...peer];
    input.vehicles.push(other);
  }
  return input;
}
const summary = (world, id = "drone-1") =>
  world.getVehicle(id).snapshot.inspection;

test("world derives per-vehicle inspection receipts from current actual camera pose without altering flight or training", () => {
  const world = createWorld(fixture([0, 5]));
  const before = world.snapshot();
  const command = value();
  world.command("drone-1", "inspection", command);
  const after = world.snapshot();
  assert.equal(after.tick, before.tick);
  assert.equal(after.time, before.time);
  for (const key of Object.keys(before.vehicles[0].snapshot).filter(
    (key) => key !== "inspection",
  ))
    assert.deepEqual(
      after.vehicles[0].snapshot[key],
      before.vehicles[0].snapshot[key],
      key,
    );
  assert.deepEqual(after.vehicles[1], before.vehicles[1]);
  assert.deepEqual(after.vehicles[0].inspection, summary(world));
  assert.equal(summary(world).passed, 1);
  assert.equal(summary(world).complete, true);
  const report = world.inspectionReport("drone-1");
  assert.equal(report.format, "flight-lab-inspection");
  assert.equal(report.version, 1);
  assert.deepEqual(report.summary, {
    passed: 1,
    total: 1,
    attempts: 1,
    complete: true,
  });
  const receipt = report.receipts[0];
  assert.deepEqual(
    receipt.metadata,
    cameraMetadata(
      before.vehicles[0].snapshot,
      command.config,
      before.vehicles[0].snapshot.geography ?? null,
      {
        sourceVehicleId: "drone-1",
        worldTick: 0,
        captureId: command.captureId,
      },
    ),
  );
  assert.equal(receipt.sourceVehicleId, "drone-1");
  assert.equal(receipt.worldTick, 0);
  assert.equal(receipt.assessment.visibleSamples, 25);
  report.receipts[0].assessment.passed = false;
  report.scene.obstacles.length = 0;
  after.vehicles[0].inspection.latest.assessment.passed = false;
  assert.equal(summary(world).passed, 1);
  assert.equal(world.inspectionReport("drone-1").scene.obstacles.length, 1);
  world.command("drone-1", "inspection", value(2));
  assert.equal(summary(world).passed, 1);
  assert.equal(summary(world).attempts, 2);
});

test("world inspection uses current peers and actual gimbal motion instead of a requested future target", () => {
  const blocked = createWorld(fixture([4, 0]));
  blocked.command("drone-1", "inspection", value());
  assert.equal(summary(blocked).passed, 0);
  assert.ok(summary(blocked).latest.assessment.reasons.includes("occluded"));
  const world = createWorld(fixture());
  world.command("drone-1", "gimbal", {
    ...DEFAULT_GIMBAL_SETTINGS,
    target: { yaw: Math.PI / 2, pitch: 0, roll: 0 },
  });
  world.command("drone-1", "inspection", value());
  assert.equal(summary(world).passed, 1);
  assert.equal(
    world.inspectionReport("drone-1").receipts[0].metadata.gimbal.actual.yaw,
    0,
  );
  for (let i = 0; i < 210; i++) world.step();
  world.command("drone-1", "inspection", value(2));
  const latest = summary(world).latest.assessment;
  assert.equal(latest.passed, false);
  assert.ok(latest.reasons.includes("frustum"));
  assert.equal(summary(world).passed, 1);
  assert.equal(summary(world).attempts, 2);
});

test("inspection trust boundary rejects malformed values, global duplicate IDs and capped histories atomically", () => {
  const world = createWorld(fixture([0, 5]));
  for (const input of [
    null,
    [],
    new Date(),
    { ...value(), score: 1000 },
    value(1, { captureId: "00000000-0000-1000-8000-000000000001" }),
    value(1, { captureId: uuid(1).toUpperCase().replace("4000", "4A00") }),
    value(1, { sha256: "A".repeat(64) }),
    value(1, { targetId: "missing" }),
    value(1, { config: { ...DEFAULT_CAMERA_CONFIG, imaginary: 1 } }),
  ]) {
    const before = world.snapshot();
    const report = world.inspectionReport("drone-1");
    assert.throws(() => world.command("drone-1", "inspection", input));
    assert.deepEqual(world.snapshot(), before);
    assert.deepEqual(world.inspectionReport("drone-1"), report);
  }
  world.command("drone-1", "inspection", value());
  const before = world.snapshot();
  assert.throws(
    () => world.command("drone-2", "inspection", value()),
    /already/,
  );
  assert.deepEqual(world.snapshot(), before);
  for (let n = 2; n <= 100; n++)
    world.command("drone-1", "inspection", value(n));
  const full = world.snapshot();
  assert.throws(
    () => world.command("drone-1", "inspection", value(101)),
    /limit/,
  );
  assert.deepEqual(world.snapshot(), full);
  assert.equal(summary(world).attempts, 100);
});

test("scene resets and opaque world rollback preserve inspection transaction consistency and deterministic replay", () => {
  const input = fixture();
  const world = createWorld(input),
    replay = createWorld(input);
  const initial = world.checkpoint();
  world.command("drone-1", "inspection", value());
  initial();
  assert.equal(summary(world).attempts, 0);
  world.command("drone-1", "inspection", value());
  replay.command("drone-1", "inspection", value());
  for (let i = 0; i < 25; i++) {
    world.step();
    replay.step();
  }
  assert.deepEqual(world.snapshot(), replay.snapshot());
  const restore = world.checkpoint(),
    report = world.inspectionReport("drone-1");
  const scene = structuredClone(world.configuration.scene);
  scene.environment.fog = 0.5;
  world.setScene(scene);
  assert.equal(summary(world).attempts, 0);
  assert.equal(world.inspectionReport("drone-1").receipts.length, 0);
  restore();
  assert.deepEqual(world.inspectionReport("drone-1"), report);
  assert.throws(
    () => world.command("drone-1", "inspection", value()),
    /already/,
  );
  const invalid = structuredClone(world.configuration.scene);
  invalid.obstacles = [];
  assert.throws(() => world.setScene(invalid));
  assert.deepEqual(world.inspectionReport("drone-1"), report);
  world.setScene({ ...world.configuration.scene, inspectionTargets: [] });
  assert.equal(summary(world), null);
  assert.deepEqual(world.inspectionReport("drone-1").summary, {
    passed: 0,
    total: 0,
    attempts: 0,
    complete: false,
  });
  assert.deepEqual(inspectionSummary([], []), {
    passed: 0,
    total: 0,
    attempts: 0,
    complete: false,
    latest: null,
  });
});

// Fixed-wing captures remain fly-by observations: speed is assessed, not suppressed.
test("all five airframes share canonical inspection geometry and retain their genuine flight state", () => {
  for (const [airframe, config] of Object.entries(AIRFRAME_PRESETS)) {
    const input = fixture();
    input.scene.obstacles[0].height = 60;
    input.scene.inspectionTargets[0].maxSpeedMps = 40;
    const spec = structuredClone(
      defaultWorldConfig(config, input.vehicles[0].setup, input.scene)
        .vehicles[0],
    );
    spec.spawn.altitude = 30;
    input.vehicles = [spec];
    const world = createWorld(input),
      before = world.getVehicle(spec.id).snapshot;
    world.command(spec.id, "inspection", value());
    assert.equal(summary(world).complete, true, airframe);
    const report = world.inspectionReport(spec.id);
    assert.equal(report.receipts[0].metadata.airframe, airframe);
    assert.equal(
      report.receipts[0].assessment.speedMps,
      Math.hypot(...before.velocity),
    );
    assert.deepEqual(
      world.getVehicle(spec.id).snapshot.position,
      before.position,
    );
    assert.deepEqual(
      world.getVehicle(spec.id).snapshot.velocity,
      before.velocity,
    );
  }
});
