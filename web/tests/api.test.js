import test from "node:test";
import assert from "node:assert/strict";
import { createFlightApi } from "../api.js";

test("vehicle operations require and preserve explicit vehicle IDs", async () => {
  const calls = [];
  const api = createFlightApi(
    async (command) => {
      calls.push(command);
      return command;
    },
    () => () => {},
  );
  assert.equal(api.version, "2.6");
  await api.getState("one");
  const target = { altitude: 3 };
  await api.setTargets("two", target);
  target.altitude = 10;
  await api.takeoff("two");
  await api.exportRadarData("two", "csv");
  await api.exportProximityData("one", "csv");
  await api.captureInspection("two", "tower-south");
  await api.getInspectionReport("two");
  await api.getWorldState();
  assert.deepEqual(calls, [
    { type: "inspect", vehicleId: "one" },
    { type: "targets", vehicleId: "two", value: { altitude: 3 } },
    { type: "takeoff", vehicleId: "two" },
    { type: "radarExport", vehicleId: "two", value: "csv" },
    { type: "proximityExport", vehicleId: "one", value: "csv" },
    { type: "captureInspection", vehicleId: "two", value: "tower-south" },
    { type: "inspectionReport", vehicleId: "two" },
    { type: "inspectWorld" },
  ]);
  assert.throws(() => api.getState(), /vehicleId/);
  assert.throws(() => api.takeoff("bad id"), /vehicleId/);
  assert.throws(
    () => api.captureInspection(undefined, "tower-south"),
    /vehicleId/,
  );
  assert.throws(() => api.getInspectionReport(), /vehicleId/);
  assert.throws(() => api.exportProximityData(), /vehicleId/);
});

test("automatic camera uses an explicit source and returns independent session state", async () => {
  const calls = [];
  const status = { active: false, saved: 2 };
  const api = createFlightApi(async (command) => {
    calls.push(command);
    return status;
  }, () => () => {});
  const settings = { mode: "distance", interval: 5, maxFrames: 4 };
  const returned = await api.startAutomaticCamera("one", settings);
  settings.interval = 9;
  returned.saved = 100;
  await api.getAutomaticCamera();
  await api.stopAutomaticCamera();
  assert.deepEqual(calls, [
    { type: "cameraAutoStart", vehicleId: "one", value: { mode: "distance", interval: 5, maxFrames: 4 } },
    { type: "cameraAutoState" },
    { type: "cameraAutoStop" },
  ]);
  assert.equal(status.saved, 2);
  assert.throws(() => api.startAutomaticCamera(undefined, settings), /vehicleId/);
});

test("instructor plans are global commands and isolate submitted plans and returned reports", async () => {
  const calls = [], status = { phase: "armed", receipts: [] };
  const api = createFlightApi(async command => { calls.push(command); return status; }, () => () => {});
  const plan = { name: "教官測試", events: [{ tick: 200, vehicleId: "one", type: "gust" }] };
  const returned = await api.applyInstructorPlan(plan);
  plan.events[0].tick = 999;
  returned.receipts.push({ index: 0 });
  await api.getInstructorStatus();
  await api.cancelInstructorPlan();
  assert.deepEqual(calls, [
    { type: "instructorApply", value: { name: "教官測試", events: [{ tick: 200, vehicleId: "one", type: "gust" }] } },
    { type: "instructorStatus" }, { type: "instructorCancel" },
  ]);
  assert.equal(status.receipts.length, 0);
});

test("subscriptions isolate delivered state and can be removed", () => {
  let listener;
  let removed = false;
  const api = createFlightApi(
    async () => ({}),
    (callback) => {
      listener = callback;
      return () => {
        removed = true;
      };
    },
  );
  let received;
  const unsubscribe = api.subscribe((value) => {
    received = value;
    value.world.tick = 9;
  });
  const state = { world: { tick: 2 } };
  listener(state);
  assert.equal(state.world.tick, 2);
  assert.equal(received.world.tick, 9);
  unsubscribe();
  assert.equal(removed, true);
});
