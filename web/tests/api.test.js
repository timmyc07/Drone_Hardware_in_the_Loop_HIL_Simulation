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
  assert.equal(api.version, "2.2");
  await api.getState("one");
  const target = { altitude: 3 };
  await api.setTargets("two", target);
  target.altitude = 10;
  await api.takeoff("two");
  await api.exportRadarData("two", "csv");
  await api.captureInspection("two", "tower-south");
  await api.getInspectionReport("two");
  await api.getWorldState();
  assert.deepEqual(calls, [
    { type: "inspect", vehicleId: "one" },
    { type: "targets", vehicleId: "two", value: { altitude: 3 } },
    { type: "takeoff", vehicleId: "two" },
    { type: "radarExport", vehicleId: "two", value: "csv" },
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
