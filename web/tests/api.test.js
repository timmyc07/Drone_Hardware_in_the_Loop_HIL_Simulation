import test from "node:test";
import assert from "node:assert/strict";
import { createFlightApi } from "../api.js";

test("API sends the documented commands and copies input and output", async () => {
  const commands = [];
  const response = { snapshot: { position: [1, 2, 3] } };
  const api = createFlightApi(
    async (command) => {
      commands.push(command);
      return response;
    },
    () => () => {},
  );
  assert.equal(api.version, "1.1");
  assert.ok(Object.isFrozen(api));
  const targets = { roll: 1 };
  const pending = api.setTargets(targets);
  targets.roll = 9;
  const result = await pending;
  result.snapshot.position[0] = 99;
  assert.deepEqual(response.snapshot.position, [1, 2, 3]);
  await api.getState();
  await api.getSensors();
  await api.setMode("mission");
  await api.setTransition("cruise");
  await api.setPusherFault(0.5);
  await api.setPositionTarget({ north: 1 });
  await api.setMission([{ north: 2 }]);
  await api.play();
  await api.pause();
  await api.reset({ scenario: "hover" });
  await api.setSensorConfig({ gps: { enabled: true } });
  await api.setSensorFault("gps", true);
  await api.exportRecording();
  assert.deepEqual(commands, [
    { type: "targets", value: { roll: 1 } },
    { type: "inspect" },
    { type: "inspectSensors" },
    { type: "mode", value: "mission" },
    { type: "transition", value: "cruise" },
    { type: "fault", fault: "pusher", value: { efficiency: 0.5 } },
    { type: "position", value: { north: 1 } },
    { type: "mission", value: [{ north: 2 }] },
    { type: "play" },
    { type: "pause" },
    { type: "reset", scenario: "hover" },
    { type: "sensorConfig", value: { gps: { enabled: true } } },
    { type: "sensorFault", value: { sensor: "gps", failed: true } },
    { type: "apiExport" },
  ]);
});

test("rejections propagate and reset rejects unexpected fields", async () => {
  const failure = new Error("rejected");
  const api = createFlightApi(
    async () => {
      throw failure;
    },
    () => () => {},
  );
  await assert.rejects(api.setMode("invalid"), (error) => error === failure);
  await assert.rejects(api.reset({ type: "play" }), TypeError);
  await assert.rejects(api.reset({ extra: true }), TypeError);
  await assert.rejects(api.reset(null), TypeError);
  assert.throws(() => createFlightApi(null, () => {}), TypeError);
  assert.throws(() => createFlightApi(() => {}, null), TypeError);
});

test("subscriptions isolate updates and unsubscribe", () => {
  let listener;
  const api = createFlightApi(
    async () => {},
    (callback) => {
      listener = callback;
      return () => {
        listener = null;
      };
    },
  );
  assert.throws(() => api.subscribe(null), TypeError);
  let received;
  const unsubscribe = api.subscribe((state) => {
    received = state;
    state.position[0] = 99;
  });
  const original = { position: [1, 2, 3] };
  listener(original);
  assert.deepEqual(original.position, [1, 2, 3]);
  assert.equal(received.position[0], 99);
  unsubscribe();
  assert.equal(listener, null);
});
