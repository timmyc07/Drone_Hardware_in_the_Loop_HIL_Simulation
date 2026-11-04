import test from "node:test";
import assert from "node:assert/strict";
import { MissionTransfer } from "../mission-transfer.js";
const base = { targetSystem: 255, targetComponent: 191, missionType: 0 };
const item = {
  seq: 0,
  frame: 6,
  command: 22,
  autocontinue: 1,
  x: 1,
  y: 2,
  z: 3,
  param1: 0,
  param2: 0,
  param3: 0,
  param4: 0,
  missionType: 0,
};
test("shared transfer returns downloaded items and handles request variants", async () => {
  const sent = [],
    phases = [];
  const t = new MissionTransfer({
    send: (m) => sent.push(m),
    verifyItem: (a, e) => assert.equal(a.x, e.x),
    diagnostic: () => {},
    onPhase: (p) => phases.push(p),
  });
  const result = t.upload([item]);
  assert.equal(t.active, true);
  assert.equal(t.handle(51, { ...base, seq: 0 }), true);
  assert.equal(sent.at(-1).constructor.MSG_ID, 73);
  t.handle(47, { ...base, type: 0 });
  t.handle(44, { ...base, count: 1 });
  t.handle(73, { ...item, ...base });
  assert.equal((await result)[0].x, 1);
  assert.deepEqual(phases, ["uploading", "verifying", "ready"]);
  assert.equal(t.phase, "ready");
  assert.equal(t.active, false);
});
test("shared transfer rejects float packets and abort never throws when callbacks fail", async () => {
  const t = new MissionTransfer({
    send: () => {},
    verifyItem: () => {},
    diagnostic: () => {
      throw Error("diagnostic");
    },
    onPhase: (phase) => {
      if (phase === "error") throw Error("phase");
    },
  });
  const p = t.upload([item]);
  assert.doesNotThrow(() => t.handle(39, base));
  await assert.rejects(p, /Float/);
  assert.equal(t.active, false);
  assert.equal(t.phase, "error");
  assert.doesNotThrow(() => t.abort("again"));
});

test("invalid mission item lists never transmit a count", async () => {
  for (const items of [
    [],
    Array(1),
    [{ seq: 1 }],
    Array.from({ length: 54 }, (_, seq) => ({ seq })),
  ]) {
    const sent = [],
      phases = [];
    const transfer = new MissionTransfer({
      send: (value) => sent.push(value),
      verifyItem: () => {},
      diagnostic: () => {},
      onPhase: (phase) => phases.push(phase),
    });
    await assert.rejects(transfer.upload(items), /dense sequence/);
    assert.deepEqual(sent, []);
    assert.deepEqual(phases, ["error"]);
    assert.equal(transfer.phase, "error");
  }
});
