import test from "node:test";
import assert from "node:assert/strict";
import { common, minimal, MavLinkProtocolV2 } from "node-mavlink";
import { ControllerHealth } from "../controller-health.js";
import { decodeDatagram } from "../mavlink.js";
const PREARM = 0x10000000;
const flags = { verified: true, live: true };
const heartbeat = (kind = "px4", armed = false) => ({
  autopilot: kind === "px4" ? 12 : 3,
  baseMode: armed ? 128 : 0,
});
const status = (present = 0, enabled = 0, health = 0) => ({
  onboardControlSensorsPresent: present,
  onboardControlSensorsEnabled: enabled,
  onboardControlSensorsHealth: health,
});
function fixture(kind = "px4") {
  let now = 0;
  const cache = new ControllerHealth(kind, () => now);
  return {
    cache,
    advance(value) {
      now = value;
    },
    snapshot: () => cache.snapshot(flags),
  };
}
function wire(message) {
  return decodeDatagram(new MavLinkProtocolV2(1, 1).serialize(message, 0))[0]
    .message;
}

test("clock zero is a real receipt; pinned PX4 prearm uses health bit without present/enabled", () => {
  const f = fixture();
  assert.deepEqual(f.snapshot(), {
    sysStatus: null,
    heartbeatAgeMs: null,
    prearm: "unknown",
    messages: [],
  });
  f.cache.update(0, heartbeat());
  f.cache.update(1, status(0, 0, PREARM));
  assert.equal(f.snapshot().heartbeatAgeMs, 0);
  assert.equal(f.snapshot().sysStatus.ageMs, 0);
  assert.equal(f.snapshot().prearm, "reported-ok");
  f.cache.update(1, status());
  assert.equal(f.snapshot().prearm, "failed");
});
test("ArduPilot disabled checks are not a pass, and armed is solely a fresh heartbeat fact", () => {
  const f = fixture("ardupilot");
  f.cache.update(0, heartbeat("ardupilot"));
  f.cache.update(1, status(PREARM, 0, PREARM));
  assert.equal(f.snapshot().prearm, "checks-disabled");
  f.cache.update(1, status(PREARM, PREARM, 0));
  assert.equal(f.snapshot().prearm, "failed");
  f.cache.update(1, status(PREARM, PREARM, PREARM));
  assert.equal(f.snapshot().prearm, "reported-ok");
  const armed = fixture("ardupilot");
  armed.cache.update(0, heartbeat("ardupilot", true));
  assert.equal(armed.snapshot().prearm, "armed");
  assert.equal(armed.snapshot().sysStatus, null);
  armed.advance(5001);
  assert.equal(armed.snapshot().prearm, "unknown");
});
test("every prearm conclusion requires live verified fresh heartbeat; non-armed conclusions also require fresh status", () => {
  const f = fixture();
  f.cache.update(1, status(0, 0, PREARM));
  assert.equal(f.snapshot().prearm, "unknown");
  f.cache.update(0, heartbeat());
  for (const options of [
    {},
    { verified: true, live: false },
    { verified: false, live: true },
    { verified: "true", live: true },
  ])
    assert.equal(f.cache.snapshot(options).prearm, "unknown");
  f.advance(5000);
  assert.equal(f.snapshot().prearm, "reported-ok");
  f.advance(5001);
  assert.equal(f.snapshot().prearm, "unknown");
  f.cache.update(0, heartbeat());
  assert.equal(f.snapshot().prearm, "unknown");
  f.cache.update(1, status(0, 0, PREARM));
  assert.equal(f.snapshot().prearm, "reported-ok");
});
test("real passive RC masks keep enabled-but-not-present failures; uint32 bit31 never becomes negative", () => {
  const f = fixture();
  f.cache.update(0, heartbeat());
  f.cache.update(1, status(50380847, 50446383, 320915519));
  assert.equal(f.snapshot().sysStatus.unhealthyMask, 65536);
  assert.equal(f.snapshot().prearm, "reported-ok");
  f.cache.update(
    1,
    wire(
      Object.assign(new common.SysStatus(), status(0x80000000, 0x80000000, 0)),
    ),
  );
  assert.equal(f.snapshot().sysStatus.present, 2147483648);
  assert.equal(f.snapshot().sysStatus.unhealthyMask, 2147483648);
  f.cache.update(1, status(0xffffffff, 0xffffffff, 0xffffffff));
  assert.equal(f.snapshot().sysStatus.health, 4294967295);
  assert.equal(f.snapshot().sysStatus.unhealthyMask, 0);
});
test("wrong controller identity and malformed diagnostics are ignored without touching old state", () => {
  const f = fixture();
  f.cache.update(0, heartbeat());
  f.cache.update(1, status(1, 1, 1));
  const before = f.snapshot();
  for (const [id, message] of [
    [0, heartbeat("ardupilot")],
    [0, { autopilot: 12, baseMode: NaN }],
    [1, status(-1, 0, 0)],
    [1, status(0, 2 ** 32, 0)],
    [1, status(0, 0, 1.1)],
    [253, { severity: 8, text: "x", id: 0, chunkSeq: 0 }],
    [253, { severity: 0, text: "x".repeat(51), id: 0, chunkSeq: 0 }],
    [253, { severity: 0, text: "非wire文字", id: 0, chunkSeq: 0 }],
    [2, { timeUnixUsec: 10n }],
    [253, null],
    [1, []],
  ])
    assert.equal(f.cache.update(id, message), false);
  assert.equal(
    f.cache.update(
      1,
      new Proxy(
        {},
        {
          get() {
            throw Error("bad getter");
          },
        },
      ),
    ),
    false,
  );
  assert.deepEqual(f.snapshot(), before);
});
test("the latest 20 text payloads are bounded raw fragments and independent snapshot clones", () => {
  const f = fixture();
  for (let i = 0; i < 25; i++) {
    f.advance(i);
    f.cache.update(253, {
      severity: i % 8,
      text: `packet ${i}`,
      id: 7,
      chunkSeq: i,
    });
  }
  f.advance(100);
  const result = f.snapshot();
  assert.equal(result.messages.length, 20);
  assert.equal(result.messages[0].chunkSeq, 5);
  assert.equal(result.messages[0].text, "packet 5");
  assert.equal(result.messages[0].ageMs, 95);
  assert.ok(result.messages.every((m) => m.chunked));
  f.cache.update(1, status(1, 2, 3));
  const copy = f.snapshot();
  copy.sysStatus.present = 99;
  copy.messages[0].text = "changed";
  copy.messages.length = 0;
  assert.equal(f.snapshot().sysStatus.present, 1);
  assert.equal(f.snapshot().messages[0].text, "packet 5");
});
test("true pinned Latin1 decoding restores complete UTF8 while split or invalid UTF8 stays raw with no invented text", () => {
  const f = fixture(),
    send = (bytes, id = 0, chunkSeq = 0) =>
      f.cache.update(
        253,
        wire(
          Object.assign(new common.StatusText(), {
            severity: 4,
            text: Buffer.from(bytes).toString("latin1"),
            id,
            chunkSeq,
          }),
        ),
      );
  send(Buffer.from("需要定位", "utf8"));
  assert.equal(f.snapshot().messages.at(-1).text, "需要定位");
  assert.equal(
    f.snapshot().messages.at(-1).rawBytesHex,
    Buffer.from("需要定位").toString("hex"),
  );
  assert.equal(f.snapshot().messages.at(-1).chunked, false);
  const bytes = Buffer.from("A".repeat(49) + "需", "utf8");
  send(bytes.subarray(0, 50), 9, 0);
  send(bytes.subarray(50), 9, 1);
  const fragments = f.snapshot().messages.slice(-2);
  assert.ok(fragments.every((m) => m.text === null && m.chunked));
  assert.equal(fragments[0].rawBytesHex.length, 100);
  assert.equal(fragments[1].rawBytesHex, bytes.subarray(50).toString("hex"));
  send(Buffer.from([0xff]));
  assert.equal(f.snapshot().messages.at(-1).text, null);
  send(Buffer.from("next ASCII"));
  assert.equal(f.snapshot().messages.at(-1).text, "next ASCII");
});
test("NUL cuts a payload per protocol and display cleanup retains original byte evidence", () => {
  const f = fixture();
  f.cache.update(253, {
    severity: 0,
    text: "bad\n\x1b[31m\0ignored",
    id: 3,
    chunkSeq: 0,
  });
  const message = f.snapshot().messages[0];
  assert.equal(message.text, "bad  [31m");
  assert.equal(
    message.rawBytesHex,
    Buffer.from("bad\n\x1b[31m").toString("hex"),
  );
  assert.equal(message.chunked, true);
});
