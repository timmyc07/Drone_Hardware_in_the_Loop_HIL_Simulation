import test from "node:test";
import assert from "node:assert/strict";
import {
  createHilClient,
  isCompanionOrigin,
  hilFlightReadiness,
  shouldFollowTakeoff,
} from "../hil-client.js";

class Socket {
  static OPEN = 1;
  static current;
  readyState = Socket.OPEN;
  constructor(url) {
    this.url = url;
    Socket.current = this;
    queueMicrotask(() => {
      this.onopen();
      this.onmessage({
        data: JSON.stringify({ type: "state", value: { phase: "paused" } }),
      });
    });
  }
  send(text) {
    this.sent = JSON.parse(text);
  }
  close() {
    this.readyState = 3;
    this.onclose?.();
  }
  reply(data) {
    this.onmessage({ data: JSON.stringify(data) });
  }
}
function makeClient(extra = {}) {
  const seen = [];
  const client = createHilClient({
    location: new URL("http://127.0.0.1:8787/"),
    fetch: async () => ({
      ok: true,
      json: async () => ({
        token: "secret",
        wsPath: "/bridge",
        state: { phase: "running" },
      }),
    }),
    WebSocket: Socket,
    onState: (value) => seen.push(value.phase),
    ...extra,
  });
  return { client, seen };
}
test("bootstrap state is followed by immediate socket state without stale overwrite", async () => {
  const { client, seen } = makeClient();
  await client.connect();
  assert.deepEqual(seen, ["running", "paused"]);
  assert.equal(isCompanionOrigin(new URL("https://example.com")), false);
  assert.equal(isCompanionOrigin(new URL("http://localhost:8787")), true);
  client.close();
});
test("errors, malformed messages and socket close reject requests", async () => {
  const { client } = makeClient();
  await client.connect();
  const bad = client.request("inspect");
  Socket.current.onmessage({ data: "{" });
  Socket.current.reply({
    type: "reply",
    id: Socket.current.sent.id,
    ok: false,
    error: "rejected",
  });
  await assert.rejects(bad, /rejected/);
  const pending = client.request("inspect");
  Socket.current.close();
  await assert.rejects(pending, /中斷/);
});
test("request timeouts and invalid bootstrap are rejected", async () => {
  const { client } = makeClient();
  await client.connect();
  const original = globalThis.setTimeout;
  let expire;
  try {
    globalThis.setTimeout = (fn, delay) => {
      assert.equal(delay, 20000);
      expire = fn;
      return 0;
    };
    const request = client.request("inspect");
    expire();
    await assert.rejects(request, /逾時/);
  } finally {
    globalThis.setTimeout = original;
  }
  client.close();
  const bad = makeClient({
    fetch: async () => ({
      ok: true,
      json: async () => ({ token: "x", wsPath: "//remote" }),
    }),
  }).client;
  await assert.rejects(bad.connect(), /無效/);
});
test("reconnecting only reads state and never starts or resumes a session", async () => {
  const first = makeClient().client;
  await first.connect();
  first.close();
  const second = makeClient().client;
  await second.connect();
  assert.equal(Socket.current.sent, undefined);
  second.close();
});

test("ArduPilot readiness requires identity, parameters and heartbeat without changing PX4", () => {
  assert.deepEqual(hilFlightReadiness({ controller: { kind: "ardupilot" } }), [
    "飛控身分",
    "參數",
    "心跳",
  ]);
  assert.deepEqual(
    hilFlightReadiness({
      controller: {
        kind: "ardupilot",
        verified: true,
        parametersVerified: true,
        armedKnown: true,
      },
    }),
    [],
  );
  assert.deepEqual(hilFlightReadiness({ controller: { kind: "px4" } }), []);
});

test("follow view switches only on an unfollowed ground-to-air transition", () => {
  assert.equal(shouldFollowTakeoff(true, false, false), true);
  assert.equal(shouldFollowTakeoff(true, false, true), false);
  assert.equal(shouldFollowTakeoff(false, false, false), false);
  assert.equal(shouldFollowTakeoff(undefined, false, false), false);
});
