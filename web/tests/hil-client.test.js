import test from "node:test";
import assert from "node:assert/strict";
import { createHilClient, isCompanionOrigin } from "../hil-client.js";

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
    timeoutMs: 20,
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
  await assert.rejects(client.request("inspect"), /逾時/);
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
