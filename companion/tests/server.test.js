import test from "node:test";
import assert from "node:assert/strict";
import { EventEmitter, once } from "node:events";
import { request } from "node:http";
import WebSocket from "ws";
import { startCompanion } from "../server.js";

class FakeSession extends EventEmitter {
  actions = [];
  record = { samples: [1, 2, 3] };
  state() {
    return { running: true };
  }
  async start(v) {
    this.actions.push(["start", v]);
    return this.state();
  }
  async command(v) {
    this.actions.push(v);
    if (v === "arm") throw Error("no arm");
    return this.state();
  }
  exportRecording() {
    return this.record;
  }
  async close() {
    this.closed = true;
  }
}
const http = (port, path, headers = {}) =>
  new Promise((resolve, reject) => {
    const req = request({ host: "127.0.0.1", port, path, headers }, (res) => {
      let data = "";
      res.on("data", (v) => (data += v));
      res.on("end", () => resolve({ status: res.statusCode, data }));
    });
    req.on("error", reject);
    req.end();
  });
const wsOpen = (url, origin, headers = {}) =>
  new Promise((resolve, reject) => {
    const ws = new WebSocket(url, { origin, headers });
    ws.once("open", () => resolve(ws));
    ws.once("unexpected-response", (_req, res) =>
      reject(Error(String(res.statusCode))),
    );
    ws.once("error", reject);
  });

test("loopback token, origin, single controller, validation and disconnect preservation", async (t) => {
  const session = new FakeSession();
  const app = await startCompanion({ port: 0, sessionFactory: () => session });
  t.after(() => app.close());
  const { port } = app.address,
    host = `127.0.0.1:${port}`,
    origin = `http://${host}`;
  assert.equal(
    (await http(port, "/api/bootstrap", { host: "evil.test" })).status,
    403,
  );
  assert.equal(
    (await http(port, "/api/bootstrap", { origin: "https://evil.test" }))
      .status,
    403,
  );
  assert.equal(
    (await http(port, "/api/bootstrap", { "sec-fetch-site": "cross-site" }))
      .status,
    403,
  );
  assert.equal((await http(port, "/api/recording")).status, 403);
  assert.notEqual((await http(port, "/%2e%2e%2f.env")).status, 200);
  const bootstrap = JSON.parse((await http(port, "/api/bootstrap")).data),
    url = `ws://${host}/bridge?token=${bootstrap.token}`;
  await assert.rejects(wsOpen(url, "https://evil.test"), /403/);
  await assert.rejects(wsOpen(`ws://${host}/bridge?token=bad`, origin), /403/);
  await assert.rejects(wsOpen(url, origin, { host: "evil.test" }), /403/);
  const ws = await wsOpen(url, origin);
  t.after(() => ws.terminate());
  await assert.rejects(wsOpen(url, origin), /409/);
  const replies = [];
  ws.on("message", (data) => {
    const v = JSON.parse(data);
    if (v.type === "reply") replies.push(v);
  });
  ws.send(JSON.stringify({ id: 1, type: "command", value: "arm", extra: 1 }));
  ws.send(JSON.stringify({ id: 2, type: "command", value: "arm" }));
  ws.send(JSON.stringify({ id: 3, type: "inspect" }));
  while (replies.length < 3) await new Promise((r) => setTimeout(r, 5));
  assert.equal(replies[0].ok, false);
  assert.equal(replies[1].error, "no arm");
  assert.equal(replies[2].ok, true);
  assert.deepEqual(session.actions, ["arm"]);
  assert.deepEqual(
    JSON.parse(
      (
        await http(port, "/api/recording", {
          "x-flight-token": bootstrap.token,
        })
      ).data,
    ),
    session.record,
  );
  const closed = once(ws, "close");
  ws.close();
  await closed;
  while (!session.actions.includes("pause"))
    await new Promise((r) => setTimeout(r, 5));
  assert.deepEqual(session.record.samples, [1, 2, 3]);
});

test("oversized incoming messages disconnect and pause", async (t) => {
  const session = new FakeSession(),
    app = await startCompanion({ port: 0, sessionFactory: () => session });
  t.after(() => app.close());
  const { port } = app.address,
    host = `127.0.0.1:${port}`,
    token = JSON.parse((await http(port, "/api/bootstrap")).data).token;
  const ws = await wsOpen(
    `ws://${host}/bridge?token=${token}`,
    `http://${host}`,
  );
  const closed = once(ws, "close");
  ws.send("x".repeat(65537));
  await closed;
  while (!session.actions.includes("pause"))
    await new Promise((r) => setTimeout(r, 5));
  assert.deepEqual(session.actions, ["pause"]);
});

test("bounded command queue drops commands after client disconnects", async (t) => {
  let release, started;
  const waiting = new Promise((resolve) => {
    release = resolve;
  });
  const entered = new Promise((resolve) => {
    started = resolve;
  });
  const session = new FakeSession();
  session.start = async () => {
    session.actions.push("start");
    started();
    await waiting;
    return session.state();
  };
  const app = await startCompanion({ port: 0, sessionFactory: () => session });
  t.after(() => app.close());
  const { port } = app.address,
    host = `127.0.0.1:${port}`,
    token = JSON.parse((await http(port, "/api/bootstrap")).data).token;
  const ws = await wsOpen(
    `ws://${host}/bridge?token=${token}`,
    `http://${host}`,
  );
  ws.send(
    JSON.stringify({
      id: 1,
      type: "start",
      value: { binaryPath: "/test", config: {} },
    }),
  );
  await entered;
  for (let i = 2; i < 40; i++)
    ws.send(JSON.stringify({ id: i, type: "command", value: "resume" }));
  const [code] = await once(ws, "close");
  assert.equal(code, 1008);
  release();
  while (!session.actions.includes("pause"))
    await new Promise((r) => setTimeout(r, 5));
  assert.deepEqual(session.actions, ["start", "pause"]);
});

test("failed listen closes session; session close failure still closes HTTP server", async () => {
  const first = await startCompanion({
    port: 0,
    sessionFactory: () => new FakeSession(),
  });
  const failed = new FakeSession();
  await assert.rejects(
    startCompanion({ port: first.address.port, sessionFactory: () => failed }),
  );
  assert.equal(failed.closed, true);
  await first.close();
  const bad = new FakeSession();
  bad.close = async () => {
    throw Error("close failed");
  };
  const second = await startCompanion({ port: 0, sessionFactory: () => bad });
  await assert.rejects(second.close(), /close failed/);
  assert.equal(second.server.listening, false);
});

test("CLI signals close the owned session exactly once", async () => {
  const { runCompanionCli } = await import("../server.js");
  const signals = new EventEmitter();
  let closes = 0;
  await runCompanionCli(
    async () => ({
      address: { port: 1234 },
      close: async () => {
        closes++;
      },
    }),
    signals,
  );
  signals.emit("SIGTERM");
  signals.emit("SIGINT");
  await new Promise((resolve) => setImmediate(resolve));
  assert.equal(closes, 1);
});

test("restart preserves stopped recording on failure and swaps only after success", async (t) => {
  const sessions = [];
  class RestartSession extends FakeSession {
    phase = "idle";
    state() {
      return { phase: this.phase, number: sessions.indexOf(this) };
    }
    async start({ binaryPath }) {
      if (binaryPath === "invalid") throw Error("invalid binary");
      this.phase = "starting";
      return this.state();
    }
    async command(action) {
      if (action === "stop") {
        this.phase = "stopped";
        this.emit("state", this.state());
      }
      return this.state();
    }
    exportRecording() {
      return { number: sessions.indexOf(this) };
    }
    async close() {
      this.phase = "stopped";
    }
  }
  const app = await startCompanion({
    port: 0,
    sessionFactory: () => {
      const s = new RestartSession();
      sessions.push(s);
      return s;
    },
  });
  t.after(() => app.close());
  const { port } = app.address,
    host = `127.0.0.1:${port}`,
    token = JSON.parse((await http(port, "/api/bootstrap")).data).token;
  const ws = await wsOpen(
    `ws://${host}/bridge?token=${token}`,
    `http://${host}`,
  );
  t.after(() => ws.terminate());
  const messages = [];
  ws.on("message", (data) => messages.push(JSON.parse(data)));
  const command = async (data) => {
    ws.send(JSON.stringify(data));
    while (!messages.some((m) => m.type === "reply" && m.id === data.id))
      await new Promise((r) => setTimeout(r, 5));
    return messages.find((m) => m.type === "reply" && m.id === data.id);
  };
  await command({
    id: 1,
    type: "start",
    value: { binaryPath: "valid", config: {} },
  });
  await command({ id: 2, type: "command", value: "stop" });
  assert.deepEqual(
    JSON.parse(
      (await http(port, "/api/recording", { "x-flight-token": token })).data,
    ),
    { number: 0 },
  );
  assert.equal(
    (
      await command({
        id: 3,
        type: "start",
        value: { binaryPath: "invalid", config: {} },
      })
    ).ok,
    false,
  );
  assert.deepEqual(
    JSON.parse(
      (await http(port, "/api/recording", { "x-flight-token": token })).data,
    ),
    { number: 0 },
  );
  assert.equal(
    (
      await command({
        id: 4,
        type: "start",
        value: { binaryPath: "valid", config: {} },
      })
    ).ok,
    true,
  );
  assert.deepEqual(
    JSON.parse(
      (await http(port, "/api/recording", { "x-flight-token": token })).data,
    ),
    { number: 2 },
  );
  assert.equal(
    messages.filter((m) => m.type === "state" && m.value.number === 2).length,
    1,
  );
  sessions[0].emit("state", { phase: "stopped", number: 0 });
  await new Promise((r) => setTimeout(r, 5));
  assert.equal(
    messages.filter(
      (m) =>
        m.type === "state" &&
        m.value.number === 0 &&
        m.value.phase === "stopped",
    ).length,
    1,
  );
});
