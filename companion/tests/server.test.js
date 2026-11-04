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

test("ground station configuration is strict and ownership blocks queued web flight commands", async (t) => {
  class GroundSession extends FakeSession {
    owner = "web";
    state() {
      return { phase: "running", groundStation: { owner: this.owner } };
    }
    async configureGroundStation(value) {
      this.actions.push(["groundStation", value]);
      this.owner = value.owner;
      return this.state();
    }
    async uploadMission(value) {
      this.actions.push(["mission", value]);
    }
  }
  const session = new GroundSession();
  const app = await startCompanion({ port: 0, sessionFactory: () => session });
  t.after(() => app.close());
  const { port } = app.address,
    host = `127.0.0.1:${port}`;
  const token = JSON.parse((await http(port, "/api/bootstrap")).data).token;
  const ws = await wsOpen(
    `ws://${host}/bridge?token=${token}`,
    `http://${host}`,
  );
  t.after(() => ws.terminate());
  let sequence = 0;
  const rpc = (type, value) =>
    new Promise((resolve) => {
      const id = ++sequence;
      const receive = (data) => {
        const message = JSON.parse(data);
        if (message.type === "reply" && message.id === id) {
          ws.off("message", receive);
          resolve(message);
        }
      };
      ws.on("message", receive);
      ws.send(JSON.stringify({ id, type, value }));
    });
  const settings = { enabled: true, targetPort: 14550, owner: "web" };
  for (const invalid of [
    null,
    {},
    { ...settings, extra: 0 },
    { ...settings, targetPort: 0 },
    { ...settings, targetPort: 1024.5 },
    { ...settings, targetPort: 65536 },
    { ...settings, enabled: 1 },
    { ...settings, owner: "other" },
    { ...settings, enabled: false, owner: "gcs" },
  ])
    assert.equal((await rpc("groundStation", invalid)).ok, false);
  assert.equal(session.actions.length, 0);
  assert.equal((await rpc("groundStation", settings)).ok, true);
  assert.equal(
    (await rpc("groundStation", { ...settings, owner: "gcs" })).ok,
    true,
  );
  const before = session.actions.length;
  for (const action of ["arm", "takeoff", "land", "disarm", "mission"])
    assert.equal((await rpc("command", action)).ok, false);
  assert.equal(
    (await rpc("mission", [{ north: 0, east: 0, altitude: 3, hold: 1 }])).ok,
    false,
  );
  assert.equal(session.actions.length, before);
  for (const action of ["pause", "resume", "stop"])
    assert.equal((await rpc("command", action)).ok, true);
  assert.equal((await rpc("groundStation", settings)).ok, true);
  assert.equal((await rpc("command", "takeoff")).ok, true);
});

test(
  "mission uploads reach the selected controller and malformed payloads never reach either",
  { timeout: 5000 },
  async (t) => {
    const sessions = [];
    class MissionSession extends FakeSession {
      constructor(kind) {
        super();
        this.kind = kind;
        sessions.push(this);
      }
      state() {
        return { phase: "idle", controller: { kind: this.kind } };
      }
      async uploadMission(points) {
        this.actions.push(["mission", points]);
        return { verified: true, points };
      }
    }
    const app = await startCompanion({
      port: 0,
      sessionFactory: (kind) => new MissionSession(kind),
    });
    t.after(() => app.close());
    const { port } = app.address,
      host = `127.0.0.1:${port}`;
    const token = JSON.parse((await http(port, "/api/bootstrap")).data).token;
    const ws = await wsOpen(
      `ws://${host}/bridge?token=${token}`,
      `http://${host}`,
    );
    t.after(() => ws.terminate());
    let sequence = 0;
    const rpc = (type, value) =>
      new Promise((resolve) => {
        const id = ++sequence;
        const receive = (data) => {
          const message = JSON.parse(data);
          if (message.type !== "reply" || message.id !== id) return;
          ws.off("message", receive);
          resolve(message);
        };
        ws.on("message", receive);
        ws.send(JSON.stringify({ id, type, value }));
      });
    const points = [{ north: 3, east: -2, altitude: 3, hold: 1 }];
    for (const controller of ["px4", "ardupilot"]) {
      assert.equal(
        (
          await rpc("start", {
            controller,
            binaryPath: "valid",
            config: {},
          })
        ).ok,
        true,
      );
      const selected = sessions.at(-1),
        previousCalls = selected.actions.length;
      for (const invalid of [
        [],
        Array(51).fill(points[0]),
        {},
        [null],
        [{ ...points[0], arbitrary: 0 }],
        [{ north: 0, east: 0, altitude: 2 }],
        [{ ...points[0], north: "3" }],
        [{ ...points[0], altitude: null }],
      ])
        assert.equal((await rpc("mission", invalid)).ok, false);
      assert.equal(selected.actions.length, previousCalls);
      assert.deepEqual((await rpc("mission", points)).value, {
        verified: true,
        points,
      });
      assert.deepEqual(selected.actions.at(-1), ["mission", points]);
      assert.equal((await rpc("command", "mission")).ok, true);
      assert.equal(selected.actions.at(-1), "mission");
    }
    sessions.at(-1).uploadMission = undefined;
    assert.equal((await rpc("mission", points)).ok, false);
  },
);

test(
  "controller selection rejects live switches and preserves the previous record on failed replacement",
  { timeout: 5000 },
  async (t) => {
    const sessions = [];
    class SelectedSession extends EventEmitter {
      constructor(kind) {
        super();
        this.kind = kind;
        this.phase = "idle";
        this.record = { kind, number: sessions.length };
        sessions.push(this);
      }
      state() {
        return { phase: this.phase, controller: { kind: this.kind } };
      }
      async start({ binaryPath }) {
        if (binaryPath === "invalid") throw Error("invalid binary");
        this.phase = "running";
        return this.state();
      }
      async command(action) {
        if (action === "stop") this.phase = "stopped";
        return this.state();
      }
      exportRecording() {
        return this.record;
      }
      async close() {
        this.closed = true;
      }
    }
    const app = await startCompanion({
      port: 0,
      sessionFactory: (kind) => new SelectedSession(kind),
    });
    t.after(() => app.close());
    const { port } = app.address,
      host = `127.0.0.1:${port}`,
      origin = `http://${host}`;
    const token = JSON.parse((await http(port, "/api/bootstrap")).data).token;
    const ws = await wsOpen(`ws://${host}/bridge?token=${token}`, origin);
    t.after(() => ws.terminate());
    let id = 0;
    const received = [];
    ws.on("message", (data) => received.push(JSON.parse(data)));
    async function start(controller, binaryPath = "valid") {
      const requestId = ++id;
      ws.send(
        JSON.stringify({
          id: requestId,
          type: "start",
          value: { controller, binaryPath, config: {} },
        }),
      );
      while (
        !received.some(
          (reply) => reply.type === "reply" && reply.id === requestId,
        )
      )
        await new Promise((resolve) => setTimeout(resolve, 5));
      return received.find(
        (reply) => reply.type === "reply" && reply.id === requestId,
      );
    }
    assert.equal((await start("ardupilot")).value.controller.kind, "ardupilot");
    assert.equal(sessions[0].closed, true);
    assert.equal((await start("other")).ok, false);
    assert.equal((await start("px4")).ok, false);
    assert.equal(sessions.length, 2);
    const stopId = ++id;
    ws.send(JSON.stringify({ id: stopId, type: "command", value: "stop" }));
    while (
      !received.some((reply) => reply.type === "reply" && reply.id === stopId)
    )
      await new Promise((resolve) => setTimeout(resolve, 5));
    assert.equal((await start("px4", "invalid")).ok, false);
    assert.equal(sessions[2].closed, true);
    assert.deepEqual(
      JSON.parse(
        (await http(port, "/api/recording", { "x-flight-token": token })).data,
      ),
      { kind: "ardupilot", number: 1 },
    );
    assert.equal((await start("px4")).value.controller.kind, "px4");
    assert.deepEqual(
      JSON.parse(
        (await http(port, "/api/recording", { "x-flight-token": token })).data,
      ),
      { kind: "px4", number: 3 },
    );
  },
);
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

test("disconnect checks queued ownership and GCS flight survives reconnect", async (t) => {
  for (const owner of [undefined, "web", "gcs"]) {
    await t.test(String(owner), async (t) => {
      let release, entered;
      const waiting = new Promise((resolve) => {
        release = resolve;
      });
      const started = new Promise((resolve) => {
        entered = resolve;
      });
      const session = new FakeSession();
      session.owner = "web";
      session.phase = "running";
      session.state = () => ({
        phase: session.phase,
        groundStation: { owner: session.owner },
      });
      session.command = async (action) => {
        session.actions.push(action);
        if (action === "resume") {
          entered();
          await waiting;
        }
        if (action === "pause") session.phase = "paused";
        return session.state();
      };
      const app = await startCompanion({
        port: 0,
        sessionFactory: () => session,
      });
      t.after(() => app.close());
      const { port } = app.address,
        host = `127.0.0.1:${port}`;
      const token = JSON.parse((await http(port, "/api/bootstrap")).data).token;
      const url = `ws://${host}/bridge?token=${token}`,
        origin = `http://${host}`;
      const ws = await wsOpen(url, origin);
      ws.send(JSON.stringify({ id: 1, type: "command", value: "resume" }));
      await started;
      const closed = once(ws, "close");
      ws.close();
      await closed;
      // Ownership changes while the disconnect cleanup waits for the command.
      session.owner = owner;
      release();
      // Reconnect is accepted only after queued cleanup releases the old client.
      let next;
      for (let attempt = 0; attempt < 100 && !next; attempt++) {
        try {
          next = await wsOpen(url, origin);
        } catch (error) {
          assert.match(error.message, /409/);
          await new Promise((resolve) => setTimeout(resolve, 5));
        }
      }
      assert.ok(next, "disconnect cleanup must release the client");
      t.after(() => next.terminate());
      const reply = once(next, "message");
      next.send(JSON.stringify({ id: 2, type: "inspect" }));
      const state = JSON.parse((await reply)[0]).value;
      assert.equal(state.phase, owner === "gcs" ? "running" : "paused");
      assert.deepEqual(
        session.actions,
        owner === "gcs" ? ["resume"] : ["resume", "pause"],
      );
    });
  }
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
      value: { controller: "px4", binaryPath: "/test", config: {} },
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
    value: { controller: "px4", binaryPath: "valid", config: {} },
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
        value: { controller: "px4", binaryPath: "invalid", config: {} },
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
        value: { controller: "px4", binaryPath: "valid", config: {} },
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
