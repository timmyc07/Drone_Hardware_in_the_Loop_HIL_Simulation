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
      assert.equal(delay, 25000);
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
  assert.deepEqual(
    hilFlightReadiness({ controller: { kind: "px4", verified: true } }),
    [],
  );
  assert.deepEqual(
    hilFlightReadiness({
      controller: { kind: "px4", verified: true, armedKnown: false },
    }),
    ["心跳"],
  );
});

test("follow view switches only on an unfollowed ground-to-air transition", () => {
  assert.equal(shouldFollowTakeoff(true, false, false), true);
  assert.equal(shouldFollowTakeoff(true, false, true), false);
  assert.equal(shouldFollowTakeoff(false, false, false), false);
  assert.equal(shouldFollowTakeoff(undefined, false, false), false);
});

test("ArduPilot mission gates upload and start on verified matching route", async () => {
  const { externalMissionControls, externalMissionIndex } =
    await import("../hil-client.js");
  const points = [{ north: 5, east: 0, altitude: 3, hold: 1 }];
  const state = {
    groundStation: { owner: "web" },
    phase: "running",
    controller: {
      kind: "ardupilot",
      verified: true,
      parametersVerified: true,
      armedKnown: true,
      armed: false,
    },
    snapshot: { runway: { weightOnWheels: true } },
    mission: {
      phase: "ready",
      homeReady: true,
      readyToUpload: true,
      verified: true,
      points,
      limits: {
        radius: 50,
        height: 20,
        minAltitude: 1,
        holdRequiresInteger: true,
      },
      waypointStartSeq: 2,
      currentSeq: 2,
    },
  };
  assert.equal(externalMissionControls(state, points).upload, true);
  assert.equal(externalMissionControls(state, points).start, true);
  assert.equal(
    externalMissionControls(
      { ...state, mission: { ...state.mission, readyToUpload: false } },
      points,
    ).start,
    false,
  );
  assert.equal(
    externalMissionControls(state, [{ ...points[0], altitude: 0 }]).upload,
    false,
  );
  assert.equal(
    externalMissionControls(
      { ...state, mission: { ...state.mission, homeReady: false } },
      points,
    ).upload,
    false,
  );
  assert.equal(
    externalMissionControls(
      { ...state, mission: { ...state.mission, homeReady: false } },
      points,
    ).start,
    false,
  );
  assert.equal(externalMissionIndex(state.mission), 0);
  assert.equal(externalMissionIndex({ ...state.mission, currentSeq: 1 }), null);
  assert.equal(externalMissionIndex({ ...state.mission, currentSeq: 3 }), null);
  assert.equal(
    externalMissionControls(state, [{ ...points[0], hold: 1.5 }]).upload,
    false,
  );
  assert.equal(
    externalMissionControls(state, [{ ...points[0], north: 51 }]).upload,
    false,
  );
  assert.equal(
    externalMissionControls(state, [{ ...points[0], north: 6 }]).start,
    false,
  );
  assert.equal(externalMissionControls(state, points, true).upload, false);
  assert.equal(
    externalMissionControls(
      { ...state, controller: { ...state.controller, armed: true } },
      points,
    ).start,
    false,
  );
  assert.equal(
    externalMissionControls(
      {
        ...state,
        controller: { ...state.controller, parametersVerified: false },
      },
      points,
    ).start,
    false,
  );
  assert.equal(
    externalMissionControls(
      { ...state, snapshot: { runway: { weightOnWheels: false } } },
      points,
    ).start,
    false,
  );
  assert.equal(
    externalMissionControls(
      { ...state, controller: { ...state.controller, armed: true } },
      points,
    ).upload,
    false,
  );
  assert.equal(
    externalMissionControls(
      { ...state, mission: { ...state.mission, phase: "interrupted" } },
      points,
    ).start,
    false,
  );
});

test("PX4 and ArduPilot mission sequences and hold rules use reported contract", async () => {
  const {
    externalMissionControls,
    externalMissionIndex,
    externalMissionReachedCount,
  } = await import("../hil-client.js");
  const points = [
    { north: 5, east: 0, altitude: 3, hold: 0.5 },
    { north: 6, east: 0, altitude: 3, hold: 0 },
  ];
  const mission = {
    phase: "ready",
    homeReady: true,
    readyToUpload: true,
    verified: true,
    points,
    waypointStartSeq: 1,
    currentSeq: 1,
    reachedSeqs: [0, 1, 1, 2, 3],
    limits: {
      radius: 50,
      height: 20,
      minAltitude: 1,
      holdRequiresInteger: false,
    },
  };
  const state = {
    groundStation: { owner: "web" },
    phase: "running",
    controller: { kind: "px4", armed: false, verified: true },
    snapshot: { runway: { weightOnWheels: true } },
    mission,
  };
  assert.equal(externalMissionControls(state, points).upload, true);
  assert.equal(externalMissionControls(state, points).start, true);
  assert.equal(externalMissionIndex(mission), 0);
  assert.equal(externalMissionIndex({ ...mission, currentSeq: 2 }), 1);
  assert.equal(externalMissionIndex({ ...mission, currentSeq: 0 }), null);
  assert.equal(externalMissionIndex({ ...mission, currentSeq: 3 }), null);
  assert.equal(externalMissionReachedCount(mission), 2);
  assert.equal(
    externalMissionIndex({ ...mission, waypointStartSeq: undefined }),
    null,
  );
  assert.equal(
    externalMissionReachedCount({ ...mission, waypointStartSeq: undefined }),
    null,
  );
  const ap = {
    ...mission,
    waypointStartSeq: 2,
    currentSeq: 2,
    reachedSeqs: [0, 1, 2, 2, 3, 4],
    limits: { ...mission.limits, holdRequiresInteger: true },
  };
  assert.equal(externalMissionIndex(ap), 0);
  assert.equal(externalMissionIndex({ ...ap, currentSeq: 1 }), null);
  assert.equal(externalMissionIndex({ ...ap, currentSeq: 4 }), null);
  assert.equal(externalMissionReachedCount(ap), 2);
  assert.equal(
    externalMissionControls(
      {
        ...state,
        controller: {
          kind: "ardupilot",
          armed: false,
          verified: true,
          parametersVerified: true,
          armedKnown: true,
        },
        mission: ap,
      },
      points,
    ).upload,
    false,
  );
  assert.equal(
    externalMissionControls(
      { ...state, mission: { ...mission, readyToUpload: false } },
      points,
    ).start,
    false,
  );
  assert.equal(
    externalMissionControls(state, [{ ...points[0], north: 6 }, points[1]])
      .start,
    false,
  );
});

test("ground station configuration uses the authenticated request and ownership gates missions", async () => {
  const { client } = makeClient();
  await client.connect();
  for (const targetPort of [1023, 4560, 5760, 9002, 14560, 14561, 65536])
    await assert.rejects(client.configureGroundStation({ targetPort }), /UDP/);
  const value = { enabled: true, targetPort: 14550, owner: "gcs" };
  const pending = client.configureGroundStation(value);
  assert.deepEqual(Socket.current.sent, {
    id: 1,
    type: "groundStation",
    value,
  });
  Socket.current.reply({
    type: "reply",
    id: 1,
    ok: true,
    value: { owner: "gcs" },
  });
  assert.deepEqual(await pending, { owner: "gcs" });
  client.close();
  const { hilWebControlAllowed, externalMissionControls } =
    await import("../hil-client.js");
  const points = [{ north: 0, east: 0, altitude: 2, hold: 1 }];
  const state = {
    phase: "running",
    groundStation: { owner: "web" },
    controller: { kind: "px4", verified: true, armed: false },
    snapshot: { runway: { weightOnWheels: true } },
    mission: {
      phase: "ready",
      homeReady: true,
      readyToUpload: true,
      verified: true,
      points,
      limits: {
        radius: 20,
        height: 10,
        minAltitude: 1,
        holdRequiresInteger: false,
      },
    },
  };
  assert.equal(hilWebControlAllowed(state), true);
  assert.equal(externalMissionControls(state, points).start, true);
  const controlled = { ...state, groundStation: { owner: "gcs" } };
  assert.equal(hilWebControlAllowed(controlled), false);
  assert.equal(externalMissionControls(controlled, points).upload, false);
  assert.equal(externalMissionControls(controlled, points).start, false);
  assert.equal(hilWebControlAllowed({}), false);
});
