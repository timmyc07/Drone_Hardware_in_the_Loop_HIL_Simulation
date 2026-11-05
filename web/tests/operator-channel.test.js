import test from 'node:test';
import assert from 'node:assert/strict';
import { createOperatorHost, createOperatorClient } from '../operator-channel.js';
import { cameraMetadata } from '../camera.js';
import { createSimulation } from '../sim.js';
import { DEFAULT_GIMBAL_SETTINGS } from '../gimbal.js';

const flush = async () => { for (let i = 0; i < 8; i++) await Promise.resolve(); };
function fixture() {
  let time = 0;
  const channels = new Set(), timers = new Set(), messages = [];
  const options = {
    sessionId: crypto.randomUUID(), now: () => time,
    setInterval(fn) { timers.add(fn); return fn; }, clearInterval(fn) { timers.delete(fn); },
    channelFactory(name) {
      const channel = { name, onmessage: null, postMessage(data) { messages.push(structuredClone(data)); for (const other of channels) if (other !== channel && other.name === name) queueMicrotask(() => other.onmessage?.({ data: structuredClone(data) })); }, close() { channels.delete(channel); } };
      channels.add(channel); return channel;
    },
  };
  const context = { available: true, reason: '', vehicles: [{ id: 'drone-1', name: 'Drone', airframe: 'quadrotor', status: 'ready', positionNed: [0,0,0], gimbal: createSimulation().snapshot().gimbal }], worldTick: 0, time: 0, running: false };
  let captures = 0, commands = 0;
  const frame = () => {
    const metadata = cameraMetadata(createSimulation().snapshot(), { width: 64, height: 48 }, null, { sourceVehicleId: 'drone-1', worldTick: 0, captureId: crypto.randomUUID() });
    return { ...metadata, rgbRgba: new Uint8Array(64*48*4), depthM: new Float32Array(64*48), classes: new Uint8Array(64*48) };
  };
  const hostOptions = { ...options, getContext: () => context, capture: async () => { captures++; return frame(); }, applyGimbal: async (_, settings) => { commands++; return { gimbal: { ...settings, maxRate: DEFAULT_GIMBAL_SETTINGS.maxRate }, appliedWorldTick: 1 }; } };
  return { options, hostOptions, context, frame, messages, channels, timers, get captures() { return captures; }, get commands() { return commands; }, async advance(ms) { time += ms; for (const fn of timers) fn(); await flush(); } };
}

test('single client binds actual camera, limits capture and ACKs only applied gimbal', async () => {
  const f = fixture(), host = createOperatorHost(f.hostOptions), client = createOperatorClient(f.options);
  try {
    await flush(); assert.equal(client.getState().connection, 'connected');
    client.bind('drone-1'); await flush();
    assert.equal(client.requestFrame(), true); assert.equal(client.requestFrame(), false); await flush();
    assert.equal(f.captures, 1); assert.ok(client.getState().frame.depthM instanceof Float32Array);
    client.requestFrame(); await flush(); assert.equal(f.captures, 1); assert.equal(client.getState().pendingFrame, false);
    await f.advance(500); client.requestFrame(); await flush(); assert.equal(f.captures, 2);
    client.setGimbal({ mode: 'ned', target: { yaw: 0, pitch: 0, roll: 0 } }); await flush();
    assert.equal(f.commands, 1); assert.equal(client.getState().lastResult.appliedWorldTick, 1);
    const second = createOperatorClient(f.options); await flush(); assert.equal(second.getState().connection, 'busy'); second.close();
  } finally { client.close(); host.close(); }
  assert.equal(f.channels.size, 0); assert.equal(f.timers.size, 0);
});

test('rejects forbidden fields, stale binding and replayed request sequence', async () => {
  const f = fixture(), host = createOperatorHost(f.hostOptions), client = createOperatorClient(f.options);
  try {
    await flush(); client.bind('drone-1'); await flush();
    const base = f.messages.find(m => m.type === 'bind');
    const attacker = f.options.channelFactory(`drone-hil-operator-v1:${f.options.sessionId}`);
    const gimbal = { ...base, type: 'gimbal', requestId: 50, command: { mode: 'body', target: { yaw: 0, pitch: 0, roll: 0 } } }; delete gimbal.vehicleId;
    attacker.postMessage({ ...gimbal, flight: true });
    attacker.postMessage({ ...gimbal, bindingId: crypto.randomUUID() });
    await flush(); assert.equal(f.commands, 0);
    attacker.postMessage({ ...gimbal, requestId: 51 }); await flush();
    attacker.postMessage({ ...gimbal, requestId: 51 }); await flush(); assert.equal(f.commands, 1);
    attacker.close();
  } finally { client.close(); host.close(); }
});

test('switching binding discards pending capture and gimbal ACK; host closure terminates client', async () => {
  const f = fixture(); let resolveFrame, resolveGimbal;
  const host = createOperatorHost({ ...f.hostOptions, capture: () => new Promise(resolve => { resolveFrame = resolve; }), applyGimbal: () => new Promise(resolve => { resolveGimbal = resolve; }) });
  const client = createOperatorClient(f.options);
  try {
    await flush(); client.bind('drone-1'); await flush(); client.requestFrame(); client.setGimbal({ mode: 'body', target: { yaw:0,pitch:0,roll:0 } }); await flush();
    client.bind('drone-1'); await flush(); resolveFrame(f.frame()); resolveGimbal({ gimbal: DEFAULT_GIMBAL_SETTINGS, appliedWorldTick: 1 }); await flush();
    assert.equal(client.getState().frame, null); assert.equal(client.getState().lastResult, null);
    host.close(); await flush(); assert.equal(client.getState().connection, 'closed'); assert.equal(client.setGimbal({ mode: 'body', target: { yaw:0,pitch:0,roll:0 } }), false);
  } finally { client.close(); host.close(); }
});

test('native BroadcastChannel structured clone retains three camera typed buffers', async () => {
  const f = fixture(); const options = { sessionId: f.options.sessionId };
  let client;
  const host = createOperatorHost({ ...f.hostOptions, ...options, channelFactory: undefined, setInterval: undefined, clearInterval: undefined, now: undefined });
  try {
    await new Promise((resolve, reject) => {
      const timeout = setTimeout(() => reject(Error('Native channel timeout')), 3000);
      client = createOperatorClient({ ...options, onChange(state) {
        if (state.connection === 'connected' && !state.bindingId) client.bind('drone-1');
        else if (state.bindingId && !state.frame && !state.pendingFrame) client.requestFrame();
        if (state.frame) { clearTimeout(timeout); assert.ok(state.frame.rgbRgba instanceof Uint8Array); assert.ok(state.frame.depthM instanceof Float32Array); assert.ok(state.frame.classes instanceof Uint8Array); resolve(); }
      } });
    });
  } finally { client?.close(); host.close(); }
});


test('heartbeat silence expires lease, retains image and requires a fresh bind', async () => {
  const f = fixture(), host = createOperatorHost(f.hostOptions), client = createOperatorClient(f.options);
  try {
    await flush(); client.bind('drone-1'); await flush(); client.requestFrame(); await flush();
    const image = client.getState().frame;
    const hostChannel = [...f.channels][0], receive = hostChannel.onmessage;
    hostChannel.onmessage = null;
    await f.advance(10000);
    assert.equal(client.getState().connection, 'stale'); assert.equal(client.getState().frame, image);
    assert.equal(client.getState().bindingId, null); assert.equal(client.requestFrame(), false);
    hostChannel.onmessage = receive;
    await f.advance(1000);
    assert.equal(client.getState().connection, 'connected'); assert.equal(client.getState().bindingId, null);
    assert.equal(client.requestFrame(), false); assert.equal(f.captures, 1);
  } finally { client.close(); host.close(); }
});


test('all world airframes and dotted vehicle IDs bind and capture', async () => {
  for (const airframe of ['quadrotor', 'hexarotor', 'octorotor', 'fixedwing', 'vtol']) {
    const f = fixture(); f.context.vehicles[0].id = 'B.2'; f.context.vehicles[0].airframe = airframe;
    const frame = f.frame(); frame.sourceVehicleId = 'B.2'; frame.airframe = airframe;
    const host = createOperatorHost({ ...f.hostOptions, capture: async () => frame }), client = createOperatorClient(f.options);
    try {
      await flush(); assert.equal(client.getState().connection, 'connected'); client.bind('B.2'); await flush();
      assert.equal(client.requestFrame(), true); await flush(); assert.equal(client.getState().frame?.sourceVehicleId, 'B.2');
    } finally { client.close(); host.close(); }
  }
});

test('explicit disconnect closes channel and never reconnects', async () => {
  const f = fixture(), host = createOperatorHost(f.hostOptions), client = createOperatorClient(f.options);
  try {
    await flush(); client.disconnect(); await flush();
    const count = f.messages.length;
    await f.advance(20000);
    assert.equal(client.getState().connection, 'closed'); assert.equal(host.getState().clientId, null);
    assert.equal(f.messages.length, count); assert.equal(f.channels.size, 1);
  } finally { client.close(); host.close(); }
});

test('gimbal timeout reports unknown and drops a late ACK without retry', async () => {
  const f = fixture(); let resolveGimbal, commands = 0;
  const host = createOperatorHost({ ...f.hostOptions, applyGimbal: () => { commands++; return new Promise(resolve => { resolveGimbal = resolve; }); } });
  const client = createOperatorClient(f.options);
  try {
    await flush(); client.bind('drone-1'); await flush();
    client.setGimbal({ mode: 'body', target: { yaw:0, pitch:0, roll:0 } }); await flush();
    await f.advance(5000); await f.advance(5000);
    assert.equal(client.getState().pendingGimbal, false); assert.match(client.getState().error, /Result unknown/);
    resolveGimbal({ gimbal: DEFAULT_GIMBAL_SETTINGS, appliedWorldTick: 1 }); await flush();
    assert.equal(client.getState().lastResult, null); assert.equal(commands, 1);
  } finally { client.close(); host.close(); }
});

test('nonexistent vehicle bind reports error and clears pending binding', async () => {
  const f = fixture(), host = createOperatorHost(f.hostOptions), client = createOperatorClient(f.options);
  try {
    await flush(); client.bind('missing'); await flush();
    assert.equal(client.getState().bindingId, null); assert.match(client.getState().error, /unavailable/);
    assert.equal(client.bind('drone-1'), true); await flush(); assert.equal(client.requestFrame(), true); await flush();
    assert.ok(client.getState().frame);
  } finally { client.close(); host.close(); }
});

test('successful frame clears capture errors but preserves unknown gimbal result across frames and heartbeats', async () => {
  const f = fixture(), host = createOperatorHost({ ...f.hostOptions, applyGimbal: async () => { throw Error('Apply failed'); } });
  const client = createOperatorClient(f.options);
  try {
    await flush(); client.bind('drone-1'); await flush(); client.requestFrame(); await flush();
    client.requestFrame(); await flush(); assert.match(client.getState().error, /Capture busy/);
    await f.advance(500); client.requestFrame(); await flush(); assert.equal(client.getState().error, null);
    client.setGimbal({ mode: 'body', target: { yaw:0, pitch:0, roll:0 } }); await flush();
    const unknown = client.getState().error; assert.match(unknown, /Result unknown/);
    client.requestFrame(); await flush(); assert.equal(client.getState().error, unknown);
    await f.advance(1000); client.requestFrame(); await flush();
    assert.equal(client.getState().pendingFrame, false); assert.equal(client.getState().error, unknown);
  } finally { client.close(); host.close(); }
});


test('invalid telemetry gimbal target or actual is rejected without refreshing heartbeat', async () => {
  for (const mutate of [
    g => { g.target.yaw = Infinity; },
    g => { g.target.pitch = Math.PI; },
    g => { g.actual.roll = NaN; },
    g => { g.actual.pitch = Math.PI; },
    g => { delete g.actual.yaw; },
  ]) {
    const f = fixture(), host = createOperatorHost(f.hostOptions), client = createOperatorClient(f.options);
    const sender = f.options.channelFactory(`drone-hil-operator-v1:${f.options.sessionId}`);
    try {
      await flush(); assert.equal(client.getState().connection, 'connected');
      [...f.channels][0].onmessage = null;
      await f.advance(5000);
      const request = f.messages.findLast(m => m.type === 'status');
      const invalid = structuredClone(f.context); mutate(invalid.vehicles[0].gimbal);
      sender.postMessage({ version: 1, sessionId: f.options.sessionId, clientId: request.clientId, requestId: request.requestId, type: 'status', bindingId: null, context: invalid });
      await flush(); assert.deepEqual(client.getState().context, f.context);
      await f.advance(5000); assert.equal(client.getState().connection, 'stale');
    } finally { sender.close(); client.close(); host.close(); }
  }
});


test('host closure terminates session, retains last image and unknown result without retries', async () => {
  const f = fixture(), host = createOperatorHost({ ...f.hostOptions, applyGimbal: async () => { throw Error('Apply failed'); } });
  const client = createOperatorClient(f.options);
  try {
    await flush(); client.bind('drone-1'); await flush(); client.requestFrame(); await flush();
    client.setGimbal({ mode: 'body', target: { yaw:0, pitch:0, roll:0 } }); await flush();
    const image = client.getState().frame, error = client.getState().error;
    assert.match(error, /Result unknown/);
    host.close(); await flush(); const count = f.messages.length;
    await f.advance(20000);
    assert.equal(client.getState().connection, 'closed'); assert.equal(client.getState().frame, image);
    assert.equal(client.getState().error, error); assert.equal(client.getState().bindingId, null);
    assert.equal(client.getState().pendingFrame, false); assert.equal(client.getState().pendingGimbal, false);
    assert.equal(client.requestFrame(), false); assert.equal(client.bind('drone-1'), false);
    assert.equal(client.setGimbal({ mode: 'body', target: { yaw:0, pitch:0, roll:0 } }), false);
    assert.equal(f.messages.length, count); assert.equal(f.channels.size, 0); assert.equal(f.timers.size, 0);
  } finally { client.close(); host.close(); }
});
