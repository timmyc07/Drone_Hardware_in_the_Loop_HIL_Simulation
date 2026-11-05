import { AIRFRAMES } from './rotor-geometry.js';
import { validateCameraFrame } from './camera.js';
import { DEFAULT_GIMBAL_SETTINGS, GIMBAL_LIMITS, validateGimbalSettings } from './gimbal.js';

const UUID = /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i;
const integer = v => Number.isSafeInteger(v) && v >= 0 && v <= 0x7fffffff;
const keys = (v, expected) => v && Object.getPrototypeOf(v) === Object.prototype && Object.keys(v).sort().join() === [...expected].sort().join();
const vehicleId = v => typeof v === 'string' && /^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(v);
const command = v => {
  if (!keys(v, ['mode', 'target'])) throw Error('Invalid gimbal command');
  const { mode, target } = validateGimbalSettings({ ...v, maxRate: DEFAULT_GIMBAL_SETTINGS.maxRate });
  return { mode, target };
};
const requestFields = {
  hello: [], bind: ['bindingId', 'vehicleId'], status: ['bindingId', 'frame'],
  gimbal: ['bindingId', 'command'], disconnect: [],
};
const responseFields = {
  hello: ['context'], busy: [], bind: ['bindingId', 'vehicleId', 'context'],
  status: ['bindingId', 'context'], frame: ['bindingId', 'frame'],
  gimbal: ['bindingId', 'result'], error: ['bindingId', 'error'], closed: [],
};
function validMessage(m, sessionId, fields) {
  return m && Object.hasOwn(fields, m.type) && keys(m, ['version', 'sessionId', 'clientId', 'requestId', 'type', ...fields[m.type]]) &&
    m.version === 1 && m.sessionId === sessionId && UUID.test(m.clientId) && integer(m.requestId);
}
function endpoint(sessionId, channelFactory) {
  if (!UUID.test(sessionId)) throw Error('Invalid operator session');
  return channelFactory(`drone-hil-operator-v1:${sessionId}`);
}
const defaultFactory = name => new BroadcastChannel(name);
function validTelemetryGimbal(value) {
  try {
    validateGimbalSettings({ mode: value?.mode, target: value?.target, maxRate: value?.maxRate });
    return keys(value.actual, ['yaw', 'pitch', 'roll']) && Object.entries(GIMBAL_LIMITS).every(([axis, [min, max]]) =>
      Number.isFinite(value.actual[axis]) && value.actual[axis] >= min && value.actual[axis] <= max);
  } catch { return false; }
}
function validContext(v) {
  return keys(v, ['available', 'reason', 'vehicles', 'worldTick', 'time', 'running']) &&
    typeof v.available === 'boolean' && typeof v.reason === 'string' && v.reason.length <= 500 &&
    typeof v.running === 'boolean' && integer(v.worldTick) && Number.isFinite(v.time) && v.time >= 0 &&
    Array.isArray(v.vehicles) && v.vehicles.length <= 8 && new Set(v.vehicles.map(x => x?.id)).size === v.vehicles.length &&
    v.vehicles.every(x => keys(x, ['id', 'name', 'airframe', 'status', 'positionNed', 'gimbal']) &&
      vehicleId(x.id) && typeof x.name === 'string' && x.name.length <= 200 &&
      AIRFRAMES.includes(x.airframe) && typeof x.status === 'string' && x.status.length <= 200 &&
      Array.isArray(x.positionNed) && x.positionNed.length === 3 && x.positionNed.every(Number.isFinite) && validTelemetryGimbal(x.gimbal));
}

export function createOperatorHost({ sessionId, getContext, capture, applyGimbal, onChange = () => {}, channelFactory = defaultFactory, now = Date.now, setInterval: interval = globalThis.setInterval, clearInterval: clear = globalThis.clearInterval }) {
  const channel = endpoint(sessionId, channelFactory);
  let closed = false, active = null, capturing = false, applying = false, lastCapture = -Infinity;
  const state = () => ({ sessionId, clientId: active?.clientId ?? null, bindingId: active?.bindingId ?? null, vehicleId: active?.vehicleId ?? null });
  const changed = () => onChange(state());
  const send = (m, type, fields = {}) => { if (!closed) channel.postMessage({ version: 1, sessionId, clientId: m.clientId, requestId: m.requestId, type, ...fields }); };
  const expire = () => { if (active && now() - active.seen >= 10000) { active = null; changed(); } };
  const context = () => { const value = structuredClone(getContext()); if (!validContext(value)) throw Error('Invalid operator context'); return value; };
  const timer = interval(expire, 1000);
  channel.onmessage = async ({ data: m }) => {
    if (closed || !validMessage(m, sessionId, requestFields)) return;
    if (m.type === 'bind' && (!UUID.test(m.bindingId) || !vehicleId(m.vehicleId))) return;
    if (['status', 'gimbal'].includes(m.type) && !(m.bindingId === null || UUID.test(m.bindingId))) return;
    if (m.type === 'status' && typeof m.frame !== 'boolean') return;
    let settings;
    if (m.type === 'gimbal') { try { settings = command(m.command); } catch { return; } }
    expire();
    if (active && active.clientId !== m.clientId) { if (m.type === 'hello') send(m, 'busy'); return; }
    if (!active) {
      if (m.type !== 'hello') return;
      active = { clientId: m.clientId, seen: now(), sequence: -1, bindingId: null, vehicleId: null };
    }
    if (m.requestId <= active.sequence) return;
    active.sequence = m.requestId;
    if (m.type !== 'hello' && ['status', 'gimbal'].includes(m.type) && m.bindingId !== active.bindingId) return;
    active.seen = now();
    if (m.type === 'disconnect') { active = null; changed(); return; }
    const owner = active;
    const binding = owner.bindingId;
    const current = () => !closed && active === owner && active.bindingId === binding;
    const fail = error => { if (current()) send(m, 'error', { bindingId: m.type === 'bind' ? m.bindingId : binding, error: String(error.message || error).slice(0, 200) }); };
    try {
      const ctx = context();
      if (m.type === 'hello') { send(m, 'hello', { context: ctx }); changed(); return; }
      if (m.type === 'bind') {
        if (!ctx.available || !ctx.vehicles.some(v => v.id === m.vehicleId)) throw Error('Vehicle unavailable');
        owner.bindingId = m.bindingId; owner.vehicleId = m.vehicleId;
        send(m, 'bind', { bindingId: m.bindingId, vehicleId: m.vehicleId, context: ctx }); changed(); return;
      }
      if (m.type === 'status') send(m, 'status', { bindingId: binding, context: ctx });
      if (!ctx.available || !binding || !ctx.vehicles.some(v => v.id === owner.vehicleId)) {
        if (m.type === 'gimbal' || m.frame) throw Error('Vehicle unavailable');
        return;
      }
      if (m.type === 'status' && m.frame) {
        if (capturing || now() - lastCapture < 500) throw Error('Capture busy');
        capturing = true; lastCapture = now();
        try {
          const frame = validateCameraFrame(await capture(owner.vehicleId));
          if (frame.sourceVehicleId !== owner.vehicleId) throw Error('Camera vehicle mismatch');
          if (current()) send(m, 'frame', { bindingId: binding, frame });
        } finally { capturing = false; }
      }
      if (m.type === 'gimbal') {
        if (applying) throw Error('Gimbal busy');
        applying = true;
        try {
          const result = await applyGimbal(owner.vehicleId, settings);
          if (!keys(result, ['gimbal', 'appliedWorldTick']) || !integer(result.appliedWorldTick)) throw Error('Invalid gimbal result');
          validateGimbalSettings(result.gimbal);
          if (current()) send(m, 'gimbal', { bindingId: binding, result });
        } finally { applying = false; }
      }
    } catch (error) { fail(error); }
  };
  return { getState: state, close() { if (closed) return; if (active) send({ ...active, requestId: active.sequence }, 'closed'); closed = true; active = null; clear(timer); channel.onmessage = null; channel.close(); changed(); } };
}

export function createOperatorClient({ sessionId, onChange = () => {}, channelFactory = defaultFactory, now = Date.now, setInterval: interval = globalThis.setInterval, clearInterval: clear = globalThis.clearInterval }) {
  const channel = endpoint(sessionId, channelFactory), clientId = crypto.randomUUID(), responses = new Map();
  let errorSource = null;
  let closed = false, sequence = 0, seen = now(), bindAt = 0, bindRequest = null, frameRequest = null, gimbalRequest = null;
  let state = { connection: 'connecting', context: null, bindingId: null, vehicleId: null, frame: null, pendingFrame: false, pendingGimbal: false, lastResult: null, error: null };
  const changed = () => onChange({ ...state });
  const send = (type, fields = {}) => {
    if (sequence >= 0x7fffffff) throw Error('Operator request sequence exhausted');
    const requestId = ++sequence;
    channel.postMessage({ version: 1, sessionId, clientId, requestId, type, ...fields });
    return requestId;
  };
  const abandon = () => { bindRequest = frameRequest = gimbalRequest = null; state.bindingId = null; state.vehicleId = null; state.pendingFrame = state.pendingGimbal = false; };
  const terminate = () => { closed = true; clear(timer); channel.onmessage = null; channel.close(); abandon(); state.connection = 'closed'; };
  channel.onmessage = ({ data: m }) => {
    if (closed || !validMessage(m, sessionId, responseFields) || m.clientId !== clientId || m.requestId > sequence || m.requestId <= (responses.get(m.type) ?? -1)) return;
    if ('bindingId' in m && m.bindingId !== state.bindingId) return;
    if (m.type === 'bind' && m.requestId !== bindRequest) return;
    if (m.type === 'frame' && m.requestId !== frameRequest?.id) return;
    if (m.type === 'gimbal' && m.requestId !== gimbalRequest?.id) return;
    try {
      if ('context' in m && !validContext(m.context)) return;
      if (m.type === 'bind' && m.vehicleId !== state.vehicleId) return;
      if (m.type === 'frame') {
        const frame = validateCameraFrame(m.frame);
        if (frame.sourceVehicleId !== state.vehicleId) return;
        state.frame = frame; frameRequest = null; state.pendingFrame = false;
        if (errorSource === 'frame') { state.error = null; errorSource = null; }
      }
      if (m.type === 'gimbal') {
        if (!keys(m.result, ['gimbal','appliedWorldTick']) || !integer(m.result.appliedWorldTick)) return;
        validateGimbalSettings(m.result.gimbal);
        state.lastResult = m.result; gimbalRequest = null; state.pendingGimbal = false;
      }
    } catch { return; }
    if (m.type === 'error' && (typeof m.error !== 'string' || m.error.length > 200)) return;
    responses.set(m.type, m.requestId);
    seen = now();
    if (m.type === 'busy') { state.connection = 'busy'; abandon(); }
    else if (m.type === 'closed') terminate();
    else {
      if (m.type === 'hello' || m.type === 'bind') state.connection = 'connected';
      if ('context' in m) state.context = m.context;
      if (m.type === 'bind') { bindRequest = null; state.frame = null; state.error = null; errorSource = null; }
      if (m.type === 'error') {
        if (frameRequest?.id === m.requestId) {
          frameRequest = null; state.pendingFrame = false;
          if (errorSource !== 'gimbal') { state.error = m.error; errorSource = 'frame'; }
        }
        if (gimbalRequest?.id === m.requestId) { gimbalRequest = null; state.pendingGimbal = false; state.error = `Result unknown: ${m.error}`; errorSource = 'gimbal'; }
        if (bindRequest === m.requestId) { abandon(); state.error = m.error; errorSource = 'bind'; }
      }
    }
    changed();
  };
  const timer = interval(() => {
    if (closed) return;
    if (now() - seen >= 10000 && state.connection !== 'stale') { state.connection = 'stale'; abandon(); state.error = 'Connection lost; last image is stale'; errorSource = 'connection'; changed(); }
    if (bindRequest && now() - bindAt >= 10000) { abandon(); state.error = 'Bind timed out; bind again'; errorSource = 'bind'; changed(); }
    for (const [pending, field] of [[frameRequest, 'pendingFrame'], [gimbalRequest, 'pendingGimbal']]) {
      if (pending && now() - pending.at >= 10000) { state[field] = false; if (field === 'pendingGimbal' || errorSource !== 'gimbal') { state.error = 'Result unknown: request timed out'; errorSource = field === 'pendingFrame' ? 'frame' : 'gimbal'; } if (field === 'pendingFrame') frameRequest = null; else gimbalRequest = null; changed(); }
    }
    if (state.connection === 'connected') send('status', { bindingId: state.bindingId, frame: false });
    else send('hello');
  }, 1000);
  send('hello');
  return {
    getState: () => ({ ...state }),
    bind(id) {
      if (closed || state.connection !== 'connected' || !vehicleId(id)) return false;
      abandon(); state.bindingId = crypto.randomUUID(); state.vehicleId = id; state.frame = null; state.lastResult = null;
      bindAt = now(); bindRequest = send('bind', { bindingId: state.bindingId, vehicleId: id }); changed(); return true;
    },
    requestFrame() {
      if (closed || state.connection !== 'connected' || !state.bindingId || bindRequest || frameRequest) return false;
      frameRequest = { id: send('status', { bindingId: state.bindingId, frame: true }), at: now() }; state.pendingFrame = true; changed(); return true;
    },
    setGimbal(value) {
      const settings = command(value);
      if (closed || state.connection !== 'connected' || !state.bindingId || bindRequest || gimbalRequest) return false;
      gimbalRequest = { id: send('gimbal', { bindingId: state.bindingId, command: settings }), at: now() }; state.pendingGimbal = true; state.error = null; errorSource = null; changed(); return true;
    },
    disconnect() { if (closed) return; send('disconnect'); terminate(); changed(); },
    close() { if (closed) return; send('disconnect'); terminate(); changed(); },
  };
}
