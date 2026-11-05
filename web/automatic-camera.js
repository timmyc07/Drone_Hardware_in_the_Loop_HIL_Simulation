const finite = (value) => typeof value === "number" && Number.isFinite(value);
const vector = (value) =>
  Array.isArray(value) && value.length === 3 && Array.from(value).every(finite);

function validateObservation(value) {
  if (!value ||
      typeof value.vehicleId !== "string" ||
      !/^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(value.vehicleId) ||
      !Number.isSafeInteger(value.epoch) || value.epoch < 0 ||
      !Number.isSafeInteger(value.worldTick) || value.worldTick < 0 ||
      !finite(value.time) || value.time < 0 ||
      !finite(value.nowMs) || value.nowMs < 0 || !vector(value.position) ||
      !["running", "airborne", "available", "busy"].every(
        (key) => typeof value[key] === "boolean",
      ))
    throw new Error("無效的自動相機觀測資料");
}

function validateSettings(value) {
  const minimum = value?.mode === "time" ? 0.5 : 0.25;
  const maximum = value?.mode === "time" ? 300 : 500;
  const keys = value && typeof value === "object" ? Reflect.ownKeys(value) : [];
  const prototype = value && typeof value === "object"
    ? Object.getPrototypeOf(value)
    : undefined;
  if (!value || ![Object.prototype, null].includes(prototype) ||
      keys.length !== 3 ||
      keys.some((key) => !["mode", "interval", "maxFrames"].includes(key)) ||
      !["time", "distance"].includes(value.mode) ||
      !finite(value.interval) || value.interval < minimum || value.interval > maximum ||
      !Number.isInteger(value.maxFrames) || value.maxFrames < 1 || value.maxFrames > 30)
    throw new Error("無效的自動相機排程設定");
}

export function createAutomaticCamera({ capture, save, onChange = () => {} }) {
  if (typeof capture !== "function" ||
      typeof save !== "function" ||
      typeof onChange !== "function")
    throw new Error("自動相機需要拍攝與保存函式");
  let state = {
    active: false,
    pending: false,
    phase: "idle",
    vehicleId: null,
    settings: null,
    saved: 0,
    reason: "",
    last: null,
  };
  let generation = 0;
  let epoch;
  let previous;
  let distance = 0;
  let anchor = null;
  let lastWall = -Infinity;
  let published = JSON.stringify(state);
  const getState = () => ({
    ...state,
    settings: state.settings && { ...state.settings },
    last: state.last && { ...state.last, position: [...state.last.position] },
  });
  function publish() {
    const next = JSON.stringify(state);
    if (next === published) return;
    published = next;
    // Observers cannot break the capture lifecycle or create unhandled rejections.
    try {
      Promise.resolve(onChange(getState())).catch(() => {});
    } catch { /* UI callback only. */ }
  }
  function stop(reason = "使用者停止") {
    generation++;
    state.active = false;
    state.phase = "stopped";
    state.reason = String(reason);
    publish();
    return getState();
  }
  function fail(error) {
    generation++;
    state.active = false;
    state.phase = "error";
    state.reason = error instanceof Error ? error.message : String(error);
    publish();
  }
  function phase(observation) {
    return !observation.running ? "paused" : "waiting";
  }
  function observe(observation) {
    try {
      validateObservation(observation);
    } catch (error) {
      if (state.active) fail(error);
      throw error;
    }
    if (!state.active) return getState();
    if (!observation.available ||
        observation.vehicleId !== state.vehicleId || observation.epoch !== epoch)
      return stop("載具或實驗已變更，或相機不可用");
    if (previous && (observation.time < previous.time ||
        observation.worldTick < previous.worldTick))
      return stop("模擬時間或世界更新序號倒退");
    if (previous && observation.worldTick !== previous.worldTick &&
        observation.airborne && previous.airborne) {
      distance += Math.hypot(
        observation.position[0] - previous.position[0],
        observation.position[1] - previous.position[1],
      );
    }
    previous = { ...observation, position: [...observation.position] };
    if (state.pending) return getState();
    state.phase = phase(observation);
    const due = !anchor || (state.settings.mode === "time"
      ? observation.time - anchor.time >= state.settings.interval
      : distance - anchor.distance >= state.settings.interval);
    if (!observation.running || !observation.airborne || observation.busy ||
        !due || observation.nowMs - lastWall < 500) {
      publish();
      return getState();
    }
    const requested = { ...previous, distance };
    const token = generation;
    state.pending = true;
    state.phase = "capturing";
    lastWall = observation.nowMs;
    // Freeze the requested snapshot before an observer can reenter and advance the world.
    let result;
    try {
      result = capture(state.vehicleId);
    } catch (error) {
      result = Promise.reject(error);
    }
    publish();
    Promise.resolve(result).then((frame) => {
      if (token !== generation || !state.active) return;
      const position = frame?.pose?.positionNed;
      if (!frame || typeof frame.captureId !== "string" || !frame.captureId ||
          frame.sourceVehicleId !== requested.vehicleId ||
          frame.worldTick !== requested.worldTick ||
          !finite(frame.time) || Math.abs(frame.time - requested.time) > 1e-9 ||
          !vector(position) ||
          position.some((value, index) =>
            Math.abs(value - requested.position[index]) > 1e-9))
        throw new Error("拍攝影格與請求的世界快照不一致");
      save(frame);
      state.saved++;
      anchor = { time: requested.time, distance: requested.distance };
      state.last = {
        captureId: frame.captureId,
        worldTick: frame.worldTick,
        time: frame.time,
        position: [...position],
        distanceM: requested.distance,
      };
      if (state.active && state.saved >= state.settings.maxFrames) {
        state.active = false;
        state.phase = "complete";
        state.reason = "已達影格上限";
      }
    }).catch((error) => {
      if (token === generation && state.active) fail(error);
    }).finally(() => {
      state.pending = false;
      if (state.active) state.phase = phase(previous);
      publish();
    }).catch(() => {});
    return getState();
  }
  function start(settings, observation) {
    if (state.active || state.pending)
      throw new Error("自動相機仍在執行或等待拍攝完成");
    validateSettings(settings);
    validateObservation(observation);
    generation++;
    epoch = observation.epoch;
    previous = null;
    distance = 0;
    anchor = null;
    state = {
      active: true,
      pending: false,
      phase: "waiting",
      vehicleId: observation.vehicleId,
      settings: { ...settings },
      saved: 0,
      reason: "",
      last: null,
    };
    return observe(observation);
  }
  return { start, observe, stop, getState };
}
