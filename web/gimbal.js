// Ideal kinematic joints only, not a calibrated actuator or MAVLink gimbal implementation.
// Concepts: github.com/microsoft/AirSim/blob/main/docs/settings.md and
// github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_Gimbal.cpp (no code copied).
const AXES = ["yaw", "pitch", "roll"];
const axes = (value) =>
  Object.freeze({ yaw: value, pitch: value, roll: value });
export const GIMBAL_LIMITS = Object.freeze({
  yaw: Object.freeze([-Math.PI, Math.PI]),
  pitch: Object.freeze([-Math.PI / 2, Math.PI / 2]),
  roll: Object.freeze([-Math.PI / 4, Math.PI / 4]),
});
export const DEFAULT_GIMBAL_SETTINGS = Object.freeze({
  mode: "body",
  target: Object.freeze({ yaw: 0, pitch: Math.PI / 12, roll: 0 }),
  maxRate: axes(Math.PI / 2),
});
function object(value, keys, label) {
  if (
    !value ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value)) ||
    Object.keys(value).sort().join(",") !== [...keys].sort().join(",")
  )
    throw Error(`Invalid ${label}: expected ${keys.join(",")}`);
  return value;
}
const finite = (value, min, max, label) => {
  if (!Number.isFinite(value) || value < min || value > max)
    throw Error(`Invalid gimbal ${label}`);
  return value;
};
function angles(value, label) {
  object(value, AXES, label);
  return Object.fromEntries(
    AXES.map((axis) => [
      axis,
      finite(value[axis], ...GIMBAL_LIMITS[axis], `${label}.${axis}`),
    ]),
  );
}
export function validateGimbalSettings(value = DEFAULT_GIMBAL_SETTINGS) {
  object(value, ["mode", "target", "maxRate"], "gimbal settings");
  if (!["body", "ned"].includes(value.mode))
    throw Error("Gimbal mode must be body or ned");
  const target = angles(value.target, "target");
  object(value.maxRate, AXES, "maxRate");
  const maxRate = Object.fromEntries(
    AXES.map((axis) => [
      axis,
      finite(
        value.maxRate[axis],
        Math.PI / 180,
        2 * Math.PI,
        `maxRate.${axis}`,
      ),
    ]),
  );
  return { mode: value.mode, target, maxRate };
}
const multiply = (a, b) =>
  Array.from({ length: 9 }, (_, i) => {
    const r = Math.floor(i / 3) * 3,
      c = i % 3;
    return a[r] * b[c] + a[r + 1] * b[c + 3] + a[r + 2] * b[c + 6];
  });
const transpose = (a) => [a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]];
const clamp = (x, min, max) => Math.max(min, Math.min(max, x));
function bodyRotation(quaternion) {
  if (
    !Array.isArray(quaternion) ||
    quaternion.length !== 4 ||
    !Array.from(quaternion).every(Number.isFinite) ||
    Math.abs(Math.hypot(...quaternion) - 1) > 1e-5
  )
    throw Error("Gimbal requires a unit body quaternion");
  const n = Math.hypot(...quaternion),
    [w, x, y, z] = quaternion.map((v) => v / n);
  return [
    1 - 2 * (y * y + z * z),
    2 * (x * y - w * z),
    2 * (x * z + w * y),
    2 * (x * y + w * z),
    1 - 2 * (x * x + z * z),
    2 * (y * z - w * x),
    2 * (x * z - w * y),
    2 * (y * z + w * x),
    1 - 2 * (x * x + y * y),
  ];
}
export function gimbalRotation(actual) {
  const { yaw, pitch, roll } = angles(actual, "actual");
  const c = Math.cos(yaw),
    s = Math.sin(yaw),
    cp = Math.cos(pitch),
    sp = Math.sin(pitch),
    cr = Math.cos(roll),
    sr = Math.sin(roll);
  return [
    c * cp,
    -c * sp * sr - s * cr,
    -c * sp * cr + s * sr,
    s * cp,
    -s * sp * sr + c * cr,
    -s * sp * cr - c * sr,
    sp,
    cp * sr,
    cp * cr,
  ];
}
function goal(settings, body) {
  return settings.mode === "body"
    ? multiply(body, gimbalRotation(settings.target))
    : gimbalRotation(settings.target);
}
function orientationError(actualWorld, desiredWorld) {
  const relative = multiply(transpose(actualWorld), desiredWorld);
  // atan2 is well-conditioned at both zero error and a half turn.
  const sin =
    Math.hypot(
      relative[7] - relative[5],
      relative[2] - relative[6],
      relative[3] - relative[1],
    ) / 2;
  return Math.atan2(
    sin,
    clamp((relative[0] + relative[4] + relative[8] - 1) / 2, -1, 1),
  );
}
function rotationQuaternion(matrix) {
  const diagonal = [
    1 + matrix[0] + matrix[4] + matrix[8],
    1 + matrix[0] - matrix[4] - matrix[8],
    1 - matrix[0] + matrix[4] - matrix[8],
    1 - matrix[0] - matrix[4] + matrix[8],
  ];
  const largest = diagonal.indexOf(Math.max(...diagonal));
  const component = Math.sqrt(Math.max(0, diagonal[largest])) / 2;
  const divisor = 4 * component;
  const q = largest === 0
    ? [component, (matrix[7] - matrix[5]) / divisor,
      (matrix[2] - matrix[6]) / divisor, (matrix[3] - matrix[1]) / divisor]
    : largest === 1
      ? [(matrix[7] - matrix[5]) / divisor, component,
        (matrix[1] + matrix[3]) / divisor, (matrix[2] + matrix[6]) / divisor]
      : largest === 2
        ? [(matrix[2] - matrix[6]) / divisor, (matrix[1] + matrix[3]) / divisor,
          component, (matrix[5] + matrix[7]) / divisor]
        : [(matrix[3] - matrix[1]) / divisor, (matrix[2] + matrix[6]) / divisor,
          (matrix[5] + matrix[7]) / divisor, component];
  const norm = Math.hypot(...q);
  return q.map((value) => value / norm);
}
function nearestYaw(yaw, previous) {
  const wrapped = Math.atan2(Math.sin(yaw), Math.cos(yaw));
  if (Math.abs(Math.abs(wrapped) - Math.PI) < 1e-14)
    return Math.abs(Math.PI - previous) < Math.abs(-Math.PI - previous) - 1e-14
      ? Math.PI : -Math.PI;
  return wrapped;
}
function solve(relative, previous) {
  const candidates = [];
  const add = (yaw, pitch, roll) => {
    candidates.push({
      yaw: nearestYaw(yaw, previous.yaw),
      pitch: clamp(pitch, ...GIMBAL_LIMITS.pitch),
      roll: clamp(roll, ...GIMBAL_LIMITS.roll),
    });
  };
  const horizontal = Math.hypot(relative[0], relative[3]);
  const pitch = Math.atan2(relative[6], horizontal);
  const yaw = Math.atan2(relative[3], relative[0]);
  const roll = Math.atan2(relative[7], relative[8]);
  // Near a pitch pole, equivalent joint coordinates must not amplify matrix noise.
  const held = { yaw: previous.yaw, pitch, roll: previous.roll };
  if (orientationError(gimbalRotation(held), relative) <= 1e-14) return held;
  add(held.yaw, held.pitch, held.roll);
  if (roll >= GIMBAL_LIMITS.roll[0] && roll <= GIMBAL_LIMITS.roll[1]) {
    // Away from machine-zero poles the feasible Euler representation is exact.
    if (horizontal > 32 * Number.EPSILON)
      return { yaw: nearestYaw(yaw, previous.yaw), pitch, roll };
    add(yaw, pitch, roll);
  }

  // A nonzero-error interior stationary rotation cannot be a minimum; fit the
  // roll boundaries and pitch poles. Fixed-roll fitting is a 2x2 eigenproblem.
  for (const fixedRoll of [...GIMBAL_LIMITS.roll, previous.roll]) {
    const removed = multiply(relative, gimbalRotation({ yaw: 0, pitch: 0, roll: -fixedRoll }));
    const [w, x, y, z] = rotationQuaternion(removed);
    const a = w * w + z * z, b = -w * y + z * x, d = y * y + x * x;
    const direction = Math.atan2(2 * b, a - d) / 2;
    const halves = [-Math.PI / 4, Math.PI / 4, previous.pitch / 2];
    for (const turn of [-Math.PI, 0, Math.PI]) {
      const half = direction + turn;
      if (half >= -Math.PI / 4 && half <= Math.PI / 4) halves.push(half);
    }
    for (const half of halves) {
      const c = Math.cos(half), s = Math.sin(half);
      const u = w * c - y * s, v = z * c + x * s;
      add(Math.hypot(u, v) < 1e-15 ? previous.yaw : 2 * Math.atan2(v, u),
        2 * half, fixedRoll);
    }
  }
  // At either pitch pole, yaw +/- roll is the observable angle. Project the
  // previous joints onto that line, with bounds, for minimum joint travel.
  for (const sign of [-1, 1]) {
    const polePitch = sign * Math.PI / 2;
    const fixed = candidates.filter((value) => value.pitch === polePitch);
    for (const reference of fixed) {
      const combination = reference.yaw + sign * reference.roll;
      for (const turn of [-1, 0, 1]) {
        const total = combination + turn * 2 * Math.PI;
        const minimum = Math.max(-Math.PI, total - Math.PI / 4);
        const maximum = Math.min(Math.PI, total + Math.PI / 4);
        if (minimum > maximum) continue;
        const nextYaw = clamp((previous.yaw + total - sign * previous.roll) / 2,
          minimum, maximum);
        add(nextYaw, polePitch, sign * (total - nextYaw));
      }
    }
  }
  let best, bestError = Infinity, bestTravel = Infinity;
  for (const candidate of candidates) {
    const error = orientationError(gimbalRotation(candidate), relative);
    const travel = AXES.reduce((sum, axis) => sum + (candidate[axis] - previous[axis]) ** 2, 0);
    if (error < bestError - 1e-14 ||
        (Math.abs(error - bestError) <= 1e-14 && travel < bestTravel)) {
      best = candidate;
      bestError = error;
      bestTravel = travel;
    }
  }
  return best;
}
export function validateGimbalState(value, quaternion) {
  object(
    value,
    [
      "mode",
      "target",
      "maxRate",
      "actual",
      "limited",
      "rateLimited",
      "trackingErrorRad",
    ],
    "gimbal state",
  );
  const settings = validateGimbalSettings({
      mode: value.mode,
      target: value.target,
      maxRate: value.maxRate,
    }),
    actual = angles(value.actual, "actual");
  const flags = (key) => {
    object(value[key], AXES, key);
    return Object.fromEntries(
      AXES.map((axis) => {
        if (typeof value[key][axis] !== "boolean")
          throw Error(`Invalid gimbal ${key}.${axis}`);
        return [axis, value[key][axis]];
      }),
    );
  };
  const limited = flags("limited"),
    rateLimited = flags("rateLimited"),
    trackingErrorRad = finite(
      value.trackingErrorRad,
      0,
      Math.PI,
      "trackingErrorRad",
    );
  if (quaternion !== undefined) {
    const body = bodyRotation(quaternion),
      error = orientationError(
        multiply(body, gimbalRotation(actual)),
        goal(settings, body),
      );
    if (Math.abs(error - trackingErrorRad) > 1e-9)
      throw Error("Gimbal tracking error does not match pose");
  }
  return { ...settings, actual, limited, rateLimited, trackingErrorRad };
}
export function createGimbal(
  input = DEFAULT_GIMBAL_SETTINGS,
  initialQuaternion = [1, 0, 0, 0],
) {
  const initial = validateGimbalSettings(input);
  bodyRotation(initialQuaternion);
  const initialPose = [...initialQuaternion];
  let settings, actual, limited, rateLimited, trackingErrorRad;
  function update(dt, quaternion) {
    finite(dt, 0, 0.05, "time step");
    const nextBody = bodyRotation(quaternion),
      desiredWorld = goal(settings, nextBody);
    const demand =
      settings.mode === "body"
        ? settings.target
        : solve(multiply(transpose(nextBody), desiredWorld), actual);
    const demandError = settings.mode === "ned"
      ? orientationError(gimbalRotation(demand), multiply(transpose(nextBody), desiredWorld))
      : 0;
    const nextActual = {},
      nextLimited = {},
      nextRateLimited = {};
    for (const axis of AXES) {
      nextLimited[axis] = demandError > 1e-10 && GIMBAL_LIMITS[axis].some(
        (limit) => Math.abs(demand[axis] - limit) < 1e-12,
      );
      const delta = demand[axis] - actual[axis],
        travel = settings.maxRate[axis] * dt;
      nextActual[axis] = clamp(
        actual[axis] + clamp(delta, -travel, travel),
        ...GIMBAL_LIMITS[axis],
      );
      nextRateLimited[axis] = Math.abs(delta) > travel + 1e-10;
    }
    const nextError = orientationError(
      multiply(nextBody, gimbalRotation(nextActual)),
      desiredWorld,
    );
    actual = nextActual;
    limited = nextLimited;
    rateLimited = nextRateLimited;
    trackingErrorRad = nextError;
    return snapshot();
  }
  function set(value) {
    const validated = validateGimbalSettings(value);
    settings = validated;
    // A command changes demand only; paused joints never move.
    return update(0, lastQuaternion);
  }
  let lastQuaternion;
  function advance(dt, quaternion) {
    const result = update(dt, quaternion);
    lastQuaternion = [...quaternion];
    return result;
  }
  function reset(quaternion = initialPose) {
    const nextBody = bodyRotation(quaternion);
    settings = structuredClone(initial);
    const demand =
      settings.mode === "body"
        ? settings.target
        : solve(multiply(transpose(nextBody), goal(settings, nextBody)), {
            yaw: 0,
            pitch: 0,
            roll: 0,
          });
    actual = { ...demand };
    return advance(0, quaternion);
  }
  function snapshot() {
    return structuredClone({
      ...settings,
      actual,
      limited,
      rateLimited,
      trackingErrorRad,
    });
  }
  function checkpoint() {
    const saved = structuredClone({
      settings,
      actual,
      limited,
      rateLimited,
      trackingErrorRad,
      lastQuaternion,
    });
    let used = false;
    return () => {
      if (used) throw new Error("Gimbal rollback was already used");
      used = true;
      ({
        settings,
        actual,
        limited,
        rateLimited,
        trackingErrorRad,
        lastQuaternion,
      } = saved);
    };
  }
  reset();
  return { set, update: advance, snapshot, reset, checkpoint };
}
