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
function solve(relative, previous) {
  const pitch = Math.asin(clamp(relative[6], -1, 1));
  if (Math.abs(Math.cos(pitch)) > 1e-7) {
    let yaw = Math.atan2(relative[3], relative[0]);
    // At exactly a hard-stop endpoint both signed representations are feasible.
    if (
      Math.abs(Math.abs(yaw) - Math.PI) < 1e-12 &&
      Math.abs(-yaw - previous.yaw) < Math.abs(yaw - previous.yaw)
    )
      yaw = -yaw;
    return { yaw, pitch, roll: Math.atan2(relative[7], relative[8]) };
  }
  // At the pole only yaw+roll (down) or roll-yaw (up) is observable.
  // Select the feasible pair nearest the previous joints instead of snapping yaw to zero.
  const down = pitch > 0,
    combination = down
      ? Math.atan2(-relative[1], relative[4])
      : Math.atan2(relative[1], relative[4]);
  const candidates = [];
  for (let turn = -1; turn <= 1; turn++) {
    const total = combination + turn * 2 * Math.PI;
    const minYaw = Math.max(
      -Math.PI,
      down ? total - Math.PI / 4 : -total - Math.PI / 4,
    );
    const maxYaw = Math.min(
      Math.PI,
      down ? total + Math.PI / 4 : -total + Math.PI / 4,
    );
    if (minYaw > maxYaw) continue;
    const yaw = clamp(previous.yaw, minYaw, maxYaw),
      roll = down ? total - yaw : total + yaw;
    candidates.push({
      yaw,
      pitch,
      roll,
      cost: (yaw - previous.yaw) ** 2 + (roll - previous.roll) ** 2,
    });
  }
  candidates.sort((a, b) => a.cost - b.cost);
  if (!candidates.length)
    throw Error("Gimbal pole has no feasible angle representation");
  const { yaw, roll } = candidates[0];
  return { yaw, pitch, roll };
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
    const nextActual = {},
      nextLimited = {},
      nextRateLimited = {};
    for (const axis of AXES) {
      const bounded = clamp(demand[axis], ...GIMBAL_LIMITS[axis]);
      nextLimited[axis] = Math.abs(bounded - demand[axis]) > 1e-10;
      const delta = bounded - actual[axis],
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
    actual = Object.fromEntries(
      AXES.map((axis) => [axis, clamp(demand[axis], ...GIMBAL_LIMITS[axis])]),
    );
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
  reset();
  return { set, update: advance, snapshot, reset };
}
