// SI units; NED world, FRD body; scalar-first quaternion rotates body into world.
// Equations/conventions: https://github.com/bobzwik/Quadcopter_SimCon
export const DEFAULT_CONFIG = Object.freeze({
  mass: 1.2,
  arm: 0.23,
  motorTau: 0.04,
  maxMotorThrust: 8,
  drag: 0.25,
  attitudeKp: 6,
  rateKp: 0.12,
  rateKi: 0.02,
  altitudeKp: 3,
  altitudeKd: 2.5,
  wind: 0,
  noise: 0,
  latency: 0,
  dropout: 0,
  seed: 42,
  windDirection: 0,
  turbulence: 0,
  geofenceRadius: 100,
  geofenceHeight: 100,
  batteryCapacityWh: 60,
  batteryInitialSoc: 1,
  batteryBasePower: 20,
  batteryMotorPower: 160,
  batteryInternalResistance: 0.05,
  batteryCells: 4,
  lowBatteryThreshold: 0.2,
  criticalBatteryThreshold: 0.07,
});
const bounds = {
  mass: [0.1, 20],
  arm: [0.05, 2],
  motorTau: [0.01, 1],
  maxMotorThrust: [0.1, 200],
  drag: [0, 10],
  attitudeKp: [0, 20],
  rateKp: [0, 2],
  rateKi: [0, 1],
  altitudeKp: [0, 20],
  altitudeKd: [0, 20],
  wind: [0, 30],
  noise: [0, 10],
  latency: [0, 2],
  dropout: [0, 1],
  seed: [0, 4294967295],
  windDirection: [0, 360],
  turbulence: [0, 15],
  geofenceRadius: [1, 500],
  geofenceHeight: [2, 500],
  batteryCapacityWh: [0.1, 1000],
  batteryInitialSoc: [0, 1],
  batteryBasePower: [0, 200],
  batteryMotorPower: [1, 1000],
  batteryInternalResistance: [0, 2],
  batteryCells: [1, 12],
  lowBatteryThreshold: [0.01, 0.5],
  criticalBatteryThreshold: [0, 0.3],
};
export function validateConfig(config = {}) {
  if (!config || typeof config !== "object" || Array.isArray(config))
    throw new Error("Configuration must be an object");
  for (const [key, value] of Object.entries(config)) {
    if (!Object.hasOwn(bounds, key))
      throw new Error(`Unknown configuration: ${key}`);
    const [min, max] = bounds[key];
    if (!Number.isFinite(value) || value < min || value > max)
      throw new Error(`${key} must be finite and between ${min} and ${max}`);
  }
  const result = { ...DEFAULT_CONFIG, ...config };
  if (!Number.isInteger(result.seed))
    throw new Error("seed must be an integer");
  if (result.maxMotorThrust * 4 <= result.mass * 9.80665)
    throw new Error("maxMotorThrust must provide more than vehicle weight");
  if (!Number.isInteger(result.batteryCells))
    throw new Error("batteryCells must be an integer");
  if (result.criticalBatteryThreshold >= result.lowBatteryThreshold)
    throw new Error(
      "criticalBatteryThreshold must be below lowBatteryThreshold",
    );
  return result;
}
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const G = 9.80665;
function rotation([w, x, y, z]) {
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
function angles(q) {
  const r = rotation(q);
  return [
    Math.atan2(r[7], r[8]),
    Math.asin(clamp(-r[6], -1, 1)),
    Math.atan2(r[3], r[0]),
  ];
}
export function createSimulation(configuration = {}) {
  const config = validateConfig(configuration);
  const lever = config.arm / Math.SQRT2;
  const inertia = [0.5, 0.5, 1].map((v) => v * config.mass * config.arm ** 2);
  // Motors: front-right, rear-right, rear-left, front-left. Positive torque signs.
  const rollSign = [-1, -1, 1, 1],
    pitchSign = [1, -1, -1, 1],
    yawSign = [1, -1, 1, -1];
  const yawLever = 0.02;
  let state,
    time,
    target,
    baseTarget,
    scenario,
    scenarioStart,
    commands,
    integrals;
  let randomState,
    queue,
    observed,
    imu,
    dropped,
    steps,
    squaredError,
    peakError,
    altitudeSquaredError,
    altitudePeakError,
    failed,
    wind,
    gustUntil,
    gustVector;
  let navigation,
    positionTarget,
    explicitPosition,
    horizontalIntegral,
    holdSince,
    rtlPhase;
  let armed,
    crashed,
    faults,
    obstacles,
    usedWh,
    battery,
    safetyReason,
    turbulenceState,
    turbulenceGoal,
    nextTurbulence;
  function random() {
    randomState = (Math.imul(1664525, randomState) + 1013904223) >>> 0;
    return (randomState + 0.5) / 4294967296;
  }
  const noise = (sigma) =>
    config.noise === 0
      ? 0
      : config.noise *
        sigma *
        Math.sqrt(-2 * Math.log(random())) *
        Math.cos(2 * Math.PI * random());
  function reset() {
    time = 0;
    state = [
      0,
      0,
      -2,
      0,
      0,
      0,
      1,
      0,
      0,
      0,
      0,
      0,
      0,
      ...Array(4).fill(
        Math.sqrt((config.mass * G) / (4 * config.maxMotorThrust)),
      ),
    ];
    target = { altitude: 2, roll: 0, pitch: 0, yawRate: 0 };
    baseTarget = { ...target };
    scenario = "hover";
    scenarioStart = 0;
    commands = state.slice(13);
    integrals = [0, 0, 0];
    randomState = config.seed >>> 0;
    queue = [];
    dropped = 0;
    steps = 0;
    squaredError = 0;
    peakError = 0;
    altitudeSquaredError = 0;
    altitudePeakError = 0;
    failed = false;
    navigation = {
      mode: "attitude",
      waypoints: [],
      index: 0,
      completed: false,
      home: [0, 0],
      targetPosition: [0, 0],
      fenceBreached: false,
      reason: "",
    };
    positionTarget = { north: 0, east: 0, altitude: 2 };
    explicitPosition = false;
    horizontalIntegral = [0, 0];
    holdSince = null;
    rtlPhase = "";
    armed = true;
    crashed = false;
    faults = { motorEfficiency: [1, 1, 1, 1], linkLost: false };
    obstacles = [];
    usedWh = 0;
    safetyReason = "";
    turbulenceState = [0, 0, 0];
    turbulenceGoal = [0, 0, 0];
    nextTurbulence = 0;
    wind = currentWind();
    gustUntil = 0;
    gustVector = [0, 0, 0];
    observed = {
      time: 0,
      attitude: [0, 0, 0],
      rates: [0, 0, 0],
      altitude: 2,
      downSpeed: 0,
      position: [0, 0],
      velocity: [0, 0],
    };
    imu = { accel: [0, 0, -G], gyro: [0, 0, 0] };
    battery = batterySnapshot();
    return snapshot();
  }
  function derivative(s) {
    if (crashed || (!armed && s[2] >= -0.001)) return Array(17).fill(0);
    const r = rotation(s.slice(6, 10));
    // Motor states/commands are normalized rotor speeds; thrust is proportional to speed squared.
    const thrusts = s
      .slice(13)
      .map(
        (v, i) =>
          clamp(v, 0, 1) ** 2 *
          config.maxMotorThrust *
          faults.motorEfficiency[i],
      );
    const thrust = thrusts.reduce((a, b) => a + b, 0);
    const acceleration = [0, 1, 2].map(
      (i) =>
        (-r[i * 3 + 2] * thrust) / config.mass -
        (config.drag * (s[i + 3] - wind[i])) / config.mass +
        (i === 2 ? G : 0),
    );
    const torque = [rollSign, pitchSign, yawSign].map(
      (signs, axis) =>
        thrusts.reduce((sum, f, i) => sum + f * signs[i], 0) *
        (axis === 2 ? yawLever : lever),
    );
    const [p, q, rRate] = s.slice(10, 13),
      [w, x, y, z] = s.slice(6, 10);
    // Euler rigid-body equation includes omega cross J*omega.
    const angular = [
      (torque[0] + (inertia[1] - inertia[2]) * q * rRate) / inertia[0],
      (torque[1] + (inertia[2] - inertia[0]) * rRate * p) / inertia[1],
      (torque[2] + (inertia[0] - inertia[1]) * p * q) / inertia[2],
    ];
    return [
      ...s.slice(3, 6),
      ...acceleration,
      -(x * p + y * q + z * rRate) / 2,
      (w * p + y * rRate - z * q) / 2,
      (w * q + z * p - x * rRate) / 2,
      (w * rRate + x * q - y * p) / 2,
      ...angular,
      ...commands.map((v, i) => (v - s[13 + i]) / config.motorTau),
    ];
  }
  function sample() {
    const d = derivative(state),
      r = rotation(state.slice(6, 10));
    const specific = d.slice(3, 6);
    specific[2] -= G;
    if (!armed && state[2] >= -0.001) specific.splice(0, 3, 0, 0, -G);
    // Independent per-sample Gaussian sigma at noise=1: accel .1m/s², gyro .01rad/s,
    // ideal attitude .005rad, altitude/position .02m, velocity .02m/s. No estimator/EKF/GPS model.
    imu = {
      accel: [0, 1, 2].map(
        (i) =>
          r[i] * specific[0] +
          r[3 + i] * specific[1] +
          r[6 + i] * specific[2] +
          noise(0.1),
      ),
      gyro: state.slice(10, 13).map((v) => v + noise(0.01)),
    };
    const observation = {
      time,
      attitude: angles(state.slice(6, 10)).map((v) => v + noise(0.005)),
      rates: [...imu.gyro],
      altitude: -state[2] + noise(0.02),
      downSpeed: state[5] + noise(0.02),
      position: state.slice(0, 2).map((v) => v + noise(0.02)),
      velocity: state.slice(3, 5).map((v) => v + noise(0.02)),
    };
    if (random() < config.dropout) dropped++;
    else queue.push(observation);
    while (queue.length && queue[0].time <= time - config.latency + 1e-10)
      observed = queue.shift();
  }
  function control(dt) {
    if (armed && time - observed.time > 0.5 + 1e-10) {
      failed = true;
      navigation.reason = "stale-sensor";
    }
    if (failed || !armed || crashed || battery.soc <= 0) {
      commands.fill(0);
      integrals.fill(0);
      return;
    }
    const [phi, theta] = observed.attitude;
    // Convert Euler angle demands and heading rate to body rates near the commanded attitude.
    const desiredRates = [
      config.attitudeKp * (target.roll - phi) -
        Math.sin(theta) * target.yawRate,
      config.attitudeKp * (target.pitch - theta) +
        Math.sin(phi) * Math.cos(theta) * target.yawRate,
      Math.cos(phi) * Math.cos(theta) * target.yawRate,
    ];
    const errors = desiredRates.map(
      (v, i) => clamp(v, -4, 4) - observed.rates[i],
    );
    const candidate = integrals.map((v, i) => clamp(v + errors[i] * dt, -2, 2));
    const torque = errors.map(
      (v, i) => config.rateKp * v + config.rateKi * candidate[i],
    );
    const upAcceleration = clamp(
      config.altitudeKp * (target.altitude - observed.altitude) +
        config.altitudeKd * observed.downSpeed,
      -G,
      G,
    );
    const total =
      (config.mass * (G + upAcceleration)) /
      Math.max(0.3, Math.cos(phi) * Math.cos(theta));
    const raw = rollSign.map(
      (_, i) =>
        (total +
          (rollSign[i] * torque[0]) / lever +
          (pitchSign[i] * torque[1]) / lever +
          (yawSign[i] * torque[2]) / yawLever) /
        (4 * config.maxMotorThrust),
    );
    commands = raw.map((v) => Math.sqrt(clamp(v, 0, 1)));
    // Conditional integration: freeze the integral while any motor is saturated.
    if (raw.every((v) => v >= 0 && v <= 1)) integrals = candidate;
  }
  function setTargets(values) {
    if (!values || typeof values !== "object" || Array.isArray(values))
      throw new Error("Targets must be an object");
    const limits = {
      altitude: [0, 500],
      roll: [-Math.PI / 6, Math.PI / 6],
      pitch: [-Math.PI / 6, Math.PI / 6],
      yawRate: [-2, 2],
    };
    for (const [key, value] of Object.entries(values)) {
      if (!Object.hasOwn(limits, key))
        throw new Error(`Unknown target: ${key}`);
      if (
        !Number.isFinite(value) ||
        value < limits[key][0] ||
        value > limits[key][1]
      )
        throw new Error(
          `${key} target must be between ${limits[key].join(" and ")}`,
        );
    }
    baseTarget = { ...baseTarget, ...values };
    target = { ...baseTarget };
    if (Object.hasOwn(values, "altitude") && navigation.mode === "position")
      positionTarget.altitude = values.altitude;
  }
  function validateFields(value, fields, label) {
    if (!value || typeof value !== "object" || Array.isArray(value))
      throw new Error(`${label} must be an object`);
    if (Object.keys(value).some((key) => !Object.hasOwn(fields, key)))
      throw new Error(`Unknown ${label} field`);
    for (const [key, [min, max]] of Object.entries(fields)) {
      if (!Number.isFinite(value[key]) || value[key] < min || value[key] > max)
        throw new Error(
          `${label}.${key} must be finite and between ${min} and ${max}`,
        );
    }
    return { ...value };
  }
  function validatePoint(point, hold = false) {
    const fields = {
      north: [-500, 500],
      east: [-500, 500],
      altitude: [0, 500],
    };
    if (hold) fields.hold = [0, 300];
    const value = validateFields(point, fields, "Waypoint");
    if (
      Math.hypot(value.north, value.east) > config.geofenceRadius ||
      value.altitude > config.geofenceHeight
    )
      throw new Error("Waypoint is outside the geofence");
    return value;
  }
  function setPositionTarget(value) {
    if (safetyReason || ["rtl", "land", "mission"].includes(navigation.mode))
      throw new Error(
        "Select position mode before changing a navigation target; safety return/landing cannot be overridden",
      );
    positionTarget = validatePoint(value);
    explicitPosition = true;
    navigation.targetPosition = [positionTarget.north, positionTarget.east];
  }
  function setMission(points) {
    if (!Array.isArray(points) || points.length < 1 || points.length > 50)
      throw new Error("Mission must contain 1 to 50 waypoints");
    const validated = Array.from(points, (value) => validatePoint(value, true));
    navigation.waypoints = validated;
    navigation.index = 0;
    navigation.completed = false;
    holdSince = null;
    if (navigation.mode === "mission") positionTarget = { ...validated[0] };
  }
  function enterMode(mode, reason = "") {
    navigation.mode = mode;
    navigation.reason = reason;
    navigation.completed = false;
    horizontalIntegral = [0, 0];
    holdSince = null;
    if (mode === "attitude") {
      baseTarget = {
        altitude: Math.max(0, -state[2]),
        roll: 0,
        pitch: 0,
        yawRate: 0,
      };
      target = { ...baseTarget };
    } else if (mode === "position") {
      // Capture current position unless an explicit target was supplied immediately beforehand.
      if (!explicitPosition)
        positionTarget = {
          north: state[0],
          east: state[1],
          altitude: Math.max(0, -state[2]),
        };
    } else if (mode === "mission") {
      navigation.index = 0;
      positionTarget = { ...navigation.waypoints[0] };
    } else if (mode === "rtl") {
      rtlPhase = "climb";
      positionTarget = {
        north: state[0],
        east: state[1],
        altitude: Math.min(
          config.geofenceHeight - 0.25,
          Math.max(3, -state[2]),
        ),
      };
    } else if (mode === "land") {
      positionTarget = {
        north: state[0],
        east: state[1],
        altitude: Math.max(0, -state[2]),
      };
    }
    explicitPosition = false;
    navigation.targetPosition = [positionTarget.north, positionTarget.east];
  }
  function setMode(mode) {
    if (!["attitude", "position", "mission", "rtl", "land"].includes(mode))
      throw new Error("Unknown flight mode");
    if (crashed || failed)
      throw new Error("Reset is required after crash or stale-sensor failsafe");
    if (mode === "mission" && !navigation.waypoints.length)
      throw new Error("Load a mission before selecting mission mode");
    if (safetyReason && !["rtl", "land"].includes(mode)) {
      if (safetyReason === "link-loss" && !faults.linkLost) safetyReason = "";
      else
        throw new Error(
          "Safety return/landing is active; reset before changing flight mode",
        );
    }
    if (safetyReason === "critical-battery" && mode !== "land")
      throw new Error("Critical battery requires landing");
    enterMode(mode, safetyReason);
  }
  function arm() {
    if (crashed || failed || battery.critical || faults.linkLost)
      throw new Error(
        "Cannot arm after crash, failsafe, critical battery or lost link",
      );
    armed = true;
    integrals.fill(0);
  }
  function disarm() {
    armed = false;
    commands.fill(0);
    integrals.fill(0);
  }
  function takeoff() {
    if (safetyReason)
      throw new Error(
        "Cannot take off during a safety return/landing; reset first",
      );
    arm();
    positionTarget = { north: state[0], east: state[1], altitude: 2 };
    explicitPosition = true;
    enterMode("position");
  }
  function injectFault(kind, value) {
    if (kind === "motor") {
      const checked = validateFields(
        value,
        { index: [0, 3], efficiency: [0, 1] },
        "Motor fault",
      );
      if (!Number.isInteger(checked.index))
        throw new Error("Motor index must be an integer");
      faults.motorEfficiency[checked.index] = checked.efficiency;
    } else if (kind === "link") {
      if (
        !value ||
        typeof value !== "object" ||
        Array.isArray(value) ||
        Object.keys(value).length !== 1 ||
        typeof value.lost !== "boolean"
      )
        throw new Error("Link fault must contain lost:boolean");
      faults.linkLost = value.lost;
    } else throw new Error("Unknown fault type");
  }
  function clearFaults() {
    faults = { motorEfficiency: [1, 1, 1, 1], linkLost: false };
  }
  function setObstacles(boxes) {
    if (!Array.isArray(boxes) || boxes.length > 50)
      throw new Error("Obstacles must be an array of at most 50 boxes");
    obstacles = Array.from(boxes, (box) =>
      validateFields(
        box,
        {
          north: [-500, 500],
          east: [-500, 500],
          width: [0.01, 100],
          depth: [0.01, 100],
          height: [0.01, 500],
        },
        "Obstacle",
      ),
    );
  }
  function intersectsBox(from, to, box) {
    // shortcut: swept .25m collision envelope is conservative at box corners; use mesh contact for precise landing research.
    const min = [
      box.north - box.width / 2 - 0.25,
      box.east - box.depth / 2 - 0.25,
      -box.height - 0.25,
    ];
    const max = [
      box.north + box.width / 2 + 0.25,
      box.east + box.depth / 2 + 0.25,
      0.25,
    ];
    let first = 0,
      last = 1;
    for (let axis = 0; axis < 3; axis++) {
      const delta = to[axis] - from[axis];
      if (Math.abs(delta) < 1e-12) {
        if (from[axis] < min[axis] || from[axis] > max[axis]) return false;
      } else {
        const a = (min[axis] - from[axis]) / delta,
          b = (max[axis] - from[axis]) / delta;
        first = Math.max(first, Math.min(a, b));
        last = Math.min(last, Math.max(a, b));
        if (first > last) return false;
      }
    }
    return true;
  }
  function batterySnapshot() {
    const availableWh = Math.max(
      0,
      config.batteryCapacityWh * config.batteryInitialSoc - usedWh,
    );
    const soc = availableWh / config.batteryCapacityWh;
    const openVoltage = config.batteryCells * (3.3 + 0.9 * soc);
    const demandedPower =
      availableWh > 0
        ? config.batteryBasePower +
          config.batteryMotorPower *
            state.slice(13).reduce((sum, speed) => sum + speed ** 3, 0)
        : 0;
    const resistance = config.batteryInternalResistance;
    // Simplified Thevenin pack; cap electrical demand at maximum deliverable power.
    const power = resistance
      ? Math.min(demandedPower, (0.99 * openVoltage ** 2) / (4 * resistance))
      : demandedPower;
    const voltage = resistance
      ? (openVoltage +
          Math.sqrt(Math.max(0, openVoltage ** 2 - 4 * resistance * power))) /
        2
      : openVoltage;
    const current = power / Math.max(0.1, voltage);
    return {
      soc,
      voltage,
      current,
      power,
      usedWh,
      remainingSeconds: power > 0 ? (availableWh * 3600) / power : 0,
      low: soc <= config.lowBatteryThreshold,
      critical: soc <= config.criticalBatteryThreshold,
    };
  }
  function safetyMode(mode, reason) {
    if (safetyReason === reason) return;
    safetyReason = reason;
    enterMode(mode, reason);
  }
  function navigate(dt) {
    if (!armed || crashed || failed) return;
    if (battery.critical) safetyMode("land", "critical-battery");
    else if (safetyReason !== "critical-battery") {
      if (faults.linkLost && !safetyReason) safetyMode("rtl", "link-loss");
      if (
        (Math.hypot(state[0], state[1]) > config.geofenceRadius ||
          -state[2] > config.geofenceHeight) &&
        !navigation.fenceBreached
      ) {
        navigation.fenceBreached = true;
        safetyMode("rtl", "geofence");
      }
      if (battery.low && !safetyReason && navigation.mode !== "land")
        safetyMode("rtl", "low-battery");
    }
    if (navigation.mode === "attitude") return;
    if (navigation.mode === "mission") {
      const waypoint = navigation.waypoints[navigation.index];
      const close =
        Math.hypot(
          positionTarget.north - observed.position[0],
          positionTarget.east - observed.position[1],
        ) < 0.75 && Math.abs(positionTarget.altitude - observed.altitude) < 0.4;
      if (close) {
        if (holdSince === null) holdSince = time;
        if (time - holdSince >= waypoint.hold) {
          if (navigation.index + 1 < navigation.waypoints.length) {
            navigation.index++;
            positionTarget = { ...navigation.waypoints[navigation.index] };
            holdSince = null;
          } else navigation.completed = true;
        }
      } else holdSince = null;
    } else if (navigation.mode === "rtl") {
      if (
        rtlPhase === "climb" &&
        Math.abs(positionTarget.altitude - observed.altitude) < 0.4
      ) {
        rtlPhase = "return";
        positionTarget.north = 0;
        positionTarget.east = 0;
      }
      if (
        rtlPhase === "return" &&
        Math.hypot(...observed.position) < 0.75 &&
        Math.hypot(...observed.velocity) < 0.5
      ) {
        enterMode("land", navigation.reason || "rtl");
        positionTarget.north = 0;
        positionTarget.east = 0;
      }
    }
    if (navigation.mode === "land")
      positionTarget.altitude = Math.max(0, positionTarget.altitude - 0.5 * dt);
    const errors = [
      positionTarget.north - observed.position[0],
      positionTarget.east - observed.position[1],
    ];
    if (Math.hypot(...errors) < 5)
      horizontalIntegral = horizontalIntegral.map((v, i) =>
        clamp(v + errors[i] * dt, -6, 6),
      );
    const velocityScale = Math.min(
      0.7,
      3 / Math.max(1e-9, Math.hypot(...errors)),
    );
    let acceleration = errors.map(
      (v, i) =>
        2.2 * (v * velocityScale - observed.velocity[i]) +
        0.12 * horizontalIntegral[i],
    );
    const scale = Math.min(1, 3 / Math.max(1e-9, Math.hypot(...acceleration)));
    acceleration = acceleration.map((v) => v * scale);
    const yaw = observed.attitude[2],
      north = acceleration[0],
      east = acceleration[1];
    target = {
      altitude: positionTarget.altitude,
      roll: Math.atan2(-Math.sin(yaw) * north + Math.cos(yaw) * east, G),
      pitch: -Math.atan2(Math.cos(yaw) * north + Math.sin(yaw) * east, G),
      yawRate: 0,
    };
    navigation.targetPosition = [positionTarget.north, positionTarget.east];
  }
  function gust() {
    gustVector = [4, 3, -1];
    gustUntil = time + 1;
    wind = currentWind();
  }
  function setScenario(value) {
    if (!["hover", "step", "gust"].includes(value))
      throw new Error("Unknown scenario");
    scenario = value;
    scenarioStart = time;
    target = { ...baseTarget };
    if (value === "gust") gust();
  }
  function currentWind() {
    const periodicGust =
      scenario === "gust" && (time - scenarioStart + 1e-10) % 8 < 1;
    const heading = (config.windDirection * Math.PI) / 180;
    return [
      config.wind * Math.cos(heading),
      config.wind * Math.sin(heading),
      0,
    ].map(
      (v, i) =>
        v +
        turbulenceState[i] +
        (time < gustUntil - 1e-10
          ? gustVector[i]
          : periodicGust
            ? [4, 3, -1][i]
            : 0),
    );
  }
  function step(dt = 0.005) {
    if (!Number.isFinite(dt) || dt <= 0 || dt > 0.02)
      throw new Error("dt must be greater than zero and at most 0.02 seconds");
    const previousTarget = target;
    const elapsed = time - scenarioStart;
    target = { ...baseTarget };
    if (scenario === "step" && elapsed >= 2 && elapsed < 6)
      target.roll = Math.PI / 18;
    if (config.turbulence > 0) {
      if (time >= nextTurbulence) {
        turbulenceGoal = [0, 1, 2].map(
          () => (random() * 2 - 1) * config.turbulence,
        );
        nextTurbulence = time + 0.5;
      }
      turbulenceState = turbulenceState.map(
        (v, i) => v + (turbulenceGoal[i] - v) * (1 - Math.exp(-dt / 0.4)),
      );
    }
    wind = currentWind();
    const previousImu = imu,
      previousCommands = commands;
    sample();
    navigate(dt);
    control(dt);
    const add = (s, k, f) => s.map((v, i) => v + k[i] * f);
    const a = derivative(state),
      b = derivative(add(state, a, dt / 2));
    const c = derivative(add(state, b, dt / 2)),
      d = derivative(add(state, c, dt));
    const nextState = state.map(
      (v, i) => v + (dt * (a[i] + 2 * b[i] + 2 * c[i] + d[i])) / 6,
    );
    const norm = Math.hypot(...nextState.slice(6, 10));
    if (
      !nextState.every(Number.isFinite) ||
      !Number.isFinite(norm) ||
      norm < 1e-12 ||
      !Number.isFinite(
        altitudeSquaredError + (target.altitude + nextState[2]) ** 2,
      )
    ) {
      imu = previousImu;
      commands = previousCommands;
      target = previousTarget;
      throw new Error(
        "Numerical integration diverged; reduce controller gains/noise or use a smaller integration step",
      );
    }
    const previousPosition = state.slice(0, 3);
    if (!crashed) {
      state = nextState;
      for (let i = 6; i < 10; i++) state[i] /= norm;
    }
    for (let i = 13; i < 17; i++) state[i] = clamp(state[i], 0, 1);
    // shortcut: flat inelastic ground, add a contact solver before terrain/landing research.
    if (state[2] >= 0) {
      state[2] = 0;
      state[5] = Math.min(0, state[5]);
      state[3] *= Math.exp(-8 * dt);
      state[4] *= Math.exp(-8 * dt);
    }
    if (
      !crashed &&
      armed &&
      obstacles.some((box) =>
        intersectsBox(previousPosition, state.slice(0, 3), box),
      )
    ) {
      crashed = true;
      navigation.reason = "obstacle-collision";
      disarm();
      state.splice(3, 3, 0, 0, 0);
      state.splice(10, 7, 0, 0, 0, 0, 0, 0, 0);
    }
    if (
      !crashed &&
      navigation.mode === "land" &&
      state[2] > -0.12 &&
      Math.abs(state[5]) < 0.6
    ) {
      state[2] = 0;
      state.splice(3, 3, 0, 0, 0);
      navigation.completed = true;
      disarm();
    }
    if (crashed || (!armed && state[2] >= -0.001)) {
      state.splice(3, 3, 0, 0, 0);
      state.splice(10, 7, 0, 0, 0, 0, 0, 0, 0);
    }
    usedWh = Math.min(
      config.batteryCapacityWh * config.batteryInitialSoc,
      usedWh + (batterySnapshot().power * dt) / 3600,
    );
    battery = batterySnapshot();
    if (battery.soc <= 0) disarm();
    time += dt;
    steps++;
    wind = currentWind();
    // Cumulative roll/pitch tracking norm in radians; altitude tracked separately in meters.
    const attitude = angles(state.slice(6, 10));
    const attitudeError = Math.hypot(
      target.roll - attitude[0],
      target.pitch - attitude[1],
    );
    squaredError += attitudeError ** 2;
    peakError = Math.max(peakError, attitudeError);
    const altitudeError = target.altitude + state[2];
    altitudeSquaredError += altitudeError ** 2;
    altitudePeakError = Math.max(altitudePeakError, Math.abs(altitudeError));
    return snapshot();
  }
  function snapshot() {
    return {
      time,
      position: state.slice(0, 3),
      velocity: state.slice(3, 6),
      quaternion: state.slice(6, 10),
      attitude: angles(state.slice(6, 10)),
      rates: state.slice(10, 13),
      motors: state.slice(13),
      pwm: commands.map((v) => 1000 + v * 1000),
      target: { ...target },
      imu: { accel: [...imu.accel], gyro: [...imu.gyro] },
      metrics: {
        rmse: steps ? Math.sqrt(squaredError / steps) : 0,
        peakError,
        altitudeRmse: steps ? Math.sqrt(altitudeSquaredError / steps) : 0,
        altitudePeakError,
        sampleAge: time - observed.time,
        dropped,
        steps,
      },
      navigation: {
        ...navigation,
        waypoints: navigation.waypoints.map((v) => ({ ...v })),
        home: [...navigation.home],
        targetPosition: [...navigation.targetPosition],
      },
      battery: { ...battery },
      faults: {
        motorEfficiency: [...faults.motorEfficiency],
        linkLost: faults.linkLost,
      },
      armed,
      status: crashed
        ? "crashed"
        : failed
          ? "failsafe"
          : state[2] >= -0.001
            ? "landed"
            : steps
              ? "flying"
              : "ready",
      wind: [...wind],
    };
  }
  reset();
  return {
    setTargets,
    setScenario,
    gust,
    step,
    snapshot,
    reset,
    setMode,
    setPositionTarget,
    setMission,
    takeoff,
    arm,
    disarm,
    injectFault,
    clearFaults,
    setObstacles,
  };
}
