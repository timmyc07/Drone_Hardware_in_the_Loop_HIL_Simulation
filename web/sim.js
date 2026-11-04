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
    wind = [config.wind, 0, 0];
    gustUntil = 0;
    gustVector = [0, 0, 0];
    observed = {
      time: 0,
      attitude: [0, 0, 0],
      rates: [0, 0, 0],
      altitude: 2,
      downSpeed: 0,
    };
    imu = { accel: [0, 0, -G], gyro: [0, 0, 0] };
    return snapshot();
  }
  function derivative(s) {
    const r = rotation(s.slice(6, 10));
    // Motor states/commands are normalized rotor speeds; thrust is proportional to speed squared.
    const thrusts = s
      .slice(13)
      .map((v) => clamp(v, 0, 1) ** 2 * config.maxMotorThrust);
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
    // Independent per-sample Gaussian sigma at noise=1: accel .1m/s², gyro .01rad/s,
    // ideal attitude .005rad, altitude .02m, vertical speed .02m/s. No estimator/EKF.
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
    };
    if (random() < config.dropout) dropped++;
    else queue.push(observation);
    while (queue.length && queue[0].time <= time - config.latency + 1e-10)
      observed = queue.shift();
  }
  function control(dt) {
    if (time - observed.time > 0.5 + 1e-10) failed = true;
    if (failed) {
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
      altitude: [0, 100],
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
    return [config.wind, 0, 0].map(
      (v, i) =>
        v +
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
    const elapsed = time - scenarioStart;
    target = { ...baseTarget };
    if (scenario === "step" && elapsed >= 2 && elapsed < 6)
      target.roll = Math.PI / 18;
    wind = currentWind();
    const previousImu = imu,
      previousCommands = commands;
    sample();
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
      throw new Error(
        "Numerical integration diverged; reduce controller gains/noise or use a smaller integration step",
      );
    }
    state = nextState;
    for (let i = 6; i < 10; i++) state[i] /= norm;
    for (let i = 13; i < 17; i++) state[i] = clamp(state[i], 0, 1);
    // shortcut: flat inelastic ground, add a contact solver before terrain/landing research.
    if (state[2] >= 0) {
      state[2] = 0;
      state[5] = Math.min(0, state[5]);
      state[3] *= Math.exp(-8 * dt);
      state[4] *= Math.exp(-8 * dt);
    }
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
      status: failed
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
  return { setTargets, setScenario, gust, step, snapshot, reset };
}
