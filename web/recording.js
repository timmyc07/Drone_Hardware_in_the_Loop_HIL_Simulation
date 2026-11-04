import { validateConfig } from "./sim.js";

const targetLimits = {
  altitude: [0, 100],
  roll: [-Math.PI / 6, Math.PI / 6],
  pitch: [-Math.PI / 6, Math.PI / 6],
  yawRate: [-2, 2],
};
const object = (value) => {
  if (
    !value ||
    typeof value !== "object" ||
    Array.isArray(value) ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value))
  )
    throw new Error("Expected a plain object");
  return value;
};
const number = (value, min = -1e9, max = 1e9) => {
  if (!Number.isFinite(value) || value < min || value > max)
    throw new Error("Invalid number");
  return value;
};
const vector = (value, length, min = -1e9, max = 1e9) => {
  if (!Array.isArray(value) || value.length !== length)
    throw new Error("Invalid vector");
  return Array.from(value, (v) => number(v, min, max));
};
function targets(value, partial = false) {
  object(value);
  for (const key of Object.keys(value))
    if (!Object.hasOwn(targetLimits, key)) throw new Error("Unknown target");
  const result = {};
  for (const [key, [min, max]] of Object.entries(targetLimits)) {
    if (!partial || Object.hasOwn(value, key))
      result[key] = number(value[key], min, max);
  }
  return result;
}
function sample(value) {
  object(value);
  const quaternion = vector(value.quaternion, 4, -1, 1);
  if (Math.abs(Math.hypot(...quaternion) - 1) > 0.01)
    throw new Error("Invalid quaternion");
  const imu = object(value.imu),
    metrics = object(value.metrics);
  if (!["ready", "flying", "landed", "failsafe"].includes(value.status))
    throw new Error("Invalid status");
  const resultMetrics = {};
  for (const key of ["rmse", "peakError", "sampleAge", "dropped", "steps"])
    resultMetrics[key] = number(metrics[key], 0);
  for (const key of ["dropped", "steps"])
    if (!Number.isInteger(resultMetrics[key])) throw new Error("Invalid count");
  for (const key of ["altitudeRmse", "altitudePeakError"])
    if (Object.hasOwn(metrics, key))
      resultMetrics[key] = number(metrics[key], 0);
  return {
    time: number(value.time, 0),
    position: vector(value.position, 3),
    velocity: vector(value.velocity, 3),
    quaternion,
    attitude: vector(value.attitude, 3),
    rates: vector(value.rates, 3),
    motors: vector(value.motors, 4, 0, 1),
    pwm: vector(value.pwm, 4, 1000, 2000),
    target: targets(value.target),
    imu: { accel: vector(imu.accel, 3), gyro: vector(imu.gyro, 3) },
    metrics: resultMetrics,
    status: value.status,
    wind: vector(value.wind, 3),
  };
}
export function validateRecording(value) {
  object(value);
  if (
    value.format !== "flight-lab" ||
    value.version !== 1 ||
    typeof value.createdAt !== "string" ||
    !Number.isFinite(Date.parse(value.createdAt))
  )
    throw new Error("Invalid recording header");
  if (!["hover", "step", "gust"].includes(value.scenario) || value.dt !== 0.005)
    throw new Error("Invalid recording settings");
  object(value.config);
  const config = validateConfig(value.config);
  if (
    !Array.isArray(value.events) ||
    !Array.isArray(value.samples) ||
    !value.samples.length ||
    value.samples.length > 36001
  )
    throw new Error("Invalid recording length");
  let lastEvent = -1;
  const events = value.events.map((event) => {
    object(event);
    const time = number(event.time, 0);
    if (time < lastEvent) throw new Error("Events out of order");
    lastEvent = time;
    if (event.type === "targets")
      return { time, type: "targets", value: targets(event.value, true) };
    if (event.type === "gust") return { time, type: "gust" };
    throw new Error("Invalid event");
  });
  let lastTime = -1;
  const samples = value.samples.map((value) => {
    const result = sample(value);
    if (result.time <= lastTime) throw new Error("Samples out of order");
    lastTime = result.time;
    return result;
  });
  return {
    format: "flight-lab",
    version: 1,
    createdAt: value.createdAt,
    config,
    scenario: value.scenario,
    dt: 0.005,
    events,
    samples,
  };
}
export function recordingToCsv(record) {
  const { samples } = validateRecording(record);
  const header = [
    "time_s",
    "north_m",
    "east_m",
    "down_m",
    "velocity_n_m_s",
    "velocity_e_m_s",
    "velocity_d_m_s",
    "quat_w",
    "quat_x",
    "quat_y",
    "quat_z",
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
    "rate_p_rad_s",
    "rate_q_rad_s",
    "rate_r_rad_s",
    ...[1, 2, 3, 4].map((i) => `motor_${i}_normalized`),
    ...[1, 2, 3, 4].map((i) => `pwm_${i}_us`),
    "target_altitude_m",
    "target_roll_rad",
    "target_pitch_rad",
    "target_yaw_rate_rad_s",
    "accel_x_m_s2",
    "accel_y_m_s2",
    "accel_z_m_s2",
    "gyro_x_rad_s",
    "gyro_y_rad_s",
    "gyro_z_rad_s",
    "wind_n_m_s",
    "wind_e_m_s",
    "wind_d_m_s",
    "attitude_rmse_rad",
    "attitude_peak_error_rad",
    "altitude_rmse_m",
    "altitude_peak_error_m",
    "sample_age_s",
    "dropped_count",
    "steps_count",
  ];
  const rows = samples.map((s) =>
    [
      s.time,
      ...s.position,
      ...s.velocity,
      ...s.quaternion,
      ...s.attitude,
      ...s.rates,
      ...s.motors,
      ...s.pwm,
      s.target.altitude,
      s.target.roll,
      s.target.pitch,
      s.target.yawRate,
      ...s.imu.accel,
      ...s.imu.gyro,
      ...s.wind,
      s.metrics.rmse,
      s.metrics.peakError,
      s.metrics.altitudeRmse ?? "",
      s.metrics.altitudePeakError ?? "",
      s.metrics.sampleAge,
      s.metrics.dropped,
      s.metrics.steps,
    ].join(","),
  );
  return [header.join(","), ...rows].join("\n");
}
