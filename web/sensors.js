import { bodyToNedMatrix } from "./sim.js";

// Deterministic virtual observations only; independent of the flight controller.
// References: https://microsoft.github.io/AirSim/sensors/ https://microsoft.github.io/AirSim/lidar/
// Gaussian noise sigmas are per acquisition; no drift, constellation, WMM or calibration model.
// GPS is a local WGS84 tangent approximation; optical flow uses geometry, not rendered images.
export const DEFAULT_SENSOR_CONFIG = Object.freeze({
  gpsHz: 5,
  barometerHz: 20,
  magnetometerHz: 20,
  rangefinderHz: 20,
  lidarHz: 10,
  opticalFlowHz: 20,
  latency: 0,
  dropout: 0,
  originLatitudeDeg: 25.033,
  originLongitudeDeg: 121.5654,
  originAltitudeM: 20,
  gpsNoiseM: 1.5,
  gpsVerticalNoiseM: 2.5,
  gpsVelocityNoiseMps: 0.1,
  barometerNoisePa: 2.7,
  magneticFieldMicroT: 50,
  magneticDeclinationDeg: 0,
  magneticInclinationDeg: 60,
  magnetometerNoiseMicroT: 0.05,
  rangefinderMinM: 0.1,
  rangefinderMaxM: 40,
  rangefinderNoiseM: 0.01,
  lidarRays: 72,
  lidarMinM: 0.2,
  lidarMaxM: 50,
  lidarNoiseM: 0.01,
  opticalFlowNoiseRadS: 0.01,
});
const NAMES = [
  "gps",
  "barometer",
  "magnetometer",
  "rangefinder",
  "lidar",
  "opticalFlow",
];
const bounds = {
  ...Object.fromEntries(NAMES.map((name) => [`${name}Hz`, [1, 20]])),
  latency: [0, 1],
  dropout: [0, 1],
  originLatitudeDeg: [-85, 85],
  originLongitudeDeg: [-180, 180],
  originAltitudeM: [-500, 9000],
  gpsNoiseM: [0, 100],
  gpsVerticalNoiseM: [0, 100],
  gpsVelocityNoiseMps: [0, 10],
  barometerNoisePa: [0, 1000],
  magneticFieldMicroT: [1, 100],
  magneticDeclinationDeg: [-180, 180],
  magneticInclinationDeg: [-90, 90],
  magnetometerNoiseMicroT: [0, 10],
  rangefinderMinM: [0.01, 100],
  rangefinderMaxM: [0.02, 200],
  rangefinderNoiseM: [0, 10],
  lidarRays: [12, 180],
  lidarMinM: [0.01, 100],
  lidarMaxM: [0.02, 200],
  lidarNoiseM: [0, 10],
  opticalFlowNoiseRadS: [0, 10],
};
function object(value) {
  if (
    !value ||
    typeof value !== "object" ||
    Array.isArray(value) ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value))
  )
    throw new Error("Expected a plain object");
  return value;
}
function number(value, min, max, label) {
  if (!Number.isFinite(value) || value < min || value > max)
    throw new Error(`${label} must be finite and between ${min} and ${max}`);
  return value;
}
function vector(value, length) {
  if (!Array.isArray(value) || value.length !== length)
    throw new Error("Invalid sensor input vector");
  return Array.from(value, (v) => number(v, -1e9, 1e9, "Vector element"));
}
export function validateSensorConfig(value = {}) {
  object(value);
  for (const [key, entry] of Object.entries(value)) {
    if (!Object.hasOwn(bounds, key))
      throw new Error(`Unknown sensor configuration: ${key}`);
    number(entry, ...bounds[key], key);
  }
  const config = { ...DEFAULT_SENSOR_CONFIG, ...value };
  if (!Number.isInteger(config.lidarRays))
    throw new Error("lidarRays must be an integer");
  for (const name of ["rangefinder", "lidar"])
    if (config[`${name}MinM`] >= config[`${name}MaxM`])
      throw new Error(`${name} minimum must be below maximum`);
  return config;
}
const rotate = (r, v) =>
  [0, 1, 2].map(
    (i) => r[i * 3] * v[0] + r[i * 3 + 1] * v[1] + r[i * 3 + 2] * v[2],
  );
const inverseRotate = (r, v) =>
  [0, 1, 2].map((i) => r[i] * v[0] + r[3 + i] * v[1] + r[6 + i] * v[2]);
const radians = (degrees) => (degrees * Math.PI) / 180;
function nearestRay(origin, direction, obstacles, min, max) {
  let nearest = Infinity;
  if (Math.abs(direction[2]) > 1e-12) {
    const distance = -origin[2] / direction[2];
    if (distance >= 0) nearest = distance;
  }
  for (const box of obstacles) {
    const low = [
      box.north - box.width / 2,
      box.east - box.depth / 2,
      -box.height,
    ];
    const high = [box.north + box.width / 2, box.east + box.depth / 2, 0];
    let enter = -Infinity,
      exit = Infinity;
    for (let axis = 0; axis < 3; axis++) {
      if (Math.abs(direction[axis]) < 1e-12) {
        if (origin[axis] < low[axis] || origin[axis] > high[axis]) {
          exit = -Infinity;
          break;
        }
      } else {
        const a = (low[axis] - origin[axis]) / direction[axis],
          b = (high[axis] - origin[axis]) / direction[axis];
        enter = Math.max(enter, Math.min(a, b));
        exit = Math.min(exit, Math.max(a, b));
      }
    }
    if (exit >= Math.max(0, enter))
      nearest = Math.min(nearest, enter >= 0 ? enter : exit);
  }
  // A surface inside the blind zone blocks distant returns; do not see through it.
  return nearest >= min && nearest <= max ? nearest : null;
}
export function lidarPoints(reading) {
  object(reading);
  if (!reading.valid) return [];
  const pose = vector(reading.positionNed, 3),
    quaternion = vector(reading.quaternion, 4);
  const matrix = bodyToNedMatrix(quaternion);
  if (!Array.isArray(reading.rangesM)) throw new Error("Invalid lidar ranges");
  number(reading.angleStartRad, -Math.PI * 2, Math.PI * 2, "Lidar start angle");
  number(reading.angleStepRad, 0, Math.PI * 2, "Lidar angle step");
  return reading.rangesM.flatMap((range, index) => {
    if (range === null) return [];
    number(range, 0, 200, "Lidar range");
    const angle = reading.angleStartRad + index * reading.angleStepRad;
    const direction = rotate(matrix, [Math.cos(angle), Math.sin(angle), 0]);
    return [pose.map((v, i) => v + direction[i] * range)];
  });
}
export function createSensorSuite(configuration = {}, seed = 42) {
  const config = validateSensorConfig(configuration);
  number(seed, 0, 4294967295, "Sensor seed");
  if (!Number.isInteger(seed))
    throw new Error("Sensor seed must be an integer");
  let randomState,
    time,
    lastInputTime,
    readings,
    acquired,
    queues,
    deadlines,
    faults,
    obstacles;
  function random() {
    randomState = (Math.imul(1664525, randomState) + 1013904223) >>> 0;
    return (randomState + 0.5) / 4294967296;
  }
  const noise = (sigma) =>
    sigma === 0
      ? 0
      : sigma *
        Math.sqrt(-2 * Math.log(random())) *
        Math.cos(2 * Math.PI * random());
  function invalid(name, at, reason, input) {
    const common = { time: at, valid: false, reason };
    if (name === "gps")
      return {
        ...common,
        latitudeDeg: null,
        longitudeDeg: null,
        altitudeM: null,
        velocityNedMps: null,
      };
    if (name === "barometer")
      return {
        ...common,
        pressurePa: null,
        altitudeM: null,
        temperatureK: null,
      };
    if (name === "magnetometer") return { ...common, fieldBodyMicroT: null };
    if (name === "rangefinder") return { ...common, distanceM: null };
    if (name === "opticalFlow")
      return { ...common, flowRadS: null, groundDistanceM: null };
    return {
      ...common,
      rangesM: Array(config.lidarRays).fill(null),
      positionNed: input ? [...input.position] : null,
      quaternion: input ? [...input.quaternion] : null,
      angleStartRad: -Math.PI,
      angleStepRad: (2 * Math.PI) / config.lidarRays,
      minM: config.lidarMinM,
      maxM: config.lidarMaxM,
    };
  }
  function reset() {
    randomState = seed >>> 0;
    time = 0;
    lastInputTime = -1;
    obstacles = [];
    faults = Object.fromEntries(NAMES.map((name) => [name, false]));
    readings = Object.fromEntries(
      NAMES.map((name) => [name, invalid(name, 0, "startup")]),
    );
    acquired = { ...readings };
    queues = Object.fromEntries(NAMES.map((name) => [name, []]));
    deadlines = Object.fromEntries(NAMES.map((name) => [name, 0]));
    return snapshot();
  }
  function measure(name, input, matrix) {
    const common = { time: input.time, valid: true, reason: "" };
    if (name === "gps") {
      const latitude = radians(config.originLatitudeDeg),
        sin = Math.sin(latitude),
        eccentricity2 = 6.69437999014e-3;
      const denominator = Math.sqrt(1 - eccentricity2 * sin * sin);
      const primeRadius = 6378137 / denominator,
        meridianRadius = (6378137 * (1 - eccentricity2)) / denominator ** 3;
      const latitudeDeg =
        config.originLatitudeDeg +
        (((input.position[0] + noise(config.gpsNoiseM)) /
          (meridianRadius + config.originAltitudeM)) *
          180) /
          Math.PI;
      if (Math.abs(latitudeDeg) > 90)
        return invalid(name, input.time, "no-return", input);
      const longitude =
        config.originLongitudeDeg +
        (((input.position[1] + noise(config.gpsNoiseM)) /
          ((primeRadius + config.originAltitudeM) * Math.cos(latitude))) *
          180) /
          Math.PI;
      return {
        ...common,
        latitudeDeg,
        longitudeDeg: ((((longitude + 180) % 360) + 360) % 360) - 180,
        altitudeM:
          config.originAltitudeM -
          input.position[2] +
          noise(config.gpsVerticalNoiseM),
        velocityNedMps: input.velocity.map(
          (v) => v + noise(config.gpsVelocityNoiseMps),
        ),
      };
    }
    if (name === "barometer") {
      const altitude = config.originAltitudeM - input.position[2],
        temperatureK = 288.15 - 0.0065 * altitude;
      const pressurePa =
        101325 * (temperatureK / 288.15) ** 5.25588 +
        noise(config.barometerNoisePa);
      if (!Number.isFinite(pressurePa) || pressurePa <= 0 || temperatureK <= 0)
        return invalid(name, input.time, "no-return", input);
      return {
        ...common,
        pressurePa,
        temperatureK,
        altitudeM:
          ((1 - (pressurePa / 101325) ** (1 / 5.25588)) * 288.15) / 0.0065,
      };
    }
    if (name === "magnetometer") {
      const dip = radians(config.magneticInclinationDeg),
        dec = radians(config.magneticDeclinationDeg),
        strength = config.magneticFieldMicroT;
      const field = [
        Math.cos(dip) * Math.cos(dec),
        Math.cos(dip) * Math.sin(dec),
        Math.sin(dip),
      ].map((v) => v * strength);
      return {
        ...common,
        fieldBodyMicroT: inverseRotate(matrix, field).map(
          (v) => v + noise(config.magnetometerNoiseMicroT),
        ),
      };
    }
    if (name === "rangefinder") {
      const distance = nearestRay(
        input.position,
        rotate(matrix, [0, 0, 1]),
        obstacles,
        config.rangefinderMinM,
        config.rangefinderMaxM,
      );
      if (distance === null)
        return invalid(name, input.time, "no-return", input);
      return {
        ...common,
        distanceM: Math.max(
          config.rangefinderMinM,
          Math.min(
            config.rangefinderMaxM,
            distance + noise(config.rangefinderNoiseM),
          ),
        ),
      };
    }
    if (name === "lidar") {
      const rangesM = Array.from({ length: config.lidarRays }, (_, i) => {
        const angle = -Math.PI + (i * 2 * Math.PI) / config.lidarRays;
        const distance = nearestRay(
          input.position,
          rotate(matrix, [Math.cos(angle), Math.sin(angle), 0]),
          obstacles,
          config.lidarMinM,
          config.lidarMaxM,
        );
        return distance === null
          ? null
          : Math.max(
              config.lidarMinM,
              Math.min(config.lidarMaxM, distance + noise(config.lidarNoiseM)),
            );
      });
      return {
        ...common,
        valid: rangesM.some((v) => v !== null),
        reason: rangesM.some((v) => v !== null) ? "" : "no-return",
        rangesM,
        positionNed: [...input.position],
        quaternion: [...input.quaternion],
        angleStartRad: -Math.PI,
        angleStepRad: (2 * Math.PI) / config.lidarRays,
        minM: config.lidarMinM,
        maxM: config.lidarMaxM,
      };
    }
    // Geometric center flow, not an image/texture/calibration/weather model. Range is an explicit dependency.
    const range = acquired.rangefinder;
    if (!range.valid || range.distanceM === null || faults.rangefinder)
      return invalid(name, input.time, "no-return", input);
    const bodyVelocity = inverseRotate(matrix, input.velocity);
    return {
      ...common,
      flowRadS: [
        -bodyVelocity[1] / range.distanceM +
          input.rates[0] +
          noise(config.opticalFlowNoiseRadS),
        bodyVelocity[0] / range.distanceM +
          input.rates[1] +
          noise(config.opticalFlowNoiseRadS),
      ],
      groundDistanceM: range.distanceM,
    };
  }
  function update(value) {
    object(value);
    const input = {
      time: number(value.time, 0, 1e9, "Sensor simulation time"),
      position: vector(value.position, 3),
      velocity: vector(value.velocity, 3),
      quaternion: vector(value.quaternion, 4),
      rates: vector(value.rates, 3),
    };
    if (Math.abs(Math.hypot(...input.quaternion) - 1) > 1e-6)
      throw new Error("Sensor quaternion must be normalized");
    if (input.time < lastInputTime - 1e-10)
      throw new Error("Sensor time moved backwards; reset first");
    time = input.time;
    lastInputTime = time;
    const matrix = bodyToNedMatrix(input.quaternion);
    for (const name of NAMES) {
      if (time + 1e-10 >= deadlines[name]) {
        const period = 1 / config[`${name}Hz`];
        deadlines[name] +=
          Math.max(
            1,
            Math.floor((time - deadlines[name] + 1e-10) / period) + 1,
          ) * period;
        const reading = faults[name]
          ? invalid(name, time, "fault", input)
          : random() < config.dropout
            ? invalid(name, time, "dropout", input)
            : measure(name, input, matrix);
        acquired[name] = reading;
        queues[name].push(reading);
        if (queues[name].length > 32)
          throw new Error("Sensor latency queue overflow");
      }
      while (
        queues[name].length &&
        queues[name][0].time + config.latency <= time + 1e-10
      )
        readings[name] = queues[name].shift();
    }
    return snapshot();
  }
  function setObstacles(value) {
    if (!Array.isArray(value) || value.length > 50)
      throw new Error("Sensor obstacles must contain at most 50 boxes");
    const limits = {
      north: [-500, 500],
      east: [-500, 500],
      width: [0.01, 100],
      depth: [0.01, 100],
      height: [0.01, 500],
    };
    obstacles = Array.from(value, (box) => {
      object(box);
      if (Object.keys(box).some((key) => !Object.hasOwn(limits, key)))
        throw new Error("Unknown sensor obstacle field");
      return Object.fromEntries(
        Object.entries(limits).map(([key, range]) => [
          key,
          number(box[key], ...range, `Obstacle ${key}`),
        ]),
      );
    });
  }
  function setFault(name, failed) {
    if (!NAMES.includes(name) || typeof failed !== "boolean")
      throw new Error("Invalid sensor fault");
    faults[name] = failed;
    queues[name] = [];
    readings[name] = invalid(name, time, failed ? "fault" : "startup");
    acquired[name] = readings[name];
    deadlines[name] = time;
  }
  function snapshot() {
    return structuredClone({ time, ...readings, faults });
  }
  reset();
  return { update, snapshot, reset, setObstacles, setFault };
}
