import { createNistLaneCollider } from "./nist-geometry.js";
import {
  PROXIMITY_AXES,
  PROXIMITY_DEFAULT_CONFIG,
  PROXIMITY_BOUNDS,
  validateProximityConfig,
  proximityMetadata,
} from "./proximity.js";
import {
  validateObstacles,
  obstacleBounds,
  rayAabb,
} from "./obstacle-geometry.js";
import { bodyToNedMatrix } from "./sim.js";
import { rayPeerSphere } from "./peer-geometry.js";
import { createTerrain } from "./terrain.js";
import { localToGeographic, validateGeography } from "./geography.js";
import {
  RADAR_DEFAULT_CONFIG,
  RADAR_BOUNDS,
  validateRadarConfig,
  radarMetadata,
  radarDirections,
} from "./radar.js";

// Deterministic virtual observations only; independent of the flight controller.
// References: https://microsoft.github.io/AirSim/sensors/ https://microsoft.github.io/AirSim/lidar/
// Gaussian noise sigmas are per acquisition; no drift, constellation, WMM or calibration model.
// GPS is a local WGS84 tangent approximation; optical flow uses geometry, not rendered images.
export const MAX_LIDAR_POINTS = 32 * 180;
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
  originVerticalDatum: "unspecified",
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
  lidarChannels: 8,
  lidarVerticalMinDeg: -30,
  lidarVerticalMaxDeg: 10,
  lidarMinM: 0.2,
  lidarMaxM: 50,
  lidarNoiseM: 0.01,
  opticalFlowNoiseRadS: 0.01,
  ...RADAR_DEFAULT_CONFIG,
  ...PROXIMITY_DEFAULT_CONFIG,
});
export const SENSOR_NAMES = Object.freeze([
  "gps",
  "barometer",
  "magnetometer",
  "rangefinder",
  "lidar",
  "opticalFlow",
  "radar",
  "proximity",
]);
const bounds = {
  ...Object.fromEntries(SENSOR_NAMES.map((name) => [`${name}Hz`, [1, 20]])),
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
  lidarChannels: [1, 32],
  lidarVerticalMinDeg: [-90, 90],
  lidarVerticalMaxDeg: [-90, 90],
  lidarMinM: [0.01, 100],
  lidarMaxM: [0.02, 200],
  lidarNoiseM: [0, 10],
  opticalFlowNoiseRadS: [0, 10],
  ...RADAR_BOUNDS,
  ...PROXIMITY_BOUNDS,
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
    if (key === "originVerticalDatum") {
      if (!["unspecified", "ellipsoidal", "orthometric"].includes(entry))
        throw new Error("Invalid originVerticalDatum");
      continue;
    }
    if (!Object.hasOwn(bounds, key))
      throw new Error(`Unknown sensor configuration: ${key}`);
    number(entry, ...bounds[key], key);
  }
  const config = { ...DEFAULT_SENSOR_CONFIG, ...value };
  if (!Number.isInteger(config.lidarRays))
    throw new Error("lidarRays must be an integer");
  if (!Number.isInteger(config.lidarChannels))
    throw new Error("lidarChannels must be an integer");
  if (
    config.lidarChannels === 1
      ? config.lidarVerticalMinDeg !== config.lidarVerticalMaxDeg
      : config.lidarVerticalMinDeg >= config.lidarVerticalMaxDeg
  )
    throw new Error(
      "Single-channel lidar requires equal vertical angles; multiple channels require minimum below maximum",
    );
  for (const name of ["rangefinder", "lidar"])
    if (config[`${name}MinM`] >= config[`${name}MaxM`])
      throw new Error(`${name} minimum must be below maximum`);
  validateRadarConfig(
    Object.fromEntries(
      Object.keys(RADAR_DEFAULT_CONFIG).map((key) => [key, config[key]]),
    ),
  );
  validateProximityConfig(
    Object.fromEntries(
      Object.keys(PROXIMITY_DEFAULT_CONFIG).map((key) => [key, config[key]]),
    ),
  );
  return config;
}
export function assertGeographicSensors(scene, sensors) {
  object(scene);
  const config = validateSensorConfig(sensors);
  if (scene.geography == null) return;
  const { origin } = validateGeography(scene.geography);
  for (const [field, key] of [
    ["originLatitudeDeg", "latitudeDeg"],
    ["originLongitudeDeg", "longitudeDeg"],
    ["originAltitudeM", "altitudeM"],
    ["originVerticalDatum", "verticalDatum"],
  ])
    if (config[field] !== origin[key])
      throw new Error(`Geographic sensor ${field} must match the scene origin`);
}
const rotate = (r, v) =>
  [0, 1, 2].map(
    (i) => r[i * 3] * v[0] + r[i * 3 + 1] * v[1] + r[i * 3 + 2] * v[2],
  );
const inverseRotate = (r, v) =>
  [0, 1, 2].map((i) => r[i] * v[0] + r[3 + i] * v[1] + r[6 + i] * v[2]);
const radians = (degrees) => (degrees * Math.PI) / 180;
const STATIC_VELOCITY = Object.freeze([0, 0, 0]);
function nearestHit(origin, direction, obstacles, terrain, min, max, peers, nistCollider) {
  const length = Math.hypot(...direction);
  direction = direction.map((v) => v / length);
  let nearest =
    terrain.raycast(origin, direction, 0, max)?.distanceM ?? Infinity;
  nearest = Math.min(nearest, nistCollider?.ray(origin, direction, max) ?? Infinity);
  let velocityNed = STATIC_VELOCITY;
  for (const bounds of obstacles) {
    const interval = rayAabb(origin, direction, bounds);
    if (interval !== null && interval.exit >= Math.max(0, interval.enter)) {
      const distance = Math.max(0, interval.enter);
      if (distance < nearest) {
        nearest = distance;
        velocityNed = STATIC_VELOCITY;
      }
    }
  }
  for (const peer of peers) {
    const distance = rayPeerSphere(origin, direction, peer, 0, max);
    if (distance !== null && distance < nearest) {
      nearest = distance;
      velocityNed = peer.velocity;
    }
  }
  // A surface inside the blind zone blocks distant returns; do not see through it.
  return nearest >= min && nearest <= max
    ? { distanceM: nearest, velocityNed }
    : null;
}
export function lidarPoints(reading) {
  object(reading);
  number(reading.time, 0, 1e9, "Lidar acquisition time");
  if (
    typeof reading.valid !== "boolean" ||
    !["", "startup", "fault", "dropout", "no-return"].includes(
      reading.reason,
    ) ||
    (reading.valid ? reading.reason !== "" : reading.reason === "")
  )
    throw new Error("Invalid lidar validity/reason");
  const channels = number(reading.channelCount, 1, 32, "Lidar channels");
  const azimuths = number(reading.azimuthCount, 12, 180, "Lidar azimuth count");
  if (
    !Number.isInteger(channels) ||
    !Number.isInteger(azimuths) ||
    channels * azimuths > MAX_LIDAR_POINTS
  )
    throw new Error("Invalid lidar point budget");
  if (
    !Array.isArray(reading.rangesM) ||
    reading.rangesM.length !== channels * azimuths
  )
    throw new Error("Lidar ranges must match channel/azimuth counts");
  number(reading.minM, 0.01, 100, "Lidar minimum range");
  number(reading.maxM, 0.02, 200, "Lidar maximum range");
  if (reading.minM >= reading.maxM)
    throw new Error("Lidar minimum must be below maximum");
  const start = number(
    reading.angleStartRad,
    -Math.PI,
    -Math.PI,
    "Lidar start angle",
  );
  const step = number(reading.angleStepRad, 0, 2 * Math.PI, "Lidar angle step");
  if (Math.abs(step - (2 * Math.PI) / azimuths) > 1e-12)
    throw new Error("Lidar azimuth spacing is inconsistent");
  const elevation = number(
    reading.elevationStartRad,
    -Math.PI / 2,
    Math.PI / 2,
    "Lidar elevation start",
  );
  const elevationStep = number(
    reading.elevationStepRad,
    0,
    Math.PI,
    "Lidar elevation step",
  );
  if (
    (channels === 1 ? elevationStep !== 0 : elevationStep <= 0) ||
    elevation + (channels - 1) * elevationStep > Math.PI / 2 + 1e-12
  )
    throw new Error("Lidar elevation spacing is inconsistent");
  const ranges = Array.from(reading.rangesM, (value) =>
    value === null
      ? null
      : number(value, reading.minM, reading.maxM, "Lidar range"),
  );
  const hasReturn = ranges.some((value) => value !== null);
  if (reading.valid !== hasReturn)
    throw new Error("Lidar validity must match its returns");
  if (reading.positionNed === null && reading.quaternion === null) {
    if (reading.valid || !["startup", "fault"].includes(reading.reason))
      throw new Error("Acquired lidar requires its pose");
    return [];
  }
  if (reading.reason === "startup")
    throw new Error("Startup lidar cannot have an acquisition pose");
  const pose = vector(reading.positionNed, 3),
    quaternion = vector(reading.quaternion, 4);
  if (Math.abs(Math.hypot(...quaternion) - 1) > 1e-6)
    throw new Error("Lidar quaternion must be normalized");
  const matrix = bodyToNedMatrix(quaternion),
    points = [];
  for (let index = 0; index < ranges.length; index++) {
    if (ranges[index] === null) continue;
    const angle = start + (index % azimuths) * step;
    const e = elevation + Math.floor(index / azimuths) * elevationStep;
    const direction = rotate(matrix, [
      Math.cos(e) * Math.cos(angle),
      Math.cos(e) * Math.sin(angle),
      -Math.sin(e),
    ]);
    points.push(pose.map((v, i) => v + direction[i] * ranges[index]));
  }
  return points;
}
export function createSensorSuite(configuration = {}, seed = 42) {
  const config = validateSensorConfig(configuration);
  const proximityInfo = proximityMetadata(config);
  const radarInfo = radarMetadata(config),
    radarBeams = radarDirections(config);
  const lidarMetadata = {
    channelCount: config.lidarChannels,
    azimuthCount: config.lidarRays,
    angleStartRad: -Math.PI,
    angleStepRad: (2 * Math.PI) / config.lidarRays,
    elevationStartRad: radians(config.lidarVerticalMinDeg),
    elevationStepRad:
      config.lidarChannels === 1
        ? 0
        : radians(config.lidarVerticalMaxDeg - config.lidarVerticalMinDeg) /
          (config.lidarChannels - 1),
    minM: config.lidarMinM,
    maxM: config.lidarMaxM,
  };
  // Channel-major acquisition: all rays share the same instantaneous physical pose.
  const lidarDirections = Array.from(
    { length: config.lidarChannels * config.lidarRays },
    (_, index) => {
      const angle =
        lidarMetadata.angleStartRad +
        (index % config.lidarRays) * lidarMetadata.angleStepRad;
      const elevation =
        lidarMetadata.elevationStartRad +
        Math.floor(index / config.lidarRays) * lidarMetadata.elevationStepRad;
      return [
        Math.cos(elevation) * Math.cos(angle),
        Math.cos(elevation) * Math.sin(angle),
        -Math.sin(elevation),
      ];
    },
  );
  number(seed, 0, 4294967295, "Sensor seed");
  if (!Number.isInteger(seed))
    throw new Error("Sensor seed must be an integer");
  let randomState,
    radarRandomState,
    proximityRandomState,
    time,
    lastInputTime,
    readings,
    acquired,
    queues,
    deadlines,
    faults,
    nistCollider,
    obstacles,
    terrain,
    peers;
  function random() {
    randomState = (Math.imul(1664525, randomState) + 1013904223) >>> 0;
    return (randomState + 0.5) / 4294967296;
  }
  function radarRandom() {
    radarRandomState =
      (Math.imul(1664525, radarRandomState) + 1013904223) >>> 0;
    return (radarRandomState + 0.5) / 4294967296;
  }
  function proximityRandom() {
    proximityRandomState =
      (Math.imul(1664525, proximityRandomState) + 1013904223) >>> 0;
    return (proximityRandomState + 0.5) / 4294967296;
  }
  const noise = (sigma, rng = random) =>
    sigma === 0
      ? 0
      : sigma * Math.sqrt(-2 * Math.log(rng())) * Math.cos(2 * Math.PI * rng());
  function invalid(name, at, reason, input) {
    const common = { time: at, valid: false, reason };
    if (name === "proximity")
      return {
        ...common,
        positionNed: input ? [...input.position] : null,
        quaternion: input ? [...input.quaternion] : null,
        ...proximityInfo,
        rangesM: Array(6).fill(null),
        tooClose: Array(6).fill(false),
      };
    if (name === "radar")
      return {
        ...common,
        positionNed: input ? [...input.position] : null,
        quaternion: input ? [...input.quaternion] : null,
        velocityNedMps: input ? [...input.velocity] : null,
        detections: [],
        ...radarInfo,
      };
    if (name === "gps")
      return {
        ...common,
        verticalDatum: config.originVerticalDatum,
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
      rangesM: Array(config.lidarRays * config.lidarChannels).fill(null),
      positionNed: input ? [...input.position] : null,
      quaternion: input ? [...input.quaternion] : null,
      ...lidarMetadata,
    };
  }
  function reset() {
    randomState = seed >>> 0;
    // Deterministic stream partition from the one explicit suite seed; not a calibrated independence guarantee.
    radarRandomState = (seed ^ 0x7f4a7c15) >>> 0;
    proximityRandomState = (seed ^ 0x9e3779b9) >>> 0;
    time = 0;
    lastInputTime = -1;
    obstacles = [];
    nistCollider = null;
    peers = [];
    terrain = createTerrain();
    faults = Object.fromEntries(SENSOR_NAMES.map((name) => [name, false]));
    readings = Object.fromEntries(
      SENSOR_NAMES.map((name) => [name, invalid(name, 0, "startup")]),
    );
    acquired = { ...readings };
    queues = Object.fromEntries(SENSOR_NAMES.map((name) => [name, []]));
    deadlines = Object.fromEntries(SENSOR_NAMES.map((name) => [name, 0]));
    return snapshot();
  }
  function measure(name, input, matrix) {
    const common = { time: input.time, valid: true, reason: "" };
    if (name === "proximity") {
      const tooClose = Array(6).fill(false);
      const rangesM = PROXIMITY_AXES.map((axis, i) => {
        const hit = nearestHit(
          input.position,
          rotate(matrix, axis.direction),
          obstacles,
          terrain,
          0,
          config.proximityMaxM,
          peers,
          nistCollider,
        );
        if (hit === null) return null;
        if (hit.distanceM < config.proximityMinM) {
          tooClose[i] = true;
          return null;
        }
        return Math.max(
          config.proximityMinM,
          Math.min(
            config.proximityMaxM,
            hit.distanceM + noise(config.proximityNoiseM, proximityRandom),
          ),
        );
      });
      const valid = rangesM.some((v) => v !== null) || tooClose.some(Boolean);
      return {
        ...common,
        valid,
        reason: valid ? "" : "no-return",
        positionNed: [...input.position],
        quaternion: [...input.quaternion],
        ...proximityInfo,
        rangesM,
        tooClose,
      };
    }
    if (name === "gps") {
      let coordinates;
      try {
        coordinates = localToGeographic(
          input.position[0] + noise(config.gpsNoiseM),
          input.position[1] + noise(config.gpsNoiseM),
          {
            latitudeDeg: config.originLatitudeDeg,
            longitudeDeg: config.originLongitudeDeg,
            altitudeM: config.originAltitudeM,
          },
        );
      } catch {
        return invalid(name, input.time, "no-return", input);
      }
      return {
        ...common,
        ...coordinates,
        verticalDatum: config.originVerticalDatum,
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
      const hit = nearestHit(
        input.position,
        rotate(matrix, [0, 0, 1]),
        obstacles,
        terrain,
        config.rangefinderMinM,
        config.rangefinderMaxM,
        peers,
        nistCollider,
      );
      if (hit === null) return invalid(name, input.time, "no-return", input);
      return {
        ...common,
        distanceM: Math.max(
          config.rangefinderMinM,
          Math.min(
            config.rangefinderMaxM,
            hit.distanceM + noise(config.rangefinderNoiseM),
          ),
        ),
      };
    }
    if (name === "lidar") {
      const rangesM = lidarDirections.map((direction) => {
        const hit = nearestHit(
          input.position,
          rotate(matrix, direction),
          obstacles,
          terrain,
          config.lidarMinM,
          config.lidarMaxM,
          peers,
          nistCollider,
        );
        return hit === null
          ? null
          : Math.max(
              config.lidarMinM,
              Math.min(
                config.lidarMaxM,
                hit.distanceM + noise(config.lidarNoiseM),
              ),
            );
      });
      return {
        ...common,
        valid: rangesM.some((v) => v !== null),
        reason: rangesM.some((v) => v !== null) ? "" : "no-return",
        rangesM,
        positionNed: [...input.position],
        quaternion: [...input.quaternion],
        ...lidarMetadata,
      };
    }
    if (name === "radar") {
      const detections = [];
      for (const beam of radarBeams) {
        const direction = rotate(matrix, beam.direction);
        const hit = nearestHit(
          input.position,
          direction,
          obstacles,
          terrain,
          config.radarMinM,
          config.radarMaxM,
          peers,
          nistCollider,
        );
        if (hit === null) continue;
        const length = Math.hypot(...direction);
        const rate = hit.velocityNed.reduce(
          (sum, v, i) =>
            sum + ((v - input.velocity[i]) * direction[i]) / length,
          0,
        );
        detections.push({
          beam: beam.beam,
          rangeM: Math.max(
            config.radarMinM,
            Math.min(
              config.radarMaxM,
              hit.distanceM + noise(config.radarNoiseM, radarRandom),
            ),
          ),
          azimuthRad: beam.azimuthRad,
          elevationRad: beam.elevationRad,
          rangeRateMps: rate + noise(config.radarVelocityNoiseMps, radarRandom),
        });
      }
      return {
        ...common,
        valid: detections.length > 0,
        reason: detections.length ? "" : "no-return",
        positionNed: [...input.position],
        quaternion: [...input.quaternion],
        velocityNedMps: [...input.velocity],
        detections,
        ...radarInfo,
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
    for (const name of SENSOR_NAMES) {
      if (time + 1e-10 >= deadlines[name]) {
        const period = 1 / config[`${name}Hz`];
        deadlines[name] +=
          Math.max(
            1,
            Math.floor((time - deadlines[name] + 1e-10) / period) + 1,
          ) * period;
        const reading = faults[name]
          ? invalid(name, time, "fault", input)
          : (name === "radar"
                ? radarRandom()
                : name === "proximity"
                  ? proximityRandom()
                  : random()) < config.dropout
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
  function setNistLane(enabled) {
    if (typeof enabled !== "boolean") throw new Error("Invalid NIST lane enabled");
    nistCollider = enabled ? createNistLaneCollider() : null;
  }
  function setObstacles(value) {
    obstacles = validateObstacles(value).map((box) => obstacleBounds(box));
  }
  function setPeers(value, sourceVehicleId) {
    const validId = (id) =>
      typeof id === "string" && id.length > 0 && id.length <= 64;
    if (!Array.isArray(value) || value.length > 8 || !validId(sourceVehicleId))
      throw new Error("Invalid sensor peers or source vehicle ID");
    const seen = new Set();
    const next = Array.from(value, (peer) => {
      object(peer);
      if (
        Object.keys(peer).length !== 4 ||
        Object.keys(peer).some(
          (key) => !["id", "position", "velocity", "radius"].includes(key),
        ) ||
        !validId(peer.id) ||
        seen.has(peer.id)
      )
        throw new Error("Invalid or duplicate sensor peer");
      seen.add(peer.id);
      return {
        id: peer.id,
        position: vector(peer.position, 3),
        velocity: vector(peer.velocity, 3),
        radius: number(peer.radius, 0.001, 1000, "Peer radius"),
      };
    });
    peers = next
      .filter((peer) => peer.id !== sourceVehicleId)
      .sort((a, b) => (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
  }
  function setTerrain(value) {
    const next = createTerrain(value);
    terrain = next;
  }
  function setFault(name, failed) {
    if (!SENSOR_NAMES.includes(name) || typeof failed !== "boolean")
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
  function checkpoint() {
    // Acquisitions and geometry are owned immutable values; only maps and queues mutate.
    const saved = {
      randomState,
      radarRandomState,
      proximityRandomState,
      time,
      lastInputTime,
      readings: { ...readings },
      acquired: { ...acquired },
      queues: Object.fromEntries(
        SENSOR_NAMES.map((name) => [name, [...queues[name]]]),
      ),
      deadlines: { ...deadlines },
      faults: { ...faults },
      nistCollider,
      obstacles,
      terrain,
      peers,
    };
    let used = false;
    return () => {
      if (used) throw new Error("Sensor checkpoint already restored");
      used = true;
      ({
        randomState,
        radarRandomState,
        proximityRandomState,
        time,
        lastInputTime,
        readings,
        acquired,
        queues,
        deadlines,
        faults,
        nistCollider,
        obstacles,
        terrain,
        peers,
      } = saved);
    };
  }
  reset();
  return {
    update,
    snapshot,
    reset,
    setNistLane,
    setObstacles,
    setTerrain,
    setPeers,
    setFault,
    checkpoint,
  };
}
