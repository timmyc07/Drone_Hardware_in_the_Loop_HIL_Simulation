// Geometric range/angle/range-rate observations, not an RF or radar-cross-section model.
// Reference: CARLA 0.9.16 Radar.cpp; range rate is positive away from the sensor.
export const MAX_RADAR_DETECTIONS = 81 * 41;
export const RADAR_DEFAULT_CONFIG = Object.freeze({
  radarHz: 10,
  radarAzimuthRays: 31,
  radarElevationRays: 9,
  radarHorizontalFovDeg: 60,
  radarVerticalFovDeg: 20,
  radarMinM: 0.2,
  radarMaxM: 100,
  radarNoiseM: 0.05,
  radarVelocityNoiseMps: 0.05,
});
export const RADAR_BOUNDS = Object.freeze(
  Object.fromEntries(
    Object.entries({
      radarHz: [1, 20],
      radarAzimuthRays: [1, 81],
      radarElevationRays: [1, 41],
      radarHorizontalFovDeg: [1, 180],
      radarVerticalFovDeg: [1, 90],
      radarMinM: [0.01, 100],
      radarMaxM: [0.02, 200],
      radarNoiseM: [0, 10],
      radarVelocityNoiseMps: [0, 10],
    }).map(([key, range]) => [key, Object.freeze(range)]),
  ),
);
const REASONS = Object.freeze(["", "startup", "fault", "dropout", "no-return"]);
const METADATA_KEYS = [
  "azimuthCount",
  "elevationCount",
  "horizontalFovRad",
  "verticalFovRad",
  "minM",
  "maxM",
];
const READING_KEYS = [
  "time",
  "valid",
  "reason",
  "positionNed",
  "quaternion",
  "velocityNedMps",
  "detections",
  ...METADATA_KEYS,
];
const DETECTION_KEYS = [
  "beam",
  "rangeM",
  "azimuthRad",
  "elevationRad",
  "rangeRateMps",
];
const radians = (degrees) => (degrees * Math.PI) / 180;
function object(value) {
  if (
    !value ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value))
  )
    throw new Error("Radar data requires a plain object");
  return value;
}
function fields(value, keys) {
  object(value);
  const ownKeys = Reflect.ownKeys(value);
  if (
    ownKeys.length !== keys.length ||
    ownKeys.some((key) => !keys.includes(key))
  )
    throw new Error("Invalid radar data fields");
}
function number(value, min, max, label) {
  if (!Number.isFinite(value) || value < min || value > max)
    throw new Error(`Invalid radar ${label}`);
  return value;
}
function integer(value, min, max, label) {
  number(value, min, max, label);
  if (!Number.isInteger(value))
    throw new Error(`Radar ${label} must be an integer`);
  return value;
}
function array(value, min, max, label) {
  if (
    !Array.isArray(value) ||
    value.length < min ||
    value.length > max ||
    Reflect.ownKeys(value).length !== value.length + 1 ||
    !Array.from({ length: value.length }, (_, i) =>
      Object.hasOwn(value, i),
    ).every(Boolean)
  )
    throw new Error(`Invalid dense radar ${label}`);
  return value;
}
function vector(value, length, label) {
  return array(value, length, length, label).map((v) =>
    number(v, -1e9, 1e9, label),
  );
}
export function validateRadarConfig(value = {}) {
  object(value);
  if (
    Reflect.ownKeys(value).some(
      (key) => !Object.hasOwn(RADAR_DEFAULT_CONFIG, key),
    )
  )
    throw new Error("Unknown radar configuration field");
  const config = Object.fromEntries(
    Object.entries(RADAR_DEFAULT_CONFIG).map(([key, fallback]) => {
      const entry = Object.hasOwn(value, key) ? value[key] : fallback;
      return [key, number(entry, ...RADAR_BOUNDS[key], key)];
    }),
  );
  for (const key of ["radarAzimuthRays", "radarElevationRays"])
    integer(config[key], ...RADAR_BOUNDS[key], key);
  if (config.radarMinM >= config.radarMaxM)
    throw new Error("Radar minimum range must be below maximum range");
  return config;
}
function configSubset(value) {
  object(value);
  if (
    Reflect.ownKeys(value).some(
      (key) =>
        typeof key !== "string" ||
        (key.startsWith("radar") && !Object.hasOwn(RADAR_DEFAULT_CONFIG, key)),
    )
  )
    throw new Error("Unknown radar configuration field");
  return validateRadarConfig(
    Object.fromEntries(
      Object.keys(RADAR_DEFAULT_CONFIG)
        .filter((key) => Object.hasOwn(value, key))
        .map((key) => [key, value[key]]),
    ),
  );
}
export function radarMetadata(config = {}) {
  const checked = configSubset(config);
  return {
    azimuthCount: checked.radarAzimuthRays,
    elevationCount: checked.radarElevationRays,
    horizontalFovRad: radians(checked.radarHorizontalFovDeg),
    verticalFovRad: radians(checked.radarVerticalFovDeg),
    minM: checked.radarMinM,
    maxM: checked.radarMaxM,
  };
}
const axisAngle = (index, count, fov) =>
  count === 1 ? 0 : fov * (index / (count - 1) - 0.5);
function beamAngles(beam, metadata) {
  return {
    azimuthRad: axisAngle(
      beam % metadata.azimuthCount,
      metadata.azimuthCount,
      metadata.horizontalFovRad,
    ),
    elevationRad: axisAngle(
      Math.floor(beam / metadata.azimuthCount),
      metadata.elevationCount,
      metadata.verticalFovRad,
    ),
  };
}
export function radarDirections(config = {}) {
  const metadata = radarMetadata(config);
  return Array.from(
    { length: metadata.azimuthCount * metadata.elevationCount },
    (_, beam) => {
      const { azimuthRad, elevationRad } = beamAngles(beam, metadata);
      return {
        beam,
        azimuthRad,
        elevationRad,
        direction: [
          Math.cos(elevationRad) * Math.cos(azimuthRad),
          Math.cos(elevationRad) * Math.sin(azimuthRad),
          -Math.sin(elevationRad),
        ],
      };
    },
  );
}
function ownMetadata(reading) {
  return {
    azimuthCount: integer(reading.azimuthCount, 1, 81, "azimuth count"),
    elevationCount: integer(reading.elevationCount, 1, 41, "elevation count"),
    horizontalFovRad: number(
      reading.horizontalFovRad,
      radians(1),
      Math.PI,
      "horizontal FOV",
    ),
    verticalFovRad: number(
      reading.verticalFovRad,
      radians(1),
      Math.PI / 2,
      "vertical FOV",
    ),
    minM: number(reading.minM, 0.01, 100, "minimum range"),
    maxM: number(reading.maxM, 0.02, 200, "maximum range"),
  };
}
function fullReading(value, metadata) {
  fields(value, READING_KEYS);
  const supplied = ownMetadata(value);
  if (
    supplied.minM >= supplied.maxM ||
    METADATA_KEYS.some((key) => supplied[key] !== metadata[key])
  )
    throw new Error("Radar metadata does not match its configuration");
  const time = number(value.time, 0, 1e9, "acquisition time");
  if (typeof value.valid !== "boolean" || !REASONS.includes(value.reason))
    throw new Error("Invalid radar validity or reason");
  const count = metadata.azimuthCount * metadata.elevationCount;
  let previous = -1;
  const detections = array(value.detections, 0, count, "detections").map(
    (raw) => {
      fields(raw, DETECTION_KEYS);
      const beam = integer(raw.beam, 0, count - 1, "beam index");
      if (beam <= previous)
        throw new Error("Radar beam indices must be ordered and unique");
      previous = beam;
      const angles = beamAngles(beam, metadata);
      number(raw.azimuthRad, -Math.PI / 2, Math.PI / 2, "azimuth");
      number(raw.elevationRad, -Math.PI / 4, Math.PI / 4, "elevation");
      if (
        raw.azimuthRad !== angles.azimuthRad ||
        raw.elevationRad !== angles.elevationRad
      )
        throw new Error("Radar detection angles must match their beam");
      return {
        beam,
        rangeM: number(raw.rangeM, metadata.minM, metadata.maxM, "range"),
        ...angles,
        rangeRateMps: number(raw.rangeRateMps, -1e10, 1e10, "range rate"),
      };
    },
  );
  if (
    value.valid !== detections.length > 0 ||
    value.valid !== (value.reason === "")
  )
    throw new Error("Radar validity must agree with its detections and reason");
  const emptyPose = [
    value.positionNed,
    value.quaternion,
    value.velocityNedMps,
  ].every((v) => v === null);
  let positionNed = null,
    quaternion = null,
    velocityNedMps = null;
  if (emptyPose) {
    if (!["startup", "fault"].includes(value.reason))
      throw new Error(
        "Acquired radar readings require position, orientation and velocity",
      );
  } else {
    if (value.reason === "startup")
      throw new Error("Startup radar cannot have an acquisition pose");
    positionNed = vector(value.positionNed, 3, "position");
    quaternion = vector(value.quaternion, 4, "quaternion");
    if (Math.abs(Math.hypot(...quaternion) - 1) > 1e-6)
      throw new Error("Radar quaternion must be normalized");
    velocityNedMps = vector(value.velocityNedMps, 3, "source velocity");
  }
  return {
    time,
    valid: value.valid,
    reason: value.reason,
    positionNed,
    quaternion,
    velocityNedMps,
    detections,
    ...metadata,
  };
}
export function validateRadarReading(value, sensorConfig = {}) {
  const metadata = radarMetadata(sensorConfig);
  if (Array.isArray(value)) {
    const [time, reasonCode, positionNed, quaternion, velocityNedMps, raw] =
      array(value, 6, 6, "compact reading");
    integer(reasonCode, 0, REASONS.length - 1, "reason code");
    const reason = REASONS[reasonCode],
      count = metadata.azimuthCount * metadata.elevationCount;
    const detections = array(raw, 0, count, "compact detections").map(
      (tuple) => {
        const [beam, rangeM, rangeRateMps] = array(
          tuple,
          3,
          3,
          "compact detection",
        );
        integer(beam, 0, count - 1, "beam index");
        return { beam, rangeM, ...beamAngles(beam, metadata), rangeRateMps };
      },
    );
    value = {
      time,
      valid: reasonCode === 0,
      reason,
      positionNed,
      quaternion,
      velocityNedMps,
      detections,
      ...metadata,
    };
  }
  return fullReading(value, metadata);
}
export function compactRadarReading(value) {
  object(value);
  const reading = fullReading(value, ownMetadata(value));
  return [
    reading.time,
    REASONS.indexOf(reading.reason),
    reading.positionNed,
    reading.quaternion,
    reading.velocityNedMps,
    reading.detections.map((d) => [d.beam, d.rangeM, d.rangeRateMps]),
  ];
}
export const RADAR_CSV_HEADER = Object.freeze([
  "source_vehicle_id",
  "export_world_tick",
  "acquisition_time_s",
  "valid_boolean",
  "reason",
  "position_north_m",
  "position_east_m",
  "position_down_m",
  "quat_w",
  "quat_x",
  "quat_y",
  "quat_z",
  "velocity_north_m_s",
  "velocity_east_m_s",
  "velocity_down_m_s",
  "azimuth_count",
  "elevation_count",
  "horizontal_fov_rad",
  "vertical_fov_rad",
  "min_m",
  "max_m",
  "beam",
  "range_m",
  "azimuth_rad",
  "elevation_rad",
  "range_rate_m_s",
]);
export function exportRadarData(kind, value, context, sensorConfig = {}) {
  if (!["json", "csv"].includes(kind))
    throw new Error("Unsupported radar export format");
  fields(context, ["sourceVehicleId", "worldTick"]);
  if (
    typeof context.sourceVehicleId !== "string" ||
    !/^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(context.sourceVehicleId)
  )
    throw new Error("Invalid radar source vehicle ID");
  const exportWorldTick = integer(
    context.worldTick,
    0,
    360000,
    "export world tick",
  );
  const config = configSubset(sensorConfig),
    reading = validateRadarReading(value, config);
  if (reading.time > exportWorldTick * 0.005 + 1e-7)
    throw new Error("Radar acquisition cannot be after its export world tick");
  if (kind === "json")
    return JSON.stringify({
      format: "flight-lab-radar",
      version: 1,
      sourceVehicleId: context.sourceVehicleId,
      exportWorldTick,
      frame: "NED",
      detectionFrame: "FRD",
      units: "m",
      config,
      reading,
    });
  const common = [
    context.sourceVehicleId,
    exportWorldTick,
    reading.time,
    Number(reading.valid),
    reading.reason,
    ...(reading.positionNed ?? ["", "", ""]),
    ...(reading.quaternion ?? ["", "", "", ""]),
    ...(reading.velocityNedMps ?? ["", "", ""]),
    ...METADATA_KEYS.map((key) => reading[key]),
  ];
  const rows = reading.detections.length
    ? reading.detections.map((d) => [
        ...common,
        d.beam,
        d.rangeM,
        d.azimuthRad,
        d.elevationRad,
        d.rangeRateMps,
      ])
    : [[...common, "", "", "", "", ""]];
  return [RADAR_CSV_HEADER.join(","), ...rows.map((row) => row.join(","))].join(
    "\n",
  );
}
