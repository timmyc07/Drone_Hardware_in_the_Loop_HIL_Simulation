// Six CG-origin FRD rays; geometric observations, not a vehicle clearance model.
export const PROXIMITY_AXES = Object.freeze(
  [
    ["front", [1, 0, 0]],
    ["back", [-1, 0, 0]],
    ["right", [0, 1, 0]],
    ["left", [0, -1, 0]],
    ["up", [0, 0, -1]],
    ["down", [0, 0, 1]],
  ].map(([id, direction]) =>
    Object.freeze({ id, direction: Object.freeze(direction) }),
  ),
);
export const PROXIMITY_DEFAULT_CONFIG = Object.freeze({
  proximityHz: 10,
  proximityMinM: 0.1,
  proximityMaxM: 40,
  proximityNoiseM: 0.01,
  proximityWarningM: 3,
});
export const PROXIMITY_BOUNDS = Object.freeze(
  Object.fromEntries(
    Object.entries({
      proximityHz: [1, 20],
      proximityMinM: [0.01, 100],
      proximityMaxM: [0.02, 200],
      proximityNoiseM: [0, 10],
      proximityWarningM: [0.01, 200],
    }).map(([key, value]) => [key, Object.freeze(value)]),
  ),
);
function object(value) {
  if (
    !value ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value))
  )
    throw new Error("Proximity data requires a plain object");
}
function fields(value, keys) {
  object(value);
  if (
    Reflect.ownKeys(value).length !== keys.length ||
    Reflect.ownKeys(value).some((key) => !keys.includes(key))
  )
    throw new Error("Invalid proximity data fields");
}
function number(value, min, max, label) {
  if (!Number.isFinite(value) || value < min || value > max)
    throw new Error(`Invalid proximity ${label}`);
  return value;
}
function array(value, size) {
  if (
    !Array.isArray(value) ||
    value.length !== size ||
    Reflect.ownKeys(value).length !== size + 1 ||
    !Array.from({ length: size }, (_, i) => Object.hasOwn(value, i)).every(
      Boolean,
    )
  )
    throw new Error("Invalid dense proximity array");
  return value;
}
export function validateProximityConfig(value = {}) {
  object(value);
  if (
    Reflect.ownKeys(value).some(
      (key) => !Object.hasOwn(PROXIMITY_DEFAULT_CONFIG, key),
    )
  )
    throw new Error("Unknown proximity configuration field");
  const config = Object.fromEntries(
    Object.entries(PROXIMITY_DEFAULT_CONFIG).map(([key, fallback]) => [
      key,
      number(
        Object.hasOwn(value, key) ? value[key] : fallback,
        ...PROXIMITY_BOUNDS[key],
        key,
      ),
    ]),
  );
  if (
    config.proximityMinM >= config.proximityMaxM ||
    config.proximityWarningM < config.proximityMinM ||
    config.proximityWarningM > config.proximityMaxM
  )
    throw new Error(
      "Proximity requires minimum < maximum and warning within range",
    );
  return config;
}
function subset(value) {
  object(value);
  if (
    Reflect.ownKeys(value).some(
      (key) =>
        typeof key !== "string" ||
        (key.startsWith("proximity") &&
          !Object.hasOwn(PROXIMITY_DEFAULT_CONFIG, key)),
    )
  )
    throw new Error("Unknown proximity configuration field");
  return Object.fromEntries(
    Object.keys(PROXIMITY_DEFAULT_CONFIG)
      .filter((key) => Object.hasOwn(value, key))
      .map((key) => [key, value[key]]),
  );
}
export function proximityMetadata(config = {}) {
  const c = validateProximityConfig(subset(config));
  return {
    minM: c.proximityMinM,
    maxM: c.proximityMaxM,
    warningM: c.proximityWarningM,
  };
}
export function validateProximityReading(value, expectedConfig) {
  fields(value, [
    "time",
    "valid",
    "reason",
    "positionNed",
    "quaternion",
    "minM",
    "maxM",
    "warningM",
    "rangesM",
    "tooClose",
  ]);
  const metadata = proximityMetadata({
    proximityMinM: value.minM,
    proximityMaxM: value.maxM,
    proximityWarningM: value.warningM,
  });
  if (expectedConfig !== undefined) {
    const expected = proximityMetadata(expectedConfig);
    if (Object.keys(metadata).some((key) => metadata[key] !== expected[key]))
      throw new Error("Proximity metadata does not match configuration");
  }
  const time = number(value.time, 0, 1e9, "acquisition time");
  const rangesM = array(value.rangesM, 6).map((v) =>
    v === null ? null : number(v, metadata.minM, metadata.maxM, "range"),
  );
  const tooClose = array(value.tooClose, 6).map((v) => {
    if (typeof v !== "boolean") throw new Error("Invalid proximity tooClose");
    return v;
  });
  const valid = rangesM.some((v) => v !== null) || tooClose.some(Boolean);
  if (
    tooClose.some((v, i) => v && rangesM[i] !== null) ||
    typeof value.valid !== "boolean" ||
    value.valid !== valid ||
    !["", "startup", "fault", "dropout", "no-return"].includes(value.reason) ||
    valid !== (value.reason === "")
  )
    throw new Error("Proximity validity, reason and returns disagree");
  let positionNed = null,
    quaternion = null;
  if (value.positionNed === null && value.quaternion === null) {
    if (!["startup", "fault"].includes(value.reason))
      throw new Error("Acquired proximity requires pose");
  } else {
    if (value.reason === "startup")
      throw new Error("Startup proximity cannot have pose");
    positionNed = array(value.positionNed, 3).map((v) =>
      number(v, -1e9, 1e9, "position"),
    );
    quaternion = array(value.quaternion, 4).map((v) =>
      number(v, -1e9, 1e9, "quaternion"),
    );
    if (Math.abs(Math.hypot(...quaternion) - 1) > 1e-6)
      throw new Error("Proximity quaternion must be normalized");
  }
  return {
    time,
    valid,
    reason: value.reason,
    positionNed,
    quaternion,
    ...metadata,
    rangesM,
    tooClose,
  };
}
export const PROXIMITY_CSV_HEADER = Object.freeze([
  "source_vehicle_id",
  "observation_world_tick",
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
  "min_m",
  "max_m",
  "warning_m",
  "direction",
  "direction_forward",
  "direction_right",
  "direction_down",
  "range_m",
  "too_close_boolean",
]);
export function exportProximityData(format, value, identity) {
  if (!["json", "csv"].includes(format))
    throw new Error("Unsupported proximity export format");
  fields(identity, ["sourceVehicleId", "worldTick"]);
  if (
    typeof identity.sourceVehicleId !== "string" ||
    !/^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(identity.sourceVehicleId)
  )
    throw new Error("Invalid proximity source vehicle ID");
  const worldTick = number(
    identity.worldTick,
    0,
    360000,
    "observation world tick",
  );
  if (!Number.isInteger(worldTick))
    throw new Error("Proximity world tick must be an integer");
  const reading = validateProximityReading(value);
  if (reading.time > worldTick * 0.005 + 1e-7)
    throw new Error(
      "Proximity acquisition cannot be after observation world tick",
    );
  if (format === "json")
    return JSON.stringify({
      format: "flight-lab-proximity",
      version: 1,
      sourceVehicleId: identity.sourceVehicleId,
      worldTick,
      reading,
    });
  const common = [
    identity.sourceVehicleId,
    worldTick,
    reading.time,
    Number(reading.valid),
    reading.reason,
    ...(reading.positionNed ?? ["", "", ""]),
    ...(reading.quaternion ?? ["", "", "", ""]),
    reading.minM,
    reading.maxM,
    reading.warningM,
  ];
  return [
    PROXIMITY_CSV_HEADER.join(","),
    ...PROXIMITY_AXES.map((axis, i) =>
      [
        ...common,
        axis.id,
        ...axis.direction,
        reading.rangesM[i] ?? "",
        Number(reading.tooClose[i]),
      ].join(","),
    ),
  ].join("\n");
}
