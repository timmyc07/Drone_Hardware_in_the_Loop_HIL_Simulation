import { DEFAULT_COLLISION_PREVENTION, validateCollisionPreventionSettings, validateCollisionPreventionState } from "./collision-prevention.js";
import { NIST_EVENT_TYPES, createNistSession, validateNistEventValue } from './nist.js';
import { createSimulation, validateConfig, bodyToNedAngles } from "./sim.js";
import {
  DEFAULT_SENSOR_CONFIG,
  validateSensorConfig,
  assertGeographicSensors,
  SENSOR_NAMES,
} from "./sensors.js";
import {
  TRAINING_COURSES,
  TUTORIAL_STEPS,
  isCourseSupported,
  resolveTrainingGates,
} from "./training.js";
import { DEFAULT_SCENE, validateScene } from "./scene-data.js";
import {
  validateWorldConfig,
  validateVehicleSetup,
  inspectionSummary,
} from "./world.js";
import { validateCameraConfig, validateCameraMetadata } from "./camera.js";
import { assessInspection, INSPECTION_REPORT_VERSION } from "./inspection.js";
import { obstacleContainsVehicle } from "./obstacle-geometry.js";
import { getVehicleEnvelope } from "./peer-geometry.js";
import { validateRadarReading, compactRadarReading } from "./radar.js";
import { PROXIMITY_AXES, validateProximityReading } from "./proximity.js";
import { AIRFRAMES, rotorCount } from "./rotor-geometry.js";
import { createTerrain } from "./terrain.js";
import {
  DEFAULT_GIMBAL_SETTINGS,
  validateGimbalSettings,
  validateGimbalState,
} from "./gimbal.js";

export const RECORDING_VERSION = 20;
export const MODEL_VERSION = "0.14.0";
const BATTERY_FIELDS = [
  "soc", "voltage", "current", "power", "usedWh", "remainingSeconds", "low", "critical",
  "openCircuitVoltage", "motorSpeedScale", "powerLimited",
];
function unpackBattery(value) {
  if (Array.isArray(value)) {
    if (value.length !== BATTERY_FIELDS.length || Reflect.ownKeys(value).length !== BATTERY_FIELDS.length + 1)
      throw new Error("Invalid compact battery");
    value = Object.fromEntries(BATTERY_FIELDS.map((key, i) => [key, value[i]]));
  }
  exactKeys(value, BATTERY_FIELDS, "battery");
  return value;
}
const MODES = ["attitude", "position", "mission", "rtl", "land", "takeoff"];
const TAKEOFF_PHASES = [
  "idle",
  "runup",
  "roll",
  "rotate",
  "climb",
  "complete",
  "aborted",
];
const ABORT_REASONS = [
  "",
  "pilot",
  "pusher-failure",
  "runway-end",
  "runway-excursion",
  "insufficient-airspeed",
  "safety",
];
const COURSES = Object.keys(TRAINING_COURSES);
const GIMBAL_AXES = ["yaw", "pitch", "roll"];
export const DEFAULT_SETUP = Object.freeze({
  sensors: DEFAULT_SENSOR_CONFIG,
  gimbal: DEFAULT_GIMBAL_SETTINGS,
  collisionPrevention: DEFAULT_COLLISION_PREVENTION,
  mode: "attitude",
  mission: Object.freeze([]),
  scene: DEFAULT_SCENE,
  training: "free",
});
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
const count = (value, max = 1e9) => {
  number(value, 0, max);
  if (!Number.isInteger(value)) throw new Error("Invalid count");
  return value;
};
const boolean = (value) => {
  if (typeof value !== "boolean") throw new Error("Expected boolean");
  return value;
};
const choice = (value, values) => {
  if (!values.includes(value)) throw new Error("Invalid enum value");
  return value;
};
const vector = (value, length, min = -1e9, max = 1e9) => {
  if (!Array.isArray(value) || value.length !== length)
    throw new Error("Invalid vector");
  return Array.from(value, (item) => number(item, min, max));
};
export function validateSetup(value, config = {}) {
  const input = value === undefined ? DEFAULT_SETUP : object(value);
  if (Object.keys(input).some((key) => !Object.hasOwn(DEFAULT_SETUP, key)))
    throw new Error("Unknown setup field");
  const { scene: sceneInput, ...setup } = { ...DEFAULT_SETUP, ...input };
  const scene = validateScene(sceneInput);
  return {
    ...validateVehicleSetup(setup, validateConfig(config), scene),
    scene,
  };
}

function targets(value, partial = false, automatic = false) {
  object(value);
  const limits = {
    altitude: [-1000, 1500],
    roll: [-Math.PI / 6, Math.PI / 6],
    pitch: [-Math.PI / 6, Math.PI / 6],
    yawRate: [-2, 2],
    airspeed: [0, 40],
  };
  if (Object.keys(value).some((key) => !Object.hasOwn(limits, key)))
    throw new Error("Unknown target");
  const result = {};
  for (const [key, [min, max]] of Object.entries(limits))
    if (!partial || Object.hasOwn(value, key))
      result[key] = number(
        value[key],
        automatic && key === "altitude" ? -1000 : min,
        automatic && key === "altitude" ? 1e9 : max,
      );
  return result;
}
function training(value, airframe, scene, activeCourse, home, groundHeight) {
  object(value);
  const course = choice(value.course, COURSES),
    gates = resolveTrainingGates(
      scene,
      airframe,
      activeCourse,
      home,
      groundHeight,
    ),
    total = course === "tutorial" ? TUTORIAL_STEPS.length : gates.length;
  if (course !== activeCourse)
    throw new Error("Training course does not match active scene");
  if (count(value.total, 50) !== total)
    throw new Error("Invalid training gate total");
  const passed = count(value.passed, total);
  if (gates.length && boolean(value.complete) !== (passed === total))
    throw new Error("Training completion disagrees with gate progress");
  if (course === "tutorial") {
    const reason = choice(value.reason, [
      "",
      "crashed",
      "procedure-order",
      "failsafe",
      "unsafe",
    ]);
    const minimumElapsed = TUTORIAL_STEPS.slice(0, passed).reduce(
      (sum, step) => sum + step.holdSeconds,
      0,
    );
    if (
      boolean(value.complete) !== (passed === total) ||
      (value.complete && reason) ||
      value.misses !== 0 ||
      value.score !== (reason ? 0 : Math.round((1000 * passed) / total)) ||
      number(value.elapsed, 0) + 1e-8 < minimumElapsed ||
      number(value.holdSeconds, 0) >
        (TUTORIAL_STEPS[passed]?.holdSeconds ?? 0) ||
      ((reason || value.complete) && value.holdSeconds !== 0)
    )
      throw new Error("Invalid tutorial progress");
  } else if (!gates.length) {
    const hold = number(value.holdSeconds, 0),
      complete = boolean(value.complete);
    if (
      course === "free"
        ? complete || hold !== 0 || value.score !== 0
        : complete !== hold >= (course === "hover" ? 10 : 2)
    )
      throw new Error("Training completion disagrees with course progress");
  }
  const expectedGate = gates[passed] ?? null;
  let nextGate = null;
  if (expectedGate) {
    const gate = object(value.nextGate);
    nextGate = {
      north: number(gate.north, -1000, 1000),
      east: number(gate.east, -1000, 1000),
      altitude: number(gate.altitude, -1000, 1500),
      yaw: number(gate.yaw, -Math.PI, Math.PI),
      radius: number(gate.radius, 0.01, 100),
    };
    if (JSON.stringify(nextGate) !== JSON.stringify(expectedGate))
      throw new Error("Invalid next training gate");
  } else if (value.nextGate !== null)
    throw new Error("Invalid next training gate");
  return {
    course,
    elapsed: number(value.elapsed, 0),
    passed,
    total,
    misses: count(value.misses),
    complete: boolean(value.complete),
    score: number(value.score, 0, 1000),
    nextGate,
    holdSeconds: number(value.holdSeconds, 0),
    reason: choice(
      value.reason,
      course === "tutorial"
        ? ["", "crashed", "procedure-order", "failsafe", "unsafe"]
        : ["", "crashed", "takeoff-aborted"],
    ),
  };
}
function sample(
  value,
  route,
  routeText,
  routeCache,
  airframe,
  scene,
  gimbalSettings,
  terrain,
  expectedHome,
  activeCourse,
  collisionPreventionSettings,
) {
  object(value);
  if (
    Object.hasOwn(value, "airframe") &&
    choice(value.airframe, AIRFRAMES) !== airframe
  )
    throw new Error("Snapshot airframe does not match configuration");
  const quaternion = vector(value.quaternion, 4, -1, 1);
  if (Math.abs(Math.hypot(...quaternion) - 1) > 0.01)
    throw new Error("Invalid quaternion");
  const attitude = bodyToNedAngles(quaternion).map((v) => (v === 0 ? 0 : v));
  if (
    Object.hasOwn(value, "attitude") &&
    vector(value.attitude, 3).some((v, i) => Math.abs(v - attitude[i]) > 1e-8)
  )
    throw new Error("Attitude does not match body quaternion");
  const imu = object(value.imu),
    metrics = object(value.metrics),
    nav = object(value.navigation),
    battery = unpackBattery(value.battery),
    faults = object(value.faults);
  if (Object.hasOwn(nav, "waypoints")) {
    if (!Array.isArray(nav.waypoints))
      throw new Error("Invalid navigation waypoints");
    let text = routeCache.get(nav.waypoints);
    if (text === undefined) {
      text = JSON.stringify(nav.waypoints);
      routeCache.set(nav.waypoints, text);
    }
    if (text !== routeText)
      throw new Error("Navigation waypoints do not match mission history");
  } else if (!Object.hasOwn(nav, "waypointCount"))
    throw new Error("Missing navigation waypoint count");
  if (
    Object.hasOwn(nav, "waypointCount") &&
    count(nav.waypointCount, 50) !== route.length
  )
    throw new Error("Invalid waypoint count");
  const resultMetrics = {};
  for (const key of [
    "rmse",
    "peakError",
    "altitudeRmse",
    "altitudePeakError",
    "sampleAge",
  ])
    resultMetrics[key] = number(metrics[key], 0);
  for (const key of ["dropped", "steps"])
    resultMetrics[key] = count(metrics[key]);
  const mode = choice(nav.mode, MODES);
  if (mode === "takeoff" && airframe !== "fixedwing")
    throw new Error("Runway takeoff mode requires a fixed-wing airframe");
  if (mode === "mission" && !route.length)
    throw new Error("Mission mode requires waypoints");
  const home = vector(nav.home, 2, -500, 500);
  if (home.some((v, i) => v !== expectedHome[i]))
    throw new Error("Invalid home position");
  let rawGimbal = value.gimbal;
  if (Array.isArray(rawGimbal)) {
    if (rawGimbal.length !== 6) throw new Error("Invalid compact gimbal pose");
    const [yaw, pitch, roll, limited, rateLimited, trackingErrorRad] =
      Array.from(rawGimbal);
    count(limited, 7);
    count(rateLimited, 7);
    const flags = (mask) =>
      Object.fromEntries(
        GIMBAL_AXES.map((axis, i) => [axis, Boolean(mask & (1 << i))]),
      );
    rawGimbal = {
      ...gimbalSettings,
      actual: { yaw, pitch, roll },
      limited: flags(limited),
      rateLimited: flags(rateLimited),
      trackingErrorRad,
    };
  }
  const gimbal = validateGimbalState(rawGimbal, quaternion);
  if (
    JSON.stringify({
      mode: gimbal.mode,
      target: gimbal.target,
      maxRate: gimbal.maxRate,
    }) !== JSON.stringify(gimbalSettings)
  )
    throw new Error("Gimbal settings do not match command history");
  const position = vector(value.position, 3),
    surface = terrain.sample(position[0], position[1]);
  const ground = {
    heightM: surface.heightM,
    clearanceM: number(-position[2] - surface.heightM, -1e-6),
    normalNed: surface.normalNed.map((v) => (v === 0 ? 0 : v)),
  };
  if (Object.hasOwn(value, "ground")) {
    const supplied = object(value.ground);
    if (
      Object.keys(supplied).sort().join(",") !==
        "clearanceM,heightM,normalNed" ||
      Math.abs(number(supplied.heightM) - ground.heightM) > 1e-8 ||
      Math.abs(number(supplied.clearanceM) - ground.clearanceM) > 1e-8 ||
      vector(supplied.normalNed, 3).some(
        (v, i) => Math.abs(v - ground.normalNed[i]) > 1e-8,
      )
    )
      throw new Error("Ground pose does not match terrain history");
  }
  let reference = nav.altitudeReference,
    positionHeight = nav.positionAltitude;
  if (Object.hasOwn(nav, "altitude")) {
    if (
      !Array.isArray(nav.altitude) ||
      nav.altitude.length !== 2 ||
      Object.hasOwn(nav, "altitudeReference") ||
      Object.hasOwn(nav, "positionAltitude")
    )
      throw new Error("Invalid compact navigation altitude");
    reference = ["datum", "agl"][count(nav.altitude[0], 1)];
    positionHeight = nav.altitude[1];
  }
  const altitudeReference = choice(reference, ["datum", "agl"]),
    positionAltitude = number(positionHeight, -1000);
  if (
    altitudeReference === "agl" &&
    (mode !== "position" ||
      positionAltitude < 0 ||
      positionAltitude > 500 ||
      airframe === "fixedwing" ||
      (airframe === "vtol" &&
        (value.flight?.transition !== "hover" ||
          value.flight?.transitionProgress !== 0)))
  )
    throw new Error("Invalid terrain-following state");
  return {
    airframe,
    time: number(value.time, 0, 1800.001),
    position,
    ground,
    velocity: vector(value.velocity, 3),
    quaternion,
    gimbal,
    collisionPrevention: unpackCollisionPrevention(value.collisionPrevention, collisionPreventionSettings),
    attitude,
    rates: vector(value.rates, 3),
    motors: vector(value.motors, rotorCount(airframe), 0, 1),
    pwm: vector(value.pwm, rotorCount(airframe), 1000, 2000),
    target: targets(value.target, false, true),
    imu: { accel: vector(imu.accel, 3), gyro: vector(imu.gyro, 3) },
    metrics: resultMetrics,
    status: choice(value.status, [
      "ready",
      "flying",
      "landed",
      "failsafe",
      "crashed",
    ]),
    wind: vector(value.wind, 3),
    armed: boolean(value.armed),
    navigation: {
      mode,
      waypoints: route,
      index: count(nav.index, Math.max(0, route.length - 1)),
      completed: boolean(nav.completed),
      home,
      targetPosition: vector(nav.targetPosition, 2),
      altitudeReference,
      positionAltitude,
      fenceBreached: boolean(nav.fenceBreached),
      reason: choice(nav.reason, [
        "",
        "rtl",
        "stale-sensor",
        "critical-battery",
        "low-battery",
        "geofence",
        "link-loss",
        "obstacle-collision",
        "vehicle-collision",
        "ground-impact",
        "terrain-impact",
        "takeoff-aborted",
      ]),
    },
    battery: {
      soc: number(battery.soc, 0, 1),
      voltage: number(battery.voltage, 0, 100),
      openCircuitVoltage: number(battery.openCircuitVoltage, 0, 100),
      motorSpeedScale: number(battery.motorSpeedScale, 0, 1),
      powerLimited: boolean(battery.powerLimited),
      current: number(battery.current, 0, 1e6),
      power: number(battery.power, 0, 1e6),
      usedWh: number(battery.usedWh, 0, 1000),
      remainingSeconds: number(battery.remainingSeconds, 0, 1e9),
      low: boolean(battery.low),
      critical: boolean(battery.critical),
    },
    faults: {
      motorEfficiency: vector(
        faults.motorEfficiency,
        rotorCount(airframe),
        0,
        1,
      ),
      linkLost: boolean(faults.linkLost),
      pusherEfficiency: number(faults.pusherEfficiency, 0, 1),
    },
    propulsion: propulsion(value.propulsion),
    controlSurfaces: surfaces(value.controlSurfaces),
    flight: flight(value.flight),
    runway: runway(value.runway),
    training: training(
      value.training,
      airframe,
      scene,
      activeCourse,
      expectedHome,
      terrain.sample(...expectedHome).heightM,
    ),
  };
}
function runway(value) {
  object(value);
  return {
    onRunway: boolean(value.onRunway),
    weightOnWheels: boolean(value.weightOnWheels),
    distanceRemainingM: number(value.distanceRemainingM, 0),
    crossTrackM: number(value.crossTrackM),
    rotationSpeedMps: number(value.rotationSpeedMps, 0),
    takeoffPhase: choice(value.takeoffPhase, TAKEOFF_PHASES),
    abortReason: choice(value.abortReason, ABORT_REASONS),
    brake: number(value.brake, 0, 1),
  };
}
function propulsion(value) {
  object(value);
  return {
    command: number(value.command, 0, 1),
    motor: number(value.motor, 0, 1),
    thrustN: number(value.thrustN, 0),
  };
}
function surfaces(value) {
  object(value);
  return Object.fromEntries(
    ["aileron", "elevator", "rudder"].map((key) => [
      key,
      number(value[key], -Math.PI / 2, Math.PI / 2),
    ]),
  );
}
function flight(value) {
  object(value);
  return {
    airspeedMps: number(value.airspeedMps, 0),
    groundspeedMps: number(value.groundspeedMps, 0),
    alphaRad: number(value.alphaRad, -Math.PI, Math.PI),
    betaRad: number(value.betaRad, -Math.PI / 2, Math.PI / 2),
    dynamicPressurePa: number(value.dynamicPressurePa, 0),
    liftN: number(value.liftN),
    dragN: number(value.dragN, 0),
    loadFactor: number(value.loadFactor),
    stalled: boolean(value.stalled),
    transition: choice(value.transition, ["hover", "cruise"]),
    transitionPhase: choice(value.transitionPhase, [
      "hover",
      "accelerating",
      "cruise",
      "decelerating",
    ]),
    transitionProgress: number(value.transitionProgress, 0, 1),
  };
}
function event(input, sim, initialScene) {
  object(input);
  const result = { time: number(input.time, 0, 1800.001), type: input.type };
  if (NIST_EVENT_TYPES.includes(input.type)) return {...result,value:validateNistEventValue(input.type,input.value)};
  switch (input.type) {
    case "inspection": {
      exactKeys(
        input.value,
        ["captureId", "targetId", "config", "sha256"],
        "inspection command",
      );
      if (
        typeof input.value.captureId !== "string" ||
        !/^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/.test(
          input.value.captureId,
        ) ||
        typeof input.value.targetId !== "string" ||
        typeof input.value.sha256 !== "string" ||
        !/^[0-9a-f]{64}$/.test(input.value.sha256)
      )
        throw new Error("Invalid inspection command");
      result.value = {
        ...input.value,
        config: validateCameraConfig(input.value.config),
      };
      result.receipt = structuredClone(object(input.receipt));
      break;
    }
    case "collisionPrevention":
      result.value = validateCollisionPreventionSettings(input.value);
      sim.setCollisionPrevention(result.value);
      break;
    case "operatorGimbal":
    case "gimbal":
      result.value = validateGimbalSettings(input.value);
      break;
    case "abortTakeoff":
      if (sim.snapshot().airframe !== "fixedwing")
        throw new Error("Runway takeoff abort requires a fixed-wing airframe");
      break;
    case "transition":
      if (sim.snapshot().airframe !== "vtol")
        throw new Error("Transition events require a VTOL airframe");
      result.value = choice(input.value, ["hover", "cruise"]);
      break;
    case "sensorConfig":
      result.value = validateSensorConfig(input.value);
      assertGeographicSensors(initialScene, result.value);
      break;
    case "sensorFault": {
      const fault = object(input.value);
      if (Object.keys(fault).some((key) => !["sensor", "failed"].includes(key)))
        throw new Error("Unknown sensor fault field");
      result.value = {
        sensor: choice(fault.sensor, SENSOR_NAMES),
        failed: boolean(fault.failed),
      };
      break;
    }
    case "targets":
      result.value = targets(input.value, true);
      sim.setTargets(result.value);
      break;
    case "mode":
      result.value = choice(input.value, MODES);
      if (result.value === "takeoff" && sim.snapshot().airframe !== "fixedwing")
        throw new Error("Runway takeoff mode requires a fixed-wing airframe");
      break;
    case "position":
      sim.setMode("position");
      sim.setPositionTarget(object(input.value));
      result.value = { ...input.value };
      break;
    case "mission":
      if (!Array.isArray(input.value)) throw new Error("Invalid mission event");
      result.value = Array.from(input.value, (point) => ({ ...object(point) }));
      sim.setMission(result.value);
      break;
    case "training":
      result.value = choice(input.value, COURSES);
      break;
    case "wind": {
      const wind = object(input.value);
      if (Object.keys(wind).sort().join(",") !== "direction,speed")
        throw new Error("Invalid wind fields");
      result.value = {
        speed: number(wind.speed, 0, 30),
        direction: number(wind.direction, 0, 360),
      };
      break;
    }
    case "scene":
      result.value = validateScene(input.value);
      if (
        JSON.stringify(result.value.geography ?? null) !==
          JSON.stringify(initialScene.geography ?? null) ||
        JSON.stringify(result.value.terrain) !==
          JSON.stringify(initialScene.terrain)
      )
        throw new Error(
          "Scene geography must remain fixed throughout a recording",
        );
      sim.setTerrain(result.value.terrain);
      break;
    case "fault":
      result.fault = choice(input.fault, ["motor", "link", "pusher"]);
      sim.injectFault(result.fault, object(input.value));
      result.value = { ...input.value };
      sim.clearFaults();
      break;
    case "render":
      result.value = boolean(input.value);
      break;
    case "gust":
    case "clearFaults":
    case "takeoff":
    case "arm":
    case "disarm":
      break;
    default:
      throw new Error("Invalid recording event");
  }
  return result;
}
function unpackCollisionPrevention(value, settings) {
  if (Array.isArray(value)) {
    if (value.length !== 12 || Reflect.ownKeys(value).length !== 13 || !Array.from({ length: 12 }, (_, i) => Object.hasOwn(value, i)).every(Boolean))
      throw new Error("Invalid compact collision prevention state");
    value = { applicable: value[0], active: value[1], reason: value[2], obstacleType: value[3], obstacleId: value[4],
      desiredVelocityNe: value.slice(5, 7), constrainedVelocityNe: value.slice(7, 9), commandedAccelerationNe: value.slice(9, 11), clearanceM: value[11] };
  }
  const state = validateCollisionPreventionState(value);
  if ((state !== null) !== settings.enabled || (state && state.clearanceM !== settings.clearanceM))
    throw new Error("Collision prevention state disagrees with settings");
  return state;
}
function compactCollisionPrevention(state) {
  return state === null ? null : [state.applicable, state.active, state.reason, state.obstacleType, state.obstacleId,
    ...state.desiredVelocityNe, ...state.constrainedVelocityNe, ...state.commandedAccelerationNe, state.clearanceM];
}
export function compactSnapshot(snapshot) {
  const { waypoints, altitudeReference, positionAltitude, ...navigation } =
    object(snapshot.navigation);
  const { airframe, attitude, ground, gimbal, ...state } = snapshot;
  const mask = (flags) =>
    GIMBAL_AXES.reduce((bits, axis, i) => bits | (Number(flags[axis]) << i), 0);
  return {
    ...state,
    battery: BATTERY_FIELDS.map(key => snapshot.battery[key]),
    collisionPrevention: compactCollisionPrevention(snapshot.collisionPrevention),
    // Settings live in setup/events; retain every acquired angle and status without repeating them.
    gimbal: [
      ...GIMBAL_AXES.map((axis) => gimbal.actual[axis]),
      mask(gimbal.limited),
      mask(gimbal.rateLimited),
      gimbal.trackingErrorRad,
    ],
    navigation: {
      ...navigation,
      altitude: [["datum", "agl"].indexOf(altitudeReference), positionAltitude],
      waypointCount: waypoints?.length ?? navigation.waypointCount ?? 0,
    },
  };
}
const PROXIMITY_REASONS = ["", "startup", "fault", "dropout", "no-return"];
function compactProximityReading(value) {
  const reading = validateProximityReading(value);
  return [
    reading.time,
    PROXIMITY_REASONS.indexOf(reading.reason),
    reading.positionNed,
    reading.quaternion?.every((v, i) => Object.is(v, i === 0 ? 1 : 0)) ? 1 : reading.quaternion,
    reading.rangesM.every((v) => v === null) ? null : reading.rangesM,
    reading.tooClose.reduce((mask, flag, i) => mask | (Number(flag) << i), 0),
  ];
}
function proximityReading(value, config) {
  if (Array.isArray(value)) {
    if (value.length !== 6 || Reflect.ownKeys(value).length !== 7 ||
        !Array.from({ length: 6 }, (_, i) => Object.hasOwn(value, i)).every(Boolean))
      throw new Error("Invalid dense compact proximity reading");
    const [time, reasonCode, positionNed, quaternion, rangesM, mask] = value;
    count(reasonCode, PROXIMITY_REASONS.length - 1);
    count(mask, 63);
    value = {
      time,
      valid: reasonCode === 0,
      reason: PROXIMITY_REASONS[reasonCode],
      positionNed,
      quaternion: quaternion === 1 ? [1, 0, 0, 0] : quaternion,
      rangesM: rangesM === null ? Array(6).fill(null) : rangesM,
      tooClose: Array.from({ length: 6 }, (_, i) => Boolean(mask & (1 << i))),
      minM: config.proximityMinM,
      maxM: config.proximityMaxM,
      warningM: config.proximityWarningM,
    };
  }
  return validateProximityReading(value, config);
}
export function compactSensorFrame(value) {
  return {
    ...value,
    faults: SENSOR_NAMES.reduce(
      (mask, name, i) => mask | (Number(value.faults[name]) << i),
      0,
    ),
    radar: compactRadarReading(value.radar),
    proximity: compactProximityReading(value.proximity),
  };
}
export function validateSensorFrame(value, sensorConfig = {}) {
  exactKeys(value, ["time", "faults", ...SENSOR_NAMES], "sensor frame");
  const config = validateSensorConfig(sensorConfig),
    time = number(value.time, 0, 1800.001);
  const result = { time };
  const faults =
    typeof value.faults === "number"
      ? Object.fromEntries(
          SENSOR_NAMES.map((name, i) => [
            name,
            Boolean(
              count(value.faults, (1 << SENSOR_NAMES.length) - 1) & (1 << i),
            ),
          ]),
        )
      : object(value.faults);
  exactKeys(faults, SENSOR_NAMES, "sensor faults");
  for (const name of SENSOR_NAMES) {
    if (name === "proximity") {
      const proximity = proximityReading(value.proximity, config);
      if (proximity.time > time + 1e-8)
        throw new Error("Proximity acquisition is after its sensor frame");
      result.proximity = proximity;
      continue;
    }
    if (name === "radar") {
      const radar = validateRadarReading(value.radar, config);
      if (radar.time > time + 1e-8)
        throw new Error("Radar acquisition is after its sensor frame");
      result.radar = radar;
      continue;
    }
    const raw = object(value[name]);
    const valid = boolean(raw.valid),
      reason = choice(raw.reason, [
        "",
        "startup",
        "fault",
        "dropout",
        "no-return",
      ]);
    if (valid !== (reason === ""))
      throw new Error("Sensor validity and reason disagree");
    const channel = { time: number(raw.time, 0, time + 1e-8), valid, reason };
    const payload = (key, parse) => {
      if (!valid) {
        if (raw[key] !== null)
          throw new Error(`Invalid ${name} payload must be null`);
        return null;
      }
      return parse(raw[key]);
    };
    if (name === "gps")
      Object.assign(channel, {
        verticalDatum: choice(raw.verticalDatum, [config.originVerticalDatum]),
        latitudeDeg: payload("latitudeDeg", (v) => number(v, -90, 90)),
        longitudeDeg: payload("longitudeDeg", (v) => number(v, -180, 180)),
        altitudeM: payload("altitudeM", (v) => number(v, -1e6, 1e6)),
        velocityNedMps: payload("velocityNedMps", (v) => vector(v, 3)),
      });
    else if (name === "barometer")
      Object.assign(channel, {
        pressurePa: payload("pressurePa", (v) => number(v, 1e-9, 1e9)),
        altitudeM: payload("altitudeM", (v) => number(v, -1e6, 1e6)),
        temperatureK: payload("temperatureK", (v) => number(v, 100, 400)),
      });
    else if (name === "magnetometer")
      channel.fieldBodyMicroT = payload("fieldBodyMicroT", (v) => vector(v, 3));
    else if (name === "rangefinder")
      channel.distanceM = payload("distanceM", (v) =>
        number(v, config.rangefinderMinM, config.rangefinderMaxM),
      );
    else if (name === "opticalFlow")
      Object.assign(channel, {
        flowRadS: payload("flowRadS", (v) => vector(v, 2)),
        groundDistanceM: payload("groundDistanceM", (v) =>
          number(v, config.rangefinderMinM, config.rangefinderMaxM),
        ),
      });
    else {
      if (
        !Array.isArray(raw.rangesM) ||
        raw.rangesM.length !== config.lidarRays * config.lidarChannels
      )
        throw new Error("Invalid lidar range count");
      channel.rangesM = Array.from(raw.rangesM, (v) =>
        v === null ? null : number(v, config.lidarMinM, config.lidarMaxM),
      );
      const hasReturn = channel.rangesM.some((v) => v !== null);
      if (valid !== hasReturn)
        throw new Error("Lidar validity requires at least one real return");
      if (raw.positionNed === null && raw.quaternion === null) {
        if (valid || !["startup", "fault"].includes(reason))
          throw new Error("Missing lidar acquisition pose");
        channel.positionNed = null;
        channel.quaternion = null;
      } else {
        if (reason === "startup")
          throw new Error("Startup lidar cannot have an acquisition pose");
        channel.positionNed = vector(raw.positionNed, 3);
        channel.quaternion = vector(raw.quaternion, 4, -1, 1);
        if (Math.abs(Math.hypot(...channel.quaternion) - 1) > 1e-6)
          throw new Error("Invalid lidar pose quaternion");
      }
      channel.angleStartRad = number(raw.angleStartRad, -Math.PI, -Math.PI);
      channel.angleStepRad = number(
        raw.angleStepRad,
        (2 * Math.PI) / config.lidarRays,
        (2 * Math.PI) / config.lidarRays,
      );
      channel.channelCount = number(
        raw.channelCount,
        config.lidarChannels,
        config.lidarChannels,
      );
      channel.azimuthCount = number(
        raw.azimuthCount,
        config.lidarRays,
        config.lidarRays,
      );
      const elevationStart = (config.lidarVerticalMinDeg * Math.PI) / 180;
      const elevationStep =
        config.lidarChannels === 1
          ? 0
          : ((config.lidarVerticalMaxDeg - config.lidarVerticalMinDeg) *
              Math.PI) /
            180 /
            (config.lidarChannels - 1);
      channel.elevationStartRad = number(
        raw.elevationStartRad,
        elevationStart,
        elevationStart,
      );
      channel.elevationStepRad = number(
        raw.elevationStepRad,
        elevationStep,
        elevationStep,
      );
      channel.minM = number(raw.minM, config.lidarMinM, config.lidarMinM);
      channel.maxM = number(raw.maxM, config.lidarMaxM, config.lidarMaxM);
    }
    result[name] = channel;
  }
  for (const name of SENSOR_NAMES)
    if (boolean(faults[name]) !== (result[name].reason === "fault"))
      throw new Error("Sensor fault flag and reading reason disagree");
  result.faults = Object.fromEntries(
    SENSOR_NAMES.map((name) => [name, boolean(faults[name])]),
  );
  return result;
}

const GLOBAL_EVENTS = ["scene", "wind", "render"];
const VEHICLE_EVENTS = [
  ...NIST_EVENT_TYPES,
  "targets",
  "gust",
  "mode",
  "gimbal",
  "operatorGimbal",
  "collisionPrevention",
  "transition",
  "position",
  "mission",
  "fault",
  "clearFaults",
  "takeoff",
  "abortTakeoff",
  "arm",
  "disarm",
  "training",
  "sensorConfig",
  "sensorFault",
  "inspection",
];
function inspectionEqual(actual, expected) {
  if (typeof expected === "number")
    return (
      Number.isFinite(actual) &&
      Math.abs(actual - expected) <= 1e-9 * Math.max(1, Math.abs(expected))
    );
  if (expected === null || typeof expected !== "object")
    return actual === expected;
  if (Array.isArray(actual) !== Array.isArray(expected)) return false;
  return (
    actual &&
    typeof actual === "object" &&
    Object.keys(actual).length === Object.keys(expected).length &&
    Object.keys(expected).every(
      (key) =>
        Object.hasOwn(actual, key) &&
        inspectionEqual(actual[key], expected[key]),
    )
  );
}
function validateInspectionReceipt(e, scene, frame, roster, gimbal) {
  const receipt = e.receipt;
  exactKeys(
    receipt,
    [
      "captureId",
      "targetId",
      "sourceVehicleId",
      "worldTick",
      "metadata",
      "sha256",
      "assessment",
    ],
    "inspection receipt",
  );
  const metadata = validateCameraMetadata(receipt.metadata),
    source = frame.vehicles.find((v) => v.id === e.vehicleId).snapshot;
  exactKeys(receipt.metadata, Object.keys(metadata), "inspection metadata");
  const target = scene.inspectionTargets.find((t) => t.id === e.value.targetId);
  for (const key of ["mode", "target", "maxRate"])
    if (!inspectionEqual(metadata.gimbal[key], gimbal[key]))
      throw new Error(
        "Inspection gimbal settings disagree with exposure event order",
      );
  if (
    !target ||
    frame.tick !== e.tick ||
    receipt.captureId !== e.value.captureId ||
    receipt.targetId !== target.id ||
    receipt.sha256 !== e.value.sha256 ||
    receipt.sourceVehicleId !== e.vehicleId ||
    receipt.worldTick !== e.tick ||
    metadata.captureId !== receipt.captureId ||
    metadata.sourceVehicleId !== e.vehicleId ||
    metadata.worldTick !== e.tick ||
    !inspectionEqual(metadata.config, e.value.config) ||
    !inspectionEqual(metadata.pose.positionNed, source.position) ||
    !inspectionEqual(metadata.pose.bodyQuaternionWxyz, source.quaternion) ||
    !inspectionEqual(metadata.time, source.time) ||
    metadata.airframe !== source.airframe ||
    !inspectionEqual(metadata.gimbal.actual, source.gimbal.actual) ||
    !inspectionEqual(metadata.geography, scene.geography)
  )
    throw new Error("Inspection receipt disagrees with its exposure tick");
  const peers = frame.vehicles
    .filter((v) => v.id !== e.vehicleId)
    .map((v) => ({
      id: v.id,
      position: v.snapshot.position,
      velocity: v.snapshot.velocity,
      radius: getVehicleEnvelope(roster.find((spec) => spec.id === v.id).config)
        .radius,
    }));
  const assessment = assessInspection(
    target,
    scene.obstacles,
    createTerrain(scene.terrain),
    metadata,
    peers,
    source.velocity,
  );
  if (!inspectionEqual(receipt.assessment, assessment))
    throw new Error("Inspection assessment disagrees with captured geometry");
  return receipt;
}
function exactKeys(value, keys, label) {
  object(value);
  if (
    Reflect.ownKeys(value).length !== keys.length ||
    Reflect.ownKeys(value).some((key) => !keys.includes(key))
  )
    throw new Error(`Invalid ${label} fields`);
}
function frameTime(value) {
  const tick = count(value.tick, 360000),
    time = number(value.time, 0, 1800.001);
  if (Math.abs(time - tick * 0.005) > 1e-7)
    throw new Error("World tick and time disagree");
  return { tick, time };
}
function orderedVehicles(input, roster, label) {
  if (
    !Array.isArray(input) ||
    input.length !== roster.length ||
    Array.from(input).some((entry, i) => object(entry).id !== roster[i].id)
  )
    throw new Error(`Invalid ${label} roster or order`);
}
function collisions(input, roster, tick, time) {
  if (!Array.isArray(input) || input.length > 28)
    throw new Error("Invalid collision list");
  let previous = null;
  return Array.from(input, (value) => {
    exactKeys(value, ["time", "tick", "vehicleIds", "fraction"], "collision");
    const pair = value.vehicleIds;
    if (
      !Array.isArray(pair) ||
      pair.length !== 2 ||
      typeof pair[0] !== "string" ||
      pair[0] >= pair[1] ||
      pair.some((id) => !roster.some((v) => v.id === id)) ||
      (previous &&
        (pair[0] < previous[0] ||
          (pair[0] === previous[0] && pair[1] <= previous[1]))) ||
      count(value.tick, 360000) !== tick ||
      Math.abs(number(value.time, 0, 1800.001) - time) > 1e-8
    )
      throw new Error("Invalid collision identity or time");
    previous = [...pair];
    return {
      time,
      tick,
      vehicleIds: [...pair],
      fraction: number(value.fraction, 0, 1),
    };
  });
}
export function compactWorldSnapshot(world) {
  return {
    tick: world.tick,
    time: world.time,
    vehicles: world.vehicles.map((v) => ({
      id: v.id,
      snapshot: compactSnapshot({
        ...v.snapshot,
        training: v.training ?? v.snapshot.training,
      }),
    })),
    collisions: structuredClone(world.collisions),
  };
}
export function worldSensorFrame(world) {
  return {
    tick: world.tick,
    time: world.time,
    vehicles: world.vehicles.map((v) => ({
      id: v.id,
      sensors: structuredClone(v.sensors),
    })),
  };
}
export function validateRecording(value) {
  object(value);
  if (value.version !== RECORDING_VERSION)
    throw new Error(
      "Unsupported recording version: version 20 required; legacy recordings are not supported",
    );
  exactKeys(
    value,
    [
      "format",
      "version",
      "modelVersion",
      "createdAt",
      "dt",
      "world",
      "runtime",
      "events",
      "samples",
      "sensorFrames",
      "termination",
    ],
    "recording",
  );
  if (value.modelVersion !== MODEL_VERSION)
    throw new Error(`Unsupported simulation model: ${MODEL_VERSION} required`);
  if (
    value.format !== "flight-lab" ||
    typeof value.createdAt !== "string" ||
    !Number.isFinite(Date.parse(value.createdAt)) ||
    value.dt !== 0.005
  )
    throw new Error("Invalid recording header");
  const world = validateWorldConfig(value.world),
    runtime = object(value.runtime);
  exactKeys(
    runtime,
    [
      "speed",
      "wallSeconds",
      "p95LateMs",
      "lateTicks",
      "skippedWallSeconds",
      "renderEnabled",
    ],
    "runtime",
  );
  const cleanRuntime = {
    speed: choice(runtime.speed, [0.5, 1, 2, 4]),
    wallSeconds: number(runtime.wallSeconds, 0),
    p95LateMs: number(runtime.p95LateMs, 0),
    lateTicks: count(runtime.lateTicks),
    skippedWallSeconds: number(runtime.skippedWallSeconds, 0),
    renderEnabled: boolean(runtime.renderEnabled),
  };
  if (
    !Array.isArray(value.events) ||
    value.events.length > 100000 ||
    !Array.isArray(value.samples) ||
    !value.samples.length ||
    value.samples.length > 136002 ||
    !Array.isArray(value.sensorFrames) ||
    value.sensorFrames.length !== value.samples.length
  )
    throw new Error("Invalid recording length");
  const contexts = new Map(
    world.vehicles.map((v) => [
      v.id,
      {
        vehicle: v,
        setup: structuredClone(v.setup),
        routeText: JSON.stringify(v.setup.mission),
        routeCache: new WeakMap(),
        trainingEpochTime: 0,
        previousTutorial: undefined,
        inspectionReceipts: [],
        channelTimes: {},
        validator: createSimulation(v.config, {
          gimbal: v.setup.gimbal,
          collisionPrevention: v.setup.collisionPrevention,
          terrain: world.scene.terrain,
          spawn: v.spawn,
          home: v.home,
        }),
      },
    ]),
  );
  let lastEvent = -1;
  const events = Array.from(value.events, (input) => {
    object(input);
    const { tick, time } = frameTime(input),
      global = GLOBAL_EVENTS.includes(input.type);
    if (!global && !VEHICLE_EVENTS.includes(input.type))
      throw new Error("Invalid recording event");
    const context = global
      ? contexts.values().next().value
      : contexts.get(input.vehicleId);
    if (!context) throw new Error("Unknown event vehicleId");
    const parsed = event(input, context.validator, world.scene);
    const keys = [
      "tick",
      "time",
      "type",
      ...(!global ? ["vehicleId"] : []),
      ...(Object.hasOwn(parsed, "value") ? ["value"] : []),
      ...(Object.hasOwn(parsed, "fault") ? ["fault"] : []),
      ...(input.type === "inspection" ? ["receipt"] : []),
    ];
    exactKeys(input, keys, "event");
    if (tick < lastEvent) throw new Error("Events out of order");
    lastEvent = tick;
    return {
      ...parsed,
      tick,
      ...(!global ? { vehicleId: input.vehicleId } : {}),
    };
  });
  let activeScene = world.scene,
    activeTerrain = createTerrain(activeScene.terrain),
    eventIndex = 0,
    lastTick = -1,
    lastTime = -1;
  const sensorFrames = [];
  const captureIds = new Set();
  const samples = Array.from(value.samples, (input, index) => {
    exactKeys(
      input,
      ["tick", "time", "vehicles", "collisions"],
      "world sample",
    );
    const { tick, time } = frameTime(input);
    if (tick <= lastTick || (index === 0 && tick !== 0))
      throw new Error("Samples out of order or missing initial tick");
    const inspectionActions = [];
    const geometryChanges = [];
    while (eventIndex < events.length && events[eventIndex].tick <= tick) {
      const e = events[eventIndex++];
      if (e.type === "scene") {
        if (
          JSON.stringify(activeScene.obstacles) !==
          JSON.stringify(e.value.obstacles)
        )
          geometryChanges.push(e);
        inspectionActions.push({ clear: true });
        const changed =
          JSON.stringify(activeScene.customGates) !==
          JSON.stringify(e.value.customGates);
        activeScene = e.value;
        activeTerrain = createTerrain(activeScene.terrain);
        if (changed)
          for (const c of contexts.values())
            if (c.setup.training === "custom") {
              c.trainingEpochTime = e.time;
              c.previousTutorial = undefined;
            }
      } else if (e.vehicleId) {
        const c = contexts.get(e.vehicleId);
        if (e.type === "inspection")
          inspectionActions.push({
            event: e,
            scene: activeScene,
            gimbal: structuredClone(c.setup.gimbal),
          });
        if (e.type === "training") {
          c.setup.training = e.value;
          c.trainingEpochTime = e.time;
          c.previousTutorial = undefined;
        }
        if (["mission", "gimbal", "operatorGimbal", "mode", "collisionPrevention"].includes(e.type))
          c.setup[e.type === "operatorGimbal" ? "gimbal" : e.type] = e.value;
        if (e.type === "mission") c.routeText = JSON.stringify(e.value);
        if (e.type === "sensorConfig") {
          c.setup.sensors = e.value;
          c.channelTimes = {};
        }
      }
    }
    orderedVehicles(input.vehicles, world.vehicles, "sample");
    const rawSensors = value.sensorFrames[index];
    exactKeys(rawSensors, ["tick", "time", "vehicles"], "sensor frame");
    frameTime(rawSensors);
    if (rawSensors.tick !== tick || Math.abs(rawSensors.time - time) > 1e-8)
      throw new Error("Sensor frame time must match world sample");
    orderedVehicles(rawSensors.vehicles, world.vehicles, "sensor frame");
    const sensorVehicles = [];
    const vehicles = Array.from(input.vehicles, (entry, i) => {
      exactKeys(entry, ["id", "snapshot"], "vehicle sample");
      const c = contexts.get(entry.id),
        v = c.vehicle;
      if (
        !isCourseSupported(c.setup.training, v.config.airframe) ||
        (c.setup.training === "custom" && !activeScene.customGates.length) ||
        (c.setup.training === "runway" && !activeTerrain.runwayCompatible)
      )
        throw new Error("Invalid active training course");
      const state = sample(
        entry.snapshot,
        c.setup.mission,
        c.routeText,
        c.routeCache,
        v.config.airframe,
        activeScene,
        c.setup.gimbal,
        activeTerrain,
        v.home,
        c.setup.training,
        c.setup.collisionPrevention,
      );
      const prevention = state.collisionPrevention;
      if ((prevention?.obstacleType === "obstacle" && Number(prevention.obstacleId) >= activeScene.obstacles.length) ||
          (prevention?.obstacleType === "peer" && (!contexts.has(prevention.obstacleId) || prevention.obstacleId === entry.id)))
        throw new Error("Collision prevention references unknown geometry");
      if (Math.abs(state.time - time) > 1e-8 || state.metrics.steps !== tick)
        throw new Error("Vehicle clock must match world clock");
      if (state.training.course === "tutorial") {
        const p = state.training,
          old = c.previousTutorial;
        if (
          !old
            ? p.passed !== 0 || p.reason !== "" || p.elapsed > 0.051
            : time - lastTime > 0.051 ||
              p.passed < old.passed ||
              p.passed > old.passed + 1 ||
              ((old.complete || old.reason) &&
                JSON.stringify(p) !== JSON.stringify(old))
        )
          throw new Error("Invalid tutorial history");
        if (
          !p.complete &&
          !p.reason &&
          Math.abs(p.elapsed - (time - c.trainingEpochTime)) > 1e-7
        )
          throw new Error("Tutorial time does not match course epoch");
        c.previousTutorial = p;
      }
      const sensorEntry = rawSensors.vehicles[i];
      exactKeys(sensorEntry, ["id", "sensors"], "vehicle sensor frame");
      const sensors = validateSensorFrame(sensorEntry.sensors, c.setup.sensors);
      if (Math.abs(sensors.time - time) > 1e-8)
        throw new Error("Vehicle sensors must match world clock");
      for (const name of SENSOR_NAMES) {
        if (sensors[name].time < (c.channelTimes[name] ?? 0) - 1e-8)
          throw new Error("Sensor acquisition time moved backwards");
        c.channelTimes[name] = sensors[name].time;
      }
      sensorVehicles.push({ id: entry.id, sensors });
      return { id: entry.id, snapshot: state };
    });
    for (const change of geometryChanges) {
      if (change.tick !== tick)
        throw new Error("Obstacle changes require their retained world tick");
      for (const vehicle of vehicles) {
        const radius = getVehicleEnvelope(
          contexts.get(vehicle.id).vehicle.config,
        ).radius;
        if (
          change.value.obstacles.some((box) =>
            obstacleContainsVehicle(box, vehicle.snapshot.position, radius),
          )
        )
          throw new Error(
            "Obstacle scene change intersects a retained vehicle envelope",
          );
      }
    }
    for (const action of inspectionActions) {
      if (action.clear) {
        for (const context of contexts.values())
          context.inspectionReceipts = [];
        captureIds.clear();
      } else {
        const e = action.event,
          context = contexts.get(e.vehicleId);
        if (
          captureIds.has(e.value.captureId) ||
          context.inspectionReceipts.length >= 100
        )
          throw new Error("Duplicate or excessive inspection captures");
        const receipt = validateInspectionReceipt(
          e,
          action.scene,
          { tick, vehicles },
          world.vehicles,
          action.gimbal,
        );
        captureIds.add(receipt.captureId);
        context.inspectionReceipts.push(receipt);
      }
    }
    for (const [i, vehicle] of vehicles.entries()) {
      const expected = activeScene.inspectionTargets.length
        ? inspectionSummary(
            contexts.get(vehicle.id).inspectionReceipts,
            activeScene.inspectionTargets,
          )
        : null;
      if (!inspectionEqual(input.vehicles[i].snapshot.inspection, expected))
        throw new Error("Inspection progress disagrees with retained receipts");
      vehicle.snapshot.inspection = structuredClone(expected);
    }
    const contacts = collisions(input.collisions, world.vehicles, tick, time);
    for (const contact of contacts)
      for (const id of contact.vehicleIds) {
        const state = vehicles.find((v) => v.id === id).snapshot;
        if (
          state.status !== "crashed" ||
          state.navigation.reason !== "vehicle-collision"
        )
          throw new Error("Collision must crash both aircraft");
      }
    lastTick = tick;
    lastTime = time;
    sensorFrames.push({ tick, time, vehicles: sensorVehicles });
    return { tick, time, vehicles, collisions: contacts };
  });
  if (lastEvent > lastTick) throw new Error("Event occurs after last sample");
  let termination = null;
  if (value.termination !== null) {
    exactKeys(value.termination, ["reason", "time"], "termination");
    termination = {
      reason: choice(value.termination.reason, [
        "terrain-coverage",
        "vehicle-collision",
      ]),
      time: number(value.termination.time, 0, 1800.001),
    };
    if (termination.time !== lastTime)
      throw new Error("Termination must match final world time");
    if (
      termination.reason === "terrain-coverage" &&
      activeTerrain.data.kind !== "dem"
    )
      throw new Error("Coverage termination requires DEM");
    if (
      termination.reason === "vehicle-collision" &&
      !samples.at(-1).collisions.length
    )
      throw new Error("Missing final collision");
  }
  if (
    samples.some(
      (frame, i) =>
        frame.collisions.length &&
        (i !== samples.length - 1 ||
          frame.tick === 0 ||
          termination?.reason !== "vehicle-collision"),
    )
  )
    throw new Error("Collision requires the final stopped world frame");
  const nist = createNistSession();
  let nistScene = world.scene;
  const exposureFrames = new Map(samples.map(frame => [frame.tick,frame]));
  const evidenceIds = new Set();
  const nistGimbals = new Map(world.vehicles.map(v => [v.id, structuredClone(v.setup.gimbal)]));
  for (const e of events) {
    if (["gimbal", "operatorGimbal"].includes(e.type)) nistGimbals.set(e.vehicleId,e.value);
    if (["inspection","nistCapture"].includes(e.type)) {
      if (evidenceIds.has(e.value.captureId)) throw new Error("拍攝識別碼重複");
      evidenceIds.add(e.value.captureId);
    }
    if (e.type === "scene") {
      if (nist.getState().phase === "running") throw new Error("NIST 練習期間不得更換場景");
      nistScene = e.value;
    }
    if (NIST_EVENT_TYPES.includes(e.type)) {
      const frame = exposureFrames.get(e.tick);
      if (!frame) throw new Error("NIST 事件缺少對應完整世界影格");
      if (e.type === "nistEnd" && e.value.reason === "time-limit") {
        const session=nist.getState();
        if (e.tick !== Math.min(360000,session.startTick + session.durationSeconds*200)) throw new Error("NIST 結束時刻不符合時間上限");
      }
      nist.apply(e,nistExposureFrame(frame,e,nistGimbals.get(e.vehicleId)),nistScene);
    }
  }
  const finalSession = nist.getState(lastTick);
  if (finalSession.phase === "running" && (termination !== null || lastTick === 360000 || samples.at(-1).vehicles.find(v => v.id === finalSession.vehicleId)?.snapshot.status === "crashed")) throw new Error("NIST 練習缺少模擬終止事件");
  if (finalSession.phase === "running" && lastTick >= finalSession.startTick + finalSession.durationSeconds*200) throw new Error("NIST 練習缺少時間上限結束事件");
  return {
    format: "flight-lab",
    version: RECORDING_VERSION,
    modelVersion: MODEL_VERSION,
    createdAt: value.createdAt,
    dt: 0.005,
    world,
    runtime: cleanRuntime,
    events,
    samples,
    sensorFrames,
    termination,
  };
}
export function recordingToJson(record) {
  const clean = validateRecording(record);
  return JSON.stringify({
    ...clean,
    sensorFrames: clean.sensorFrames.map((frame) => ({
      ...frame,
      vehicles: frame.vehicles.map((v) => ({
        id: v.id,
        sensors: compactSensorFrame(v.sensors),
      })),
    })),
    samples: clean.samples.map((frame) => ({
      ...frame,
      vehicles: frame.vehicles.map((v) => ({
        id: v.id,
        snapshot: compactSnapshot(v.snapshot),
      })),
    })),
  });
}
export function recordingWorldAt(record, index) {
  if (!Number.isInteger(index) || index < 0 || index >= record.samples.length)
    throw new Error("Invalid recording frame index");
  const frame = record.samples[index],
    config = structuredClone(record.world);
  for (const e of record.events) {
    if (e.tick > frame.tick) break;
    if (e.type === "scene") config.scene = e.value;
    if (e.type === "wind") {
      config.wind = e.value;
      for (const v of config.vehicles) {
        v.config.wind = e.value.speed;
        v.config.windDirection = e.value.direction;
      }
    }
    if (e.vehicleId) {
      const v = config.vehicles.find((v) => v.id === e.vehicleId);
      if (["mission", "gimbal", "operatorGimbal", "mode", "training", "collisionPrevention"].includes(e.type))
        v.setup[e.type === "operatorGimbal" ? "gimbal" : e.type] = e.value;
      if (e.type === "sensorConfig") v.setup.sensors = e.value;
    }
  }
  return {
    tick: frame.tick,
    time: frame.time,
    scene: config.scene,
    wind: config.wind,
    collisions: frame.collisions,
    vehicles: config.vehicles.map((v, i) => ({
      ...v,
      setup: { ...v.setup, mode: frame.vehicles[i].snapshot.navigation.mode },
      snapshot: frame.vehicles[i].snapshot,
      sensors: record.sensorFrames[index].vehicles[i].sensors,
      training: frame.vehicles[i].snapshot.training,
      inspection: frame.vehicles[i].snapshot.inspection,
    })),
  };
}
function nistExposureFrame(frame, event, gimbal) {
  if (event.type !== "nistCapture") return frame;
  const acquired = event.value.metadata.gimbal;
  for (const key of ["mode","target","maxRate"])
    if (!inspectionEqual(acquired[key],gimbal[key])) throw new Error("NIST 雲台設定不符合曝光事件順序");
  const source = frame.vehicles.find(v => v.id === event.vehicleId);
  if (!source || !inspectionEqual(acquired.actual,source.snapshot.gimbal.actual)) throw new Error("NIST 雲台姿態不符合曝光時刻");
  return {...frame, vehicles:frame.vehicles.map(v => v.id !== event.vehicleId ? v : ({...v,snapshot:{...v.snapshot,gimbal:acquired}}))};
}
export function recordingNistState(record, index) {
  if (!Number.isInteger(index) || index < 0 || index >= record.samples.length) throw new Error("NIST 重播影格索引不正確");
  const tick = record.samples[index].tick, session = createNistSession();
  const frames = new Map(record.samples.map(frame => [frame.tick, frame]));
  let scene = record.world.scene;
  const gimbals = new Map(record.world.vehicles.map(v => [v.id, v.setup.gimbal]));
  for (const event of record.events) {
    if (event.tick > tick) break;
    if (event.type === "scene") scene = event.value;
    if (["gimbal", "operatorGimbal"].includes(event.type)) gimbals.set(event.vehicleId,event.value);
    if (NIST_EVENT_TYPES.includes(event.type)) session.apply(event, nistExposureFrame(frames.get(event.tick),event,gimbals.get(event.vehicleId)), scene);
  }
  return session.getState(tick);
}
export function recordingInspectionReport(record, index, vehicleId) {
  const state = recordingWorldAt(record, index);
  if (!state.vehicles.some((v) => v.id === vehicleId))
    throw new Error("Unknown inspection source vehicle");
  let receipts = [];
  for (const e of record.events) {
    if (e.tick > state.tick) break;
    if (e.type === "scene") receipts = [];
    else if (e.type === "inspection" && e.vehicleId === vehicleId)
      receipts.push(e.receipt);
  }
  const { latest, ...summary } = inspectionSummary(
    receipts,
    state.scene.inspectionTargets,
  );
  return structuredClone({
    format: "flight-lab-inspection",
    version: INSPECTION_REPORT_VERSION,
    sourceVehicleId: vehicleId,
    scene: state.scene,
    receipts,
    summary,
  });
}
export function recordingToCsv(record) {
  const clean = validateRecording(record);
  const samples = clean.samples.flatMap((frame) =>
    frame.vehicles.map((v) => ({
      ...v.snapshot,
      vehicleId: v.id,
      worldTick: frame.tick,
    })),
  );
  const sensorFrames = clean.sensorFrames.flatMap((frame) =>
    frame.vehicles.map((v) => v.sensors),
  );
  const header = [
    "vehicle_id",
    "world_tick",
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
    "navigation_mode_code",
    "battery_soc_fraction",
    "battery_voltage_V",
    "battery_current_A",
    "battery_power_W",
    "battery_used_Wh",
    "battery_remaining_s",
    "armed_boolean",
    ...[1, 2, 3, 4].map((i) => `motor_${i}_efficiency_fraction`),
    "link_lost_boolean",
    "navigation_waypoint_index",
    "fence_breached_boolean",
    "training_course_code",
    "training_elapsed_s",
    "training_passed_count",
    "training_total_count",
    "training_misses_count",
    "training_complete_boolean",
    "training_score_points",
    "training_hold_s",
    ...SENSOR_NAMES.filter((name) => !["radar", "proximity"].includes(name)).flatMap((name) => [
      `${name}_sample_time_s`,
      `${name}_valid_boolean`,
    ]),
    "gps_latitude_deg",
    "gps_longitude_deg",
    "gps_altitude_m",
    "gps_velocity_n_m_s",
    "gps_velocity_e_m_s",
    "gps_velocity_d_m_s",
    "barometer_pressure_Pa",
    "barometer_altitude_m",
    "barometer_temperature_K",
    "magnetometer_x_uT",
    "magnetometer_y_uT",
    "magnetometer_z_uT",
    "rangefinder_distance_m",
    "lidar_return_count",
    "lidar_nearest_m",
    "optical_flow_u_rad_s",
    "optical_flow_v_rad_s",
    "optical_flow_ground_distance_m",
    "airframe_code",
    "target_airspeed_m_s",
    "pusher_command_normalized",
    "pusher_motor_normalized",
    "pusher_thrust_N",
    "aileron_rad",
    "elevator_rad",
    "rudder_rad",
    "airspeed_m_s",
    "groundspeed_m_s",
    "alpha_rad",
    "beta_rad",
    "dynamic_pressure_Pa",
    "lift_N",
    "drag_N",
    "load_factor",
    "stalled_boolean",
    "transition_code",
    "transition_phase_code",
    "transition_progress_fraction",
    "pusher_efficiency_fraction",
    "on_runway_boolean",
    "weight_on_wheels_boolean",
    "runway_distance_remaining_m",
    "runway_cross_track_m",
    "rotation_speed_m_s",
    "takeoff_phase_code",
    "takeoff_abort_reason_code",
    "wheel_brake_fraction",
    "gimbal_mode_code",
    ...["target", "maxRate", "actual"].flatMap((field) =>
      GIMBAL_AXES.map(
        (axis) =>
          `gimbal_${field}_${axis}_${field === "maxRate" ? "rad_s" : "rad"}`,
      ),
    ),
    ...["limited", "rateLimited"].flatMap((field) =>
      GIMBAL_AXES.map((axis) => `gimbal_${field}_${axis}_boolean`),
    ),
    "gimbal_tracking_error_rad",
    "lift_motor_channel_count",
    ...[5, 6, 7, 8].map((i) => `motor_${i}_normalized`),
    ...[5, 6, 7, 8].map((i) => `pwm_${i}_us`),
    ...[5, 6, 7, 8].map((i) => `motor_${i}_efficiency_fraction`),
    "ground_height_m",
    "ground_clearance_m",
    "ground_normal_n",
    "ground_normal_e",
    "ground_normal_d",
    "position_altitude_reference_code",
    "position_altitude_m",
    "gps_vertical_datum_code",
    "radar_sample_time_s",
    "radar_valid_boolean",
    "radar_return_count",
    "radar_nearest_m",
    "radar_nearest_range_rate_m_s",
    "radar_nearest_azimuth_rad",
    "radar_nearest_elevation_rad",
    "proximity_sample_time_s",
    "proximity_valid_boolean",
    "proximity_reason",
    ...["north", "east", "down"].map((axis) => `proximity_acquisition_${axis}_m`),
    ...["w", "x", "y", "z"].map((axis) => `proximity_acquisition_quat_${axis}`),
    "proximity_min_m",
    "proximity_max_m",
    "proximity_warning_m",
    ...PROXIMITY_AXES.map(({ id }) => `proximity_${id}_range_m`),
    ...PROXIMITY_AXES.map(({ id }) => `proximity_${id}_too_close_boolean`),
    "battery_open_circuit_voltage_V",
    "battery_motor_speed_scale_fraction",
    "battery_power_limited_boolean",
    "collision_prevention_enabled_boolean",
    "collision_prevention_applicable_boolean",
    "collision_prevention_active_boolean",
    "collision_prevention_reason",
    "collision_prevention_obstacle_type",
    "collision_prevention_obstacle_id",
    "collision_prevention_desired_velocity_n_m_s",
    "collision_prevention_desired_velocity_e_m_s",
    "collision_prevention_constrained_velocity_n_m_s",
    "collision_prevention_constrained_velocity_e_m_s",
    "collision_prevention_commanded_acceleration_n_m_s2",
    "collision_prevention_commanded_acceleration_e_m_s2",
    "collision_prevention_clearance_m",
  ];
  const rows = samples.map((s, index) =>
    [
      s.vehicleId,
      s.worldTick,
      s.time,
      ...s.position,
      ...s.velocity,
      ...s.quaternion,
      ...s.attitude,
      ...s.rates,
      ...s.motors.slice(0, 4),
      ...s.pwm.slice(0, 4),
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
      MODES.indexOf(s.navigation.mode),
      s.battery.soc,
      s.battery.voltage,
      s.battery.current,
      s.battery.power,
      s.battery.usedWh,
      s.battery.remainingSeconds,
      Number(s.armed),
      ...s.faults.motorEfficiency.slice(0, 4),
      Number(s.faults.linkLost),
      s.navigation.index,
      Number(s.navigation.fenceBreached),
      COURSES.indexOf(s.training.course),
      s.training.elapsed,
      s.training.passed,
      s.training.total,
      s.training.misses,
      Number(s.training.complete),
      s.training.score,
      s.training.holdSeconds,
      ...SENSOR_NAMES.filter((name) => !["radar", "proximity"].includes(name)).flatMap((name) => [
        sensorFrames[index][name].time,
        Number(sensorFrames[index][name].valid),
      ]),
      ...sensorCsv(sensorFrames[index]),
      AIRFRAMES.indexOf(s.airframe),
      s.target.airspeed,
      s.propulsion.command,
      s.propulsion.motor,
      s.propulsion.thrustN,
      s.controlSurfaces.aileron,
      s.controlSurfaces.elevator,
      s.controlSurfaces.rudder,
      s.flight.airspeedMps,
      s.flight.groundspeedMps,
      s.flight.alphaRad,
      s.flight.betaRad,
      s.flight.dynamicPressurePa,
      s.flight.liftN,
      s.flight.dragN,
      s.flight.loadFactor,
      Number(s.flight.stalled),
      ["hover", "cruise"].indexOf(s.flight.transition),
      ["hover", "accelerating", "cruise", "decelerating"].indexOf(
        s.flight.transitionPhase,
      ),
      s.flight.transitionProgress,
      s.faults.pusherEfficiency,
      Number(s.runway.onRunway),
      Number(s.runway.weightOnWheels),
      s.runway.distanceRemainingM,
      s.runway.crossTrackM,
      s.runway.rotationSpeedMps,
      TAKEOFF_PHASES.indexOf(s.runway.takeoffPhase),
      ABORT_REASONS.indexOf(s.runway.abortReason),
      s.runway.brake,
      ["body", "ned"].indexOf(s.gimbal.mode),
      ...["target", "maxRate", "actual"].flatMap((field) =>
        GIMBAL_AXES.map((axis) => s.gimbal[field][axis]),
      ),
      ...["limited", "rateLimited"].flatMap((field) =>
        GIMBAL_AXES.map((axis) => Number(s.gimbal[field][axis])),
      ),
      s.gimbal.trackingErrorRad,
      s.motors.length,
      ...[4, 5, 6, 7].map((i) => s.motors[i] ?? ""),
      ...[4, 5, 6, 7].map((i) => s.pwm[i] ?? ""),
      ...[4, 5, 6, 7].map((i) => s.faults.motorEfficiency[i] ?? ""),
      s.ground.heightM,
      s.ground.clearanceM,
      ...s.ground.normalNed,
      ["datum", "agl"].indexOf(s.navigation.altitudeReference),
      s.navigation.positionAltitude,
      ["unspecified", "ellipsoidal", "orthometric"].indexOf(
        sensorFrames[index].gps.verticalDatum,
      ),
      ...radarCsv(sensorFrames[index].radar),
      ...proximityCsv(sensorFrames[index].proximity),
      s.battery.openCircuitVoltage,
      s.battery.motorSpeedScale,
      Number(s.battery.powerLimited),
      ...collisionPreventionCsv(s.collisionPrevention),
    ].join(","),
  );
  return [header.join(","), ...rows].join("\n");
}

function sensorCsv(frame) {
  const lidarRanges = frame.lidar.rangesM.filter((v) => v !== null);
  return [
    frame.gps.latitudeDeg,
    frame.gps.longitudeDeg,
    frame.gps.altitudeM,
    ...(frame.gps.velocityNedMps ?? [null, null, null]),
    frame.barometer.pressurePa,
    frame.barometer.altitudeM,
    frame.barometer.temperatureK,
    ...(frame.magnetometer.fieldBodyMicroT ?? [null, null, null]),
    frame.rangefinder.distanceM,
    frame.lidar.valid ? lidarRanges.length : null,
    frame.lidar.valid ? Math.min(...lidarRanges) : null,
    ...(frame.opticalFlow.flowRadS ?? [null, null]),
    frame.opticalFlow.groundDistanceM,
  ].map((v) => v ?? "");
}

function radarCsv(reading) {
  const nearest = reading.detections.reduce(
    (best, d) => (!best || d.rangeM < best.rangeM ? d : best),
    null,
  );
  return [
    reading.time,
    Number(reading.valid),
    reading.detections.length,
    nearest?.rangeM ?? "",
    nearest?.rangeRateMps ?? "",
    nearest?.azimuthRad ?? "",
    nearest?.elevationRad ?? "",
  ];
}

function proximityCsv(reading) {
  return [
    reading.time,
    Number(reading.valid),
    reading.reason,
    ...(reading.positionNed ?? [null, null, null]),
    ...(reading.quaternion ?? [null, null, null, null]),
    reading.minM,
    reading.maxM,
    reading.warningM,
    ...reading.rangesM,
    ...reading.tooClose.map(Number),
  ].map((v) => v ?? "");
}


function collisionPreventionCsv(state) {
  return state === null ? [0, ...Array(12).fill("")] : [1, Number(state.applicable), Number(state.active), state.reason,
    state.obstacleType ?? "", state.obstacleId ?? "", ...state.desiredVelocityNe, ...state.constrainedVelocityNe,
    ...state.commandedAccelerationNe, state.clearanceM];
}
