import { createSimulation, validateConfig, bodyToNedAngles } from "./sim.js";
import { DEFAULT_SENSOR_CONFIG, validateSensorConfig } from "./sensors.js";
import { TRAINING_COURSES, resolveTrainingGates } from "./training.js";
import { DEFAULT_SCENE, validateScene } from "./scene-data.js";
import { AIRFRAMES, rotorCount } from "./rotor-geometry.js";
import {
  DEFAULT_GIMBAL_SETTINGS,
  validateGimbalSettings,
  validateGimbalState,
} from "./gimbal.js";

export const RECORDING_VERSION = 9;
export const MODEL_VERSION = "0.6.0";
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
const COURSES = [...Object.keys(TRAINING_COURSES), "custom"];
const GIMBAL_AXES = ["yaw", "pitch", "roll"];
export const DEFAULT_SETUP = Object.freeze({
  sensors: DEFAULT_SENSOR_CONFIG,
  gimbal: DEFAULT_GIMBAL_SETTINGS,
  mode: "attitude",
  mission: Object.freeze([]),
  scene: DEFAULT_SCENE,
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
  if (
    Object.keys(input).some(
      (key) => !["mode", "mission", "sensors", "scene", "gimbal"].includes(key),
    )
  )
    throw new Error("Unknown setup field");
  const merged = { ...DEFAULT_SETUP, ...input };
  const gimbal = validateGimbalSettings(merged.gimbal);
  const sim = createSimulation(validateConfig(object(config)), { gimbal });
  if (!Array.isArray(merged.mission))
    throw new Error("Mission must be an array");
  const mission = Array.from(merged.mission, (point) => ({ ...object(point) }));
  if (mission.length) sim.setMission(mission);
  const scene = validateScene(merged.scene, sim.snapshot().airframe);
  const mode = choice(merged.mode, MODES);
  sim.setMode(mode);
  return {
    mode,
    mission,
    scene,
    sensors: validateSensorConfig(merged.sensors),
    gimbal,
  };
}
function targets(value, partial = false, automatic = false) {
  object(value);
  const limits = {
    altitude: [0, 500],
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
        automatic && key === "altitude" ? 0 : min,
        automatic && key === "altitude" ? 1e9 : max,
      );
  return result;
}
function training(value, airframe, scene) {
  object(value);
  const course = choice(value.course, COURSES),
    gates = resolveTrainingGates(scene, airframe),
    total = gates.length;
  if (course !== scene.training)
    throw new Error("Training course does not match active scene");
  if (count(value.total, 50) !== total)
    throw new Error("Invalid training gate total");
  const passed = count(value.passed, total);
  if (gates.length && boolean(value.complete) !== (passed === total))
    throw new Error("Training completion disagrees with gate progress");
  if (!gates.length) {
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
      north: number(gate.north, -500, 500),
      east: number(gate.east, -500, 500),
      altitude: number(gate.altitude, 0, 500),
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
    reason: choice(value.reason, ["", "crashed", "takeoff-aborted"]),
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
    battery = object(value.battery),
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
  if (home.some((v) => v !== 0)) throw new Error("Invalid home position");
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
  return {
    airframe,
    time: number(value.time, 0, 1800.001),
    position: vector(value.position, 3),
    velocity: vector(value.velocity, 3),
    quaternion,
    gimbal,
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
        "ground-impact",
        "takeoff-aborted",
      ]),
    },
    battery: {
      soc: number(battery.soc, 0, 1),
      voltage: number(battery.voltage, 0, 100),
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
    training: training(value.training, airframe, scene),
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
function event(input, sim) {
  object(input);
  const result = { time: number(input.time, 0, 1800.001), type: input.type };
  switch (input.type) {
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
      sim.setMode("attitude");
      sim.setPositionTarget(object(input.value));
      result.value = { ...input.value };
      break;
    case "mission":
      if (!Array.isArray(input.value)) throw new Error("Invalid mission event");
      result.value = Array.from(input.value, (point) => ({ ...object(point) }));
      sim.setMission(result.value);
      break;
    case "scene":
      result.value = validateScene(input.value, sim.snapshot().airframe);
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
export function compactSnapshot(snapshot) {
  const { waypoints, ...navigation } = object(snapshot.navigation);
  const { airframe, attitude, gimbal, ...state } = snapshot;
  const mask = (flags) =>
    GIMBAL_AXES.reduce((bits, axis, i) => bits | (Number(flags[axis]) << i), 0);
  return {
    ...state,
    // Settings live in setup/events; retain every acquired angle and status without repeating them.
    gimbal: [
      ...GIMBAL_AXES.map((axis) => gimbal.actual[axis]),
      mask(gimbal.limited),
      mask(gimbal.rateLimited),
      gimbal.trackingErrorRad,
    ],
    navigation: {
      ...navigation,
      waypointCount: waypoints?.length ?? navigation.waypointCount ?? 0,
    },
  };
}
const SENSOR_NAMES = [
  "gps",
  "barometer",
  "magnetometer",
  "rangefinder",
  "lidar",
  "opticalFlow",
];
export function validateSensorFrame(value, sensorConfig = {}) {
  object(value);
  const config = validateSensorConfig(sensorConfig),
    time = number(value.time, 0, 1800.001);
  const result = { time },
    faults = object(value.faults);
  for (const name of SENSOR_NAMES) {
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
  result.faults = Object.fromEntries(
    SENSOR_NAMES.map((name) => [name, boolean(faults[name])]),
  );
  return result;
}

export function validateRecording(value) {
  object(value);
  if (value.version !== RECORDING_VERSION)
    throw new Error(
      "Unsupported recording version: version 9 required; legacy recordings are not supported",
    );
  if (
    value.format !== "flight-lab" ||
    value.modelVersion !== MODEL_VERSION ||
    typeof value.createdAt !== "string" ||
    !Number.isFinite(Date.parse(value.createdAt))
  )
    throw new Error("Invalid recording header");
  if (!["hover", "step", "gust"].includes(value.scenario) || value.dt !== 0.005)
    throw new Error("Invalid recording settings");
  object(object(value.setup).gimbal);
  const config = validateConfig(object(value.config)),
    setup = validateSetup(object(value.setup), config),
    runtime = object(value.runtime);
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
    value.samples.length > 36002
  )
    throw new Error("Invalid recording length");
  if (
    !Array.isArray(value.sensorFrames) ||
    value.sensorFrames.length !== value.samples.length
  )
    throw new Error("Sensor frames must match sample count");
  const sim = createSimulation(config);
  let lastEvent = -1;
  const events = Array.from(value.events, (input) => {
    const result = event(input, sim);
    if (result.time < lastEvent) throw new Error("Events out of order");
    lastEvent = result.time;
    return result;
  });
  let lastTime = -1,
    eventIndex = 0,
    activeScene = setup.scene,
    gimbalSettings = setup.gimbal,
    route = setup.mission,
    routeText = JSON.stringify(route);
  const routeCache = new WeakMap();
  const samples = Array.from(value.samples, (input) => {
    object(input);
    const time = number(input.time, 0, 1800.001);
    if (time <= lastTime) throw new Error("Samples out of order");
    while (
      eventIndex < events.length &&
      events[eventIndex].time <= time + 1e-8
    ) {
      if (events[eventIndex].type === "scene")
        activeScene = events[eventIndex].value;
      if (events[eventIndex].type === "gimbal")
        gimbalSettings = events[eventIndex].value;
      if (events[eventIndex].type === "mission") {
        route = events[eventIndex].value;
        routeText = JSON.stringify(route);
      }
      eventIndex++;
    }
    const result = sample(
      input,
      route,
      routeText,
      routeCache,
      config.airframe,
      activeScene,
      gimbalSettings,
    );
    lastTime = time;
    return result;
  });
  let sensorConfig = setup.sensors,
    sensorEventIndex = 0,
    previousChannelTimes = {};
  const sensorFrames = Array.from(value.sensorFrames, (input, index) => {
    while (
      sensorEventIndex < events.length &&
      events[sensorEventIndex].time <= samples[index].time + 1e-8
    ) {
      if (events[sensorEventIndex].type === "sensorConfig") {
        sensorConfig = events[sensorEventIndex].value;
        // Recreating a sensor suite starts a new acquisition epoch; startup may legitimately be time zero.
        previousChannelTimes = {};
      }
      sensorEventIndex++;
    }
    const result = validateSensorFrame(input, sensorConfig);
    if (Math.abs(result.time - samples[index].time) > 1e-8)
      throw new Error("Sensor frame time must match sample time");
    for (const name of SENSOR_NAMES) {
      if (result[name].time < (previousChannelTimes[name] ?? 0) - 1e-8)
        throw new Error("Sensor acquisition time moved backwards");
      previousChannelTimes[name] = result[name].time;
    }
    return result;
  });
  if (lastEvent > lastTime + 1e-8)
    throw new Error("Event occurs after the last sample");
  return {
    format: "flight-lab",
    version: RECORDING_VERSION,
    modelVersion: MODEL_VERSION,
    createdAt: value.createdAt,
    config,
    scenario: value.scenario,
    dt: 0.005,
    setup,
    runtime: cleanRuntime,
    events,
    samples,
    sensorFrames,
  };
}
export function recordingToJson(record) {
  const clean = validateRecording(record);
  return JSON.stringify({
    ...clean,
    samples: clean.samples.map(compactSnapshot),
  });
}
export function recordingToCsv(record) {
  const { samples, sensorFrames } = validateRecording(record);
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
    ...SENSOR_NAMES.flatMap((name) => [
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
  ];
  const rows = samples.map((s, index) =>
    [
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
      ...SENSOR_NAMES.flatMap((name) => [
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
