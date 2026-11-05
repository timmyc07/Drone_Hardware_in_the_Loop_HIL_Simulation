import { validateCollisionPreventionSettings, collisionPreventionInactive, constrainHorizontalMotion } from "./collision-prevention.js";
import { validateGeofenceZones, geofenceContains, geofenceSegmentAllowed } from "./geofence-zones.js";
import { batterySupply } from "./battery.js";
import { createNistLaneCollider } from "./nist-geometry.js";
import {
  validateObstacles,
  obstacleBounds,
  rayAabb,
} from "./obstacle-geometry.js";
import { getVehicleEnvelope } from "./peer-geometry.js";
import {
  createTerrain,
  TerrainCoverageError,
  RUNWAY_FLAT_TOLERANCE_M,
} from "./terrain.js";
import { getRotorLayout, rotorCount } from "./rotor-geometry.js";
import { createGimbal, validateGimbalSettings } from "./gimbal.js";
import {
  airflowAngles,
  aerodynamicForces,
  wingTrim,
  stallSpeed,
  WING,
} from "./aerodynamics.js";
// SI units; NED world, FRD body; scalar-first quaternion rotates body into world.
// Equations/conventions: https://github.com/bobzwik/Quadcopter_SimCon
export const DEFAULT_CONFIG = Object.freeze({
  airframe: "quadrotor",
  startOnGround: false,
  wingArea: 0.4,
  wingSpan: 1.6,
  wingChord: 0.25,
  maxPusherThrust: 8,
  servoTau: 0.06,
  cruiseAirspeed: 15,
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
export const RUNWAY = Object.freeze({
  startNorth: 0,
  endNorth: 200,
  halfWidth: 15,
  headingRad: 0,
});
export const AIRFRAME_PRESETS = Object.freeze({
  quadrotor: DEFAULT_CONFIG,
  fixedwing: Object.freeze({
    ...DEFAULT_CONFIG,
    airframe: "fixedwing",
    geofenceRadius: 500,
  }),
  vtol: Object.freeze({
    ...DEFAULT_CONFIG,
    airframe: "vtol",
    geofenceRadius: 300,
  }),
  hexarotor: Object.freeze({
    ...DEFAULT_CONFIG,
    airframe: "hexarotor",
    mass: 1.8,
    arm: 0.3,
    rateKp: 0.3,
    rateKi: 0.05,
  }),
  octorotor: Object.freeze({
    ...DEFAULT_CONFIG,
    airframe: "octorotor",
    mass: 2.4,
    arm: 0.35,
    rateKp: 0.56,
    rateKi: 0.09,
  }),
});
const bounds = {
  wingArea: [0.1, 3],
  wingSpan: [0.5, 5],
  wingChord: [0.1, 1],
  maxPusherThrust: [0.5, 80],
  servoTau: [0.02, 0.5],
  cruiseAirspeed: [8, 35],
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
  if (
    !config ||
    typeof config !== "object" ||
    Array.isArray(config) ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(config))
  )
    throw new Error("Configuration must be an object");
  for (const [key, value] of Object.entries(config)) {
    if (key === "startOnGround") {
      if (typeof value !== "boolean")
        throw new Error("startOnGround must be a boolean");
      continue;
    }
    if (key === "airframe") {
      if (typeof value !== "string" || !Object.hasOwn(AIRFRAME_PRESETS, value))
        throw new Error("Unknown airframe");
      continue;
    }
    if (!Object.hasOwn(bounds, key))
      throw new Error(`Unknown configuration: ${key}`);
    const [min, max] = bounds[key];
    if (!Number.isFinite(value) || value < min || value > max)
      throw new Error(`${key} must be finite and between ${min} and ${max}`);
  }
  const result = {
    ...AIRFRAME_PRESETS[config.airframe ?? "quadrotor"],
    ...config,
  };
  if (!Number.isInteger(result.seed))
    throw new Error("seed must be an integer");
  if (
    result.airframe !== "fixedwing" &&
    result.maxMotorThrust * rotorCount(result.airframe) <= result.mass * 9.80665
  )
    throw new Error("maxMotorThrust must provide more than vehicle weight");
  if (!Number.isInteger(result.batteryCells))
    throw new Error("batteryCells must be an integer");
  if (result.criticalBatteryThreshold >= result.lowBatteryThreshold)
    throw new Error(
      "criticalBatteryThreshold must be below lowBatteryThreshold",
    );
  if (["fixedwing", "vtol"].includes(result.airframe)) wingTrim(result);
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
export { rotation as bodyToNedMatrix };

function angles(q) {
  const r = rotation(q);
  return [
    Math.atan2(r[7], r[8]),
    Math.asin(clamp(-r[6], -1, 1)),
    Math.atan2(r[3], r[0]),
  ];
}
export { angles as bodyToNedAngles };
export function createSimulation(configuration = {}, options = {}) {
  const config = validateConfig(configuration);
  const collisionRadius = getVehicleEnvelope(config).radius;
  if (
    !options ||
    Object.getPrototypeOf(options) !== Object.prototype ||
    Object.keys(options).some(
      (key) =>
        !["controlSource", "gimbal", "terrain", "spawn", "home", "collisionPrevention", "geofenceZones"].includes(key),
    )
  )
    throw new Error(
      "Simulation options must contain only controlSource, gimbal, terrain, spawn, home, collisionPrevention and geofenceZones",
    );
  const controlSource = options.controlSource ?? "internal";
  if (!["internal", "external"].includes(controlSource))
    throw new Error("controlSource must be internal or external");
  const external = controlSource === "external";
  if (external && Object.hasOwn(options, "gimbal"))
    throw new Error(
      "External control supports only the default fixed camera mount",
    );
  if (
    external &&
    (Object.hasOwn(options, "spawn") || Object.hasOwn(options, "home"))
  )
    throw new Error(
      "External control supports only the default spawn and home",
    );
  const worldCoordinates =
    Object.hasOwn(options, "spawn") || Object.hasOwn(options, "home");
  const initialHome = options.home === undefined ? [0, 0] : options.home;
  if (
    !Array.isArray(initialHome) ||
    initialHome.length !== 2 ||
    ![0, 1].every(
      (i) =>
        Object.hasOwn(initialHome, i) &&
        Number.isFinite(initialHome[i]) &&
        Math.abs(initialHome[i]) <= 500,
    )
  )
    throw new Error("Home must be a finite N/E pair within 500m");
  const home = [...initialHome];
  const initialSpawn =
    options.spawn === undefined ? undefined : structuredClone(options.spawn);
  if (
    initialSpawn !== undefined &&
    (!initialSpawn ||
      ![Object.prototype, null].includes(Object.getPrototypeOf(initialSpawn)) ||
      Reflect.ownKeys(initialSpawn).sort().join(",") !==
        "altitude,east,heading,north" ||
      !Object.values(initialSpawn).every(Number.isFinite) ||
      Math.abs(initialSpawn.north) > 500 ||
      Math.abs(initialSpawn.east) > 500 ||
      initialSpawn.altitude < 0 ||
      initialSpawn.altitude > 500 ||
      Math.abs(initialSpawn.heading) > Math.PI ||
      (config.startOnGround
        ? initialSpawn.altitude !== 0
        : initialSpawn.altitude <= 0))
  )
    throw new Error(
      "Invalid physical spawn; altitude is AGL and must match ground/air start",
    );
  const initialCollisionPrevention = validateCollisionPreventionSettings(options.collisionPrevention);
  const initialGeofenceZones = validateGeofenceZones(options.geofenceZones);
  if (external && initialGeofenceZones.enabled) throw Error("External control does not support enabled geofence zones");
  let geofenceZones = structuredClone(initialGeofenceZones);
  if (initialCollisionPrevention.enabled && (config.airframe === "fixedwing" || external)) throw Error("Collision prevention requires an internal multirotor or hovering VTOL");
  let collisionPreventionSettings = initialCollisionPrevention, collisionPreventionState = null, collisionPreventionPeers = [];
  const initialGimbal = validateGimbalSettings(options.gimbal);
  const initialTerrain = createTerrain(options.terrain).data;
  if (external && initialTerrain.kind !== "flat")
    throw new Error("External control supports only flat terrain");
  let terrain = createTerrain(initialTerrain),
    gimbal,
    altitudeReference;
  const flatGround = () => terrain.data.kind === "flat";
  const groundAt = (s = state) =>
    flatGround()
      ? { heightM: 0, normalNed: [0, 0, -1] }
      : terrain.sample(s[0], s[1]);
  const groundDown = (s = state) => (flatGround() ? 0 : -groundAt(s).heightM);
  const homeHeight = () =>
    flatGround() ? 0 : terrain.sample(home[0], home[1]).heightM;
  const ceiling = () => homeHeight() + config.geofenceHeight;
  if (external && (config.airframe !== "quadrotor" || !config.startOnGround))
    throw new Error(
      "External control requires quadrotor and startOnGround=true",
    );
  let externalInput = null,
    externalPending = false,
    externalTimeUs = 0;
  const requireInternal = () => {
    if (external)
      throw new Error(
        "Internal flight commands are disabled under external control",
      );
  };
  const lever = config.arm / Math.SQRT2;
  const hasWing = ["fixedwing", "vtol"].includes(config.airframe);
  const count = rotorCount(config.airframe),
    motorEnd = 13 + count,
    pusherIndex = motorEnd,
    servoIndex = motorEnd + 1,
    stateSize = motorEnd + 4;
  const layout = getRotorLayout(config.airframe, config.arm);
  const fixedWing = config.airframe === "fixedwing";
  if (fixedWing && config.startOnGround && !terrain.runwayCompatible)
    throw new Error(
      "Fixed-wing ground start requires covered flat runway terrain; use an air start",
    );
  const inertia = hasWing
    ? [
        config.mass * (config.wingSpan ** 2 / 48 + 0.015),
        config.mass * ((4 * config.wingChord) ** 2 / 18 + 0.01),
        0,
      ]
    : [0.5, 0.5, 1].map((v) => v * config.mass * config.arm ** 2);
  if (hasWing) inertia[2] = inertia[0] + inertia[1];
  // Motors: front-right, rear-right, rear-left, front-left. Positive torque signs.
  const rollSign = layout.map((rotor) => rotor.rollScale),
    pitchSign = layout.map((rotor) => rotor.pitchScale),
    yawSign = layout.map((rotor) => rotor.yawTorqueSign);
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
    nistCollider,
    obstacles,
    usedWh,
    battery,
    safetyReason,
    turbulenceState,
    turbulenceGoal,
    nextTurbulence;
  let pusherCommand,
    surfaceCommands,
    speedIntegral,
    transition,
    transitionPhase,
    transitionProgress;
  let landing, groundRolling, missionLegStart, missionRejoin;
  let takeoffPhase,
    takeoffAbortReason,
    takeoffStarted,
    runwayBrake,
    launchAuthorized;
  const rotate = (r, v) =>
    [0, 1, 2].map(
      (i) => r[3 * i] * v[0] + r[3 * i + 1] * v[1] + r[3 * i + 2] * v[2],
    );
  const inverseRotate = (r, v) =>
    [0, 1, 2].map((i) => r[i] * v[0] + r[3 + i] * v[1] + r[6 + i] * v[2]);
  const wrap = (value) => Math.atan2(Math.sin(value), Math.cos(value));
  function aero(s = state) {
    if (!hasWing) {
      const bodyVelocity = inverseRotate(
        rotation(s.slice(6, 10)),
        s.slice(3, 6).map((v, i) => v - wind[i]),
      );
      const { airspeedMps, alphaRad, betaRad } = airflowAngles(bodyVelocity);
      return {
        forceBody: [0, 0, 0],
        momentBody: [0, 0, 0],
        airspeedMps,
        alphaRad,
        betaRad,
        dynamicPressurePa: 0.6125 * Math.exp(s[2] / 8500) * airspeedMps ** 2,
        liftN: 0,
        dragN: 0,
        loadFactor: 0,
        stalled: false,
        pHat: 0,
        qHat: 0,
        rHat: 0,
      };
    }
    return aerodynamicForces(
      config,
      inverseRotate(
        rotation(s.slice(6, 10)),
        s.slice(3, 6).map((v, i) => v - wind[i]),
      ),
      s.slice(10, 13),
      s.slice(servoIndex, stateSize),
      -s[2],
    );
  }
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
    geofenceZones = structuredClone(initialGeofenceZones);
    collisionPreventionSettings = { ...initialCollisionPrevention };
    collisionPreventionState = collisionPreventionInactive(collisionPreventionSettings);
    collisionPreventionPeers = [];
    terrain = createTerrain(initialTerrain);
    altitudeReference = "datum";
    externalInput = null;
    externalPending = false;
    externalTimeUs = 0;
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
      ...Array(count).fill(
        Math.sqrt((config.mass * G) / (count * config.maxMotorThrust)),
      ),
    ];
    state.push(0, 0, 0, 0);
    pusherCommand = 0;
    surfaceCommands = [0, 0, 0];
    speedIntegral = 0;
    transition = fixedWing ? "cruise" : "hover";
    transitionPhase = transition;
    transitionProgress = fixedWing ? 1 : 0;
    landing = null;
    groundRolling = false;
    takeoffPhase = "idle";
    takeoffAbortReason = "";
    takeoffStarted = 0;
    runwayBrake = fixedWing ? 1 : 0;
    launchAuthorized = !config.startOnGround;
    missionLegStart = [0, 0];
    missionRejoin = false;
    target = {
      altitude: fixedWing ? 30 : 2,
      roll: 0,
      pitch: 0,
      yawRate: 0,
      airspeed: hasWing ? config.cruiseAirspeed : 0,
    };
    baseTarget = { ...target };
    scenario = "hover";
    scenarioStart = 0;
    commands = state.slice(13, motorEnd);
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
      home: [...home],
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
    faults = {
      motorEfficiency: Array(count).fill(1),
      linkLost: false,
      pusherEfficiency: 1,
    };
    obstacles = [];
    nistCollider = null;
    usedWh = 0;
    safetyReason = "";
    turbulenceState = [0, 0, 0];
    turbulenceGoal = [0, 0, 0];
    nextTurbulence = 0;
    gustUntil = 0;
    gustVector = [0, 0, 0];
    wind = currentWind();
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
    if (fixedWing && !config.startOnGround) {
      const trim = wingTrim(config);
      state[2] = -30;
      state[3] = config.cruiseAirspeed + wind[0];
      state[4] = wind[1];
      state[5] = wind[2];
      state[6] = Math.cos(trim.alpha / 2);
      state[8] = Math.sin(trim.alpha / 2);
      for (let i = 13; i < motorEnd; i++) state[i] = 0;
      state[pusherIndex] = Math.sqrt(trim.thrust / config.maxPusherThrust);
      state[servoIndex + 1] = trim.elevator;
      commands.fill(0);
      pusherCommand = state[pusherIndex];
      surfaceCommands[1] = trim.elevator;
      observed.altitude = 30;
      observed.attitude[1] = trim.alpha;
      positionTarget.altitude = 30;
    }
    if (config.startOnGround) {
      state[2] = groundDown();
      state[3] = 0;
      state[4] = 0;
      state[5] = 0;
      state[6] = 1;
      state[7] = 0;
      state[8] = 0;
      state[9] = 0;
      for (let i = 10; i < stateSize; i++) state[i] = 0;
      armed = false;
      commands.fill(0);
      pusherCommand = 0;
      surfaceCommands = [0, 0, 0];
      observed.altitude = -state[2];
      observed.attitude = [0, 0, 0];
      groundRolling = fixedWing;
      target = {
        altitude: groundAt().heightM,
        roll: 0,
        pitch: 0,
        yawRate: 0,
        airspeed: 0,
      };
      baseTarget = { ...target };
      positionTarget.altitude = -state[2];
    }
    if (terrain.data.kind === "dem") {
      const homeHeight = groundAt().heightM;
      if (!config.startOnGround) {
        state[2] -= homeHeight;
        target.altitude += homeHeight;
        baseTarget.altitude = target.altitude;
        positionTarget.altitude = target.altitude;
        observed.altitude = -state[2];
      }
    }
    if (initialSpawn !== undefined) {
      const { north, east, altitude, heading } = initialSpawn;
      state[0] = north;
      state[1] = east;
      const height = groundAt().heightM + altitude;
      const pitch =
        fixedWing && !config.startOnGround ? wingTrim(config).alpha : 0;
      const cy = Math.cos(heading / 2),
        sy = Math.sin(heading / 2),
        cp = Math.cos(pitch / 2),
        sp = Math.sin(pitch / 2);
      state.splice(
        6,
        4,
        ...[cy * cp, -sy * sp, cy * sp, sy * cp].map((v) => (v === 0 ? 0 : v)),
      );
      state[2] = -height;
      if (fixedWing && !config.startOnGround) {
        state[3] = config.cruiseAirspeed * Math.cos(heading) + wind[0];
        state[4] = config.cruiseAirspeed * Math.sin(heading) + wind[1];
      }
      target.altitude = height;
      baseTarget = { ...target };
      positionTarget = { north, east, altitude: height };
      navigation.targetPosition = [north, east];
      missionLegStart = [north, east];
      observed.position = [north, east];
      observed.velocity = state.slice(3, 5);
      observed.altitude = height;
      observed.attitude = [0, pitch, heading];
    }
    homeHeight();
    observed.airspeed = aero().airspeedMps;
    gimbal = createGimbal(initialGimbal, state.slice(6, 10));
    // Preserve a feasible physical hover/wing trim after voltage derating.
    const desired = state.slice(13, servoIndex), maximum = Math.max(...desired);
    if (maximum > 0) {
      let low = 1, high = 1 / maximum;
      for (let i = 0; i < 40; i++) {
        const middle = (low + high) / 2;
        const supply = batterySupply(config, config.batteryInitialSoc, desired.map(v => v * middle));
        if (middle * supply.motorSpeedScale < 1) low = middle;
        else high = middle;
      }
      state.splice(13, desired.length, ...desired.map(v => Math.min(1, v * high)));
      commands = state.slice(13, motorEnd);
      pusherCommand = state[pusherIndex];
    }
    battery = batterySnapshot();
    if (geofenceZones.enabled) {
      if (!geofenceContains(geofenceZones, home)) throw Error("Home is outside the geofence zones");
      if (!positionInsideGeofence(state, geofenceZones)) throw Error("Initial spawn is outside the geofence");
    }
    return snapshot();
  }
  function derivative(s) {
    if (
      crashed ||
      ((!armed || !launchAuthorized) &&
        s[2] >= groundDown(s) - 0.001 &&
        (!fixedWing || Math.hypot(s[3], s[4]) < 0.03))
    )
      return Array(stateSize).fill(0);
    const r = rotation(s.slice(6, 10));
    // States/commands are lagged full-voltage speed demands; supply sets actual speed once.
    const thrustScale = supply(s).motorSpeedScale ** 2;
    const thrusts = s
      .slice(13, motorEnd)
      .map(
        (v, i) =>
          clamp(v, 0, 1) ** 2 *
          config.maxMotorThrust *
          thrustScale *
          faults.motorEfficiency[i],
      );
    const thrust = thrusts.reduce((a, b) => a + b, 0);
    const wing = aero(s);
    const pusherThrust = hasWing
      ? config.maxPusherThrust *
        clamp(s[pusherIndex], 0, 1) ** 2 *
        thrustScale *
        faults.pusherEfficiency
      : 0;
    const forceBody = [...wing.forceBody];
    forceBody[0] += pusherThrust;
    forceBody[2] -= thrust;
    const forceWorld = rotate(r, forceBody);
    const acceleration = [0, 1, 2].map(
      (i) =>
        forceWorld[i] / config.mass +
        (i === 2 ? G : 0) -
        (hasWing ? 0 : (config.drag * (s[i + 3] - wind[i])) / config.mass),
    );
    const torque = [rollSign, pitchSign, yawSign].map(
      (signs, axis) =>
        thrusts.reduce((sum, f, i) => sum + f * signs[i], 0) *
          (axis === 2 ? yawLever : lever) +
        wing.momentBody[axis],
    );
    if (fixedWing && groundRolling) {
      const [roll, pitch, yaw] = angles(s.slice(6, 10));
      const normal = Math.max(0, config.mass * acceleration[2]);
      const forward = [Math.cos(yaw), Math.sin(yaw)],
        lateral = [-Math.sin(yaw), Math.cos(yaw)];
      const longitudinal = s[3] * forward[0] + s[4] * forward[1],
        sideways = s[3] * lateral[0] + s[4] * lateral[1];
      const engineForce =
        forceWorld[0] * forward[0] + forceWorld[1] * forward[1];
      const rollingCoefficient = 0.025 + 0.35 * runwayBrake;
      const friction =
        Math.abs(longitudinal) < (runwayBrake > 0.5 ? 0.2 : 0.03)
          ? -clamp(
              engineForce + config.mass * 20 * longitudinal,
              -normal * (0.04 + 0.76 * runwayBrake),
              normal * (0.04 + 0.76 * runwayBrake),
            )
          : -Math.sign(longitudinal) * rollingCoefficient * normal;
      const lateralForce =
        forceWorld[0] * lateral[0] + forceWorld[1] * lateral[1];
      const sideFriction =
        Math.abs(sideways) < 0.2 && runwayBrake > 0.5
          ? -clamp(
              lateralForce + config.mass * 20 * sideways,
              -0.7 * normal,
              0.7 * normal,
            )
          : clamp(-config.mass * 10 * sideways, -0.7 * normal, 0.7 * normal);
      for (let i = 0; i < 2; i++)
        acceleration[i] +=
          (friction * forward[i] + sideFriction * lateral[i]) / config.mass;
      // Rigid unilateral wheel support: the normal reaction cannot pull the aircraft down.
      acceleration[2] -= normal / config.mass;
      // Educational runway restraint stays engaged until the rotation gate authorizes liftoff.
      if (takeoffPhase !== "rotate") acceleration[2] = 0;
      torque[0] += clamp(
        -torque[0] - inertia[0] * (120 * roll + 20 * s[10]),
        -normal * 0.4,
        normal * 0.4,
      );
      if (takeoffPhase !== "rotate")
        torque[1] += clamp(
          -torque[1] - inertia[1] * (120 * pitch + 20 * s[11]),
          -normal * 0.6,
          normal * 0.6,
        );
      else if (pitch > 0.2 || pitch < -0.03)
        torque[1] +=
          inertia[1] * (120 * (clamp(pitch, -0.03, 0.2) - pitch) - 20 * s[11]);
      const steering = clamp(
        -0.7 * wrap(yaw) - 0.07 * s[1] - 0.15 * s[12],
        -0.35,
        0.35,
      );
      const desiredYawRate = (longitudinal * Math.tan(steering)) / 0.6;
      const steeringMoment =
        Math.abs(longitudinal) < 0.2
          ? -torque[2] - inertia[2] * 20 * s[12]
          : inertia[2] * 8 * (desiredYawRate - s[12]);
      torque[2] += clamp(steeringMoment, -normal * 0.5, normal * 0.5);
    }
    if (!flatGround() && !fixedWing && s[2] >= groundDown(s) - 1e-8) {
      const normal = groundAt(s).normalNed;
      const intoGround = acceleration.reduce(
        (sum, v, i) => sum + v * normal[i],
        0,
      );
      const leaving =
        s.slice(3, 6).reduce((sum, v, i) => sum + v * normal[i], 0) > 1e-9;
      if (intoGround < 0 && !leaving) {
        for (let i = 0; i < 3; i++) acceleration[i] -= intoGround * normal[i];
        // Simplified Coulomb contact, mu=.8; an educational assumption, not measured landing-gear friction.
        const normalLoad = -intoGround;
        const normalSpeed = s
          .slice(3, 6)
          .reduce((sum, v, i) => sum + v * normal[i], 0);
        const friction = acceleration.map(
          (v, i) => -v - 20 * (s[3 + i] - normalSpeed * normal[i]),
        );
        const scale = Math.min(
          1,
          (0.8 * normalLoad) / Math.max(1e-12, Math.hypot(...friction)),
        );
        for (let i = 0; i < 3; i++) acceleration[i] += friction[i] * scale;
      }
    }
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
      (pusherCommand - s[pusherIndex]) / config.motorTau,
      ...surfaceCommands.map(
        (v, i) => (v - s[servoIndex + i]) / config.servoTau,
      ),
    ];
  }
  function sample() {
    const d = derivative(state),
      r = rotation(state.slice(6, 10));
    const specific = d.slice(3, 6);
    specific[2] -= G;
    if (!armed && state[2] >= groundDown() - 0.001)
      specific.splice(0, 3, 0, 0, -G);
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
      airspeed: hasWing ? aero().airspeedMps + noise(0.05) : 0,
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
      if (
        fixedWing &&
        ["runup", "roll", "rotate", "climb"].includes(takeoffPhase)
      ) {
        stopTakeoff("safety");
        navigation.reason = "stale-sensor";
      }
    }
    if (
      failed ||
      !armed ||
      crashed ||
      battery.soc <= 0 ||
      (!launchAuthorized && state[2] >= groundDown() - 0.001)
    ) {
      commands.fill(0);
      pusherCommand = 0;
      surfaceCommands = [0, 0, 0];
      integrals.fill(0);
      return;
    }
    if (groundRolling && !["runup", "roll", "rotate"].includes(takeoffPhase)) {
      commands.fill(0);
      pusherCommand = 0;
      surfaceCommands = [0, 0, 0];
      return;
    }
    updateTransition(dt);
    if (hasWing) wingControl(dt);
    if (fixedWing) {
      commands.fill(0);
      return;
    }
    const [phi, theta] = observed.attitude;
    // Convert Euler angle demands and heading rate to body rates near the commanded attitude.
    const desiredRates = [
      config.attitudeKp * (target.roll - phi) -
        Math.sin(theta) * target.yawRate,
      config.attitudeKp *
        ((hasWing ? wingPitchDemand() : target.pitch) - theta) +
        Math.sin(phi) * Math.cos(theta) * target.yawRate,
      Math.cos(phi) * Math.cos(theta) * target.yawRate,
    ];
    const errors = desiredRates.map(
      (v, i) => clamp(v, -4, 4) - observed.rates[i],
    );
    const candidate = integrals.map((v, i) => clamp(v + errors[i] * dt, -2, 2));
    const torque = errors.map(
      (v, i) =>
        (config.rateKp * v + config.rateKi * candidate[i]) *
        (hasWing
          ? inertia[i] / ([0.5, 0.5, 1][i] * config.mass * config.arm ** 2)
          : 1) *
        (1 - transitionProgress),
    );
    const upAcceleration = clamp(
      config.altitudeKp * (target.altitude - observed.altitude) +
        config.altitudeKd * observed.downSpeed,
      -G,
      G,
    );
    const wingUp = hasWing
      ? -rotate(rotation(state.slice(6, 10)), aero().forceBody)[2] +
        config.maxPusherThrust *
          state[pusherIndex] ** 2 *
          supply().motorSpeedScale ** 2 *
          faults.pusherEfficiency *
          Math.sin(theta)
      : 0;
    const total =
      Math.max(0, config.mass * (G + upAcceleration) - wingUp) /
      Math.max(0.3, Math.cos(phi) * Math.cos(theta));
    const availableMotorThrust = config.maxMotorThrust * Math.max(1e-12, supply().motorSpeedScale ** 2);
    const raw = rollSign.map(
      (_, i) =>
        (total +
          (rollSign[i] * torque[0]) / lever +
          (pitchSign[i] * torque[1]) / lever +
          (yawSign[i] * torque[2]) / yawLever) /
        (count * availableMotorThrust),
    );
    commands = raw.map((v) => Math.sqrt(clamp(v, 0, 1)));
    // Conditional integration: freeze the integral while any motor is saturated.
    if (raw.every((v) => v >= 0 && v <= 1)) integrals = candidate;
  }
  function flightSnapshot() {
    const wing = aero();
    return {
      airspeedMps: wing.airspeedMps,
      groundspeedMps: Math.hypot(state[3], state[4]),
      alphaRad: wing.alphaRad,
      betaRad: wing.betaRad,
      dynamicPressurePa: wing.dynamicPressurePa,
      liftN: wing.liftN,
      dragN: wing.dragN,
      loadFactor: wing.loadFactor,
      stalled: wing.stalled,
      transition,
      transitionPhase,
      transitionProgress,
    };
  }
  function setTransition(value) {
    if (value === "cruise" && collisionPreventionSettings.enabled) throw Error("Disable collision prevention before VTOL cruise transition");
    if (config.airframe !== "vtol")
      throw new Error("Transition is only available for lift+cruise VTOL");
    if (!["hover", "cruise"].includes(value))
      throw new Error("Unknown VTOL transition");
    if (crashed || failed || !armed || safetyReason)
      throw new Error(
        "Cannot transition while disarmed or during a safety action",
      );
    if (value === transition) return;
    if (value === "cruise" && altitudeReference === "agl")
      throw new Error("Select datum altitude before cruise transition");
    if (value === "cruise" && -state[2] < 8)
      throw new Error("Climb to at least 8 meters before cruise transition");
    if (value === "cruise")
      positionTarget.altitude = Math.max(
        -state[2],
        baseTarget.altitude,
        positionTarget.altitude,
      );
    transition = value;
    transitionPhase = value === "cruise" ? "accelerating" : "decelerating";
    speedIntegral = 0;
    if (value === "hover") {
      positionTarget = {
        north: state[0],
        east: state[1],
        altitude: Math.max(8, -state[2]),
      };
      enterMode("position");
    }
  }
  function updateTransition(dt) {
    if (config.airframe !== "vtol") return;
    const wing = aero(),
      safeSpeed = 1.3 * stallSpeed(config, -state[2]);
    if (transition === "cruise") {
      const available = Math.min(
        clamp((wing.airspeedMps - safeSpeed * 0.65) / (safeSpeed * 0.35), 0, 1),
        clamp(wing.liftN / (config.mass * G), 0, 1),
      );
      transitionProgress += clamp(
        available - transitionProgress,
        -dt / 2,
        dt / 3,
      );
      transitionPhase =
        transitionProgress > 0.97 && wing.airspeedMps >= safeSpeed
          ? "cruise"
          : "accelerating";
      if (transitionPhase === "cruise") transitionProgress = 1;
    } else {
      if (transitionPhase === "hover") return;
      transitionProgress = Math.max(0, transitionProgress - dt / 3);
      transitionPhase =
        transitionProgress < 0.01 && wing.airspeedMps < 2
          ? "hover"
          : "decelerating";
      if (transitionPhase === "hover") transitionProgress = 0;
    }
  }
  function wingPitchDemand() {
    if (fixedWing && navigation.mode === "takeoff") {
      if (["runup", "roll"].includes(takeoffPhase)) return 0;
      if (takeoffPhase === "rotate") return 0.14;
      const safe = 1.4 * stallSpeed(config, observed.altitude);
      const climb =
        clamp(
          (observed.airspeed - 1.15 * stallSpeed(config, observed.altitude)) /
            (safe - 1.15 * stallSpeed(config, observed.altitude)),
          0,
          1,
        ) * 0.12;
      const pressure =
        0.5 *
        1.225 *
        Math.exp(-observed.altitude / 8500) *
        Math.max(3, observed.airspeed) ** 2;
      const alpha = clamp(
        ((config.mass * G) / (pressure * config.wingArea) - WING.cl0) /
          WING.clAlpha,
        0,
        0.2,
      );
      return clamp(alpha + climb, 0, 0.3);
    }
    if (!fixedWing && transition === "hover") return target.pitch;
    const speed = Math.max(3, observed.airspeed),
      height = observed.altitude;
    const density = 1.225 * Math.exp(-height / 8500),
      pressure = 0.5 * density * speed ** 2;
    const liftRequired =
      (config.mass * G) / Math.max(0.7, Math.cos(target.roll));
    const alpha = clamp(
      (liftRequired / (pressure * config.wingArea) - WING.cl0) / WING.clAlpha,
      -0.1,
      0.22,
    );
    let path = clamp(
      0.035 * (target.altitude - height) + 0.12 * observed.downSpeed,
      -0.16,
      0.16,
    );
    if (fixedWing && navigation.mode === "land" && landing?.phase === "final") {
      const sink = height < 2 ? 0.45 : 1.2;
      path = clamp(
        -sink / speed +
          0.035 * (target.altitude - height) +
          0.12 * (observed.downSpeed - sink),
        -0.2,
        0.08,
      );
    }
    return clamp(
      alpha + path + (navigation.mode === "attitude" ? target.pitch : 0),
      -0.4,
      0.45,
    );
  }
  function wingControl(dt) {
    const wing = aero(),
      speed = Math.max(1, wing.airspeedMps),
      [phi, theta] = observed.attitude;
    const [p, q, r] = observed.rates;
    let demandSpeed = target.airspeed;
    if (navigation.mode !== "attitude")
      demandSpeed = Math.max(
        1.25 * stallSpeed(config, observed.altitude),
        demandSpeed,
      );
    const powered = fixedWing || transition === "cruise";
    if (powered) {
      const error = demandSpeed - observed.airspeed;
      const candidate = clamp(speedIntegral + error * dt, -10, 10);
      const thrust = wing.dragN + 1.4 * error + 0.3 * candidate;
      const availableThrust = config.maxPusherThrust * supply().motorSpeedScale ** 2;
      const requestedPusher =
        groundRolling && ["runup", "roll", "rotate"].includes(takeoffPhase)
          ? 1
          : Math.sqrt(clamp(thrust / Math.max(1e-12, availableThrust), 0, 1));
      pusherCommand =
        takeoffPhase === "runup"
          ? Math.min(pusherCommand + dt / 0.8, requestedPusher)
          : requestedPusher;
      if (thrust >= 0 && thrust <= availableThrust)
        speedIntegral = candidate;
    } else {
      pusherCommand = 0;
      speedIntegral = 0;
    }
    const yawRate =
      (G * Math.tan(target.roll)) / Math.max(4, speed) + target.yawRate;
    const desiredRates = [
      -Math.sin(theta) * yawRate,
      Math.sin(phi) * Math.cos(theta) * yawRate,
      Math.cos(phi) * Math.cos(theta) * yawRate,
    ];
    const pitchDemand = wingPitchDemand();
    const requested = [
      inertia[0] * (12 * (target.roll - phi) + 5 * (desiredRates[0] - p)),
      inertia[1] * (12 * (pitchDemand - theta) + 5 * (desiredRates[1] - q)),
      inertia[2] * (4 * (desiredRates[2] - r) + 2 * wing.betaRad),
    ];
    const weight = fixedWing ? 1 : transitionProgress;
    const reference = Math.max(0.01, wing.dynamicPressurePa * config.wingArea);
    const passive = [
      -0.08 * wing.betaRad - 0.5 * wing.pHat + 0.08 * wing.rHat,
      wing.pitchStatic - 8 * wing.qHat,
      0.12 * wing.betaRad - 0.3 * wing.rHat - 0.01 * state[servoIndex],
    ];
    surfaceCommands = requested.map((moment, i) =>
      clamp(
        ((moment * weight) /
          (reference * (i === 1 ? config.wingChord : config.wingSpan)) -
          passive[i]) /
          [WING.rollAileron, WING.cmElevator, WING.yawRudder][i],
        -WING.surfaceLimit,
        WING.surfaceLimit,
      ),
    );
  }
  function landingPlan() {
    const height = clamp(-state[2], 15, 30),
      distance = (height + 6) / 0.15;
    return {
      phase: "entry",
      index: 0,
      height,
      start: state.slice(0, 2),
      rejoin: false,
      points: [
        { north: -distance - 80, east: -80 },
        { north: -distance - 80, east: 0 },
        { north: -distance, east: 0 },
      ],
    };
  }
  function navigateWing(dt) {
    const speed = Math.max(1, observed.airspeed),
      course = Math.atan2(observed.velocity[1], observed.velocity[0]);
    let desiredSpeed = Math.max(
      baseTarget.airspeed,
      1.3 * stallSpeed(config, observed.altitude),
    );
    let radius = Math.max(25, desiredSpeed ** 2 / (G * Math.tan(0.4)));
    const dn = observed.position[0] - positionTarget.north,
      de = observed.position[1] - positionTarget.east;
    const distance = Math.hypot(dn, de);
    let desiredCourse,
      feedForward = 0;
    if (navigation.mode === "mission" && !navigation.completed) {
      const waypoint = navigation.waypoints[navigation.index];
      const leg = [
          positionTarget.north - missionLegStart[0],
          positionTarget.east - missionLegStart[1],
        ],
        length = Math.hypot(...leg);
      const unit =
        length > 0.01
          ? leg.map((v) => v / length)
          : [Math.cos(course), Math.sin(course)];
      const offset = [
        observed.position[0] - missionLegStart[0],
        observed.position[1] - missionLegStart[1],
      ];
      const along = offset[0] * unit[0] + offset[1] * unit[1],
        cross = -offset[0] * unit[1] + offset[1] * unit[0];
      // Slow for short legs instead of demanding a turn radius the wing cannot fly.
      desiredSpeed = Math.max(
        1.3 * stallSpeed(config, observed.altitude),
        Math.min(
          desiredSpeed,
          Math.sqrt(G * Math.tan(0.45) * Math.max(15, length * 0.3)),
        ),
      );
      const captured = distance < Math.max(8, Math.min(12, speed * 0.8));
      const passed =
        along >= length && along - length < 20 && Math.abs(cross) < 12;
      if (
        (captured || passed) &&
        Math.abs(observed.altitude - positionTarget.altitude) < 4 &&
        holdSince === null
      )
        holdSince = time;
      if (holdSince !== null && time - holdSince >= waypoint.hold) {
        if (navigation.index + 1 < navigation.waypoints.length) {
          missionLegStart = [positionTarget.north, positionTarget.east];
          navigation.index++;
          positionTarget = { ...navigation.waypoints[navigation.index] };
          holdSince = null;
          missionRejoin = false;
          desiredCourse = Math.atan2(
            positionTarget.east - observed.position[1],
            positionTarget.north - observed.position[0],
          );
        } else navigation.completed = true;
      } else if (holdSince === null) {
        if (along > length + 20) missionRejoin = true;
        if (missionRejoin && along < length - 30 && Math.abs(cross) < 20)
          missionRejoin = false;
        desiredCourse = missionRejoin
          ? Math.atan2(
              positionTarget.east - unit[1] * 60 - observed.position[1],
              positionTarget.north - unit[0] * 60 - observed.position[0],
            )
          : Math.atan2(unit[1], unit[0]) -
            Math.atan2(cross, Math.max(15, speed * 1.5));
      }
    }
    if (navigation.mode === "land") {
      if (!landing) landing = landingPlan();
      if (landing.phase === "entry") {
        const point = landing.points[landing.index];
        const leg = [
            point.north - landing.start[0],
            point.east - landing.start[1],
          ],
          length = Math.hypot(...leg);
        const unit = length > 0.01 ? leg.map((v) => v / length) : [1, 0];
        const offset = [
          observed.position[0] - landing.start[0],
          observed.position[1] - landing.start[1],
        ];
        const along = offset[0] * unit[0] + offset[1] * unit[1],
          cross = -offset[0] * unit[1] + offset[1] * unit[0];
        const close =
          Math.hypot(
            point.north - observed.position[0],
            point.east - observed.position[1],
          ) < 10 ||
          (along >= length && along - length < 20 && Math.abs(cross) < 10);
        if (close && Math.abs(observed.altitude - landing.height) < 4) {
          if (landing.index < landing.points.length - 1) {
            landing.start = [point.north, point.east];
            landing.index++;
            landing.rejoin = false;
          } else if (
            Math.abs(wrap(course)) < 0.6 &&
            Math.abs(observed.position[1]) < 10
          )
            landing.phase = "final";
        }
        if (along > length + 20) landing.rejoin = true;
        if (landing.rejoin && along < length - 30 && Math.abs(cross) < 20)
          landing.rejoin = false;
        desiredCourse = landing.rejoin
          ? Math.atan2(
              point.east - unit[1] * 60 - observed.position[1],
              point.north - unit[0] * 60 - observed.position[0],
            )
          : Math.atan2(unit[1], unit[0]) -
            Math.atan2(cross, Math.max(15, speed * 1.5));
        positionTarget = { ...point, altitude: landing.height };
        desiredSpeed = Math.max(
          1.4 * stallSpeed(config, observed.altitude),
          Math.min(baseTarget.airspeed, 11),
        );
      }
      if (landing.phase === "final") {
        desiredCourse = -Math.atan2(observed.position[1], 25);
        positionTarget = {
          north: 0,
          east: 0,
          altitude: Math.max(0, -observed.position[0] * 0.15 - 6),
        };
        desiredSpeed = 1.25 * stallSpeed(config, observed.altitude);
      }
    } else if (desiredCourse === undefined) {
      radius = Math.max(25, desiredSpeed ** 2 / (G * Math.tan(0.4)));
      const bearing = distance > 1 ? Math.atan2(de, dn) : course - Math.PI / 2;
      desiredCourse =
        bearing + Math.PI / 2 + Math.atan((2 * (distance - radius)) / radius);
      feedForward = desiredSpeed / radius;
      if (
        navigation.mode === "rtl" &&
        Math.abs(distance - radius) < 10 &&
        Math.abs(observed.altitude - positionTarget.altitude) < 4
      )
        navigation.completed = true;
    }
    const turnRate = clamp(
      0.7 * wrap(desiredCourse - course) + feedForward,
      -0.4,
      0.4,
    );
    target = {
      altitude: positionTarget.altitude,
      roll: clamp(Math.atan((speed * turnRate) / G), -0.5, 0.5),
      pitch: 0,
      yawRate: 0,
      airspeed: desiredSpeed,
    };
    navigation.targetPosition = [positionTarget.north, positionTarget.east];
  }

  function takeoffActive() {
    return (
      fixedWing && ["runup", "roll", "rotate", "climb"].includes(takeoffPhase)
    );
  }
  function setTargets(values) {
    if (takeoffActive())
      throw new Error("Abort runway takeoff before changing targets");
    if (
      !values ||
      typeof values !== "object" ||
      Array.isArray(values) ||
      ![Object.prototype, null].includes(Object.getPrototypeOf(values))
    )
      throw new Error("Targets must be an object");
    const limits = {
      altitude: [
        terrain.data.kind === "dem" || worldCoordinates ? -1000 : 0,
        worldCoordinates ? Math.min(1500, ceiling()) : 500,
      ],
      airspeed: [0, 40],
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
    if (Object.hasOwn(values, "altitude")) {
      altitudeReference = "datum";
      if (navigation.mode === "position")
        positionTarget.altitude = values.altitude;
    }
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
  function validatePoint(point, hold = false, reference = "datum", zones = geofenceZones) {
    const fields = {
      north: [-500, 500],
      east: [-500, 500],
      altitude: [
        (terrain.data.kind === "dem" || worldCoordinates) &&
        reference === "datum"
          ? -1000
          : 0,
        worldCoordinates && reference === "datum"
          ? Math.min(1500, ceiling())
          : 500,
      ],
    };
    if (hold) fields.hold = [0, 300];
    const value = validateFields(point, fields, "Waypoint");
    if (
      Math.hypot(value.north - home[0], value.east - home[1]) >
        config.geofenceRadius ||
      value.altitude > (reference === "agl" ? config.geofenceHeight : ceiling()) ||
      !geofenceContains(zones, [value.north, value.east])
    )
      throw new Error("Waypoint is outside the geofence");
    return value;
  }
  function setPositionTarget(value) {
    if (takeoffActive())
      throw new Error("Abort runway takeoff before changing position target");
    if (safetyReason || ["rtl", "land", "mission"].includes(navigation.mode))
      throw new Error(
        "Select position mode before changing a navigation target; safety return/landing cannot be overridden",
      );
    if (!value || typeof value !== "object" || Array.isArray(value))
      throw new Error("Position target must be an object");
    const reference = value.altitudeReference ?? "datum";
    if (!["datum", "agl"].includes(reference))
      throw new Error("Unknown altitude reference");
    if (
      reference === "agl" &&
      (fixedWing ||
        (hasWing && (transition !== "hover" || transitionProgress !== 0)) ||
        navigation.mode !== "position")
    )
      throw new Error(
        "AGL requires position mode on a multirotor or VTOL in hover",
      );
    const { altitudeReference: ignoredReference, ...point } = value;
    const checked = validatePoint(point, false, reference);
    if (
      reference === "agl" &&
      terrain.sample(checked.north, checked.east).heightM + checked.altitude >
        ceiling()
    )
      throw new Error("AGL target is above geofence height");
    positionTarget = checked;
    altitudeReference = reference;
    explicitPosition = true;
    navigation.targetPosition = [positionTarget.north, positionTarget.east];
  }
  function setMission(points) {
    if (takeoffActive())
      throw new Error("Abort runway takeoff before changing mission");
    if (!Array.isArray(points) || points.length < 1 || points.length > 50)
      throw new Error("Mission must contain 1 to 50 waypoints");
    const validated = Array.from(points, (value) => validatePoint(value, true));
    navigation.waypoints = validated;
    navigation.index = 0;
    navigation.completed = false;
    holdSince = null;
    if (navigation.mode === "mission") {
      positionTarget = { ...validated[0] };
      missionLegStart = state.slice(0, 2);
      missionRejoin = false;
    }
  }
  function enterMode(mode, reason = "") {
    if (fixedWing && mode === "land" && !terrain.runwayCompatible)
      throw new Error("Terrain does not support fixed-wing runway landing");
    if (mode !== "position") altitudeReference = "datum";
    navigation.mode = mode;
    navigation.reason = reason;
    navigation.completed = false;
    horizontalIntegral = [0, 0];
    holdSince = null;
    if (mode === "attitude") {
      baseTarget = {
        altitude: Math.max(groundAt().heightM, -state[2]),
        roll: 0,
        pitch: 0,
        yawRate: 0,
        airspeed: hasWing ? config.cruiseAirspeed : 0,
      };
      target = { ...baseTarget };
    } else if (mode === "position") {
      // Capture current position unless an explicit target was supplied immediately beforehand.
      if (!explicitPosition)
        positionTarget = {
          north: state[0],
          east: state[1],
          altitude:
            altitudeReference === "agl"
              ? Math.max(0, -state[2] - groundAt().heightM)
              : Math.max(groundAt().heightM, -state[2]),
        };
    } else if (mode === "mission") {
      navigation.index = 0;
      missionLegStart = state.slice(0, 2);
      missionRejoin = false;
      positionTarget = { ...navigation.waypoints[0] };
    } else if (mode === "rtl") {
      rtlPhase = "climb";
      positionTarget = {
        north: state[0],
        east: state[1],
        altitude: Math.min(
          ceiling() - 0.25,
          Math.max(homeHeight() + 3, -state[2]),
        ),
      };
    } else if (mode === "land") {
      positionTarget = {
        north: state[0],
        east: state[1],
        altitude: Math.max(groundAt().heightM, -state[2]),
      };
    }
    if (hasWing && (fixedWing || transition === "cruise")) {
      if (mode === "rtl")
        positionTarget = {
          north: home[0],
          east: home[1],
          altitude: Math.max(homeHeight() + 20, -state[2]),
        };
      if (mode === "land") {
        landing = landingPlan();
      }
    }
    if (
      config.airframe === "vtol" &&
      ["rtl", "land"].includes(mode) &&
      (transition === "cruise" || transitionProgress > 0.01)
    ) {
      transition = "hover";
      transitionPhase = "decelerating";
      positionTarget = {
        north: mode === "rtl" ? home[0] : state[0],
        east: mode === "rtl" ? home[1] : state[1],
        altitude: Math.max(8, -state[2]),
      };
    }
    explicitPosition = false;
    navigation.targetPosition = [positionTarget.north, positionTarget.east];
    refreshCollisionPreventionState();
  }
  function setMode(mode) {
    if (
      fixedWing &&
      !terrain.runwayCompatible &&
      ["land", "takeoff"].includes(mode)
    )
      throw new Error(
        "Terrain does not support fixed-wing runway takeoff/landing",
      );
    if (mode === "takeoff") {
      if (!fixedWing || !groundRolling || state[2] < -0.1)
        throw new Error(
          "Takeoff mode requires a fixed-wing aircraft on the runway",
        );
      return takeoff();
    }
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
    if (takeoffActive())
      throw new Error("Abort runway takeoff before changing flight mode");
    enterMode(mode, safetyReason);
  }
  function arm() {
    if (
      fixedWing &&
      takeoffPhase === "aborted" &&
      !groundRolling &&
      navigation.mode === "takeoff"
    )
      throw new Error(
        "Select an airborne flight mode before rearming an aborted takeoff",
      );
    if (crashed || failed || battery.critical || faults.linkLost)
      throw new Error(
        "Cannot arm after crash, failsafe, critical battery or lost link",
      );
    armed = true;
    integrals.fill(0);
  }
  function disarm() {
    if (takeoffActive() && groundRolling) {
      stopTakeoff("pilot");
      return;
    }
    if (takeoffActive()) {
      takeoffPhase = "aborted";
      takeoffAbortReason = "pilot";
      runwayBrake = groundRolling ? 1 : 0;
      navigation.reason = "takeoff-aborted";
    }
    armed = false;
    if (state[2] >= groundDown() - 0.001) launchAuthorized = false;
    commands.fill(0);
    pusherCommand = 0;
    surfaceCommands = [0, 0, 0];
    integrals.fill(0);
  }
  function runwaySnapshot() {
    return {
      onRunway:
        state[0] >= RUNWAY.startNorth - 1e-8 &&
        state[0] <= RUNWAY.endNorth + 1e-8 &&
        Math.abs(state[1]) <= RUNWAY.halfWidth,
      weightOnWheels: fixedWing
        ? groundRolling
        : state[2] >= groundDown() - 0.001,
      distanceRemainingM: Math.max(
        0,
        RUNWAY.endNorth - Math.max(RUNWAY.startNorth, state[0]),
      ),
      crossTrackM: state[1],
      rotationSpeedMps: fixedWing
        ? 1.2 * stallSpeed(config, Math.max(0, -state[2]))
        : 0,
      takeoffPhase,
      abortReason: takeoffAbortReason,
      brake: runwayBrake,
    };
  }
  function stopTakeoff(reason) {
    takeoffPhase = "aborted";
    takeoffAbortReason = reason;
    if (groundRolling) {
      armed = false;
      launchAuthorized = false;
      runwayBrake = 1;
      pusherCommand = 0;
      surfaceCommands = [0, 0, 0];
      commands.fill(0);
      integrals.fill(0);
      navigation.reason = "takeoff-aborted";
      target = {
        altitude: groundAt().heightM,
        roll: 0,
        pitch: 0,
        yawRate: 0,
        airspeed: 0,
      };
      baseTarget = { ...target };
    } else if (reason === "pilot") {
      enterMode("position", "takeoff-aborted");
      positionTarget.altitude = Math.max(30, -state[2]);
    } else {
      enterMode("land", "takeoff-aborted");
      landing = { phase: "final" };
    }
  }
  function abortTakeoff() {
    if (
      !fixedWing ||
      !["runup", "roll", "rotate", "climb"].includes(takeoffPhase)
    )
      throw new Error("No runway takeoff is active");
    stopTakeoff("pilot");
  }
  function navigateTakeoff(dt) {
    const wing = aero(),
      runway = runwaySnapshot();
    const forwardAirspeed = inverseRotate(
      rotation(state.slice(6, 10)),
      state.slice(3, 6).map((v, i) => v - wind[i]),
    )[0];
    if (faults.pusherEfficiency <= 0.01) {
      stopTakeoff("pusher-failure");
      return;
    }
    if (battery.low || faults.linkLost) {
      stopTakeoff("safety");
      return;
    }
    if (groundRolling) {
      if (
        !runway.onRunway ||
        (Math.abs(wrap(angles(state.slice(6, 10))[2])) > 0.35 &&
          Math.hypot(state[3], state[4]) > 1)
      ) {
        stopTakeoff("runway-excursion");
        return;
      }
      const stopping = Math.hypot(state[3], state[4]) ** 2 / (2 * 0.2 * G) + 5;
      if (runway.distanceRemainingM < stopping + 5) {
        stopTakeoff("runway-end");
        return;
      }
      if (
        time - takeoffStarted > 12 &&
        forwardAirspeed < runway.rotationSpeedMps * 0.8
      ) {
        stopTakeoff("insufficient-airspeed");
        return;
      }
      if (
        takeoffPhase === "runup" &&
        state[pusherIndex] > 0.85 &&
        pusherCommand > 0.9
      ) {
        takeoffPhase = "roll";
        runwayBrake = 0;
      }
      if (
        takeoffPhase === "roll" &&
        wing.airspeedMps >= runway.rotationSpeedMps &&
        state[3] > 2 &&
        forwardAirspeed >= 0.9 * runway.rotationSpeedMps &&
        Math.abs(wrap(angles(state.slice(6, 10))[2])) < 0.25
      )
        takeoffPhase = "rotate";
    } else {
      takeoffPhase = "climb";
      if (-state[2] >= 28) {
        takeoffPhase = "complete";
        positionTarget = { north: state[0], east: state[1], altitude: 30 };
        explicitPosition = true;
        enterMode("position");
        return;
      }
    }
    const course = Math.atan2(
      observed.velocity[1],
      Math.max(0.1, observed.velocity[0]),
    );
    const bank = groundRolling
      ? 0
      : clamp(
          -0.6 * course - 0.015 * observed.position[1],
          -(-state[2] < 10 ? 0.1 : 0.25),
          -state[2] < 10 ? 0.1 : 0.25,
        );
    target = {
      altitude: 30,
      roll: bank,
      pitch: 0,
      yawRate: 0,
      airspeed: Math.max(
        config.cruiseAirspeed,
        1.4 * stallSpeed(config, Math.max(0, -state[2])),
      ),
    };
    navigation.targetPosition = [RUNWAY.endNorth, 0];
  }

  function takeoff() {
    if (fixedWing && !terrain.runwayCompatible && groundRolling)
      throw new Error("Terrain does not support fixed-wing runway takeoff");
    if (takeoffActive()) throw new Error("Runway takeoff is already active");
    if (safetyReason)
      throw new Error(
        "Cannot take off during a safety return/landing; reset first",
      );
    if (fixedWing && state[2] >= -0.1) {
      if (!runwaySnapshot().onRunway)
        throw new Error("Aircraft must be on the runway before takeoff");
      if (runwaySnapshot().distanceRemainingM < 25)
        throw new Error("Insufficient runway remaining for takeoff");
      if (Math.hypot(state[3], state[4]) > 0.5)
        throw new Error("Stop on the runway before beginning takeoff");
      if (Math.abs(wrap(angles(state.slice(6, 10))[2])) > 0.25)
        throw new Error("Align North with the runway before takeoff");
      arm();
      launchAuthorized = true;
      groundRolling = true;
      runwayBrake = 1;
      takeoffPhase = "runup";
      takeoffAbortReason = "";
      takeoffStarted = time;
      navigation.mode = "takeoff";
      navigation.reason = "";
      navigation.completed = false;
      speedIntegral = 0;
      pusherCommand = 0;
      target = {
        altitude: 30,
        roll: 0,
        pitch: 0,
        yawRate: 0,
        airspeed: Math.max(config.cruiseAirspeed, 1.4 * stallSpeed(config)),
      };
      baseTarget = { ...target };
      return;
    }
    arm();
    altitudeReference = "datum";
    launchAuthorized = true;
    if (hasWing) baseTarget.airspeed = config.cruiseAirspeed;
    positionTarget = {
      north: state[0],
      east: state[1],
      altitude:
        fixedWing || (hasWing && transition === "cruise")
          ? worldCoordinates
            ? groundAt().heightM + 30
            : 30
          : groundAt().heightM + 2,
    };
    explicitPosition = true;
    enterMode("position");
  }
  function injectFault(kind, value) {
    if (kind === "pusher") {
      if (!hasWing) throw new Error("Multirotor has no cruise pusher");
      faults.pusherEfficiency = validateFields(
        value,
        { efficiency: [0, 1] },
        "Pusher fault",
      ).efficiency;
    } else if (kind === "motor") {
      if (fixedWing) throw new Error("Fixed wing has no lift rotors");
      const checked = validateFields(
        value,
        { index: [0, count - 1], efficiency: [0, 1] },
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
    faults = {
      motorEfficiency: Array(count).fill(1),
      linkLost: false,
      pusherEfficiency: 1,
    };
  }
  function setTerrain(data) {
    const next = createTerrain(data);
    next.sample(home[0], home[1]);
    if (
      fixedWing &&
      !next.runwayCompatible &&
      (groundRolling || navigation.mode === "land" || takeoffActive())
    )
      throw new Error(
        "Cannot activate incompatible terrain during fixed-wing runway operations",
      );
    if (next.sample(state[0], state[1]).heightM > -state[2] + 1e-8)
      throw new Error("Terrain update would intersect the aircraft");
    if (
      altitudeReference === "agl" &&
      next.sample(positionTarget.north, positionTarget.east).heightM +
        positionTarget.altitude >
        next.sample(home[0], home[1]).heightM + config.geofenceHeight
    )
      throw new Error(
        "Terrain update would raise the AGL target above geofence height",
      );
    terrain = next;
  }
  function setNistLane(enabled) {
    if (typeof enabled !== "boolean") throw new Error("Invalid NIST lane enabled");
    nistCollider = enabled ? createNistLaneCollider() : null;
    refreshCollisionPreventionState();
  }
  function setObstacles(boxes) {
    obstacles = validateObstacles(boxes).map((box) =>
      obstacleBounds(box, collisionRadius),
    );
    refreshCollisionPreventionState();
  }
  function intersectsBox(from, to, bounds) {
    // Conservative expanded-AABB sweep of the CG envelope, not mesh contact or elastic impact.
    const interval = rayAabb(
      from,
      to.map((value, i) => value - from[i]),
      bounds,
    );
    return (
      interval !== null &&
      Math.max(0, interval.enter) <= Math.min(1, interval.exit)
    );
  }
  function supply(s = state) {
    const soc = Math.max(0, config.batteryInitialSoc - usedWh / config.batteryCapacityWh);
    return batterySupply(config, soc, s.slice(13, servoIndex));
  }
  function batterySnapshot() {
    const availableWh = Math.max(
      0,
      config.batteryCapacityWh * config.batteryInitialSoc - usedWh,
    );
    const soc = availableWh / config.batteryCapacityWh;
    const electrical = batterySupply(config, soc, state.slice(13, servoIndex));
    return {
      soc,
      ...electrical,
      usedWh,
      remainingSeconds: electrical.power > 0 ? (availableWh * 3600) / electrical.power : 0,
      low: soc <= config.lowBatteryThreshold,
      critical: soc <= config.criticalBatteryThreshold,
    };
  }
  function safetyMode(mode, reason) {
    if (safetyReason === reason && navigation.mode === mode) return;
    if (takeoffActive()) {
      takeoffPhase = "aborted";
      takeoffAbortReason = "safety";
      runwayBrake = groundRolling ? 1 : 0;
    }
    safetyReason = reason;
    enterMode(mode, reason);
  }
  function refreshCollisionPreventionState() {
    collisionPreventionState = collisionPreventionInactive(collisionPreventionSettings);
    if (collisionPreventionState && ["position", "mission", "rtl", "land"].includes(navigation.mode) &&
        !fixedWing && (!hasWing || (transition === "hover" && transitionProgress === 0))) {
      collisionPreventionState.applicable = true;
      collisionPreventionState.reason = "clear";
    }
  }
  function setCollisionPrevention(value) {
    if (value === undefined) throw Error("Collision prevention requires complete settings");
    const next = validateCollisionPreventionSettings(value);
    if (next.enabled && (external || fixedWing || (hasWing && (transition !== "hover" || transitionProgress !== 0))))
      throw Error("Collision prevention requires an internal multirotor or fully hovering VTOL");
    collisionPreventionSettings = next;
    refreshCollisionPreventionState();
    return { ...next };
  }
  function positionInsideGeofence(position, zones = geofenceZones) {
    return Math.hypot(position[0] - home[0], position[1] - home[1]) <= config.geofenceRadius &&
      -position[2] <= ceiling() && geofenceContains(zones, position.slice(0, 2));
  }
  function setGeofenceZones(value) {
    if (value === undefined) throw Error("Geofence zones require complete settings");
    const next = validateGeofenceZones(value);
    if (external && next.enabled) throw Error("External control does not support enabled geofence zones");
    if (next.enabled) {
      if (!geofenceContains(next, home)) throw Error("Home is outside the geofence zones");
      const spawn = initialSpawn ?? { north: 0, east: 0, altitude: config.startOnGround ? 0 : fixedWing ? 30 : 2 };
      const spawnHeight = createTerrain(initialTerrain).sample(spawn.north, spawn.east).heightM + spawn.altitude;
      if (!positionInsideGeofence([spawn.north, spawn.east, -spawnHeight], next)) throw Error("Initial spawn is outside the geofence");
      if (!positionInsideGeofence(state, next)) throw Error("Current position is outside the geofence");
      if (explicitPosition || ["position", "mission", "rtl", "land"].includes(navigation.mode))
        validatePoint(positionTarget, Object.hasOwn(positionTarget, "hold"), altitudeReference, next);
      for (const point of navigation.waypoints) validatePoint(point, true, "datum", next);
    }
    geofenceZones = next;
    return structuredClone(next);
  }
  function checkGeofence(from = state, to = state) {
    if (external || !armed || crashed || failed || (!launchAuthorized && state[2] >= groundDown() - .001)) return;
    if (positionInsideGeofence(from) && positionInsideGeofence(to) &&
      geofenceSegmentAllowed(geofenceZones, from.slice(0, 2), to.slice(0, 2))) return;
    navigation.fenceBreached = true;
    // The fence requests the existing direct RTL; it does not clamp physics or plan a route around exclusions.
    if (!battery.critical && safetyReason !== "critical-battery" && navigation.mode !== "land") safetyMode("rtl", "geofence");
  }
  function navigate(dt) {
    refreshCollisionPreventionState();
    if (
      !armed ||
      crashed ||
      failed ||
      (!launchAuthorized && state[2] >= groundDown() - 0.001)
    )
      return;
    checkGeofence();
    if (battery.critical) safetyMode("land", "critical-battery");
    else if (safetyReason !== "critical-battery") {
      if (faults.linkLost && !safetyReason) safetyMode("rtl", "link-loss");
      if (battery.low && !safetyReason && navigation.mode !== "land")
        safetyMode("rtl", "low-battery");
    }
    if (fixedWing && navigation.mode === "takeoff") {
      if (takeoffActive()) navigateTakeoff(dt);
      return;
    }
    if (hasWing && (fixedWing || transition === "cruise")) {
      if (config.airframe === "vtol" && transitionPhase === "accelerating") {
        target = {
          ...baseTarget,
          altitude: positionTarget.altitude,
          roll: 0,
          pitch: 0,
          yawRate: 0,
        };
        return;
      }
      if (navigation.mode !== "attitude") navigateWing(dt);
      return;
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
        positionTarget.north = home[0];
        positionTarget.east = home[1];
      }
      if (
        rtlPhase === "return" &&
        Math.hypot(
          observed.position[0] - home[0],
          observed.position[1] - home[1],
        ) < 0.75 &&
        Math.hypot(...observed.velocity) < 0.5
      ) {
        enterMode("land", navigation.reason || "rtl");
        positionTarget.north = home[0];
        positionTarget.east = home[1];
      }
    }
    if (
      navigation.mode === "land" &&
      (config.airframe !== "vtol" ||
        (transitionProgress < 0.02 && Math.hypot(...state.slice(3, 5)) < 1))
    )
      positionTarget.altitude = Math.max(
        groundAt().heightM,
        positionTarget.altitude - 0.5 * dt,
      );
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
    const desiredVelocityNe = errors.map(v => v * velocityScale);
    let acceleration = desiredVelocityNe.map(
      (v, i) => 2.2 * (v - observed.velocity[i]) + 0.12 * horizontalIntegral[i],
    );
    if (collisionPreventionSettings.enabled && (!hasWing || (transition === "hover" && transitionProgress === 0))) {
      collisionPreventionState = constrainHorizontalMotion({
        settings: collisionPreventionSettings, positionNed: state.slice(0, 3),
        velocityNe: state.slice(3, 5), desiredVelocityNe, radius: collisionRadius,
        obstacles, nistCollider, peers: collisionPreventionPeers,
      });
      if (collisionPreventionState.active) {
        horizontalIntegral = [0, 0];
        acceleration = collisionPreventionState.commandedAccelerationNe;
      }
    }
    const scale = Math.min(1, 3 / Math.max(1e-9, Math.hypot(...acceleration)));
    acceleration = acceleration.map((v) => v * scale);
    if (collisionPreventionState?.applicable) collisionPreventionState.commandedAccelerationNe = [...acceleration];
    const yaw = observed.attitude[2],
      north = acceleration[0],
      east = acceleration[1];
    target = {
      altitude:
        altitudeReference === "agl"
          ? groundAt().heightM + positionTarget.altitude
          : positionTarget.altitude,
      roll: Math.atan2(-Math.sin(yaw) * north + Math.cos(yaw) * east, G),
      pitch: -Math.atan2(Math.cos(yaw) * north + Math.sin(yaw) * east, G),
      yawRate: 0,
      airspeed: hasWing ? baseTarget.airspeed : 0,
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
    if (external) throw new Error("External control requires stepExternal()");
    return integrate(dt);
  }
  function applyExternalActuators(input) {
    if (!external)
      throw new Error("External actuators require external control");
    if (
      !input ||
      Object.getPrototypeOf(input) !== Object.prototype ||
      Object.keys(input).some(
        (key) =>
          !["sequence", "timeUs", "armed", "rotorSpeedCommands"].includes(key),
      ) ||
      !Number.isSafeInteger(input.sequence) ||
      input.sequence <= 0 ||
      input.sequence <= (externalInput?.sequence ?? 0) ||
      externalPending ||
      !Number.isSafeInteger(input.timeUs) ||
      input.timeUs !== externalTimeUs ||
      typeof input.armed !== "boolean" ||
      !Array.isArray(input.rotorSpeedCommands) ||
      input.rotorSpeedCommands.length !== 4 ||
      !Array.from(input.rotorSpeedCommands).every(
        (value) => Number.isFinite(value) && value >= 0 && value <= 1,
      )
    )
      throw new Error(
        "External input requires a fresh positive sequence, exact plant timeUs, armed boolean and four rotor speeds in [0,1]",
      );
    externalInput = {
      ...input,
      rotorSpeedCommands: [...input.rotorSpeedCommands],
    };
    externalPending = true;
  }
  function stepExternal(dt = 0.004) {
    if (!external) throw new Error("stepExternal requires external control");
    if (
      !Number.isFinite(dt) ||
      dt <= 0 ||
      dt > 0.02 ||
      !Number.isSafeInteger(dt * 1e6)
    )
      throw new Error(
        "External dt must be positive integer microseconds and at most 0.02 seconds",
      );
    if (!externalPending)
      throw new Error(
        "A fresh external actuator input is required for each step",
      );
    const result = integrate(dt);
    externalPending = false;
    externalTimeUs += Math.round(dt * 1e6);
    return result;
  }
  function readSensors() {
    const acceleration = derivative(state).slice(3, 6);
    // Flat unilateral ground support cancels downward acceleration, including while armed.
    if (flatGround()) {
      if (state[2] >= -1e-9 && state[5] >= 0)
        acceleration[2] = Math.min(0, acceleration[2]);
    } else if (state[2] >= groundDown() - 1e-9) {
      const normal = groundAt().normalNed,
        inward = acceleration.reduce((sum, v, i) => sum + v * normal[i], 0);
      const leaving =
        state.slice(3, 6).reduce((sum, v, i) => sum + v * normal[i], 0) > 1e-9;
      if (inward < 0 && !leaving)
        for (let i = 0; i < 3; i++) acceleration[i] -= inward * normal[i];
    }
    acceleration[2] -= G;
    return {
      timeUs: external ? externalTimeUs : Math.round(time * 1e6),
      positionNed: state.slice(0, 3),
      velocityNed: state.slice(3, 6),
      quaternionWxyz: state.slice(6, 10),
      ratesFrd: state.slice(10, 13),
      specificForceFrd: inverseRotate(
        rotation(state.slice(6, 10)),
        acceleration,
      ),
    };
  }
  function checkpoint() {
    const ground = terrain;
    const nistGeometry = nistCollider;
    const restoreGimbal = gimbal.checkpoint();
    const saved = structuredClone({
      geofenceZones,
      collisionPreventionSettings,
      collisionPreventionState,
      collisionPreventionPeers,
      state,
      time,
      target,
      baseTarget,
      scenario,
      scenarioStart,
      commands,
      integrals,
      randomState,
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
      gustVector,
      navigation,
      positionTarget,
      explicitPosition,
      horizontalIntegral,
      holdSince,
      rtlPhase,
      armed,
      crashed,
      faults,
      obstacles,
      usedWh,
      battery,
      safetyReason,
      turbulenceState,
      turbulenceGoal,
      nextTurbulence,
      pusherCommand,
      surfaceCommands,
      speedIntegral,
      transition,
      transitionPhase,
      transitionProgress,
      landing,
      groundRolling,
      missionLegStart,
      missionRejoin,
      takeoffPhase,
      takeoffAbortReason,
      takeoffStarted,
      runwayBrake,
      launchAuthorized,
      altitudeReference,
      externalInput,
      externalPending,
      externalTimeUs,
      config,
    });
    let used = false;
    return () => {
      if (used) throw new Error("Simulation rollback was already used");
      used = true;
      Object.assign(config, saved.config);
      terrain = ground;
      nistCollider = nistGeometry;
      restoreGimbal();
      ({
        geofenceZones,
        collisionPreventionSettings,
        collisionPreventionState,
        collisionPreventionPeers,
        state,
        time,
        target,
        baseTarget,
        scenario,
        scenarioStart,
        commands,
        integrals,
        randomState,
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
        gustVector,
        navigation,
        positionTarget,
        explicitPosition,
        horizontalIntegral,
        holdSince,
        rtlPhase,
        armed,
        crashed,
        faults,
        obstacles,
        usedWh,
        battery,
        safetyReason,
        turbulenceState,
        turbulenceGoal,
        nextTurbulence,
        pusherCommand,
        surfaceCommands,
        speedIntegral,
        transition,
        transitionPhase,
        transitionProgress,
        landing,
        groundRolling,
        missionLegStart,
        missionRejoin,
        takeoffPhase,
        takeoffAbortReason,
        takeoffStarted,
        runwayBrake,
        launchAuthorized,
        altitudeReference,
        externalInput,
        externalPending,
        externalTimeUs,
      } = saved);
    };
  }
  function integrate(dt) {
    if (terrain.data.kind !== "dem") return integrateUnchecked(dt);
    const rollback = checkpoint();
    try {
      return integrateUnchecked(dt);
    } catch (error) {
      rollback();
      throw error;
    }
  }
  function integrateUnchecked(dt) {
    const startingPower = supply().power;
    if (!Number.isFinite(dt) || dt <= 0 || dt > 0.02)
      throw new Error("dt must be greater than zero and at most 0.02 seconds");
    const previousTarget = target;
    const elapsed = time - scenarioStart;
    target = { ...baseTarget };
    if (!external && scenario === "step" && elapsed >= 2 && elapsed < 6)
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
      previousCommands = commands,
      previousPusher = pusherCommand,
      previousSurfaces = surfaceCommands;
    if (external) {
      armed = externalInput.armed && !crashed && !failed && battery.soc > 0;
      launchAuthorized = armed;
      commands = armed ? [...externalInput.rotorSpeedCommands] : [0, 0, 0, 0];
      pusherCommand = 0;
      surfaceCommands = [0, 0, 0];
    } else {
      sample();
      navigate(dt);
      control(dt);
    }
    const add = (s, k, f) => s.map((v, i) => v + k[i] * f);
    const a = derivative(state),
      b = derivative(add(state, a, dt / 2));
    const c = derivative(add(state, b, dt / 2)),
      d = derivative(add(state, c, dt));
    const nextState = state.map(
      (v, i) => v + (dt * (a[i] + 2 * b[i] + 2 * c[i] + d[i])) / 6,
    );
    if (!terrain.contains(nextState[0], nextState[1]))
      throw new TerrainCoverageError(nextState[0], nextState[1]);
    const norm = Math.hypot(...nextState.slice(6, 10));
    const nextAero = aero(nextState);
    const forceFinite = [
      ...nextAero.forceBody,
      ...nextAero.momentBody,
      nextAero.airspeedMps,
      nextAero.dynamicPressurePa,
      nextAero.liftN,
      nextAero.dragN,
      nextAero.loadFactor,
    ].every(Number.isFinite);
    if (
      !nextState.every(Number.isFinite) ||
      !forceFinite ||
      !Number.isFinite(norm) ||
      norm < 1e-12 ||
      !Number.isFinite(
        altitudeSquaredError + (target.altitude + nextState[2]) ** 2,
      )
    ) {
      imu = previousImu;
      commands = previousCommands;
      pusherCommand = previousPusher;
      surfaceCommands = previousSurfaces;
      target = previousTarget;
      throw new Error(
        "Numerical integration diverged; reduce controller gains/noise or use a smaller integration step",
      );
    }
    const previousPosition = state.slice(0, 3);
    const restingContact =
      !flatGround() &&
      !fixedWing &&
      Math.abs(state[2] - groundDown()) <= 1e-8 &&
      Math.hypot(...state.slice(3, 6)) < 0.03;
    if (!crashed) {
      state = nextState;
      for (let i = 6; i < 10; i++) state[i] /= norm;
    }
    for (let i = 13; i < servoIndex; i++) state[i] = clamp(state[i], 0, 1);
    for (let i = servoIndex; i < stateSize; i++)
      state[i] = clamp(state[i], -WING.surfaceLimit, WING.surfaceLimit);
    if (
      fixedWing &&
      groundRolling &&
      takeoffPhase === "rotate" &&
      state[2] < -0.002 &&
      state[5] < 0
    ) {
      groundRolling = false;
      takeoffPhase = "climb";
      runwayBrake = 0;
    }
    const localGround = groundAt();
    const runwayContact =
      terrain.runwayCompatible &&
      fixedWing &&
      Math.abs(localGround.heightM) <= RUNWAY_FLAT_TOLERANCE_M &&
      state[0] >= RUNWAY.startNorth &&
      state[0] <= RUNWAY.endNorth &&
      Math.abs(state[1]) <= RUNWAY.halfWidth;
    if (!flatGround() && !crashed) {
      const hit = terrain.sweep(previousPosition, state.slice(0, 3));
      const penetrating = state[2] > -localGround.heightM + 1e-9;
      if (hit || penetrating) {
        const contact = hit ?? {
          positionNed: [state[0], state[1], -localGround.heightM],
          normalNed: localGround.normalNed,
        };
        const normal = contact.normalNed;
        const inward = state
          .slice(3, 6)
          .reduce((sum, v, i) => sum + v * normal[i], 0);
        const rotorLanding =
          !fixedWing &&
          (!hasWing || (transition === "hover" && transitionProgress < 0.02));
        const gentle =
          rotorLanding &&
          (restingContact || normal[2] < -Math.cos(Math.PI / 12)) &&
          -inward < 2.5 &&
          Math.hypot(state[3], state[4]) < 1.5 &&
          Math.abs(angles(state.slice(6, 10))[0]) < 0.5 &&
          Math.abs(angles(state.slice(6, 10))[1]) < 0.5;
        if (!runwayContact) {
          state.splice(0, 3, ...contact.positionNed);
          if (!gentle) {
            crashed = true;
            navigation.reason = "terrain-impact";
            disarm();
            state.splice(3, 3, 0, 0, 0);
            for (let i = 10; i < stateSize; i++) state[i] = 0;
          } else {
            if (inward < 0)
              for (let i = 0; i < 3; i++) state[3 + i] -= inward * normal[i];
            for (let i = 3; i < 6; i++) state[i] *= Math.exp(-8 * dt);
          }
        }
      }
    }
    if (flatGround() || runwayContact) {
      if (hasWing && !crashed && state[2] >= 0 && !groundRolling) {
        if (
          fixedWing &&
          navigation.mode === "land" &&
          state[5] < 1.6 &&
          Math.abs(angles(state.slice(6, 10))[0]) < 0.25
        ) {
          groundRolling = true;
          runwayBrake = 1;
          state[2] = 0;
          state[5] = 0;
          for (let i = 10; i < stateSize; i++) state[i] = 0;
        } else if (fixedWing || state[5] > 2.5) {
          crashed = true;
          navigation.reason = "ground-impact";
          disarm();
        }
      }
      // shortcut: flat inelastic ground; conventional wing landing additionally models wheel rollout.
      if (state[2] >= 0) {
        state[2] = 0;
        state[5] = Math.min(0, state[5]);
        if (!groundRolling) {
          state[3] *= Math.exp(-8 * dt);
          state[4] *= Math.exp(-8 * dt);
        }
      }
    }
    const nistHit = !crashed && armed
      ? nistCollider?.sweep(previousPosition, state.slice(0, 3), collisionRadius) ?? null
      : null;
    if (
      !crashed &&
      armed &&
      (obstacles.some((box) =>
        intersectsBox(previousPosition, state.slice(0, 3), box),
      ) || nistHit !== null)
    ) {
      crashed = true;
      navigation.reason = "obstacle-collision";
      if (nistHit !== null)
        for (let i = 0; i < 3; i++)
          state[i] = previousPosition[i] + (state[i] - previousPosition[i]) * nistHit;
      disarm();
      state.splice(3, 3, 0, 0, 0);
      for (let i = 10; i < stateSize; i++) state[i] = 0;
    }
    if (
      !crashed &&
      !fixedWing &&
      navigation.mode === "land" &&
      (flatGround() ||
        restingContact ||
        groundAt().normalNed[2] < -Math.cos(Math.PI / 12)) &&
      state[2] > groundDown() - 0.12 &&
      Math.abs(state[5]) < 0.6
    ) {
      state[2] = groundDown();
      state.splice(3, 3, 0, 0, 0);
      navigation.completed = true;
      disarm();
    }
    if (
      groundRolling &&
      !["runup", "roll", "rotate"].includes(takeoffPhase) &&
      Math.hypot(state[3], state[4]) < 0.08 &&
      (navigation.mode === "land" || takeoffPhase === "aborted")
    ) {
      if (navigation.mode === "land") navigation.completed = true;
      disarm();
    }
    if (
      crashed ||
      (!armed &&
        state[2] >= groundDown() - 0.001 &&
        (!fixedWing || Math.hypot(state[3], state[4]) < 0.08))
    ) {
      state.splice(3, 3, 0, 0, 0);
      for (let i = 10; i < stateSize; i++) state[i] = 0;
    }
    if (navigation.mode === "land" && navigation.completed && !armed)
      target = {
        altitude: groundAt().heightM,
        roll: 0,
        pitch: 0,
        yawRate: 0,
        airspeed: 0,
      };
    const previousWh = usedWh;
    // Trapezoidal electrical work includes the SOC-dependent voltage at the end of the tick.
    for (let i = 0; i < 3; i++)
      usedWh = Math.min(
        config.batteryCapacityWh * config.batteryInitialSoc,
        previousWh + ((startingPower + supply().power) * dt) / 7200,
      );
    battery = batterySnapshot();
    checkGeofence(previousPosition, state);
    if (battery.soc <= 0) disarm();
    time += dt;
    gimbal.update(dt, state.slice(6, 10));
    if (external) {
      const sensors = readSensors();
      imu = { accel: sensors.specificForceFrd, gyro: sensors.ratesFrd };
    }
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
    const surface = groundAt(),
      clearance = -state[2] - surface.heightM;
    return {
      time,
      collisionPrevention: structuredClone(collisionPreventionState),
      gimbal: gimbal.snapshot(),
      ground: {
        heightM: surface.heightM,
        normalNed: [...surface.normalNed],
        clearanceM: clearance === 0 ? 0 : clearance,
      },
      ...(external
        ? {
            controlSource,
            externalInput: externalInput
              ? {
                  ...externalInput,
                  rotorSpeedCommands: [...externalInput.rotorSpeedCommands],
                }
              : null,
          }
        : {}),
      position: state.slice(0, 3),
      velocity: state.slice(3, 6),
      quaternion: state.slice(6, 10),
      attitude: angles(state.slice(6, 10)),
      rates: state.slice(10, 13),
      airframe: config.airframe,
      runway: runwaySnapshot(),
      motors: state.slice(13, motorEnd),
      propulsion: {
        command: pusherCommand,
        motor: state[pusherIndex],
        thrustN: hasWing
          ? config.maxPusherThrust *
            state[pusherIndex] ** 2 *
            supply().motorSpeedScale ** 2 *
            faults.pusherEfficiency
          : 0,
      },
      controlSurfaces: {
        aileron: state[servoIndex],
        elevator: state[servoIndex + 1],
        rudder: state[servoIndex + 2],
      },
      flight: flightSnapshot(),
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
        altitudeReference,
        positionAltitude: positionTarget.altitude,
        waypoints: navigation.waypoints.map((v) => ({ ...v })),
        home: [...navigation.home],
        targetPosition: [...navigation.targetPosition],
      },
      battery: { ...battery },
      faults: {
        motorEfficiency: [...faults.motorEfficiency],
        linkLost: faults.linkLost,
        pusherEfficiency: faults.pusherEfficiency,
      },
      armed,
      status: crashed
        ? "crashed"
        : failed
          ? "failsafe"
          : state[2] >= groundDown() - 0.001
            ? "landed"
            : steps
              ? "flying"
              : "ready",
      wind: [...wind],
    };
  }
  function setWind(value) {
    if (
      !value ||
      Reflect.ownKeys(value).sort().join(",") !== "direction,speed" ||
      !Number.isFinite(value.speed) ||
      value.speed < 0 ||
      value.speed > 30 ||
      !Number.isFinite(value.direction) ||
      value.direction < 0 ||
      value.direction > 360
    )
      throw new Error("Invalid world wind");
    config.wind = value.speed;
    config.windDirection = value.direction;
    wind = currentWind();
  }
  function crash(reason = "vehicle-collision") {
    if (reason !== "vehicle-collision")
      throw new Error("Invalid peer collision reason");
    crashed = true;
    disarm();
    navigation.reason = reason;
    state.splice(3, 3, 0, 0, 0);
    for (let i = 10; i < stateSize; i++) state[i] = 0;
  }
  const internalCommand =
    (fn) =>
    (...args) => {
      requireInternal();
      return fn(...args);
    };
  reset();
  return {
    checkpoint,
    getGeofenceZones: () => structuredClone(geofenceZones),
    setGeofenceZones,
    setCollisionPrevention,
    setCollisionPreventionPeers: peers => { collisionPreventionPeers = structuredClone(peers); },
    crash: internalCommand(crash),
    setWind: internalCommand(setWind),
    setTerrain: internalCommand(setTerrain),
    setGimbal: internalCommand((settings) => gimbal.set(settings)),
    setTargets: internalCommand(setTargets),
    setScenario,
    gust,
    step,
    applyExternalActuators,
    stepExternal,
    readSensors,
    snapshot,
    reset,
    setMode: internalCommand(setMode),
    setPositionTarget: internalCommand(setPositionTarget),
    setMission: internalCommand(setMission),
    takeoff: internalCommand(takeoff),
    arm: internalCommand(arm),
    disarm: internalCommand(disarm),
    injectFault,
    clearFaults,
    setNistLane: internalCommand(setNistLane),
    setObstacles: internalCommand(setObstacles),
    setTransition: internalCommand(setTransition),
    abortTakeoff: internalCommand(abortTakeoff),
  };
}
