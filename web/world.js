import { createSimulation, validateConfig, DEFAULT_CONFIG } from "./sim.js";
import { DEFAULT_SCENE, validateScene } from "./scene-data.js";
import {
  DEFAULT_SENSOR_CONFIG,
  validateSensorConfig,
  assertGeographicSensors,
  createSensorSuite,
} from "./sensors.js";
import { DEFAULT_GIMBAL_SETTINGS, validateGimbalSettings } from "./gimbal.js";
import {
  createTrainer,
  isCourseSupported,
  resolveTrainingGates,
} from "./training.js";
import { createTerrain } from "./terrain.js";
import { getVehicleEnvelope, relativeSweep } from "./peer-geometry.js";

const DT = 0.005;
const plain = (value) =>
  value && [Object.prototype, null].includes(Object.getPrototypeOf(value));
function fields(value, allowed, required = allowed) {
  if (
    !plain(value) ||
    Reflect.ownKeys(value).some((key) => !allowed.includes(key)) ||
    required.some((key) => !Object.hasOwn(value, key))
  )
    throw new Error(`Expected world fields: ${allowed.join(", ")}`);
}
const finite = (value, min, max, label) => {
  if (!Number.isFinite(value) || value < min || value > max)
    throw new Error(`Invalid ${label}`);
  return value;
};
const freeze = (value) => {
  if (value && typeof value === "object" && !Object.isFrozen(value)) {
    Object.freeze(value);
    Object.values(value).forEach(freeze);
  }
  return value;
};
const copy = (value) => structuredClone(value);
function windConfig(value) {
  fields(value, ["speed", "direction"]);
  return {
    speed: finite(value.speed, 0, 30, "wind speed"),
    direction: finite(value.direction, 0, 360, "wind direction"),
  };
}
function simulation(config, setup, scene, spawn, home) {
  return createSimulation(config, {
    terrain: scene.terrain,
    gimbal: setup.gimbal,
    ...(spawn === undefined ? {} : { spawn }),
    ...(home === undefined ? {} : { home }),
  });
}
export function validateVehicleSetup(
  value = {},
  configuration = {},
  scene = DEFAULT_SCENE,
  spawn,
  home,
) {
  fields(value, ["mode", "mission", "sensors", "gimbal", "training"], []);
  const config = validateConfig(configuration),
    surface = createTerrain(scene.terrain);
  const sensors = validateSensorConfig(value.sensors ?? {}),
    gimbal = validateGimbalSettings(value.gimbal);
  assertGeographicSensors(scene, sensors);
  const training = value.training ?? "free",
    mode = value.mode ?? "attitude";
  if (!isCourseSupported(training, config.airframe))
    throw new Error("Training course is not supported by this airframe");
  if (training === "runway" && !surface.runwayCompatible)
    throw new Error("Runway training requires covered flat runway terrain");
  const sim = simulation(config, { gimbal }, scene, spawn, home);
  if (value.mission !== undefined && !Array.isArray(value.mission))
    throw new Error("Mission must be an array");
  const mission = copy(value.mission ?? []);
  if (mission.length) sim.setMission(mission);
  sim.setMode(mode);
  const groundHeight = surface.sample(...(home ?? [0, 0])).heightM;
  createTrainer(
    training,
    config.airframe,
    scene.customGates,
    home ?? [0, 0],
    groundHeight,
  ).update(sim.snapshot());
  return { mode, mission, sensors, gimbal, training };
}
export function defaultWorldConfig(
  configuration = {},
  setup = {},
  scene = DEFAULT_SCENE,
) {
  const config = validateConfig(configuration);
  return validateWorldConfig({
    scene,
    wind: { speed: config.wind, direction: config.windDirection },
    vehicles: [
      {
        id: "drone-1",
        name: "飛行器 1",
        config,
        spawn: {
          north: 0,
          east: 0,
          altitude: config.startOnGround
            ? 0
            : config.airframe === "fixedwing"
              ? 30
              : 2,
          heading: 0,
        },
        home: [0, 0],
        scenario: "hover",
        setup,
      },
    ],
  });
}
export function validateWorldConfig(input) {
  fields(input, ["scene", "wind", "vehicles"]);
  const scene = validateScene(input.scene),
    wind = windConfig(input.wind),
    surface = createTerrain(scene.terrain);
  if (
    !Array.isArray(input.vehicles) ||
    input.vehicles.length < 1 ||
    input.vehicles.length > 8
  )
    throw new Error("A world requires one to eight vehicles");
  const ids = new Set();
  const vehicles = Array.from(input.vehicles, (value) => {
    fields(value, [
      "id",
      "name",
      "config",
      "spawn",
      "home",
      "scenario",
      "setup",
    ]);
    if (
      typeof value.id !== "string" ||
      !/^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(value.id) ||
      ids.has(value.id)
    )
      throw new Error("Vehicle IDs must be unique canonical identifiers");
    ids.add(value.id);
    if (
      typeof value.name !== "string" ||
      !value.name.trim() ||
      value.name.length > 40 ||
      /[\u0000-\u001f\u007f]/.test(value.name)
    )
      throw new Error("Vehicle name must contain 1 to 40 characters");
    const config = validateConfig(value.config);
    if (config.wind !== wind.speed || config.windDirection !== wind.direction)
      throw new Error("Vehicle wind must equal the world wind");
    fields(value.spawn, ["north", "east", "altitude", "heading"]);
    const spawn = {
      north: finite(value.spawn.north, -500, 500, "spawn north"),
      east: finite(value.spawn.east, -500, 500, "spawn east"),
      altitude: finite(value.spawn.altitude, 0, 500, "spawn AGL altitude"),
      heading: finite(value.spawn.heading, -Math.PI, Math.PI, "spawn heading"),
    };
    if (config.startOnGround ? spawn.altitude !== 0 : spawn.altitude <= 0)
      throw new Error("Spawn AGL altitude must match ground/air start");
    if (
      !Array.isArray(value.home) ||
      value.home.length !== 2 ||
      ![0, 1].every((i) => Object.hasOwn(value.home, i))
    )
      throw new Error("Invalid vehicle home");
    const home = value.home.map((v) => finite(v, -500, 500, "home"));
    surface.sample(spawn.north, spawn.east);
    surface.sample(...home);
    if (
      Math.hypot(spawn.north - home[0], spawn.east - home[1]) >
        config.geofenceRadius ||
      surface.sample(spawn.north, spawn.east).heightM + spawn.altitude >
        surface.sample(...home).heightM + config.geofenceHeight
    )
      throw new Error("Initial spawn is outside its home geofence");
    if (!["hover", "step", "gust"].includes(value.scenario))
      throw new Error("Invalid vehicle scenario");
    const setup = validateVehicleSetup(value.setup, config, scene, spawn, home);
    const radius = getVehicleEnvelope(config).radius;
    const position = [
      spawn.north,
      spawn.east,
      -surface.sample(spawn.north, spawn.east).heightM - spawn.altitude,
    ];
    for (const box of scene.obstacles) {
      const low = [
          box.north - box.width / 2,
          box.east - box.depth / 2,
          -box.height,
        ],
        high = [box.north + box.width / 2, box.east + box.depth / 2, 0];
      if (
        position.reduce(
          (sum, v, i) =>
            sum + (v - Math.max(low[i], Math.min(high[i], v))) ** 2,
          0,
        ) <=
        radius ** 2
      )
        throw new Error("Initial vehicle envelope intersects an obstacle");
    }
    return {
      id: value.id,
      name: value.name,
      config,
      spawn,
      home,
      scenario: value.scenario,
      setup,
    };
  }).sort((a, b) => (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
  for (let i = 0; i < vehicles.length; i++)
    for (let j = i + 1; j < vehicles.length; j++) {
      const a = vehicles[i],
        b = vehicles[j];
      const position = (v) => [
        v.spawn.north,
        v.spawn.east,
        -surface.sample(v.spawn.north, v.spawn.east).heightM - v.spawn.altitude,
      ];
      if (
        relativeSweep(
          position(a),
          position(a),
          getVehicleEnvelope(a.config).radius,
          position(b),
          position(b),
          getVehicleEnvelope(b.config).radius,
        ) !== null
      )
        throw new Error("Initial vehicle envelopes overlap");
    }
  return freeze({ scene, wind, vehicles });
}

export function createWorld(input) {
  const initial = validateWorldConfig(input);
  let scene = copy(initial.scene),
    wind = copy(initial.wind),
    tick = 0,
    time = 0,
    collisions = [],
    collisionLatched = false;
  const vehicles = initial.vehicles.map((value) => {
    const spec = copy(value),
      sim = simulation(spec.config, spec.setup, scene, spec.spawn, spec.home);
    sim.setScenario(spec.scenario);
    sim.setObstacles(scene.obstacles);
    if (spec.setup.mission.length) sim.setMission(spec.setup.mission);
    sim.setMode(spec.setup.mode);
    const suite = createSensorSuite(spec.setup.sensors, spec.config.seed);
    suite.setTerrain(scene.terrain);
    suite.setObstacles(scene.obstacles);
    const trainer = createTrainer(
      spec.setup.training,
      spec.config.airframe,
      scene.customGates,
      spec.home,
      createTerrain(scene.terrain).sample(...spec.home).heightM,
    );
    return {
      spec,
      sim,
      suite,
      trainer,
      radius: getVehicleEnvelope(spec.config).radius,
      training: trainer.update(sim.snapshot()),
      sensors: null,
    };
  });
  const peers = () =>
    vehicles.map((v) => ({
      id: v.spec.id,
      position: v.sim.snapshot().position,
      radius: v.radius,
    }));
  function updateSensors() {
    const targets = peers();
    for (const v of vehicles) {
      v.suite.setPeers(targets, v.spec.id);
      v.sensors = v.suite.update(v.sim.snapshot());
    }
  }
  updateSensors();
  const find = (id) => {
    const v = vehicles.find((v) => v.spec.id === id);
    if (!v) throw new Error("Unknown vehicle ID");
    return v;
  };
  function vehicleConfiguration(v, state) {
    const value = copy(v.spec);
    value.setup.mode = state.navigation.mode;
    return value;
  }
  const configuration = () =>
    freeze({
      scene: copy(scene),
      wind: copy(wind),
      vehicles: vehicles.map((v) => vehicleConfiguration(v, v.sim.snapshot())),
    });
  function snapshot() {
    return {
      tick,
      time,
      vehicles: vehicles.map((v) => {
        const state = v.sim.snapshot();
        return {
          ...vehicleConfiguration(v, state),
          snapshot: { ...state, training: copy(v.training) },
          sensors: copy(v.sensors),
          training: copy(v.training),
        };
      }),
      collisions: copy(collisions),
    };
  }
  function checkpoint() {
    const saved = { scene, wind, tick, time, collisions, collisionLatched };
    const restores = vehicles.map((v) => ({
      v,
      spec: copy(v.spec),
      suite: v.suite,
      trainer: v.trainer,
      sensors: v.sensors,
      training: v.training,
      simRestore: v.sim.checkpoint(),
      sensorRestore: v.suite.checkpoint(),
      trainerRestore: v.trainer.checkpoint(),
    }));
    let used = false;
    return () => {
      if (used) throw new Error("World rollback was already used");
      used = true;
      for (const x of restores) {
        x.simRestore();
        x.sensorRestore();
        x.trainerRestore();
        Object.assign(x.v, {
          spec: x.spec,
          suite: x.suite,
          trainer: x.trainer,
          sensors: x.sensors,
          training: x.training,
        });
      }
      ({ scene, wind, tick, time, collisions, collisionLatched } = saved);
    };
  }
  function transaction(fn) {
    const rollback = checkpoint();
    try {
      return fn();
    } catch (error) {
      rollback();
      throw error;
    }
  }
  const requireMutable = () => {
    if (collisionLatched)
      throw new Error(
        "Vehicle collision stopped the world; reset before continuing",
      );
  };
  function step(dt = DT) {
    requireMutable();
    if (dt !== DT)
      throw new Error("The shared world requires fixed 0.005 second steps");
    return transaction(() => {
      const before = vehicles.map((v) => v.sim.snapshot().position);
      for (const v of vehicles) {
        try {
          v.sim.step(dt);
        } catch (error) {
          error.vehicleId = v.spec.id;
          throw error;
        }
      }
      tick++;
      time = vehicles[0].sim.snapshot().time;
      const after = vehicles.map((v) => v.sim.snapshot().position);
      collisions = [];
      for (let i = 0; i < vehicles.length; i++)
        for (let j = i + 1; j < vehicles.length; j++) {
          const fraction = relativeSweep(
            before[i],
            after[i],
            vehicles[i].radius,
            before[j],
            after[j],
            vehicles[j].radius,
          );
          if (fraction !== null)
            collisions.push({
              time,
              tick,
              vehicleIds: [vehicles[i].spec.id, vehicles[j].spec.id],
              fraction,
            });
        }
      for (const collision of collisions)
        for (const id of collision.vehicleIds) find(id).sim.crash();
      collisionLatched = collisions.length > 0;
      for (const v of vehicles) v.training = v.trainer.update(v.sim.snapshot());
      updateSensors();
      return snapshot();
    });
  }
  function command(vehicleId, type, value, fault) {
    requireMutable();
    const v = find(vehicleId);
    return transaction(() => {
      const setters = {
        targets: "setTargets",
        mode: "setMode",
        position: "setPositionTarget",
        mission: "setMission",
        gimbal: "setGimbal",
        transition: "setTransition",
        scenario: "setScenario",
      };
      const actions = {
        arm: "arm",
        disarm: "disarm",
        takeoff: "takeoff",
        abortTakeoff: "abortTakeoff",
        gust: "gust",
        clearFaults: "clearFaults",
      };
      if (Object.hasOwn(setters, type)) v.sim[setters[type]](value);
      else if (Object.hasOwn(actions, type)) v.sim[actions[type]]();
      else if (type === "fault") v.sim.injectFault(fault, value);
      else if (type === "sensorConfig") {
        const config = validateSensorConfig(value);
        assertGeographicSensors(scene, config);
        const suite = createSensorSuite(config, v.spec.config.seed);
        suite.setTerrain(scene.terrain);
        suite.setObstacles(scene.obstacles);
        v.suite = suite;
        v.spec.setup.sensors = config;
      } else if (type === "sensorFault") {
        fields(value, ["sensor", "failed"]);
        v.suite.setFault(value.sensor, value.failed);
      } else if (type === "training") {
        if (!isCourseSupported(value, v.spec.config.airframe))
          throw new Error("Unsupported training course");
        const surface = createTerrain(scene.terrain);
        if (value === "runway" && !surface.runwayCompatible)
          throw new Error("Runway training requires a covered flat runway");
        const next = createTrainer(
          value,
          v.spec.config.airframe,
          scene.customGates,
          v.spec.home,
          surface.sample(...v.spec.home).heightM,
        );
        v.training = next.update(v.sim.snapshot());
        v.trainer = next;
        v.spec.setup.training = value;
      } else throw new Error("Unknown vehicle command");
      if (type === "mission")
        v.spec.setup.mission = copy(v.sim.snapshot().navigation.waypoints);
      if (type === "gimbal")
        v.spec.setup.gimbal = validateGimbalSettings(value);
      if (type === "scenario") v.spec.scenario = value;
      updateSensors();
      return snapshot();
    });
  }
  function setScene(value) {
    requireMutable();
    const next = validateScene(value);
    if (
      JSON.stringify(next.geography) !== JSON.stringify(scene.geography) ||
      JSON.stringify(next.terrain) !== JSON.stringify(scene.terrain)
    )
      throw new Error("Geography or terrain changes require a world reset");
    return transaction(() => {
      for (const v of vehicles) {
        const args = [
          v.spec.config.airframe,
          v.spec.setup.training,
          v.spec.home,
          createTerrain(scene.terrain).sample(...v.spec.home).heightM,
        ];
        const changed =
          JSON.stringify(resolveTrainingGates(scene, ...args)) !==
          JSON.stringify(resolveTrainingGates(next, ...args));
        if (changed) {
          const trainer = createTrainer(
            v.spec.setup.training,
            v.spec.config.airframe,
            next.customGates,
            v.spec.home,
            args[3],
          );
          v.training = trainer.update(v.sim.snapshot());
          v.trainer = trainer;
        }
        v.sim.setTerrain(next.terrain);
        v.sim.setObstacles(next.obstacles);
        v.suite.setTerrain(next.terrain);
        v.suite.setObstacles(next.obstacles);
      }
      scene = next;
      updateSensors();
      return snapshot();
    });
  }
  function setWind(value) {
    requireMutable();
    const next = windConfig(value);
    return transaction(() => {
      for (const v of vehicles) {
        v.sim.setWind(next);
        v.spec.config.wind = next.speed;
        v.spec.config.windDirection = next.direction;
      }
      wind = next;
      return snapshot();
    });
  }
  return {
    get configuration() {
      return configuration();
    },
    snapshot,
    step,
    command,
    setScene,
    setWind,
    checkpoint,
    getVehicle: (id) =>
      snapshot().vehicles.find((v) => v.id === find(id).spec.id),
  };
}
