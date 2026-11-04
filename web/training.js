import { AIRFRAMES, MULTIROTOR_AIRFRAMES } from "./rotor-geometry.js";

const deepFreeze = (value) => {
  Object.freeze(value);
  for (const item of Object.values(value))
    if (item && typeof item === "object") deepFreeze(item);
  return value;
};
export const TRAINING_COURSES = deepFreeze({
  free: {
    title: "自由飛行",
    airframes: AIRFRAMES,
    gates: [],
  },
  hover: {
    title: "定點懸停",
    airframes: [...MULTIROTOR_AIRFRAMES, "vtol"],
    gates: [],
  },
  gates: {
    title: "四門航線",
    airframes: [...MULTIROTOR_AIRFRAMES, "vtol"],
    gates: [
      { north: 8, east: 0, altitude: 2, yaw: 0, radius: 1.5 },
      { north: 8, east: 8, altitude: 2, yaw: Math.PI / 2, radius: 1.5 },
      { north: 0, east: 8, altitude: 2, yaw: Math.PI, radius: 1.5 },
      { north: 0, east: 0, altitude: 2, yaw: -Math.PI / 2, radius: 1.5 },
    ],
  },
  landing: {
    title: "精準降落",
    airframes: [...MULTIROTOR_AIRFRAMES, "vtol"],
    gates: [],
  },
  runway: { title: "跑道起降循環", airframes: ["fixedwing"], gates: [] },
  circuit: {
    title: "巡航航線",
    airframes: AIRFRAMES,
    gates: [
      { north: 100, east: 0, altitude: 30, yaw: 0, radius: 20 },
      { north: 100, east: 120, altitude: 30, yaw: Math.PI / 2, radius: 20 },
      { north: 0, east: 120, altitude: 30, yaw: Math.PI, radius: 20 },
      { north: 0, east: 0, altitude: 30, yaw: -Math.PI / 2, radius: 20 },
    ],
  },
});

export function isCourseSupported(course, airframe) {
  return (
    typeof course === "string" &&
    AIRFRAMES.includes(airframe) &&
    (course === "custom" ||
      (Object.hasOwn(TRAINING_COURSES, course) &&
        TRAINING_COURSES[course].airframes.includes(airframe)))
  );
}

export function validateCustomGates(value) {
  if (!Array.isArray(value) || value.length > 50)
    throw new Error("Custom course requires at most 50 gates");
  const limits = {
    north: [-500, 500],
    east: [-500, 500],
    altitude: [0.25, 500],
    yaw: [-Math.PI, Math.PI],
    radius: [0.25, 100],
  };
  return Array.from(value, (gate) => {
    if (
      !gate ||
      ![Object.prototype, null].includes(Object.getPrototypeOf(gate)) ||
      Object.keys(gate).length !== 5 ||
      Object.keys(gate).some((key) => !Object.hasOwn(limits, key))
    )
      throw new Error("Invalid custom gate fields");
    return Object.fromEntries(
      Object.entries(limits).map(([key, [min, max]]) => {
        const v = gate[key];
        if (!Number.isFinite(v) || v < min || v > max)
          throw new Error(`Invalid gate ${key}`);
        return [key, v];
      }),
    );
  });
}
export function resolveTrainingGates(scene, airframe = "quadrotor") {
  if (!scene || !isCourseSupported(scene.training, airframe))
    throw new Error("Invalid or unsupported training course");
  const custom = validateCustomGates(scene.customGates);
  if (scene.training === "custom" && !custom.length)
    throw new Error("Custom training needs at least one gate");
  return scene.training === "custom"
    ? custom
    : TRAINING_COURSES[scene.training].gates.map((gate) => ({ ...gate }));
}
export function createTrainer(
  course = "free",
  airframe = "quadrotor",
  customGates = [],
) {
  if (!isCourseSupported(course, airframe))
    throw new Error("Invalid or unsupported training course");
  const gates = resolveTrainingGates(
    { training: course, customGates },
    airframe,
  );
  let previous,
    start,
    passed,
    misses,
    complete,
    score,
    holdSeconds,
    reason,
    descended,
    inside,
    fencePenalty,
    elapsed,
    runwayStage;
  function reset() {
    previous = undefined;
    start = undefined;
    passed = 0;
    misses = 0;
    complete = false;
    score = 0;
    holdSeconds = 0;
    reason = "";
    descended = false;
    inside = false;
    fencePenalty = false;
    elapsed = 0;
    runwayStage = 0;
  }
  function result() {
    return {
      course,
      elapsed,
      passed,
      total: gates.length,
      misses,
      complete,
      score,
      nextGate: gates[passed] ? { ...gates[passed] } : null,
      holdSeconds,
      reason,
    };
  }
  function update(snapshot) {
    if (
      !snapshot ||
      typeof snapshot !== "object" ||
      !Number.isFinite(snapshot.time) ||
      ![snapshot.position, snapshot.velocity].every(
        (v) =>
          Array.isArray(v) &&
          v.length === 3 &&
          Array.from(v).every(Number.isFinite),
      ) ||
      typeof snapshot.status !== "string" ||
      typeof snapshot.armed !== "boolean"
    ) {
      throw new Error("Invalid training snapshot");
    }
    if (previous && snapshot.time < previous.time) reset();
    if (complete || reason === "crashed" || reason === "takeoff-aborted")
      return result();
    if (start === undefined) start = snapshot.time;
    elapsed = snapshot.time - start;
    const dt = previous ? snapshot.time - previous.time : 0;
    const [north, east, down] = snapshot.position;
    const altitude = -down;
    const distance = Math.hypot(north, east);
    const speed = Math.hypot(...snapshot.velocity);
    if (snapshot.status === "crashed") {
      reason = "crashed";
      score = 0;
      previous = snapshot;
      return result();
    }
    if (course === "runway") {
      const runway = snapshot.runway;
      if (
        !runway ||
        typeof runway.onRunway !== "boolean" ||
        typeof runway.weightOnWheels !== "boolean" ||
        !Number.isFinite(runway.rotationSpeedMps) ||
        !Number.isFinite(snapshot.flight?.airspeedMps)
      ) {
        throw new Error("Invalid runway snapshot");
      }
      if (
        runway.takeoffPhase === "aborted" ||
        snapshot.navigation?.reason === "takeoff-aborted"
      ) {
        reason = "takeoff-aborted";
        score = 0;
        previous = { time: snapshot.time, position: [...snapshot.position] };
        return result();
      }
      if (!previous && runway.onRunway && runway.weightOnWheels && speed < 0.25)
        runwayStage = 1;
      if (runwayStage === 1 && ["runup", "roll"].includes(runway.takeoffPhase))
        runwayStage = 2;
      if (
        runwayStage === 2 &&
        ["rotate", "climb"].includes(runway.takeoffPhase)
      )
        runwayStage = 3;
      if (
        runwayStage === 3 &&
        runway.takeoffPhase === "complete" &&
        !runway.weightOnWheels &&
        altitude >= 25 &&
        snapshot.flight.airspeedMps >= runway.rotationSpeedMps
      )
        runwayStage = 4;
      const landed =
        runwayStage === 4 &&
        snapshot.status === "landed" &&
        runway.onRunway &&
        runway.weightOnWheels &&
        !snapshot.armed &&
        speed < 0.25;
      holdSeconds = landed ? holdSeconds + (inside ? dt : 0) : 0;
      inside = landed;
      complete = holdSeconds >= 2;
      score = complete ? 1000 : 0;
      previous = { time: snapshot.time, position: [...snapshot.position] };
      return result();
    }
    if (snapshot.navigation?.fenceBreached) fencePenalty = true;
    const penalty = fencePenalty ? 100 : 0;
    if (course === "hover" || course === "landing") {
      if (altitude > 0.5) descended = true;
      const nowInside =
        course === "hover"
          ? distance <= 0.5 && Math.abs(altitude - 2) <= 0.25 && speed < 0.3
          : descended &&
            Math.abs(altitude) < 0.05 &&
            distance < 1 &&
            speed < 0.2;
      holdSeconds = nowInside ? holdSeconds + (inside ? dt : 0) : 0;
      inside = nowInside;
      const duration = course === "hover" ? 10 : 2;
      complete = holdSeconds >= duration;
      score =
        course === "hover"
          ? 1000 * Math.min(1, holdSeconds / duration)
          : complete
            ? 1000 - distance * 100
            : 0;
      score = Math.max(0, Math.round(score - penalty));
    } else if (gates.length && previous) {
      let lastIntersection = -1;
      while (passed < gates.length) {
        const gate = gates[passed],
          normal = [Math.cos(gate.yaw), Math.sin(gate.yaw)];
        const planeDistance = (p) =>
          (p[0] - gate.north) * normal[0] + (p[1] - gate.east) * normal[1];
        const before = planeDistance(previous.position),
          after = planeDistance(snapshot.position);
        if (!(before < 0 && after >= 0)) break;
        const t = -before / (after - before);
        // Ordered gates must be crossed in increasing time along this physical segment.
        if (t <= lastIntersection) break;
        const cross = previous.position.map(
          (v, i) => v + t * (snapshot.position[i] - v),
        );
        const lateral =
          -(cross[0] - gate.north) * normal[1] +
          (cross[1] - gate.east) * normal[0];
        if (
          Math.hypot(lateral, -cross[2] - gate.altitude) >
          gate.radius - 0.2
        ) {
          misses++;
          break;
        }
        passed++;
        lastIntersection = t;
      }
      complete = passed === gates.length;
      score = Math.max(
        0,
        Math.round(1000 - misses * 100 - elapsed * 2 - penalty),
      );
    }
    previous = { time: snapshot.time, position: [...snapshot.position] };
    return result();
  }
  reset();
  return { update, reset };
}
