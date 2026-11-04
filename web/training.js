import {
  AIRFRAMES,
  MULTIROTOR_AIRFRAMES,
  rotorCount,
} from "./rotor-geometry.js";

const deepFreeze = (value) => {
  Object.freeze(value);
  for (const item of Object.values(value))
    if (item && typeof item === "object") deepFreeze(item);
  return value;
};
export const TUTORIAL_STEPS = deepFreeze([
  {
    id: "preflight",
    title: "地面檢查",
    instruction: "開始模擬，保持家點地面、未解鎖，確認電量與故障狀態。",
    holdSeconds: 2,
  },
  {
    id: "armed",
    title: "解鎖並等待",
    instruction: "按解鎖，保持地面一秒；完成後才按起飛。",
    holdSeconds: 1,
  },
  {
    id: "takeoff",
    title: "起飛至兩米",
    instruction: "按起飛，等待機體在家點地面高度上方兩米穩定。",
    holdSeconds: 1,
  },
  {
    id: "hover",
    title: "穩定懸停",
    instruction: "保持位置模式，在家點地面高度上方兩米連續懸停五秒。",
    holdSeconds: 5,
  },
  {
    id: "move",
    title: "向北移動",
    instruction:
      "在位置模式設定家點向北8米、東座標不變、高度為家點地面加2米，抵達後停留三秒。",
    holdSeconds: 3,
  },
  {
    id: "return",
    title: "返回原點",
    instruction:
      "在位置模式設定家點北／東座標、高度為家點地面加2米，抵達後停留三秒。",
    holdSeconds: 3,
  },
  {
    id: "land",
    title: "降落並上鎖",
    instruction: "按降落，等待機體返回地面並自動上鎖，停穩兩秒。",
    holdSeconds: 2,
  },
]);
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
  custom: { title: "自訂課程", airframes: AIRFRAMES, gates: [] },
  tutorial: {
    title: "基礎教學與程序檢核",
    airframes: [...MULTIROTOR_AIRFRAMES, "vtol"],
    gates: [],
  },
});

export function isCourseSupported(course, airframe) {
  return (
    typeof course === "string" &&
    AIRFRAMES.includes(airframe) &&
    Object.hasOwn(TRAINING_COURSES, course) &&
    TRAINING_COURSES[course].airframes.includes(airframe)
  );
}

export function validateCustomGates(value) {
  if (!Array.isArray(value) || value.length > 50)
    throw new Error("Custom course requires at most 50 gates");
  const limits = {
    north: [-1000, 1000],
    east: [-1000, 1000],
    altitude: [-1000, 1500],
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
function validateHome(home, groundHeight) {
  if (
    !Array.isArray(home) ||
    home.length !== 2 ||
    !Array.from(home).every((v) => Number.isFinite(v) && Math.abs(v) <= 500) ||
    !Number.isFinite(groundHeight) ||
    Math.abs(groundHeight) > 1000
  )
    throw new Error("Invalid training home or ground height");
  return [...home];
}
export function resolveTrainingGates(
  scene,
  airframe,
  course,
  home = [0, 0],
  groundHeight = 0,
) {
  if (!scene || !isCourseSupported(course, airframe))
    throw new Error("Invalid or unsupported training course");
  const checkedHome = validateHome(home, groundHeight);
  const custom = validateCustomGates(scene.customGates);
  if (course === "custom" && !custom.length)
    throw new Error("Custom training needs at least one gate");
  return course === "custom"
    ? custom
    : TRAINING_COURSES[course].gates.map((gate) => ({
        ...gate,
        north: gate.north + checkedHome[0],
        east: gate.east + checkedHome[1],
        altitude: gate.altitude + groundHeight,
      }));
}
export function createTrainer(
  course = "free",
  airframe = "quadrotor",
  customGates = [],
  home = [0, 0],
  groundHeight = 0,
) {
  home = validateHome(home, groundHeight);
  const gates = resolveTrainingGates(
    { customGates },
    airframe,
    course,
    home,
    groundHeight,
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
      total: course === "tutorial" ? TUTORIAL_STEPS.length : gates.length,
      misses,
      complete,
      score,
      nextGate: gates[passed] ? { ...gates[passed] } : null,
      holdSeconds,
      reason,
    };
  }
  function tutorialUpdate(snapshot) {
    if (previous && snapshot.time < previous.time) reset();
    if (complete || reason) return result();
    const {
      ground,
      battery,
      faults,
      navigation: nav,
      flight,
      attitude,
    } = snapshot;
    if (
      !ground ||
      !Number.isFinite(ground.clearanceM) ||
      !Number.isFinite(ground.heightM) ||
      !battery ||
      !Number.isFinite(battery.soc) ||
      battery.soc < 0 ||
      battery.soc > 1 ||
      typeof battery.low !== "boolean" ||
      typeof battery.critical !== "boolean" ||
      !faults ||
      !Array.isArray(faults.motorEfficiency) ||
      faults.motorEfficiency.length !== rotorCount(airframe) ||
      !Array.from(faults.motorEfficiency).every(
        (v) => Number.isFinite(v) && v >= 0 && v <= 1,
      ) ||
      typeof faults.linkLost !== "boolean" ||
      !Number.isFinite(faults.pusherEfficiency) ||
      !nav ||
      typeof nav.fenceBreached !== "boolean" ||
      !Array.isArray(nav.targetPosition) ||
      nav.targetPosition.length !== 2 ||
      !Array.from(nav.targetPosition).every(Number.isFinite) ||
      !flight ||
      !Number.isFinite(flight.transitionProgress) ||
      !Array.isArray(attitude) ||
      attitude.length !== 3 ||
      !Array.from(attitude).every(Number.isFinite)
    )
      throw new Error("Invalid tutorial snapshot");
    const clearance = ground.clearanceM,
      speed = Math.hypot(...snapshot.velocity),
      distance = Math.hypot(
        snapshot.position[0] - home[0],
        snapshot.position[1] - home[1],
      );
    const healthy =
      snapshot.controlSource !== "external" &&
      snapshot.airframe === airframe &&
      !battery.low &&
      !battery.critical &&
      battery.soc > 0.2 &&
      !faults.linkLost &&
      faults.motorEfficiency.every((v) => v === 1) &&
      faults.pusherEfficiency === 1 &&
      !nav.fenceBreached &&
      !nav.reason &&
      flight.transition === "hover" &&
      flight.transitionProgress === 0;
    const parked =
      snapshot.status === "landed" &&
      Math.abs(clearance) < 0.05 &&
      speed < 0.1 &&
      distance <= 0.25 &&
      Math.abs(ground.heightM - groundHeight) < 0.01 &&
      Math.abs(attitude[0]) < 0.17 &&
      Math.abs(attitude[1]) < 0.17;
    if (!previous) {
      if (
        !healthy ||
        !parked ||
        snapshot.armed ||
        !["attitude", "position"].includes(nav.mode)
      )
        throw new Error(
          "Tutorial requires an internal, healthy, disarmed aircraft parked at home; reset with ground start before selecting the course",
        );
      start = snapshot.time;
    }
    elapsed = snapshot.time - start;
    const dt = previous ? snapshot.time - previous.time : 0;
    const fail = (why) => {
      reason = why;
      score = 0;
      holdSeconds = 0;
      inside = false;
      previous = {
        time: snapshot.time,
        position: [...snapshot.position],
        clearance,
        mode: nav.mode,
      };
      return result();
    };
    if (snapshot.status === "crashed") return fail("crashed");
    if (snapshot.status === "failsafe") return fail("failsafe");
    if (!healthy) return fail("unsafe");
    if (
      (passed === 0 && snapshot.armed) ||
      (passed === 1 && clearance > 0.1) ||
      (passed >= 2 &&
        passed <= 5 &&
        (!snapshot.armed || nav.mode === "land")) ||
      !["attitude", "position", "land"].includes(nav.mode)
    )
      return fail("procedure-order");
    const validDt = dt > 0 && dt <= 0.1 + 1e-9;
    if (
      passed === 6 &&
      validDt &&
      previous?.mode === "land" &&
      nav.mode === "land" &&
      snapshot.armed &&
      clearance > 0.1 &&
      clearance < 1.75 &&
      snapshot.velocity[2] > 0.02 &&
      clearance < previous.clearance
    )
      descended = true;
    const at = (north, east, maximumSpeed) =>
      snapshot.armed &&
      snapshot.status === "flying" &&
      nav.mode === "position" &&
      Math.hypot(
        snapshot.position[0] - home[0] - north,
        snapshot.position[1] - home[1] - east,
      ) <= 0.5 &&
      Math.abs(-snapshot.position[2] - groundHeight - 2) <= 0.25 &&
      speed < maximumSpeed &&
      Math.abs(attitude[0]) < 0.26 &&
      Math.abs(attitude[1]) < 0.26;
    const commanded = (north, east) =>
      Math.hypot(
        nav.targetPosition[0] - home[0] - north,
        nav.targetPosition[1] - home[1] - east,
      ) < 0.1;
    const eligible = [
      () => parked && !snapshot.armed,
      () => parked && snapshot.armed,
      () => at(0, 0, 0.5) && commanded(0, 0),
      () => at(0, 0, 0.3) && commanded(0, 0),
      () => at(8, 0, 0.3) && commanded(8, 0),
      () => at(0, 0, 0.3) && commanded(0, 0),
      () =>
        descended &&
        nav.mode === "land" &&
        snapshot.status === "landed" &&
        !snapshot.armed &&
        Math.abs(clearance) < 0.05 &&
        distance <= 0.5 &&
        speed < 0.2,
    ][passed]();
    if (dt !== 0)
      holdSeconds = eligible && inside && validDt ? holdSeconds + dt : 0;
    inside = eligible;
    if (holdSeconds + 1e-9 >= TUTORIAL_STEPS[passed].holdSeconds) {
      passed++;
      holdSeconds = 0;
      inside = false;
      complete = passed === TUTORIAL_STEPS.length;
    }
    score = Math.round((1000 * passed) / TUTORIAL_STEPS.length);
    previous = {
      time: snapshot.time,
      position: [...snapshot.position],
      clearance,
      mode: nav.mode,
    };
    return result();
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
    if (course === "tutorial") return tutorialUpdate(snapshot);
    if (previous && snapshot.time < previous.time) reset();
    if (complete || reason === "crashed" || reason === "takeoff-aborted")
      return result();
    if (start === undefined) start = snapshot.time;
    elapsed = snapshot.time - start;
    const dt = previous ? snapshot.time - previous.time : 0;
    const [north, east, down] = snapshot.position;
    const altitude = -down;
    const distance = Math.hypot(north - home[0], east - home[1]);
    const homeAltitude = altitude - groundHeight;
    const speed = Math.hypot(...snapshot.velocity);
    if (snapshot.status === "crashed") {
      reason = "crashed";
      score = 0;
      previous = { time: snapshot.time, position: [...snapshot.position] };
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
      if (homeAltitude > 0.5) descended = true;
      const nowInside =
        course === "hover"
          ? distance <= 0.5 && Math.abs(homeAltitude - 2) <= 0.25 && speed < 0.3
          : descended &&
            Math.abs(homeAltitude) < 0.05 &&
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
  function checkpoint() {
    const saved = structuredClone({
      previous,
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
      runwayStage,
    });
    let used = false;
    return () => {
      if (used) throw new Error("Training checkpoint already restored");
      used = true;
      ({
        previous,
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
        runwayStage,
      } = saved);
    };
  }
  reset();
  return { update, reset, checkpoint };
}
