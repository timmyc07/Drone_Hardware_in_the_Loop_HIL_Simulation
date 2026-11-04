export const TRAINING_COURSES = {
  free: { title: "自由飛行", gates: [] },
  hover: { title: "定點懸停", gates: [] },
  gates: {
    title: "四門航線",
    gates: [
      { north: 8, east: 0, altitude: 2, yaw: 0, radius: 1.5 },
      { north: 8, east: 8, altitude: 2, yaw: Math.PI / 2, radius: 1.5 },
      { north: 0, east: 8, altitude: 2, yaw: Math.PI, radius: 1.5 },
      { north: 0, east: 0, altitude: 2, yaw: -Math.PI / 2, radius: 1.5 },
    ],
  },
  landing: { title: "精準降落", gates: [] },
};

export function createTrainer(course = "free") {
  if (!Object.hasOwn(TRAINING_COURSES, course))
    throw new Error("Invalid training course");
  const gates = TRAINING_COURSES[course].gates;
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
    elapsed;
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
      nextGate: gates[passed] ?? null,
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
        (v) => Array.isArray(v) && v.length === 3 && v.every(Number.isFinite),
      ) ||
      typeof snapshot.status !== "string" ||
      typeof snapshot.armed !== "boolean"
    ) {
      throw new Error("Invalid training snapshot");
    }
    if (previous && snapshot.time < previous.time) reset();
    if (complete || reason === "crashed") return result();
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
    } else if (course === "gates" && previous) {
      const gate = gates[passed];
      const normal = [Math.cos(gate.yaw), Math.sin(gate.yaw)];
      const planeDistance = (p) =>
        (p[0] - gate.north) * normal[0] + (p[1] - gate.east) * normal[1];
      const before = planeDistance(previous.position),
        after = planeDistance(snapshot.position);
      if (before < 0 && after >= 0) {
        const t = -before / (after - before);
        const cross = previous.position.map(
          (v, i) => v + t * (snapshot.position[i] - v),
        );
        const lateral =
          -(cross[0] - gate.north) * normal[1] +
          (cross[1] - gate.east) * normal[0];
        if (Math.hypot(lateral, -cross[2] - gate.altitude) <= gate.radius - 0.2)
          passed++;
        else misses++;
        complete = passed === gates.length;
      }
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
