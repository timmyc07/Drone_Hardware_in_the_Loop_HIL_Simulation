// NED solid AABBs shared by physics, sensors and inspection; no renderer dependency.
const LIMITS = Object.freeze({
  north: [-500, 500],
  east: [-500, 500],
  width: [0.01, 100],
  depth: [0.01, 100],
  height: [0.01, 500],
  baseAltitude: [-1000, 1500],
});
const KEYS = Object.keys(LIMITS);

export function validateObstacles(value) {
  if (
    !Array.isArray(value) ||
    value.length > 50 ||
    Reflect.ownKeys(value).length !== value.length + 1 ||
    !Array.from({ length: value.length }, (_, i) =>
      Object.hasOwn(value, i),
    ).every(Boolean)
  )
    throw new Error(
      "Invalid obstacles: expected a dense array of at most 50 boxes",
    );
  return value.map((box) => {
    if (
      !box ||
      ![Object.prototype, null].includes(Object.getPrototypeOf(box)) ||
      Reflect.ownKeys(box).length !== KEYS.length ||
      Reflect.ownKeys(box).some((key) => !KEYS.includes(key))
    )
      throw new Error("Invalid obstacle fields");
    const result = Object.fromEntries(
      KEYS.map((key) => {
        const number = box[key],
          [low, high] = LIMITS[key];
        if (!Number.isFinite(number) || number < low || number > high)
          throw new Error(`Invalid obstacle ${key}`);
        return [key, number];
      }),
    );
    if (result.baseAltitude + result.height > 1500)
      throw new Error("Obstacle top must not exceed altitude 1500m");
    return result;
  });
}

// Hot geometry helpers accept validated inputs; callers can cache these bounds.
export function obstacleBounds(box, padding = 0) {
  return {
    low: [
      box.north - box.width / 2 - padding,
      box.east - box.depth / 2 - padding,
      -(box.baseAltitude + box.height) - padding,
    ],
    high: [
      box.north + box.width / 2 + padding,
      box.east + box.depth / 2 + padding,
      -box.baseAltitude + padding,
    ],
  };
}

export function obstacleContainsVehicle(box, position, radius) {
  const bounds = obstacleBounds(box, radius);
  return position.every(
    (value, i) => value >= bounds.low[i] && value <= bounds.high[i],
  );
}

// Raw line interval, including negative t and infinite all-parallel intervals.
// Callers clip to their ray/segment domain and choose their inside-origin policy.
export function rayAabb(origin, direction, bounds) {
  let enter = -Infinity,
    exit = Infinity;
  for (let i = 0; i < 3; i++) {
    if (Math.abs(direction[i]) < 1e-12) {
      if (origin[i] < bounds.low[i] || origin[i] > bounds.high[i]) return null;
    } else {
      const a = (bounds.low[i] - origin[i]) / direction[i],
        b = (bounds.high[i] - origin[i]) / direction[i];
      enter = Math.max(enter, Math.min(a, b));
      exit = Math.min(exit, Math.max(a, b));
      if (exit < enter) return null;
    }
  }
  return { enter, exit };
}
