export const DEFAULT_FOLLOW = Object.freeze({
  leaderId: null,
  frame: "world",
  offset: Object.freeze({ x: -5, y: 0, altitude: 0 }),
  maxDistanceM: 100,
  lostAction: "hold",
  yawMode: "hold",
});
export const FOLLOW_REASONS = Object.freeze([
  "", "pilot", "mode-change", "settings-changed", "follower-disarmed",
  "follower-crashed", "follower-failsafe", "safety-action", "leader-missing",
  "leader-stale", "leader-invalid", "leader-disarmed", "leader-crashed",
  "leader-failsafe", "leader-link-loss", "leader-landed", "target-range",
  "target-geofence", "target-terrain", "target-ground", "target-invalid",
]);
const exact = (value, keys) =>
  value &&
  [Object.prototype, null].includes(Object.getPrototypeOf(value)) &&
  Reflect.ownKeys(value).length === keys.length &&
  keys.every((key) => Object.hasOwn(value, key));
const id = (value) =>
  typeof value === "string" && /^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(value);

export function validateFollowSettings(value = DEFAULT_FOLLOW) {
  if (
    !exact(value, Object.keys(DEFAULT_FOLLOW)) ||
    !(value.leaderId === null || id(value.leaderId)) ||
    !["world", "leader-heading"].includes(value.frame) ||
    !exact(value.offset, ["x", "y", "altitude"]) ||
    !Object.values(value.offset).every((v) => Number.isFinite(v) && Math.abs(v) <= 100) ||
    !Number.isFinite(value.maxDistanceM) || value.maxDistanceM < 1 || value.maxDistanceM > 500 ||
    !["hold", "rtl"].includes(value.lostAction) ||
    !["hold", "leader", "face-leader"].includes(value.yawMode)
  ) throw Error("Invalid follow settings");
  return structuredClone(value);
}

export function validateFollowState(value) {
  if (value === null) return null;
  if (
    !exact(value, ["active", "leaderId", "reason", "sourceTick", "targetPosition"]) ||
    typeof value.active !== "boolean" || !id(value.leaderId) ||
    !FOLLOW_REASONS.includes(value.reason) ||
    (value.active ? value.reason !== "" : value.reason === "") ||
    !Number.isSafeInteger(value.sourceTick) || value.sourceTick < 0 || value.sourceTick > 360000 ||
    !Array.isArray(value.targetPosition) || value.targetPosition.length !== 3 ||
    Reflect.ownKeys(value.targetPosition).length !== 4 ||
    !Array.from(value.targetPosition).every(Number.isFinite)
  ) throw Error("Invalid follow state");
  return structuredClone(value);
}

export function validateFollowRoster(vehicles) {
  const byId = new Map(vehicles.map((v) => [v.id, v]));
  for (const vehicle of vehicles) {
    const seen = new Set([vehicle.id]);
    let leader = vehicle.setup.follow.leaderId;
    while (leader !== null) {
      if (!byId.has(leader)) throw Error("Unknown follow leader ID");
      if (seen.has(leader))
        throw Error("Follow relationships cannot contain self references or cycles");
      seen.add(leader);
      leader = byId.get(leader).setup.follow.leaderId;
    }
  }
}
