import { validateConfig } from "./sim.js";

const vector = (value) => {
  if (
    !Array.isArray(value) ||
    value.length !== 3 ||
    ![0, 1, 2].every(
      (i) =>
        Object.hasOwn(value, i) &&
        Number.isFinite(value[i]) &&
        Math.abs(value[i]) <= 1e9,
    )
  )
    throw new Error("Peer geometry requires finite dense three-vectors");
  return value;
};
const radius = (value) => {
  if (!Number.isFinite(value) || value <= 0 || value > 1000)
    throw new Error("Invalid peer radius");
  return value;
};
// Conservative CG-centered sphere; neither mesh contact, elastic impact nor exact rotor/wing TOI.
export function getVehicleEnvelope(input) {
  const config = validateConfig(input);
  const wing = ["fixedwing", "vtol"].includes(config.airframe);
  return Object.freeze({
    radius: Math.max(
      0.25,
      config.arm + 0.15,
      wing ? Math.hypot(config.wingSpan / 2, 2 * config.wingChord, 0.15) : 0,
    ),
  });
}
export function relativeSweep(fromA, toA, radiusA, fromB, toB, radiusB) {
  [fromA, toA, fromB, toB].forEach(vector);
  const sum = radius(radiusA) + radius(radiusB);
  const start = fromA.map((v, i) => v - fromB[i]);
  const delta = toA.map((v, i) => v - fromA[i] - toB[i] + fromB[i]);
  const c = start.reduce((sum, v) => sum + v * v, 0) - sum * sum;
  if (c <= 0) return 0;
  const a = delta.reduce((sum, v) => sum + v * v, 0);
  if (a === 0) return null;
  const b = start.reduce((sum, v, i) => sum + v * delta[i], 0);
  const discriminant = b * b - a * c;
  if (discriminant < 0) return null;
  const t = (-b - Math.sqrt(discriminant)) / a;
  return t >= 0 && t <= 1 ? t : null;
}
export function rayPeerSphere(origin, unitDir, peer, minM = 0, maxM = 1e9) {
  vector(origin);
  vector(unitDir);
  if (
    Math.abs(Math.hypot(...unitDir) - 1) > 1e-8 ||
    !Number.isFinite(minM) ||
    !Number.isFinite(maxM) ||
    minM < 0 ||
    maxM < minM
  )
    throw new Error("Invalid peer ray");
  if (!peer || typeof peer !== "object") throw new Error("Invalid peer");
  vector(peer.position);
  const r = radius(peer.radius);
  const offset = origin.map((v, i) => v - peer.position[i]);
  const c = offset.reduce((sum, v) => sum + v * v, 0) - r * r;
  // A sensor inside a peer is occluded at the origin, never sees through its blind zone.
  if (c <= 0) return minM === 0 ? 0 : null;
  const b = offset.reduce((sum, v, i) => sum + v * unitDir[i], 0),
    d = b * b - c;
  if (d < 0) return null;
  const t = -b - Math.sqrt(d);
  return t >= minM && t <= maxM ? t : null;
}
