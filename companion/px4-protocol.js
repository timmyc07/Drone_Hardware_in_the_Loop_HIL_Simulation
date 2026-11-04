import { magneticFieldNed } from "./px4-wmm.js";
import { common, MavLinkProtocolV2 } from "node-mavlink";
import { bodyToNedMatrix } from "../web/sim.js";
// Pinned PX4 v1.17.0 10016_none_iris: outputs FR, RL, FL, RR; positive KM gives positive body yaw.
export const OUTPUT_MAP = [0, 3, 1, 2];
export function mapActuators(message) {
  if (
    typeof message.timeUsec !== "bigint" ||
    message.timeUsec < 0n ||
    message.timeUsec > BigInt(Number.MAX_SAFE_INTEGER) ||
    !Number.isInteger(message.mode) ||
    message.mode < 0 ||
    message.mode > 255 ||
    typeof message.flags !== "bigint" ||
    message.flags < 0n ||
    message.flags > BigInt(Number.MAX_SAFE_INTEGER) ||
    !(message.flags & 1n) ||
    !Array.isArray(message.controls) ||
    message.controls.length !== 16 ||
    !Array.from(message.controls).every(Number.isFinite)
  )
    throw new Error("Invalid HIL_ACTUATOR_CONTROLS");
  const armed = Boolean(message.mode & 128);
  const rotorSpeedCommands = OUTPUT_MAP.map((i) => {
    const value = message.controls[i];
    if (armed && (value < 0 || value > 1))
      throw new Error("PX4 motor output outside [0,1]");
    return armed ? value : 0;
  });
  return {
    armed,
    rotorSpeedCommands,
    rawTimeUs: Number(message.timeUsec),
    rawControls: [...message.controls],
    mode: message.mode,
    flags: Number(message.flags),
  };
}
export const DEFAULT_ORIGIN = Object.freeze({
  latitude: 25.033,
  longitude: 121.5654,
  altitudeMsl: 20,
});
export function sensorMessages(truth, origin = DEFAULT_ORIGIN, seed = 42) {
  const vector = (value, length) =>
    Array.isArray(value) &&
    value.length === length &&
    Array.from(value).every(Number.isFinite);
  if (
    !truth ||
    !Number.isSafeInteger(truth.timeUs) ||
    truth.timeUs < 0 ||
    !Number.isSafeInteger(truth.timeUs + 1_000_000) ||
    !vector(truth.positionNed, 3) ||
    !vector(truth.velocityNed, 3) ||
    !vector(truth.quaternionWxyz, 4) ||
    Math.abs(Math.hypot(...truth.quaternionWxyz) - 1) > 1e-6 ||
    !vector(truth.ratesFrd, 3) ||
    !vector(truth.specificForceFrd, 3) ||
    !origin ||
    !Number.isFinite(origin.latitude) ||
    Math.abs(origin.latitude) > 85 ||
    !Number.isFinite(origin.longitude) ||
    Math.abs(origin.longitude) > 180 ||
    !Number.isFinite(origin.altitudeMsl) ||
    !Number.isSafeInteger(seed)
  )
    throw new Error("Invalid HIL sensor truth, origin or seed");
  const t = BigInt(truth.timeUs + 1_000_000);
  const r = bodyToNedMatrix(truth.quaternionWxyz);
  const fieldNed = magneticFieldNed(origin.latitude, origin.longitude);
  const mag = [0, 1, 2].map(
    (i) => r[i] * fieldNed[0] + r[3 + i] * fieldNed[1] + r[6 + i] * fieldNed[2],
  );
  // Deterministic small sensor noise prevents PX4 constant-value sensor-stuck detection on an ideal stationary plant.
  const noise = (axis) => {
    let x = (truth.timeUs + 1 + seed + axis * 2654435761) >>> 0;
    x ^= x << 13;
    x ^= x >>> 17;
    x ^= x << 5;
    return ((x >>> 0) / 4294967296 - 0.5) * Math.sqrt(12);
  };
  const altitude = origin.altitudeMsl - truth.positionNed[2];
  const sensor = Object.assign(new common.HilSensor(), {
    timeUsec: t,
    xacc: truth.specificForceFrd[0] + 0.01 * noise(1),
    yacc: truth.specificForceFrd[1] + 0.01 * noise(2),
    zacc: truth.specificForceFrd[2] + 0.01 * noise(3),
    xgyro: truth.ratesFrd[0] + 0.001 * noise(4),
    ygyro: truth.ratesFrd[1] + 0.001 * noise(5),
    zgyro: truth.ratesFrd[2] + 0.001 * noise(6),
    xmag: mag[0] + 0.0001 * noise(7),
    ymag: mag[1] + 0.0001 * noise(8),
    zmag: mag[2] + 0.0001 * noise(9),
    absPressure:
      1013.25 * (1 - (0.0065 * altitude) / 288.15) ** 5.25588 +
      0.027 * noise(10),
    diffPressure: 0,
    pressureAlt: altitude,
    temperature: 15 - 0.0065 * altitude,
    fieldsUpdated: 7167,
    id: 0,
  });
  const { latitude, longitude } = projectNed(
    truth.positionNed[0],
    truth.positionNed[1],
    origin,
  );
  const [vn, ve, vd] = truth.velocityNed;
  if (
    Math.abs(latitude) > 90 ||
    Math.abs(longitude) > 180 ||
    altitude < -1000 ||
    altitude > 20000 ||
    [vn, ve, vd].some((value) => value * 100 < -32768 || value * 100 > 32767) ||
    Math.hypot(vn, ve) * 100 > 65535
  )
    throw new Error("HIL GPS truth exceeds protocol position/velocity bounds");
  // Ideal synthetic 3D GPS: declared quality and satellite count are simulator assumptions, not a constellation model.
  const gps = Object.assign(new common.HilGps(), {
    timeUsec: t,
    fixType: 3,
    lat: Math.round(latitude * 1e7),
    lon: Math.round(longitude * 1e7),
    alt: Math.round(altitude * 1000),
    eph: 100,
    epv: 150,
    vel: Math.round(Math.hypot(vn, ve) * 100),
    vn: Math.round(vn * 100),
    ve: Math.round(ve * 100),
    vd: Math.round(vd * 100),
    cog:
      Math.hypot(vn, ve) > 0.01
        ? Math.round(
            (((Math.atan2(ve, vn) * 180) / Math.PI + 360) % 360) * 100,
          ) % 36000
        : 65535,
    satellitesVisible: 10,
    id: 0,
    yaw: 0,
  });
  return { sensor, gps };
}
export function encoder() {
  const protocol = new MavLinkProtocolV2(1, 200);
  let sequence = 0;
  return (message) => protocol.serialize(message, sequence++ % 256);
}

export function projectNed(north, east, origin = DEFAULT_ORIGIN) {
  if (![north, east].every(Number.isFinite))
    throw new Error("Invalid NED coordinates");
  const latitude = origin.latitude + ((north / 6378137) * 180) / Math.PI;
  const longitude =
    origin.longitude +
    ((east / (6378137 * Math.cos((origin.latitude * Math.PI) / 180))) * 180) /
      Math.PI;
  return { latitude, longitude };
}
