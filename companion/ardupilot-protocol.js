export const ARDUPILOT_SOURCE_SHA = "fb773567896014da007bca58358704d4a0d8516f";
export const ARDUPILOT_VERSION = "4.7.2";
export const ARDUPILOT_DT = 0.001;
export const ARDUPILOT_OUTPUT_MAP = Object.freeze([0, 3, 1, 2]);
// Copter-4.7.2 native little-endian JSON backend servo_packet_16; no CRC is present.
export function decodeServoPacket(buffer) {
  if (
    !Buffer.isBuffer(buffer) ||
    buffer.length !== 40 ||
    buffer.readUInt16LE(0) !== 18458
  )
    throw new Error("Expected 40-byte ArduPilot JSON servo packet, magic18458");
  const frameRate = buffer.readUInt16LE(2),
    frameCount = buffer.readUInt32LE(4),
    pwm = Array.from({ length: 16 }, (_, i) => buffer.readUInt16LE(8 + i * 2));
  if (![1000, 1200].includes(frameRate))
    throw new Error(
      `Unsupported ArduPilot JSON rate ${frameRate}; expected1000Hz (1200Hz initial frame allowed)`,
    );
  if (pwm.some((value) => value !== 0 && (value < 800 || value > 2200)))
    throw new Error("Invalid ArduPilot PWM");
  return { frameRate, frameCount, pwm };
}
export function mapServoSpeeds(packet, armed) {
  if (
    !packet ||
    !Array.isArray(packet.pwm) ||
    packet.pwm.length !== 16 ||
    !Array.from(packet.pwm).every(
      (value) => Number.isInteger(value) && value >= 0 && value <= 65535,
    )
  )
    throw new Error("Invalid servo PWM vector");
  if (typeof armed !== "boolean")
    throw new Error("Armed state must be boolean");
  return ARDUPILOT_OUTPUT_MAP.map((i) => {
    const value = packet.pwm[i];
    if (
      !Number.isFinite(value) ||
      !Number.isInteger(value) ||
      (armed && (value < 1000 || value > 2000))
    )
      throw new Error("Motor PWM must be1000..2000 while armed");
    return armed ? (value - 1000) / 1000 : 0;
  });
}
export function physicsJson(truth, battery) {
  const vector = (value, length) =>
    Array.isArray(value) &&
    value.length === length &&
    Array.from(value).every(Number.isFinite);
  if (
    !truth ||
    !Number.isSafeInteger(truth.timeUs) ||
    truth.timeUs < 0 ||
    !vector(truth.positionNed, 3) ||
    !vector(truth.velocityNed, 3) ||
    !vector(truth.quaternionWxyz, 4) ||
    Math.abs(Math.hypot(...truth.quaternionWxyz) - 1) > 1e-6 ||
    !vector(truth.ratesFrd, 3) ||
    !vector(truth.specificForceFrd, 3)
  )
    throw new Error("Invalid ArduPilot physics truth");
  const data = {
    timestamp: truth.timeUs / 1e6,
    imu: { gyro: truth.ratesFrd, accel_body: truth.specificForceFrd },
    position: truth.positionNed,
    velocity: truth.velocityNed,
    quaternion: truth.quaternionWxyz,
    no_time_sync: false,
    no_lockstep: false,
    rc: {
      rc_1: 1500,
      rc_2: 1500,
      rc_3: 1000,
      rc_4: 1500,
      rc_5: 1500,
      rc_6: 1500,
      rc_7: 1500,
      rc_8: 1500,
    },
  };
  if (battery) {
    if (
      !Number.isFinite(battery.voltage) ||
      battery.voltage < 0 ||
      !Number.isFinite(battery.current) ||
      battery.current < 0
    )
      throw new Error("Invalid simulated battery");
    data.battery = { voltage: battery.voltage, current: battery.current };
  }
  return Buffer.from("\n" + JSON.stringify(data) + "\n");
}
export function autopilotIdentity(message) {
  const number = message.flightSwVersion;
  if (
    !Number.isSafeInteger(number) ||
    number < 0 ||
    number > 0xffffffff ||
    !Array.isArray(message.flightCustomVersion) ||
    message.flightCustomVersion.length !== 8 ||
    !Array.from(message.flightCustomVersion).every(
      (value) => Number.isInteger(value) && value >= 0 && value <= 255,
    )
  )
    throw new Error("Malformed AUTOPILOT_VERSION");
  const version = `${(number >>> 24) & 255}.${(number >>> 16) & 255}.${(number >>> 8) & 255}`;
  const reportedGitHash = Buffer.from(message.flightCustomVersion)
    .toString("ascii")
    .replace(/\0+$/, "");
  return {
    version,
    reportedGitHash,
    verified:
      version === ARDUPILOT_VERSION &&
      reportedGitHash === ARDUPILOT_SOURCE_SHA.slice(0, 8),
  };
}
