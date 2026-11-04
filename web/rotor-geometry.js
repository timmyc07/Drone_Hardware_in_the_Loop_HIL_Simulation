// FRD geometry, positive lift along -Z. Internal clockwise order is not an autopilot output map.
// Official geometry concepts: github.com/PX4/PX4-Autopilot/tree/v1.17.0/src/modules/control_allocator
// and github.com/ArduPilot/ardupilot/blob/Copter-4.7.2/libraries/SITL/SIM_Frame.cpp.
export const AIRFRAMES = Object.freeze([
  "quadrotor",
  "fixedwing",
  "vtol",
  "hexarotor",
  "octorotor",
]);
export const MULTIROTOR_AIRFRAMES = Object.freeze([
  "quadrotor",
  "hexarotor",
  "octorotor",
]);
const unitLever = 1 / Math.SQRT2;
function rotor(index, x, y, name = `M${index + 1}`) {
  const yawTorqueSign = index % 2 === 0 ? 1 : -1;
  return Object.freeze({
    index,
    name,
    positionFrd: Object.freeze([x, y, 0]),
    spin: yawTorqueSign === 1 ? "ccw" : "cw",
    rollScale: -y / unitLever,
    pitchScale: x / unitLever,
    yawTorqueSign,
  });
}
const quad = Object.freeze(
  [
    [1, 1, "FR"],
    [-1, 1, "RR"],
    [-1, -1, "RL"],
    [1, -1, "FL"],
  ].map(([x, y, name], i) =>
    Object.freeze({
      ...rotor(i, x * unitLever, y * unitLever, name),
      rollScale: -y,
      pitchScale: x,
    }),
  ),
);
const h = Math.sqrt(3) / 2;
const hexa = Object.freeze(
  [
    [h, 0.5],
    [0, 1],
    [-h, 0.5],
    [-h, -0.5],
    [0, -1],
    [h, -0.5],
  ].map(([x, y], i) => rotor(i, x, y)),
);
const c = Math.cos(Math.PI / 8),
  s = Math.sin(Math.PI / 8);
const octo = Object.freeze(
  [
    [c, s],
    [s, c],
    [-s, c],
    [-c, s],
    [-c, -s],
    [-s, -c],
    [s, -c],
    [c, -s],
  ].map(([x, y], i) => rotor(i, x, y)),
);
// Fixed-wing entries describe reserved channels, not four physical lift rotors.
export const ROTOR_LAYOUTS = Object.freeze({
  quadrotor: quad,
  fixedwing: quad,
  vtol: quad,
  hexarotor: hexa,
  octorotor: octo,
});
export function rotorCount(airframe) {
  if (typeof airframe !== "string" || !Object.hasOwn(ROTOR_LAYOUTS, airframe))
    throw Error("Unknown rotor airframe");
  return ROTOR_LAYOUTS[airframe].length;
}
export function getRotorLayout(airframe, arm = 1) {
  const count = rotorCount(airframe);
  if (!Number.isFinite(arm) || arm <= 0 || arm > 2)
    throw Error("Rotor arm must be positive and at most 2 meters");
  const lever = arm / Math.SQRT2;
  return Object.freeze(
    ROTOR_LAYOUTS[airframe].map((item) =>
      Object.freeze({
        ...item,
        positionFrd: Object.freeze(
          count === 4
            ? [item.pitchScale * lever, -item.rollScale * lever, 0]
            : item.positionFrd.map((v) => v * arm),
        ),
      }),
    ),
  );
}
