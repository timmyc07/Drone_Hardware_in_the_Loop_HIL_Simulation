// NED/FRD aerodynamic equations reference (no source copied):
// https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/src/liftdrag_plugin/README.md
// Educational generic wing coefficients, not a calibrated aircraft/propeller model.
export const WING = Object.freeze({
  cl0: 0.2,
  clAlpha: 4.5,
  clElevator: 0.2,
  stallAlpha: 0.28,
  cd0: 0.035,
  efficiency: 0.8,
  cm0: 0.025,
  cmAlpha: -0.6,
  cmElevator: 1,
  rollAileron: 0.18,
  yawRudder: 0.1,
  surfaceLimit: 0.45,
});
const clamp = (v, low, high) => Math.max(low, Math.min(high, v));
export function aerodynamicForces(
  config,
  velocityBody,
  rates = [0, 0, 0],
  surfaces = [0, 0, 0],
  altitude = 0,
) {
  const [u, v, w] = velocityBody,
    airspeedMps = Math.hypot(u, v, w);
  const alphaRad =
    Math.hypot(u, w) > 1e-8 * Math.max(1, airspeedMps) ? Math.atan2(w, u) : 0;
  const betaRad =
    airspeedMps > 1e-8 ? Math.asin(clamp(v / airspeedMps, -1, 1)) : 0;
  const density = 1.225 * Math.exp(-clamp(altitude, -500, 11000) / 8500);
  const dynamicPressurePa = 0.5 * density * airspeedMps ** 2;
  const aspect = config.wingSpan ** 2 / config.wingArea;
  const blend =
    1 /
    (1 +
      Math.exp(clamp(-60 * (Math.abs(alphaRad) - WING.stallAlpha), -40, 40)));
  const linearLift =
    WING.cl0 + WING.clAlpha * alphaRad + WING.clElevator * surfaces[1];
  // Spanwise flow cannot provide the same wing lift as flow over the chord.
  const cl =
    ((1 - blend) * linearLift + blend * 1.1 * Math.sin(2 * alphaRad)) *
    Math.cos(betaRad) ** 2;
  const cd =
    WING.cd0 +
    cl ** 2 / (Math.PI * WING.efficiency * aspect) +
    1.4 * blend * Math.sin(alphaRad) ** 2 +
    0.3 * Math.sin(betaRad) ** 2;
  const liftN = dynamicPressurePa * config.wingArea * cl,
    dragN = dynamicPressurePa * config.wingArea * cd;
  const pHat = (rates[0] * config.wingSpan) / (2 * Math.max(1, airspeedMps));
  const qHat = (rates[1] * config.wingChord) / (2 * Math.max(1, airspeedMps));
  const rHat = (rates[2] * config.wingSpan) / (2 * Math.max(1, airspeedMps));
  const pitchStatic =
    (1 - blend) * (WING.cm0 + WING.cmAlpha * alphaRad) -
    blend * 0.12 * Math.sin(2 * alphaRad);
  const coefficients = [
    -0.08 * betaRad - 0.5 * pHat + 0.08 * rHat + WING.rollAileron * surfaces[0],
    pitchStatic - 8 * qHat + WING.cmElevator * surfaces[1],
    0.12 * betaRad -
      0.3 * rHat -
      0.01 * surfaces[0] +
      WING.yawRudder * surfaces[2],
  ];
  const direction =
    airspeedMps > 1e-8
      ? velocityBody.map((value) => value / airspeedMps)
      : [1, 0, 0];
  const forceBody = direction.map((value) => -dragN * value);
  forceBody[0] += liftN * Math.sin(alphaRad);
  const sideForce =
    dynamicPressurePa *
    config.wingArea *
    (-0.8 * betaRad + 0.12 * surfaces[2]) *
    Math.cos(betaRad);
  const sideDirection = [
    -Math.cos(alphaRad) * Math.sin(betaRad),
    Math.cos(betaRad),
    -Math.sin(alphaRad) * Math.sin(betaRad),
  ];
  for (let i = 0; i < 3; i++) forceBody[i] += sideForce * sideDirection[i];
  forceBody[2] -= liftN * Math.cos(alphaRad);
  return {
    forceBody,
    momentBody: coefficients.map(
      (c, i) =>
        c *
        dynamicPressurePa *
        config.wingArea *
        (i === 1 ? config.wingChord : config.wingSpan),
    ),
    airspeedMps,
    alphaRad,
    betaRad,
    dynamicPressurePa,
    liftN,
    dragN,
    loadFactor: Math.abs(liftN) / (config.mass * 9.80665),
    stalled: airspeedMps > 1 && (Math.abs(alphaRad) > WING.stallAlpha || u < 0),
    pHat,
    qHat,
    rHat,
    pitchStatic,
  };
}
export function stallSpeed(config, altitude = 0) {
  return Math.sqrt(
    (2 * config.mass * 9.80665) /
      (1.225 * Math.exp(-altitude / 8500) * config.wingArea * 1.15),
  );
}
export function wingTrim(
  config,
  airspeed = config.cruiseAirspeed,
  altitude = 30,
) {
  function at(alpha) {
    const elevator = -(WING.cm0 + WING.cmAlpha * alpha) / WING.cmElevator;
    const aero = aerodynamicForces(
      config,
      [airspeed * Math.cos(alpha), 0, airspeed * Math.sin(alpha)],
      [0, 0, 0],
      [0, elevator, 0],
      altitude,
    );
    const thrust = aero.dragN / Math.cos(alpha);
    return {
      alpha,
      elevator,
      thrust,
      aero,
      residual: aero.liftN + thrust * Math.sin(alpha) - config.mass * 9.80665,
    };
  }
  let low = -0.15,
    high = 0.22;
  if (at(low).residual > 0 || at(high).residual < 0)
    throw new Error(
      "Wing cannot trim at cruiseAirspeed; increase speed/wing area or reduce mass",
    );
  for (let i = 0; i < 50; i++) {
    const mid = (low + high) / 2;
    if (at(mid).residual > 0) high = mid;
    else low = mid;
  }
  const result = at((low + high) / 2);
  if (result.thrust >= config.maxPusherThrust)
    throw new Error("Pusher thrust is insufficient for cruise trim");
  return result;
}
