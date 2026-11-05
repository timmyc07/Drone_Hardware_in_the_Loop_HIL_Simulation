export const DEFAULT_GROUND_OBSERVER = Object.freeze({
  north: -8,
  east: -8,
  eyeHeightM: 1.6,
  fovDeg: 50,
});

export function validateGroundObserver(value) {
  const keys = Object.keys(DEFAULT_GROUND_OBSERVER);
  if (
    !value ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value)) ||
    Reflect.ownKeys(value).length !== keys.length ||
    Reflect.ownKeys(value).some((key) => !keys.includes(key)) ||
    keys.some((key) => !Number.isFinite(value[key]))
  )
    throw new Error("Ground observer requires north, east, eyeHeightM and fovDeg");
  const { north, east, eyeHeightM, fovDeg } = value;
  if (
    Math.abs(north) > 500 ||
    Math.abs(east) > 500 ||
    eyeHeightM < 0.5 ||
    eyeHeightM > 10 ||
    fovDeg < 15 ||
    fovDeg > 100
  )
    throw new Error("Ground observer position, eye height or FOV is out of range");
  return { north, east, eyeHeightM, fovDeg };
}

export function groundObserverEye(config, terrain) {
  const { north, east, eyeHeightM } = validateGroundObserver(config);
  if (!terrain.contains(north, east))
    throw new Error("Ground observer position is outside terrain coverage");
  return [north, east, -(terrain.sample(north, east).heightM + eyeHeightM)];
}

export function applyGroundObserverCamera(camera, eyeNed, target, fovDeg) {
  camera.position.set(eyeNed[1], -eyeNed[2], -eyeNed[0]);
  if (camera.fov !== fovDeg) {
    camera.fov = fovDeg;
    camera.updateProjectionMatrix();
  }
  camera.lookAt(target);
}
