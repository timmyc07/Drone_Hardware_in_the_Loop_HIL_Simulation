import { createSimulation } from "./sim.js";
import { isCourseSupported, validateCustomGates } from "./training.js";
export const SCENE_MAX_IMPORT_BYTES = 128 * 1024;
const freeze = (value) => {
  Object.freeze(value);
  for (const child of Object.values(value))
    if (child && typeof child === "object") freeze(child);
  return value;
};
export const DEFAULT_SCENE = freeze({
  name: "預設場景",
  environment: { preset: "lab", timeOfDay: 12, fog: 0, rain: 0 },
  obstacles: [],
  training: "free",
  customGates: [],
});
function exact(value, keys, label) {
  if (
    !value ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value)) ||
    Object.keys(value).length !== keys.length ||
    Object.keys(value).some((key) => !keys.includes(key))
  )
    throw new Error(`Invalid ${label} fields`);
  return value;
}
export function validateEnvironment(value) {
  exact(value, ["preset", "timeOfDay", "fog", "rain"], "environment");
  if (!["lab", "field", "warehouse"].includes(value.preset))
    throw new Error("Invalid environment preset");
  for (const [key, max] of [
    ["timeOfDay", 24],
    ["fog", 1],
    ["rain", 1],
  ])
    if (!Number.isFinite(value[key]) || value[key] < 0 || value[key] > max)
      throw new Error(`Invalid environment ${key}`);
  return { ...value };
}
export function validateScene(value = DEFAULT_SCENE, airframe = "quadrotor") {
  exact(
    value,
    ["name", "environment", "obstacles", "training", "customGates"],
    "scene",
  );
  if (
    typeof value.name !== "string" ||
    !value.name.trim() ||
    value.name.length > 80
  )
    throw new Error("Scene name must contain 1..80 characters");
  if (!isCourseSupported(value.training, airframe))
    throw new Error("Training course is not supported by this airframe");
  const environment = validateEnvironment(value.environment),
    customGates = validateCustomGates(value.customGates);
  if (value.training === "custom" && !customGates.length)
    throw new Error("Custom training needs at least one gate");
  const sim = createSimulation();
  sim.setObstacles(value.obstacles);
  return {
    name: value.name,
    environment,
    obstacles: Array.from(value.obstacles, (box) => ({ ...box })),
    training: value.training,
    customGates,
  };
}
export function parseSceneJson(text, airframe = "quadrotor") {
  if (
    typeof text !== "string" ||
    new TextEncoder().encode(text).length > SCENE_MAX_IMPORT_BYTES
  )
    throw new Error("Scene JSON exceeds 128KiB");
  const artifact = exact(
    JSON.parse(text),
    ["format", "version", "frame", "units", "scene"],
    "scene artifact",
  );
  if (
    artifact.format !== "flight-lab-scene" ||
    artifact.version !== 1 ||
    artifact.frame !== "NED" ||
    artifact.units !== "m"
  )
    throw new Error("Unsupported scene artifact");
  return validateScene(artifact.scene, airframe);
}
export function sceneToJson(scene, airframe = "quadrotor") {
  const text = JSON.stringify(
    {
      format: "flight-lab-scene",
      version: 1,
      frame: "NED",
      units: "m",
      scene: validateScene(scene, airframe),
    },
    null,
    2,
  );
  if (new TextEncoder().encode(text).length > SCENE_MAX_IMPORT_BYTES)
    throw new Error("Scene JSON exceeds 128KiB");
  return text;
}
