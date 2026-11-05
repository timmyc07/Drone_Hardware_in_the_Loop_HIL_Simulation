import { validateInspectionTargets, inspectionPanel } from "./inspection.js";
import { validateObstacles } from "./obstacle-geometry.js";
import { validateCustomGates } from "./training.js";
import { DEFAULT_TERRAIN, validateTerrain, createTerrain } from "./terrain.js";
import { validateGeography } from "./geography.js";
export const SCENE_MAX_IMPORT_BYTES = 512 * 1024;
export const SCENE_VERSION = 7;
const freeze = (value) => {
  Object.freeze(value);
  for (const child of Object.values(value))
    if (child && typeof child === "object") freeze(child);
  return value;
};
export const DEFAULT_SCENE = freeze({
  name: "預設場景",
  environment: { preset: "lab", timeOfDay: 12, fog: 0, rain: 0 },
  terrain: DEFAULT_TERRAIN,
  geography: null,
  obstacles: [],
  customGates: [],
  inspectionTargets: [],
  nistLane: false,
});
function exact(value, keys, label) {
  if (
    !value ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value)) ||
    Reflect.ownKeys(value).length !== keys.length ||
    Reflect.ownKeys(value).some((key) => !keys.includes(key))
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
export function validateScene(value = DEFAULT_SCENE) {
  exact(
    value,
    [
      "name",
      "environment",
      "terrain",
      "geography",
      "obstacles",
      "customGates",
      "inspectionTargets",
      "nistLane",
    ],
    "scene",
  );
  if (
    typeof value.name !== "string" ||
    !value.name.trim() ||
    value.name.length > 80
  )
    throw new Error("Scene name must contain 1..80 characters");
  if (!Object.hasOwn(value, "terrain"))
    throw new Error("Scene terrain is required");
  const terrain = validateTerrain(value.terrain);
  const geography =
    value.geography === null ? null : validateGeography(value.geography);
  if ((terrain.kind === "dem") !== (geography !== null))
    throw new Error(
      "DEM terrain requires its geographic origin and source metadata",
    );
  if (typeof value.nistLane !== "boolean" || (value.nistLane && (terrain.kind !== "flat" || geography !== null)))
    throw new Error("NIST 場地需要零高度平地且不得設定地理參考");
  const surface = createTerrain(terrain);
  if (
    geography &&
    (!surface.contains(0, 0) || Math.abs(surface.sample(0, 0).heightM) > 1e-6)
  )
    throw new Error(
      "DEM home must be covered and use its origin elevation as zero",
    );
  const environment = validateEnvironment(value.environment),
    customGates = validateCustomGates(value.customGates);
  const obstacles = validateObstacles(value.obstacles);
  const inspectionTargets = validateInspectionTargets(
    value.inspectionTargets,
    obstacles,
  );
  if (
    terrain.kind === "dem" &&
    inspectionTargets.some((target) =>
      inspectionPanel(target, obstacles).cornersNed.some(
        (point) => !surface.contains(point[0], point[1]),
      ),
    )
  )
    throw new Error("Inspection panels must be covered by the DEM");
  return {
    name: value.name,
    environment,
    terrain,
    geography,
    obstacles,
    customGates,
    inspectionTargets,
    nistLane: value.nistLane,
  };
}
export function parseSceneJson(text) {
  if (
    typeof text !== "string" ||
    new TextEncoder().encode(text).length > SCENE_MAX_IMPORT_BYTES
  )
    throw new Error("Scene JSON exceeds 512KiB");
  const artifact = exact(
    JSON.parse(text),
    ["format", "version", "frame", "units", "scene"],
    "scene artifact",
  );
  if (
    artifact.format !== "flight-lab-scene" ||
    artifact.version !== SCENE_VERSION ||
    artifact.frame !== "NED" ||
    artifact.units !== "m"
  )
    throw new Error("Unsupported scene artifact");
  return validateScene(artifact.scene);
}
export function sceneToJson(scene) {
  const text = JSON.stringify(
    {
      format: "flight-lab-scene",
      version: SCENE_VERSION,
      frame: "NED",
      units: "m",
      scene: validateScene(scene),
    },
    null,
    2,
  );
  if (new TextEncoder().encode(text).length > SCENE_MAX_IMPORT_BYTES)
    throw new Error("Scene JSON exceeds 512KiB");
  return text;
}
