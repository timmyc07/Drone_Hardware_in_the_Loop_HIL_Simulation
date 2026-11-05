import { defaultWorldConfig, validateWorldConfig } from './world.js';
import { DEFAULT_SCENE } from './scene-data.js';
import { DEFAULT_GIMBAL_SETTINGS } from './gimbal.js';

export function createNistTrainingWorld(vehicleSpec) {
  const world = structuredClone(defaultWorldConfig(
    { ...vehicleSpec.config, startOnGround: true, wind: 0, windDirection: 0 },
    {
      sensors: {
        ...vehicleSpec.setup.sensors,
        lidarRays: 12,
        lidarChannels: 1,
        lidarVerticalMinDeg: 0,
        lidarVerticalMaxDeg: 0,
        radarAzimuthRays: 1,
        radarElevationRays: 1,
      },
      mode: 'position',
      training: 'free',
      mission: [],
      gimbal: DEFAULT_GIMBAL_SETTINGS,
    },
    { ...DEFAULT_SCENE, name: 'NIST Level 1 開放航道', nistLane: true },
  ));
  world.vehicles[0].id = vehicleSpec.id;
  world.vehicles[0].name = vehicleSpec.name;
  return validateWorldConfig(world);
}
