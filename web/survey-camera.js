import { validateCameraConfig } from "./camera.js";

const fail = (message) => {
  throw new Error(`測繪相機：${message}`);
};

export function calculateSurveyCamera(cameraConfig, options) {
  const keys = ["altitudeM", "planeAltitudeM", "frontOverlapPct", "sideOverlapPct"];
  if (
    !options || typeof options !== "object" ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(options)) ||
    Reflect.ownKeys(options).length !== keys.length ||
    !keys.every((key) => Object.hasOwn(options, key) && Number.isFinite(options[key]))
  ) fail("參數須完整且只包含高度、平面高度及前向／側向重疊率的有限數值");
  const { altitudeM, planeAltitudeM, frontOverlapPct, sideOverlapPct } = options;
  if ([altitudeM, planeAltitudeM].some((value) => value < -1000 || value > 1500))
    fail("高度與平面高度須在 -1000 至 1500 公尺內");
  if ([frontOverlapPct, sideOverlapPct].some((value) => value < 0 || value > 95))
    fail("重疊率須在 0 至 95% 內");
  let config;
  try {
    config = validateCameraConfig(cameraConfig);
  } catch {
    fail("相機解析度、視野或裁切距離無效");
  }
  const heightAbovePlaneM = altitudeM - planeAltitudeM;
  if (heightAbovePlaneM <= config.nearM || heightAbovePlaneM >= config.farM)
    fail("相機離平面高度須嚴格介於近、遠裁切距離之間");
  const footprintAcrossM = 2 * heightAbovePlaneM * Math.tan(config.horizontalFovDeg * Math.PI / 360),
    footprintAlongM = footprintAcrossM * config.height / config.width,
    gsdMPerPixel = footprintAcrossM / config.width,
    spacingM = footprintAcrossM * (1 - sideOverlapPct / 100),
    triggerDistanceM = footprintAlongM * (1 - frontOverlapPct / 100);
  if (spacingM < 0.5 || spacingM > 200)
    fail("側向航線間距須在 0.5 至 200 公尺內，請調整高度或側向重疊率");
  if (triggerDistanceM < 0.25 || triggerDistanceM > 500)
    fail("前向拍攝間距須在 0.25 至 500 公尺內，請調整高度或前向重疊率");
  return {
    cameraConfig: config, altitudeM, planeAltitudeM, frontOverlapPct, sideOverlapPct,
    heightAbovePlaneM, footprintAcrossM, footprintAlongM, gsdMPerPixel,
    spacingM, triggerDistanceM,
  };
}

export function surveyFootprint(center, angleDeg, plan) {
  if (
    !center || !Number.isFinite(center.north) || !Number.isFinite(center.east) ||
    !Number.isFinite(angleDeg) || !plan ||
    ![plan.footprintAlongM, plan.footprintAcrossM].every((value) => Number.isFinite(value) && value > 0)
  ) fail("足跡中心、角度或尺寸無效");
  const angle = (angleDeg % 360) * Math.PI / 180,
    cos = Math.cos(angle), sin = Math.sin(angle),
    along = plan.footprintAlongM / 2, across = plan.footprintAcrossM / 2;
  return [[along, -across], [along, across], [-along, across], [-along, -across]]
    .map(([a, b]) => ({
      north: center.north + a * cos - b * sin,
      east: center.east + a * sin + b * cos,
    }));
}
