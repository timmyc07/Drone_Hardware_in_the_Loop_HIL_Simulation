export function createFlightApi(request, onState) {
  if (typeof request !== "function" || typeof onState !== "function") {
    throw new TypeError("request and onState must be functions");
  }
  const send = async (command) =>
    structuredClone(await request(structuredClone(command)));
  const withValue = (type) => (value) => send({ type, value });
  return Object.freeze({
    version: "1.4",
    getState: () => send({ type: "inspect" }),
    getSensors: () => send({ type: "inspectSensors" }),
    exportLidarPointCloud: (format = "pcd") =>
      send({ type: "pointCloudExport", value: format }),
    getCameraConfig: () => send({ type: "getCameraConfig" }),
    setCameraConfig: withValue("setCameraConfig"),
    captureCamera: () => send({ type: "captureCamera" }),
    setTargets: withValue("targets"),
    setMode: withValue("mode"),
    takeoff: () => send({ type: "takeoff" }),
    abortTakeoff: () => send({ type: "abortTakeoff" }),
    setTransition: withValue("transition"),
    setPusherFault: (efficiency) =>
      send({ type: "fault", fault: "pusher", value: { efficiency } }),
    setPositionTarget: withValue("position"),
    setMission: withValue("mission"),
    play: () => send({ type: "play" }),
    pause: () => send({ type: "pause" }),
    reset: async (options = {}) => {
      if (
        !options ||
        typeof options !== "object" ||
        Array.isArray(options) ||
        Reflect.ownKeys(options).some(
          (key) => !["config", "scenario", "setup"].includes(key),
        )
      ) {
        throw new TypeError("Invalid reset options");
      }
      return send({ type: "reset", ...options });
    },
    setSensorConfig: withValue("sensorConfig"),
    setSensorFault: (sensor, failed) =>
      send({ type: "sensorFault", value: { sensor, failed } }),
    exportRecording: () => send({ type: "apiExport" }),
    subscribe: (callback) => {
      if (typeof callback !== "function")
        throw new TypeError("callback must be a function");
      return onState((state) => callback(structuredClone(state)));
    },
  });
}
