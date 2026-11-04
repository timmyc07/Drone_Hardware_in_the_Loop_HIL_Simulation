export function createFlightApi(request, onState) {
  if (typeof request !== "function" || typeof onState !== "function")
    throw new TypeError("request and onState must be functions");
  const send = async (command) =>
    structuredClone(await request(structuredClone(command)));
  const id = (value) => {
    if (
      typeof value !== "string" ||
      !/^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(value)
    )
      throw new TypeError("vehicleId is required");
    return value;
  };
  const vehicleRead = (type) => (vehicleId) =>
    send({ type, vehicleId: id(vehicleId) });
  const vehicleValue = (type) => (vehicleId, value) =>
    send({ type, vehicleId: id(vehicleId), value });
  const vehicleAction = (type) => (vehicleId) =>
    send({ type, vehicleId: id(vehicleId) });
  return Object.freeze({
    version: "2.0",
    getWorldState: () => send({ type: "inspectWorld" }),
    getState: vehicleRead("inspect"),
    getSensors: vehicleRead("inspectSensors"),
    getScene: () => send({ type: "inspectScene" }),
    setScene: (value) => send({ type: "scene", value }),
    exportLidarPointCloud: (vehicleId, format = "pcd") =>
      send({
        type: "pointCloudExport",
        vehicleId: id(vehicleId),
        value: format,
      }),
    getGimbal: vehicleRead("inspectGimbal"),
    setGimbal: vehicleValue("gimbal"),
    getCameraConfig: () => send({ type: "getCameraConfig" }),
    setCameraConfig: (value) => send({ type: "setCameraConfig", value }),
    captureCamera: vehicleRead("captureCamera"),
    setTargets: vehicleValue("targets"),
    setMode: vehicleValue("mode"),
    takeoff: vehicleAction("takeoff"),
    abortTakeoff: vehicleAction("abortTakeoff"),
    setTransition: vehicleValue("transition"),
    setPusherFault: (vehicleId, efficiency) =>
      send({
        type: "fault",
        vehicleId: id(vehicleId),
        fault: "pusher",
        value: { efficiency },
      }),
    setPositionTarget: vehicleValue("position"),
    setMission: vehicleValue("mission"),
    play: () => send({ type: "play" }),
    pause: () => send({ type: "pause" }),
    resetWorld: (value) => send({ type: "resetWorld", value }),
    setSensorConfig: vehicleValue("sensorConfig"),
    setSensorFault: (vehicleId, sensor, failed) =>
      send({
        type: "sensorFault",
        vehicleId: id(vehicleId),
        value: { sensor, failed },
      }),
    exportRecording: () => send({ type: "apiExport" }),
    subscribe: (callback) => {
      if (typeof callback !== "function")
        throw new TypeError("callback must be a function");
      return onState((state) => callback(structuredClone(state)));
    },
  });
}
