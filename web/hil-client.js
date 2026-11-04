export function isCompanionOrigin(location) {
  return (
    location.protocol === "http:" &&
    ["127.0.0.1", "localhost"].includes(location.hostname) &&
    location.port === "8787"
  );
}
export function createHilClient({
  location,
  fetch,
  WebSocket,
  onState,
  onDisconnect,
}) {
  if (!isCompanionOrigin(location))
    throw new Error("請使用本機 companion 網址。");
  let token,
    socket,
    serial = 0;
  const pending = new Map();
  const rejectPending = (message) => {
    for (const item of pending.values()) {
      clearTimeout(item.timer);
      item.reject(new Error(message));
    }
    pending.clear();
  };
  async function connect() {
    const response = await fetch("/api/bootstrap", { cache: "no-store" });
    if (!response.ok) throw new Error("無法連接本機服務。");
    const initial = await response.json();
    if (
      typeof initial.token !== "string" ||
      !initial.token ||
      initial.wsPath !== "/bridge"
    )
      throw new Error("本機服務回應無效。");
    token = initial.token;
    const url = new URL(initial.wsPath, location.href);
    url.protocol = "ws:";
    url.searchParams.set("token", token);
    if (initial.state) onState(initial.state);
    socket = new WebSocket(url.href);
    let rejectOpen;
    socket.onmessage = (event) => {
      let data;
      try {
        data = JSON.parse(event.data);
      } catch {
        return;
      }
      if (
        data?.type === "state" &&
        data.value &&
        typeof data.value === "object"
      )
        onState(data.value);
      else if (data?.type === "reply" && Number.isSafeInteger(data.id)) {
        const item = pending.get(data.id);
        if (!item) return;
        pending.delete(data.id);
        clearTimeout(item.timer);
        if (data.ok === true) item.resolve(data.value);
        else
          item.reject(
            new Error(
              typeof data.error === "string"
                ? data.error
                : "本機服務拒絕操作。",
            ),
          );
      }
    };
    socket.onclose = () => {
      token = undefined;
      rejectOpen?.(new Error("本機連線已中斷。"));
      rejectPending("本機連線已中斷。");
      onDisconnect?.();
    };
    await new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        socket.close();
        reject(new Error("連線逾時。"));
      }, 10000);
      rejectOpen = (error) => {
        clearTimeout(timer);
        reject(error);
      };
      socket.onopen = () => {
        clearTimeout(timer);
        rejectOpen = undefined;
        resolve();
      };
      socket.onerror = () => {
        clearTimeout(timer);
        rejectOpen = undefined;
        reject(new Error("無法建立本機連線。"));
      };
    });
  }
  function request(type, value) {
    if (!socket || socket.readyState !== WebSocket.OPEN)
      return Promise.reject(new Error("本機服務尚未連線。"));
    const id = ++serial;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        pending.delete(id);
        reject(new Error("本機服務回應逾時。"));
      }, 25000);
      pending.set(id, { resolve, reject, timer });
      try {
        socket.send(
          JSON.stringify(
            value === undefined ? { id, type } : { id, type, value },
          ),
        );
      } catch (error) {
        clearTimeout(timer);
        pending.delete(id);
        reject(error);
      }
    });
  }
  async function recording() {
    if (!token) throw new Error("本機服務尚未連線。");
    const response = await fetch("/api/recording", {
      headers: { "X-Flight-Token": token },
      cache: "no-store",
    });
    if (!response.ok) throw new Error("無法取得 SITL 紀錄。");
    return response.text();
  }
  return {
    connect,
    request,
    configureGroundStation(value) {
      if (!validGroundStationPort(value?.targetPort))
        return Promise.reject(
          new Error(
            "地面站 UDP 埠須介於 1024 至 65535，且不可使用飛控保留埠。",
          ),
        );
      return request("groundStation", value);
    },
    recording,
    close() {
      socket?.close();
      rejectPending("本機連線已關閉。");
      token = undefined;
    },
  };
}

export function validGroundStationPort(port) {
  return (
    Number.isInteger(port) &&
    port >= 1024 &&
    port <= 65535 &&
    ![4560, 5760, 9002, 14560, 14561].includes(port)
  );
}

export function hilWebControlAllowed(state) {
  return state?.groundStation?.owner === "web";
}

export function hilFlightReadiness(state) {
  if (!["ardupilot", "px4"].includes(state?.controller?.kind)) return [];
  const controller = state.controller;
  if (controller.kind === "px4")
    return [
      ...(!controller.verified ? ["飛控身分"] : []),
      ...(Object.hasOwn(controller, "parametersVerified") &&
      !controller.parametersVerified
        ? ["參數"]
        : []),
      ...(Object.hasOwn(controller, "armedKnown") && !controller.armedKnown
        ? ["心跳"]
        : []),
    ];
  return [
    ...(!controller.verified ? ["飛控身分"] : []),
    ...(!controller.parametersVerified ? ["參數"] : []),
    ...(!controller.armedKnown ? ["心跳"] : []),
  ];
}

export function shouldFollowTakeoff(
  previousGrounded,
  grounded,
  alreadyFollowed,
) {
  return previousGrounded === true && grounded === false && !alreadyFollowed;
}

function missionSequenceIndex(mission, seq) {
  const start = mission?.waypointStartSeq;
  return Number.isSafeInteger(start) &&
    start >= 0 &&
    Array.isArray(mission?.points) &&
    Number.isSafeInteger(seq) &&
    seq >= start &&
    seq < start + mission.points.length
    ? seq - start
    : null;
}

export function externalMissionIndex(mission) {
  return missionSequenceIndex(mission, mission?.currentSeq);
}

export function externalMissionReachedCount(mission) {
  if (
    !Number.isSafeInteger(mission?.waypointStartSeq) ||
    mission.waypointStartSeq < 0 ||
    !Array.isArray(mission?.points) ||
    !Array.isArray(mission?.reachedSeqs)
  )
    return null;
  return new Set(
    mission.reachedSeqs
      .map((seq) => missionSequenceIndex(mission, seq))
      .filter((index) => index !== null),
  ).size;
}

export function externalMissionControls(
  state,
  points,
  invalid = false,
  busy = false,
) {
  const mission = state?.mission;
  const editable = ["ardupilot", "px4"].includes(state?.controller?.kind);
  const valid =
    !invalid &&
    points.length > 0 &&
    points.length <= 50 &&
    typeof mission?.limits?.holdRequiresInteger === "boolean" &&
    points.every(
      (p) =>
        ["north", "east", "altitude", "hold"].every((key) =>
          Number.isFinite(p[key]),
        ) &&
        p.altitude >= (mission?.limits?.minAltitude ?? 1) &&
        (mission?.limits?.holdRequiresInteger !== true ||
          Number.isInteger(p.hold)) &&
        p.hold >= 0 &&
        p.hold <= 60 &&
        (!mission?.limits ||
          (Math.hypot(p.north, p.east) <= mission.limits.radius &&
            p.altitude <= mission.limits.height)),
    );
  const ready =
    editable &&
    hilWebControlAllowed(state) &&
    state.phase === "running" &&
    mission?.homeReady === true &&
    mission.readyToUpload === true &&
    !busy &&
    !["uploading", "verifying"].includes(mission?.phase) &&
    !hilFlightReadiness(state).length &&
    state.controller.armed === false &&
    state.snapshot?.runway?.weightOnWheels === true;
  return {
    editable,
    upload: !!(ready && valid),
    start: !!(
      ready &&
      valid &&
      mission?.verified === true &&
      mission.phase === "ready" &&
      Array.isArray(mission.points) &&
      points.length === mission.points.length &&
      points.every((p, i) =>
        ["north", "east", "altitude", "hold"].every(
          (key) => p[key] === mission.points[i][key],
        ),
      )
    ),
  };
}
