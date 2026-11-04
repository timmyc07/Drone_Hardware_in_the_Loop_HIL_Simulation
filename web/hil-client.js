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

const HEALTH_BITS = Object.freeze({
  0: "陀螺儀",
  1: "加速度計",
  2: "磁力計",
  3: "氣壓計",
  4: "差壓感測器",
  5: "GPS",
  16: "遙控接收器",
  21: "姿態與航向參考",
  28: "預檢",
});
export function formatHilHealth(
  health,
  { verified = false, connected = false } = {},
) {
  const sys = health?.sysStatus;
  const validAge = (value) => Number.isFinite(value) && value >= 0;
  const heartbeatFresh =
    connected &&
    verified &&
    validAge(health?.heartbeatAgeMs) &&
    health.heartbeatAgeMs <= 5000;
  const fresh = heartbeatFresh && validAge(sys?.ageMs) && sys.ageMs <= 5000;
  const labels = {
    unknown: "未知",
    "reported-ok": "飛控預檢位元回報正常",
    failed: "飛控預檢回報失敗",
    "checks-disabled": "飛控回報預檢位元未啟用",
    armed: "飛控目前已解鎖",
  };
  const prearm =
    fresh || (heartbeatFresh && health.prearm === "armed")
      ? labels[health.prearm] || labels.unknown
      : labels.unknown;
  const age = (value) =>
    validAge(value) ? `${(value / 1000).toFixed(1)} 秒` : "未知";
  const hex = (value) =>
    `0x${(value >>> 0).toString(16).padStart(8, "0").toUpperCase()}`;
  const unhealthy = sys
    ? Array.from({ length: 32 }, (_, bit) => bit)
        .filter((bit) => ((sys.unhealthyMask >>> bit) & 1) !== 0)
        .map(
          (bit) => `${HEALTH_BITS[bit] || `位元 ${bit}`}（${hex(2 ** bit)}）`,
        )
    : [];
  const status = `${connected ? "飛控健康回報" : "飛控健康回報（連線中斷，以下為斷線時歷史快照）"}：${prearm} · ${connected ? "" : "斷線時"}心跳齡期 ${age(health?.heartbeatAgeMs)} · ${connected ? "" : "斷線時"}狀態齡期 ${age(sys?.ageMs)}`;
  const names = sys
    ? Array.from({ length: 32 }, (_, bit) => bit)
        .filter((bit) => ((sys.unhealthyMask >>> bit) & 1) !== 0)
        .map((bit) => HEALTH_BITS[bit] || `位元 ${bit}`)
    : [];
  const warning = !sys
    ? "尚未收到飛控健康旗標。"
    : `${fresh ? "飛控回報" : "上次回報（過期或未驗證）"}${names.length ? "尚未健康：" + names.join("、") : "無未健康位元"}。`;
  const details = sys
    ? `存在 ${hex(sys.present)} · 啟用 ${hex(sys.enabled)} · 健康 ${hex(sys.health)} · 啟用但不健康 ${hex(sys.unhealthyMask)}：${unhealthy.length ? unhealthy.join("、") : "無回報位元"}${fresh ? "" : "（資料過期或未驗證）"}`
    : "尚未收到飛控健康旗標。";
  const messages = (Array.isArray(health?.messages) ? health.messages : [])
    .slice(-20)
    .map((message) => {
      const chunk =
        message.chunked || message.id !== 0
          ? ` · 未組合片段 ${message.id}/${message.chunkSeq}`
          : "";
      const content =
        message.text === null
          ? `無法獨立解碼（${message.rawBytesHex}）`
          : message.text;
      return `歷史訊息 · ${connected ? "" : "斷線時齡期 "}${age(message.ageMs)}前 · 嚴重度 ${message.severity}${chunk}：${content}`;
    });
  return { status, warning, details, messages };
}
