import {
  replayFrameAtTime,
  replayEventFrame,
  replayEventIndices,
  adjacentReplayEvent,
} from "./replay.js";

const eventLabels = {
  scene: "場景變更",
  wind: "風向與風速",
  render: "畫面渲染",
  targets: "飛行目標",
  gust: "陣風",
  mode: "飛行模式",
  gimbal: "雲台設定",
  operatorGimbal: "操作員雲台設定",
  collisionPrevention: "幾何防撞設定",
  transition: "飛行狀態轉換",
  position: "位置目標",
  mission: "任務航線",
  fault: "注入故障",
  clearFaults: "清除故障",
  takeoff: "起飛",
  abortTakeoff: "中止起飛",
  arm: "解鎖",
  disarm: "上鎖",
  training: "訓練課程",
  sensorConfig: "感測器設定",
  sensorFault: "感測器故障",
  inspection: "巡檢拍攝",
};
const reasonLabels = {
  range: "距離不符",
  angle: "角度不符",
  frustum: "目標不在視野內",
  pixels: "影像解析度不足",
  speed: "飛行速度過快",
  occluded: "目標被遮擋",
};

export function createReplayControls({ seek }) {
  const element = (name) => document.getElementById(`replay-${name}`);
  const panel = element("controls"),
    frame = element("frame"),
    position = element("position"),
    seconds = element("seconds"),
    error = element("error"),
    type = element("event-type"),
    vehicle = element("event-vehicle"),
    details = element("event-details"),
    first = element("first"),
    previous = element("previous"),
    next = element("next"),
    last = element("last"),
    eventPrevious = element("event-previous"),
    eventNext = element("event-next");
  let record = null,
    frameIndex = 0,
    selectedEvent = -1,
    indices = [];

  function refresh() {
    if (!record) return;
    const sample = record.samples[frameIndex],
      end = record.samples.length - 1;
    frame.max = String(end);
    frame.value = String(frameIndex);
    position.textContent = `影格 ${frameIndex + 1} / ${end + 1} · ${sample.time.toFixed(3)} 秒（範圍 ${record.samples[0].time.toFixed(3)}–${record.samples[end].time.toFixed(3)} 秒）`;
    frame.setAttribute(
      "aria-valuetext",
      `影格 ${frameIndex + 1} / ${end + 1}，${sample.time.toFixed(3)} 秒`,
    );
    first.disabled = previous.disabled = frameIndex === 0;
    next.disabled = last.disabled = frameIndex === end;
    eventPrevious.disabled =
      adjacentReplayEvent(record, indices, frameIndex, -1, selectedEvent) ===
      null;
    eventNext.disabled =
      adjacentReplayEvent(record, indices, frameIndex, 1, selectedEvent) ===
      null;
    if (selectedEvent >= 0) {
      const event = record.events[selectedEvent];
      const spec = record.world.vehicles.find((v) => v.id === event.vehicleId);
      const subject = spec ? `${spec.name || spec.id}（${spec.id}）` : "世界";
      let inspection = "";
      if (event.type === "inspection") {
        let scene = record.world.scene;
        for (let i = 0; i < selectedEvent; i++)
          if (record.events[i].type === "scene") scene = record.events[i].value;
        const id = event.value.targetId;
        const target = scene?.inspectionTargets.find((item) => item.id === id);
        const assessment = event.receipt.assessment;
        inspection = ` · 目標 ${target?.name || id}（${id}） · ${assessment.passed ? "通過" : `未通過：${assessment.reasons.map((reason) => reasonLabels[reason] || reason).join("、")}`}`;
      }
      details.textContent = `事件 ${indices.indexOf(selectedEvent) + 1} / ${indices.length}：${eventLabels[event.type] || event.type} · ${event.time.toFixed(3)} 秒 · ${subject}${inspection}`;
    } else
      details.textContent = `符合篩選的事件：${indices.length}；尚未選擇事件。`;
  }
  function filter() {
    selectedEvent = -1;
    indices = replayEventIndices(record, {
      type: type.value,
      vehicleId: vehicle.value === "@world" ? null : vehicle.value,
    });
    refresh();
  }
  function manualSeek(index) {
    selectedEvent = -1;
    error.textContent = "";
    seek(index);
    update(index);
  }
  function eventSeek(direction) {
    const index = adjacentReplayEvent(
      record,
      indices,
      frameIndex,
      direction,
      selectedEvent,
    );
    if (index === null) return;
    selectedEvent = index;
    const target = replayEventFrame(record, index);
    error.textContent = "";
    seek(target);
    update(target);
  }
  function options(select, values) {
    while (select.options.length > 1) select.remove(1);
    for (const [value, label] of values) {
      const option = document.createElement("option");
      option.value = value;
      option.textContent = label;
      select.append(option);
    }
    select.value = "";
  }
  function setRecording(value) {
    record = value;
    panel.hidden = !record;
    frameIndex = 0;
    selectedEvent = -1;
    seconds.value = "";
    error.textContent = "";
    options(
      type,
      record
        ? [...new Set(record.events.map((event) => event.type))]
            .sort()
            .map((v) => [v, eventLabels[v] || v])
        : [],
    );
    options(
      vehicle,
      record
        ? [
            ["@world", "世界事件"],
            ...record.world.vehicles.map((v) => [
              v.id,
              `${v.name || v.id}（${v.id}）`,
            ]),
          ]
        : [],
    );
    if (!record) {
      details.textContent = "";
      position.textContent = "";
      return;
    }
    filter();
  }
  function update(index) {
    if (!record) return;
    frameIndex = index;
    if (selectedEvent >= 0 && replayEventFrame(record, selectedEvent) !== index)
      selectedEvent = -1;
    refresh();
  }
  first.addEventListener("click", () => manualSeek(0));
  previous.addEventListener("click", () => manualSeek(frameIndex - 1));
  next.addEventListener("click", () => manualSeek(frameIndex + 1));
  last.addEventListener("click", () => manualSeek(record.samples.length - 1));
  frame.addEventListener("input", () => manualSeek(Number(frame.value)));
  element("seek-time").addEventListener("click", () => {
    const value = seconds.value.trim(),
      time = Number(value);
    if (!value || !Number.isFinite(time)) {
      error.textContent = "請輸入有效時間。";
      return;
    }
    manualSeek(replayFrameAtTime(record, time));
  });
  type.addEventListener("change", filter);
  vehicle.addEventListener("change", filter);
  eventPrevious.addEventListener("click", () => eventSeek(-1));
  eventNext.addEventListener("click", () => eventSeek(1));
  return { setRecording, update };
}
