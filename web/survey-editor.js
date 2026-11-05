import { buildSurveyRoute } from "./survey.js";
import { calculateSurveyCamera, surveyFootprint } from "./survey-camera.js";

export function createSurveyEditor({ getContext, onChange, onUse }) {
  const $ = (id) => document.getElementById(id);
  const drafts = new Map();
  const ids = [
    "survey-tool", "survey-spacing", "survey-angle", "survey-altitude",
    "survey-reverse", "survey-camera-enabled", "survey-plane-altitude", "survey-front-overlap", "survey-side-overlap", "survey-add", "survey-example", "survey-preview", "survey-clear",
  ];
  let current, key, limits, readonly, blocked, cameraBlocked, cameraAvailable, pending = false;
  const cameraKey = (c) => JSON.stringify(c.cameraConfig);
  const contextLimits = (c) => JSON.stringify([c.home, c.radius, c.minAltitude, c.maxAltitude]);
  const canEdit = () => {
    const c = getContext();
    return !pending && !c.readonly && !c.blocked && c.vehicleId === key && contextLimits(c) === limits;
  };
  function invalidate() {
    current.preview = null;
    $("survey-status").textContent = "區域或設定已變更，請重新預覽。";
    $("survey-use").disabled = true;
  }
  function updateDisabled() {
    const disabled = !canEdit();
    const c = getContext();
    $("survey-altitude").min = c.minAltitude;
    $("survey-altitude").max = c.maxAltitude;
    for (const id of ids) $(id).disabled = disabled;
    $("survey-spacing").disabled = disabled || current.cameraEnabled;
    $("survey-use").textContent = current.cameraEnabled ? "套用航點與拍攝設定" : "取代航點草稿";
    $("survey-use").disabled = disabled || !current.preview ||
      (current.cameraEnabled && (!c.cameraAvailable || c.cameraBlocked || current.cameraKey !== cameraKey(c)));
    for (const row of $("survey-vertices").children) {
      for (const label of [...row.children].slice(0, 2)) label.children[0].disabled = disabled;
      row.children[2].disabled = disabled;
    }
  }
  function render() {
    $("survey-tool").value = current.tool;
    for (const name of ["spacing", "angle", "altitude"]) {
      $("survey-" + name).value = current[name];
    }
    $("survey-reverse").checked = current.reverse;
    $("survey-camera-enabled").checked = current.cameraEnabled;
    for (const [id, name] of [["plane-altitude", "planeAltitude"], ["front-overlap", "frontOverlap"], ["side-overlap", "sideOverlap"]]) $("survey-" + id).value = current[name];
    const rows = current.polygon.map((point, index) => {
      const row = document.createElement("div");
      row.className = "survey-vertex";
      for (const [axis, label] of [["north", "北"], ["east", "東"]]) {
        const wrapper = document.createElement("label");
        wrapper.textContent = `頂點 ${index + 1} ${label} (m)`;
        const input = document.createElement("input");
        input.type = "number";
        input.min = "-500";
        input.max = "500";
        input.step = "any";
        input.value = point[axis];
        input.oninput = () => {
          if (!canEdit()) return;
          point[axis] = input.value;
          invalidate();
          onChange();
        };
        wrapper.append(input);
        row.append(wrapper);
      }
      const remove = document.createElement("button");
      remove.textContent = `移除頂點 ${index + 1}`;
      remove.onclick = () => {
        if (!canEdit()) return;
        current.polygon.splice(index, 1);
        invalidate();
        render();
        onChange();
      };
      row.append(remove);
      return row;
    });
    $("survey-vertices").replaceChildren(...rows);
    updateDisabled();
  }
  function summary(result) {
    const base = `航點 ${result.waypoints.length}，航段 ${result.transects.length}，面積 ${result.areaM2.toFixed(1)} m²，長度 ${result.routeLengthM.toFixed(1)} m，實際線距 ${result.actualSpacingM.toFixed(3)} m。`;
    if (!result.cameraPlan) return base;
    const p = result.cameraPlan;
    return `${base} 預估範圍 ${p.footprintAcrossM.toFixed(3)} × ${p.footprintAlongM.toFixed(3)} m，GSD ${(p.gsdMPerPixel * 100).toFixed(2)} cm/px，距離拍攝間距 ${p.triggerDistanceM.toFixed(3)} m，估計 ${Math.floor(result.routeLengthM / p.triggerDistanceM) + 1} 張照片（不含進場與實際誤差，非上界；資料集上限 30 張）。${getContext().cameraAvailable ? "" : "僅內建相機模式可套用。"}`;
  }
  function refresh() {
    const c = getContext();
    const nextLimits = contextLimits(c);
    const vehicleChanged = !current || key !== c.vehicleId;
    const overlayChanged = vehicleChanged || limits !== nextLimits || readonly !== c.readonly;
    const cameraChanged = current && current.cameraKey !== cameraKey(c);
    const disabledChanged = readonly !== c.readonly || blocked !== c.blocked || cameraBlocked !== c.cameraBlocked || cameraAvailable !== c.cameraAvailable;
    if (!overlayChanged && !disabledChanged && !cameraChanged) return false;
    if (vehicleChanged) {
      key = c.vehicleId;
      if (!drafts.has(key)) {
        drafts.set(key, { polygon: [], spacing: "6", angle: "0", altitude: "6", reverse: false, cameraEnabled: false, planeAltitude: "0", frontOverlap: "70", sideOverlap: "60", tool: "waypoint", preview: null });
      }
      current = drafts.get(key);
      $("survey-status").textContent = current.preview ? summary(current.preview) : "請建立區域並預覽航線。";
    }
    if (current.cameraKey !== undefined && current.cameraKey !== cameraKey(c) && current.cameraEnabled && current.preview) invalidate();
    current.cameraKey = cameraKey(c);
    if (current.limits !== undefined && current.limits !== nextLimits && current.preview) invalidate();
    if (c.readonly) {
      $("survey-status").textContent = "重播唯讀，測繪草稿不會套用至紀錄。";
    } else if (readonly) {
      $("survey-status").textContent = current.preview
        ? summary(current.preview)
        : "請建立區域並預覽航線。";
    }
    current.limits = nextLimits;
    limits = nextLimits;
    readonly = c.readonly;
    blocked = c.blocked;
    cameraBlocked = c.cameraBlocked;
    cameraAvailable = c.cameraAvailable;
    if (!c.readonly && current.preview && !vehicleChanged) $("survey-status").textContent = summary(current.preview);
    if (vehicleChanged) render();
    else updateDisabled();
    return overlayChanged || !!cameraChanged;
  }
  function addPoint(point) {
    if (current.polygon.length >= 20) {
      $("survey-status").textContent = "測繪區域最多 20 個頂點。";
      return;
    }
    current.polygon.push(point);
    invalidate();
    render();
    onChange();
  }
  $("survey-tool").onchange = () => {
    if (canEdit()) current.tool = $("survey-tool").value;
  };
  for (const name of ["spacing", "angle", "altitude", "reverse"]) {
    $("survey-" + name).oninput = () => {
      if (!canEdit()) return;
      current[name] = name === "reverse" ? $("survey-reverse").checked : $("survey-" + name).value;
      invalidate();
      onChange();
    };
  }
  for (const [id, name] of [["camera-enabled", "cameraEnabled"], ["plane-altitude", "planeAltitude"], ["front-overlap", "frontOverlap"], ["side-overlap", "sideOverlap"]]) {
    $("survey-" + id).oninput = () => {
      if (!canEdit()) return;
      current[name] = name === "cameraEnabled" ? $("survey-" + id).checked : $("survey-" + id).value;
      invalidate();
      updateDisabled();
      onChange();
    };
  }
  $("survey-add").onclick = () => {
    if (!canEdit()) return;
    $("survey-boundary").open = true;
    addPoint({ north: 0, east: 0 });
  };
  $("survey-example").onclick = () => {
    if (!canEdit()) return;
    current.polygon = [
      { north: 4, east: -8 }, { north: 20, east: -8 },
      { north: 20, east: 8 }, { north: 4, east: 8 },
    ];
    invalidate();
    render();
    onChange();
  };
  $("survey-clear").onclick = () => {
    if (!canEdit()) return;
    current.polygon = [];
    invalidate();
    render();
    onChange();
  };
  $("survey-preview").onclick = () => {
    if (!canEdit()) return;
    try {
      const number = (value) => value === "" ? NaN : Number(value);
      const c = getContext();
      const plan = current.cameraEnabled ? calculateSurveyCamera(c.cameraConfig, { altitudeM: number(current.altitude), planeAltitudeM: number(current.planeAltitude), frontOverlapPct: number(current.frontOverlap), sideOverlapPct: number(current.sideOverlap) }) : null;
      const result = buildSurveyRoute(
        current.polygon.map((p) => ({ north: number(p.north), east: number(p.east) })),
        {
          spacingM: plan ? plan.spacingM : number(current.spacing),
          angleDeg: number(current.angle),
          altitudeM: number(current.altitude),
          reverse: current.reverse,
        },
      );
      if (plan) result.cameraPlan = { ...plan, angleDeg: number(current.angle) };
      if (result.waypoints.some((p) =>
        Math.hypot(p.north - c.home[0], p.east - c.home[1]) > c.radius ||
        p.altitude < c.minAltitude || p.altitude > c.maxAltitude
      )) throw new Error("航線超出返航點半徑或高度限制。");
      current.cameraKey = cameraKey(c);
      current.preview = result;
      $("survey-status").textContent = summary(result);
    } catch (error) {
      current.preview = null;
      $("survey-status").textContent = error.message;
    }
    updateDisabled();
    onChange();
  };
  $("survey-use").onclick = async () => {
    const c = getContext();
    if (!canEdit() || !current.preview || (current.cameraEnabled &&
      (!c.cameraAvailable || c.cameraBlocked || current.cameraKey !== cameraKey(c)))) return;
    const draft = current;
    pending = true;
    updateDisabled();
    try {
      const points = structuredClone(draft.preview.waypoints);
      if (draft.cameraEnabled) await onUse(points, structuredClone(draft.preview.cameraPlan));
      else await onUse(points);
      if (current === draft && getContext().vehicleId === key && !getContext().readonly) $("survey-status").textContent = draft.cameraEnabled
        ? "已設定正下視雲台與距離拍攝，仍須開始任務與自動拍攝。"
        : "已更新航點草稿，仍須套用／開始任務。";
    } catch (error) {
      if (current === draft && getContext().vehicleId === key && !getContext().readonly) $("survey-status").textContent = error.message;
    } finally {
      pending = false;
      updateDisabled();
    }
  };
  refresh();
  return {
    refresh,
    extent() {
      if (getContext().readonly) return 0;
      const coordinates = current.polygon.flatMap((p) => [p.north, p.east]);
      if (current.preview?.cameraPlan) {
        const plan = current.preview.cameraPlan;
        for (const point of surveyFootprint(current.preview.waypoints[0], plan.angleDeg, plan)) coordinates.push(point.north, point.east);
      }
      return Math.max(0, ...coordinates.flatMap((value) => {
        const n = value === "" ? NaN : Number(value);
        return Number.isFinite(n) && (current.preview?.cameraPlan || Math.abs(n) <= 500) ? [Math.abs(n)] : [];
      }));
    },
    mapClick({ north, east }) {
      if (!canEdit() || current.tool !== "polygon") return false;
      addPoint({ north, east });
      return true;
    },
    draw(ctx, width, height, extent) {
      if (getContext().readonly) return;
      const x = (east) => (east / (2 * extent) + 0.5) * width;
      const y = (north) => (0.5 - north / (2 * extent)) * height;
      const points = current.polygon.filter((p) =>
        p.north !== "" && p.east !== "" &&
        Number.isFinite(Number(p.north)) && Number.isFinite(Number(p.east)),
      );
      ctx.save();
      if (points.length) {
        ctx.beginPath();
        points.forEach((p, i) => i ? ctx.lineTo(x(p.east), y(p.north)) : ctx.moveTo(x(p.east), y(p.north)));
        if (points.length >= 3) {
          ctx.closePath();
          ctx.fillStyle = "rgba(158, 97, 255, .18)";
          ctx.fill();
        }
        ctx.strokeStyle = "#bd8cff";
        ctx.stroke();
        points.forEach((p, i) => {
          ctx.fillStyle = "#fff";
          ctx.fillText(String(i + 1), x(p.east), y(p.north));
        });
      }
      if (current.preview) {
        const route = current.preview.waypoints;
        if (current.preview.cameraPlan) {
          const corners = surveyFootprint(route[0], current.preview.cameraPlan.angleDeg, current.preview.cameraPlan);
          ctx.beginPath();
          corners.forEach((p, i) => i ? ctx.lineTo(x(p.east), y(p.north)) : ctx.moveTo(x(p.east), y(p.north)));
          ctx.closePath();
          ctx.fillStyle = "rgba(255, 226, 120, .25)";
          ctx.fill();
          ctx.strokeStyle = "#ffe278";
          ctx.stroke();
        }
        ctx.beginPath();
        route.forEach((p, i) => i ? ctx.lineTo(x(p.east), y(p.north)) : ctx.moveTo(x(p.east), y(p.north)));
        ctx.strokeStyle = "#bd8cff";
        ctx.setLineDash([6, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath();
        for (const { start, end } of current.preview.transects) {
          ctx.moveTo(x(start.east), y(start.north));
          ctx.lineTo(x(end.east), y(end.north));
        }
        ctx.strokeStyle = "#40d9e8";
        ctx.stroke();
        for (const [p, label] of [[route[0], "起"], [route.at(-1), "終"]]) {
          ctx.fillStyle = "#40d9e8";
          ctx.fillText(label, x(p.east), y(p.north));
        }
      }
      ctx.restore();
    },
  };
}
