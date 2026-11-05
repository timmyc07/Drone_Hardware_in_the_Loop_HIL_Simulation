import { buildSurveyRoute } from "./survey.js";

export function createSurveyEditor({ getContext, onChange, onUse }) {
  const $ = (id) => document.getElementById(id);
  const drafts = new Map();
  const ids = [
    "survey-tool", "survey-spacing", "survey-angle", "survey-altitude",
    "survey-reverse", "survey-add", "survey-example", "survey-preview", "survey-clear",
  ];
  let current, key, limits, readonly, blocked;
  const contextLimits = (c) => JSON.stringify([c.home, c.radius, c.minAltitude, c.maxAltitude]);
  const canEdit = () => {
    const c = getContext();
    return !c.readonly && !c.blocked && c.vehicleId === key && contextLimits(c) === limits;
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
    $("survey-use").disabled = disabled || !current.preview;
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
    return `航點 ${result.waypoints.length}，航段 ${result.transects.length}，面積 ${result.areaM2.toFixed(1)} m²，長度 ${result.routeLengthM.toFixed(1)} m，實際間距 ${result.actualSpacingM.toFixed(1)} m。`;
  }
  function refresh() {
    const c = getContext();
    const nextLimits = contextLimits(c);
    const vehicleChanged = !current || key !== c.vehicleId;
    const overlayChanged = vehicleChanged || limits !== nextLimits || readonly !== c.readonly;
    const disabledChanged = readonly !== c.readonly || blocked !== c.blocked;
    if (!overlayChanged && !disabledChanged) return false;
    if (vehicleChanged) {
      key = c.vehicleId;
      if (!drafts.has(key)) {
        drafts.set(key, { polygon: [], spacing: "6", angle: "0", altitude: "6", reverse: false, tool: "waypoint", preview: null });
      }
      current = drafts.get(key);
      $("survey-status").textContent = current.preview ? summary(current.preview) : "請建立區域並預覽航線。";
    }
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
    if (vehicleChanged) render();
    else updateDisabled();
    return overlayChanged;
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
      const result = buildSurveyRoute(
        current.polygon.map((p) => ({ north: number(p.north), east: number(p.east) })),
        {
          spacingM: number(current.spacing),
          angleDeg: number(current.angle),
          altitudeM: number(current.altitude),
          reverse: current.reverse,
        },
      );
      const c = getContext();
      if (result.waypoints.some((p) =>
        Math.hypot(p.north - c.home[0], p.east - c.home[1]) > c.radius ||
        p.altitude < c.minAltitude || p.altitude > c.maxAltitude
      )) throw new Error("航線超出返航點半徑或高度限制。");
      current.preview = result;
      $("survey-status").textContent = summary(result);
    } catch (error) {
      current.preview = null;
      $("survey-status").textContent = error.message;
    }
    updateDisabled();
    onChange();
  };
  $("survey-use").onclick = () => {
    if (!canEdit() || !current.preview) return;
    onUse(structuredClone(current.preview.waypoints));
    $("survey-status").textContent = "已更新航點草稿，仍須套用／開始任務。";
  };
  refresh();
  return {
    refresh,
    extent() {
      if (getContext().readonly) return 0;
      const coordinates = current.polygon.flatMap((p) => [p.north, p.east]);
      return Math.max(0, ...coordinates.flatMap((value) => {
        const n = value === "" ? NaN : Number(value);
        return Number.isFinite(n) && Math.abs(n) <= 500 ? [Math.abs(n)] : [];
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
