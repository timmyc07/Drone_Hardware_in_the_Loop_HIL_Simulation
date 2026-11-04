import {
  DEFAULT_SCENE,
  validateScene,
  parseSceneJson,
  sceneToJson,
  SCENE_MAX_IMPORT_BYTES,
} from "./scene-data.js";
import { validateTerrain } from "./terrain.js";

const warehouse = [
  { north: 12, east: 3, width: 2, depth: 3, height: 3 },
  { north: 12, east: -5, width: 3, depth: 2, height: 2 },
  { north: -8, east: 7, width: 3, depth: 2, height: 2.5 },
  { north: -8, east: -7, width: 2, depth: 3, height: 3 },
];
export function createSceneEditor({ apply, confirm, download, notify }) {
  const $ = (id) => document.getElementById(id);
  const clone = (value) => structuredClone(value);
  let applied = clone(DEFAULT_SCENE),
    draft = clone(applied),
    savedDraft,
    readonly = false,
    selected = null,
    dragging = false;
  const invalidInputs = new Set();
  const dirty = () => JSON.stringify(draft) !== JSON.stringify(applied);
  const items = (kind) =>
    kind === "obstacle" ? draft.obstacles : draft.customGates;
  const terrainInputs = [
    "terrain-north",
    "terrain-east",
    "terrain-cell",
    "terrain-heights",
  ];
  const terrainPresets = ["flat", "hill", "ridge", "valley"];
  function updateStatus() {
    $("scene-dirty").textContent = readonly
      ? "唯讀：顯示目前場景，可匯出。"
      : invalidInputs.size
        ? "請修正數值後再套用或匯出"
        : dirty()
          ? "草稿尚未套用"
          : "已套用";
    for (const id of [
      "scene-name",
      "scene-tool",
      "scene-add-obstacle",
      "scene-add-gate",
      "scene-template",
      "scene-apply",
      "scene-import",
    ])
      $(id).disabled = readonly;
    for (const id of terrainInputs)
      $(id).disabled =
        readonly ||
        draft.terrain.kind === "flat" ||
        draft.terrain.kind === "dem";
    for (const name of terrainPresets) $("terrain-" + name).disabled = readonly;
    $("scene-apply").disabled = readonly || invalidInputs.size > 0;
    $("scene-export").disabled = !readonly && invalidInputs.size > 0;
  }
  function draw() {
    const canvas = $("scene-map"),
      ctx = canvas.getContext("2d"),
      w = canvas.width,
      h = canvas.height,
      extent = Number($("scene-map-range").value);
    const x = (east) => (east / (2 * extent) + 0.5) * w,
      y = (north) => (0.5 - north / (2 * extent)) * h;
    ctx.fillStyle = "#101c2d";
    ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = "#35566b";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 10; i++) {
      ctx.beginPath();
      ctx.moveTo((i * w) / 10, 0);
      ctx.lineTo((i * w) / 10, h);
      ctx.moveTo(0, (i * h) / 10);
      ctx.lineTo(w, (i * h) / 10);
      ctx.stroke();
    }
    draft.obstacles.forEach((box, i) => {
      ctx.fillStyle =
        selected?.kind === "obstacle" && selected.index === i
          ? "#ffd29c"
          : "#a7795a";
      ctx.fillRect(
        x(box.east - box.depth / 2),
        y(box.north + box.width / 2),
        (box.depth * w) / (2 * extent),
        (box.width * h) / (2 * extent),
      );
    });
    draft.customGates.forEach((gate, i) => {
      ctx.fillStyle =
        selected?.kind === "gate" && selected.index === i
          ? "#ffffff"
          : "#65dfd1";
      ctx.beginPath();
      ctx.arc(x(gate.east), y(gate.north), 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillText(String(i + 1), x(gate.east) + 8, y(gate.north) - 6);
    });
  }
  function render() {
    const invalidTerrain = invalidInputs.has($("terrain-heights"));
    invalidInputs.clear();
    if (invalidTerrain) invalidInputs.add($("terrain-heights"));
    $("scene-name").value = draft.name;
    const terrain = draft.terrain;
    if (!invalidTerrain) {
      $("terrain-north").value = terrain.kind === "flat" ? "" : terrain.north;
      $("terrain-east").value = terrain.kind === "flat" ? "" : terrain.east;
      $("terrain-cell").value = terrain.kind === "flat" ? "" : terrain.cellSize;
      $("terrain-heights").value =
        terrain.kind === "flat" || terrain.kind === "dem"
          ? ""
          : Array.from({ length: terrain.rows }, (_, i) =>
              terrain.heights
                .slice(i * terrain.cols, (i + 1) * terrain.cols)
                .join(" "),
            ).join("\n");
      $("terrain-error").textContent =
        terrain.kind === "dem"
          ? "匯入地形的高度網格唯讀；選擇程序地形會移除地理來源。"
          : applied.geography && !draft.geography
            ? "套用程序地形時將移除地理來源與座標原點，並重設實驗。"
            : "";
    }
    const list = $("scene-object-list");
    list.replaceChildren();
    for (const [kind, label] of [
      ["obstacle", "障礙"],
      ["gate", "門"],
    ])
      items(kind).forEach((item, index) => {
        const button = document.createElement("button");
        button.type = "button";
        button.textContent = `${label} ${index + 1}`;
        button.classList.toggle(
          "active",
          selected?.kind === kind && selected.index === index,
        );
        button.onclick = () => {
          selected = { kind, index };
          render();
        };
        list.append(button);
      });
    const fields = $("scene-object-fields");
    fields.replaceChildren();
    const object = selected && items(selected.kind)[selected.index];
    if (object)
      for (const [key, label, min, max] of selected.kind === "obstacle"
        ? [
            ["north", "北 (m)", -500, 500],
            ["east", "東 (m)", -500, 500],
            ["width", "南北寬 (m)", 0.01, 100],
            ["depth", "東西深 (m)", 0.01, 100],
            ["height", "高度 (m)", 0.01, 500],
          ]
        : [
            ["north", "北 (m)", -500, 500],
            ["east", "東 (m)", -500, 500],
            ["altitude", "高度 (m)", 0.25, 500],
            ["yaw", "方向 (°)", -180, 180],
            ["radius", "半徑 (m)", 0.25, 100],
          ]) {
        const labelElement = document.createElement("label");
        labelElement.textContent = label;
        const input = document.createElement("input");
        input.type = "number";
        input.min = min;
        input.max = max;
        input.step = "any";
        input.disabled = readonly;
        input.value =
          key === "yaw"
            ? Number(((object[key] * 180) / Math.PI).toFixed(4))
            : object[key];
        const validValue = () => {
          const n = Number(input.value);
          return input.value !== "" &&
            Number.isFinite(n) &&
            n >= min &&
            n <= max
            ? n
            : null;
        };
        input.oninput = () => {
          const n = validValue();
          if (n === null) invalidInputs.add(input);
          else {
            invalidInputs.delete(input);
            object[key] = key === "yaw" ? (n * Math.PI) / 180 : n;
            draw();
          }
          updateStatus();
        };
        input.onchange = input.onblur = () => {
          if (validValue() === null) {
            notify("數值超出範圍，已還原。");
            input.value =
              key === "yaw"
                ? Number(((object[key] * 180) / Math.PI).toFixed(4))
                : object[key];
            invalidInputs.delete(input);
            updateStatus();
          } else input.oninput();
        };
        labelElement.append(input);
        fields.append(labelElement);
      }
    updateStatus();
    $("scene-remove").disabled = readonly || !object;
    $("scene-up").disabled =
      readonly || !object || selected.kind !== "gate" || selected.index === 0;
    $("scene-down").disabled =
      readonly ||
      !object ||
      selected.kind !== "gate" ||
      selected.index === draft.customGates.length - 1;
    draw();
  }
  function add(kind, north = 0, east = 0) {
    if (readonly) return;
    const list = items(kind);
    if (list.length >= 50) return notify("每種物件最多 50 個。");
    list.push(
      kind === "obstacle"
        ? { north, east, width: 2, depth: 2, height: 3 }
        : { north, east, altitude: 2, yaw: 0, radius: 2 },
    );
    selected = { kind, index: list.length - 1 };
    render();
  }
  $("scene-name").oninput = (event) => {
    draft.name = event.target.value;
    updateStatus();
  };
  $("scene-add-obstacle").onclick = () => add("obstacle");
  $("scene-add-gate").onclick = () => add("gate");
  $("scene-template").onclick = () => {
    draft.obstacles = clone(warehouse);
    draft.environment.preset = "warehouse";
    selected = null;
    render();
  };
  $("scene-remove").onclick = () => {
    if (!selected || readonly) return;
    items(selected.kind).splice(selected.index, 1);
    selected = null;
    render();
  };
  for (const [id, delta] of [
    ["scene-up", -1],
    ["scene-down", 1],
  ])
    $(id).onclick = () => {
      if (!selected || selected.kind !== "gate" || readonly) return;
      const other = selected.index + delta,
        list = draft.customGates;
      if (other < 0 || other >= list.length) return;
      [list[selected.index], list[other]] = [list[other], list[selected.index]];
      selected.index = other;
      render();
    };
  $("scene-map-range").onchange = draw;
  const canvas = $("scene-map");
  function coordinates(event) {
    const rect = canvas.getBoundingClientRect(),
      extent = Number($("scene-map-range").value);
    return {
      north: Math.max(
        -500,
        Math.min(
          500,
          (0.5 - (event.clientY - rect.top) / rect.height) * 2 * extent,
        ),
      ),
      east: Math.max(
        -500,
        Math.min(
          500,
          ((event.clientX - rect.left) / rect.width - 0.5) * 2 * extent,
        ),
      ),
    };
  }
  canvas.onpointerdown = (event) => {
    if (readonly) return;
    const p = coordinates(event),
      extent = Number($("scene-map-range").value),
      tolerance = extent * 0.04;
    let found = null,
      best = tolerance;
    for (const kind of ["gate", "obstacle"])
      items(kind).forEach((object, index) => {
        const dn = Math.abs(object.north - p.north),
          de = Math.abs(object.east - p.east);
        const d =
          kind === "obstacle"
            ? Math.hypot(
                Math.max(0, dn - object.width / 2),
                Math.max(0, de - object.depth / 2),
              )
            : Math.hypot(dn, de);
        if (d < best) {
          best = d;
          found = { kind, index };
        }
      });
    if (found) {
      selected = found;
      dragging = true;
      canvas.setPointerCapture(event.pointerId);
      render();
    } else if ($("scene-tool").value !== "select")
      add(
        $("scene-tool").value,
        Number(p.north.toFixed(1)),
        Number(p.east.toFixed(1)),
      );
  };
  canvas.onpointermove = (event) => {
    if (!dragging || !selected || readonly) return;
    const p = coordinates(event),
      object = items(selected.kind)[selected.index];
    object.north = Number(p.north.toFixed(1));
    object.east = Number(p.east.toFixed(1));
    render();
  };
  canvas.onpointerup = canvas.onpointercancel = () => {
    dragging = false;
  };
  $("scene-apply").onclick = async () => {
    if (invalidInputs.size) return notify("請先修正數值。");
    try {
      const valid = validateScene(draft);
      await apply(valid);
      notify("場景已套用。");
    } catch (error) {
      notify(`無法套用場景：${error.message}`);
    }
  };
  $("scene-export").onclick = () => {
    if (!readonly && invalidInputs.size) return notify("請先修正數值。");
    try {
      download(
        sceneToJson(readonly ? applied : draft),
        "json",
        "application/json",
        "flight-lab-scene.json",
      );
    } catch (error) {
      notify(`無法匯出場景：${error.message}`);
    }
  };
  $("scene-import").onclick = () => $("scene-import-file").click();
  $("scene-import-file").onchange = async (event) => {
    const file = event.target.files?.[0];
    event.target.value = "";
    if (!file || readonly) return;
    if (file.size > SCENE_MAX_IMPORT_BYTES)
      return notify("場景檔案超過 128 KiB。");
    try {
      const value = parseSceneJson(await file.text());
      if (
        dirty() &&
        !(await confirm("匯入將取代尚未套用的場景草稿。確定繼續？"))
      )
        return;
      invalidInputs.delete(terrainInvalid);
      draft = clone(value);
      selected = null;
      render();
    } catch (error) {
      notify(`無法匯入場景：${error.message}`);
    }
  };
  for (const name of terrainPresets)
    $("terrain-" + name).onclick = () => {
      if (readonly) return;
      invalidInputs.delete(terrainInvalid);
      draft.geography = null;
      if (name === "flat") draft.terrain = { kind: "flat" };
      else {
        const heights = Array(81).fill(0);
        for (let i = 1; i < 8; i++)
          for (let j = 6; j < 8; j++) {
            const weight =
              Math.max(0, 1 - Math.abs(i - 4) / (name === "ridge" ? 4 : 3)) *
              (j === 7 ? 1 : 0.5);
            heights[i * 9 + j] = Number(
              (
                weight * (name === "valley" ? -8 : name === "ridge" ? 10 : 12)
              ).toFixed(2),
            );
          }
        draft.terrain = validateTerrain({
          kind: "heightfield",
          north: -80,
          east: -80,
          cellSize: 20,
          rows: 9,
          cols: 9,
          heights,
        });
      }
      render();
    };
  const terrainInvalid = $("terrain-heights");
  function editTerrain() {
    if (
      readonly ||
      draft.terrain.kind === "flat" ||
      draft.terrain.kind === "dem"
    )
      return;
    try {
      const text = $("terrain-heights").value.trim();
      if (text.length > 100000) throw new Error("矩陣內容過長");
      const rows = text.split(/\r?\n/).map((line) => line.trim().split(/\s+/));
      if (
        !text ||
        rows.some(
          (row) =>
            row.length !== rows[0].length ||
            row.some(
              (value) => value === "" || !Number.isFinite(Number(value)),
            ),
        )
      )
        throw new Error("矩陣格式不正確");
      const numbers = ["terrain-north", "terrain-east", "terrain-cell"].map(
        (id) => {
          const input = $(id);
          if (!String(input.value).trim())
            throw new Error("請填寫網格位置與間距");
          return Number(input.value);
        },
      );
      draft.terrain = validateTerrain({
        kind: "heightfield",
        north: numbers[0],
        east: numbers[1],
        cellSize: numbers[2],
        rows: rows.length,
        cols: rows[0].length,
        heights: rows.flat().map(Number),
      });
      invalidInputs.delete(terrainInvalid);
      $("terrain-error").textContent = "";
      draw();
    } catch (error) {
      invalidInputs.add(terrainInvalid);
      $("terrain-error").textContent = "地形無效：" + error.message;
    }
    updateStatus();
  }
  for (const id of terrainInputs) $(id).oninput = editTerrain;
  render();
  return {
    getApplied: () => clone(applied),
    setScene(value, viewOnly = false) {
      const wasDirty = dirty(),
        changed = JSON.stringify(value) !== JSON.stringify(applied);
      if (!changed && viewOnly === readonly) return;
      if (viewOnly && !readonly) savedDraft = clone(draft);
      invalidInputs.delete(terrainInvalid);
      if (!viewOnly && readonly) {
        draft = savedDraft ?? clone(value);
        savedDraft = undefined;
      } else if (viewOnly || (!wasDirty && changed)) draft = clone(value);
      applied = clone(value);
      readonly = viewOnly;
      if (changed || viewOnly) selected = null;
      render();
    },
    markApplied(value, submitted) {
      const unchanged = JSON.stringify(draft) === JSON.stringify(submitted);
      applied = clone(value);
      if (unchanged) {
        draft = clone(value);
        selected = null;
      }
      render();
    },
  };
}
