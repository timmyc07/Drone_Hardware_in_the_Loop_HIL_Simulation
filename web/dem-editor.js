const MAX_FILE_BYTES = 64 * 1024 * 1024;

export function createDemEditor({ getReadonly, onApply }) {
  const $ = (id) => document.getElementById(id);
  const fields = [
    "dem-file",
    "dem-radius",
    "dem-grid",
    "dem-latitude",
    "dem-longitude",
    "dem-band",
    "dem-unit",
    "dem-datum",
  ];
  let readonly = false,
    pending = false,
    applying = false,
    worker,
    requestId = 0,
    preview,
    sceneKey = "";
  const locked = () => readonly || getReadonly();
  function update() {
    const disabled = locked() || pending || applying;
    for (const id of fields) $(id).disabled = disabled;
    $("dem-preview").disabled = disabled;
    $("dem-cancel").disabled = locked() || !pending;
    $("dem-apply").disabled = disabled || !preview;
  }
  function cancel() {
    requestId++;
    if (worker) {
      worker.postMessage({ type: "cancel", requestId: requestId - 1 });
      worker.terminate();
      worker = undefined;
    }
    pending = false;
    update();
  }
  function describe(result) {
    const { terrain, geography } = result,
      { origin, source } = geography;
    const extentN = terrain.north + (terrain.rows - 1) * terrain.cellSize;
    const extentE = terrain.east + (terrain.cols - 1) * terrain.cellSize;
    const low = Math.min(...terrain.heights),
      high = Math.max(...terrain.heights);
    return `來源：${source.name} · ${source.crs} · ${source.width} × ${source.height} · 波段 ${source.band}\n原點：${origin.latitudeDeg.toFixed(6)}°, ${origin.longitudeDeg.toFixed(6)}° · 高程 ${origin.altitudeM.toFixed(2)} m · 垂直基準 ${origin.verticalDatum}\n模型：${terrain.rows} × ${terrain.cols} · 間距 ${terrain.cellSize.toFixed(2)} m · 北 ${terrain.north.toFixed(1)} 至 ${extentN.toFixed(1)} m · 東 ${terrain.east.toFixed(1)} 至 ${extentE.toFixed(1)} m\n相對高程範圍 ${low.toFixed(2)} 至 ${high.toFixed(2)} m · 採用高程單位 ${source.heightUnit}`;
  }
  function options(file) {
    const radiusM = Number($("dem-radius").value),
      gridSize = Number($("dem-grid").value),
      band = Number($("dem-band").value);
    const lat = $("dem-latitude").value.trim(),
      lon = $("dem-longitude").value.trim();
    if (
      !Number.isFinite(radiusM) ||
      radiusM < 1 ||
      radiusM > 500 ||
      ![33, 65, 129].includes(gridSize) ||
      (2 * radiusM) / (gridSize - 1) < 1 ||
      (2 * radiusM) / (gridSize - 1) > 50 ||
      !Number.isInteger(band) ||
      band < 1 ||
      band > 8
    )
      throw new Error("半徑、網格或波段超出範圍；網格間距須介於 1 至 50 m。");
    if (!!lat !== !!lon)
      throw new Error("請同時填寫原點緯度與經度，或兩者留空。");
    const value = {
      name: file.name,
      radiusM,
      gridSize,
      band,
      heightUnit: $("dem-unit").value,
      verticalDatum: $("dem-datum").value,
    };
    if (lat) {
      const latitudeDeg = Number(lat),
        longitudeDeg = Number(lon);
      if (
        !Number.isFinite(latitudeDeg) ||
        latitudeDeg < -85 ||
        latitudeDeg > 85 ||
        !Number.isFinite(longitudeDeg) ||
        longitudeDeg < -180 ||
        longitudeDeg > 180
      )
        throw new Error("原點座標超出範圍。");
      value.origin = { latitudeDeg, longitudeDeg };
    }
    return value;
  }
  for (const id of fields)
    $(id).addEventListener(id === "dem-file" ? "change" : "input", () => {
      cancel();
      preview = undefined;
      $("dem-status").textContent = "設定已變更，請重新預覽。";
      update();
    });
  $("dem-preview").onclick = async () => {
    if (locked() || pending || applying) return;
    const file = $("dem-file").files?.[0];
    if (!file) {
      $("dem-status").textContent = "請先選擇 GeoTIFF 檔案。";
      return;
    }
    if (file.size > MAX_FILE_BYTES) {
      $("dem-status").textContent = "檔案超過 64 MiB 上限。";
      return;
    }
    let settings;
    try {
      settings = options(file);
    } catch (error) {
      $("dem-status").textContent = error.message;
      return;
    }
    cancel();
    preview = undefined;
    pending = true;
    const id = ++requestId;
    update();
    $("dem-status").textContent = "正在本機讀取與預覽地形…";
    try {
      const buffer = await file.arrayBuffer();
      if (id !== requestId || locked()) return;
      worker = new Worker(new URL("./dem-worker.js", import.meta.url), {
        type: "module",
      });
      worker.onmessage = ({ data }) => {
        if (id !== requestId || locked() || data?.requestId !== id) return;
        if (data.type !== "result" && data.type !== "error") return;
        worker.terminate();
        worker = undefined;
        pending = false;
        if (data.type === "result") {
          preview = data.result;
          $("dem-status").textContent = describe(preview);
        } else $("dem-status").textContent = "無法預覽地形：" + data.error;
        update();
      };
      worker.onerror = () => {
        if (id !== requestId) return;
        worker?.terminate();
        worker = undefined;
        pending = false;
        $("dem-status").textContent = "無法啟動地形處理。";
        update();
      };
      worker.postMessage(
        { type: "import", requestId: id, buffer, options: settings },
        [buffer],
      );
    } catch (error) {
      if (id !== requestId) return;
      worker?.terminate();
      worker = undefined;
      pending = false;
      $("dem-status").textContent = "無法讀取檔案：" + error.message;
      update();
    }
  };
  $("dem-cancel").onclick = () => {
    cancel();
    preview = undefined;
    $("dem-status").textContent = "已取消處理。";
    update();
  };
  $("dem-apply").onclick = async () => {
    if (locked() || pending || applying || !preview) return;
    applying = true;
    update();
    try {
      await onApply(structuredClone(preview));
    } catch (error) {
      $("dem-status").textContent = "無法套用地形：" + error.message;
    } finally {
      applying = false;
      update();
    }
  };
  update();
  return {
    setReadonly(value) {
      readonly = !!value;
      if (locked()) cancel();
      update();
    },
    setScene(scene) {
      const key = JSON.stringify([scene.terrain, scene.geography]);
      if (key === sceneKey) return;
      sceneKey = key;
      cancel();
      preview = undefined;
      $("dem-status").textContent = scene.geography
        ? `目前地形來源：${scene.geography.source.name} · 原點 ${scene.geography.origin.latitudeDeg.toFixed(6)}°, ${scene.geography.origin.longitudeDeg.toFixed(6)}°`
        : "目前使用程序地形；可選擇 GeoTIFF 預覽。";
      update();
    },
  };
}
