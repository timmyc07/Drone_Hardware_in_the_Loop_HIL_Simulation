import { importDem } from "./dem-import.js";
let active = null;
self.onmessage = async ({ data }) => {
  if (!data || !Number.isSafeInteger(data.requestId) || data.requestId < 1)
    return;
  if (data.type === "cancel") {
    if (active?.requestId === data.requestId) active.controller.abort();
    return;
  }
  if (data.type !== "import") return;
  if (active) {
    self.postMessage({
      type: "error",
      requestId: data.requestId,
      name: "Error",
      error: "DEM import is already running",
    });
    return;
  }
  const task = { requestId: data.requestId, controller: new AbortController() };
  active = task;
  try {
    const result = await importDem(
      data.buffer,
      data.options,
      task.controller.signal,
    );
    task.controller.signal.throwIfAborted();
    self.postMessage({ type: "result", requestId: task.requestId, result });
  } catch (error) {
    self.postMessage({
      type: "error",
      requestId: task.requestId,
      name: error.name,
      error: String(error.message ?? error),
    });
  } finally {
    if (active === task) active = null;
  }
};
