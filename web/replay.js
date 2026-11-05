// Records have already passed validateRecording; navigation never synthesizes state.
function lowerBound(length, before) {
  let low = 0,
    high = length;
  while (low < high) {
    const middle = Math.floor((low + high) / 2);
    if (before(middle)) low = middle + 1;
    else high = middle;
  }
  return low;
}

function frameIndex(record, index) {
  if (!Number.isInteger(index) || index < 0 || index >= record.samples.length)
    throw new Error("Invalid replay frame index");
}

export function replayFrameAtTime(record, time) {
  if (!Number.isFinite(time)) throw new Error("Invalid replay time");
  if (!record.samples.length) throw new Error("Recording has no replay frames");
  return Math.max(
    0,
    lowerBound(record.samples.length, (i) => record.samples[i].time <= time) -
      1,
  );
}

export function replayEventFrame(record, eventIndex) {
  if (
    !Number.isInteger(eventIndex) ||
    eventIndex < 0 ||
    eventIndex >= record.events.length
  )
    throw new Error("Invalid replay event index");
  if (!record.samples.length) throw new Error("Recording has no replay frames");
  const tick = record.events[eventIndex].tick;
  return Math.min(
    record.samples.length - 1,
    lowerBound(record.samples.length, (i) => record.samples[i].tick < tick),
  );
}

export function replayEventIndices(record, { type = "", vehicleId = "" } = {}) {
  const indices = [];
  record.events.forEach((event, index) => {
    if (
      (!type || event.type === type) &&
      (vehicleId === "" ||
        (vehicleId === null
          ? event.vehicleId === undefined
          : event.vehicleId === vehicleId))
    )
      indices.push(index);
  });
  return indices;
}

export function adjacentReplayEvent(
  record,
  indices,
  currentFrame,
  direction,
  selectedEventIndex = -1,
) {
  frameIndex(record, currentFrame);
  if (direction !== -1 && direction !== 1)
    throw new Error("Invalid replay event direction");
  if (
    !Number.isInteger(selectedEventIndex) ||
    selectedEventIndex < -1 ||
    selectedEventIndex >= record.events.length
  )
    throw new Error("Invalid selected replay event index");
  const selected = lowerBound(
    indices.length,
    (i) => indices[i] < selectedEventIndex,
  );
  if (
    selectedEventIndex !== -1 &&
    indices[selected] === selectedEventIndex &&
    replayEventFrame(record, selectedEventIndex) === currentFrame
  )
    return indices[selected + direction] ?? null;
  const tick = record.samples[currentFrame].tick;
  const next = lowerBound(indices.length, (i) =>
    direction === 1
      ? record.events[indices[i]].tick < tick
      : record.events[indices[i]].tick <= tick,
  );
  return indices[direction === 1 ? next : next - 1] ?? null;
}
