import { createReadStream } from "node:fs";
import { pathToFileURL } from "node:url";
import { isDeepStrictEqual } from "node:util";
import { validateScene } from "../web/scene-data.js";
import {
  validateCameraMetadata,
  parseCameraDataset,
  hashCameraFrame,
  CAMERA_DATASET_MAX_IMPORT_BYTES,
} from "../web/camera.js";
import { INSPECTION_REASONS } from "../web/inspection.js";

const UUID =
  /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/;
const ID = /^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/;
function exact(value, keys, label) {
  if (
    !value ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value)) ||
    Reflect.ownKeys(value).length !== keys.length ||
    Reflect.ownKeys(value).some((key) => !keys.includes(key))
  )
    throw new Error(`Invalid inspection ${label}`);
}
function number(value, min, max, label, integer = false) {
  if (
    !Number.isFinite(value) ||
    value < min ||
    value > max ||
    (integer && !Number.isInteger(value))
  )
    throw new Error(`Invalid inspection ${label}`);
  return value;
}
function dense(value, max, label) {
  if (
    !Array.isArray(value) ||
    value.length > max ||
    Reflect.ownKeys(value).length !== value.length + 1 ||
    !Array.from({ length: value.length }, (_, i) =>
      Object.hasOwn(value, i),
    ).every(Boolean)
  )
    throw new Error(`Invalid inspection ${label}`);
}
function checkAssessment(value, targetId) {
  exact(
    value,
    [
      "targetId",
      "passed",
      "reasons",
      "rangeM",
      "angleDeg",
      "minPixels",
      "speedMps",
      "visibleSamples",
      "totalSamples",
    ],
    "assessment",
  );
  dense(value.reasons, INSPECTION_REASONS.length, "assessment reasons");
  let previous = -1;
  for (const reason of value.reasons) {
    const index = INSPECTION_REASONS.indexOf(reason);
    if (index <= previous)
      throw new Error("Inspection reasons must be canonical and ordered");
    previous = index;
  }
  if (
    value.targetId !== targetId ||
    typeof value.passed !== "boolean" ||
    value.passed !== (value.reasons.length === 0)
  )
    throw new Error("Inspection assessment identity or verdict disagrees");
  number(value.rangeM, 0, 1e10, "range");
  if (value.angleDeg !== null) number(value.angleDeg, 0, 180, "angle");
  if (value.minPixels !== null)
    number(value.minPixels, 0, 1e20, "projected pixels");
  number(value.speedMps, 0, 1e10, "speed");
  number(value.visibleSamples, 0, 25, "visible samples", true);
  if (
    value.totalSamples !== 25 ||
    value.reasons.includes("occluded") !== (value.visibleSamples !== 25)
  )
    throw new Error("Inspection sampled visibility disagrees");
}
function validateReport(report) {
  exact(
    report,
    ["format", "version", "sourceVehicleId", "scene", "receipts", "summary"],
    "report",
  );
  if (
    report.format !== "flight-lab-inspection" ||
    report.version !== 1 ||
    typeof report.sourceVehicleId !== "string" ||
    !ID.test(report.sourceVehicleId)
  )
    throw new Error("Unsupported inspection report");
  const scene = validateScene(report.scene),
    targets = new Set(scene.inspectionTargets.map((target) => target.id));
  dense(report.receipts, 100, "receipts");
  const captures = new Set(),
    passedTargets = new Set();
  let lastTick = -1;
  const receipts = report.receipts.map((receipt) => {
    exact(
      receipt,
      [
        "captureId",
        "targetId",
        "sourceVehicleId",
        "worldTick",
        "metadata",
        "sha256",
        "assessment",
      ],
      "receipt",
    );
    if (
      typeof receipt.captureId !== "string" ||
      !UUID.test(receipt.captureId) ||
      captures.has(receipt.captureId) ||
      !targets.has(receipt.targetId) ||
      receipt.sourceVehicleId !== report.sourceVehicleId ||
      typeof receipt.sha256 !== "string" ||
      !/^[0-9a-f]{64}$/.test(receipt.sha256)
    )
      throw new Error("Invalid inspection receipt identity or hash");
    captures.add(receipt.captureId);
    number(receipt.worldTick, 0, 360000, "world tick", true);
    if (receipt.worldTick < lastTick)
      throw new Error("Inspection receipts moved backwards");
    lastTick = receipt.worldTick;
    const metadata = validateCameraMetadata(receipt.metadata);
    exact(receipt.metadata, Object.keys(metadata), "receipt metadata");
    if (
      metadata.captureId !== receipt.captureId ||
      metadata.sourceVehicleId !== receipt.sourceVehicleId ||
      metadata.worldTick !== receipt.worldTick ||
      Math.abs(metadata.time - receipt.worldTick * 0.005) > 1e-7 ||
      !isDeepStrictEqual(metadata.geography, scene.geography)
    )
      throw new Error("Inspection receipt metadata disagrees with its context");
    checkAssessment(receipt.assessment, receipt.targetId);
    if (receipt.assessment.passed) passedTargets.add(receipt.targetId);
    return { ...receipt, metadata };
  });
  exact(report.summary, ["passed", "total", "attempts", "complete"], "summary");
  const summary = {
    passed: passedTargets.size,
    total: targets.size,
    attempts: receipts.length,
    complete: targets.size > 0 && passedTargets.size === targets.size,
  };
  if (!isDeepStrictEqual(report.summary, summary))
    throw new Error("Inspection report summary disagrees with receipts");
  return receipts;
}

// Checks exported file correspondence, not renderer authenticity or real-world inspection accuracy.
export async function verifyInspectionReport(report, datasetJsonText) {
  const receipts = validateReport(report),
    frames = parseCameraDataset(datasetJsonText);
  const byId = new Map(frames.map((frame) => [frame.captureId, frame]));
  const missingCaptureIds = [],
    mismatchedCaptureIds = [];
  let matched = 0;
  for (const receipt of receipts) {
    const frame = byId.get(receipt.captureId);
    if (!frame) {
      missingCaptureIds.push(receipt.captureId);
      continue;
    }
    if (
      !isDeepStrictEqual(validateCameraMetadata(frame), receipt.metadata) ||
      (await hashCameraFrame(frame)) !== receipt.sha256
    ) {
      mismatchedCaptureIds.push(receipt.captureId);
      continue;
    }
    matched++;
  }
  return {
    passed:
      receipts.length > 0 &&
      missingCaptureIds.length === 0 &&
      mismatchedCaptureIds.length === 0,
    sourceVehicleId: report.sourceVehicleId,
    receipts: receipts.length,
    matched,
    missingCaptureIds,
    mismatchedCaptureIds,
  };
}
async function readBounded(path, maximum) {
  const chunks = [];
  let size = 0;
  for await (const chunk of createReadStream(path, { highWaterMark: 65536 })) {
    size += chunk.length;
    if (size > maximum)
      throw new Error(`Inspection input exceeds ${maximum} bytes`);
    chunks.push(chunk);
  }
  return Buffer.concat(chunks, size).toString("utf8");
}
if (
  process.argv[1] &&
  import.meta.url === pathToFileURL(process.argv[1]).href
) {
  try {
    if (process.argv.length !== 4)
      throw new Error(
        "Usage: node tools/verify-inspection.mjs /path/to/inspection-report.json /path/to/camera-dataset.json",
      );
    const result = await verifyInspectionReport(
      JSON.parse(await readBounded(process.argv[2], 2 * 1024 * 1024)),
      await readBounded(process.argv[3], CAMERA_DATASET_MAX_IMPORT_BYTES),
    );
    console.log(JSON.stringify(result, null, 2));
    if (!result.passed) process.exitCode = 1;
  } catch (error) {
    console.error(error.message);
    process.exitCode = 1;
  }
}
