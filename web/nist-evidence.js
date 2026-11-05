import { parseNistReport } from './nist.js';
import { hashCameraFrame, validateCameraMetadata, CAMERA_DATASET_MAX_FRAMES } from './camera.js';

// Keep only matched IDs; complete trials can span several bounded camera datasets.
export function createNistEvidenceVerifier(report) {
  const artifact = parseNistReport(typeof report === 'string' ? report : JSON.stringify(report));
  if (artifact.session.phase === 'idle' || !artifact.session.captures.length)
    throw new Error('尚無 NIST 拍攝證據可核對。');
  const expected = new Map(artifact.session.captures.map(c => [c.captureId, c]));
  let matched = new Set();
  function result() {
    const requiredCaptureIds = [...expected.keys()];
    const matchedCaptureIds = requiredCaptureIds.filter(id => matched.has(id));
    const missingCaptureIds = requiredCaptureIds.filter(id => !matched.has(id));
    return { sessionId: artifact.session.sessionId, requiredCaptureIds, matchedCaptureIds, missingCaptureIds, complete: missingCaptureIds.length === 0, allAlignmentsCaptured: requiredCaptureIds.length === 36, assessment: 'manual' };
  }
  return {
    result,
    async addFrames(frames) {
      if (!Array.isArray(frames) || frames.length > CAMERA_DATASET_MAX_FRAMES) throw new Error(`每份影像資料卷最多 ${CAMERA_DATASET_MAX_FRAMES} 張。`);
      const next = new Set(matched);
      for (const frame of frames) {
        const receipt = expected.get(frame?.captureId);
        if (!receipt) continue;
        if (JSON.stringify(validateCameraMetadata(frame)) !== JSON.stringify(validateCameraMetadata(receipt.metadata)))
          throw new Error(`影像 ${frame.captureId} 的來源或拍攝資料不符。`);
        if (await hashCameraFrame(frame) !== receipt.sha256)
          throw new Error(`影像 ${frame.captureId} 的內容與拍攝收據不符。`);
        next.add(frame.captureId);
      }
      matched = next;
      return result();
    },
  };
}
