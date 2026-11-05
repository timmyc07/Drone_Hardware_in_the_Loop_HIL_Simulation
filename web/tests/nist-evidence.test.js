import test from 'node:test';
import assert from 'node:assert/strict';
import { randomUUID } from 'node:crypto';
import { writeFile, rm, truncate } from 'node:fs/promises';
import { createLocalDirectory } from '../../companion/local-directory.js';
import { spawnSync } from 'node:child_process';
import { createWorld, defaultWorldConfig } from '../world.js';
import { createNistSession, nistReportToJson } from '../nist.js';
import { cameraMetadata, hashCameraFrame, cameraDatasetToJson, DEFAULT_CAMERA_CONFIG, CAMERA_DATASET_MAX_IMPORT_BYTES } from '../camera.js';
import { createNistEvidenceVerifier } from '../nist-evidence.js';
import { verifyNistFiles } from '../../tools/verify-nist.mjs';

// Synthetic pixels and world state exercise integrity checks, not GUI capture.
async function fixture() {
  const config = structuredClone(defaultWorldConfig({ startOnGround: true })); config.scene.nistLane = true;
  const world = createWorld(config), frame = world.snapshot(), vehicleId = frame.vehicles[0].id, session = createNistSession();
  const apply = (type, value) => session.apply({ type, value, vehicleId, tick: frame.tick, time: frame.time }, frame, config.scene);
  apply('nistStart', { sessionId: randomUUID(), durationSeconds: 600 });
  const images = [];
  for (const slotId of ['s01-1', 's01-2']) {
    const captureId = randomUUID(), metadata = cameraMetadata(frame.vehicles[0].snapshot, { ...DEFAULT_CAMERA_CONFIG, width: 64, height: 48 }, null, { captureId, sourceVehicleId: vehicleId, worldTick: frame.tick });
    const n = metadata.width * metadata.height;
    const image = { ...metadata, rgbRgba: new Uint8Array(n * 4), depthM: new Float32Array(n), classes: new Uint8Array(n) };
    apply('nistCapture', { slotId, captureId, metadata, sha256: await hashCameraFrame(image) }); images.push(image);
  }
  apply('nistEnd', { reason: 'manual' });
  return { images, report: nistReportToJson(session.getState()) };
}
test('matching across volumes deduplicates frames and reports missing alignments', async () => {
  const { images, report } = await fixture(), verifier = createNistEvidenceVerifier(report);
  let result = await verifier.addFrames([images[0], images[0]]);
  assert.equal(result.matchedCaptureIds.length, 1); assert.equal(result.complete, false); assert.equal(result.allAlignmentsCaptured, false);
  result = await verifier.addFrames([images[1], images[0]]);
  assert.equal(result.matchedCaptureIds.length, 2); assert.equal(result.complete, true); assert.equal(result.allAlignmentsCaptured, false); assert.equal(result.assessment, 'manual');
});
test('bad hash or pose rejects the entire volume without partial matches', async () => {
  const { images, report } = await fixture();
  for (const mutate of [image => { image.rgbRgba[0] = 1; }, image => { image.pose.positionNed[0] += 1; }]) {
    const verifier = createNistEvidenceVerifier(report), bad = structuredClone(images[1]); mutate(bad);
    await assert.rejects(verifier.addFrames([images[0], bad]));
    assert.deepEqual(verifier.result().matchedCaptureIds, []);
  }
});
test('file verifier handles separate bounded volumes, missing evidence and missing files', async t => {
  const { images, report } = await fixture();
  const dir = await createLocalDirectory('nist-evidence-'); t.after(() => rm(dir, { recursive: true, force: true }));
  const reportPath = `${dir}/report.json`, a = `${dir}/a.json`, b = `${dir}/b.json`;
  await Promise.all([writeFile(reportPath, report), writeFile(a, cameraDatasetToJson([images[0]])), writeFile(b, cameraDatasetToJson([images[1]]))]);
  assert.equal((await verifyNistFiles(reportPath, [a, b])).complete, true);
  assert.equal((await verifyNistFiles(reportPath, [a])).complete, false);
  await assert.rejects(verifyNistFiles(reportPath, [`${dir}/missing.json`]));
  await assert.rejects(verifyNistFiles(reportPath, Array(37).fill(a)));
  const oversized = `${dir}/oversized.json`; await writeFile(oversized, '');
  await truncate(oversized, CAMERA_DATASET_MAX_IMPORT_BYTES + 1);
  await assert.rejects(verifyNistFiles(reportPath, [oversized]), /上限/);
  await truncate(oversized, 512 * 1024 + 1);
  await assert.rejects(verifyNistFiles(oversized, [a]), /上限/);
  const cli = spawnSync(process.execPath, ['tools/verify-nist.mjs', reportPath, a], { encoding: 'utf8' });
  assert.equal(cli.status, 1); assert.equal(JSON.parse(cli.stdout).complete, false);
});

test('idle and zero-capture reports cannot claim evidence verification', () => {
  const session = createNistSession();
  assert.throws(() => createNistEvidenceVerifier(nistReportToJson(session.getState())));
  const config = structuredClone(defaultWorldConfig({ startOnGround: true })); config.scene.nistLane = true;
  const frame = createWorld(config).snapshot(), vehicleId = frame.vehicles[0].id;
  session.apply({ type: 'nistStart', value: { sessionId: randomUUID(), durationSeconds: 600 }, vehicleId, tick: frame.tick, time: frame.time }, frame, config.scene);
  assert.throws(() => createNistEvidenceVerifier(nistReportToJson(session.getState())));
});
