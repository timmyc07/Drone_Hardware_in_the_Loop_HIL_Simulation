import { createReadStream } from 'node:fs';
import { pathToFileURL } from 'node:url';
import { parseCameraDataset, CAMERA_DATASET_MAX_IMPORT_BYTES } from '../web/camera.js';
import { createNistEvidenceVerifier } from '../web/nist-evidence.js';

async function readBounded(path, maxBytes) {
  const chunks = [];
  let size = 0;
  for await (const chunk of createReadStream(path)) {
    size += chunk.length;
    if (size > maxBytes) throw new Error('檔案超過匯入上限。');
    chunks.push(chunk);
  }
  return Buffer.concat(chunks).toString('utf8');
}

export async function verifyNistFiles(reportPath, datasetPaths) {
  if (!Array.isArray(datasetPaths) || datasetPaths.length < 1 || datasetPaths.length > 36)
    throw new Error('請提供 1 至 36 份影像資料卷。');
  const verifier = createNistEvidenceVerifier(await readBounded(reportPath, 512 * 1024));
  for (const path of datasetPaths)
    await verifier.addFrames(parseCameraDataset(await readBounded(path, CAMERA_DATASET_MAX_IMPORT_BYTES)));
  return verifier.result();
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  const [report, ...datasets] = process.argv.slice(2);
  if (!report || !datasets.length) throw new Error('用法：node tools/verify-nist.mjs 報告.json 影像卷1.json [影像卷2.json ...]');
  const result = await verifyNistFiles(report, datasets);
  console.log(JSON.stringify(result, null, 2));
  if (!result.complete) process.exitCode = 1;
}
