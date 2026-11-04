import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { createHash } from "node:crypto";
import Worker from "web-worker";
import proj4 from "proj4";
import { importDem, DEM_MAX_IMPORT_BYTES } from "../dem-import.js";
import { localToGeographic } from "../geography.js";
const load = async (name) => {
  const bytes = await readFile(
    new URL(`fixtures/${name}.tif`, import.meta.url),
  );
  return bytes.buffer.slice(
    bytes.byteOffset,
    bytes.byteOffset + bytes.byteLength,
  );
};
const options = { radiusM: 128, gridSize: 33 };
const near = (a, b, eps = 1e-6) =>
  assert.ok(Math.abs(a - b) <= eps, `${a} != ${b}`);
function patchScalar(buffer, tag, value) {
  const copy = buffer.slice(0),
    v = new DataView(copy),
    le = v.getUint16(0) === 0x4949,
    ifd = v.getUint32(4, le),
    count = v.getUint16(ifd, le);
  for (let i = 0; i < count; i++) {
    const at = ifd + 2 + 12 * i;
    if (v.getUint16(at, le) !== tag) continue;
    const type = v.getUint16(at + 2, le);
    if (type === 3) v.setUint16(at + 8, value, le);
    else v.setUint32(at + 8, value, le);
    return copy;
  }
  throw Error("Missing tag");
}
test("real GeoTIFF raster becomes an unflattened DEM with center zero, true origin height and source provenance", async () => {
  const buffer = await load("dem-area"),
    result = await importDem(buffer, { ...options, name: "actual.tif" });
  const { terrain, geography } = result;
  assert.equal(terrain.kind, "dem");
  assert.equal(terrain.rows, 33);
  assert.equal(terrain.cellSize, 8);
  assert.equal(terrain.heights[544], 0);
  near(geography.origin.altitudeM, 144);
  near(geography.origin.latitudeDeg, 25.033);
  near(geography.origin.longitudeDeg, 121.5654);
  assert.equal(
    geography.source.sha256,
    createHash("sha256").update(new Uint8Array(buffer)).digest("hex"),
  );
  assert.equal(geography.source.heightUnit, "m");
  assert.equal(geography.origin.verticalDatum, "orthometric");
  const corner = localToGeographic(-128, -128, geography.origin),
    x = (corner.longitudeDeg - 121.5654) / 0.0001,
    y = -(corner.latitudeDeg - 25.033) / 0.0001;
  near(terrain.heights[0], 0.5 * x + 0.25 * y);
  assert.notEqual(terrain.heights[0], 0);
  assert.ok(geography.source.window[2] - geography.source.window[0] < 65);
});
test("an asynchronous import owns a byte snapshot, so later caller edits cannot change geometry or provenance", async () => {
  const buffer = await load("dem-area"),
    hash = createHash("sha256").update(new Uint8Array(buffer)).digest("hex");
  const pending = importDem(buffer, options);
  new Uint8Array(buffer).fill(0);
  const result = await pending;
  assert.equal(result.geography.source.sha256, hash);
  near(result.geography.origin.altitudeM, 144);
});
test("area vs point half-pixel conventions give the same physical terrain; Deflate reads the actual pixels", async () => {
  const area = await importDem(await load("dem-area"), options),
    point = await importDem(await load("dem-point"), options),
    deflate = await importDem(await load("dem-deflate"), options);
  assert.equal(point.geography.source.pixelInterpretation, "point");
  area.terrain.heights.forEach((value, i) =>
    near(value, point.terrain.heights[i], 1e-6),
  );
  assert.deepEqual(deflate.terrain, area.terrain);
  for (const name of ["dem-lzw", "dem-packbits", "dem-zstd"])
    assert.deepEqual(
      (await importDem(await load(name), options)).terrain,
      area.terrain,
    );
});
test("UTM, Web Mercator and a rotated/sheared 2D affine raster retain the geographic center and real slopes", async () => {
  for (const [name, crs] of [
    ["dem-utm", "EPSG:32651"],
    ["dem-mercator", "EPSG:3857"],
    ["dem-affine", "EPSG:32651"],
  ]) {
    const { terrain, geography } = await importDem(await load(name), options);
    assert.equal(geography.source.crs, crs);
    near(geography.origin.latitudeDeg, 25.033, 1e-8);
    near(geography.origin.longitudeDeg, 121.5654, 1e-8);
    near(geography.origin.altitudeM, 144);
    const corner = localToGeographic(-128, -128, geography.origin);
    const from = proj4("EPSG:4326", crs, [
      geography.origin.longitudeDeg,
      geography.origin.latitudeDeg,
    ]);
    const to = proj4("EPSG:4326", crs, [
      corner.longitudeDeg,
      corner.latitudeDeg,
    ]);
    const dx = to[0] - from[0],
      dy = to[1] - from[1];
    const di = name === "dem-affine" ? (10 * dx + 2 * dy) / 102 : dx / 10;
    const dj = name === "dem-affine" ? (dx - 10 * dy) / 102 : -dy / 10;
    near(terrain.heights[0], 0.5 * di + 0.25 * dj);
    assert.equal(terrain.heights[544], 0);
    assert.ok(terrain.heights.some((value) => value > 0));
    assert.ok(terrain.heights.some((value) => value < 0));
  }
});
test("band scale/offset and declared feet are applied before origin subtraction; absent unit/datum require an explicit choice", async () => {
  await assert.rejects(
    importDem(await load("dem-unknown-units"), options),
    /height unit/,
  );
  await assert.rejects(
    importDem(await load("dem-unknown-units"), { ...options, heightUnit: "m" }),
    /vertical datum/,
  );
  const unspecified = await importDem(await load("dem-unknown-units"), {
    ...options,
    heightUnit: "us-ft",
    verticalDatum: "unspecified",
  });
  near(unspecified.geography.origin.altitudeM, (144 * 1200) / 3937);
  assert.equal(unspecified.geography.origin.verticalDatum, "unspecified");
  const feet = await importDem(await load("dem-scale-ft"), options);
  near(feet.geography.origin.altitudeM, (144 * 2 + 3) * 0.3048);
  assert.equal(feet.geography.source.scale, 2);
  assert.equal(feet.geography.source.offset, 3);
  assert.equal(feet.geography.source.heightUnit, "ft");
  assert.equal(feet.geography.origin.verticalDatum, "ellipsoidal");
});
test("auto units reject conflicting or unknown band metadata; explicit choices are recorded and applied", async () => {
  const conflict = await load("dem-unit-conflict"),
    unknown = await load("dem-unit-unknown");
  await assert.rejects(
    importDem(conflict, options),
    /Conflicting.*explicitly choose/,
  );
  await assert.rejects(
    importDem(unknown, options),
    /Unknown DEM band.*explicitly choose/,
  );
  const feet = await importDem(conflict, { ...options, heightUnit: "ft" });
  near(feet.geography.origin.altitudeM, 144 * 0.3048);
  assert.equal(feet.geography.source.heightUnit, "ft");
  const meters = await importDem(unknown, { ...options, heightUnit: "m" });
  near(meters.geography.origin.altitudeM, 144);
  assert.equal(meters.geography.source.heightUnit, "m");
});
test("DEM keeps deep real relief and selects the requested numeric band; NoData outside the sampled window does not poison valid coverage", async () => {
  const deep = await importDem(await load("dem-deep"), options);
  assert.ok(Math.min(...deep.terrain.heights) < -50);
  assert.ok(Math.max(...deep.terrain.heights) > 150);
  const band = await importDem(await load("dem-bands"), {
    ...options,
    band: 2,
  });
  near(band.geography.origin.altitudeM, 1144);
  assert.equal(band.geography.source.band, 2);
  assert.equal(band.terrain.heights[544], 0);
  const corner = await importDem(await load("dem-nodata-corner"), options),
    area = await importDem(await load("dem-area"), options);
  assert.deepEqual(corner.terrain, area.terrain);
});
test("NoData and absent coverage never become zero ground; malformed/options/resource and oversized decoded blocks reject", async () => {
  await assert.rejects(importDem(await load("dem-nodata"), options), /NoData/);
  // Float32 stores -9999.9 as -9999.900390625. Scale must never disguise that missing sample as a valid home.
  await assert.rejects(
    importDem(await load("dem-fractional-nodata"), options),
    /NoData/,
  );
  const buffer = await load("dem-area");
  await assert.rejects(
    importDem(buffer, {
      ...options,
      origin: { latitudeDeg: 24, longitudeDeg: 120 },
    }),
    /covered/,
  );
  await assert.rejects(
    importDem(buffer, { ...options, gridSize: 129, radiusM: 32 }),
    /cells/,
  );
  await assert.rejects(
    importDem(buffer, { ...options, unexpected: true }),
    /options/,
  );
  await assert.rejects(importDem(buffer, { ...options, band: 2 }), /layout/);
  await assert.rejects(
    importDem(new ArrayBuffer(DEM_MAX_IMPORT_BYTES + 1), options),
    /64 MiB/,
  );
  await assert.rejects(importDem(new ArrayBuffer(8), options), /byte order/);
  await assert.rejects(
    importDem(
      patchScalar(
        patchScalar(patchScalar(buffer, 256, 32768), 257, 129),
        278,
        129,
      ),
      options,
    ),
    /16 MiB/,
  );
  await assert.rejects(
    importDem(patchScalar(buffer, 259, 7), options),
    /compression/,
  );
  await assert.rejects(
    importDem(patchScalar(buffer, 274, 2), options),
    /orientation/,
  );
  await assert.rejects(
    importDem(await load("dem-deflate-overflow"), options),
    /exceeds its declared/,
  );
  await assert.rejects(
    importDem(await load("dem-zstd-overflow"), options),
    /length does not match/,
  );
  for (const name of ["dem-lzw", "dem-packbits", "dem-zstd"]) {
    const original = await load(name),
      small = patchScalar(
        patchScalar(patchScalar(original, 256, 32), 257, 32),
        278,
        32,
      );
    await assert.rejects(
      importDem(small, { radiusM: 16, gridSize: 33 }),
      /exceeds its declared|length does not match/,
    );
  }
});
test("cancelled import cannot publish a partial scene; the real worker can accept another import afterward", async (t) => {
  const controller = new AbortController();
  controller.abort();
  await assert.rejects(
    importDem(await load("dem-area"), options, controller.signal),
    { name: "AbortError" },
  );
  const worker = new Worker(new URL("../dem-worker.js", import.meta.url), {
    type: "module",
  });
  t.after(() => worker.terminate());
  const receive = () =>
    new Promise((resolve, reject) => {
      const timer = setTimeout(() => reject(Error("Worker timeout")), 5000);
      worker.onmessage = (event) => {
        clearTimeout(timer);
        resolve(event.data);
      };
      worker.onerror = (error) => {
        clearTimeout(timer);
        reject(error);
      };
    });
  const pending = receive(),
    buffer = await load("dem-area");
  worker.postMessage({
    type: "import",
    requestId: 1,
    buffer,
    options: { radiusM: 128, gridSize: 129 },
  });
  worker.postMessage({ type: "cancel", requestId: 1 });
  const cancelled = await pending;
  assert.equal(cancelled.type, "error");
  assert.equal(cancelled.name, "AbortError");
  const completed = receive();
  worker.postMessage({ type: "import", requestId: 2, buffer, options });
  const result = await completed;
  assert.equal(result.type, "result");
  assert.equal(result.requestId, 2);
  assert.equal(result.result.terrain.heights[544], 0);
});
