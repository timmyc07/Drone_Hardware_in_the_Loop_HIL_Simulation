import { fromArrayBuffer, BaseDecoder, getDecoder } from "geotiff";
import proj4 from "proj4";
import { f16round } from "@petamoriken/float16";
import { localToGeographic, validateGeography } from "./geography.js";
import { validateTerrain } from "./terrain.js";

// GeoTIFF 3.0.5 / Proj4 2.22.0 installed APIs; version tags checked because Context7 has no version-specific index.
export const DEM_MAX_IMPORT_BYTES = 64 * 1024 * 1024;
const MAX_DECODE_BYTES = 32 * 1024 * 1024,
  MAX_BLOCK_BYTES = 16 * 1024 * 1024;
const requireValue = (ok, message) => {
  if (!ok) throw new Error(message);
};
const plain = (value) =>
  value && [Object.prototype, null].includes(Object.getPrototypeOf(value));
const finite = (v, min, max) => Number.isFinite(v) && v >= min && v <= max;
const integer = (v, min, max) => Number.isInteger(v) && v >= min && v <= max;
const checkAbort = (signal) => signal?.throwIfAborted();
async function checkpoint(signal) {
  checkAbort(signal);
  await new Promise((resolve) => setTimeout(resolve, 0));
  checkAbort(signal);
}
function optionsFrom(value) {
  requireValue(
    plain(value) &&
      Reflect.ownKeys(value).every((key) =>
        [
          "name",
          "radiusM",
          "gridSize",
          "origin",
          "band",
          "heightUnit",
          "verticalDatum",
        ].includes(key),
      ),
    "Invalid DEM import options",
  );
  const o = {
    name: "terrain.tif",
    radiusM: 250,
    gridSize: 65,
    band: 1,
    heightUnit: "auto",
    verticalDatum: "auto",
    ...value,
  };
  requireValue(
    typeof o.name === "string" &&
      o.name.trim() &&
      o.name.length <= 160 &&
      !/[\u0000-\u001f\u007f]/.test(o.name),
    "Invalid DEM filename",
  );
  requireValue(
    [33, 65, 129].includes(o.gridSize) &&
      finite(o.radiusM, 1, 500) &&
      finite((2 * o.radiusM) / (o.gridSize - 1), 1, 50),
    "DEM radius/grid requires 1..50 meter cells within a 500 meter radius",
  );
  requireValue(
    integer(o.band, 1, 8) &&
      ["auto", "m", "ft", "us-ft"].includes(o.heightUnit) &&
      ["auto", "ellipsoidal", "orthometric", "unspecified"].includes(
        o.verticalDatum,
      ),
    "Invalid DEM band, height unit or vertical datum",
  );
  if (o.origin !== undefined)
    requireValue(
      plain(o.origin) &&
        Reflect.ownKeys(o.origin).sort().join(",") ===
          "latitudeDeg,longitudeDeg" &&
        finite(o.origin.latitudeDeg, -85, 85) &&
        finite(o.origin.longitudeDeg, -180, 180),
      "Invalid DEM geographic origin",
    );
  return o;
}
function preflight(buffer) {
  requireValue(
    buffer instanceof ArrayBuffer &&
      buffer.byteLength >= 8 &&
      buffer.byteLength <= DEM_MAX_IMPORT_BYTES,
    "DEM file must be an ArrayBuffer of at most 64 MiB",
  );
  const view = new DataView(buffer),
    marker = view.getUint16(0),
    little = marker === 0x4949;
  requireValue(little || marker === 0x4d4d, "Invalid TIFF byte order");
  requireValue(
    view.getUint16(2, little) === 42,
    "Only classic TIFF is supported; BigTIFF must be converted first",
  );
  const offset = view.getUint32(4, little);
  requireValue(
    offset >= 8 && offset + 2 <= buffer.byteLength,
    "Invalid TIFF directory offset",
  );
  const count = view.getUint16(offset, little);
  requireValue(
    count > 0 &&
      count <= 128 &&
      offset + 2 + count * 12 + 4 <= buffer.byteLength,
    "Invalid or oversized TIFF directory",
  );
  const sizes = {
    1: 1,
    2: 1,
    3: 2,
    4: 4,
    5: 8,
    6: 1,
    7: 1,
    8: 2,
    9: 4,
    10: 8,
    11: 4,
    12: 8,
    13: 4,
    16: 8,
    17: 8,
    18: 8,
  };
  const tags = new Set();
  let total = 0;
  for (let i = 0; i < count; i++) {
    const at = offset + 2 + i * 12,
      tag = view.getUint16(at, little),
      type = view.getUint16(at + 2, little);
    const length = view.getUint32(at + 4, little) * sizes[type];
    requireValue(
      !tags.has(tag) && Number.isSafeInteger(length) && length >= 0,
      "Invalid TIFF field",
    );
    tags.add(tag);
    total += length;
    requireValue(total <= 1024 * 1024, "TIFF directory metadata exceeds 1 MiB");
    if (length > 4) {
      const start = view.getUint32(at + 8, little);
      requireValue(
        start >= 8 && start + length <= buffer.byteLength,
        "TIFF metadata points outside the file",
      );
    }
  }
}
function affine(image) {
  const fd = image.fileDirectory,
    matrix = fd.getValue("ModelTransformation"),
    ties = fd.getValue("ModelTiepoint"),
    scale = fd.getValue("ModelPixelScale");
  let a, b, c, d, x, y;
  if (matrix) {
    requireValue(
      matrix.length === 16 &&
        Array.from(matrix).every(Number.isFinite) &&
        [2, 6, 8, 9, 12, 13, 14].every((i) => matrix[i] === 0) &&
        matrix[15] === 1,
      "Unsupported non-2D GeoTIFF transformation",
    );
    requireValue(!ties && !scale, "Ambiguous GeoTIFF affine metadata");
    [a, b, x, c, d, y] = [
      matrix[0],
      matrix[1],
      matrix[3],
      matrix[4],
      matrix[5],
      matrix[7],
    ];
  } else {
    requireValue(
      ties?.length >= 6 &&
        ties.length % 6 === 0 &&
        scale?.length === 3 &&
        Array.from(ties).every(Number.isFinite) &&
        Array.from(scale).every(Number.isFinite) &&
        scale[0] > 0 &&
        scale[1] > 0,
      "GeoTIFF requires a valid affine transform",
    );
    [a, b, c, d] = [scale[0], 0, 0, -scale[1]];
    [x, y] = [ties[3] - ties[0] * a, ties[4] - ties[1] * d];
    for (let i = 0; i < ties.length; i += 6)
      requireValue(
        Math.abs(a * ties[i] + x - ties[i + 3]) <=
          1e-7 * Math.max(1, Math.abs(ties[i + 3])) &&
          Math.abs(d * ties[i + 1] + y - ties[i + 4]) <=
            1e-7 * Math.max(1, Math.abs(ties[i + 4])),
        "GeoTIFF tiepoints do not define one affine transform",
      );
  }
  const det = a * d - b * c;
  requireValue(
    Number.isFinite(det) && det !== 0,
    "Singular GeoTIFF affine transform",
  );
  return {
    forward: (i, j) => [a * i + b * j + x, c * i + d * j + y],
    inverse: (px, py) => [
      (d * (px - x) - b * (py - y)) / det,
      (-c * (px - x) + a * (py - y)) / det,
    ],
  };
}
function sourceCrs(keys) {
  const code = keys?.ProjectedCSTypeGeoKey ?? keys?.GeographicTypeGeoKey;
  requireValue(
    code === 4326 ||
      code === 3857 ||
      integer(code, 32601, 32660) ||
      integer(code, 32701, 32760),
    "Unsupported DEM CRS; use WGS84 geographic, Web Mercator or WGS84 UTM",
  );
  const crs = `EPSG:${code}`;
  requireValue(
    Boolean(proj4.defs(crs)),
    "DEM CRS is not available in installed Proj4",
  );
  if (keys?.ProjectedCSTypeGeoKey !== undefined)
    requireValue(
      keys.ProjLinearUnitsGeoKey === undefined ||
        keys.ProjLinearUnitsGeoKey === 9001,
      "Projected DEM coordinates must be meters",
    );
  else
    requireValue(
      keys.GeogAngularUnitsGeoKey === undefined ||
        keys.GeogAngularUnitsGeoKey === 9102,
      "Geographic DEM coordinates must be degrees",
    );
  return crs;
}
function metadataNumber(metadata, key, fallback) {
  if (metadata[key] === undefined) return fallback;
  const value = Number(metadata[key]);
  requireValue(
    String(metadata[key]).trim() && Number.isFinite(value),
    `Invalid DEM ${key} metadata`,
  );
  return value;
}
function heightMetadata(keys, metadata, options) {
  const units = new Map([
    [9001, "m"],
    [9002, "ft"],
    [9003, "us-ft"],
  ]);
  const names = new Map([
    ["m", "m"],
    ["meter", "m"],
    ["meters", "m"],
    ["metre", "m"],
    ["metres", "m"],
    ["ft", "ft"],
    ["foot", "ft"],
    ["feet", "ft"],
    ["us-ft", "us-ft"],
    ["us survey foot", "us-ft"],
    ["us_survey_foot", "us-ft"],
  ]);
  const geographicUnit = units.get(keys.VerticalUnitsGeoKey);
  const bandUnit = names.get(
    String(metadata.UNITTYPE ?? "")
      .trim()
      .toLowerCase(),
  );
  if (options.heightUnit === "auto") {
    requireValue(
      keys.VerticalUnitsGeoKey === undefined || geographicUnit,
      "Unknown DEM vertical unit metadata; explicitly choose m, ft or us-ft",
    );
    requireValue(
      !Object.hasOwn(metadata, "UNITTYPE") || bandUnit,
      "Unknown DEM band height unit; explicitly choose m, ft or us-ft",
    );
    requireValue(
      !geographicUnit || !bandUnit || geographicUnit === bandUnit,
      "Conflicting DEM height unit metadata; explicitly choose m, ft or us-ft",
    );
  }
  const heightUnit =
    options.heightUnit === "auto"
      ? (bandUnit ?? geographicUnit)
      : options.heightUnit;
  requireValue(
    heightUnit,
    "DEM height unit is unknown; explicitly choose m, ft or us-ft",
  );
  const explicitDatum = String(metadata.VERTICAL_DATUM ?? "")
    .trim()
    .toLowerCase();
  const detectedDatum = ["ellipsoidal", "orthometric"].includes(explicitDatum)
    ? explicitDatum
    : [5703, 5773, 3855, 5714].includes(keys.VerticalCSTypeGeoKey)
      ? "orthometric"
      : undefined;
  const verticalDatum =
    options.verticalDatum === "auto" ? detectedDatum : options.verticalDatum;
  requireValue(
    verticalDatum,
    "DEM vertical datum is unknown; explicitly choose ellipsoidal, orthometric or unspecified",
  );
  const scale = metadataNumber(metadata, "SCALE", 1),
    offset = metadataNumber(metadata, "OFFSET", 0);
  requireValue(
    finite(scale, -1e6, 1e6) && scale !== 0 && finite(offset, -1e9, 1e9),
    "Invalid DEM height scale or offset",
  );
  return {
    heightUnit,
    verticalDatum,
    scale,
    offset,
    metersPerUnit: { m: 1, ft: 0.3048, "us-ft": 1200 / 3937 }[heightUnit],
  };
}
/*! @license
 * LZW / PackBits adapted from geotiff.js v3.0.5 compression decoders:
 * https://github.com/geotiffjs/geotiff.js/tree/v3.0.5/src/compression
 * Changes: fixed-size outputs, bounds/cycle/truncation checks, cancellation, PackBits -128 NOP.
 * The MIT License (MIT), Copyright (c) 2015 EOX IT Services GmbH
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
async function boundedLzw(buffer, limit, signal) {
  const input = new Uint8Array(buffer),
    dictionaryIndex = new Uint16Array(4096),
    dictionaryChar = new Uint8Array(4096);
  const reversed = new Uint8Array(4096),
    output = new Uint8Array(limit);
  for (let i = 0; i <= 257; i++) {
    dictionaryIndex[i] = 4096;
    dictionaryChar[i] = i;
  }
  let dictionaryLength = 258,
    bits = 9,
    position = 0,
    used = 0,
    oldCode,
    steps = 0;
  function next() {
    requireValue(position + bits <= input.length * 8, "Truncated LZW block");
    let code = 0;
    for (let bit = 0; bit < bits; bit++, position++)
      code =
        (code << 1) | ((input[position >>> 3] >>> (7 - (position & 7))) & 1);
    return code;
  }
  function dictionary(code) {
    let length = 0;
    for (let i = code; i !== 4096; i = dictionaryIndex[i]) {
      requireValue(
        i < dictionaryLength && length < 4096,
        "Invalid or cyclic LZW dictionary",
      );
      reversed[length++] = dictionaryChar[i];
    }
    return length;
  }
  function append(length, repeatFirst = false) {
    requireValue(
      used + length + (repeatFirst ? 1 : 0) <= limit,
      "LZW block exceeds its declared dimensions",
    );
    const first = reversed[length - 1];
    for (let i = length - 1; i >= 0; i--) output[used++] = reversed[i];
    if (repeatFirst) output[used++] = first;
    return first;
  }
  function add(code, first) {
    requireValue(
      dictionaryLength < 4096,
      "LZW dictionary exceeds its maximum size",
    );
    dictionaryIndex[dictionaryLength] = code;
    dictionaryChar[dictionaryLength++] = first;
  }
  let code = next();
  while (code !== 257) {
    if (++steps % 4096 === 0) await checkpoint(signal);
    if (code === 256) {
      dictionaryLength = 258;
      bits = 9;
      oldCode = undefined;
      do {
        code = next();
        if (++steps % 4096 === 0) await checkpoint(signal);
      } while (code === 256);
      if (code === 257) break;
      requireValue(code < 256, "Invalid first LZW dictionary code");
      append(dictionary(code));
      oldCode = code;
    } else if (code < dictionaryLength) {
      const first = append(dictionary(code));
      if (oldCode !== undefined) add(oldCode, first);
      oldCode = code;
    } else {
      requireValue(
        code === dictionaryLength && oldCode !== undefined,
        "Invalid LZW dictionary code",
      );
      const first = append(dictionary(oldCode), true);
      add(oldCode, first);
      oldCode = code;
    }
    if (dictionaryLength + 1 >= 1 << bits) {
      if (bits === 12) oldCode = undefined;
      else bits++;
    }
    code = next();
  }
  checkAbort(signal);
  return output.buffer.slice(0, used);
}
async function boundedPackBits(buffer, limit, signal) {
  const input = new Uint8Array(buffer),
    output = new Uint8Array(limit);
  let used = 0;
  for (let i = 0; i < input.length;) {
    if (i % 8192 === 0) await checkpoint(signal);
    const header = (input[i++] << 24) >> 24;
    if (header === -128) continue;
    const count = header < 0 ? 1 - header : 1 + header;
    requireValue(
      used + count <= limit,
      "PackBits block exceeds its declared dimensions",
    );
    if (header < 0) {
      requireValue(i < input.length, "Truncated PackBits repeat");
      output.fill(input[i++], used, used + count);
    } else {
      requireValue(i + count <= input.length, "Truncated PackBits literal");
      output.set(input.subarray(i, i + count), used);
      i += count;
    }
    used += count;
  }
  checkAbort(signal);
  return output.buffer.slice(0, used);
}
async function decodeBudget(image, window, band, fileBytes, signal, budget) {
  const fd = image.fileDirectory,
    samples = image.getSamplesPerPixel(),
    planar = fd.getValue("PlanarConfiguration") ?? 1;
  const bits = Array.from(await fd.loadValue("BitsPerSample"));
  requireValue(
    integer(samples, 1, 8) &&
      band < samples &&
      [1, 2].includes(planar) &&
      bits.length === samples &&
      bits.every((v) => [8, 16, 32, 64].includes(v)),
    "Unsupported DEM sample layout",
  );
  const compression = fd.getValue("Compression") ?? 1;
  requireValue(
    [1, 5, 8, 32946, 32773, 50000].includes(compression),
    "Unsupported DEM compression; use uncompressed, Deflate, LZW, PackBits or ZSTD",
  );
  const formats = fd.getValue("SampleFormat");
  requireValue(
    !formats || Array.from(formats).every((v) => [1, 2, 3].includes(v)),
    "Unsupported DEM numeric sample format",
  );
  const width = image.getTileWidth(),
    height = image.getTileHeight();
  requireValue(
    integer(width, 1, 32768) && integer(height, 1, 32768),
    "Invalid DEM tile or strip dimensions",
  );
  const across = Math.ceil(image.getWidth() / width),
    down = Math.ceil(image.getHeight() / height);
  const bytesPerPixel =
    (planar === 1 ? bits.reduce((a, b) => a + b, 0) : bits[band]) / 8;
  const maxBlockBytes = width * height * bytesPerPixel;
  requireValue(
    maxBlockBytes <= MAX_BLOCK_BYTES,
    "DEM tile or strip expands beyond 16 MiB",
  );
  budget.decodedBytes += (window[2] - window[0]) * (window[3] - window[1]) * 8;
  requireValue(
    budget.decodedBytes <= MAX_DECODE_BYTES,
    "DEM sampling window exceeds the 32 MiB decode budget",
  );
  const blockSizes = new Map(),
    blockCounts = [];
  const offsetTag = image.isTiled ? "TileOffsets" : "StripOffsets",
    countTag = image.isTiled ? "TileByteCounts" : "StripByteCounts";
  for (
    let y = Math.floor(window[1] / height);
    y < Math.ceil(window[3] / height);
    y++
  ) {
    for (
      let x = Math.floor(window[0] / width);
      x < Math.ceil(window[2] / width);
      x++
    ) {
      checkAbort(signal);
      requireValue(
        ++budget.blocks <= 1024,
        "DEM sampling window requires too many blocks",
      );
      const index = y * across + x + (planar === 2 ? band * across * down : 0);
      const start = Number(await fd.loadValueIndexed(offsetTag, index)),
        count = Number(await fd.loadValueIndexed(countTag, index));
      requireValue(
        integer(start, 8, fileBytes) &&
          integer(count, 1, MAX_BLOCK_BYTES) &&
          start + count <= fileBytes,
        "DEM block is missing or outside the source file",
      );
      const expected = width * image.getBlockHeight(y) * bytesPerPixel;
      if (!blockSizes.has(count)) blockSizes.set(count, new Set());
      blockSizes.get(count).add(expected);
      blockCounts.push(count);
      requireValue(
        compression !== 1 || count === expected,
        "Uncompressed DEM block length does not match its dimensions",
      );
    }
  }
  budget.decodedBytes += blockCounts.reduce(
    (sum, count) => sum + Math.max(...blockSizes.get(count)),
    0,
  );
  requireValue(
    budget.decodedBytes <= MAX_DECODE_BYTES,
    "DEM blocks exceed the 32 MiB decode budget",
  );
  let zstd;
  if (compression === 50000) {
    const { ZSTDDecoder } = await import("zstddec");
    zstd = new ZSTDDecoder();
    await zstd.init();
    checkAbort(signal);
  }
  // The stock inflater allocates the complete output before checking TIFF dimensions.
  // Stream into a capped buffer instead, retaining GeoTIFF's predictor and raster readers.
  class BoundedDecoder extends BaseDecoder {
    constructor(parameters, limit) {
      super(parameters);
      this.limit = limit;
    }
    async decodeBlock(buffer) {
      const limit = this.limit;
      if (compression === 5) return boundedLzw(buffer, limit, signal);
      if (compression === 32773) return boundedPackBits(buffer, limit, signal);
      // zstddec 0.2.0 uses the supplied destination size, never the untrusted frame's size.
      if (compression === 50000)
        return zstd.decode(new Uint8Array(buffer), limit).buffer;
      const reader = new Blob([buffer])
        .stream()
        .pipeThrough(new DecompressionStream("deflate"))
        .getReader();
      const output = new Uint8Array(limit);
      let used = 0;
      const cancel = () => {
        void reader.cancel(signal.reason).catch(() => {});
      };
      signal?.addEventListener("abort", cancel, { once: true });
      try {
        while (true) {
          checkAbort(signal);
          const { value, done } = await reader.read();
          if (done) break;
          requireValue(
            used + value.byteLength <= limit,
            "Deflate block exceeds its declared dimensions",
          );
          output.set(value, used);
          used += value.byteLength;
        }
        checkAbort(signal);
        return output.buffer.slice(0, used);
      } finally {
        signal?.removeEventListener("abort", cancel);
        await reader.cancel().catch(() => {});
        reader.releaseLock();
      }
    }
  }
  return {
    bindParameters: (_compression, parameters) => ({
      decode: async (buffer) => {
        checkAbort(signal);
        const sizes = blockSizes.get(buffer.byteLength);
        requireValue(sizes, "Unexpected DEM compressed block size");
        const decoder =
          compression === 1
            ? await getDecoder(1, parameters)
            : new BoundedDecoder(parameters, Math.max(...sizes));
        const output = await decoder.decode(buffer);
        requireValue(
          sizes.has(output.byteLength),
          "Decoded DEM block length does not match its declared dimensions",
        );
        checkAbort(signal);
        return output;
      },
    }),
  };
}
export async function importDem(buffer, options = {}, signal) {
  const o = optionsFrom(options);
  checkAbort(signal);
  preflight(buffer);
  // Keep provenance and decoded geometry tied to one immutable byte snapshot across awaits.
  buffer = buffer.slice(0);
  await checkpoint(signal);
  const tiff = await fromArrayBuffer(buffer, signal),
    image = await tiff.getImage();
  const width = image.getWidth(),
    height = image.getHeight();
  requireValue(
    (image.fileDirectory.getValue("Orientation") ?? 1) === 1,
    "Unsupported TIFF orientation; normalize the raster to top-left orientation first",
  );
  requireValue(
    integer(width, 2, 32768) &&
      integer(height, 2, 32768) &&
      width * height <= 100000000,
    "DEM source dimensions exceed limits",
  );
  const keys = image.getGeoKeys();
  requireValue(
    keys && [1, 2].includes(keys.GTRasterTypeGeoKey),
    "DEM must declare pixel-is-area or pixel-is-point",
  );
  const pixelInterpretation = keys.GTRasterTypeGeoKey === 1 ? "area" : "point",
    pixelOffset = pixelInterpretation === "area" ? 0.5 : 0;
  const crs = sourceCrs(keys),
    transform = affine(image),
    projection = proj4("EPSG:4326", crs);
  const bandMetadata = await image.getGDALMetadata(o.band - 1),
    generalMetadata = await image.getGDALMetadata();
  const metadata = Object.fromEntries(
    Object.entries({ ...generalMetadata, ...bandMetadata }).map(
      ([key, value]) => [key.toUpperCase(), value],
    ),
  );
  const heightInfo = heightMetadata(keys, metadata, o);
  const center = projection.inverse(
    transform.forward(
      (width - 1) / 2 + pixelOffset,
      (height - 1) / 2 + pixelOffset,
    ),
  );
  const origin = {
    ...(o.origin ?? { latitudeDeg: center[1], longitudeDeg: center[0] }),
    altitudeM: 0,
  };
  requireValue(
    finite(origin.latitudeDeg, -85, 85) &&
      finite(origin.longitudeDeg, -180, 180),
    "DEM center is outside supported geographic limits",
  );
  const budget = { decodedBytes: 0, blocks: 0 };
  let nodata = image.getGDALNoData();
  if (image.fileDirectory.hasTag("GDAL_NODATA")) {
    const text = String(image.fileDirectory.getValue("GDAL_NODATA"))
      .replace(/\0$/, "")
      .trim();
    requireValue(
      text && (Number.isFinite(Number(text)) || /^nan$/i.test(text)),
      "Invalid DEM NoData metadata",
    );
  }
  if (Number.isFinite(nodata)) {
    const sample = image.getArrayForSample(o.band - 1, 1);
    sample[0] = nodata;
    const format = image.getSampleFormat(o.band - 1),
      bits = image.getBitsPerSample(o.band - 1);
    const stored = format === 3 && bits === 16 ? f16round(nodata) : sample[0];
    requireValue(
      Number.isFinite(stored) && (format === 3 || stored === nodata),
      "DEM NoData is outside the selected band's numeric storage range",
    );
    // The ASCII sentinel must be rounded to the selected band's on-disk numeric precision.
    nodata = stored;
  }
  function pixelAt(latitudeDeg, longitudeDeg) {
    const pixel = transform
      .inverse(...projection.forward([longitudeDeg, latitudeDeg]))
      .map((v) => v - pixelOffset);
    requireValue(
      pixel.every(Number.isFinite) &&
        pixel[0] >= -1e-7 &&
        pixel[0] <= width - 1 + 1e-7 &&
        pixel[1] >= -1e-7 &&
        pixel[1] <= height - 1 + 1e-7,
      "Requested DEM square is not fully covered by source sample centers",
    );
    return [
      Math.max(0, Math.min(width - 1, pixel[0])),
      Math.max(0, Math.min(height - 1, pixel[1])),
    ];
  }
  function windowOf(coordinates) {
    const xs = coordinates.map((p) => p[0]),
      ys = coordinates.map((p) => p[1]);
    return [
      Math.floor(Math.min(...xs)),
      Math.floor(Math.min(...ys)),
      Math.min(width, Math.floor(Math.max(...xs)) + 2),
      Math.min(height, Math.floor(Math.max(...ys)) + 2),
    ];
  }
  async function readElevations(coordinates, window) {
    const pool = await decodeBudget(
      image,
      window,
      o.band - 1,
      buffer.byteLength,
      signal,
      budget,
    );
    await checkpoint(signal);
    const rasters = await image.readRasters({
        window,
        samples: [o.band - 1],
        pool,
        signal,
      }),
      raster = rasters[0],
      rowWidth = window[2] - window[0];
    const elevations = [];
    for (let index = 0; index < coordinates.length; index++) {
      if (index % 1024 === 0) await checkpoint(signal);
      const [x, y] = coordinates[index],
        x0 = Math.floor(x),
        y0 = Math.floor(y),
        x1 = Math.min(width - 1, x0 + 1),
        y1 = Math.min(height - 1, y0 + 1);
      const u = x - x0,
        v = y - y0;
      let sum = 0;
      for (const [px, py, weight] of [
        [x0, y0, (1 - u) * (1 - v)],
        [x1, y0, u * (1 - v)],
        [x0, y1, (1 - u) * v],
        [x1, y1, u * v],
      ]) {
        if (weight === 0) continue;
        const raw = raster[(py - window[1]) * rowWidth + px - window[0]];
        requireValue(
          Number.isFinite(raw) && (nodata === null || raw !== nodata),
          "DEM sampling touches NoData or a nonfinite elevation",
        );
        sum += raw * weight;
      }
      const elevation =
        (sum * heightInfo.scale + heightInfo.offset) * heightInfo.metersPerUnit;
      requireValue(
        Number.isFinite(elevation),
        "Nonfinite DEM elevation after unit conversion",
      );
      elevations.push(elevation);
    }
    return elevations;
  }
  const homePixel = pixelAt(origin.latitudeDeg, origin.longitudeDeg);
  const [home] = await readElevations([homePixel], windowOf([homePixel]));
  requireValue(
    finite(home, -500, 9000),
    "DEM home elevation exceeds geographic origin altitude limits",
  );
  origin.altitudeM = home;
  const coordinates = [];
  const cellSize = (2 * o.radiusM) / (o.gridSize - 1);
  for (let i = 0; i < o.gridSize; i++) {
    if (i % 8 === 0) await checkpoint(signal);
    for (let j = 0; j < o.gridSize; j++) {
      const geo = localToGeographic(
        -o.radiusM + i * cellSize,
        -o.radiusM + j * cellSize,
        origin,
      );
      coordinates.push(
        i === (o.gridSize - 1) / 2 && j === (o.gridSize - 1) / 2
          ? homePixel
          : pixelAt(geo.latitudeDeg, geo.longitudeDeg),
      );
    }
  }
  const window = windowOf(coordinates),
    elevations = await readElevations(coordinates, window);
  const heights = elevations.map((elevation) => elevation - home);
  const digest = await crypto.subtle.digest("SHA-256", buffer);
  checkAbort(signal);
  const sha256 = Array.from(new Uint8Array(digest), (value) =>
    value.toString(16).padStart(2, "0"),
  ).join("");
  const geography = validateGeography({
    origin: {
      ...origin,
      altitudeM: home,
      verticalDatum: heightInfo.verticalDatum,
    },
    source: {
      name: o.name,
      sha256,
      crs,
      width,
      height,
      band: o.band,
      pixelInterpretation,
      heightUnit: heightInfo.heightUnit,
      scale: heightInfo.scale,
      offset: heightInfo.offset,
      resampling: "bilinear",
      window,
    },
  });
  return {
    terrain: validateTerrain({
      kind: "dem",
      north: -o.radiusM,
      east: -o.radiusM,
      cellSize,
      rows: o.gridSize,
      cols: o.gridSize,
      heights,
    }),
    geography,
  };
}
