// Reproducible local GeoTIFF fixtures, generated with installed geotiff 3.0.5; no downloaded terrain.
import { writeArrayBuffer } from "geotiff";
import proj4 from "proj4";
import { deflateSync, zstdCompressSync } from "node:zlib";
import { writeFile } from "node:fs/promises";
import { execFileSync } from "node:child_process";
const directory = new URL(".", import.meta.url),
  size = 65,
  latitude = 25.033,
  longitude = 121.5654;
const values = Float32Array.from(
  { length: size * size },
  (_, index) => 120 + 0.5 * (index % size) + 0.25 * Math.floor(index / size),
);
function make({
  point = false,
  crs = 4326,
  rotated = false,
  nodata = false,
  metadata = false,
  missing = false,
  deep = false,
  bands = 1,
  bandUnit,
} = {}) {
  const keys = {
    width: size,
    height: size,
    Orientation: 1,
    GTModelTypeGeoKey: crs === 4326 ? 2 : 1,
    GTRasterTypeGeoKey: point ? 2 : 1,
  };
  if (!missing)
    Object.assign(keys, {
      VerticalUnitsGeoKey: 9001,
      VerticalCSTypeGeoKey: 5773,
    });
  if (crs === 4326)
    Object.assign(keys, {
      GeographicTypeGeoKey: 4326,
      GeogAngularUnitsGeoKey: 9102,
    });
  else
    Object.assign(keys, {
      ProjectedCSTypeGeoKey: crs,
      ProjLinearUnitsGeoKey: 9001,
    });
  const center =
    crs === 4326
      ? [longitude, latitude]
      : proj4("EPSG:4326", `EPSG:${crs}`, [longitude, latitude]);
  const half = (size - 1) / 2 + (point ? 0 : 0.5),
    step = crs === 4326 ? 0.0001 : 10;
  if (rotated) {
    const a = 10,
      b = 2,
      c = 1,
      d = -10;
    keys.ModelTransformation = [
      a,
      b,
      0,
      center[0] - (a + b) * half,
      c,
      d,
      0,
      center[1] - (c + d) * half,
      0,
      0,
      1,
      0,
      0,
      0,
      0,
      1,
    ];
  } else
    Object.assign(keys, {
      ModelPixelScale: [step, step, 0],
      ModelTiepoint: [
        0,
        0,
        0,
        center[0] - half * step,
        center[1] + half * step,
        0,
      ],
    });
  const raster = new Float32Array(size * size * bands);
  for (let i = 0; i < values.length; i++)
    for (let band = 0; band < bands; band++)
      raster[i * bands + band] =
        (deep ? 800 + 10 * (i % size) + 10 * Math.floor(i / size) : values[i]) +
        band * 1000;
  if (nodata) {
    const sentinel = nodata === "fractional" ? -9999.9 : -9999;
    raster[nodata === "corner" ? 0 : 32 * size + 32] = sentinel;
    keys.GDAL_NODATA = String(sentinel);
  }
  let xml;
  if (metadata)
    xml =
      '<GDALMetadata><Item name="SCALE" sample="0" role="scale">2</Item><Item name="OFFSET" sample="0" role="offset">3</Item><Item name="UNITTYPE" sample="0" role="unittype">ft</Item><Item name="VERTICAL_DATUM">ellipsoidal</Item></GDALMetadata>';
  if (nodata === "fractional")
    xml =
      '<GDALMetadata><Item name="SCALE" sample="0">0.001</Item></GDALMetadata>';
  if (bandUnit !== undefined)
    xml = `<GDALMetadata><Item name="UNITTYPE" sample="0">${bandUnit}</Item></GDALMetadata>`;
  const buffer = writeArrayBuffer(raster, keys);
  return xml ? appendAsciiTag(buffer, 42112, xml) : buffer;
}
function appendAsciiTag(buffer, tag, text) {
  const input = new DataView(buffer),
    little = input.getUint16(0) === 0x4949,
    old = input.getUint32(4, little),
    count = input.getUint16(old, little);
  const encoded = new TextEncoder().encode(text + "\0"),
    offset = buffer.byteLength,
    newData = offset + 2 + (count + 1) * 12 + 4;
  const result = new Uint8Array(newData + encoded.length);
  result.set(new Uint8Array(buffer));
  const view = new DataView(result.buffer);
  view.setUint32(4, offset, little);
  view.setUint16(offset, count + 1, little);
  result.set(new Uint8Array(buffer, old + 2, count * 12), offset + 2);
  const at = offset + 2 + count * 12;
  view.setUint16(at, tag, little);
  view.setUint16(at + 2, 2, little);
  view.setUint32(at + 4, encoded.length, little);
  view.setUint32(at + 8, newData, little);
  result.set(encoded, newData);
  return result.buffer;
}
export function patchScalar(buffer, tag, value) {
  const copy = buffer.slice(0),
    view = new DataView(copy),
    little = view.getUint16(0) === 0x4949,
    ifd = view.getUint32(4, little),
    count = view.getUint16(ifd, little);
  for (let i = 0; i < count; i++) {
    const at = ifd + 2 + 12 * i;
    if (view.getUint16(at, little) !== tag) continue;
    const type = view.getUint16(at + 2, little),
      length = view.getUint32(at + 4, little);
    if (length !== 1 || ![3, 4].includes(type))
      throw Error("Not an inline scalar");
    if (type === 3) view.setUint16(at + 8, value, little);
    else view.setUint32(at + 8, value, little);
    return copy;
  }
  throw Error("Missing TIFF tag " + tag);
}
function compressed(buffer, payload, compression = 8) {
  let header = patchScalar(buffer, 259, compression),
    view = new DataView(header),
    little = view.getUint16(0) === 0x4949,
    ifd = view.getUint32(4, little),
    count = view.getUint16(ifd, little),
    start;
  for (let i = 0; i < count; i++) {
    const at = ifd + 2 + 12 * i;
    if (view.getUint16(at, little) === 273)
      start = view.getUint32(at + 8, little);
  }
  const data = payload ?? new Uint8Array(buffer, start),
    encoded =
      compression === 50000 ? zstdCompressSync(data) : deflateSync(data);
  header = patchScalar(header, 279, encoded.byteLength);
  const out = new Uint8Array(start + encoded.byteLength);
  out.set(new Uint8Array(header, 0, start));
  out.set(encoded, start);
  return out.buffer;
}
for (const [name, options] of Object.entries({
  "dem-area": {},
  "dem-point": { point: true },
  "dem-utm": { crs: 32651 },
  "dem-mercator": { crs: 3857 },
  "dem-affine": { crs: 32651, rotated: true },
  "dem-nodata": { nodata: true },
  "dem-fractional-nodata": { nodata: "fractional" },
  "dem-nodata-corner": { nodata: "corner" },
  "dem-deep": { deep: true },
  "dem-bands": { bands: 2 },
  "dem-unknown-units": { missing: true },
  "dem-scale-ft": { metadata: true, missing: true },
  "dem-unit-conflict": { bandUnit: "ft" },
  "dem-unit-unknown": { bandUnit: "yards" },
}))
  await writeFile(
    new URL(name + ".tif", directory),
    new Uint8Array(make(options)),
  );
await writeFile(
  new URL("dem-deflate.tif", directory),
  new Uint8Array(compressed(make())),
);
await writeFile(
  new URL("dem-deflate-overflow.tif", directory),
  new Uint8Array(compressed(make(), new Uint8Array(4 * 1024 * 1024))),
);

// Pillow's libtiff encoder provides independent genuine LZW/PackBits bitstreams.
execFileSync("/usr/bin/python3", [
  "-c",
  `from PIL import Image
from pathlib import Path
p=Path(${JSON.stringify(new URL(".", import.meta.url).pathname)})
image=Image.open(p/'dem-area.tif')
for name,codec in [('dem-lzw','tiff_lzw'),('dem-packbits','packbits')]:
 image.save(p/(name+'.tif'),compression=codec,tiffinfo=image.tag_v2)
`,
]);

await writeFile(
  new URL("dem-zstd.tif", directory),
  new Uint8Array(compressed(make(), undefined, 50000)),
);
await writeFile(
  new URL("dem-zstd-overflow.tif", directory),
  new Uint8Array(compressed(make(), new Uint8Array(4 * 1024 * 1024), 50000)),
);
