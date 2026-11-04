import test from "node:test";
import assert from "node:assert/strict";
import {
  localToGeographic,
  geographicToLocal,
  validateGeography,
} from "../geography.js";
const origin = {
  latitudeDeg: 25.033,
  longitudeDeg: 121.5654,
  altitudeM: 20,
  verticalDatum: "orthometric",
};
const source = {
  name: "terrain.tif",
  sha256: "a".repeat(64),
  crs: "EPSG:4326",
  width: 65,
  height: 65,
  band: 1,
  pixelInterpretation: "area",
  heightUnit: "m",
  scale: 1,
  offset: 0,
  resampling: "bilinear",
  window: [0, 0, 65, 65],
};
test("geographic/local transforms share the WGS84 sensor radii and round trip including the date line", () => {
  for (const home of [
    origin,
    { ...origin, longitudeDeg: 179.999 },
    { ...origin, latitudeDeg: -80 },
  ])
    for (const [north, east] of [
      [0, 0],
      [500, -500],
      [-500, 500],
    ]) {
      const position = localToGeographic(north, east, home),
        actual = geographicToLocal(
          position.latitudeDeg,
          position.longitudeDeg,
          home,
        );
      assert.ok(Math.abs(actual.north - north) < 1e-7);
      assert.ok(Math.abs(actual.east - east) < 1e-7);
    }
  const phi = (origin.latitudeDeg * Math.PI) / 180,
    e2 = 6.69437999014e-3,
    d = Math.sqrt(1 - e2 * Math.sin(phi) ** 2),
    meridian = (6378137 * (1 - e2)) / d ** 3;
  assert.equal(
    localToGeographic(500, 0, origin).latitudeDeg,
    origin.latitudeDeg + ((500 / (meridian + 20)) * 180) / Math.PI,
  );
  for (const bad of [NaN, Infinity, "0"])
    assert.throws(() => localToGeographic(bad, 0, origin));
  assert.throws(() => geographicToLocal(91, 0, origin));
  assert.throws(() => localToGeographic(0, 0, { ...origin, latitudeDeg: 86 }));
});
test("geography validates a strict cloned immutable provenance and origin", () => {
  const input = {
      origin: { ...origin },
      source: { ...source, window: [0, 0, 65, 65] },
    },
    result = validateGeography(input);
  const reordered = {
    source: Object.fromEntries(Object.entries(input.source).reverse()),
    origin: Object.fromEntries(Object.entries(input.origin).reverse()),
  };
  assert.equal(
    JSON.stringify(validateGeography(reordered)),
    JSON.stringify(result),
  );
  input.source.window[0] = 2;
  input.origin.altitudeM = 999;
  assert.equal(result.origin.altitudeM, 20);
  assert.equal(result.source.window[0], 0);
  assert.ok(
    Object.isFrozen(result) &&
      Object.isFrozen(result.origin) &&
      Object.isFrozen(result.source.window),
  );
  for (const value of [
    null,
    {},
    { origin, source, extra: 1 },
    { origin: { ...origin, verticalDatum: "auto" }, source },
    { origin, source: { ...source, crs: "EPSG:9999" } },
    { origin, source: { ...source, sha256: "wrong" } },
    { origin, source: { ...source, window: Array(4) } },
    { origin, source: { ...source, window: [0, 0, 66, 65] } },
    { origin, source: { ...source, scale: 0 } },
    { origin: { ...origin, altitudeM: Infinity }, source },
  ])
    assert.throws(() => validateGeography(value));
});
