const DATUMS = ["ellipsoidal", "orthometric", "unspecified"];
const UNITS = ["m", "ft", "us-ft"];
const finite = (value, min, max) =>
  Number.isFinite(value) && value >= min && value <= max;
const integer = (value, min, max) =>
  Number.isInteger(value) && value >= min && value <= max;
function exact(value, keys, label) {
  if (
    !value ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value)) ||
    Reflect.ownKeys(value).length !== keys.length ||
    Reflect.ownKeys(value).some((key) => !keys.includes(key))
  )
    throw new Error(`Invalid ${label} fields`);
}
function checkOrigin(origin) {
  if (
    !origin ||
    !finite(origin.latitudeDeg, -85, 85) ||
    !finite(origin.longitudeDeg, -180, 180) ||
    !finite(origin.altitudeM, -500, 9000)
  )
    throw new Error("Invalid geographic origin");
}
function radii(origin) {
  checkOrigin(origin);
  const latitude = (origin.latitudeDeg * Math.PI) / 180,
    eccentricity2 = 6.69437999014e-3;
  const denominator = Math.sqrt(1 - eccentricity2 * Math.sin(latitude) ** 2);
  return [
    (6378137 * (1 - eccentricity2)) / denominator ** 3 + origin.altitudeM,
    (6378137 / denominator + origin.altitudeM) * Math.cos(latitude),
  ];
}
export function localToGeographic(north, east, origin) {
  if (!Number.isFinite(north) || !Number.isFinite(east))
    throw new Error("Invalid local coordinates");
  const [meridian, parallel] = radii(origin);
  const latitudeDeg = origin.latitudeDeg + ((north / meridian) * 180) / Math.PI;
  const longitude = origin.longitudeDeg + ((east / parallel) * 180) / Math.PI;
  if (!finite(latitudeDeg, -90, 90) || !Number.isFinite(longitude))
    throw new Error("Geographic coordinate outside latitude limits");
  return {
    latitudeDeg,
    longitudeDeg: ((((longitude + 180) % 360) + 360) % 360) - 180,
  };
}
export function geographicToLocal(latitudeDeg, longitudeDeg, origin) {
  if (!finite(latitudeDeg, -90, 90) || !finite(longitudeDeg, -180, 180))
    throw new Error("Invalid geographic coordinates");
  const [meridian, parallel] = radii(origin);
  const delta =
    ((((longitudeDeg - origin.longitudeDeg + 180) % 360) + 360) % 360) - 180;
  return {
    north: (((latitudeDeg - origin.latitudeDeg) * Math.PI) / 180) * meridian,
    east: ((delta * Math.PI) / 180) * parallel,
  };
}
export function validateGeography(value) {
  exact(value, ["origin", "source"], "geography");
  exact(
    value.origin,
    ["latitudeDeg", "longitudeDeg", "altitudeM", "verticalDatum"],
    "geographic origin",
  );
  checkOrigin(value.origin);
  if (!DATUMS.includes(value.origin.verticalDatum))
    throw new Error("Invalid vertical datum");
  const s = value.source;
  exact(
    s,
    [
      "name",
      "sha256",
      "crs",
      "width",
      "height",
      "band",
      "pixelInterpretation",
      "heightUnit",
      "scale",
      "offset",
      "resampling",
      "window",
    ],
    "DEM source",
  );
  if (
    typeof s.name !== "string" ||
    !s.name.trim() ||
    s.name.length > 160 ||
    /[\u0000-\u001f\u007f]/.test(s.name) ||
    typeof s.sha256 !== "string" ||
    !/^[a-f0-9]{64}$/.test(s.sha256) ||
    typeof s.crs !== "string" ||
    !/^EPSG:(4326|3857|32[67](0[1-9]|[1-5][0-9]|60))$/.test(s.crs) ||
    !integer(s.width, 2, 32768) ||
    !integer(s.height, 2, 32768) ||
    s.width * s.height > 100000000 ||
    !integer(s.band, 1, 8) ||
    !["area", "point"].includes(s.pixelInterpretation) ||
    !UNITS.includes(s.heightUnit) ||
    !finite(s.scale, -1e6, 1e6) ||
    s.scale === 0 ||
    !finite(s.offset, -1e9, 1e9) ||
    s.resampling !== "bilinear" ||
    !Array.isArray(s.window) ||
    s.window.length !== 4 ||
    ![0, 1, 2, 3].every(
      (i) =>
        Object.hasOwn(s.window, i) &&
        integer(s.window[i], 0, i % 2 ? s.height : s.width),
    ) ||
    s.window[2] - s.window[0] < 1 ||
    s.window[3] - s.window[1] < 1
  )
    throw new Error("Invalid DEM source metadata");
  return Object.freeze({
    origin: Object.freeze({
      latitudeDeg: value.origin.latitudeDeg,
      longitudeDeg: value.origin.longitudeDeg,
      altitudeM: value.origin.altitudeM,
      verticalDatum: value.origin.verticalDatum,
    }),
    source: Object.freeze(
      Object.fromEntries(
        [
          "name",
          "sha256",
          "crs",
          "width",
          "height",
          "band",
          "pixelInterpretation",
          "heightUnit",
          "scale",
          "offset",
          "resampling",
          "window",
        ].map((key) => [
          key,
          key === "window" ? Object.freeze([...s.window]) : s[key],
        ]),
      ),
    ),
  });
}
