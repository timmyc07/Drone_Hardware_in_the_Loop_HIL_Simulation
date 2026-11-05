export const DEFAULT_GEOFENCE_ZONES = Object.freeze({ enabled: false, zones: Object.freeze([]) });
const EPSILON = 1e-9;
const exact = (value, keys) => value && [Object.prototype, null].includes(Object.getPrototypeOf(value)) &&
  Reflect.ownKeys(value).length === keys.length && keys.every(key => Object.hasOwn(value, key));
const denseArray = value => Array.isArray(value) && Reflect.ownKeys(value).length === value.length + 1 &&
  Array.from({ length: value.length }, (_, i) => Object.hasOwn(value, i)).every(Boolean);
const pair = value => denseArray(value) && value.length === 2 && value.every(v => Number.isFinite(v) && Math.abs(v) <= 500);
const cross = (a, b) => a[0] * b[1] - a[1] * b[0];
const subtract = (a, b) => [a[0] - b[0], a[1] - b[1]];

function onSegment(point, a, b) {
  const edge = subtract(b, a), offset = subtract(point, a), length = Math.hypot(...edge);
  if (length <= EPSILON) return Math.hypot(...offset) <= EPSILON;
  const projection = (offset[0] * edge[0] + offset[1] * edge[1]) / length;
  return Math.abs(cross(edge, offset)) <= EPSILON * length && projection >= -EPSILON && projection <= length + EPSILON;
}

function segmentIntersections(a, b, c, d) {
  const ab = subtract(b, a), cd = subtract(d, c), ac = subtract(c, a);
  const length = Math.hypot(...ab), otherLength = Math.hypot(...cd), determinant = cross(ab, cd);
  if (length <= EPSILON) return onSegment(a, c, d) ? [0] : [];
  if (Math.abs(determinant) > EPSILON * Math.max(length, otherLength)) {
    const t = cross(ac, cd) / determinant, u = cross(ac, ab) / determinant;
    return t >= -EPSILON / length && t <= 1 + EPSILON / length &&
      u >= -EPSILON / Math.max(EPSILON, otherLength) && u <= 1 + EPSILON / Math.max(EPSILON, otherLength)
      ? [Math.max(0, Math.min(1, t))] : [];
  }
  if (Math.abs(cross(ab, ac)) > EPSILON * length) return [];
  const norm = length * length;
  const t0 = (ac[0] * ab[0] + ac[1] * ab[1]) / norm;
  const ad = subtract(d, a), t1 = (ad[0] * ab[0] + ad[1] * ab[1]) / norm;
  const low = Math.max(0, Math.min(t0, t1)), high = Math.min(1, Math.max(t0, t1));
  return low <= high + EPSILON / length ? [Math.max(0, Math.min(1, low)), Math.max(0, Math.min(1, high))] : [];
}

function validatePolygon(vertices) {
  if (!denseArray(vertices) || vertices.length < 3 || vertices.length > 32 || !vertices.every(pair))
    throw Error('Geofence polygon requires 3 to 32 finite N/E vertices within 500m');
  let twiceArea = 0;
  for (let i = 0; i < vertices.length; i++) {
    const a = vertices[i], b = vertices[(i + 1) % vertices.length], c = vertices[(i + 2) % vertices.length];
    twiceArea += cross(a, b);
    for (let j = i + 1; j < vertices.length; j++)
      if (Math.hypot(...subtract(a, vertices[j])) <= EPSILON) throw Error('Geofence polygon has duplicate vertices');
    const ab = subtract(b, a), bc = subtract(c, b);
    if (Math.abs(cross(ab, bc)) <= EPSILON * Math.max(Math.hypot(...ab), Math.hypot(...bc)) &&
      ab[0] * bc[0] + ab[1] * bc[1] < 0) throw Error('Geofence polygon has overlapping adjacent edges');
    for (let j = i + 2; j < vertices.length; j++) {
      if (i === 0 && j === vertices.length - 1) continue;
      if (segmentIntersections(a, b, vertices[j], vertices[(j + 1) % vertices.length]).length)
        throw Error('Geofence polygon must be simple without crossing or touching edges');
    }
  }
  if (Math.abs(twiceArea) <= EPSILON) throw Error('Geofence polygon has zero area');
}

export function validateGeofenceZones(value = DEFAULT_GEOFENCE_ZONES) {
  if (!exact(value, ['enabled', 'zones']) || typeof value.enabled !== 'boolean' || !denseArray(value.zones) ||
    value.zones.length > 20 || (value.enabled && value.zones.length === 0))
    throw Error('Geofence zones require enabled:boolean and up to 20 zones; enabling requires a zone');
  const ids = new Set();
  const zones = value.zones.map(zone => {
    const circle = zone?.shape === 'circle';
    if (!exact(zone, circle ? ['id', 'shape', 'kind', 'center', 'radiusM'] : ['id', 'shape', 'kind', 'vertices']) ||
      !['circle', 'polygon'].includes(zone.shape) || !['inclusion', 'exclusion'].includes(zone.kind) ||
      typeof zone.id !== 'string' || !/^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(zone.id) || ids.has(zone.id))
      throw Error('Invalid geofence zone shape, kind, fields or unique id');
    ids.add(zone.id);
    if (circle) {
      if (!pair(zone.center) || !Number.isFinite(zone.radiusM) || zone.radiusM < .1 || zone.radiusM > 500)
        throw Error('Geofence circle requires a finite N/E center within 500m and radiusM between 0.1 and 500');
      return { id: zone.id, shape: zone.shape, kind: zone.kind, center: [...zone.center], radiusM: zone.radiusM };
    }
    validatePolygon(zone.vertices);
    return { id: zone.id, shape: zone.shape, kind: zone.kind, vertices: zone.vertices.map(point => [...point]) };
  });
  return { enabled: value.enabled, zones };
}

function zoneContains(zone, point) {
  if (zone.shape === 'circle') return Math.hypot(...subtract(point, zone.center)) <= zone.radiusM + EPSILON;
  let inside = false;
  for (let i = 0, j = zone.vertices.length - 1; i < zone.vertices.length; j = i++) {
    const a = zone.vertices[j], b = zone.vertices[i];
    if (onSegment(point, a, b)) return true;
    if ((a[1] > point[1]) !== (b[1] > point[1]) &&
      point[0] < a[0] + (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1])) inside = !inside;
  }
  return inside;
}

export function geofenceContains(settings, point) {
  if (!settings.enabled) return true;
  let hasInclusion = false, included = false;
  for (const zone of settings.zones) {
    if (zone.kind === 'inclusion') {
      hasInclusion = true;
      included ||= zoneContains(zone, point);
    } else if (zoneContains(zone, point)) return false;
  }
  return !hasInclusion || included;
}

export function geofenceSegmentAllowed(settings, from, to) {
  if (!settings.enabled) return true;
  if (!geofenceContains(settings, from) || !geofenceContains(settings, to)) return false;
  const direction = subtract(to, from), norm = direction[0] ** 2 + direction[1] ** 2;
  if (norm <= EPSILON ** 2) return true;
  const cuts = [0, 1];
  for (const zone of settings.zones) {
    if (zone.shape === 'circle') {
      const offset = subtract(from, zone.center), projection = -(offset[0] * direction[0] + offset[1] * direction[1]) / norm;
      const distance = Math.hypot(...offset.map((v, i) => v + direction[i] * projection));
      // Project first: b² - 4ac loses narrow crossings and tangencies on long segments.
      if (distance <= zone.radiusM + EPSILON) {
        const halfChord = Math.sqrt(Math.max(0, (zone.radiusM - distance) * (zone.radiusM + distance)) / norm);
        for (const t of [projection - halfChord, projection, projection + halfChord]) if (t > 0 && t < 1) cuts.push(t);
      }
    } else {
      for (let i = 0; i < zone.vertices.length; i++)
        cuts.push(...segmentIntersections(from, to, zone.vertices[i], zone.vertices[(i + 1) % zone.vertices.length]));
    }
  }
  cuts.sort((a, b) => a - b);
  const allowed = t => geofenceContains(settings, from.map((v, i) => v + direction[i] * t));
  // Check every boundary and open interval, including tangencies and gaps between inclusion zones.
  return cuts.every((t, i) => allowed(t) && (i === 0 || allowed((cuts[i - 1] + t) / 2)));
}
