const EPS = 1e-8;
const fail = (message) => {
  throw new Error(`測繪路線：${message}`);
};
const cross = (a, b, c) =>
  (b.north - a.north) * (c.east - a.east) -
  (b.east - a.east) * (c.north - a.north);
const distance = (a, b) => Math.hypot(a.north - b.north, a.east - b.east);
const between = (p, a, b) =>
  Math.abs(cross(a, b, p)) <= EPS * Math.max(1, distance(a, b)) &&
  p.north >= Math.min(a.north, b.north) - EPS &&
  p.north <= Math.max(a.north, b.north) + EPS &&
  p.east >= Math.min(a.east, b.east) - EPS &&
  p.east <= Math.max(a.east, b.east) + EPS;

function intersects(a, b, c, d) {
  const abC = cross(a, b, c),
    abD = cross(a, b, d),
    cdA = cross(c, d, a),
    cdB = cross(c, d, b);
  return (
    (abC * abD < 0 && cdA * cdB < 0) ||
    between(a, c, d) ||
    between(b, c, d) ||
    between(c, a, b) ||
    between(d, a, b)
  );
}

function inside(p, polygon) {
  let result = false;
  for (let i = 0; i < polygon.length; i++) {
    const a = polygon[i], b = polygon[(i + 1) % polygon.length];
    if (between(p, a, b)) return true;
    if (
      (a.east > p.east) !== (b.east > p.east) &&
      p.north < a.north +
        ((p.east - a.east) * (b.north - a.north)) / (b.east - a.east)
    ) result = !result;
  }
  return result;
}

// Split at every boundary contact: a midpoint alone misses narrow concave notches.
function visible(a, b, polygon) {
  const length = distance(a, b);
  if (length <= EPS) return true;
  const cuts = [0, 1], dx = b.north - a.north, dy = b.east - a.east;
  for (let i = 0; i < polygon.length; i++) {
    const c = polygon[i], d = polygon[(i + 1) % polygon.length];
    const ex = d.north - c.north,
      ey = d.east - c.east,
      denominator = dx * ey - dy * ex;
    if (Math.abs(denominator) > EPS) {
      const t =
        ((c.north - a.north) * ey - (c.east - a.east) * ex) / denominator;
      const u =
        ((c.north - a.north) * dy - (c.east - a.east) * dx) / denominator;
      if (t > 0 && t < 1 && u >= 0 && u <= 1) cuts.push(t);
    } else {
      for (const p of [c, d]) {
        if (between(p, a, b)) {
          cuts.push(
            ((p.north - a.north) * dx + (p.east - a.east) * dy) /
              (length * length),
          );
        }
      }
    }
  }
  cuts.sort((x, y) => x - y);
  return cuts.slice(1).every((t, i) => {
    const middle = (t + cuts[i]) / 2;
    return inside(
      { north: a.north + dx * middle, east: a.east + dy * middle },
      polygon,
    );
  });
}

function connector(a, b, polygon) {
  if (visible(a, b, polygon)) return [b];
  const nodes = [a, b, ...polygon],
    costs = nodes.map(() => Infinity),
    previous = [],
    visited = new Set();
  costs[0] = 0;
  for (let step = 0; step < nodes.length; step++) {
    let current = -1;
    for (let i = 0; i < nodes.length; i++) {
      if (!visited.has(i) && (current < 0 || costs[i] < costs[current]))
        current = i;
    }
    if (current === 1 || !Number.isFinite(costs[current])) break;
    visited.add(current);
    for (let i = 0; i < nodes.length; i++) {
      if (visited.has(i)) continue;
      const cost = costs[current] + distance(nodes[current], nodes[i]);
      if (cost < costs[i] && visible(nodes[current], nodes[i], polygon)) {
        costs[i] = cost;
        previous[i] = current;
      }
    }
  }
  if (!Number.isFinite(costs[1])) fail("無法在多邊形內連接掃描線");
  const path = [];
  for (let i = 1; i !== 0; i = previous[i]) path.push(nodes[i]);
  return path.reverse();
}

const exactKeys = (object, keys) =>
  object !== null &&
  typeof object === "object" &&
  !Array.isArray(object) &&
  Reflect.ownKeys(object).length === keys.length &&
  keys.every((key) => Object.hasOwn(object, key));

export function buildSurveyRoute(input, options) {
  if (
    !Array.isArray(input) || input.length < 3 || input.length > 20 ||
    Object.keys(input).length !== input.length
  ) fail("多邊形須有 3 至 20 個完整頂點");
  const polygon = Array.from(input, (p) => {
    if (
      !exactKeys(p, ["north", "east"]) ||
      ![p.north, p.east].every((v) => Number.isFinite(v) && Math.abs(v) <= 500)
    ) fail("頂點座標須為 ±500 公尺內的有限數值");
    return { north: p.north, east: p.east };
  });
  const fields = Object.hasOwn(options ?? {}, "reverse")
    ? ["spacingM", "angleDeg", "altitudeM", "reverse"]
    : ["spacingM", "angleDeg", "altitudeM"];
  if (!exactKeys(options, fields)) fail("測繪參數不完整或含未知欄位");
  const { spacingM, angleDeg, altitudeM } = options;
  const reverse = Object.hasOwn(options, "reverse") ? options.reverse : false;
  if (
    !Number.isFinite(spacingM) || spacingM < 0.5 || spacingM > 200 ||
    !Number.isFinite(angleDeg) || angleDeg < 0 || angleDeg > 180 ||
    !Number.isFinite(altitudeM) || altitudeM < -1000 || altitudeM > 1500 ||
    typeof reverse !== "boolean"
  ) fail("間距、角度、高度或反轉參數超出範圍");
  let twiceArea = 0;
  for (let i = 0; i < polygon.length; i++) {
    const a = polygon[i],
      b = polygon[(i + 1) % polygon.length],
      c = polygon[(i + 2) % polygon.length];
    if (
      distance(a, b) <= EPS ||
      (Math.abs(cross(a, b, c)) <= EPS &&
        (b.north - a.north) * (c.north - b.north) +
          (b.east - a.east) * (c.east - b.east) < 0)
    ) fail("頂點重複或邊線折回");
    twiceArea += a.north * b.east - b.north * a.east;
    for (let j = i + 2; j < polygon.length; j++) {
      if (i === 0 && j === polygon.length - 1) continue;
      if (intersects(a, b, polygon[j], polygon[(j + 1) % polygon.length]))
        fail("多邊形不可自交或重複接觸");
    }
  }
  const areaM2 = Math.abs(twiceArea) / 2;
  if (areaM2 <= EPS) fail("多邊形面積過小或為零");
  const angle = angleDeg * Math.PI / 180;
  const cos = angleDeg === 90 ? 0 : Math.cos(angle);
  const sin = angleDeg === 0 || angleDeg === 180 ? 0 : Math.sin(angle);
  const rotated = polygon.map((p) => ({
    along: p.north * cos + p.east * sin,
    across: -p.north * sin + p.east * cos,
  }));
  const min = Math.min(...rotated.map((p) => p.across)),
    max = Math.max(...rotated.map((p) => p.across));
  const count = Math.ceil((max - min) / spacingM),
    actualSpacingM = (max - min) / count;
  if (!count || actualSpacingM <= EPS) fail("多邊形寬度不足以產生掃描線");
  if (count > 25) fail("路線超過 50 個航點，請增加間距或縮小範圍");
  let transects = [];
  for (let line = 0; line < count; line++) {
    const across = min + (line + 0.5) * actualSpacingM, intersections = [];
    for (let i = 0; i < rotated.length; i++) {
      const a = rotated[i], b = rotated[(i + 1) % rotated.length];
      if (
        (a.across <= across && across < b.across) ||
        (b.across <= across && across < a.across)
      ) {
        const ratio = (across - a.across) / (b.across - a.across);
        const from = polygon[i], to = polygon[(i + 1) % polygon.length];
        // Interpolate the original edge to preserve exact ±500m boundary coordinates.
        intersections.push({
          along: a.along + ratio * (b.along - a.along),
          point: {
            north: from.north + (to.north - from.north) * ratio,
            east: from.east + (to.east - from.east) * ratio,
          },
        });
      }
    }
    intersections.sort((a, b) => a.along - b.along);
    const segments = [];
    for (let i = 0; i + 1 < intersections.length; i += 2) {
      if (intersections[i + 1].along - intersections[i].along > EPS) {
        segments.push({
          start: intersections[i].point,
          end: intersections[i + 1].point,
        });
      }
    }
    if (line % 2) {
      segments.reverse().forEach((s) => {
        [s.start, s.end] = [s.end, s.start];
      });
    }
    transects.push(...segments);
  }
  if (!transects.length) fail("沒有可用的掃描線");
  const route = [];
  const append = (p) => {
    if (!route.length || distance(route.at(-1), p) > EPS) route.push(p);
    if (route.length > 50) fail("路線超過 50 個航點，請增加間距或縮小範圍");
  };
  for (const segment of transects) {
    if (route.length)
      connector(route.at(-1), segment.start, polygon).forEach(append);
    else append(segment.start);
    append(segment.end);
  }
  if (reverse) {
    route.reverse();
    transects = transects.reverse().map(({ start, end }) => ({
      start: end, end: start,
    }));
  }
  const routeLengthM = route.slice(1).reduce(
    (total, p, i) => total + distance(route[i], p), 0,
  );
  return {
    polygon,
    waypoints: route.map((p) => ({ ...p, altitude: altitudeM, hold: 0 })),
    transects,
    areaM2,
    routeLengthM,
    actualSpacingM,
  };
}
