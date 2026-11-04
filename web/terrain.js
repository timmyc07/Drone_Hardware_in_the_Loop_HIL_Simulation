export const DEFAULT_TERRAIN = Object.freeze({ kind: "flat" });
export const TERRAIN_PROTECTED_AREA = Object.freeze({
  northMin: -10,
  northMax: 210,
  eastMin: -20,
  eastMax: 20,
});
export const TERRAIN_MESH_EXTENT = 10000;
const UP = Object.freeze([0, 0, -1]);
const EPS = 1e-9;
const validVector = (value) =>
  Array.isArray(value) &&
  value.length === 3 &&
  [0, 1, 2].every(
    (index) => Object.hasOwn(value, index) && Number.isFinite(value[index]),
  );

export function validateTerrain(value = DEFAULT_TERRAIN) {
  const keys =
    value?.kind === "flat"
      ? ["kind"]
      : ["kind", "north", "east", "cellSize", "rows", "cols", "heights"];
  if (
    !value ||
    ![Object.prototype, null].includes(Object.getPrototypeOf(value)) ||
    Reflect.ownKeys(value).length !== keys.length ||
    Reflect.ownKeys(value).some((key) => !keys.includes(key))
  )
    throw new Error("Invalid terrain fields");
  if (value.kind === "flat") return Object.freeze({ kind: "flat" });
  if (value.kind !== "heightfield") throw new Error("Invalid terrain kind");
  const { north, east, cellSize, rows, cols, heights } = value;
  if (
    ![north, east, cellSize].every(Number.isFinite) ||
    cellSize < 1 ||
    cellSize > 50 ||
    ![rows, cols].every((n) => Number.isInteger(n) && n >= 2 && n <= 65) ||
    north < -500 ||
    east < -500 ||
    north + (rows - 1) * cellSize > 500 ||
    east + (cols - 1) * cellSize > 500
  )
    throw new Error("Invalid terrain extent or resolution");
  if (!Array.isArray(heights) || heights.length !== rows * cols)
    throw new Error("Invalid terrain heights");
  const copy = [];
  for (let i = 0; i < rows; i++)
    for (let j = 0; j < cols; j++) {
      const index = i * cols + j,
        height = heights[index];
      if (
        !Object.hasOwn(heights, index) ||
        !Number.isFinite(height) ||
        height < -50 ||
        height > 150 ||
        ((i === 0 || j === 0 || i === rows - 1 || j === cols - 1) &&
          height !== 0)
      )
        throw new Error("Invalid terrain height or nonzero boundary");
      copy.push(height);
    }
  const p = TERRAIN_PROTECTED_AREA;
  for (let i = 0; i < rows - 1; i++)
    for (let j = 0; j < cols - 1; j++) {
      const n = north + i * cellSize,
        e = east + j * cellSize;
      if (
        n < p.northMax &&
        n + cellSize > p.northMin &&
        e < p.eastMax &&
        e + cellSize > p.eastMin &&
        [
          i * cols + j,
          (i + 1) * cols + j,
          i * cols + j + 1,
          (i + 1) * cols + j + 1,
        ].some((index) => copy[index] !== 0)
      )
        throw new Error("Terrain must keep the home and runway flat");
    }
  return Object.freeze({
    kind: "heightfield",
    north,
    east,
    cellSize,
    rows,
    cols,
    heights: Object.freeze(copy),
  });
}

export function createTerrain(value = DEFAULT_TERRAIN) {
  const data = validateTerrain(value),
    grid = data.kind === "heightfield";
  const {
    north = 0,
    east = 0,
    cellSize = 1,
    rows = 2,
    cols = 2,
    heights,
  } = data;
  const nMax = north + (rows - 1) * cellSize,
    eMax = east + (cols - 1) * cellSize;
  const cells = [];
  if (grid)
    for (let i = 0; i < rows - 1; i++)
      for (let j = 0; j < cols - 1; j++) {
        const a = heights[i * cols + j],
          b = heights[(i + 1) * cols + j];
        const c = heights[i * cols + j + 1],
          d = heights[(i + 1) * cols + j + 1];
        cells.push(
          [
            [b - a, d - b],
            [d - c, c - a],
          ].map(([dn, de]) => {
            const hn = dn / cellSize,
              he = de / cellSize,
              length = Math.hypot(hn, he, 1);
            return {
              height: a,
              hn,
              he,
              normal: [-hn / length, -he / length, -1 / length],
            };
          }),
        );
      }
  const inside = (n, e) =>
    grid && n >= north && n <= nMax && e >= east && e <= eMax;
  function sample(n, e) {
    if (!Number.isFinite(n) || !Number.isFinite(e))
      throw new Error("Invalid terrain coordinates");
    if (!inside(n, e)) return { heightM: 0, normalNed: [...UP] };
    const x = (n - north) / cellSize,
      y = (e - east) / cellSize;
    const i = Math.min(rows - 2, Math.floor(x)),
      j = Math.min(cols - 2, Math.floor(y));
    const u = x - i,
      v = y - j,
      face = cells[i * (cols - 1) + j][u >= v ? 0 : 1];
    return {
      heightM: face.height + cellSize * (face.hn * u + face.he * v),
      normalNed: [...face.normal],
    };
  }
  const cellAt = (coordinate, lower, count, velocity) => {
    const x = (coordinate - lower) / cellSize,
      rounded = Math.round(x);
    const index =
      Math.abs(x - rounded) < EPS
        ? rounded - (velocity < 0 ? 1 : 0)
        : Math.floor(x);
    return Math.max(0, Math.min(count - 2, index));
  };
  function enteringAt(position, direction) {
    const [n, e] = position,
      [dn, de, dd] = direction;
    if (
      !grid ||
      n < north - EPS ||
      n > nMax + EPS ||
      e < east - EPS ||
      e > eMax + EPS ||
      (n <= north + EPS && dn < 0) ||
      (n >= nMax - EPS && dn > 0) ||
      (e <= east + EPS && de < 0) ||
      (e >= eMax - EPS && de > 0)
    )
      return dd > 0;
    const i = cellAt(n, north, rows, dn),
      j = cellAt(e, east, cols, de);
    const u = (n - north) / cellSize - i,
      v = (e - east) / cellSize - j;
    const side = Math.abs(u - v) < EPS ? (dn >= de ? 0 : 1) : u >= v ? 0 : 1;
    const face = cells[i * (cols - 1) + j][side];
    return dd + face.hn * dn + face.he * de > 0;
  }
  function trace(origin, direction, min, max, entering) {
    if (
      !Number.isFinite(min) ||
      !Number.isFinite(max) ||
      min < 0 ||
      max < min ||
      !validVector(origin) ||
      !validVector(direction)
    )
      throw new Error("Invalid terrain ray or segment");
    let best = null;
    const point = (t) => origin.map((v, axis) => v + direction[axis] * t);
    function accept(t, normal) {
      if (t < min - EPS || t > max + EPS || (best && t >= best.t)) return;
      t = Math.max(min, Math.min(max, t));
      const position = point(t);
      // Use the next face's derivative so touching a ridge never becomes a collision.
      if (entering && !enteringAt(position, direction)) return;
      best = { t, position, normal };
    }
    if (Math.abs(direction[2]) > 1e-14 && (!entering || direction[2] > 0)) {
      const t = -origin[2] / direction[2],
        p = point(t);
      if (!inside(p[0], p[1])) accept(t, UP);
    }
    if (!grid) return best;
    let start = min,
      end = max;
    for (const [axis, lower, upper] of [
      [0, north, nMax],
      [1, east, eMax],
    ]) {
      const velocity = direction[axis];
      if (!velocity) {
        if (origin[axis] < lower || origin[axis] > upper) return best;
      } else {
        const a = (lower - origin[axis]) / velocity,
          b = (upper - origin[axis]) / velocity;
        start = Math.max(start, Math.min(a, b));
        end = Math.min(end, Math.max(a, b));
      }
    }
    if (start > end + EPS || (best && best.t < start)) return best;
    const first = point(start);
    let i = cellAt(first[0], north, rows, direction[0]);
    let j = cellAt(first[1], east, cols, direction[1]);
    const stepN = Math.sign(direction[0]),
      stepE = Math.sign(direction[1]);
    const nextN = () =>
      stepN
        ? (north + (i + (stepN > 0 ? 1 : 0)) * cellSize - origin[0]) /
          direction[0]
        : Infinity;
    const nextE = () =>
      stepE
        ? (east + (j + (stepE > 0 ? 1 : 0)) * cellSize - origin[1]) /
          direction[1]
        : Infinity;
    // A straight ray crosses at most rows + cols grid boundaries, irrespective of grid area.
    for (
      let visited = 0;
      visited < rows + cols && i >= 0 && i < rows - 1 && j >= 0 && j < cols - 1;
      visited++
    ) {
      const tn = nextN(),
        te = nextE(),
        exit = Math.min(end, tn, te);
      const faces = cells[i * (cols - 1) + j];
      for (let side = 0; side < 2; side++) {
        const face = faces[side];
        const denominator =
          direction[2] + face.hn * direction[0] + face.he * direction[1];
        if (Math.abs(denominator) < 1e-14 || (entering && denominator <= 0))
          continue;
        const clearance =
          origin[2] +
          face.height +
          face.hn * (origin[0] - north - i * cellSize) +
          face.he * (origin[1] - east - j * cellSize);
        const t = -clearance / denominator;
        if (t < start - EPS || t > exit + EPS) continue;
        const u = (origin[0] + direction[0] * t - north) / cellSize - i;
        const v = (origin[1] + direction[1] * t - east) / cellSize - j;
        if (
          u < -EPS ||
          u > 1 + EPS ||
          v < -EPS ||
          v > 1 + EPS ||
          (side === 0 ? u < v - EPS : u > v + EPS)
        )
          continue;
        accept(t, face.normal);
      }
      if (best && best.t <= exit + EPS) break;
      if (exit >= end) break;
      if (tn <= te) i += stepN;
      if (te <= tn) j += stepE;
      start = exit;
    }
    return best;
  }
  let geometry;
  function mesh() {
    if (geometry) return geometry;
    const positionsNed = [],
      indices = [],
      vertices = new Map(),
      m = TERRAIN_MESH_EXTENT;
    function vertex(n, e, h = 0) {
      const key = `${n},${e},${h}`;
      if (!vertices.has(key)) {
        vertices.set(key, positionsNed.length / 3);
        positionsNed.push(n, e, -h);
      }
      return vertices.get(key);
    }
    function quad(a, b, c, d) {
      indices.push(a, d, b, a, c, d);
    }
    if (!grid) quad(vertex(-m, -m), vertex(m, -m), vertex(-m, m), vertex(m, m));
    else {
      for (let i = 0; i < rows - 1; i++)
        for (let j = 0; j < cols - 1; j++) {
          const v = (r, c) =>
            vertex(
              north + r * cellSize,
              east + c * cellSize,
              heights[r * cols + c],
            );
          quad(v(i, j), v(i + 1, j), v(i, j + 1), v(i + 1, j + 1));
        }
      const outerE = (j) =>
        j === 0 ? -m : j === cols - 1 ? m : east + j * cellSize;
      const outerN = (i) =>
        i === 0 ? -m : i === rows - 1 ? m : north + i * cellSize;
      for (let j = 0; j < cols - 1; j++) {
        const e = east + j * cellSize,
          e1 = e + cellSize;
        quad(
          vertex(-m, outerE(j)),
          vertex(north, e),
          vertex(-m, outerE(j + 1)),
          vertex(north, e1),
        );
        quad(
          vertex(nMax, e),
          vertex(m, outerE(j)),
          vertex(nMax, e1),
          vertex(m, outerE(j + 1)),
        );
      }
      for (let i = 0; i < rows - 1; i++) {
        const n = north + i * cellSize,
          n1 = n + cellSize;
        quad(
          vertex(outerN(i), -m),
          vertex(outerN(i + 1), -m),
          vertex(n, east),
          vertex(n1, east),
        );
        quad(
          vertex(n, eMax),
          vertex(n1, eMax),
          vertex(outerN(i), m),
          vertex(outerN(i + 1), m),
        );
      }
    }
    geometry = Object.freeze({
      positionsNed: Object.freeze(positionsNed),
      indices: Object.freeze(indices),
    });
    return geometry;
  }
  return Object.freeze({
    data,
    sample,
    mesh,
    raycast(originNed, unitDir, minM, maxM) {
      if (!validVector(unitDir) || Math.abs(Math.hypot(...unitDir) - 1) > 1e-8)
        throw new Error("Terrain ray direction must be a unit vector");
      const hit = trace(originNed, unitDir, minM, maxM, false);
      return (
        hit && {
          distanceM: hit.t,
          positionNed: hit.position,
          normalNed: [...hit.normal],
        }
      );
    },
    sweep(fromNed, toNed) {
      if (!validVector(fromNed) || !validVector(toNed))
        throw new Error("Invalid terrain segment");
      const hit = trace(
        fromNed,
        toNed.map((v, axis) => v - fromNed[axis]),
        0,
        1,
        true,
      );
      return (
        hit && {
          fraction: hit.t,
          positionNed: hit.position,
          normalNed: [...hit.normal],
        }
      );
    },
  });
}
