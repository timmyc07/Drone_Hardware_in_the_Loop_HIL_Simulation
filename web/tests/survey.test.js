import test from "node:test";
import assert from "node:assert/strict";
import { buildSurveyRoute } from "../survey.js";
import { createWorld, defaultWorldConfig } from "../world.js";

const p = (north, east) => ({ north, east });
const rectangle = [p(0, 0), p(20, 0), p(20, 10), p(0, 10)];
const options = { spacingM: 4, angleDeg: 0, altitudeM: 30 };
const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-7, `${a} != ${b}`);

function inPolygon(point, polygon) {
  let winding = 0;
  for (let i = 0; i < polygon.length; i++) {
    const a = polygon[i], b = polygon[(i + 1) % polygon.length];
    const cross =
      (b.north - a.north) * (point.east - a.east) -
      (b.east - a.east) * (point.north - a.north);
    if (
      Math.abs(cross) < 1e-7 &&
      point.north >= Math.min(a.north, b.north) - 1e-8 &&
      point.north <= Math.max(a.north, b.north) + 1e-8 &&
      point.east >= Math.min(a.east, b.east) - 1e-8 &&
      point.east <= Math.max(a.east, b.east) + 1e-8
    ) return true;
    if (a.east <= point.east && b.east > point.east && cross < 0) winding++;
    if (a.east > point.east && b.east <= point.east && cross > 0) winding--;
  }
  return winding !== 0;
}

function assertRouteInside(route, polygon) {
  for (let i = 1; i < route.length; i++) {
    for (let j = 0; j <= 200; j++) {
      const t = j / 200, a = route[i - 1], b = route[i];
      assert.ok(
        inPolygon(p(
          a.north + (b.north - a.north) * t,
          a.east + (b.east - a.east) * t,
        ), polygon),
        `segment ${i} leaves polygon at ${t}`,
      );
    }
  }
}

test("northward rectangle uses centered spacing, full endpoints and measured length", () => {
  const result = buildSurveyRoute(rectangle, options);
  near(result.areaM2, 200);
  near(result.actualSpacingM, 10 / 3);
  assert.equal(result.transects.length, 3);
  assert.equal(result.waypoints.length, 6);
  near(result.routeLengthM, 60 + 20 / 3);
  result.transects.forEach(({ start, end }, i) => {
    near(start.east, (i + 0.5) * 10 / 3);
    near(end.east, start.east);
    near(Math.abs(end.north - start.north), 20);
  });
  assert.ok(result.waypoints.every((w) =>
    w.altitude === 30 && w.hold === 0 && Object.keys(w).length === 4,
  ));
});

test("eastward and oblique lines follow angle relative to north and stay inside", () => {
  for (const angleDeg of [30, 90, 137, 180]) {
    const result = buildSurveyRoute(rectangle, { ...options, angleDeg });
    assert.ok(result.actualSpacingM <= options.spacingM + 1e-10);
    for (const { start, end } of result.transects) {
      const dn = end.north - start.north, de = end.east - start.east;
      near(
        dn * Math.sin(angleDeg * Math.PI / 180) -
          de * Math.cos(angleDeg * Math.PI / 180),
        0,
      );
    }
    assertRouteInside(result.waypoints, rectangle);
  }
});

test("concave comb preserves disjoint intervals and connects around both notches", () => {
  const comb = [
    p(0, 0), p(12, 0), p(12, 12), p(9, 12), p(9, 3), p(7, 3),
    p(7, 12), p(5, 12), p(5, 3), p(3, 3), p(3, 12), p(0, 12),
  ];
  const result = buildSurveyRoute(comb, { ...options, spacingM: 4 });
  assert.equal(result.transects.length, 7);
  near(result.transects.reduce((sum, s) =>
    sum + Math.hypot(s.end.north - s.start.north, s.end.east - s.start.east),
  0), 28);
  assert.ok(result.waypoints.length > result.transects.length * 2);
  assertRouteInside(result.waypoints, comb);
  assert.ok(result.waypoints.some((w) => w.north === 9 && w.east === 3));
  assert.ok(result.waypoints.some((w) => w.north === 7 && w.east === 3));
});

test("clockwise input keeps geometry and reverse reverses the complete route", () => {
  const normal = buildSurveyRoute(rectangle, options);
  const clockwise = buildSurveyRoute([...rectangle].reverse(), options);
  clockwise.waypoints.forEach((point, i) => {
    near(point.north, normal.waypoints[i].north);
    near(point.east, normal.waypoints[i].east);
  });
  const reversed = buildSurveyRoute(rectangle, { ...options, reverse: true });
  assert.deepEqual(reversed.waypoints, [...normal.waypoints].reverse());
  assert.deepEqual(
    reversed.transects,
    [...normal.transects].reverse().map(({ start, end }) => ({
      start: end, end: start,
    })),
  );
  near(normal.routeLengthM, reversed.routeLengthM);
});

test("supports parameter boundaries and full 1000m coordinate span", () => {
  const square = [p(-500, -500), p(500, -500), p(500, 500), p(-500, 500)];
  const result = buildSurveyRoute(square, {
    spacingM: 200, angleDeg: 0, altitudeM: -1000,
  });
  near(result.areaM2, 1000000);
  assert.equal(result.waypoints.length, 10);
  const tiny = [p(0, 0), p(0.5, 0), p(0.5, 0.5), p(0, 0.5)];
  assert.equal(buildSurveyRoute(tiny, {
    spacingM: 0.5, angleDeg: 180, altitudeM: 1500,
  }).waypoints.length, 2);
});

test("rejects malformed, sparse, non-finite and unknown data without repairing it", () => {
  const invalid = [
    null, [], new Array(4), [p(0, 0), , p(1, 1)],
    [...rectangle, rectangle[0]],
    [p(0, 0), p(1, 1), p(2, 2)],
    [p(0, 0), p(2, 2), p(0, 2), p(2, 0)],
    [p(0, 0), p(3, 0), p(1, 0), p(1, 3)],
    rectangle.map((v, i) => i ? v : { ...v, unknown: 1 }),
    rectangle.map((v, i) => i ? v : p(NaN, 0)),
    rectangle.map((v, i) => i ? v : p(501, 0)),
    [p(0, 0), p(1e-10, 0), p(1e-10, 1e-10)],
  ];
  for (const polygon of invalid)
    assert.throws(() => buildSurveyRoute(polygon, options), /測繪路線/);
  for (const patch of [
    { spacingM: 0.49 }, { spacingM: 201 },
    { angleDeg: -1 }, { angleDeg: 181 }, { angleDeg: Infinity },
    { altitudeM: -1001 }, { altitudeM: 1501 },
    { reverse: undefined }, { reverse: 1 }, { extra: 1 },
  ]) {
    assert.throws(
      () => buildSurveyRoute(rectangle, { ...options, ...patch }),
      /測繪路線/,
    );
  }
  assert.throws(() => buildSurveyRoute(rectangle, undefined), /測繪路線/);
  assert.throws(
    () => buildSurveyRoute(rectangle, { spacingM: 1, angleDeg: 0 }),
    /測繪路線/,
  );
});

test("rejects over 50 points without truncating, including concave connectors", () => {
  const wide = [p(0, 0), p(10, 0), p(10, 26), p(0, 26)];
  assert.throws(() => buildSurveyRoute(wide, { ...options, spacingM: 1 }), /50/);
  const fifty = wide.map((v) => p(v.north, v.east * 25 / 26));
  assert.equal(buildSurveyRoute(fifty, {
    ...options, spacingM: 1,
  }).waypoints.length, 50);
  const u = [
    p(0, 0), p(20, 0), p(20, 20), p(12, 20),
    p(12, 3), p(8, 3), p(8, 20), p(0, 20),
  ];
  assert.throws(() => buildSurveyRoute(u, { ...options, spacingM: 1 }), /50/);
});

test("handles vertex touches, collinear vertices, determinism and immutable input", () => {
  const polygon = [
    p(0, 0), p(6, 0), p(6, 6), p(4, 6), p(3, 3), p(2, 6), p(0, 6),
  ];
  const copy = structuredClone(polygon);
  const settings = Object.freeze({ ...options, spacingM: 6 });
  polygon.forEach(Object.freeze);
  Object.freeze(polygon);
  const result = buildSurveyRoute(polygon, settings);
  assertRouteInside(result.waypoints, polygon);
  assert.deepEqual(buildSurveyRoute(polygon, settings), result);
  assert.deepEqual(polygon, copy);
  assert.doesNotThrow(() => buildSurveyRoute(
    [p(0, 0), p(10, 0), p(20, 0), p(20, 10), p(0, 10)], options,
  ));
});

test("narrow off-center notch needs boundary bends despite an interior midpoint", () => {
  const polygon = [
    p(0, 0), p(10, 0), p(10, 10), p(8.01, 10),
    p(8.01, 1), p(8, 1), p(8, 10), p(0, 10),
  ];
  const result = buildSurveyRoute(polygon, { ...options, spacingM: 10 });
  assert.equal(result.transects.length, 2);
  assert.equal(result.waypoints.length, 6);
  const bend = result.waypoints.slice(2, 4);
  near(bend[0].north, 8);
  near(bend[0].east, 1);
  near(bend[1].north, 8.01);
  near(bend[1].east, 1);
  near(result.routeLengthM, 9.99 + 8 + 0.01);
  assertRouteInside(result.waypoints, polygon);
  assert.deepEqual(buildSurveyRoute([...polygon].reverse(), {
    ...options, spacingM: 10,
  }).waypoints, result.waypoints);
});

test("oblique boundary intersections fit strict bounds and apply as world missions", () => {
  const polygon = [p(400, -40), p(500, -40), p(500, 40), p(400, 40)];
  const config = structuredClone(defaultWorldConfig({
    geofenceRadius: 100, startOnGround: true,
  }));
  const vehicle = config.vehicles[0];
  vehicle.home = [450, 0];
  vehicle.spawn = { ...vehicle.spawn, north: 450, east: 0 };
  const world = createWorld(config);
  for (const angleDeg of [1, 2, 8, 30, 90, 137, 179]) {
    const result = buildSurveyRoute(polygon, {
      ...options, spacingM: 10, angleDeg,
    });
    for (const point of [
      ...result.waypoints,
      ...result.transects.flatMap(({ start, end }) => [start, end]),
    ]) {
      assert.ok(Math.abs(point.north) <= 500);
      assert.ok(Math.abs(point.east) <= 500);
    }
    assert.doesNotThrow(() =>
      world.command(vehicle.id, "mission", result.waypoints),
    );
  }
});
