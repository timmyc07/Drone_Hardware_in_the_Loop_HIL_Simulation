import test from "node:test";
import assert from "node:assert/strict";
import { formatCollisionPrevention } from "../collision-prevention-panel.js";
const state = { applicable: true, active: false, reason: "clear", obstacleType: null, obstacleId: null, clearanceM: 1 };
test("reports applicability, active braking and insufficient clearance", () => {
  assert.match(formatCollisionPrevention(null), /關閉/);
  assert.match(formatCollisionPrevention({ ...state, applicable: false }), /等待位置控制模式/);
  assert.match(formatCollisionPrevention(state), /正常.*設定安全餘量 1\.0 m/);
  assert.match(formatCollisionPrevention({ ...state, active: true, reason: "peer", obstacleType: "peer", obstacleId: "drone-2" }), /減速中.*載具 drone-2/);
  assert.match(formatCollisionPrevention({ ...state, active: true, reason: "insufficient-clearance" }), /餘距不足/);
});
