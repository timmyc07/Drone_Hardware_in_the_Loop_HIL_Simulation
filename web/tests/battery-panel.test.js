import test from "node:test";
import assert from "node:assert/strict";
import { formatBattery } from "../battery-panel.js";

const battery = { soc: 0.6, voltage: 14.4, openCircuitVoltage: 15.2, current: 8, power: 115.2, motorSpeedScale: 0.8, powerLimited: false };
test("normal voltage sag reports squared thrust limit without a power warning", () => {
  const text = formatBattery(battery);
  assert.match(text, /60%.*負載電壓 14\.4 V.*開路電壓 15\.2 V.*8\.0 A.*115\.2 W/);
  assert.match(text, /同一命令推力比例 64\.0%/);
  assert.doesNotMatch(text, /電池功率不足/);
});
test("power limitation and empty battery remain distinguishable", () => {
  assert.match(formatBattery({ ...battery, powerLimited: true }), /電池功率不足/);
  const empty = formatBattery({ ...battery, soc: 0, voltage: 0, openCircuitVoltage: 0, current: 0, power: 0, motorSpeedScale: 0 });
  assert.match(empty, /電池 0%/);
  assert.match(empty, /同一命令推力比例 0\.0%/);
});
