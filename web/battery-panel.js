export function formatBattery(battery) {
  return `電池 ${(battery.soc * 100).toFixed(0)}% · 負載電壓 ${battery.voltage.toFixed(1)} V · 開路電壓 ${battery.openCircuitVoltage.toFixed(1)} V · ${battery.current.toFixed(1)} A · ${battery.power.toFixed(1)} W · 同一命令推力比例 ${(battery.motorSpeedScale ** 2 * 100).toFixed(1)}%${battery.powerLimited ? " · 電池功率不足" : ""}`;
}
