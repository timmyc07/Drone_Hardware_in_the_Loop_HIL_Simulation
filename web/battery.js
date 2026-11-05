// Motor demand is normalized speed at 4.2 V/cell; no ESC voltage compensation is implicit.
export function batterySupply(config, soc, speedDemand) {
  if (soc <= 0) return { openCircuitVoltage: 0, voltage: 0, current: 0, power: 0, motorSpeedScale: 0, powerLimited: false };
  const openCircuitVoltage = config.batteryCells * (3.3 + .9 * soc);
  const referenceVoltage = config.batteryCells * 4.2;
  const resistance = config.batteryInternalResistance;
  const motorPower = config.batteryMotorPower * speedDemand.reduce((sum, speed) => sum + Math.max(0, Math.min(1, speed)) ** 3, 0);
  const base = config.batteryBasePower;
  let voltage = openCircuitVoltage, motorSpeedScale = voltage / referenceVoltage, powerLimited = false;
  let power;
  if (!resistance) power = base + motorPower * motorSpeedScale ** 3;
  else {
    const maximumPower = openCircuitVoltage ** 2 / (4 * resistance);
    const halfVoltage = openCircuitVoltage / 2;
    const demand = value => base + motorPower * (value / referenceVoltage) ** 3;
    if (demand(halfVoltage) > maximumPower) {
      voltage = halfVoltage;
      power = maximumPower;
      motorSpeedScale = motorPower ? Math.cbrt(Math.max(0, maximumPower - base) / motorPower) : 0;
      powerLimited = true;
    } else {
      let low = halfVoltage, high = openCircuitVoltage;
      for (let i = 0; i < 48; i++) {
        const middle = (low + high) / 2;
        if (middle * (openCircuitVoltage - middle) / resistance >= demand(middle)) low = middle;
        else high = middle;
      }
      voltage = (low + high) / 2;
      motorSpeedScale = voltage / referenceVoltage;
      power = demand(voltage);
    }
  }
  return { openCircuitVoltage, voltage, current: power / voltage, power, motorSpeedScale, powerLimited };
}
