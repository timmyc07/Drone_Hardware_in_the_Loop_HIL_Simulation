import test from 'node:test';
import assert from 'node:assert/strict';
import { batterySupply } from '../battery.js';
import { DEFAULT_CONFIG } from '../sim.js';
const near = (a,b) => assert.ok(Math.abs(a-b) < 1e-8, `${a} != ${b}`);
for (const resistance of [0,.05,2]) test(`battery supply satisfies electrical and propulsive demand at resistance ${resistance}`, () => {
  const config={...DEFAULT_CONFIG,batteryInternalResistance:resistance};
  for (const soc of [.05,.5,1]) for (const speeds of [[0,0,0,0],[.5,.5,.5,.5],[1,1,1,1]]) {
    const p=batterySupply(config,soc,speeds);
    near(p.power,p.voltage*p.current);
    near(p.openCircuitVoltage-p.voltage,p.current*resistance);
    const motorPower=config.batteryMotorPower*speeds.reduce((sum,v)=>sum+(v*p.motorSpeedScale)**3,0);
    near(p.power,Math.min(config.batteryBasePower,p.power)+motorPower);
    assert.ok(p.motorSpeedScale>=0 && p.motorSpeedScale<=p.voltage/(config.batteryCells*4.2)+1e-12);
    if (resistance) assert.ok(p.power<=p.openCircuitVoltage**2/(4*resistance)+1e-8);
  }
});
test('maximum-power load shedding gives base load priority and removes powered thrust when it alone exhausts supply', () => {
  const config={...DEFAULT_CONFIG,batteryInternalResistance:2,batteryBasePower:0};
  const p=batterySupply(config,1,[1,1,1,1]);
  assert.equal(p.powerLimited,true);
  near(p.voltage,p.openCircuitVoltage/2);
  near(p.power,p.openCircuitVoltage**2/(4*config.batteryInternalResistance));
  const base=batterySupply({...config,batteryBasePower:200},1,[1,1,1,1]);
  assert.equal(base.powerLimited,true); assert.equal(base.motorSpeedScale,0);
  near(base.power,base.openCircuitVoltage**2/(4*config.batteryInternalResistance));
});
test('empty battery supplies no voltage, energy or powered speed for any reference demand', () => {
  assert.deepEqual(batterySupply(DEFAULT_CONFIG,0,[1,1,1,1]),{openCircuitVoltage:0,voltage:0,current:0,power:0,motorSpeedScale:0,powerLimited:false});
});
