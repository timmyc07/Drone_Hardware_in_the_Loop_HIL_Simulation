import test from 'node:test';
import assert from 'node:assert/strict';
import { defaultWorldConfig } from '../world.js';
import { SENSOR_NAMES } from '../sensors.js';
import { createInstructorScheduler, validateInstructorPlan, instructorPlanToJson, parseInstructorPlan, instructorReportToJson, INSTRUCTOR_MAX_EVENTS } from '../instructor.js';
const vehicles = defaultWorldConfig().vehicles;
const vehicleId = vehicles[0].id;
const action = (tick = 1) => ({tick, vehicleId, type:'gust'});
const plan = (...events) => ({name:'教官測試', events:events.length ? events : [action()]});

test('strict instructor plan validation rejects malformed fields, arrays, numbers and vehicle commands', () => {
  assert.equal(INSTRUCTOR_MAX_EVENTS, 64);
  const invalid = [null, {}, {...plan(), extra:1}, {...plan(), name:''}, {...plan(), name:'x'.repeat(81)}, plan({...action(), tick:NaN}), plan({...action(), tick:0}), plan({...action(), tick:1.1}), plan({...action(), tick:360001}), plan({...action(), vehicleId:'missing'}), plan({...action(), vehicleId:'a b'}), plan({...action(), value:{}}), plan({...action(), type:'arm'}), plan(action(2), action(1)), {...plan(), events:new Array(1)}, {...plan(), events:[]}, {...plan(), events:Array.from({length:65}, () => action())}];
  const extra = plan(); extra.events.foo = true; invalid.push(extra);
  const symbol = plan(); symbol.events[0][Symbol('extra')] = true; invalid.push(symbol);
  const inherited = Object.create(plan()); invalid.push(inherited);
  for (const value of invalid) assert.throws(() => validateInstructorPlan(value, vehicles));
  for (const sensor of SENSOR_NAMES) assert.doesNotThrow(() => validateInstructorPlan(plan({tick:1,vehicleId,type:'sensorFault',value:{sensor,failed:true}}), vehicles));
  for (const value of [{sensor:'unknown',failed:true}, {sensor:SENSOR_NAMES[0],failed:1}, {sensor:SENSOR_NAMES[0],failed:false, extra:1}]) assert.throws(() => validateInstructorPlan(plan({tick:1,vehicleId,type:'sensorFault',value}), vehicles));
  for (const value of [{speed:NaN,direction:0},{speed:31,direction:0},{speed:1,direction:361}]) assert.throws(() => validateInstructorPlan(plan({tick:1,type:'wind',value}), vehicles));
  assert.throws(() => validateInstructorPlan(plan({tick:1,vehicleId,type:'wind',value:{speed:1,direction:0}}), vehicles));
});
test('fault validation follows rotor counts and wing capabilities', () => {
  for (const [airframe,count] of [['quadrotor',4],['hexarotor',6],['octorotor',8],['vtol',4],['fixedwing',0]]) {
    const roster = [{...vehicles[0],config:{...vehicles[0].config,airframe}}];
    const fault = (fault,value) => plan({tick:1,vehicleId,type:'fault',fault,value});
    if (count) assert.doesNotThrow(() => validateInstructorPlan(fault('motor',{index:count-1,efficiency:0}),roster));
    assert.throws(() => validateInstructorPlan(fault('motor',{index:count,efficiency:0}),roster));
    assert.throws(() => validateInstructorPlan(fault('motor',{index:-1,efficiency:1}),roster));
    assert.throws(() => validateInstructorPlan(fault('link',{lost:0}),roster));
    assert.doesNotThrow(() => validateInstructorPlan(fault('link',{lost:false}),roster));
    if (['vtol','fixedwing'].includes(airframe)) assert.doesNotThrow(() => validateInstructorPlan(fault('pusher',{efficiency:1}),roster));
    else assert.throws(() => validateInstructorPlan(fault('pusher',{efficiency:1}),roster));
  }
});
test('scheduler applies atomically, clones inputs and preserves exact tick order', () => {
  const scheduler = createInstructorScheduler(), source = plan(action(2), {tick:2,type:'wind',value:{speed:2,direction:90}}, action(360000));
  scheduler.apply(source,0,vehicles); source.events[0].tick=4;
  const before = scheduler.getState(); before.plan.name='mutated';
  assert.throws(() => scheduler.apply(plan(action(3)),0,vehicles));
  assert.equal(scheduler.getState().plan.name,'教官測試');
  const executed=[];
  scheduler.run(1,command=>executed.push(command)); assert.equal(executed.length,0);
  scheduler.run(2,command=>executed.push(command));
  assert.deepEqual(executed.map(command=>command.type),['gust','wind']);
  assert.ok(executed.every(command=>!Object.hasOwn(command,'tick')));
  scheduler.run(360000,command=>executed.push(command));
  assert.equal(scheduler.getState().phase,'complete');
  assert.equal(scheduler.getState().receipts.length,3);
  const complete=scheduler.getState();
  assert.throws(()=>scheduler.apply(plan(action(1)),1,vehicles));
  assert.deepEqual(scheduler.getState(),complete);
  assert.throws(()=>scheduler.apply(plan(action(2),{...action(3),type:'invalid'}),0,vehicles));
  assert.deepEqual(scheduler.getState(),complete);
});
test('scheduler stops on missed ticks or callback failure, cancellation never undoes applied events', () => {
  const scheduler=createInstructorScheduler(), executed=[];
  scheduler.apply(plan(action(1),action(1),action(2)),0,vehicles);
  scheduler.run(1, command=>{executed.push(command);if(executed.length===2)throw new Error('拒絕');});
  const failed=scheduler.getState();
  assert.equal(failed.phase,'error'); assert.deepEqual(failed.receipts.map(r=>r.status),['applied','failed']); assert.equal(failed.nextIndex,1);
  scheduler.run(2,()=>assert.fail()); assert.deepEqual(scheduler.getState(),failed);
  scheduler.reset(); scheduler.apply(plan(action()),0,vehicles);
  scheduler.run(2,()=>assert.fail()); assert.equal(scheduler.getState().phase,'error');
  scheduler.reset(); scheduler.apply(plan(action(1),action(2)),0,vehicles);
  scheduler.run(1,()=>{});scheduler.cancel();scheduler.run(2,()=>assert.fail());
  assert.equal(scheduler.getState().phase,'cancelled');assert.equal(scheduler.getState().receipts.length,1);
  scheduler.reset();assert.deepEqual(scheduler.getState(),{phase:'idle',plan:null,appliedAtTick:null,nextIndex:0,receipts:[],reason:''});
});
test('plan artifact round trips strictly and reports preserve receipts', () => {
  const source=plan();assert.deepEqual(parseInstructorPlan(instructorPlanToJson(source,vehicles),vehicles),source);
  for (const text of ['{', ' '.repeat(65537), JSON.stringify({format:'flight-lab-instructor-plan',version:2,plan:source}),JSON.stringify({format:'flight-lab-instructor-plan',version:1,plan:source,extra:1})]) assert.throws(()=>parseInstructorPlan(text,vehicles));
  const scheduler=createInstructorScheduler();scheduler.apply(source,0,vehicles);scheduler.run(1,()=>{});
  const report=JSON.parse(instructorReportToJson(scheduler.getState()));assert.equal(report.format,'flight-lab-instructor-report');assert.equal(report.version,1);assert.equal(report.receipts[0].status,'applied');
});
