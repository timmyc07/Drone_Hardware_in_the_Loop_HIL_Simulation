import { SENSOR_NAMES } from './sensors.js';
import { rotorCount } from './rotor-geometry.js';

export const INSTRUCTOR_MAX_EVENTS = 64;
const MAX_TICK = 360000;
const copy = value => structuredClone(value);
function fields(value, keys) {
  if (!value || ![Object.prototype, null].includes(Object.getPrototypeOf(value)) ||
      Reflect.ownKeys(value).length !== keys.length || keys.some(key => !Object.hasOwn(value, key)))
    throw new Error(`教官資料欄位不正確： ${keys.join(', ')}`);
}
function number(value, min, max, integer = false) {
  if (!Number.isFinite(value) || value < min || value > max || (integer && !Number.isInteger(value)))
    throw new Error('教官事件的數值或時刻超出有效範圍');
}
function bool(value) { if (typeof value !== 'boolean') throw new Error('教官事件的開關值必須為布林值'); }
function dense(value, max) {
  if (!Array.isArray(value) || value.length < 1 || value.length > max ||
      Reflect.ownKeys(value).length !== value.length + 1 ||
      Array.from({length:value.length}, (_, i) => i).some(i => !Object.hasOwn(value, i)))
    throw new Error('教官計畫必須包含 1 至 64 筆完整事件');
}
export function validateInstructorPlan(plan, vehicles) {
  fields(plan, ['name', 'events']);
  if (typeof plan.name !== 'string' || plan.name.trim().length < 1 || plan.name.length > 80)
    throw new Error('教官計畫名稱必須為 1 至 80 個字元');
  dense(plan.events, INSTRUCTOR_MAX_EVENTS);
  let previous = 0;
  for (const event of plan.events) {
    if (!event || typeof event.type !== 'string') throw new Error('教官事件格式不正確');
    number(event.tick, 1, MAX_TICK, true);
    if (event.tick < previous) throw new Error('教官事件必須依觸發時刻排列');
    previous = event.tick;
    if (event.type === 'wind') {
      fields(event, ['tick', 'type', 'value']);
      fields(event.value, ['speed', 'direction']);
      number(event.value.speed, 0, 30); number(event.value.direction, 0, 360);
      continue;
    }
    if (typeof event.vehicleId !== 'string' || !/^[A-Za-z0-9][A-Za-z0-9._-]{0,31}$/.test(event.vehicleId))
      throw new Error('教官事件的載具編號格式不正確');
    const vehicle = vehicles.find(vehicle => vehicle.id === event.vehicleId);
    if (!vehicle) throw new Error('教官事件指定的載具不存在');
    if (['clearFaults', 'gust'].includes(event.type)) fields(event, ['tick', 'vehicleId', 'type']);
    else if (event.type === 'sensorFault') {
      fields(event, ['tick', 'vehicleId', 'type', 'value']);
      fields(event.value, ['sensor', 'failed']);
      if (!SENSOR_NAMES.includes(event.value.sensor)) throw new Error('教官事件指定的感測器不存在');
      bool(event.value.failed);
    } else if (event.type === 'fault') {
      fields(event, ['tick', 'vehicleId', 'type', 'fault', 'value']);
      const airframe = vehicle.config.airframe;
      if (event.fault === 'motor') {
        if (airframe === 'fixedwing') throw new Error('固定翼載具不支援升力馬達故障');
        fields(event.value, ['index', 'efficiency']);
        number(event.value.index, 0, rotorCount(airframe) - 1, true);
        number(event.value.efficiency, 0, 1);
      } else if (event.fault === 'link') {
        fields(event.value, ['lost']); bool(event.value.lost);
      } else if (event.fault === 'pusher') {
        if (!['fixedwing', 'vtol'].includes(airframe)) throw new Error('此機型不支援推進器故障');
        fields(event.value, ['efficiency']); number(event.value.efficiency, 0, 1);
      } else throw new Error('不支援此教官故障類型');
    } else throw new Error('不支援此教官事件類型');
  }
  return copy(plan);
}
export function instructorPlanToJson(plan, vehicles) {
  return JSON.stringify({format:'flight-lab-instructor-plan', version:1, plan:validateInstructorPlan(plan, vehicles)}, null, 2);
}
export function parseInstructorPlan(text, vehicles) {
  if (typeof text !== 'string' || new TextEncoder().encode(text).length > 65536) throw new Error('教官計畫檔案不得超過 64 KiB');
  let artifact;
  try { artifact = JSON.parse(text); }
  catch { throw new Error('教官計畫檔案不是有效的 JSON'); }
  fields(artifact, ['format', 'version', 'plan']);
  if (artifact.format !== 'flight-lab-instructor-plan' || artifact.version !== 1) throw new Error('不支援此教官計畫檔案格式或版本');
  return validateInstructorPlan(artifact.plan, vehicles);
}
export function instructorReportToJson(status) {
  fields(status, ['phase', 'plan', 'appliedAtTick', 'nextIndex', 'receipts', 'reason']);
  if (!['idle','armed','complete','cancelled','error'].includes(status.phase) || !Array.isArray(status.receipts) || status.receipts.length > 64 || typeof status.reason !== 'string') throw new Error('教官狀態格式不正確');
  number(status.nextIndex, 0, INSTRUCTOR_MAX_EVENTS, true);
  if (status.appliedAtTick !== null) number(status.appliedAtTick, 0, MAX_TICK, true);
  if (status.plan !== null) {
    fields(status.plan, ['name', 'events']);
    if (typeof status.plan.name !== 'string') throw new Error('教官狀態中的計畫格式不正確');
    dense(status.plan.events, INSTRUCTOR_MAX_EVENTS);
  }
  for (const receipt of status.receipts) {
    fields(receipt, ['index', 'tick', 'status', 'error']);
    number(receipt.index, 0, INSTRUCTOR_MAX_EVENTS - 1, true);
    number(receipt.tick, 1, MAX_TICK, true);
    if (!['applied', 'failed'].includes(receipt.status) || typeof receipt.error !== 'string') throw new Error('教官執行收據格式不正確');
  }
  return JSON.stringify({format:'flight-lab-instructor-report', version:1, ...status}, null, 2);
}
export function createInstructorScheduler() {
  let status;
  function reset() { status = {phase:'idle', plan:null, appliedAtTick:null, nextIndex:0, receipts:[], reason:''}; }
  reset();
  return {
    reset,
    getState: () => copy(status),
    apply(plan, currentTick, vehicles) {
      number(currentTick, 0, MAX_TICK, true);
      const validated = validateInstructorPlan(plan, vehicles);
      if (status.phase === 'armed') throw new Error('請先取消待執行的教官計畫，再套用新計畫');
      if (validated.events.some(event => event.tick <= currentTick)) throw new Error('教官事件的觸發時刻必須晚於目前模擬時刻');
      status = {phase:'armed', plan:validated, appliedAtTick:currentTick, nextIndex:0, receipts:[], reason:''};
      return copy(status);
    },
    cancel(reason = '使用者取消') {
      if (typeof reason !== 'string') throw new Error('教官計畫的取消原因格式不正確');
      if (status.phase === 'armed') { status.phase = 'cancelled'; status.reason = reason; }
      return copy(status);
    },
    run(currentTick, execute) {
      number(currentTick, 0, MAX_TICK, true);
      while (status.phase === 'armed' && status.nextIndex < status.plan.events.length) {
        const index = status.nextIndex, event = status.plan.events[index];
        if (event.tick > currentTick) break;
        try {
          if (event.tick < currentTick) throw new Error('已錯過教官事件的指定時刻，計畫已停止');
          const {tick, ...command} = copy(event);
          execute(command);
          status.receipts.push({index, tick:currentTick, status:'applied', error:''});
          status.nextIndex++;
          if (status.phase === 'armed' && status.nextIndex === status.plan.events.length) status.phase = 'complete';
        } catch (error) {
          status.reason = error instanceof Error ? error.message : String(error);
          status.receipts.push({index, tick:currentTick, status:'failed', error:status.reason});
          status.phase = 'error';
        }
      }
      return copy(status);
    },
  };
}
