import test from 'node:test';
import assert from 'node:assert/strict';
import { randomUUID } from 'node:crypto';
import { NIST_SLOTS,nistReportToJson,parseNistReport } from '../nist.js';
import { createWorld,defaultWorldConfig } from '../world.js';
import { cameraMetadata,hashCameraFrame,DEFAULT_CAMERA_CONFIG } from '../camera.js';
import { RECORDING_VERSION,MODEL_VERSION,compactWorldSnapshot,worldSensorFrame,validateRecording,recordingToJson,recordingNistState } from '../recording.js';
import { replayWorldRecording } from '../../tools/replay-world.mjs';

test('all 36 evidence slots survive sparse record20 serialization, independent physical replay and report import',async()=>{
  const config=structuredClone(defaultWorldConfig({startOnGround:true}));config.scene.nistLane=true;
  const world=createWorld(config), frame=world.snapshot(), vehicleId=frame.vehicles[0].id;
  const record={format:'flight-lab',version:RECORDING_VERSION,modelVersion:MODEL_VERSION,createdAt:'2026-11-05T00:00:00.000Z',dt:.005,world:world.configuration,runtime:{speed:1,wallSeconds:0,p95LateMs:0,lateTicks:0,skippedWallSeconds:0,renderEnabled:true},events:[],samples:[compactWorldSnapshot(frame)],sensorFrames:[worldSensorFrame(frame)],termination:null};
  const event=(type,value)=>record.events.push({type,value,vehicleId,tick:0,time:0});
  event('nistStart',{sessionId:randomUUID(),durationSeconds:600});
  for(const slot of NIST_SLOTS) {
    const captureId=randomUUID(), metadata=cameraMetadata(frame.vehicles[0].snapshot,{...DEFAULT_CAMERA_CONFIG,width:64,height:48},null,{captureId,sourceVehicleId:vehicleId,worldTick:0});
    const pixels=metadata.width*metadata.height;
    const image={...metadata,rgbRgba:new Uint8Array(pixels*4),depthM:new Float32Array(pixels),classes:new Uint8Array(pixels)};
    event('nistCapture',{slotId:slot.id,captureId,metadata,sha256:await hashCameraFrame(image)});
    event('nistScore',{slotId:slot.id,value:0});
  }
  event('nistEnd',{reason:'manual'});
  const clean=validateRecording(JSON.parse(recordingToJson(record)));
  const status=recordingNistState(clean,0);
  assert.equal(status.captures.length,36);assert.equal(status.totalScore,0);assert.equal(status.phase,'ended');
  assert.equal(parseNistReport(nistReportToJson(status)).session.captures.length,36);
  assert.equal(replayWorldRecording(clean).passed,true);
  assert.equal(Object.hasOwn(clean.samples[0].vehicles[0].snapshot,'nist'),false);
  for(const mutate of [
    r=>r.events[1].value.captureId=r.events[3].value.captureId,
    r=>r.events[1].vehicleId='unknown',
    r=>r.events[1].value.metadata.pose.positionNed[0]+=1,
    r=>r.events[1].value.metadata.gimbal.target.pitch=.1,
    r=>r.events[2].value.slotId='s01-2',
    r=>r.events[1].tick=1,
    r=>r.events[1].value.fake=true,
    r=>r.events[0].value.durationSeconds=29,
  ]) {const bad=structuredClone(clean);mutate(bad);assert.throws(()=>validateRecording(bad),undefined,mutate.toString());}
});

test('same-tick operator gimbal epochs bind NIST exposures before and after commands without confusing final sample settings', async () => {
  const config=structuredClone(defaultWorldConfig({startOnGround:true})); config.scene.nistLane=true;
  const world=createWorld(config), initial=world.snapshot(), vehicleId=initial.vehicles[0].id;
  const record={format:'flight-lab',version:RECORDING_VERSION,modelVersion:MODEL_VERSION,createdAt:'2026-11-05T00:00:00.000Z',dt:.005,world:world.configuration,runtime:{speed:1,wallSeconds:0,p95LateMs:0,lateTicks:0,skippedWallSeconds:0,renderEnabled:true},events:[],samples:[],sensorFrames:[],termination:null};
  const event=(type,value)=>record.events.push({type,value,vehicleId,tick:0,time:0});
  const command=yaw=>{
    const settings=structuredClone(config.vehicles[0].setup.gimbal);settings.target.yaw=yaw;
    world.command(vehicleId,'gimbal',settings); event('operatorGimbal',settings);
  };
  event('nistStart',{sessionId:randomUUID(),durationSeconds:600});
  for (const [index, yaw] of [.2,.5].entries()) {
    command(yaw);
    const frame=world.snapshot(),captureId=randomUUID();
    const metadata=cameraMetadata(frame.vehicles[0].snapshot,{...DEFAULT_CAMERA_CONFIG,width:64,height:48},null,{captureId,sourceVehicleId:vehicleId,worldTick:0});
    const pixels=metadata.width*metadata.height;
    const image={...metadata,rgbRgba:new Uint8Array(pixels*4),depthM:new Float32Array(pixels),classes:new Uint8Array(pixels)};
    event('nistCapture',{slotId:NIST_SLOTS[index].id,captureId,metadata,sha256:await hashCameraFrame(image)});
  }
  command(.8); event('nistEnd',{reason:'manual'});
  const final=world.snapshot(); record.samples=[compactWorldSnapshot(final)];record.sensorFrames=[worldSensorFrame(final)];
  const clean=validateRecording(JSON.parse(recordingToJson(record)));
  const status=recordingNistState(clean,0);
  assert.deepEqual(status.captures.map(c=>c.metadata.gimbal.target.yaw),[.2,.5]);
  assert.equal(clean.samples[0].vehicles[0].snapshot.gimbal.target.yaw,.8);
  assert.equal(replayWorldRecording(clean).passed,true);
  const bad=structuredClone(clean); [bad.events[1],bad.events[2]]=[bad.events[2],bad.events[1]];
  assert.throws(()=>validateRecording(bad));
});
