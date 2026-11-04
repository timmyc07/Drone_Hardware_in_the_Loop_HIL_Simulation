# Native PX4 SITL adapter

This adapter connects a real, locally owned PX4 process to Flight Lab's JavaScript quadrotor rigid-body plant. It is software-in-the-loop, not a physical flight controller or hardware-in-the-loop implementation. Fixed-wing and VTOL external control are unsupported.

The supported firmware is PX4 **v1.17.0**, source SHA `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, airframe `10016_none_iris`. `ver all` must report the exact SHA before flight commands are accepted. `running` means sensor/control packets are flowing; check `controller.verified` and PX4's own health checks before flight. A console command returning `sent:true` is not a flight acknowledgement. `commander takeoff` selects autonomous takeoff and requests arming; standalone `arm` can legitimately be refused in an unsuitable flight mode.

```js
const session = new Px4Session();
await session.start({ binaryPath: "/absolute/path/to/px4", config: {} });
session.on("state", (state) => {
  /* 20 wall Hz; snapshot is plant truth */
});
session.command("takeoff");
session.command("land");
const recording = session.exportRecording();
await session.close();
```

Only `arm`, `takeoff`, `land`, `disarm`, `pause`, `resume`, and `stop` are accepted. Flight commands use the owned process's fixed POSIX console commands, not MAVLink GCS commands. Start creates a fresh isolated runtime; restarting requires a new `Px4Session`. `close()` waits for the child and TCP listener to exit and removes its runtime directory. Export before close to retain logs; the verification script copies ULog files before cleanup.

TCP listens only on `127.0.0.1:4560`. The PX4 simulator connects to that loopback listener. Its GCS startup script is replaced with an owned script which starts no GCS transports and stops the optional DDS client. No remote network endpoint or arbitrary console command is accepted.

The plant uses a 4ms simulated step. Startup advances disarmed stationary ticks until PX4 produces controls; then each new timestamp-matched `HIL_ACTUATOR_CONTROLS` advances exactly one step. Waiting, pause and watchdog faults freeze the simulated clock. Wall pacing depends on the host and is not a hard real-time guarantee. Pause retains an already in-flight control packet for consumption on resume; a transport fault requires a new session. There is no internal PID fallback.

Plant coordinates are NED, body FRD, SI units, quaternion `[w,x,y,z]` body-to-NED. Accelerometers provide specific force including ground support. Sensor timestamps are integer plant microseconds plus **1,000,000µs** boot offset. PX4 output order FR/RL/FL/RR maps to plant FR/RR/RL/FL using `[0,3,1,2]`. `THR_MDL_FAC=1` makes PX4 invert its normalized thrust demand to a rotor-speed signal, matching the plant's speed-squared thrust model. `COM_RC_IN_MODE=4` explicitly declares no RC input; estimator and arming health gates remain enabled. The controller retains the tagged Iris allocator geometry/tuning while the educational plant retains its configured geometry.

GPS is an ideal synthetic 3D fix at the declared reference origin, with explicitly assumed 10 satellites, eph 1m and epv 1.5m as interpreted by PX4's simulator receiver. This is not a constellation or DOP simulation. Magnetometer earth field uses the tagged PX4 WMM-2020 lookup data, with its BSD notice retained in `px4-wmm.js`. Pressure uses the ISA model. A fixed deterministic uniform sensor noise floor avoids PX4's constant-value sensor-stuck detection: standard deviations accel .01m/s², gyro .001rad/s, magnetic field .0001 gauss, pressure 2.7Pa. It derives from exact tick time and `config.seed`; the internal controller's `config.noise` multiplier is not used. This is declared in exported metadata.

The separate `flight-lab-px4` version 1 recording stores every accepted actuator packet and mapped speed command, bootstrap tick count, command/gust events, sampled plant truth, clock convention, source identity and sensor assumptions. Samples use the next 50ms grid threshold rounded to the next 4ms tick, with no bootstrap catch-up burst; export also includes the final physical state. Raw actuator timestamps and all 16 raw controls are preserved. Recording stops at 30 simulated minutes or the 96MiB soft limit, and cannot resume after that limit. These are not the browser internal-controller recording format.

For offline plant replay, first integrate `bootstrapTicks` zero/disarmed 4ms ticks. Apply recorded gust events at their recorded plant time, then apply each actuator's `sequence`, `timeUs`, `armed`, and `rotorSpeedCommands` before one 4ms step. Raw sensor messages can be regenerated from the same current truth, origin, seed and clock convention; no external estimator state is fabricated. The existing snapshot's target/navigation/tracking-error fields belong to the unused internal controller and do not represent PX4 setpoints, modes or estimator quality. `controller.estimated` remains null.

Run protocol tests with `node --test companion/tests/px4-protocol.test.js`. Run the opt-in real firmware flight proof with:

```sh
node companion/verify-px4.mjs /absolute/path/to/px4 /absolute/artifact/directory
```

It requires real version verification, autonomous arming/takeoff, at least 60 simulated seconds of hover including a deterministic gust, landing and PX4 automatic disarming. It checks tracking limits, CRC and source timestamp consistency, then preserves accepted inputs, summary and native ULog files. It does not claim physical HIL validation.

Implementation references: [PX4 v1.17.0 simulator](https://github.com/PX4/PX4-Autopilot/tree/v1.17.0/src/modules/simulation), [tagged Iris airframe](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/ROMFS/px4fmu_common/init.d-posix/airframes/10016_none_iris), and [MAVLink common messages](https://mavlink.io/en/messages/common.html). `node-mavlink` **2.3.0** API signatures were checked against its installed distribution; Context7's `/ardupilot/node-mavlink` index has no version-specific 2.3.0 documentation, so its general documentation was supplemented with the installed source.
