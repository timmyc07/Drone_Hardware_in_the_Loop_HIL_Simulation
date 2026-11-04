# ArduPilot SITL loopback patch

`ardupilot-loopback.patch` modifies [ArduPilot Copter-4.7.2](https://github.com/ArduPilot/ardupilot/tree/Copter-4.7.2), base commit `fb773567896014da007bca58358704d4a0d8516f`. The upstream files and this derived patch are GPL-3.0-or-later; the upstream license is retained in `COPYING.ardupilot`. No ArduPilot binary is bundled.

Patch SHA-256: `892be52e36b8e6bdbc7dd6f4208a99f250b1aae116abd2e8080b9b3b068c0e42`.

The JSON backend normally sends to localhost without explicitly binding its receiving socket. The patch binds that socket to `127.0.0.1` when its target is `127.0.0.1`, and exits on bind failure. The RC UDP backend also uses a loopback ephemeral port when `--rc-in-port 0` is supplied; port zero does not disable that backend. Nonzero RC ports retain their upstream behavior.

Apply to the exact upstream checkout before building:

```sh
git apply --check /absolute/path/to/companion/patches/ardupilot-loopback.patch
git apply /absolute/path/to/companion/patches/ardupilot-loopback.patch
```

The macOS arm64 test build's patched executable SHA-256 is `a56f45f38e1fb78af5ae5cf5770d01a6a1d1a630308854a2867fe6ac9b67945c`. It identifies that build, not every valid build on another machine. Live socket inspection confirmed two loopback UDP endpoints and a loopback MAVLink TCP connection. A firmware version or short Git hash self-report does not prove that this patch is present; the build and socket validation must also be retained. Flight validation is documented separately.

## PX4 SITL explicit loopback patch

`px4-loopback.patch` modifies PX4 v1.17.0 base commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`. Its BSD-3-Clause license is retained in `LICENSE.px4`; no native executable is bundled. With explicit `mavlink start -t 127.0.0.1`, the UDP receiving socket also binds to `127.0.0.1`. Other targets and default behavior are unchanged. Apply with `git apply --check` and `git apply` to that exact checkout before building.

Patch SHA-256: `a66390ec624cb83a7e6ae2a4971634418e84629618e668b957466d12e1ec6674`.
The macOS arm64 native test build SHA-256: `04634c9d42ecc77842257dd85f96013206eb009063768931d5102c3c97303b4a`.
These are build provenance, not cryptographic firmware attestation. PX4's console still self-reports its base source SHA. After startup, live socket inspection confirmed only TCP `127.0.0.1` to the owned HIL listener on 4560 and UDP `127.0.0.1:14560` for missions. The startup hook stops DDS after rcS starts it; the inspected running process had no DDS socket. Node's mission endpoint binds `127.0.0.1:14561`. The session enables no broadcast, forwarding or FTP flag.
