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
