# SimpleBGC32 Control

Direct, standalone control and monitoring for BaseCam/AlexMos SimpleBGC 32-bit
gimbal controllers on Linux. This project talks to the controller over its
serial API and does not require ROS, an autopilot, or the vendor application.

It provides:

- an interactive command-line controller;
- a self-contained browser control and status console;
- keyboard, pointer, touch, and gamepad control;
- a strictly read-only board probe;
- a small C API for the supported SimpleBGC serial commands; and
- protocol tests checked against BaseCam's published byte examples.

The project complements the SimpleBGC configuration application. Use the
vendor application to balance, tune, configure, and set controller-side limits;
use this project when you want direct operator control or want to integrate the
controller into your own system.

> [!CAUTION]
> **This is experimental software that controls physical hardware. It comes
> with no safety guarantees.** There is no warranty that its limits, watchdogs,
> stop commands, protocol handling, or defaults will prevent injury or damage.
> A wrong axis mapping, unsuitable tuning, software defect, stalled process,
> disconnected serial cable, or controller/firmware difference can cause
> unexpected or continued motion. Keep people and property outside the motion
> envelope, provide an independent way to cut motor power, start at low speed,
> and test without a payload whenever possible. You are responsible for
> evaluating and operating the hardware safely.

## Supported hardware and platform

The control protocol targets **BaseCam/AlexMos SimpleBGC 32-bit controllers**
using Serial API protocol version 1 (`>` / `0x3e`, 8-bit checksums).

Development and hardware verification were performed with:

- SimpleBGC board version 3.1;
- firmware 2.63b0; and
- a 3-axis 43xx-class controller/gimbal setup.

The basic version 1 control protocol is intended to work across SimpleBGC
32-bit firmware versions, but that is compatibility intent, **not a guarantee**.
In particular, the detailed `CMD_READ_PARAMS_3` decoder only accepts the exact
134-byte payload verified on firmware 2.63b0. It refuses other layouts instead
of guessing at field offsets.

The current implementation is Linux-specific: serial access uses POSIX
`termios`, local gamepads use Linux `evdev`, and the build uses Make plus a C11
and C++17 compiler.

This is an unofficial community project. It is not affiliated with or endorsed
by BaseCam Electronics or AlexMos.

## Quick start

Build requirements are a C11 compiler, a C++17 compiler, GNU Make, pthreads,
and the standard Linux development headers. There are no third-party runtime
libraries.

```bash
make
make test
```

Start with simulation. It exercises the CLI and prints the exact frames it
would send without opening a serial device:

```bash
./build/gimbal_ctl --simulate
```

For read-only hardware inspection:

```bash
make probe
./build/sbgc_probe --port /dev/ttyUSB0 --out probe.txt
```

For the browser status console, which is monitor-only by default:

```bash
./build/gimbal_gui --port /dev/ttyUSB0
# Open http://127.0.0.1:8080
```

To make motion controls available, you must opt in at startup and then arm the
controls in the page:

```bash
./build/gimbal_gui --port /dev/ttyUSB0 --allow-control
```

The web server has no user authentication. It binds to `127.0.0.1` by default.
Do not use `--bind 0.0.0.0` unless the host is on a trusted, access-controlled
network.

## Before connecting hardware

1. Mechanically balance the complete gimbal and payload.
2. Tune the motors and verify the controller profile in the SimpleBGC
   application.
3. Set controller-side travel limits where the hardware and firmware support
   them. The limits in this project are supplemental software limits, not a
   safety system.
4. Close the SimpleBGC application; it normally holds the serial port
   exclusively.
5. Run `make test`, then use `gimbal_ctl --simulate` to check the command flow.
6. Clear the entire possible motion envelope and prepare an independent way to
   cut motor power.
7. Connect with the motors disabled, verify telemetry, and confirm axis mapping
   using small movements at low speed before using continuous-rate control.

Never assume that a successful build or test run proves a particular physical
setup is safe. The unit tests validate frame construction and parsing; they do
not validate wiring, tuning, motor direction, payload balance, travel clearance,
firmware behavior, or emergency-stop behavior.

## Browser console

`gimbal_gui` serves one page compiled into the binary. It displays connection
state, board replies, firmware, motor state, battery voltage, RC state, active
profile information, measured and target angles, configured travel limits, and
warnings.

```bash
./build/gimbal_gui [options]

  --port DEV        serial device (GUI setting or /dev/ttyUSB0 by default)
  --baud N          serial baud (GUI setting or 115200 by default)
  --http-port N     HTTP port (default 8080)
  --bind ADDR       bind address (default 127.0.0.1)
  --gui-dir DIR     SimpleBGC GUI installation to read defaults from
  --pad DEV         host gamepad event device; omit to auto-detect
  --no-pad          disable host gamepad discovery
  --allow-control   allow the page to arm commands that change hardware state
  --no-calib-gyro   disable automatic gyro calibration on motors-on
```

With no explicit port, baud, or GUI directory, the program looks for a local
SimpleBGC GUI installation and reads `conf/bgc.properties` plus an exported
`.profile`. Built-in defaults apply if none is found. Live configuration read
from the controller takes precedence over a saved file.

### Control gating

The console is read-only unless both of these independent conditions are met:

1. the process was started with `--allow-control`; and
2. the operator armed control in the web page.

Without both, motion and hardware-changing endpoints return HTTP 403 without
building a serial frame. The one exception is the automatic gyro calibration
described below: it is gated on `--allow-control` but not on arming, because
arming exists to gate operator-commanded motion and the calibration commands
none. The UI can control pan and tilt with pointer/touch,
`W A S D`, arrow keys, or a browser-connected gamepad. Roll is actively held at
0 degrees rather than exposed as an operator axis.

The browser republishes held rate commands at 20 Hz. If those commands stop,
the daemon attempts to send a stop within half a second. Disarming, releasing a
control, losing page focus, and a disconnected browser gamepad also request a
stop.

These are risk-reduction features, not safety guarantees.

### Critical serial-disconnect limitation

The watchdog can stop motion only while it can send a stop frame. If the serial
connection fails while a continuous rate is active, the controller may keep
turning at the last commanded rate. The application records that motion was
active and attempts to stop first after reconnecting, but it cannot stop the
controller while the link is unavailable.

For any setup where continued motion can cause harm, use an independent motor
power cutoff and consider an integration based on bounded angle targets rather
than continuous rates.

### Gyroscope calibration

When control is allowed, the gyroscope is calibrated every time the motors are
switched on. Powering the motors is what invalidates the stored zero-rate bias,
so it is the point at which recalibrating is worth doing. Use `--no-calib-gyro`
to disable it; the page's Calibrate button still works either way.

The gimbal must remain completely still during calibration — movement teaches an
incorrect bias and causes drift. Motors-on is also the moment the gimbal grabs
its position and settles, so the calibration is *armed* by motors-on but does
not fire until the controller's own reported angles show it has stopped moving.
If it never settles within 20 seconds the calibration is abandoned and reported,
rather than run on a moving gimbal. While one is running, held controls are
suppressed and no rate command is sent.

This is separate from the controller's own calibration at power-on, which is a
different and less frequent event; neither replaces the other.

The displayed duration is an estimate, not a verified completion response from
the board.

### Travel limits and angle handling

The console can enforce user-selected pitch and yaw limits against measured
angles. A component that would move farther outside its range is removed while
motion back toward the valid range remains possible.

The board can report continuous, unwrapped yaw values. The UI wraps those
values only for display. Commands derived from a measured position use the
board's raw units so a wrapped display angle cannot accidentally request a
target one or more full revolutions away.

Board-side limits remain authoritative. Software limits cannot protect against
a dead process, broken serial link, incorrect telemetry, incompatible firmware,
or commands sent by another program.

## Command-line controller

The CLI starts with a setup wizard before opening the serial device. It asks
which physical controller axis maps to pan, tilt, and roll, whether each is
inverted, and which soft limits to use.

```bash
./build/gimbal_ctl --port /dev/ttyUSB0 --baud 115200
./build/gimbal_ctl --simulate
./build/gimbal_ctl --defaults   # skip the setup wizard; use with care
```

Main commands:

```text
Direction (continuous rate until stopped)
  left | right | up | down | cw | ccw [deg/s]
  stop

Position
  pan <deg> | tilt <deg> | roll <deg>
  goto <pan> <tilt> [roll]
  nudge <pan> <tilt> [roll]
  home | level

Settings and input
  speed <deg/s> | step <deg>
  keys | live | pad | pads

Hardware and status
  read
  motors on | off
  setup | show | help | quit
```

Use `help <command>` for detailed behavior. Input is checked for unknown
commands, argument counts, complete numeric parsing, and implausible rates.
Angle requests beyond configured CLI limits are clamped and reported.

Keyboard debug mode (`keys`) shows the exact last transmitted frame. Lowercase
movement keys make discrete steps. Uppercase movement keys start continuous
rates, `Space` stops, and a five-second idle watchdog attempts to stop a rate
when no keypress arrives. Terminal input does not provide reliable key-release
events, so it cannot implement a true held-key deadman.

## Gamepads

The browser can use the viewing machine's Gamepad API. The CLI and daemon can
also discover Linux gamepads through `/dev/input/event*`.

Browser controls:

| Input | Action |
|---|---|
| Hold `LB` | Deadman; motion input is ignored unless held |
| Left stick horizontal | Pan |
| Right stick vertical | Tilt |
| `RB` | Speed boost |
| `A` | Home |
| `B` | Level |

CLI `pad` mode additionally supports coarse/fine stick mappings and roll on
the triggers. Releasing `LB` or disconnecting the controller requests a stop.
As with every software stop in this project, that request depends on a working
process, serial link, controller, and power path.

If the local gamepad is detected but cannot be opened, check permissions on
`/dev/input/event*` and membership in the system's `input` group.

## Serial connection

The default serial configuration is 115200 baud, 8N1, with no parity or flow
control. Depending on the controller and USB adapter, the device commonly
appears as `/dev/ttyUSB*` or `/dev/ttyACM*`.

Prefer a stable `/dev/serial/by-id/*` path. The browser console lists available
ports and can switch without restarting. It also attempts to follow a device
to a stable by-id path after re-enumeration.

For a custom stable name, first obtain the real USB vendor and product IDs with
`lsusb` and `udevadm`, then create a site-specific udev rule such as:

```udev
# /etc/udev/rules.d/99-simplebgc32-control.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", SYMLINK+="simplebgc32", MODE="0660", GROUP="dialout"
```

Do not copy placeholder IDs unchanged. Add the operator to the appropriate
serial-device group instead of running the controller as root.

## Tests

```bash
make test          # every suite (~45 s)
make test-quick    # same, minus one 20 s timeout case
```

Individual suites: `test-protocol`, `test-ctl`, `test-gui`, `test-page`.

| Suite | File | Covers |
|---|---|---|
| protocol | `test/test_sbgc_api.c` | framing, encoding, parsing, unit conversion |
| CLI | `test/test_gimbal_ctl.py` | `gimbal_ctl` frames under `--simulate` |
| daemon | `test/test_gimbal_gui.py` | `gimbal_gui` against a simulated controller |
| page | `test/test_web_page.js` | `web/index.html` control logic |

The protocol suite is C and always runs. The others need `python3`, and the
page suite needs `node`; a missing interpreter is reported as SKIPPED rather
than passing silently.

`test/sbgc_sim.py` simulates a SimpleBGC32 controller on a pseudo-terminal. It
answers the queries the daemon sends and lets a test drive motor state and
reported angles directly, so the daemon's behaviour can be checked when the
hardware misbehaves — a controller that stops reporting mid-move, a calibration
racing a held control — without hardware and without risk to a real gimbal.
Assertions are made on the frames the simulated controller received, not on
what the UI reports, because only the former is evidence about what would
reach the motors.

The page suite evaluates `web/index.html`'s own `<script>` block against a
minimal DOM shim and drives it with synthetic events, so it tests the shipped
page rather than a copy of its logic.

## Protocol verification

`test/test_sbgc_api.c` contains 37 checks, including byte-for-byte comparisons
with BaseCam's published
[`SBGC32 API command examples`](https://www.basecamelectronics.com/serialapi/).
The tested cases include all four documented `CMD_CONTROL` examples, home,
level, unit conversion, checksums, parser recovery, and angle decoding.

Confirmed wire units:

- angle: `360 / 16384` degrees per LSB (`45 deg` becomes `2048`);
- speed: approximately `0.1220740379 deg/s` per LSB (`5 deg/s` becomes `41`).

Protocol version 1 is used deliberately. Version 2 requires newer firmware and
can cause a controller to lock its session to version 2 after the first version
2 frame. Version 1 preserves compatibility with the older tested firmware.

## Project layout

```text
include/sbgc_api.h        serial framing, transport, and control API
include/sbgc_params.h     read-only configuration and telemetry decoder
include/sbgc_gui_config.h discovery of existing SimpleBGC GUI settings
include/httpd.h           minimal embedded HTTP server
include/gamepad.h         Linux evdev gamepad API
src/sbgc_api.c            SimpleBGC serial implementation
src/sbgc_params.c         configuration and telemetry decoding
src/sbgc_gui_config.c     local GUI configuration discovery
src/httpd.c               POSIX HTTP server
src/gamepad.c             Linux evdev implementation
src/gimbal_ctl.cpp        interactive CLI controller
src/gimbal_gui.cpp        browser console daemon
web/index.html            browser UI compiled into gimbal_gui
tools/sbgc_probe.c        strictly read-only board probe
test/test_sbgc_api.c      protocol and parser tests
test/sbgc_sim.py          simulated controller on a pty, shared by the tests
test/test_gimbal_ctl.py   CLI regression tests
test/test_gimbal_gui.py   daemon regression tests
test/test_web_page.js     browser UI regression tests
```

## Reference documentation

The implementation was checked against BaseCam's SimpleBGC32 documentation,
including the 2.6x user manual, encoder manual, 43xx hardware manual, serial API
specification, and published command examples. Obtain current copies from
[BaseCam's official documentation and Serial API pages](https://www.basecamelectronics.com/serialapi/).

The local `manuals/` directory is intentionally ignored. Vendor PDFs are not
part of this project's MIT-licensed source distribution, and their absence does
not affect the build.

## Scope and non-goals

This project does not:

- tune or balance a gimbal;
- replace the full SimpleBGC configuration application;
- update controller firmware;
- provide video capture or camera control;
- provide ROS or MAVLink integration;
- authenticate users of the browser console; or
- make the connected mechanism safe.

## Contributing

Hardware support claims should identify the board revision, firmware version,
serial adapter, and whether testing used a payload. Changes that can transmit
commands should include simulation or byte-level tests where practical and
must preserve monitor-only behavior when `--allow-control` is absent.

Please do not describe a software interlock, watchdog, travel limit, or stop
command as a safety guarantee.

## License

This project is licensed under the MIT License. See [`LICENSE`](LICENSE).

The license's warranty disclaimer and this README's hardware warning serve
different purposes. Neither makes a physical installation safe or guarantees
that the software will stop hazardous motion.
