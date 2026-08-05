"""
sbgc_sim.py — a simulated SimpleBGC32 controller on a pseudo-terminal, plus the
scaffolding the integration tests share.

Why this exists: the interesting failures in gimbal_gui are not in the protocol
encoder, which test_sbgc_api.c already covers byte-for-byte. They are in the
serial loop's decisions — when a stop is owed, when a rate must be suppressed,
what happens when the board stops answering. None of those can be reached
without a board on the other end of the port, and all of them are exactly the
cases you cannot safely stage on real hardware with a camera bolted to it.

So the board is simulated. It speaks the framing from sbgc_api.c, answers the
queries the daemon actually sends, and lets a test drive motor state and
reported angles directly. Frames it receives are recorded so a test can assert
on what went on the wire rather than on what the UI claims went on the wire.

Only the commands the daemon uses are implemented. Anything else is recorded
and ignored, which is also what an unfamiliar board would do.
"""

import json
import os
import pty
import select
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GUI_BIN = os.path.join(ROOT, "build", "gimbal_gui")
CTL_BIN = os.path.join(ROOT, "build", "gimbal_ctl")

# --- protocol constants, mirroring include/sbgc_api.h and sbgc_params.h -----
START_BYTE = 0x3E
CMD_READ_PARAMS_3 = 21
CMD_REALTIME_DATA_3 = 23
CMD_BOARD_INFO_3 = 20
CMD_BOARD_INFO = 86
CMD_CONTROL = 67
CMD_EXECUTE_MENU = 69
CMD_MOTORS_ON = 77
CMD_MOTORS_OFF = 109
MENU_CALIB_GYRO = 9

PARAMS_3_LEN = 134
REALTIME_3_LEN = 63
BOARD_INFO_LEN = 18

ANGLE_UNIT_DEG = 0.02197265625   # 360 / 16384
SPEED_UNIT_DEGS = 0.1220740379


def encode(cmd, payload=b""):
    """Frame a command exactly as sbgc_encode_frame() does."""
    head = bytes([START_BYTE, cmd, len(payload), (cmd + len(payload)) & 0xFF])
    return head + payload + bytes([sum(payload) & 0xFF])


def i16(v):
    return int(v).to_bytes(2, "little", signed=True)


def deg_to_units(deg):
    return int(round(deg / ANGLE_UNIT_DEG))


class Board:
    """
    A SimpleBGC32 on a PTY. Open it, hand `device` to gimbal_gui, and drive
    `motors_on` / `angles` from the test.

    Everything shared with the reader thread goes through `lock`.
    """

    def __init__(self, motors_on=False, angles=(0.0, 0.0, 0.0),
                 rc_limits=None, follow_mode=1, skip_gyro_calib=1,
                 answer_realtime=True):
        self.lock = threading.Lock()
        self.motors_on = motors_on
        self.angles = list(angles)          # ROLL, PITCH, YAW, board convention
        self.battery_v = 12.0
        # Per-axis (min, max) RC angle limits in the BOARD's convention, i.e.
        # pitch positive-downward. None means "no limit configured" (0, 0).
        self.rc_limits = rc_limits or {}
        self.follow_mode = follow_mode
        self.skip_gyro_calib = skip_gyro_calib
        self.answer_realtime = answer_realtime

        self.received = []                  # (cmd, payload) in arrival order
        self.calibs = []                    # monotonic time of each calib
        self.controls = []                  # decoded CMD_CONTROL frames

        self._master, self._slave = pty.openpty()
        self.device = os.ttyname(self._slave)
        self._run = True
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    # -- payload builders ---------------------------------------------------

    def _realtime_payload(self):
        b = bytearray(REALTIME_3_LEN)
        # rc_data: the -10000 sentinel on every channel means "no RC signal",
        # which is the honest state for a board with nothing plugged into it.
        for i in range(6):
            b[20 + i * 2:22 + i * 2] = i16(-10000)
        for a in range(3):
            b[32 + a * 2:34 + a * 2] = i16(deg_to_units(self.angles[a]))
        b[50:52] = (1000).to_bytes(2, "little")            # cycle time us
        b[55:57] = (int(self.battery_v * 100)).to_bytes(2, "little")
        b[57] = 0x01 if self.motors_on else 0x00           # rt_data_flags bit0
        return bytes(b)

    def _params_payload(self):
        b = bytearray(PARAMS_3_LEN)
        for idx, name in enumerate(("roll", "pitch", "yaw")):
            lo, hi = self.rc_limits.get(name, (0, 0))
            off = 22 + idx * 8
            b[off:off + 2] = i16(lo)
            b[off + 2:off + 4] = i16(hi)
        b[46] = 100                    # gyro_trust
        b[64] = self.follow_mode
        b[78] = self.skip_gyro_calib
        return bytes(b)

    def _board_info_payload(self):
        b = bytearray(BOARD_INFO_LEN)
        b[0] = 31                                          # board 3.1
        b[1:3] = (2630).to_bytes(2, "little")              # firmware 2.63 b0
        return bytes(b)

    # -- wire ---------------------------------------------------------------

    def _serve(self):
        buf = b""
        while self._run:
            try:
                r, _, _ = select.select([self._master], [], [], 0.05)
            except (OSError, ValueError):
                break
            if not r:
                continue
            try:
                buf += os.read(self._master, 4096)
            except OSError:
                break

            while len(buf) >= 5:
                if buf[0] != START_BYTE:
                    buf = buf[1:]
                    continue
                length = buf[2]
                if len(buf) < 5 + length:
                    break
                cmd, payload = buf[1], buf[4:4 + length]
                buf = buf[5 + length:]
                self._dispatch(cmd, payload)

    def _dispatch(self, cmd, payload):
        with self.lock:
            self.received.append((cmd, bytes(payload)))

            if cmd == CMD_CONTROL:
                self.controls.append(self._decode_control(payload))
            elif cmd == CMD_EXECUTE_MENU and payload and payload[0] == MENU_CALIB_GYRO:
                self.calibs.append(time.monotonic())
            elif cmd == CMD_MOTORS_ON:
                self.motors_on = True
            elif cmd == CMD_MOTORS_OFF:
                self.motors_on = False

            reply = None
            if cmd == CMD_REALTIME_DATA_3 and self.answer_realtime:
                reply = encode(CMD_REALTIME_DATA_3, self._realtime_payload())
            elif cmd == CMD_READ_PARAMS_3:
                reply = encode(CMD_READ_PARAMS_3, self._params_payload())
            elif cmd in (CMD_BOARD_INFO, CMD_BOARD_INFO_3):
                reply = encode(cmd, self._board_info_payload())

        if reply:
            try:
                os.write(self._master, reply)
            except OSError:
                pass

    @staticmethod
    def _decode_control(payload):
        """{'modes': [...], 'speed': [...], 'angle': [...]} in ROLL,PITCH,YAW."""
        if len(payload) < 15:
            return {"modes": [], "speed": [], "angle": []}
        modes = list(payload[0:3])
        speed, angle = [], []
        for a in range(3):
            off = 3 + a * 4
            speed.append(int.from_bytes(payload[off:off + 2], "little", signed=True))
            angle.append(int.from_bytes(payload[off + 2:off + 4], "little", signed=True))
        return {"modes": modes, "speed": speed, "angle": angle}

    # -- test-facing helpers ------------------------------------------------

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, list(v) if k == "angles" else v)

    def snapshot(self, name):
        with self.lock:
            return list(getattr(self, name))

    def clear(self, *names):
        with self.lock:
            for n in names:
                getattr(self, n).clear()

    def moving_controls(self):
        """CMD_CONTROL frames that command a non-zero speed on any axis."""
        with self.lock:
            return [c for c in self.controls if any(s != 0 for s in c["speed"])]

    def close(self):
        self._run = False
        self._thread.join(timeout=1.0)
        for fd in (self._master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass


def free_port():
    """A port the OS says is free, so parallel or repeated runs cannot clash."""
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class Daemon:
    """gimbal_gui pointed at a simulated board, with an HTTP client attached."""

    def __init__(self, board, extra_args=(), wait_for_board=True):
        self.port = free_port()
        self.base = f"http://127.0.0.1:{self.port}"
        self.proc = subprocess.Popen(
            [GUI_BIN, "--http-port", str(self.port), "--no-pad",
             "--port", board.device, *extra_args],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self._await_http()
        if wait_for_board:
            self.await_status(lambda s: s["link"]["board_responding"], 5.0,
                              "board never answered")

    def _await_http(self, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError("gimbal_gui exited: " + self.proc.stdout.read())
            try:
                self.status()
                return
            except Exception:
                time.sleep(0.05)
        raise RuntimeError("gimbal_gui never served /api/status")

    def status(self):
        with urllib.request.urlopen(self.base + "/api/status", timeout=3) as f:
            return json.load(f)

    def control(self):
        return self.status()["control"]

    def post(self, path, body="", origin=None):
        req = urllib.request.Request(self.base + path, data=body.encode(),
                                     method="POST")
        req.add_header("Content-Type", "application/x-www-form-urlencoded")
        if origin:
            req.add_header("Origin", origin)
        try:
            with urllib.request.urlopen(req, timeout=3) as f:
                return f.status, json.load(f)
        except urllib.error.HTTPError as e:
            return e.code, json.load(e)

    def await_status(self, pred, timeout, what):
        """Poll until `pred(status)` holds. Returns the status, or raises."""
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = self.status()
            if pred(last):
                return last
            time.sleep(0.05)
        # Report the control block only. The full document is several KB of
        # port listings and profile tables, none of which is why the wait
        # failed, and burying the relevant fields in it helps nobody.
        detail = json.dumps(last.get("control", last)) if last else "no status"
        raise AssertionError(f"timed out waiting for {what}: {detail[:400]}")

    def hold_rate(self, seconds, body="pan=1.0&tilt=0"):
        """Republish a held control at 20 Hz, exactly as the browser does."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.post("/api/control/rate", body)
            time.sleep(0.05)

    def warnings(self):
        return " ".join(w["text"] for w in self.status()["warnings"])

    def close(self):
        # SIGTERM, not SIGKILL: the daemon's shutdown path sends a stop before
        # releasing the port, and skipping it is the very hazard this project
        # is careful about.
        self.proc.send_signal(signal.SIGTERM)
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait(timeout=5)


class Results:
    """Tallies checks in the same shape test_sbgc_api.c prints."""

    def __init__(self):
        self.checks = 0
        self.failures = 0

    def check(self, what, cond, extra=""):
        self.checks += 1
        if cond:
            print(f"ok   {what}")
        else:
            self.failures += 1
            print(f"FAIL {what}" + (f"\n     {extra}" if extra else ""))
        return bool(cond)

    def section(self, title):
        print(f"\n== {title} ==")

    def run(self, fn):
        """
        Run one test, converting a crash into a reported failure.

        A test that blows up — a missing status field, a wait that never
        completed — must not take the rest of the suite with it. When several
        things regress at once, seeing all of them beats seeing the first.
        """
        try:
            fn()
        except Exception as exc:                     # noqa: BLE001
            self.checks += 1
            self.failures += 1
            print(f"FAIL {fn.__name__} raised {type(exc).__name__}: {exc}")

    def report(self):
        print("\n" + "-" * 40)
        print(f"{self.checks} checks, {self.failures} failures")
        return 1 if self.failures else 0


def quick_mode():
    """True when the caller asked to skip the slow tests."""
    return "--quick" in sys.argv[1:]


def require_binary(path):
    if not os.access(path, os.X_OK):
        print(f"SKIP: {path} not built — run 'make' first")
        sys.exit(77)
