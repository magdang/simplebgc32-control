#!/usr/bin/env python3
"""
test_gimbal_gui.py — regression tests for the daemon's serial-loop decisions.

Every case here corresponds to a defect that was found and fixed. They are
written against a simulated board (see sbgc_sim.py) because each one is about
what the daemon does when the hardware misbehaves — a board that goes silent
mid-move, a calibration racing a held stick — and those are precisely the
situations you cannot stage safely on a real gimbal.

Assertions are made on the frames the board received wherever possible. What
the daemon reports about itself is worth checking too, but it is not evidence
about what reached the motors.

    ./test_gimbal_gui.py [--quick]     --quick skips the 20 s timeout case
"""

import json
import os
import pty
import select
import socket
import subprocess
import sys
import threading
import time
import urllib.request

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from sbgc_sim import (Board, Daemon, Results, GUI_BIN, CMD_CONTROL,
                      CMD_BOARD_INFO, free_port, quick_mode, require_binary)

r = Results()

# SYSTEM_ERROR bit 11, the one the board sets for an emergency stop. Chosen
# because its high byte is non-zero, so a one-byte read of the field would not
# see it.
SBGC_ERR_EMERGENCY_STOP = 1 << 11


def test_system_error_is_reported_when_the_deprecated_code_is_clear():
    """
    The board deprecated its one-byte ERROR_CODE in favour of SYSTEM_ERROR, so
    a current firmware can sit in emergency stop with the old field reading
    zero. Reporting only the old field showed a console with no faults at all
    during a real one — which is the exact failure this project's rules exist
    to prevent.
    """
    board = Board(system_error=SBGC_ERR_EMERGENCY_STOP)
    d = Daemon(board, ["--no-calib-gyro"])
    try:
        t = d.status()["telemetry"]
        r.check("the deprecated byte really is clear",
                t.get("error_code") == 0, str(t)[:200])
        r.check("SYSTEM_ERROR reaches the status document",
                t.get("system_error") == SBGC_ERR_EMERGENCY_STOP, str(t)[:200])
        r.check("and is named rather than left as a number",
                t.get("system_error_name") == "emergency stop",
                repr(t.get("system_error_name")))
        r.check("the operator is warned", "emergency stop" in d.warnings(),
                d.warnings()[:200])
    finally:
        d.close()
        board.close()


def test_stale_angle_blocks_limited_motion():
    """
    A board that stops reporting leaves the last angle frozen and plausible.
    Travel limits must not be evaluated against it: the number never changes,
    so nothing ever reads as "at the limit" and the camera drives straight
    through. Motion is held instead, and the operator is told why.
    """
    r.section("travel limits with no current angle")
    board = Board(motors_on=True, answer_realtime=False)
    d = Daemon(board, ["--allow-control", "--no-calib-gyro"], wait_for_board=False)
    try:
        r.check("port opened", d.status()["link"].get("port_open"))
        d.post("/api/arm", "armed=1")
        d.post("/api/limits",
               "enabled=1&yaw_min=-90&yaw_max=90&pitch_min=-45&pitch_max=30")

        board.clear("controls")
        d.hold_rate(0.6)
        c = d.control()
        r.check("limit_stale is raised", c.get("limit_stale") is True, str(c)[:200])
        r.check("both axes reported blocked",
                c.get("blocked_yaw") and c.get("blocked_pitch"))
        r.check("no moving frame reached the board", not board.moving_controls(),
                f"{len(board.moving_controls())} moving frames")
        r.check("operator is told why",
                "stopped reporting its angle" in d.warnings())

        # Turning limits off is the documented way to recover a camera by hand.
        d.post("/api/limits", "enabled=0")
        board.clear("controls")
        d.hold_rate(0.5)
        r.check("motion resumes once limits are off",
                len(board.moving_controls()) > 0)
        r.check("limit_stale cleared", d.control().get("limit_stale") is False)
    finally:
        d.close()
        board.close()


def test_disarm_aborts_auto_task():
    """
    home and level are AUTO_TASKs the board owns until they finish. Tracking
    them as "not moving" meant a disarm mid-slew sent nothing at all, so the
    camera carried on to home while the UI read Safe.
    """
    r.section("disarm must abort a running home")
    board = Board(motors_on=True)
    d = Daemon(board, ["--allow-control", "--no-calib-gyro"])
    try:
        d.post("/api/arm", "armed=1")
        board.clear("controls")
        d.post("/api/control/home")
        d.await_status(lambda s: "home" in s["control"].get("last_cmd", ""), 2.0,
                       "home to be sent")
        r.check("a running home reports as moving", d.control().get("moving") is True)

        d.post("/api/arm", "armed=0")
        d.await_status(lambda s: "stop" in s["control"].get("last_cmd", ""), 2.0,
                       "the abort stop")
        r.check("disarm sent a stop mid-task", True)
        r.check("no longer reported as moving", d.control().get("moving") is False)

        zero = [c for c in board.snapshot("controls")
                if all(s == 0 for s in c["speed"])]
        r.check("a zero-rate frame reached the board", len(zero) > 0)
    finally:
        d.close()
        board.close()


def test_calibrates_when_motors_come_on():
    """
    Motors-on is what invalidates the stored gyro bias, so it is the trigger.
    It only ARMS the calibration: switching the motors on is also when the
    gimbal grabs position and swings, and measuring a zero-rate bias then is
    the exact thing that teaches a wrong one.
    """
    r.section("gyro calibration on motors-on")
    board = Board(motors_on=False)
    d = Daemon(board, ["--allow-control"])
    try:
        time.sleep(0.5)
        r.check("nothing calibrated while motors are off",
                len(board.snapshot("calibs")) == 0)

        # Motors on, but the gimbal is still swinging.
        board.set(motors_on=True)
        end = time.monotonic() + 3.0
        i = 0
        while time.monotonic() < end:
            i += 1
            board.set(angles=[0.0, 3.0 if i % 2 else -3.0, 0.0])
            time.sleep(0.05)
        c = d.control()
        r.check("calibration is armed and waiting", c.get("calib_pending") is True,
                str(c)[:200])
        r.check("nothing calibrated during the swing",
                len(board.snapshot("calibs")) == 0,
                f"{len(board.snapshot('calibs'))} calibrations")

        # It settles.
        board.set(angles=[0.0, 12.0, 0.0])
        d.await_status(lambda s: s["control"].get("calib_running", False), 4.0,
                       "calibration to start once settled")
        r.check("calibration fires once settled",
                len(board.snapshot("calibs")) == 1)
        r.check("no longer pending", d.control().get("calib_pending") is False)

        d.await_status(lambda s: not s["control"].get("calib_running", False), 8.0,
                       "calibration to finish")
        r.check("does not repeat while motors stay on",
                len(board.snapshot("calibs")) == 1)

        # A power cycle calibrates again.
        board.set(motors_on=False)
        time.sleep(0.4)
        board.set(motors_on=True)
        d.await_status(lambda s: s["control"].get("calib_running", False), 5.0,
                       "the second calibration")
        r.check("a second power-on calibrates again",
                len(board.snapshot("calibs")) == 2)
    finally:
        d.close()
        board.close()


def test_no_rates_during_calibration():
    """
    The browser republishes a held control at 20 Hz. Suppressing rates for
    only the pass that issued the calibration let the very next pass — tens of
    milliseconds later — start moving the gimbal again for the remaining four
    seconds of a calibration that is supposed to happen on a still gimbal.
    """
    r.section("a held control must not move the gimbal mid-calibration")
    board = Board(motors_on=True, angles=(0.0, 10.0, 0.0))
    d = Daemon(board, ["--allow-control"])
    try:
        # Let the automatic motors-on calibration run to completion first.
        d.await_status(lambda s: s["control"].get("calib_running", False), 6.0,
                       "the motors-on calibration")
        d.await_status(lambda s: not s["control"].get("calib_running", False), 8.0,
                       "it to finish")

        d.post("/api/arm", "armed=1")
        board.clear("controls")
        d.post("/api/control/calibgyro")
        d.hold_rate(2.5)

        r.check("calibration ran throughout",
                d.control().get("calib_running") is True)
        moving = board.moving_controls()
        r.check("no moving frame reached the board during calibration",
                not moving, f"{len(moving)} moving frames, e.g. {moving[:2]}")
    finally:
        d.close()
        board.close()


def test_gives_up_on_a_gimbal_that_never_settles():
    """
    A robot that drives off the moment its motors come on never goes still.
    Calibrating anyway would write the wrong bias; waiting forever would leave
    the operator wondering. It gives up and says so.
    """
    r.section("gimbal that never settles (slow: ~25 s)")
    board = Board(motors_on=False)
    d = Daemon(board, ["--allow-control"])
    try:
        board.set(motors_on=True)
        end = time.monotonic() + 24.0
        i = 0
        while time.monotonic() < end:
            i += 1
            board.set(angles=[0.0, 5.0 if i % 2 else -5.0, 0.0])
            time.sleep(0.05)

        r.check("never calibrated a shaking gimbal",
                len(board.snapshot("calibs")) == 0,
                f"{len(board.snapshot('calibs'))} calibrations")
        c = d.control()
        r.check("gave up rather than waiting forever",
                c.get("calib_pending") is False, str(c)[:200])
        r.check("the skip is reported", c.get("calib_skipped") is True)
        r.check("operator is told it was skipped",
                "never stopped moving" in d.warnings())
    finally:
        d.close()
        board.close()


def test_board_pitch_limits_are_converted():
    """
    The board counts pitch positive-DOWN; the UI reads it positive-UP. The
    negation swaps which end is the minimum, so the bounds trade places. The
    manual's own example — "to go only from a leveled position to down
    position, set min=0, max=90" — must surface as 0 down to 90 up-negative,
    i.e. [-90, 0] in the UI's convention.
    """
    r.section("board travel limits reach the UI in the UI's convention")
    board = Board(motors_on=False,
                  rc_limits={"pitch": (0, 90), "yaw": (-170, 170),
                             "roll": (-45, 45)})
    d = Daemon(board, ["--no-calib-gyro"])
    try:
        s = d.await_status(lambda st: st["limits"].get("source") == "board", 6.0,
                           "the board's limits to be adopted")
        lim = s["limits"]
        r.check("pitch bounds are negated and swapped",
                (lim.get("pitch_min"), lim.get("pitch_max")) == (-90.0, 0.0),
                f"got [{lim.get('pitch_min')}, {lim.get('pitch_max')}], want [-90, 0]")
        r.check("yaw is passed through unchanged",
                (lim.get("yaw_min"), lim.get("yaw_max")) == (-170.0, 170.0),
                f"got [{lim.get('yaw_min')}, {lim.get('yaw_max')}]")
        r.check("roll is passed through unchanged",
                (lim.get("roll_min"), lim.get("roll_max")) == (-45.0, 45.0),
                f"got [{lim.get('roll_min')}, {lim.get('roll_max')}]")
    finally:
        d.close()
        board.close()


def test_cross_origin_requests_are_refused():
    """
    Prefix-matching the Origin host accepted anything merely BEGINNING with a
    trusted name, and localhost.attacker.example is one wildcard DNS record
    away. The whole authority is compared, port included.
    """
    r.section("cross-origin requests")
    board = Board(motors_on=False)
    d = Daemon(board, ["--allow-control", "--no-calib-gyro"])
    try:
        body = "yaw_min=-10&yaw_max=10"
        for origin in ("http://localhost.evil.example",
                       "http://127.0.0.1.evil.example",
                       "http://evil.example",
                       f"http://localhost:{d.port + 1}"):
            code, _ = d.post("/api/limits", body, origin=origin)
            r.check(f"refused {origin}", code == 403, f"HTTP {code}")

        code, _ = d.post("/api/arm", "armed=1",
                         origin="http://localhost.attacker.tld")
        r.check("refused a cross-origin arm", code == 403, f"HTTP {code}")

        for origin in (f"http://127.0.0.1:{d.port}", f"http://localhost:{d.port}"):
            code, _ = d.post("/api/limits", "yaw_min=-90&yaw_max=90",
                             origin=origin)
            r.check(f"allowed our own page at {origin}", code == 200,
                    f"HTTP {code}")

        code, _ = d.post("/api/limits", "yaw_min=-100&yaw_max=100")
        r.check("allowed a request with no Origin (curl, scripts)", code == 200,
                f"HTTP {code}")
    finally:
        d.close()
        board.close()


def test_limits_round_trip_without_changing():
    """Reporting whole degrees made a typed 90.5 come back as 90, so the page
    showed a limit that was not the one being enforced."""
    r.section("limit values round-trip unchanged")
    board = Board(motors_on=False)
    d = Daemon(board, ["--no-calib-gyro"])
    try:
        d.post("/api/limits",
               "enabled=1&yaw_min=-90.5&yaw_max=90.5&pitch_min=-45.5&pitch_max=30.5")
        c = d.control()
        r.check("fractional limits are reported as themselves",
                (c.get("lim_yaw_min"), c.get("lim_yaw_max"),
                 c.get("lim_pitch_min"), c.get("lim_pitch_max")) ==
                (-90.5, 90.5, -45.5, 30.5), str(c)[:200])

        code, _ = d.post("/api/limits", "yaw_min=50&yaw_max=10")
        r.check("an inverted range is refused", code == 400, f"HTTP {code}")
        code, _ = d.post("/api/limits", "yaw_min=nan")
        r.check("a non-finite limit is refused", code == 400, f"HTTP {code}")
        c = d.control()
        r.check("a refused request left the stored values alone",
                (c.get("lim_yaw_min"), c.get("lim_yaw_max")) == (-90.5, 90.5))
    finally:
        d.close()
        board.close()


def yaw_speeds(board):
    """Commanded YAW speed from each CMD_CONTROL frame the board received.

    Not moving_controls(): every rate frame also carries a non-zero ROLL slew,
    because roll is actively held level rather than merely left alone, so a
    frame with yaw fully blocked still counts as "moving" by that measure.
    """
    return [c["speed"][2] for c in board.snapshot("controls")]


def test_yaw_limits_survive_the_display_fold():
    """
    Reported angles are folded into (-180, 180] for display, and a travel
    limit cannot be evaluated across that fold: the number jumps 360 deg while
    the axis moves a fraction of a degree. A gimbal that travelled out to 195
    deg read as -165, comfortably inside a [-170, 170] range, so the limit
    stopped existing in the direction that mattered.

    The limit is judged on a continuous track instead — seeded from the first
    reading and advanced by the shortest step to each next one, so it stays in
    the same frame the operator set the limits in.
    """
    r.section("yaw travel limits survive the display fold")
    board = Board(motors_on=True, angles=(0.0, 0.0, 150.0))
    d = Daemon(board, ["--allow-control", "--no-calib-gyro"])
    try:
        d.post("/api/arm", "armed=1")
        d.post("/api/limits",
               "enabled=1&yaw_min=-170&yaw_max=170&pitch_min=-45&pitch_max=30")

        # Inside the range to begin with: motion must be free.
        board.clear("controls")
        d.hold_rate(0.4, "pan=1.0&tilt=0")
        r.check("motion is allowed while inside the range",
                any(s > 0 for s in yaw_speeds(board)),
                f"yaw speeds {yaw_speeds(board)}")

        # Now walk the axis out past the limit and through the fold. Each step
        # is well under 180 deg, which is what makes the unwrapping unambiguous.
        for deg in (165.0, 175.0, 185.0, 195.0):
            board.set(angles=(0.0, 0.0, deg))
            wrapped = deg - 360.0 if deg > 180.0 else deg
            d.await_status(
                lambda st, w=wrapped:
                    abs(st["telemetry"]["angles"]["yaw"]["imu"] - w) < 1.0,
                3.0, f"the {deg} deg reading to arrive")

        board.clear("controls")
        d.hold_rate(0.6, "pan=1.0&tilt=0")
        ys = yaw_speeds(board)
        r.check("past the limit and through the fold, outward is blocked",
                ys and all(s == 0 for s in ys), f"yaw speeds {ys}")
        r.check("the block is reported to the UI",
                d.control().get("blocked_yaw") is True, str(d.control())[:200])

        # Recovery must stay available, or a gimbal that overshot is stuck.
        board.clear("controls")
        d.hold_rate(0.6, "pan=-1.0&tilt=0")
        r.check("panning back toward the range is still allowed",
                any(s < 0 for s in yaw_speeds(board)),
                f"yaw speeds {yaw_speeds(board)}")
    finally:
        d.close()
        board.close()


def test_limits_follow_the_displayed_angle_not_the_raw_count():
    """
    The board's raw count and the operator's view are different coordinates.
    A gimbal sitting at a raw 350 deg displays as -10, which is inside a
    [-170, 170] range, so both directions must be free. Gating on the raw
    count instead would block panning further positive on a camera the
    operator can see is nowhere near its limit.
    """
    r.section("limits are read in the operator's frame, not the board's")
    board = Board(motors_on=True, angles=(0.0, 0.0, 350.0))
    d = Daemon(board, ["--allow-control", "--no-calib-gyro"])
    try:
        d.post("/api/arm", "armed=1")
        d.post("/api/limits",
               "enabled=1&yaw_min=-170&yaw_max=170&pitch_min=-45&pitch_max=30")
        s = d.status()
        r.check("the board reports it as -10 deg",
                abs(s["telemetry"]["angles"]["yaw"]["imu"] + 10.0) < 1.0,
                str(s["telemetry"]["angles"]["yaw"]))

        board.clear("controls")
        d.hold_rate(0.6, "pan=1.0&tilt=0")
        r.check("panning positive is allowed",
                any(v > 0 for v in yaw_speeds(board)),
                f"yaw speeds {yaw_speeds(board)}")
        r.check("nothing is reported as blocked",
                d.control().get("blocked_yaw") is False, str(d.control())[:200])
    finally:
        d.close()
        board.close()


def test_stalled_connections_do_not_starve_the_control_path():
    """
    The server read each connection to completion before accepting the next,
    with a 2 s budget each, so half-open sockets were additive: four of them
    delayed a legitimate request by nearly eight seconds. That matters because
    the browser republishes held rates at 20 Hz and the daemon stops the
    gimbal when they stop arriving — a handful of idle sockets was enough to
    starve the republisher and drop the camera mid-move.

    Requests are now read out of one poll set, and a newcomer evicts the
    oldest incomplete request rather than being refused, so filling every slot
    does not lock anyone out either.
    """
    r.section("stalled connections must not starve real requests")
    board = Board(motors_on=True)
    d = Daemon(board, ["--allow-control", "--no-calib-gyro"])
    stalled = []
    try:
        def latency():
            t0 = time.monotonic()
            try:
                urllib.request.urlopen(d.base + "/api/live", timeout=20).read()
            except Exception as exc:                       # noqa: BLE001
                return -1.0, repr(exc)
            return time.monotonic() - t0, ""

        base_s, err = latency()
        r.check("a request is fast with nothing else connected",
                0 <= base_s < 1.0, f"{base_s:.3f}s {err}")

        # Well past the slot count, so the eviction path is exercised too.
        for _ in range(40):
            try:
                s = socket.create_connection(("127.0.0.1", d.port), timeout=2)
                s.sendall(b"GET /api/live HTTP/1.1\r\n")   # never completed
                stalled.append(s)
            except OSError:
                break
        r.check("the stalled connections were established", len(stalled) >= 20,
                f"{len(stalled)} opened")
        time.sleep(0.3)

        worst, err = 0.0, ""
        for _ in range(4):
            got, e = latency()
            if got < 0:
                worst, err = -1.0, e
                break
            worst = max(worst, got)
        r.check("requests stay fast while they are held open",
                0 <= worst < 1.0, f"worst {worst:.3f}s {err}")

        # The compiled-in page is the one body large enough to need several
        # writes; a partial send would show up as a short read here.
        with urllib.request.urlopen(d.base + "/", timeout=10) as f:
            page = f.read()
        r.check("the full page is still delivered intact", len(page) > 10000,
                f"{len(page)} bytes")
    finally:
        for s in stalled:
            try:
                s.close()
            except OSError:
                pass
        d.close()
        board.close()


class Decoy:
    """
    A serial device that is not a gimbal — a LiDAR, a GPS, anything else on the
    robot's USB hub. It answers nothing and records every byte written to it.
    """

    def __init__(self):
        self._master, self._slave = pty.openpty()
        self.device = os.ttyname(self._slave)
        self.data = bytearray()
        self._run = True
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self):
        while self._run:
            try:
                rd, _, _ = select.select([self._master], [], [], 0.1)
                if rd:
                    self.data += os.read(self._master, 4096)
            except OSError:
                break

    def commands(self):
        """Command IDs of every well-formed frame it was sent."""
        out, b, i = [], bytes(self.data), 0
        while i + 4 <= len(b):
            if b[i] != 0x3E:
                i += 1
                continue
            cmd, ln = b[i + 1], b[i + 2]
            if (cmd + ln) & 0xFF != b[i + 3] or i + 5 + ln > len(b):
                i += 1
                continue
            out.append(cmd)
            i += 5 + ln
        return out

    def close(self):
        self._run = False
        self._thread.join(timeout=1.0)
        for fd in (self._master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass


def test_a_device_must_identify_itself_before_being_adopted():
    """
    When the configured port vanished, the daemon followed it to whatever
    serial device happened to be enumerated first, with no check that the
    device was a gimbal at all. On a robot with a LiDAR or GPS on the same hub
    that means streaming CMD_CONTROL into the wrong device while telling the
    operator the link recovered.

    A candidate now has to answer CMD_BOARD_INFO before it is adopted.
    """
    r.section("a candidate port must identify itself as a SimpleBGC")
    board = Board(motors_on=True)
    decoy = Decoy()
    try:
        real = subprocess.run([GUI_BIN, "--probe-port", board.device],
                              capture_output=True, text=True, timeout=30)
        r.check("a real board is recognised", real.returncode == 0,
                real.stdout.strip())

        other = subprocess.run([GUI_BIN, "--probe-port", decoy.device],
                               capture_output=True, text=True, timeout=30)
        r.check("a device that answers nothing is rejected",
                other.returncode == 1, other.stdout.strip())

        # What it was sent matters as much as the verdict: identifying a
        # device must never be a way to move one.
        cmds = decoy.commands()
        r.check("the decoy was only ever asked to identify itself",
                cmds and all(c == CMD_BOARD_INFO for c in cmds),
                f"commands seen: {sorted(set(cmds))}")
        r.check("no control frame ever reached it",
                CMD_CONTROL not in cmds, f"commands seen: {sorted(set(cmds))}")
    finally:
        decoy.close()
        board.close()


def test_a_missing_port_is_reported_not_papered_over():
    """
    With nothing else answering, the honest report is that the configured port
    is gone — not a claim to have followed the device somewhere.
    """
    r.section("a missing port is reported rather than substituted")
    board = Board(motors_on=True)
    d = Daemon(board, ["--no-calib-gyro"])
    try:
        gone = "/dev/ttyUSB-does-not-exist"
        code, _ = d.post("/api/port", "path=" + gone)
        r.check("an unknown device is refused outright", code == 404, f"HTTP {code}")

        s = d.status()
        r.check("the daemon stayed on the working port",
                s["link"]["port"] == board.device, s["link"]["port"])
        r.check("it does not claim to have followed anything",
                "following" not in (s["link"].get("note") or ""),
                str(s["link"].get("note")))
    finally:
        d.close()
        board.close()


def test_simulate_needs_no_hardware_and_says_so():
    """
    gimbal_ctl has had --simulate since the start; the daemon had no
    hardware-free mode at all, so there was no way to bring the console up and
    see what it would send. Frames must be built and shown exactly as they
    would go on the wire, nothing may be transmitted, and the page must never
    be mistakable for a live board.
    """
    r.section("the daemon runs with no serial device")
    port = free_port()
    proc = subprocess.Popen(
        [GUI_BIN, "--http-port", str(port), "--no-pad", "--simulate",
         "--allow-control"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    base = f"http://127.0.0.1:{port}"

    def status():
        with urllib.request.urlopen(base + "/api/status", timeout=3) as f:
            return json.load(f)

    def post(path, body=""):
        req = urllib.request.Request(base + path, data=body.encode(),
                                     method="POST")
        req.add_header("Content-Type", "application/x-www-form-urlencoded")
        try:
            with urllib.request.urlopen(req, timeout=3) as f:
                return f.status
        except urllib.error.HTTPError as e:
            return e.code

    try:
        deadline = time.monotonic() + 8.0
        s = None
        while time.monotonic() < deadline:
            try:
                s = status()
                break
            except Exception:                                  # noqa: BLE001
                time.sleep(0.05)
        r.check("it serves the console with no device present", s is not None)
        if s is None:
            return

        r.check("the link reports itself as simulated",
                s["link"].get("simulated") is True, str(s["link"])[:200])
        r.check("the operator is told before anything else",
                s["warnings"] and "SIMULATION" in s["warnings"][0]["text"],
                str(s["warnings"])[:200])

        # Gating is not relaxed just because nothing is connected.
        r.check("arming is still required before a rate is accepted",
                post("/api/control/rate", "pan=1.0&tilt=0") == 403)

        post("/api/arm", "armed=1")
        for _ in range(6):
            post("/api/control/rate", "pan=1.0&tilt=0")
            time.sleep(0.05)
        time.sleep(0.3)
        c = status()["control"]
        hexed = c.get("last_cmd_hex", "")
        r.check("the exact frame it would have sent is shown",
                hexed.startswith("3E 43 0F"), hexed)
        r.check("carrying the commanded yaw rate",
                "F6 00" in hexed, hexed)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


TESTS = [
    (test_simulate_needs_no_hardware_and_says_so, False),
    (test_a_device_must_identify_itself_before_being_adopted, False),
    (test_system_error_is_reported_when_the_deprecated_code_is_clear, False),
    (test_a_missing_port_is_reported_not_papered_over, False),
    (test_stalled_connections_do_not_starve_the_control_path, False),
    (test_yaw_limits_survive_the_display_fold, False),
    (test_limits_follow_the_displayed_angle_not_the_raw_count, False),
    (test_cross_origin_requests_are_refused, False),
    (test_limits_round_trip_without_changing, False),
    (test_board_pitch_limits_are_converted, False),
    (test_stale_angle_blocks_limited_motion, False),
    (test_disarm_aborts_auto_task, False),
    (test_calibrates_when_motors_come_on, False),
    (test_no_rates_during_calibration, False),
    (test_gives_up_on_a_gimbal_that_never_settles, True),
]

if __name__ == "__main__":
    require_binary(GUI_BIN)
    quick = quick_mode()
    for fn, slow in TESTS:
        if slow and quick:
            print(f"\n== {fn.__name__} == SKIPPED (--quick)")
            continue
        r.run(fn)
    sys.exit(r.report())
