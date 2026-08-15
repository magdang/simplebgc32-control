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

import socket
import sys
import time
import urllib.request

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from sbgc_sim import (Board, Daemon, Results, GUI_BIN, CMD_CONTROL,
                      quick_mode, require_binary)

r = Results()


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


def test_yaw_limits_use_the_unwrapped_angle():
    """
    imu_deg is folded into (-180, 180] so the UI never shows an attitude no
    mount could reach. The travel-limit gate used that folded value, so a
    gimbal at a true +200 deg read as -160 — comfortably back inside a
    [-170, 170] range. The limit stopped existing in the direction that
    mattered, and the axis could run on for most of a turn before the wrapped
    reading came round to block it again.
    """
    r.section("yaw travel limits are gated on the unwrapped angle")
    # 200 deg is 30 past the maximum, and wraps to -160: inside the range.
    board = Board(motors_on=True, angles=(0.0, 0.0, 200.0))
    d = Daemon(board, ["--allow-control", "--no-calib-gyro"])
    try:
        d.post("/api/arm", "armed=1")
        d.post("/api/limits",
               "enabled=1&yaw_min=-170&yaw_max=170&pitch_min=-45&pitch_max=30")

        board.clear("controls")
        d.hold_rate(0.6, "pan=1.0&tilt=0")
        ys = yaw_speeds(board)
        r.check("panning further past the limit is blocked",
                ys and all(s == 0 for s in ys), f"yaw speeds {ys}")
        r.check("the block is reported to the UI",
                d.control().get("blocked_yaw") is True, str(d.control())[:200])

        # Recovery must stay available, or a gimbal that overshot is stuck.
        board.clear("controls")
        d.hold_rate(0.6, "pan=-1.0&tilt=0")
        ys = yaw_speeds(board)
        r.check("panning back toward the range is still allowed",
                any(s < 0 for s in ys), f"yaw speeds {ys}")

        # The mirror case, past the minimum rather than the maximum. Wait for
        # the daemon to actually observe the new angle first: telemetry is a
        # round-trip, and frames sent against the previous reading are not
        # evidence about the gate.
        board.set(angles=(0.0, 0.0, -200.0))
        d.await_status(
            lambda st: abs(st["telemetry"]["angles"]["yaw"]["imu"] - 160.0) < 1.0,
            3.0, "the -200 deg yaw reading to arrive (wraps to +160)")
        board.clear("controls")
        d.hold_rate(0.6, "pan=-1.0&tilt=0")
        ys = yaw_speeds(board)
        r.check("the same holds below the minimum",
                ys and all(s == 0 for s in ys), f"yaw speeds {ys}")
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


TESTS = [
    (test_stalled_connections_do_not_starve_the_control_path, False),
    (test_yaw_limits_use_the_unwrapped_angle, False),
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
