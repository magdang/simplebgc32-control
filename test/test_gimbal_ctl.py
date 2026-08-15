#!/usr/bin/env python3
"""
test_gimbal_ctl.py — regression tests for the interactive CLI.

Run against --simulate, where every frame is printed as hex instead of being
written to a port. Assertions decode those frames, so what is checked is the
bytes the tool would have put on the wire rather than what it printed about
them. Each case corresponds to a defect that was found and fixed.
"""

import os
import re
import signal
import subprocess
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from sbgc_sim import CTL_BIN, CMD_CONTROL, Results, require_binary

r = Results()

TX = re.compile(r"\[sim\] tx cmd=(\d+)\s+len=(\d+)\s*:\s*([0-9A-F ]+)")


def run(commands, timeout=15):
    p = subprocess.run([CTL_BIN, "--simulate", "--defaults"],
                       input="\n".join(commands) + "\nquit\n",
                       capture_output=True, text=True, timeout=timeout)
    return p.stdout + p.stderr


def frames(out, cmd=None):
    """[(cmd_id, payload_bytes)] for each simulated transmission."""
    got = []
    for m in TX.finditer(out):
        cid = int(m.group(1))
        raw = [int(b, 16) for b in m.group(3).split()]
        # 3E cmd len hdrsum | payload | bodysum
        payload = raw[4:4 + int(m.group(2))]
        if cmd is None or cid == cmd:
            got.append((cid, payload))
    return got


def speeds(payload):
    """Per-axis commanded speed in wire units, ROLL PITCH YAW."""
    return [int.from_bytes(bytes(payload[3 + a * 4:5 + a * 4]),
                           "little", signed=True) for a in range(3)]


def test_tilt_direction():
    """
    The board's PITCH is positive DOWNWARD, and this tool works positive-up.
    With the default map left un-inverted, "up" tilted the camera down — and
    the asymmetric soft limits were then applied to the wrong direction, which
    is the hard-stop collision the file header warns about.
    """
    r.section("tilt direction on a default configuration")
    up = speeds(frames(run(["up 30"]), CMD_CONTROL)[0][1])
    r.check("'up' commands a negative board pitch", up[1] < 0, f"pitch {up[1]}")
    down = speeds(frames(run(["down 30"]), CMD_CONTROL)[0][1])
    r.check("'down' commands a positive board pitch", down[1] > 0,
            f"pitch {down[1]}")
    r.check("the two are opposite and equal", up[1] == -down[1],
            f"{up[1]} vs {down[1]}")

    # Pan is unaffected: yaw needs no convention change.
    right = speeds(frames(run(["right 45"]), CMD_CONTROL)[0][1])
    r.check("'right' commands a positive yaw", right[2] > 0, f"yaw {right[2]}")


def test_non_finite_input_is_refused():
    """
    std::stod accepts "nan" and "inf". Every range check here is a comparison,
    and comparisons against NaN are all false, so a NaN passed straight through
    to an int16 cast — which is undefined behaviour, and in practice silently
    stopped the direction commands working.
    """
    r.section("NaN and inf are refused at the door")
    out = run(["speed nan"])
    r.check("'speed nan' is rejected", "not a number" in out.lower(),
            out[-200:])
    out = run(["pan inf"])
    r.check("'pan inf' is rejected", "not a number" in out.lower(), out[-200:])
    out = run(["speed -nan", "right 45"])
    fs = frames(out, CMD_CONTROL)
    r.check("a later command still works after a rejected one",
            fs and speeds(fs[0][1])[2] != 0,
            f"{speeds(fs[0][1]) if fs else 'no frames'}")


def test_rate_ceiling():
    r.section("the rate ceiling still holds")
    r.check("'speed 900' is refused",
            "implausibly high" in run(["speed 900"]))
    r.check("'right 900' is refused",
            "implausibly high" in run(["right 900"]))
    r.check("'speed 500' is accepted", "speed = 500" in run(["speed 500"]))


def test_sigint_stops_the_gimbal():
    """
    A direction command puts the board in SPEED mode and it runs until
    recalled; the board has no serial-loss failsafe. Under the default SIGINT
    disposition the process died before the stop at the bottom of main(), and
    the camera kept turning with nothing left alive to recall it.
    """
    r.section("Ctrl-C at the prompt must still stop the gimbal")
    p = subprocess.Popen([CTL_BIN, "--simulate", "--defaults"],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True)
    try:
        p.stdin.write("right 45\n")
        p.stdin.flush()
        time.sleep(0.6)
        os.kill(p.pid, signal.SIGINT)
        out, _ = p.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        p.kill()
        out, _ = p.communicate()
        r.check("process exits on SIGINT", False, "it did not")
        return

    r.check("process exits on SIGINT", p.returncode == 0, f"rc={p.returncode}")
    r.check("the interrupt is reported", "interrupted" in out)

    fs = frames(out, CMD_CONTROL)
    r.check("a rate went out first", len(fs) >= 2 and speeds(fs[0][1])[2] != 0,
            f"{[speeds(f[1]) for f in fs]}")
    r.check("the last frame before exit is a zero rate",
            fs and all(s == 0 for s in speeds(fs[-1][1])),
            f"{speeds(fs[-1][1]) if fs else 'no frames'}")


def test_quit_stops_the_gimbal():
    r.section("a normal quit stops the gimbal too")
    fs = frames(run(["right 45"]), CMD_CONTROL)
    r.check("the last frame is a zero rate",
            fs and all(s == 0 for s in speeds(fs[-1][1])),
            f"{speeds(fs[-1][1]) if fs else 'no frames'}")


def run_wizard(answers, commands, timeout=15):
    """
    Drive the setup wizard rather than skipping it with --defaults.

    Answer order per axis is: axis choice, invert y/n, soft limit min, soft
    limit max — for PAN, TILT then ROLL — followed by default speed, step and
    the frame-reference question.

    Returns (output, returncode). The return code matters here: the defect
    this drives aborted the process, and an abort is far better evidence than
    inspecting what it managed to print first.
    """
    p = subprocess.run([CTL_BIN, "--simulate"],
                       input="\n".join(answers + commands) + "\nquit\n",
                       capture_output=True, text=True, timeout=timeout)
    return p.stdout + p.stderr, p.returncode


AXES_OK = ["1", "n", "-170", "170",
           "2", "y", "-90", "40",
           "3", "n", "-45", "45"]


def test_wizard_validates_motion_defaults():
    """
    The wizard wrote speed and step through an unvalidated prompt while the
    runtime 'speed' and 'step' commands enforced ranges, so it was a way to
    store a value those commands would have refused.

    A default above the rate ceiling then reached a direction command issued
    with no argument, and the error path that reported it formatted tok[1] —
    one past the end of a one-element vector. Under -D_GLIBCXX_ASSERTIONS that
    aborts; without it, it reads whatever follows.
    """
    r.section("the setup wizard validates its motion defaults")

    out, rc = run_wizard(AXES_OK + ["99999", "30", "5", "y"], ["left"])
    r.check("the process exits cleanly rather than aborting", rc == 0,
            f"rc={rc}")
    r.check("a speed above the ceiling is refused",
            "at most 500 deg/s" in out, out[-400:])
    r.check("the wizard re-asks rather than storing it",
            out.count("default speed") >= 2)

    # It survived the bad answer and the no-argument command that used to crash.
    fs = frames(out, CMD_CONTROL)
    r.check("'left' with no argument still sends a rate",
            any(speeds(f[1])[2] != 0 for f in fs),
            f"{[speeds(f[1]) for f in fs]}")
    r.check("that rate is the accepted default, not the refused one",
            any(speeds(f[1])[2] == -246 for f in fs),
            f"{[speeds(f[1]) for f in fs]}")
    r.check("no crash or assertion escaped to the output",
            "Assertion" not in out and "Segmentation" not in out, out[-400:])

    out, _ = run_wizard(AXES_OK + ["30", "9999", "5", "y"], [])
    r.check("a step above the ceiling is refused",
            "at most 90 deg" in out, out[-400:])

    out, _ = run_wizard(["1", "n", "-9999", "170",
                         "-170", "170"] + AXES_OK[4:] + ["30", "5", "y"], [])
    r.check("a soft limit beyond a full turn is refused",
            "between -360 and 360" in out, out[-400:])


TESTS = [
    test_tilt_direction,
    test_non_finite_input_is_refused,
    test_rate_ceiling,
    test_wizard_validates_motion_defaults,
    test_quit_stops_the_gimbal,
    test_sigint_stops_the_gimbal,
]

if __name__ == "__main__":
    require_binary(CTL_BIN)
    for fn in TESTS:
        r.run(fn)
    sys.exit(r.report())
