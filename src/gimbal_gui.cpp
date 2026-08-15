/*
 * gimbal_gui.cpp — browser-based status console for the SimpleBGC gimbal.
 *
 * Runs a small HTTP server and serves a single self-contained page that shows
 * what the gimbal is doing: link state, controller presence, motor state,
 * live angles, battery, the active profile's configuration, and any warnings.
 *
 * ------------------------------------------------------------- read-only --
 * By default this program CANNOT move the gimbal. Every serial command it
 * sends goes through send_query(), which refuses any command ID that is not on
 * a whitelist of pure queries. Operator-commanded motion requires two
 * independent unlocks:
 *
 *   1. the binary must be started with --allow-control, and
 *   2. the operator must arm the control toggle in the UI.
 *
 * With either missing, the control endpoints answer 403 and no frame is built.
 * The default posture is monitor-only, which is what you want while a camera
 * is bolted to the thing.
 *
 * ONE thing is written without the second unlock: the gyro calibration that
 * runs when the motors are switched on. It is gated on --allow-control like
 * everything else, and it is listed here rather than buried because a reader
 * checking whether this program can write to the board deserves the whole
 * answer. Arming exists to gate motion the OPERATOR commands, and this
 * commands none: it waits for the gimbal to be still, and the serial loop
 * suppresses every rate for its duration. See "calibrate on power" below.
 *
 * ------------------------------------------------------------- zero-args --
 * Run it with no arguments. Port, baud and axis limits are recovered from an
 * existing SimpleBGC GUI install (its bgc.properties and exported .profile);
 * built-in defaults apply when none is found. --port and friends override.
 */

#include "sbgc_api.h"
#include "sbgc_params.h"
#include "sbgc_gui_config.h"
#include "gamepad.h"
#include "httpd.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <csignal>
#include <sys/stat.h>
#include <ctime>
#include <unistd.h>

/* The page is compiled in, so the binary is self-contained. */
#include "web_index.h"

namespace {

// -------------------------------------------------------------- read-only --

/*
 * Commands this program is permitted to transmit while in monitor mode. Only
 * queries: nothing here changes board state or moves a motor.
 */
bool is_query_command(uint8_t cmd)
{
    switch (cmd) {
        case SBGC_CMD_BOARD_INFO:
        case SBGC_CMD_GET_ANGLES:
        case SBGC_CMD_READ_PARAMS_3:
        case SBGC_CMD_REALTIME_DATA_3:
        case SBGC_CMD_BOARD_INFO_3:
            return true;
        default:
            return false;
    }
}

// ------------------------------------------------------------------ state --

double monotonic_s()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return double(ts.tv_sec) + double(ts.tv_nsec) * 1e-9;
}

struct AxisLimits {
    double min_deg = -180.0;
    double max_deg =  180.0;
};

struct Options {
    std::string port = "/dev/ttyUSB0";
    int         baud = 115200;
    std::string bind_addr = "127.0.0.1";
    int         http_port = 8080;
    std::string pad_path;              // empty = auto-detect
    std::string gui_dir;               // explicit SimpleBGC GUI install
    bool        allow_control = false; // arms the control unlock
    bool        no_pad = false;
    /*
     * Calibrate the gyro every time the motors come on. On by default: a
     * gimbal that has not zeroed its gyro drifts, powering the motors is what
     * invalidates the stored bias, and the operator asked for one less thing
     * to remember. Opt out with --no-calib-gyro. Still requires
     * --allow-control, since it writes.
     */
    bool        calib_gyro_on_motors = true;
};

/*
 * Everything the UI renders. Guarded by `mu`; the serial thread writes it and
 * the HTTP thread reads it.
 */
struct Status {
    std::mutex mu;

    // link
    bool        link_open = false;      // serial port opened
    bool        board_responding = false;
    std::string link_error;
    double      last_frame_age_s = 999.0;
    int         frames_rx = 0;
    int         timeouts = 0;

    // board identity
    bool              have_info = false;
    sbgc_board_info_t info{};

    /*
     * The measured pitch and yaw unwrapped onto a continuous track, in the
     * UI's own convention (pitch positive up). This, not the wrapped display
     * angle, is what travel limits are judged against — see unwrap_onto.
     */
    bool   angle_cont_valid = false;
    double yaw_cont = 0.0, pitch_cont = 0.0;

    // live telemetry
    bool            have_rt = false;
    sbgc_realtime_t rt{};
    /*
     * When `rt` last arrived. have_rt only ever latches true, so on its own it
     * says "an angle was read at some point", not "this angle is current". A
     * board whose TX path dies while it still accepts commands keeps the last
     * reading frozen and perfectly plausible, which is exactly the case the
     * travel limits must not be evaluated against.
     */
    double          rt_stamp = 0.0;

    // configuration of the active profile
    bool          have_params = false;
    sbgc_params_t params{};

    // controller
    bool        pad_present = false;
    std::string pad_name;
    std::string pad_path;
    bool        pad_deadman = false;

    // Limits actually in force. The live board always wins over a saved file:
    // an exported .profile is a snapshot that goes stale the moment someone
    // retunes, and trusting it over the hardware is how you end up enforcing
    // limits the gimbal does not actually have.
    AxisLimits roll, pitch, yaw;
    enum LimitSource { LIMITS_BUILTIN, LIMITS_FILE, LIMITS_BOARD };
    LimitSource limits_source = LIMITS_BUILTIN;

    // What the saved GUI profile claimed, kept so a stale export can be
    // reported rather than silently ignored.
    bool       have_file_limits = false;
    AxisLimits file_roll, file_pitch, file_yaw;

    /*
     * Operator-set travel limits, enforced by this tool on top of whatever the
     * board does. Unlike the CLI's open-loop estimate these are checked
     * against the board's own reported angle, so they cannot drift out of
     * agreement with reality over a long session.
     */
    bool       user_limits_on = false;
    AxisLimits user_pitch{ -50.0, 45.0 };
    AxisLimits user_yaw{ -170.0, 170.0 };
    bool       limit_blocked_pitch = false, limit_blocked_yaw = false;
    /*
     * Limits are on but there is no current angle to check them against, so
     * motion is being held rather than allowed through unchecked. Reported
     * separately from the per-axis blocks: "at the limit" and "cannot tell
     * where the camera is" are different situations with different fixes.
     */
    bool       limit_stale = false;

    // control gating
    bool control_allowed = false;   // --allow-control was passed
    bool control_armed   = false;   // operator flipped the UI toggle

    /*
     * What the operator is currently asking for, as a normalised -1..1 rate
     * per axis. The UI republishes this while a control is held; it is NOT
     * latched. `intent_stamp` is when it last arrived, and a rate older than
     * CONTROL_TIMEOUT_S is treated as "the operator let go or the link died"
     * and stopped. Holding a rate because the last packet said so is how a
     * dropped connection turns into a camera that sweeps into its hard stop.
     */
    double intent_pan = 0.0, intent_tilt = 0.0, intent_roll = 0.0;
    double intent_stamp = 0.0;
    bool   motion_active = false;   // a non-zero rate is currently commanded
    /*
     * A home or level AUTO_TASK we started and the board is still running.
     * Tracked apart from motion_active because the two are ended by different
     * commands, and because clearing motion_active for a task — which is what
     * keeps the watchdog from cancelling it — used to mean a disarm mid-slew
     * saw nothing in flight and sent nothing.
     */
    bool   task_active = false;
    double task_started = 0.0;

    double speed_deg_s = 30.0;      // full-deflection rate

    /*
     * A home position taught by pointing the camera and pressing "Set home
     * here", rather than the board's built-in frame-neutral.
     *
     * This is held here rather than written to the board: teaching a home
     * offset on the board means a WRITE_PARAMS, which rewrites the operator's
     * tuning profile. Keeping it in the tool means the worst case is a
     * forgotten preference, not a corrupted profile.
     */
    bool    have_custom_home = false;
    /* Raw board units, never wrapped display degrees — see sbgc_params.h. */
    int16_t home_units[SBGC_NUM_AXES] = { 0, 0, 0 };
    double  home_pitch_deg = 0.0, home_yaw_deg = 0.0;   /* for display only */

    // One-shot actions, consumed by the serial thread.
    bool pending_home = false, pending_level = false, pending_stop = false;
    bool pending_calib_gyro = false;
    int  pending_motors = -1;       // -1 none, 0 off, 1 on

    std::string config_summary;
    std::string port;               // what the serial thread is using now
    int         baud = 115200;

    /*
     * A port change requested from the UI. Re-plugging a CH340 moves it from
     * ttyUSB0 to ttyUSB1, so being able to repoint the tool without
     * restarting it is the difference between a two-second fix and a
     * restart. Changing which device this program opens does not touch the
     * gimbal, so it is deliberately allowed without arming control.
     */
    std::string requested_port;
    bool        port_change_pending = false;
    std::string port_note;          // e.g. "device moved, following by-id"

    /*
     * What was last actually put on the wire, and when. "The button did
     * nothing" is otherwise impossible to tell apart from "the command went
     * out and the gimbal ignored it" — which are fixed in completely
     * different places. The CLI's keyboard mode shows the same thing.
     */
    std::string last_cmd;           // human name, e.g. "level"
    std::string last_cmd_hex;       // exact bytes sent
    double      last_cmd_at = 0.0;  // monotonic seconds

    // Gyro calibration progress, so the operator knows when to stop holding
    // the gimbal still.
    bool   calib_running = false;
    double calib_started = 0.0;
    /*
     * Motors are on and a calibration is armed, waiting for the gimbal to stop
     * settling. Surfaced because "it is about to hold the camera still for
     * four seconds" is something the operator should see coming rather than
     * discover when the controls go quiet.
     */
    bool   calib_pending = false;
    /* The armed calibration gave up: the gimbal never went still. */
    bool   calib_skipped = false;
};

Status g_status;
std::atomic<bool> g_running{true};

/*
 * PITCH SIGN.
 *
 * The board's pitch is positive DOWNWARD. The 2.6x manual, RC Settings:
 * "if you want to configure a camera to go only from a leveled position to
 * down position, set min=0, max=90". The vendor's own CMD_CONTROL example
 * agrees — "Rotate PITCH 90 degrees up" encodes angle 0xF000, i.e. -90
 * degrees (see test/test_sbgc_api.c).
 *
 * An operator pressing "up" means up, so the UI works in the opposite
 * convention: +1 is up. The two meet here and nowhere else. Every UI-facing
 * pitch number — commands, telemetry, travel limits — passes through this.
 */
/*
 * Advance a continuous angle track with a newly arrived wrapped reading.
 *
 * The board's reported angles are folded into (-180, 180] for display, and a
 * travel limit cannot be evaluated across that fold: the number jumps 360 deg
 * while the axis moves a fraction of a degree. Seeding a track from the first
 * reading and advancing it by the shortest step to each subsequent one gives
 * an angle that is continuous through the fold AND still expressed in the
 * frame the operator set the limits in.
 *
 * Using the board's raw unwrapped count instead would be continuous but would
 * not agree with the display: a gimbal sitting at a raw 350 deg shows -10 deg,
 * and a [-170, 170] limit would then block a direction the operator can see is
 * free.
 */
double unwrap_onto(double cont, double wrapped)
{
    double d = wrapped - std::fmod(cont, 360.0);
    while (d > 180.0)   d -= 360.0;
    while (d <= -180.0) d += 360.0;
    return cont + d;
}

/* Shortest angular distance between two wrapped readings. */
double angle_gap(double a, double b)
{
    return std::fabs(std::fmod(a - b + 540.0, 360.0) - 180.0);
}

double ui_pitch_from_board(double board_deg) { return -board_deg; }
double board_pitch_from_ui(double ui_deg)    { return -ui_deg; }

/*
 * A board RC angle limit expressed the way the UI reads it.
 *
 * Pitch is the only axis that moves, and it does not merely change sign: the
 * negation swaps which end is the minimum, so the two bounds trade places as
 * well. Publishing the board's pair unconverted under the same "pitch" key the
 * page compares a positive-up angle against inverted the out-of-range
 * indicator exactly — a camera parked mid-travel read as out of range, and one
 * genuinely past the limit read as inside it.
 */
AxisLimits ui_limits_from_board(int axis, double lo, double hi)
{
    if (axis == SBGC_PITCH)
        return { ui_pitch_from_board(hi), ui_pitch_from_board(lo) };
    return { lo, hi };
}

// ------------------------------------------------------------ serial loop --

struct RxState {
    Status *st;
    bool    got_realtime;
    bool    got_params;
    bool    got_info;
};

void on_frame(uint8_t cmd, const uint8_t *payload, size_t len, void *user)
{
    RxState *rs = static_cast<RxState *>(user);
    Status *st = rs->st;

    std::lock_guard<std::mutex> lk(st->mu);
    st->frames_rx++;

    switch (cmd) {
        case SBGC_CMD_REALTIME_DATA_3: {
            sbgc_realtime_t rt;
            if (sbgc_parse_realtime_3(payload, len, &rt) == 0) {
                double yaw_w   = rt.imu_deg[SBGC_YAW];
                double pitch_w = ui_pitch_from_board(rt.imu_deg[SBGC_PITCH]);
                if (!st->angle_cont_valid) {
                    st->yaw_cont   = yaw_w;
                    st->pitch_cont = pitch_w;
                    st->angle_cont_valid = true;
                } else {
                    st->yaw_cont   = unwrap_onto(st->yaw_cont,   yaw_w);
                    st->pitch_cont = unwrap_onto(st->pitch_cont, pitch_w);
                }
                st->rt = rt;
                st->have_rt = true;
                st->rt_stamp = monotonic_s();
                rs->got_realtime = true;
            }
            break;
        }
        case SBGC_CMD_READ_PARAMS_3: {
            sbgc_params_t p;
            if (sbgc_parse_params_3(payload, len, &p) == 0) {
                st->params = p;
                st->have_params = true;
                rs->got_params = true;

                // Adopt the board's own travel limits, converted into the
                // UI's convention on the way in. An axis reporting 0..0 has
                // no RC angle limit configured at all, so the previous (file
                // or built-in) value is kept for it rather than collapsing
                // the display to a zero-width range.
                auto adopt = [](AxisLimits &dst, int axis, int16_t lo, int16_t hi) {
                    if (lo == 0 && hi == 0) return false;
                    dst = ui_limits_from_board(axis, lo, hi);
                    return true;
                };
                bool any = false;
                any |= adopt(st->roll,  SBGC_ROLL,
                                        p.rc[SBGC_ROLL].rc_min_angle,
                                        p.rc[SBGC_ROLL].rc_max_angle);
                any |= adopt(st->pitch, SBGC_PITCH,
                                        p.rc[SBGC_PITCH].rc_min_angle,
                                        p.rc[SBGC_PITCH].rc_max_angle);
                any |= adopt(st->yaw,   SBGC_YAW,
                                        p.rc[SBGC_YAW].rc_min_angle,
                                        p.rc[SBGC_YAW].rc_max_angle);
                if (any) st->limits_source = Status::LIMITS_BOARD;
            }
            break;
        }
        case SBGC_CMD_BOARD_INFO: {
            sbgc_board_info_t bi;
            if (sbgc_parse_board_info(payload, len, &bi) == 0) {
                st->info = bi;
                st->have_info = true;
                rs->got_info = true;
            }
            break;
        }
        default:
            break;   // CMD_ERROR and anything else: counted, not acted on
    }
}

/* A rate command that stops arriving must stop the gimbal, not persist. */
const double CONTROL_TIMEOUT_S = 0.5;

/*
 * How long gyro calibration takes. The 2.6x manual, "Calibrating Gyroscope":
 * "The Gyro sensor is calibrated every time you turn the controller on, and it
 * takes about 4 seconds to complete." Half a second of margin is added so the
 * UI does not declare success early.
 */
const double CALIB_SECONDS = 4.5;

/*
 * How old the board's reported angle may be and still be treated as where the
 * camera actually is. Realtime data is requested every 40 ms, so anything
 * approaching this means the board has stopped answering. Matched to
 * CONTROL_TIMEOUT_S deliberately: if we have not heard the angle for as long
 * as it takes an operator's own command to expire, we do not know where the
 * camera is pointing and must not pretend otherwise.
 */
const double ANGLE_FRESH_S = 0.5;

/*
 * How long a home or level AUTO_TASK is assumed to still be running. The board
 * sends no completion frame this tool has verified, so — as with calibration —
 * the end is inferred from a duration rather than invented from a reply. It is
 * generous: over-estimating means a disarm sends one redundant stop, while
 * under-estimating means a disarm mid-slew sends none at all.
 */
const double TASK_SECONDS = 6.0;

/*
 * Deciding when a gimbal that has just been powered has finished settling.
 *
 * CALIB_STILL_DEG is the most any axis may move between two consecutive
 * telemetry samples (40 ms apart) and still count as stationary. It is a
 * couple of times the reading noise, not zero: an actively stabilised gimbal
 * is never perfectly numerically still, and demanding that it be would mean
 * the calibration never runs.
 *
 * CALIB_STILL_S is how long that has to hold. A settling gimbal crosses
 * through zero movement on its way past, so a single quiet sample proves
 * nothing; it has to stay quiet.
 */
const double CALIB_STILL_DEG = 0.20;
const double CALIB_STILL_S   = 1.00;

/*
 * How long to wait for a gimbal that has just been powered to go still before
 * giving up on calibrating it. Generous, because a slow settle is normal and
 * the cost of waiting is nothing; finite, because a robot that drives off
 * immediately after power-on would otherwise leave the request armed forever.
 */
const double CALIB_SETTLE_TIMEOUT_S = 20.0;

/*
 * Roll is not an operator-controlled axis on this robot.
 *
 * It is held level with the horizon and never commanded to turn. Enforcing
 * that by simply sending a zero roll RATE would not be enough: a zero rate in
 * speed mode means "hold whatever angle you are at", so any roll the gimbal
 * had already acquired — from a knock, or a drift, or a previous session —
 * would be held forever instead of corrected.
 *
 * So every motion command puts ROLL in MODE_ANGLE at 0 degrees, which is the
 * gravity-referenced horizon, while pitch and yaw take the operator's rate.
 * That actively returns roll to level and keeps it there.
 */
int send_rate_roll_locked(sbgc_t *sb, double pan_deg_s, double tilt_deg_s,
                          double roll_slew_deg_s)
{
    uint8_t mode[SBGC_NUM_AXES];
    int16_t speed[SBGC_NUM_AXES], angle[SBGC_NUM_AXES];

    mode[SBGC_ROLL]  = SBGC_MODE_ANGLE;
    speed[SBGC_ROLL] = sbgc_degs_to_units(roll_slew_deg_s);
    angle[SBGC_ROLL] = 0;                       /* level with the horizon */

    mode[SBGC_PITCH]  = SBGC_MODE_SPEED;
    speed[SBGC_PITCH] = sbgc_degs_to_units(board_pitch_from_ui(tilt_deg_s));
    angle[SBGC_PITCH] = 0;

    mode[SBGC_YAW]  = SBGC_MODE_SPEED;
    speed[SBGC_YAW] = sbgc_degs_to_units(pan_deg_s);
    angle[SBGC_YAW] = 0;

    return sbgc_control_raw(sb, mode, speed, angle);
}

/* The single point through which query commands leave this program. */
bool send_query(sbgc_t &sb, uint8_t cmd, const uint8_t *payload, size_t len)
{
    if (!is_query_command(cmd)) {
        // A programming error, not a runtime condition. Fail loudly rather
        // than let an unexpected command reach hardware.
        std::fprintf(stderr,
                     "gimbal_gui: refusing to send non-query command %u\n", cmd);
        return false;
    }
    return sbgc_send(&sb, cmd, payload, len) == 0;
}

/*
 * The only path by which a command that MOVES something is transmitted.
 *
 * The arm state is re-checked here, at the moment of transmission, even
 * though the HTTP layer already checked it. That is deliberate: the HTTP
 * check protects the API, this one protects the hardware. If a future change
 * ever routes a motion request around the endpoint guard, it still cannot
 * reach the motors without the operator having armed control.
 */
bool motion_permitted()
{
    std::lock_guard<std::mutex> lk(g_status.mu);
    return g_status.control_allowed && g_status.control_armed;
}

/*
 * Wrap any call that moves the gimbal. Stopping is deliberately NOT routed
 * through here: a stop must always be allowed, including when control has
 * just been disarmed or the watchdog has fired. The guard exists to prevent
 * unwanted motion, and refusing to stop would invert its purpose.
 */
/* Record the frame that just went out, for the UI's "last command" readout. */
void note_tx(sbgc_t &sb, const char *what)
{
    char hex[3 * SBGC_MAX_FRAME + 1];
    sbgc_format_last_tx(&sb, hex, sizeof(hex));
    std::lock_guard<std::mutex> lk(g_status.mu);
    g_status.last_cmd     = what;
    g_status.last_cmd_hex = hex;
    g_status.last_cmd_at  = monotonic_s();
}

/*
 * Record a frame that did NOT reach the port. That readout exists to turn
 * "the button did nothing" into a question with an answer — reporting a
 * failed write as a successful one is the single thing it must never do,
 * because it points the operator away from the actual fault.
 */
void note_tx_failed(sbgc_t &sb, const char *what)
{
    std::string err = sbgc_last_error(&sb);
    std::lock_guard<std::mutex> lk(g_status.mu);
    g_status.last_cmd     = std::string(what) + " — NOT SENT";
    g_status.last_cmd_hex = err;
    g_status.last_cmd_at  = monotonic_s();
}

/*
 * `fn` must return the underlying sbgc_* status: 0 sent, non-zero failed.
 * A write that never made it out is reported as the failure it is. Treating
 * it as success would set motion_active for a frame the board never saw, and
 * would let the home/level branch suppress the stop that should follow.
 */
template <typename Fn>
bool send_motion(sbgc_t &sb, const char *what, Fn &&fn)
{
    if (!motion_permitted()) {
        std::fprintf(stderr,
                     "gimbal_gui: blocked a motion command; control is not armed\n");
        return false;
    }
    if (fn() != 0) {
        note_tx_failed(sb, what);
        return false;
    }
    note_tx(sb, what);
    return true;
}

/*
 * How long to give one candidate device to identify itself, and how many
 * times to ask. Opening a CH340 toggles DTR and resets the board, so the
 * first query lands in that window and is simply lost — the read-only probe
 * tool retries for the same reason.
 */
const double PROBE_REPLY_S  = 0.35;
const int    PROBE_ATTEMPTS = 4;

/*
 * Does this device actually answer as a SimpleBGC?
 *
 * Opens it, asks for CMD_BOARD_INFO — a pure query, already on the whitelist
 * send_query enforces — and waits briefly for a reply that parses. Nothing
 * else is ever transmitted, so a device that turns out to be something else
 * sees a handful of bytes it will not recognise and nothing more.
 *
 * This exists because "the configured port vanished" and "this other device
 * is the gimbal" are completely different claims, and the port list cannot
 * tell them apart. Adopting on the strength of the second without checking is
 * how a daemon ends up streaming CMD_CONTROL into a LiDAR.
 */
bool probe_is_sbgc(const char *dev, int baud)
{
    sbgc_t p;
    std::memset(&p, 0, sizeof(p));
    if (sbgc_open(&p, dev, baud) != 0) return false;
    sbgc_set_quiet(&p, 1);

    bool ok = false;
    for (int attempt = 0; attempt < PROBE_ATTEMPTS && !ok; attempt++) {
        const uint8_t zero = 0;
        if (!send_query(p, SBGC_CMD_BOARD_INFO, &zero, 1)) break;

        const double deadline = monotonic_s() + PROBE_REPLY_S;
        while (!ok && monotonic_s() < deadline) {
            if (sbgc_poll(&p, 50,
                    [](uint8_t cmd, const uint8_t *pl, size_t len, void *u) {
                        sbgc_board_info_t bi;
                        if (cmd == SBGC_CMD_BOARD_INFO &&
                            sbgc_parse_board_info(pl, len, &bi) == 0)
                            *static_cast<bool *>(u) = true;
                    }, &ok) < 0)
                break;
        }
    }

    sbgc_close(&p);
    return ok;
}

void serial_thread(Options opt)
{
    sbgc_t sb;
    std::memset(&sb, 0, sizeof(sb));
    bool open_ok = false;

    RxState rs{ &g_status, false, false, false };

    // Calibrate-on-motors-on tracking. All local to this thread: it is the
    // only writer, and none of it is meaningful to anyone else.
    bool   motors_were_on   = false;
    bool   calib_armed      = false;   // motors came on, waiting for stillness
    double calib_armed_at   = 0.0;
    double still_since      = 0.0;     // when the angles last stopped changing
    double last_still_stamp = 0.0;     // rt_stamp of the last sample judged
    bool   have_prev_angle  = false;
    double prev_angle[SBGC_NUM_AXES] = { 0, 0, 0 };

    double last_reopen  = 0.0;
    double last_rt      = 0.0;
    double last_info    = 0.0;
    double last_params  = 0.0;
    double last_rx_time = monotonic_s();

    while (g_running.load()) {
        double now = monotonic_s();

        // --- an operator-requested port change closes the current one ---
        // Decide under the lock, act outside it: close() on a tty can block
        // draining pending output, and holding the mutex through that stalls
        // every HTTP request behind it.
        bool do_port_change = false;
        bool was_moving_before_change = false;
        {
            std::lock_guard<std::mutex> lk(g_status.mu);
            if (g_status.port_change_pending) {
                g_status.port_change_pending = false;
                do_port_change = true;
                opt.port = g_status.requested_port;
                g_status.port = opt.port;
                g_status.port_note.clear();

                // Everything below describes the board we are leaving. Keeping
                // any of it would show the new port with the old board's
                // angles, battery and limits until the new one answers — and
                // cur_profile would be read from the old board to decide what
                // to ask the new one for.
                g_status.have_params = false;
                g_status.have_info   = false;
                g_status.have_rt     = false;
                g_status.angle_cont_valid = false;
                g_status.board_responding = false;
                was_moving_before_change = g_status.motion_active;
                g_status.motion_active = false;
                if (g_status.have_file_limits) {
                    g_status.roll  = g_status.file_roll;
                    g_status.pitch = g_status.file_pitch;
                    g_status.yaw   = g_status.file_yaw;
                    g_status.limits_source = Status::LIMITS_FILE;
                } else {
                    g_status.roll  = { -45.0, 45.0 };
                    g_status.pitch = { -90.0, 40.0 };
                    g_status.yaw   = { -170.0, 170.0 };
                    g_status.limits_source = Status::LIMITS_BUILTIN;
                }
            }
        }
        if (do_port_change) {
            /*
             * Stop on the OLD port while it is still open. Closing first would
             * strand that board in SPEED mode at the last rate with no link
             * left to recall it.
             */
            if (open_ok) {
                if (was_moving_before_change) sbgc_stop(&sb);
                sbgc_close(&sb);
                open_ok = false;
            }
            last_reopen = 0.0;              // reconnect immediately
        }

        // --- (re)open the port, backing off between attempts ---
        if (!open_ok) {
            if (now - last_reopen < 1.5) {
                usleep(100000);
                continue;
            }
            last_reopen = now;

            /*
             * If the configured device has vanished — the usual cause is the
             * adapter being unplugged and re-enumerated under a new number —
             * look for where it went. Every candidate has to identify itself
             * as a SimpleBGC before it is adopted: a robot carries more than
             * one USB serial device, and picking whichever one happens to be
             * enumerated first means writing gimbal frames into a LiDAR while
             * telling the operator the link recovered.
             */
            struct stat pst;
            if (::stat(opt.port.c_str(), &pst) != 0) {
                sbgc_port_t ports[SBGC_PORT_LIST_MAX];
                int np = sbgc_gui_config_list_ports(ports, SBGC_PORT_LIST_MAX);

                std::string found;
                for (int pass = 1; pass >= 0 && found.empty(); pass--) {
                    for (int i = 0; i < np; i++) {
                        if (ports[i].stable != pass) continue;
                        if (opt.port == ports[i].path) continue;
                        if (probe_is_sbgc(ports[i].path, opt.baud)) {
                            found = ports[i].path;
                            break;
                        }
                    }
                }

                std::lock_guard<std::mutex> lk(g_status.mu);
                if (!found.empty()) {
                    g_status.port_note = "device moved; following " + found +
                                         " (it answered as a SimpleBGC)";
                    g_status.port = found;
                    opt.port = found;
                } else {
                    /*
                     * Say what is actually true. Claiming to have followed the
                     * device to something that never identified itself is the
                     * failure this check exists to prevent.
                     */
                    g_status.port_note =
                        "configured port is missing and no other serial device "
                        "answered as a SimpleBGC; not adopting one blindly";
                    g_status.link_open = false;
                    g_status.board_responding = false;
                    g_status.link_error = opt.port + " is not present";
                    /* Searching probes real devices, so do it sparingly. */
                    last_reopen = now + 3.5;
                    continue;
                }
            }

            std::memset(&sb, 0, sizeof(sb));
            if (sbgc_open(&sb, opt.port.c_str(), opt.baud) == 0) {
                open_ok = true;
                sbgc_set_quiet(&sb, 1);
                last_rx_time = now;
                std::lock_guard<std::mutex> lk(g_status.mu);
                g_status.link_open = true;
                g_status.link_error.clear();
            } else {
                std::lock_guard<std::mutex> lk(g_status.mu);
                g_status.link_open = false;
                g_status.board_responding = false;
                g_status.link_error = sbgc_last_error(&sb);
                continue;
            }
        }

        // --- ask for what we need, at sensible rates ---
        // Realtime telemetry drives the whole display, so it goes out at
        // 10 Hz. Board info and profile params change rarely.
        // 25 Hz. One exchange is 73 bytes, about 6 ms of wire time at
        // 115200, so this costs roughly a sixth of the link and leaves ample
        // room for the periodic board-info and profile reads. At 10 Hz the
        // gauges visibly stepped; the display can only ever be as smooth as
        // the telemetry underneath it.
        if (now - last_rt > 0.04) {
            last_rt = now;
            if (!send_query(sb, SBGC_CMD_REALTIME_DATA_3, nullptr, 0)) {
                open_ok = false;
                sbgc_close(&sb);
                continue;
            }
        }
        if (now - last_info > 2.0) {
            last_info = now;
            const uint8_t zero = 0;
            send_query(sb, SBGC_CMD_BOARD_INFO, &zero, 1);
        }
        // Re-read the profile every 5 s, and immediately if we have none.
        // Snapshot what we need under the lock, then release it before
        // touching the port — serial I/O must never be done while holding a
        // lock the HTTP thread also wants.
        bool want_params = false;
        uint8_t profile = 0;   // 0 = "Profile 1" in the GUI
        {
            std::lock_guard<std::mutex> lk(g_status.mu);
            /*
             * Rate-limit unconditionally. Writing this as
             * (!have_params || elapsed > 5) short-circuits the limit while we
             * have no params, so a board that is slow to answer — or one
             * replying CMD_ERROR, which leaves have_params false forever —
             * gets a fresh 134-byte request on every 60 ms pass.
             */
            double interval = g_status.have_params ? 5.0 : 1.0;
            if (now - last_params > interval) {
                want_params = true;
                if (g_status.have_rt) profile = g_status.rt.cur_profile;
            }
        }
        if (want_params) {
            last_params = now;
            send_query(sb, SBGC_CMD_READ_PARAMS_3, &profile, 1);
        }

        /*
         * ---------------------------------------------- calibrate on power --
         *
         * Gyro calibration every time the motors come on.
         *
         * Motors-on is the right trigger: the bias the board learns is only
         * valid for as long as the sensor stays at the temperature and the
         * mounting stress it was measured under, and switching the motors on
         * changes both. A gimbal that has sat limp and cold, then been
         * powered, is exactly the one whose stored bias is furthest from the
         * truth. Doing it here also means the operator never has to remember.
         *
         * It is NOT fired the instant the flag flips, and that is the whole
         * subtlety. Motors engaging is when the gimbal grabs its position and
         * swings while the loop settles, so the moment of switch-on is the
         * single worst instant in the whole session to measure a zero-rate
         * bias at. Calibrating a moving gimbal is what teaches the wrong bias
         * and leaves the camera drifting afterwards — the exact failure this
         * is meant to prevent. So motors-on only ARMS the calibration; it
         * fires once the board's own reported angles say the gimbal has
         * actually stopped moving.
         *
         * If it never settles — the robot is driving, the gimbal is
         * oscillating, someone is holding it — the request is abandoned with
         * a warning rather than run on a moving gimbal or waited on forever.
         *
         * Requires --allow-control, because read-only has to mean read-only.
         * It deliberately does NOT wait for the UI arm toggle: arming guards
         * against unwanted OPERATOR-COMMANDED motion, and this commands none.
         */
        if (opt.calib_gyro_on_motors && opt.allow_control) {
            bool motors_now = false, responding = false, busy = false;
            bool moving = false;
            double angle[SBGC_NUM_AXES] = { 0, 0, 0 };
            double stamp = 0;
            {
                std::lock_guard<std::mutex> lk(g_status.mu);
                responding = g_status.board_responding && g_status.have_rt;
                motors_now = responding && g_status.rt.motors_on != 0;
                busy       = g_status.calib_running;
                moving     = g_status.motion_active || g_status.task_active;
                stamp      = g_status.rt_stamp;
                for (int a = 0; a < SBGC_NUM_AXES; a++)
                    angle[a] = g_status.rt.imu_deg[a];
            }

            // Motors off, or the board went quiet: nothing is armed, and the
            // next power-on starts the sequence again from scratch.
            if (!motors_now) {
                if (calib_armed) {
                    calib_armed = false;
                    std::lock_guard<std::mutex> lk(g_status.mu);
                    g_status.calib_pending = false;
                }
                motors_were_on = false;
            } else {
                if (!motors_were_on) {
                    motors_were_on = true;
                    calib_armed    = true;
                    calib_armed_at = now;
                    still_since    = 0.0;
                    have_prev_angle = false;
                    std::lock_guard<std::mutex> lk(g_status.mu);
                    g_status.calib_pending = true;
                    g_status.calib_skipped = false;   // a fresh attempt
                }

                /*
                 * Stillness is judged only on FRESH samples. The loop runs
                 * faster than telemetry arrives, so comparing every pass would
                 * compare a reading with itself and call a moving gimbal
                 * still within two iterations.
                 */
                if (calib_armed && !busy && stamp != last_still_stamp) {
                    last_still_stamp = stamp;
                    double moved = 0.0;
                    if (have_prev_angle) {
                        /* Shortest step, not the raw difference: a hair of
                         * movement across the display fold reads as 360 deg
                         * and would keep a perfectly still gimbal from ever
                         * being judged settled. */
                        for (int a = 0; a < SBGC_NUM_AXES; a++)
                            moved = std::max(moved, angle_gap(angle[a], prev_angle[a]));
                    }
                    for (int a = 0; a < SBGC_NUM_AXES; a++) prev_angle[a] = angle[a];

                    if (!have_prev_angle) {
                        have_prev_angle = true;         // no delta to judge yet
                    } else if (moved > CALIB_STILL_DEG) {
                        still_since = 0.0;              // still settling
                    } else if (still_since == 0.0) {
                        still_since = now;
                    }
                }

                bool settled = calib_armed && !busy && !moving &&
                               still_since > 0.0 &&
                               (now - still_since) >= CALIB_STILL_S;

                if (settled) {
                    calib_armed = false;
                    {
                        std::lock_guard<std::mutex> lk(g_status.mu);
                        g_status.calib_pending = false;
                        g_status.calib_running = true;
                        g_status.calib_started = monotonic_s();
                    }
                    if (sbgc_calib_gyro(&sb) == 0) {
                        note_tx(sb, "calibrate gyro (motors on)");
                        std::printf("  motors on and gimbal settled — calibrating "
                                    "gyro; keep it still\n");
                    } else {
                        note_tx_failed(sb, "calibrate gyro (motors on)");
                        std::lock_guard<std::mutex> lk(g_status.mu);
                        g_status.calib_running = false;
                        std::fprintf(stderr,
                                     "  gyro calibration failed to send: %s\n",
                                     sbgc_last_error(&sb));
                    }
                    std::fflush(stdout);
                } else if (calib_armed &&
                           (now - calib_armed_at) > CALIB_SETTLE_TIMEOUT_S) {
                    /*
                     * Never went still. Abandoning is the only honest option:
                     * calibrating anyway would write the wrong bias, and
                     * holding the request open forever would leave the
                     * operator wondering why it never ran.
                     */
                    calib_armed = false;
                    std::lock_guard<std::mutex> lk(g_status.mu);
                    g_status.calib_pending = false;
                    g_status.calib_skipped = true;
                }
            }
        }

        /*
         * Gyro calibration has no reply this tool has verified, so completion
         * is reported on a fixed duration rather than a confirmation frame.
         * The UI says "hold still" for that long; claiming to detect an
         * actual completion signal would be inventing one.
         */
        {
            std::lock_guard<std::mutex> lk(g_status.mu);
            if (g_status.calib_running &&
                monotonic_s() - g_status.calib_started > CALIB_SECONDS)
                g_status.calib_running = false;
            // A home or level is assumed finished after TASK_SECONDS, for the
            // same reason and with the same caveat as calibration: the board
            // announces neither.
            if (g_status.task_active &&
                monotonic_s() - g_status.task_started > TASK_SECONDS)
                g_status.task_active = false;
        }

        // --- motion ---
        // Runs at the loop rate so a held control feels continuous, and so
        // the watchdog can cut in promptly when commands stop arriving.
        {
            double pan = 0, tilt = 0, roll = 0, speed = 30.0, stamp = 0;
            bool armed, want_home = false, want_level = false, want_stop = false;
            bool want_calib = false, custom_home = false;
            int16_t home_units[SBGC_NUM_AXES] = { 0, 0, 0 };
            bool   lim_on = false, have_angle = false;
            double lim_yaw_min = 0, lim_yaw_max = 0;
            double lim_pitch_min = 0, lim_pitch_max = 0;
            double yaw_now = 0, pitch_now = 0;
            int  want_motors = -1;
            bool was_active, was_task, calibrating;
            {
                std::lock_guard<std::mutex> lk(g_status.mu);
                armed = g_status.control_allowed && g_status.control_armed;
                pan   = g_status.intent_pan;
                tilt  = g_status.intent_tilt;
                roll  = g_status.intent_roll;
                stamp = g_status.intent_stamp;
                speed = g_status.speed_deg_s;
                was_active = g_status.motion_active;
                was_task    = g_status.task_active;
                calibrating = g_status.calib_running;

                want_home   = g_status.pending_home;   g_status.pending_home   = false;
                want_level  = g_status.pending_level;  g_status.pending_level  = false;
                want_stop   = g_status.pending_stop;   g_status.pending_stop   = false;
                want_motors = g_status.pending_motors; g_status.pending_motors = -1;
                want_calib  = g_status.pending_calib_gyro;
                g_status.pending_calib_gyro = false;

                lim_on        = g_status.user_limits_on;
                lim_yaw_min   = g_status.user_yaw.min_deg;
                lim_yaw_max   = g_status.user_yaw.max_deg;
                lim_pitch_min = g_status.user_pitch.min_deg;
                lim_pitch_max = g_status.user_pitch.max_deg;
                /*
                 * The angle must be CURRENT, not merely present. have_rt only
                 * ever latches true, so a board whose TX path has died —
                 * broken RX wire, IMU fault — while it still accepts commands
                 * leaves rt frozen at a plausible-looking value. Checking a
                 * limit against that reading lets the camera drive straight
                 * through it, because the number never moves.
                 */
                have_angle    = g_status.have_rt && g_status.angle_cont_valid &&
                                (monotonic_s() - g_status.rt_stamp) < ANGLE_FRESH_S;
                /*
                 * Gated on the board's UNWRAPPED count, not on the display
                 * angle.
                 *
                 * imu_deg is folded into (-180, 180] so the UI never shows an
                 * attitude no mount could reach, and that fold is a
                 * discontinuity. A gimbal at a true +200 deg reads as -160,
                 * which lands back inside a [-170, 170] range: the gate stops
                 * seeing a limit at all and the axis runs on for most of a
                 * turn before the wrapped value comes round to block it again.
                 *
                 * imu_units is the board's own continuous count.
                 * sbgc_params.h already requires it for anything that becomes
                 * a command target, and a limit decision is the same kind of
                 * use — it has to name where the axis physically is, not where
                 * it appears on a dial.
                 */
                yaw_now   = g_status.yaw_cont;
                pitch_now = g_status.pitch_cont;
                if (!g_status.motion_active) {
                    g_status.limit_blocked_yaw = g_status.limit_blocked_pitch = false;
                    g_status.limit_stale = false;
                }

                custom_home = g_status.have_custom_home;
                for (int a = 0; a < SBGC_NUM_AXES; a++)
                    home_units[a] = g_status.home_units[a];

                // Disarming must not leave a rate queued for the next arm.
                if (!armed) {
                    g_status.intent_pan = g_status.intent_tilt =
                        g_status.intent_roll = 0.0;
                }
            }

            bool fresh = (monotonic_s() - stamp) < CONTROL_TIMEOUT_S;
            bool want_move = armed && fresh &&
                             (pan != 0.0 || tilt != 0.0 || roll != 0.0);

            if (!armed) {
                // Not armed: honour a queued stop, and if anything was in
                // flight stop once, then stay silent. Consuming pending_stop
                // above without acting on it here would drop a stop that
                // arrived in the instant the operator disarmed.
                //
                // was_task is part of the test because a home or level is
                // owned by the board until it finishes. Without it, disarming
                // mid-slew saw no rate in flight, sent nothing, and the camera
                // carried on to home while the UI read "Safe" — so disarm
                // could not be used to abort a home heading somewhere unsafe.
                if (was_active || was_task || want_stop) {
                    /*
                     * A plain hold, not send_rate_roll_locked. Re-levelling
                     * roll would be initiating a movement, and this branch runs
                     * precisely when control is not armed. Holding every axis
                     * where it stands is the conservative choice here.
                     */
                    if (sbgc_stop(&sb) == 0) {
                        note_tx(sb, "stop");
                        std::lock_guard<std::mutex> lk(g_status.mu);
                        g_status.motion_active = false;
                        // A zero-rate CMD_CONTROL replaces the AUTO_TASK, so
                        // this stop ends the home/level too.
                        g_status.task_active = false;
                    } else {
                        /*
                         * The frame never left the port. motion_active stays
                         * set: it is what makes the reconnect path stop the
                         * board before anything else, and it is what raises
                         * the "moving with no link" warning in the meantime.
                         * Clearing it here would report a stop that did not
                         * happen and discard the only record that one is owed.
                         */
                        note_tx_failed(sb, "stop");
                    }
                }
            } else {
                if (want_motors >= 0)
                    send_motion(sb, want_motors ? "motors on" : "motors off", [&] {
                        return want_motors ? sbgc_motors_on(&sb)
                                           : sbgc_motors_off(&sb);
                    });
                /*
                 * home and level are AUTO_TASKs: the board owns the move
                 * until it finishes. Any CMD_CONTROL sent afterwards replaces
                 * that task, including a zero-rate "stop" — so the stop below
                 * must not run in the same pass that issued one. Getting this
                 * wrong makes the gimbal lurch toward home and immediately be
                 * cut off, which on repeated presses reads as shaking.
                 *
                 * An explicit stop still wins: it is the safe direction, and
                 * an operator pressing stop must always be obeyed.
                 */
                bool task_issued = false;
                if (want_home && !calibrating) {
                    bool ok;
                    if (custom_home) {
                        /*
                         * A taught home: drive to the stored angles rather
                         * than the board's frame-neutral, slewing at the
                         * operator's speed so it is a controlled move.
                         *
                         * rel_frame is 0 because the angles were captured
                         * from imu_deg, which is measured against the
                         * horizon. Replaying a horizon-referenced angle as a
                         * frame-relative one would send the camera somewhere
                         * it was never taught as soon as the robot sat on a
                         * slope. Capture and replay must share a reference.
                         */
                        /*
                         * Commanded in the board's own units, so the target
                         * names the same physical attitude that was taught
                         * rather than one an integer wrap away.
                         */
                        uint8_t  mode[SBGC_NUM_AXES];
                        int16_t  sp[SBGC_NUM_AXES];
                        for (int a = 0; a < SBGC_NUM_AXES; a++) {
                            mode[a] = SBGC_MODE_ANGLE;
                            sp[a]   = sbgc_degs_to_units(speed);
                        }
                        ok = send_motion(sb, "home (taught)", [&] {
                            return sbgc_control_raw(&sb, mode, sp, home_units);
                        });
                    } else {
                        ok = send_motion(sb, "home", [&] { return sbgc_home(&sb); });
                    }
                    if (ok) task_issued = true;
                }
                if (want_level && !calibrating &&
                    send_motion(sb, "level", [&] { return sbgc_level(&sb); }))
                    task_issued = true;

                if (task_issued) {
                    std::lock_guard<std::mutex> lk(g_status.mu);
                    g_status.task_active  = true;
                    g_status.task_started = monotonic_s();
                }

                /*
                 * Calibration counts as a task too, and a rate already in
                 * flight has to be RECALLED before it starts rather than
                 * merely forgotten. The board learns its zero-rate bias from a
                 * gimbal it assumes is still; starting on one that is slewing
                 * is precisely the case that teaches a wrong bias and leaves
                 * the camera drifting for every session afterwards.
                 *
                 * If that stop cannot be sent, the calibration does not start.
                 * Better to lose the request than to run it on a moving gimbal.
                 */
                if (want_calib) {
                    bool still_moving = was_active;
                    if (still_moving) {
                        if (send_rate_roll_locked(&sb, 0.0, 0.0, speed) == 0) {
                            note_tx(sb, "stop (before calibration)");
                            still_moving = false;
                            std::lock_guard<std::mutex> lk(g_status.mu);
                            g_status.motion_active = false;
                        } else {
                            note_tx_failed(sb, "stop (before calibration)");
                        }
                    }
                    if (!still_moving &&
                        send_motion(sb, "calibrate gyro",
                                    [&] { return sbgc_calib_gyro(&sb); })) {
                        task_issued = true;
                        calibrating = true;
                        std::lock_guard<std::mutex> lk(g_status.mu);
                        g_status.calib_running = true;
                        g_status.calib_started = monotonic_s();
                        // An explicit calibration answers whatever the
                        // automatic one failed to do.
                        g_status.calib_skipped = false;
                    }
                }

                if (want_stop) {
                    if (send_rate_roll_locked(&sb, 0.0, 0.0, speed) == 0) {
                        note_tx(sb, "stop");
                        std::lock_guard<std::mutex> lk(g_status.mu);
                        g_status.motion_active = false;
                        g_status.task_active   = false;
                    } else {
                        // Still moving as far as anyone knows. Keep the flag
                        // so the next pass retries and the reconnect path
                        // knows a stop is outstanding.
                        note_tx_failed(sb, "stop");
                    }
                } else if (task_issued) {
                    // The board is running the task. Drop our rate state so
                    // the watchdog does not fire a stop that cancels it.
                    std::lock_guard<std::mutex> lk(g_status.mu);
                    g_status.motion_active = false;
                } else if (calibrating) {
                    /*
                     * Hold everything for the WHOLE calibration, not just the
                     * pass that started it.
                     *
                     * Suppressing one pass was not enough: the browser
                     * republishes at 20 Hz, so on the very next pass — tens of
                     * milliseconds later — want_calib was already false, the
                     * held control was still fresh, and a rate went out for
                     * the remaining four seconds of a calibration that is
                     * supposed to happen on a motionless gimbal.
                     *
                     * Nothing is transmitted here. Motion was recalled before
                     * the calibration began, so there is nothing left to stop,
                     * and any CMD_CONTROL now would cancel the calibration.
                     */
                } else if (want_move && lim_on && !have_angle) {
                    /*
                     * Limits are on and there is no current angle to check
                     * them against. Refuse the move rather than let it through
                     * unchecked: the operator asked for a limit to be
                     * enforced, and enforcing one against a position nobody
                     * knows is not something this can honestly do. Turning the
                     * limits off in the UI remains available to recover a
                     * camera by hand.
                     */
                    std::lock_guard<std::mutex> lk(g_status.mu);
                    g_status.limit_stale = true;
                    g_status.limit_blocked_yaw = g_status.limit_blocked_pitch = true;
                    if (was_active) g_status.motion_active = false;
                } else if (want_move) {
                    // Map the UI's normalised deflection onto the logical
                    // axes. Roll is scaled down: on a patrol camera it is
                    // almost never wanted at the same rate as pan and tilt.
                    /*
                     * Soft limits, gated on the board's measured angle rather
                     * than a dead-reckoned guess. Only the component pushing
                     * further past a limit is removed — moving back toward
                     * the allowed range always stays available, otherwise a
                     * gimbal that overshot could never be recovered.
                     */
                    double pan_r = pan * speed, tilt_r = tilt * speed;
                    bool blk_yaw = false, blk_pitch = false;
                    if (lim_on && have_angle) {
                        if (yaw_now >= lim_yaw_max && pan_r > 0) { pan_r = 0; blk_yaw = true; }
                        if (yaw_now <= lim_yaw_min && pan_r < 0) { pan_r = 0; blk_yaw = true; }
                        if (pitch_now >= lim_pitch_max && tilt_r > 0) { tilt_r = 0; blk_pitch = true; }
                        if (pitch_now <= lim_pitch_min && tilt_r < 0) { tilt_r = 0; blk_pitch = true; }
                    }
                    {
                        std::lock_guard<std::mutex> lk(g_status.mu);
                        g_status.limit_blocked_yaw   = blk_yaw;
                        g_status.limit_blocked_pitch = blk_pitch;
                    }

                    if (send_motion(sb, "rate", [&] {
                            return send_rate_roll_locked(&sb, pan_r, tilt_r, speed);
                        })) {
                        std::lock_guard<std::mutex> lk(g_status.mu);
                        g_status.motion_active = true;
                        // This CMD_CONTROL replaced any AUTO_TASK the board
                        // was running, so there is no longer one to abort.
                        g_status.task_active = false;
                    }
                } else if (was_active) {
                    // The operator released, or commands stopped arriving and
                    // the watchdog expired. Both mean stop.
                    if (send_rate_roll_locked(&sb, 0.0, 0.0, speed) == 0) {
                        note_tx(sb, "stop (watchdog)");
                        std::lock_guard<std::mutex> lk(g_status.mu);
                        g_status.motion_active = false;
                        g_status.task_active   = false;
                    } else {
                        // Leaving motion_active set means this branch runs
                        // again next pass, so a stop keeps being retried for
                        // as long as the board is believed to be moving.
                        note_tx_failed(sb, "stop (watchdog)");
                    }
                }
            }
        }

        // --- collect replies ---
        // Short block: this timeout sets the floor on the whole loop, so a
        // long one throttles motion updates and the watchdog as well as
        // telemetry.
        int n = sbgc_poll(&sb, 15, on_frame, &rs);
        if (n < 0) {
            open_ok = false;
            sbgc_close(&sb);
            std::lock_guard<std::mutex> lk(g_status.mu);
            g_status.link_open = false;
            g_status.board_responding = false;
            g_status.link_error = sbgc_last_error(&sb);
            /*
             * motion_active is deliberately NOT cleared. It is the record that
             * a rate was in flight when the link died, and it is what makes
             * the reconnect path send a stop before anything else; clearing it
             * would lose that and let the gimbal keep whatever rate it had.
             *
             * The board has no serial-loss failsafe of its own, so while the
             * port is down nothing can stop it. That state gets its own
             * warning rather than being left for the operator to infer.
             */
            continue;
        }
        if (n > 0) last_rx_time = monotonic_s();

        {
            double age = monotonic_s() - last_rx_time;
            std::lock_guard<std::mutex> lk(g_status.mu);
            g_status.last_frame_age_s = age;
            // A board that has said nothing for a second is not talking to us,
            // even though the port is still open. Report that distinctly:
            // "cable present but board silent" is a different fault from
            // "no such device".
            bool responding = age < 1.0;
            if (!responding && g_status.board_responding) g_status.timeouts++;
            g_status.board_responding = responding;
        }
    }

    /*
     * Last act before releasing the port: stop. The board has no serial-loss
     * failsafe, so exiting while a rate is in flight would leave the gimbal
     * turning with nothing left alive to recall it.
     */
    if (open_ok) {
        bool was_moving;
        {
            std::lock_guard<std::mutex> lk(g_status.mu);
            was_moving = g_status.motion_active;
            g_status.motion_active = false;
        }
        if (was_moving) sbgc_stop(&sb);
        sbgc_close(&sb);
    }
}

// ----------------------------------------------------------- gamepad loop --

/*
 * The controller is watched in its own thread purely so the UI can report
 * whether one is attached. Reading evdev cannot move the gimbal; the deadman
 * state is surfaced for the operator's benefit only.
 */
void gamepad_thread(Options opt)
{
    gp_t gp;
    bool open_ok = false;
    double last_try = 0.0;

    while (g_running.load()) {
        double now = monotonic_s();

        if (!open_ok) {
            if (now - last_try < 2.0) { usleep(200000); continue; }
            last_try = now;

            int rc = opt.pad_path.empty() ? gp_open_auto(&gp)
                                          : gp_open(&gp, opt.pad_path.c_str());
            if (rc == 0) {
                open_ok = true;
                std::lock_guard<std::mutex> lk(g_status.mu);
                g_status.pad_present = true;
                g_status.pad_name = gp_name(&gp);
                g_status.pad_path = gp.path;
            } else {
                std::lock_guard<std::mutex> lk(g_status.mu);
                g_status.pad_present = false;
                g_status.pad_name.clear();
                g_status.pad_path.clear();
                g_status.pad_deadman = false;
                continue;
            }
        }

        int pr = gp_poll(&gp, 100);
        if (pr < 0) {
            // Unplugged. Report it and go back to scanning.
            gp_close(&gp);
            open_ok = false;
            std::lock_guard<std::mutex> lk(g_status.mu);
            g_status.pad_present = false;
            g_status.pad_name.clear();
            g_status.pad_deadman = false;
            continue;
        }

        bool held = gp_button(&gp, GP_BTN_LB) != 0;
        gp_latch_buttons(&gp);
        std::lock_guard<std::mutex> lk(g_status.mu);
        g_status.pad_deadman = held;
    }

    if (open_ok) gp_close(&gp);
}

// -------------------------------------------------------------------- JSON --

void json_escape(std::string &out, const std::string &s)
{
    for (char c : s) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char b[8];
                    std::snprintf(b, sizeof(b), "\\u%04x", c);
                    out += b;
                } else {
                    out += c;
                }
        }
    }
}

void kv_str(std::string &o, const char *k, const std::string &v, bool comma = true)
{
    o += "\""; o += k; o += "\":\"";
    json_escape(o, v);
    o += "\"";
    if (comma) o += ",";
}

void kv_num(std::string &o, const char *k, double v, int prec = 2, bool comma = true)
{
    char b[64];
    std::snprintf(b, sizeof(b), "%.*f", prec, v);
    o += "\""; o += k; o += "\":";
    o += b;
    if (comma) o += ",";
}

void kv_int(std::string &o, const char *k, long v, bool comma = true)
{
    char b[32];
    std::snprintf(b, sizeof(b), "%ld", v);
    o += "\""; o += k; o += "\":"; o += b;
    if (comma) o += ",";
}

void kv_bool(std::string &o, const char *k, bool v, bool comma = true)
{
    o += "\""; o += k; o += "\":";
    o += v ? "true" : "false";
    if (comma) o += ",";
}

const char *AXIS_NAME[3] = { "roll", "pitch", "yaw" };

/*
 * Warnings are computed here rather than in the page, so the rules live next
 * to the data they judge and the UI stays a renderer.
 */
struct Warning {
    std::string level;   // "danger" | "warn" | "info"
    std::string text;
};

std::vector<Warning> compute_warnings(const Status &st)
{
    std::vector<Warning> w;

    /*
     * Raised before the link check, because this is exactly the case where
     * there is no link left to report on. The board has no serial-loss
     * failsafe, so a rate that was in flight when the port died is still
     * running and cannot be recalled.
     */
    if ((st.motion_active || st.task_active) && !st.board_responding) {
        w.push_back({ "danger",
            "The link dropped while the gimbal was moving. This board has no "
            "serial-loss failsafe, so it may still be turning and cannot be "
            "commanded until the link is back. Cut motor power if it is "
            "heading for a hard stop." });
    }

    /*
     * Enforcing a travel limit needs a current angle. When there is not one,
     * motion is held rather than allowed through unchecked, and the operator
     * has to be told why the controls have gone dead — otherwise this looks
     * identical to a broken page.
     */
    if (st.limit_stale) {
        w.push_back({ "danger",
            "Travel limits are on, but the board has stopped reporting its "
            "angle, so there is no way to tell where the camera is pointing. "
            "Motion is held until a fresh reading arrives. Turn the limits off "
            "if you need to move the camera by hand first." });
    }

    if (!st.link_open) {
        w.push_back({ "danger",
            "Serial port " + st.port + " could not be opened" +
            (st.link_error.empty() ? "" : ": " + st.link_error) +
            ". If the SimpleBGC GUI is running, disconnect it — it holds the "
            "port exclusively." });
        return w;   // nothing else is knowable
    }

    if (!st.board_responding) {
        w.push_back({ "danger",
            "Port is open but the board is not replying. Check that it is "
            "powered, that the baud rate is right, and that no other program "
            "has the port." });
    }

    if (st.have_rt) {
        const sbgc_realtime_t &rt = st.rt;

        if (!rt.motors_on) {
            w.push_back({ "warn",
                "Motors are OFF. The gimbal is limp, and Home, Level and Stop "
                "will be sent but nothing will move. Turn motors on to "
                "actually drive the camera." });
        }

        // The board's own alarm threshold is authoritative; it is stored as
        // hundredths of a volt and negative when the feature is disabled.
        if (st.have_params && st.params.bat_threshold_alarm > 0) {
            double alarm = st.params.bat_threshold_alarm / 100.0;
            if (rt.battery_volts > 0.5 && rt.battery_volts < alarm) {
                char b[160];
                std::snprintf(b, sizeof(b),
                    "Battery %.2f V is below the board's alarm threshold "
                    "of %.2f V", rt.battery_volts, alarm);
                w.push_back({ "danger", b });
            }
        }

        if (rt.error_code != 0) {
            char b[192];
            std::snprintf(b, sizeof(b),
                "Board reports error code %u. This tool does not decode the "
                "code — open the SimpleBGC GUI to see what it means.",
                rt.error_code);
            w.push_back({ "warn", b });
        }

        if (rt.i2c_error_count > 0) {
            char b[160];
            std::snprintf(b, sizeof(b),
                "%u I2C errors — usually an IMU wiring or connector fault",
                rt.i2c_error_count);
            w.push_back({ "warn", b });
        }

        if (!rt.rc_signal_present) {
            w.push_back({ "info",
                "No RC receiver detected on any channel (all inputs read the "
                "no-signal sentinel). Expected when driving over serial." });
        }
    }

    if (!st.pad_present) {
        w.push_back({ "info",
            "No gamepad detected. If one is plugged in, its device node "
            "usually needs group access: add yourself to the 'input' group "
            "and log back in." });
    }

    // A saved .profile export that no longer matches the board is worth
    // saying out loud: it means the file someone might reload, or reason
    // from, describes a gimbal that no longer exists.
    if (st.have_params && st.have_file_limits) {
        struct { const char *name; const AxisLimits *file; int idx; } axes[] = {
            { "roll",  &st.file_roll,  SBGC_ROLL  },
            { "pitch", &st.file_pitch, SBGC_PITCH },
            { "yaw",   &st.file_yaw,   SBGC_YAW   },
        };
        for (auto &a : axes) {
            // file_* are stored the way the UI reads them, so the board's
            // pair is converted before the comparison. Comparing the two
            // conventions directly would report every pitch limit as a
            // mismatch, on a board that agrees with the file exactly.
            double raw_min = st.params.rc[a.idx].rc_min_angle;
            double raw_max = st.params.rc[a.idx].rc_max_angle;
            AxisLimits board = ui_limits_from_board(a.idx, raw_min, raw_max);
            double bmin = board.min_deg, bmax = board.max_deg;
            if (a.file->min_deg == bmin && a.file->max_deg == bmax) continue;

            char b[280];
            if (bmin == 0.0 && bmax == 0.0) {
                // Nothing was adopted for this axis, so say what is actually
                // in force rather than claiming the board overrode it.
                std::snprintf(b, sizeof(b),
                    "No angle limit is configured on the board for %s. The "
                    "saved profile's [%.0f, %.0f] is shown instead, and is "
                    "not enforced by the hardware.",
                    a.name, a.file->min_deg, a.file->max_deg);
                w.push_back({ "warn", b });
            } else {
                std::snprintf(b, sizeof(b),
                    "Saved profile is out of date for %s: the file says "
                    "[%.0f, %.0f], the board says [%.0f, %.0f]. The board's "
                    "values are in use.",
                    a.name, a.file->min_deg, a.file->max_deg, bmin, bmax);
                w.push_back({ "info", b });
            }
        }
    }

    /*
     * The board calibrates its own gyro when the CONTROLLER powers on, unless
     * "Skip gyro calibration at startup" is set (2.6x manual, "IMU
     * Calibration"). That is a different event from the motors being switched
     * on, which is what this tool triggers on and which can happen many times
     * without the board ever rebooting — so the two do not make each other
     * redundant. Say which is which rather than implying one replaces the
     * other.
     */
    if (st.have_params && st.params.skip_gyro_calib == 0) {
        w.push_back({ "info",
            "The board also calibrates its gyro when the controller itself "
            "powers on. This tool calibrates when the MOTORS are switched on, "
            "which is a different and more frequent event. Run "
            "--no-calib-gyro if you only want the board's own." });
    }

    // The controls are about to go quiet for several seconds. Saying so first
    // is the difference between "it is calibrating" and "it has frozen".
    if (st.calib_pending) {
        w.push_back({ "info",
            "Motors are on, so the gyro will be calibrated as soon as the "
            "gimbal stops settling. Keep it still and it will start sooner." });
    }

    if (st.calib_skipped) {
        w.push_back({ "warn",
            "The gyro was not calibrated after the motors came on: the gimbal "
            "never stopped moving long enough to measure a bias. It may drift. "
            "Use the Calibrate button once it is stationary." });
    }

    /*
     * Per the 2.6x manual, RC Settings: "For YAW axis limits are not applied
     * in the Lock mode". follow_mode 0 is Lock, so whatever yaw range the
     * board has configured is inert and this tool's own limit is the only
     * thing bounding yaw travel.
     */
    if (st.have_params && st.params.follow_mode == 0 && !st.user_limits_on) {
        w.push_back({ "warn",
            "The board is in Lock mode, where it does not apply its own yaw "
            "angle limits at all. Nothing is bounding yaw travel — turn on "
            "Travel limits to set one." });
    }

    /*
     * The manual's tuning procedure raises P until the motor oscillates, then
     * backs off. An axis whose P is far above its neighbours is the first
     * thing to suspect when a gimbal shakes, so it is worth pointing at —
     * as an observation, not a diagnosis, since the right value depends on
     * the motor and the mass it carries.
     */
    if (st.have_params) {
        const char *names[3] = { "Roll", "Pitch", "Yaw" };
        int pmax = 0, pmin = 255, imax = 0;
        for (int a = 0; a < 3; a++) {
            int v = st.params.pid[a].p;
            if (v > pmax) { pmax = v; imax = a; }
            if (v < pmin) pmin = v;
        }
        if (pmin > 0 && pmax >= pmin * 4) {
            char b[220];
            std::snprintf(b, sizeof(b),
                "%s P is %d while the other axes are near %d. The manual tunes "
                "P up until the motor oscillates, so an outlier this large is "
                "worth checking if the gimbal shakes.",
                names[imax], pmax, pmin);
            w.push_back({ "info", b });
        }
    }

    /*
     * An axis with POWER 0 is switched off in the profile. Commands for it are
     * accepted and acknowledged by the board and simply do nothing, which is
     * indistinguishable from a broken control unless someone says so.
     */
    if (st.have_params) {
        const char *names[3] = { "Roll", "Pitch", "Yaw" };
        for (int a = 0; a < 3; a++) {
            if (st.params.pid[a].power == 0) {
                char b[200];
                std::snprintf(b, sizeof(b),
                    "%s motor power is 0 in profile %u, so that axis is "
                    "disabled. Level and Home will not move it.",
                    names[a], st.params.profile_id + 1u);
                w.push_back({ "warn", b });
            }
        }
    }

    if (st.have_params && st.limits_source != Status::LIMITS_BOARD) {
        w.push_back({ "warn",
            "No RC angle limits are configured on the board, so the displayed "
            "travel range is a local assumption, not a guarantee. Set real "
            "limits in the SimpleBGC GUI." });
    }

    if (st.control_allowed && st.control_armed) {
        w.push_back({ "danger",
            "Control is ARMED. Commands from this page can move the gimbal." });
    }

    return w;
}

std::string build_status_json()
{
    /*
     * Enumerate the ports BEFORE taking the lock. Listing them scans
     * /dev/serial/by-id and /dev and readlink()s every entry; doing that
     * while holding the mutex would put a filesystem walk in the path of the
     * motion loop, which takes the same lock several times per iteration —
     * and this file's own rule is that slow work never happens under it.
     */
    /*
     * Cached. Enumerating means two directory scans plus a readlink per entry,
     * and devices do not appear and disappear at the polling rate — doing it
     * on every request put a filesystem walk in the path of the display.
     */
    static std::mutex ports_mu;
    static sbgc_port_t cached[SBGC_PORT_LIST_MAX];
    static int cached_n = 0;
    static double cached_at = -1e9;

    sbgc_port_t ports[SBGC_PORT_LIST_MAX];
    int n_ports;
    {
        std::lock_guard<std::mutex> plk(ports_mu);
        if (monotonic_s() - cached_at > 2.0) {
            cached_n = sbgc_gui_config_list_ports(cached, SBGC_PORT_LIST_MAX);
            cached_at = monotonic_s();
        }
        n_ports = cached_n;
        for (int i = 0; i < n_ports; i++) ports[i] = cached[i];
    }

    std::lock_guard<std::mutex> lk(g_status.mu);
    const Status &st = g_status;

    std::string o = "{";

    // --- link ---
    o += "\"link\":{";
    kv_bool(o, "port_open", st.link_open);
    kv_bool(o, "board_responding", st.board_responding);
    kv_str(o, "port", st.port);
    kv_int(o, "baud", st.baud);
    kv_str(o, "error", st.link_error);
    kv_num(o, "last_frame_age_s", st.last_frame_age_s, 2);
    kv_int(o, "frames_rx", st.frames_rx);
    kv_int(o, "timeouts", st.timeouts);
    kv_str(o, "note", st.port_note);

    // The list is rebuilt on every poll so hot-plugging a device shows up in
    // the picker without the operator having to reload anything.
    o += "\"ports\":[";
    for (int i = 0; i < n_ports; i++) {
        o += "{";
        kv_str(o, "path", ports[i].path);
        kv_str(o, "label", ports[i].label);
        kv_bool(o, "stable", ports[i].stable != 0, false);
        o += "}";
        if (i + 1 < n_ports) o += ",";
    }
    o += "]},";

    // --- board ---
    o += "\"board\":{";
    kv_bool(o, "known", st.have_info);
    if (st.have_info) {
        char ver[32];
        std::snprintf(ver, sizeof(ver), "%u.%u",
                      st.info.board_ver_major, st.info.board_ver_minor);
        kv_str(o, "board_version", ver);
        char fw[32];
        std::snprintf(fw, sizeof(fw), "%u.%u b%u", st.info.firmware_major,
                      st.info.firmware_minor, st.info.firmware_beta);
        kv_str(o, "firmware", fw);
        kv_int(o, "features", st.info.board_features);
    }
    kv_int(o, "state_flags", st.have_info ? st.info.state_flags1 : 0, false);
    o += "},";

    // --- telemetry ---
    o += "\"telemetry\":{";
    kv_bool(o, "valid", st.have_rt);
    if (st.have_rt) {
        const sbgc_realtime_t &rt = st.rt;
        kv_bool(o, "motors_on", rt.motors_on);
        kv_num(o, "battery_volts", rt.battery_volts, 2);
        kv_int(o, "cycle_time_us", rt.cycle_time_us);
        kv_int(o, "i2c_errors", rt.i2c_error_count);
        kv_int(o, "error_code", rt.error_code);
        kv_int(o, "cur_profile", rt.cur_profile + 1);   // 1-based for humans
        kv_bool(o, "rc_signal", rt.rc_signal_present);

        o += "\"angles\":{";
        for (int a = 0; a < 3; a++) {
            o += "\""; o += AXIS_NAME[a]; o += "\":{";
            double imu = rt.imu_deg[a], tgt = rt.target_deg[a];
            if (a == SBGC_PITCH) {
                imu = ui_pitch_from_board(imu);
                tgt = ui_pitch_from_board(tgt);
            }
            kv_num(o, "imu", imu, 2);
            kv_num(o, "target", tgt, 2);
            kv_num(o, "frame", rt.frame_imu_deg[a], 2);
            kv_int(o, "motor_power", rt.motor_power[a], false);
            o += "}";
            if (a < 2) o += ",";
        }
        o += "}";
    } else {
        o += "\"angles\":{}";
    }
    o += "},";

    // --- controller ---
    o += "\"controller\":{";
    kv_bool(o, "present", st.pad_present);
    kv_str(o, "name", st.pad_name);
    kv_str(o, "path", st.pad_path);
    kv_bool(o, "deadman_held", st.pad_deadman, false);
    o += "},";

    // --- limits ---
    o += "\"limits\":{";
    kv_str(o, "source",
           st.limits_source == Status::LIMITS_BOARD ? "board" :
           st.limits_source == Status::LIMITS_FILE  ? "saved profile" :
                                                      "built-in default");
    kv_num(o, "roll_min", st.roll.min_deg, 1);
    kv_num(o, "roll_max", st.roll.max_deg, 1);
    kv_num(o, "pitch_min", st.pitch.min_deg, 1);
    kv_num(o, "pitch_max", st.pitch.max_deg, 1);
    kv_num(o, "yaw_min", st.yaw.min_deg, 1);
    kv_num(o, "yaw_max", st.yaw.max_deg, 1, false);
    o += "},";

    // --- active profile configuration ---
    o += "\"profile\":{";
    kv_bool(o, "valid", st.have_params);
    if (st.have_params) {
        const sbgc_params_t &p = st.params;
        kv_int(o, "number", p.profile_id + 1);
        kv_int(o, "cur_profile_id", p.cur_profile_id + 1);
        kv_str(o, "serial_speed", sbgc_serial_speed_name(p.serial_speed));
        kv_int(o, "gyro_trust", p.gyro_trust);
        kv_int(o, "pwm_freq", p.pwm_freq);
        kv_int(o, "acc_limiter_all", p.acc_limiter_all);
        kv_int(o, "follow_mode", p.follow_mode);
        kv_int(o, "follow_deadband", p.follow_deadband);
        kv_int(o, "follow_expo_rate", p.follow_expo_rate);
        kv_int(o, "adaptive_pid_enabled", p.adaptive_pid_enabled);
        kv_int(o, "profile_flags1", p.profile_flags1);
        kv_int(o, "general_flags1", p.general_flags1);
        kv_num(o, "bat_alarm_v", p.bat_threshold_alarm / 100.0, 2);
        kv_num(o, "bat_motors_v", p.bat_threshold_motors / 100.0, 2);
        kv_num(o, "bat_comp_ref_v", p.bat_comp_ref / 100.0, 2);

        o += "\"axes\":{";
        for (int a = 0; a < 3; a++) {
            o += "\""; o += AXIS_NAME[a]; o += "\":{";
            kv_int(o, "p", p.pid[a].p);
            kv_int(o, "i", p.pid[a].i);
            kv_int(o, "d", p.pid[a].d);
            kv_int(o, "power", p.pid[a].power);
            kv_int(o, "invert", p.pid[a].invert);
            kv_int(o, "poles", p.pid[a].poles);
            kv_int(o, "rc_min_angle", p.rc[a].rc_min_angle);
            kv_int(o, "rc_max_angle", p.rc[a].rc_max_angle);
            kv_int(o, "rc_speed", p.rc[a].rc_speed);
            kv_int(o, "follow_speed", p.follow_speed[a]);
            kv_int(o, "motor_output", p.motor_output[a], false);
            o += "}";
            if (a < 2) o += ",";
        }
        o += "}";
    } else {
        o += "\"axes\":{}";
    }
    o += "},";

    // --- control gating ---
    o += "\"control\":{";
    kv_bool(o, "allowed", st.control_allowed);
    kv_bool(o, "armed", st.control_armed);
    // A home or level slew is the camera moving just as much as a rate is,
    // even though this program is not the thing driving it moment to moment.
    kv_bool(o, "moving", st.motion_active || st.task_active);
    kv_num(o, "speed_deg_s", st.speed_deg_s, 0);
    kv_bool(o, "calib_running", st.calib_running);
    kv_bool(o, "calib_pending", st.calib_pending);
    kv_bool(o, "calib_skipped", st.calib_skipped);
    kv_num(o, "calib_elapsed", st.calib_running
               ? monotonic_s() - st.calib_started : 0.0, 2);
    kv_num(o, "calib_total", CALIB_SECONDS, 0);
    kv_str(o, "last_cmd", st.last_cmd);
    kv_str(o, "last_cmd_hex", st.last_cmd_hex);
    kv_num(o, "last_cmd_age", st.last_cmd_at > 0.0
               ? monotonic_s() - st.last_cmd_at : -1.0, 2);
    kv_bool(o, "limits_on", st.user_limits_on);
    // One decimal, matching the form's step. Reporting these as whole degrees
    // made a typed 90.5 come back as 90 — the page then showed a limit that
    // was not the one being enforced.
    kv_num(o, "lim_yaw_min", st.user_yaw.min_deg, 1);
    kv_num(o, "lim_yaw_max", st.user_yaw.max_deg, 1);
    kv_num(o, "lim_pitch_min", st.user_pitch.min_deg, 1);
    kv_num(o, "lim_pitch_max", st.user_pitch.max_deg, 1);
    kv_bool(o, "blocked_yaw", st.limit_blocked_yaw);
    kv_bool(o, "blocked_pitch", st.limit_blocked_pitch);
    kv_bool(o, "limit_stale", st.limit_stale);
    kv_bool(o, "custom_home", st.have_custom_home);
    kv_num(o, "home_pitch", st.home_pitch_deg, 1);
    kv_num(o, "home_yaw", st.home_yaw_deg, 1, false);
    o += "},";

    // --- warnings, computed here so the page stays a pure renderer ---
    o += "\"warnings\":[";
    {
        std::vector<Warning> ws = compute_warnings(st);
        for (size_t i = 0; i < ws.size(); i++) {
            o += "{";
            kv_str(o, "level", ws[i].level);
            kv_str(o, "text", ws[i].text, false);
            o += "}";
            if (i + 1 < ws.size()) o += ",";
        }
    }
    o += "],";

    kv_str(o, "config_summary", st.config_summary, false);
    o += "}";
    return o;
}

// ----------------------------------------------------------- HTTP handler --

/*
 * Split an HTTP authority — "host", "host:port", "[::1]:port" — into its two
 * parts. An IPv6 literal is bracketed precisely so its colons cannot be read
 * as the port separator, so the brackets have to be handled before the split.
 */
void split_authority(const std::string &authority,
                     std::string &host, std::string &port)
{
    host.clear();
    port.clear();
    if (authority.empty()) return;

    if (authority[0] == '[') {
        size_t end = authority.find(']');
        if (end == std::string::npos) { host = authority; return; }
        host = authority.substr(1, end - 1);
        if (end + 1 < authority.size() && authority[end + 1] == ':')
            port = authority.substr(end + 2);
        return;
    }

    size_t colon = authority.find(':');
    if (colon == std::string::npos) { host = authority; return; }
    host = authority.substr(0, colon);
    port = authority.substr(colon + 1);
}

/* The names that all mean "this machine". Compared whole, never as prefixes. */
bool is_loopback_host(const std::string &host)
{
    return host == "127.0.0.1" || host == "localhost" ||
           host == "::1" || host == "0:0:0:0:0:0:0:1";
}

/*
 * True when this request did not come from another site. A same-origin fetch
 * from our own page carries an Origin naming our host; a tool like curl sends
 * none at all. Anything else is another page trying to drive the gimbal.
 *
 * The authority is compared in full. Prefix-matching the host would accept
 * anything merely *beginning* with a name we trust — localhost.attacker.example
 * is one wildcard DNS record away, and it would pass while being a wholly
 * different site. The port is part of the comparison for the same reason: a
 * page served by some other daemon on this machine is not our page.
 */
bool origin_is_ours(const httpd_request_t *req)
{
    if (!req->origin[0]) return true;          /* not a browser page */

    /* Origin is "scheme://host[:port]" and nothing more, but refuse to let a
     * stray path or query become part of the host if one ever shows up. */
    const char *after_scheme = std::strstr(req->origin, "//");
    std::string authority(after_scheme ? after_scheme + 2 : req->origin);
    size_t cut = authority.find_first_of("/?#");
    if (cut != std::string::npos) authority.erase(cut);

    /* Whatever the operator typed to reach us is what Host carries, so an
     * exact match is same-origin by definition. */
    if (authority == req->host) return true;

    std::string ohost, oport, hhost, hport;
    split_authority(authority, ohost, oport);
    split_authority(std::string(req->host), hhost, hport);

    if (oport != hport) return false;

    /* Reaching the same port by a different spelling of "this machine" —
     * localhost versus 127.0.0.1 — is still our own page. */
    return is_loopback_host(ohost) && is_loopback_host(hhost);
}

void handle_request(const httpd_request_t *req, httpd_response_t *resp, void *)
{
    std::string path(req->path);
    size_t q = path.find('?');
    if (q != std::string::npos) path = path.substr(0, q);

    auto send_text = [&](int status, const char *ctype, const std::string &s) {
        resp->status = status;
        resp->content_type = ctype;
        resp->body = static_cast<char *>(std::malloc(s.size() + 1));
        if (!resp->body) { resp->status = 500; resp->body_len = 0; return; }
        std::memcpy(resp->body, s.data(), s.size());
        resp->body[s.size()] = '\0';
        resp->body_len = s.size();
        resp->owned = 1;
    };

    if (path == "/" || path == "/index.html") {
        resp->status = 200;
        resp->content_type = "text/html; charset=utf-8";
        resp->body = const_cast<char *>(WEB_INDEX_HTML);
        resp->body_len = WEB_INDEX_HTML_LEN;
        resp->owned = 0;
        return;
    }

    if (path == "/api/status") {
        send_text(200, "application/json", build_status_json());
        return;
    }

    /*
     * Choosing which serial device to open. This is not gated behind arming:
     * it changes what this program talks to, not what the gimbal does, and
     * the whole point is to recover a monitor-only session after a re-plug.
     */
    if (path == "/api/port" && std::string(req->method) == "POST") {
        if (!origin_is_ours(req)) {
            send_text(403, "application/json",
                "{\"ok\":false,\"reason\":\"cross-origin request refused\"}");
            return;
        }
        char want[256] = "";
        if (!httpd_form_value(req->body, "path", want, sizeof(want)) || !want[0]) {
            send_text(400, "application/json",
                      "{\"ok\":false,\"reason\":\"no port given\"}");
            return;
        }
        // Only offer what actually exists, so a typo cannot leave the tool
        // pointed at a path that will never appear.
        sbgc_port_t ports[SBGC_PORT_LIST_MAX];
        int n = sbgc_gui_config_list_ports(ports, SBGC_PORT_LIST_MAX);
        bool known = false;
        for (int i = 0; i < n; i++)
            if (std::strcmp(ports[i].path, want) == 0) { known = true; break; }
        if (!known) {
            send_text(404, "application/json",
                      "{\"ok\":false,\"reason\":\"no such serial device\"}");
            return;
        }

        std::lock_guard<std::mutex> lk(g_status.mu);
        // Switching ports must never leave a rate queued for the new board.
        g_status.intent_pan = g_status.intent_tilt = g_status.intent_roll = 0.0;
        g_status.requested_port = want;
        g_status.port_change_pending = true;
        send_text(200, "application/json", "{\"ok\":true}");
        return;
    }

    // Arming is a state change on this program, not on the gimbal, so it is
    // permitted even in monitor mode — but it has no effect without
    // --allow-control, and the reply says so plainly.
    if (path == "/api/arm" && std::string(req->method) == "POST") {
        if (!origin_is_ours(req)) {
            send_text(403, "application/json",
                "{\"ok\":false,\"reason\":\"cross-origin request refused\"}");
            return;
        }
        char val[32] = "";
        bool want = httpd_form_value(req->body, "armed", val, sizeof(val)) &&
                    (std::strcmp(val, "1") == 0 || std::strcmp(val, "true") == 0);

        std::lock_guard<std::mutex> lk(g_status.mu);
        if (!g_status.control_allowed) {
            g_status.control_armed = false;
            send_text(403, "application/json",
                "{\"ok\":false,\"armed\":false,\"reason\":"
                "\"control is disabled; restart with --allow-control\"}");
            return;
        }
        g_status.control_armed = want;
        send_text(200, "application/json",
                  std::string("{\"ok\":true,\"armed\":") +
                  (want ? "true" : "false") + "}");
        return;
    }

    // Motion endpoints. Present so the UI can show real controls, but they
    // refuse unless both unlocks are in place. Nothing is built or sent on
    // the refusal path.
    /*
     * Travel limits. Tightening a limit can only ever reduce what the gimbal
     * is allowed to do, so it is not gated behind arming — being unable to
     * make the thing safer while it is disarmed would be backwards.
     */
    if (path == "/api/limits" && std::string(req->method) == "POST") {
        if (!origin_is_ours(req)) {
            send_text(403, "application/json",
                "{\"ok\":false,\"reason\":\"cross-origin request refused\"}");
            return;
        }
        char v[64];
        auto get = [&](const char *k, double &dst) {
            if (!httpd_form_value(req->body, k, v, sizeof(v))) return true;
            char *end = nullptr;
            double d = std::strtod(v, &end);
            if (end == v || !std::isfinite(d) || d < -180.0 || d > 180.0) return false;
            dst = d;
            return true;
        };
        std::lock_guard<std::mutex> lk(g_status.mu);
        AxisLimits ny = g_status.user_yaw, np = g_status.user_pitch;
        bool on = g_status.user_limits_on;
        if (httpd_form_value(req->body, "enabled", v, sizeof(v)))
            on = (std::strcmp(v, "1") == 0 || std::strcmp(v, "true") == 0);
        if (!get("yaw_min", ny.min_deg) || !get("yaw_max", ny.max_deg) ||
            !get("pitch_min", np.min_deg) || !get("pitch_max", np.max_deg)) {
            send_text(400, "application/json",
                "{\"ok\":false,\"reason\":\"angles must be numbers between -180 and 180\"}");
            return;
        }
        if (ny.min_deg >= ny.max_deg || np.min_deg >= np.max_deg) {
            send_text(400, "application/json",
                "{\"ok\":false,\"reason\":\"each minimum must be below its maximum\"}");
            return;
        }
        g_status.user_yaw = ny;
        g_status.user_pitch = np;
        g_status.user_limits_on = on;
        send_text(200, "application/json", "{\"ok\":true}");
        return;
    }

    /*
     * A deliberately small payload for the gauges. The full status document is
     * several KB and rebuilding it fast enough for a smooth needle would waste
     * most of that bandwidth re-sending a profile table that changes once
     * every five seconds.
     */
    if (path == "/api/live") {
        std::string o = "{";
        {
            std::lock_guard<std::mutex> lk(g_status.mu);
            kv_bool(o, "valid", g_status.have_rt);
            kv_bool(o, "responding", g_status.board_responding);
            if (g_status.have_rt) {
                kv_num(o, "yaw", g_status.rt.imu_deg[SBGC_YAW], 2);
                kv_num(o, "pitch",
                       ui_pitch_from_board(g_status.rt.imu_deg[SBGC_PITCH]), 2);
                kv_num(o, "roll", g_status.rt.imu_deg[SBGC_ROLL], 2);
                kv_num(o, "yaw_t", g_status.rt.target_deg[SBGC_YAW], 2);
                kv_num(o, "pitch_t",
                       ui_pitch_from_board(g_status.rt.target_deg[SBGC_PITCH]), 2);
                kv_num(o, "roll_t", g_status.rt.target_deg[SBGC_ROLL], 2);
            }
            kv_bool(o, "blocked_yaw", g_status.limit_blocked_yaw);
            kv_bool(o, "blocked_pitch", g_status.limit_blocked_pitch);
            kv_bool(o, "moving", g_status.motion_active, false);
        }
        o += "}";
        send_text(200, "application/json", o);
        return;
    }

    if (path.rfind("/api/control/", 0) == 0) {
        /*
         * A method check alone only closes the <img src=...> variant. A
         * form-encoded POST is a CORS-simple request: no preflight, sent
         * cross-origin regardless of the opaque response. Any page the
         * operator happens to have open could otherwise arm control and drive
         * the camera. Browsers always attach Origin to a cross-origin POST,
         * so an Origin that is present and not ours means "not our page".
         */
        if (!origin_is_ours(req)) {
            send_text(403, "application/json",
                "{\"ok\":false,\"reason\":\"cross-origin request refused\"}");
            return;
        }

        // Same method discipline as /api/arm and /api/port. Without it these
        // endpoints are reachable by a plain GET, so any page the operator
        // happens to have open could drive the gimbal with an <img> tag while
        // control is armed.
        if (std::string(req->method) != "POST") {
            send_text(405, "application/json",
                      "{\"ok\":false,\"reason\":\"use POST\"}");
            return;
        }

        bool allowed, armed;
        {
            std::lock_guard<std::mutex> lk(g_status.mu);
            allowed = g_status.control_allowed;
            armed   = g_status.control_armed;
        }

        /*
         * Stop bypasses the ARM check — refusing an operator's stop would
         * invert the purpose of the guard, and the panic key must work
         * whatever the arm state.
         *
         * It does NOT bypass the --allow-control check. A read-only build has
         * never commanded anything, so it has nothing to stop, and letting it
         * emit CMD_CONTROL would break the guarantee that a monitor-only
         * session cannot change what the gimbal is doing — a zero-rate stop
         * still overrides RC or joystick control and freezes the camera.
         */
        if (path == "/api/control/stop" && allowed) {
            std::lock_guard<std::mutex> lk(g_status.mu);
            g_status.intent_pan = g_status.intent_tilt = g_status.intent_roll = 0.0;
            // Drop anything queued as well. A home that was already pending
            // would otherwise fire immediately after the stop and start the
            // gimbal moving again, which reads as "stop did nothing".
            g_status.pending_home = false;
            g_status.pending_level = false;
            g_status.pending_stop = true;
            send_text(200, "application/json", "{\"ok\":true}");
            return;
        }

        if (!allowed) {
            send_text(403, "application/json",
                "{\"ok\":false,\"reason\":\"this build is running read-only; "
                "restart with --allow-control\"}");
            return;
        }
        if (!armed) {
            send_text(403, "application/json",
                "{\"ok\":false,\"reason\":\"control is not armed\"}");
            return;
        }

        std::string verb = path.substr(std::strlen("/api/control/"));
        auto num = [&](const char *key, double def) {
            char v[64] = "";
            if (!httpd_form_value(req->body, key, v, sizeof(v))) return def;
            char *end = nullptr;
            double d = std::strtod(v, &end);
            if (end == v || !std::isfinite(d)) return def;
            return d;
        };
        auto clamp1 = [](double v) { return std::max(-1.0, std::min(1.0, v)); };

        std::lock_guard<std::mutex> lk(g_status.mu);

        if (verb == "rate") {
            // The UI republishes this while a control is held. Values outside
            // -1..1 are clamped rather than refused: a stuck analogue stick
            // reading 1.2 should still mean "full deflection", not an error.
            g_status.intent_pan   = clamp1(num("pan", 0.0));
            g_status.intent_tilt  = clamp1(num("tilt", 0.0));
            // Roll is not operator-controlled; any value sent for it is
            // discarded rather than acted on.
            g_status.intent_roll  = 0.0;
            g_status.intent_stamp = monotonic_s();
            send_text(200, "application/json", "{\"ok\":true}");
            return;
        }
        /* "stop" is handled earlier, before the arm check. */
        if (verb == "home" || verb == "level") {
            g_status.intent_pan = g_status.intent_tilt = g_status.intent_roll = 0.0;
            if (verb == "home") g_status.pending_home = true;
            else                g_status.pending_level = true;
            send_text(200, "application/json", "{\"ok\":true}");
            return;
        }
        if (verb == "sethome") {
            // Teach the position the camera is being held at right now.
            // Requires a live reading — storing a stale or absent angle would
            // teach a home that points somewhere the camera has never been.
            if (!g_status.have_rt) {
                send_text(409, "application/json",
                    "{\"ok\":false,\"reason\":\"no live angle reading from the "
                    "board yet\"}");
                return;
            }
            // Roll is a stabilised axis, so a taught home never carries a
            // roll angle — it is always level, whatever the camera was at.
            g_status.home_units[SBGC_ROLL]  = 0;
            g_status.home_units[SBGC_PITCH] = g_status.rt.imu_units[SBGC_PITCH];
            g_status.home_units[SBGC_YAW]   = g_status.rt.imu_units[SBGC_YAW];
            g_status.home_pitch_deg = ui_pitch_from_board(g_status.rt.imu_deg[SBGC_PITCH]);
            g_status.home_yaw_deg   = g_status.rt.imu_deg[SBGC_YAW];
            g_status.have_custom_home = true;
            send_text(200, "application/json", "{\"ok\":true}");
            return;
        }
        if (verb == "clearhome") {
            g_status.have_custom_home = false;
            send_text(200, "application/json", "{\"ok\":true}");
            return;
        }
        if (verb == "calibgyro") {
            g_status.intent_pan = g_status.intent_tilt = g_status.intent_roll = 0.0;
            g_status.pending_calib_gyro = true;
            send_text(200, "application/json", "{\"ok\":true}");
            return;
        }
        if (verb == "motors") {
            char v[16] = "";
            // A missing or misspelled parameter must not silently read as
            // "off" — that would turn a malformed request into the camera
            // going limp. Say what was wrong instead.
            if (!httpd_form_value(req->body, "on", v, sizeof(v))) {
                send_text(400, "application/json",
                    "{\"ok\":false,\"reason\":\"motors needs on=1 or on=0\"}");
                return;
            }
            bool on  = (std::strcmp(v, "1") == 0 || std::strcmp(v, "true") == 0);
            bool off = (std::strcmp(v, "0") == 0 || std::strcmp(v, "false") == 0);
            if (!on && !off) {
                send_text(400, "application/json",
                    "{\"ok\":false,\"reason\":\"on must be 1 or 0\"}");
                return;
            }
            g_status.pending_motors = on ? 1 : 0;
            send_text(200, "application/json", "{\"ok\":true}");
            return;
        }
        if (verb == "speed") {
            double s = num("deg_s", g_status.speed_deg_s);
            // Refuse implausible rates outright rather than sending them to a
            // real motor with a camera bolted to it.
            if (s < 1.0 || s > 120.0) {
                send_text(400, "application/json",
                    "{\"ok\":false,\"reason\":\"speed must be between 1 and "
                    "120 deg/s\"}");
                return;
            }
            g_status.speed_deg_s = s;
            send_text(200, "application/json", "{\"ok\":true}");
            return;
        }

        send_text(404, "application/json",
                  "{\"ok\":false,\"reason\":\"unknown control command\"}");
        return;
    }

    send_text(404, "text/plain", "not found");
}

// -------------------------------------------------------------------- main --

void usage()
{
    std::printf(
"usage: gimbal_gui [options]\n"
"  Run with no options: port, baud and limits are recovered from an existing\n"
"  SimpleBGC GUI installation.\n\n"
"  --port DEV        serial port (default: from the GUI, else /dev/ttyUSB0)\n"
"  --baud N          baud rate (default: from the GUI, else 115200)\n"
"  --http-port N     listen port (default 8080)\n"
"  --bind ADDR       bind address (default 127.0.0.1; 0.0.0.0 to expose)\n"
"  --gui-dir DIR     SimpleBGC GUI install to read defaults from\n"
"  --pad DEV         gamepad event device; omit to auto-detect\n"
"  --no-pad          do not look for a gamepad\n"
"  --probe-port DEV  ask one device whether it is a SimpleBGC, then exit\n"
"                    (sends only CMD_BOARD_INFO; exit 0 if it answered)\n"
"  --allow-control   permit the UI to arm motion commands (off by default)\n"
"  --no-calib-gyro   never calibrate the gyro automatically (it is ON by\n"
"                    default). The Calibrate button still works.\n"
"  --calib-gyro-on-motors\n"
"                    calibrate the gyro every time the motors come on.\n"
"                    ON BY DEFAULT; this flag only re-enables it after\n"
"                    --no-calib-gyro. (--calib-gyro-on-start is an alias.)\n"
"                    It waits for the gimbal to stop settling first: the\n"
"                    gimbal must be STILL while it runs, or the board learns\n"
"                    a wrong bias and drifts. Needs --allow-control.\n"
"  --help\n");
}

} // namespace

int main(int argc, char **argv)
{
    /*
     * A client that disappears mid-response makes write() on its socket raise
     * SIGPIPE, whose default action is to kill the process. That is not a
     * cosmetic crash here: if this daemon dies while a rate is commanded, the
     * board keeps slewing at that rate — the watchdog that was supposed to
     * stop it died with us. Closing a browser tab must not be able to send
     * the camera into its hard stop, so the signal is ignored and the failed
     * write is handled as the ordinary error it is.
     */
    signal(SIGPIPE, SIG_IGN);

    /*
     * Ctrl-C and systemd's SIGTERM must not kill this process outright. The
     * default disposition would take the serial thread — and its watchdog —
     * down with it while a rate was still in flight, which is exactly the
     * runaway the SIGPIPE handling above exists to prevent. Instead they ask
     * the loops to wind down, and the serial thread sends a stop on its way
     * out. A second signal still hard-kills, so a wedged shutdown is always
     * escapable.
     */
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = [](int) { g_running.store(false); };
    sa.sa_flags = SA_RESETHAND;          /* second signal uses the default */
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    Options opt;
    bool port_from_cli = false, baud_from_cli = false;
    std::string probe_port;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--port" && i + 1 < argc) { opt.port = argv[++i]; port_from_cli = true; }
        else if (a == "--baud" && i + 1 < argc) { opt.baud = std::atoi(argv[++i]); baud_from_cli = true; }
        else if (a == "--http-port" && i + 1 < argc) opt.http_port = std::atoi(argv[++i]);
        else if (a == "--bind" && i + 1 < argc) opt.bind_addr = argv[++i];
        else if (a == "--gui-dir" && i + 1 < argc) opt.gui_dir = argv[++i];
        else if (a == "--pad" && i + 1 < argc) opt.pad_path = argv[++i];
        else if (a == "--probe-port" && i + 1 < argc) probe_port = argv[++i];
        else if (a == "--no-pad") opt.no_pad = true;
        else if (a == "--allow-control") opt.allow_control = true;
        // The old spelling stays accepted: it named the same intent, and
        // silently rejecting it would break existing service units.
        else if (a == "--calib-gyro-on-motors" ||
                 a == "--calib-gyro-on-start") opt.calib_gyro_on_motors = true;
        else if (a == "--no-calib-gyro") opt.calib_gyro_on_motors = false;
        else if (a == "--help" || a == "-h") { usage(); return 0; }
        else { std::fprintf(stderr, "unknown option: %s\n", a.c_str()); return 2; }
    }

    /*
     * Identify one device and exit. Answers "is this actually the gimbal?"
     * without starting the daemon, which is the question the operator has
     * when a re-plug has shuffled the ttyUSB numbering. Sends nothing but
     * CMD_BOARD_INFO, so it is safe to point at a device that turns out to be
     * something else.
     */
    if (!probe_port.empty()) {
        bool ok = probe_is_sbgc(probe_port.c_str(), opt.baud);
        std::printf("%s: %s\n", probe_port.c_str(),
                    ok ? "answered as a SimpleBGC"
                       : "did NOT answer as a SimpleBGC");
        return ok ? 0 : 1;
    }

    // --- recover defaults so the common case needs no arguments ---
    sbgc_gui_config_t gc;
    sbgc_gui_config_discover(&gc, opt.gui_dir.empty() ? nullptr : opt.gui_dir.c_str());

    if (!port_from_cli && gc.have_port) opt.port = gc.port;
    if (!baud_from_cli && gc.have_baud) opt.baud = gc.baud;

    {
        std::lock_guard<std::mutex> lk(g_status.mu);
        g_status.port = opt.port;
        g_status.baud = opt.baud;
        g_status.control_allowed = opt.allow_control;
        g_status.config_summary = gc.summary;

        // Built-in fallbacks, matching gimbal_ctl's defaults. These are only
        // a starting point: the saved profile refines them, and the board
        // itself overrides both as soon as it answers.
        g_status.roll  = { -45.0, 45.0 };
        g_status.pitch = { -90.0, 40.0 };
        g_status.yaw   = { -170.0, 170.0 };
        g_status.limits_source = Status::LIMITS_BUILTIN;

        if (gc.have_limits) {
            // rcMinAngle/rcMaxAngle out of the .profile are the same fields
            // the board reports, so they arrive in the board's convention and
            // need the same conversion.
            g_status.roll  = ui_limits_from_board(SBGC_ROLL,  gc.roll_min,  gc.roll_max);
            g_status.pitch = ui_limits_from_board(SBGC_PITCH, gc.pitch_min, gc.pitch_max);
            g_status.yaw   = ui_limits_from_board(SBGC_YAW,   gc.yaw_min,   gc.yaw_max);
            g_status.limits_source = Status::LIMITS_FILE;

            g_status.have_file_limits = true;
            g_status.file_roll  = g_status.roll;
            g_status.file_pitch = g_status.pitch;
            g_status.file_yaw   = g_status.yaw;
        }
    }

    std::printf("SimpleBGC32 Control — status console\n");
    std::printf("  defaults: %s\n", gc.summary[0] ? gc.summary : "(built-in)");
    std::printf("  serial:   %s @ %d\n", opt.port.c_str(), opt.baud);
    std::printf("  mode:     %s\n",
                opt.allow_control ? "control CAN be armed from the UI"
                                  : "READ-ONLY (monitor); pass --allow-control to change");

    httpd_t hd;
    if (httpd_open(&hd, opt.bind_addr.c_str(), opt.http_port) != 0) {
        std::fprintf(stderr, "could not listen: %s\n", httpd_last_error(&hd));
        return 1;
    }
    std::printf("  open:     http://%s:%d/\n\n",
                opt.bind_addr == "0.0.0.0" ? "localhost" : opt.bind_addr.c_str(),
                opt.http_port);

    std::thread ser(serial_thread, opt);
    std::thread pad;
    if (!opt.no_pad) pad = std::thread(gamepad_thread, opt);

    while (g_running.load()) {
        // A short timeout so a shutdown signal is acted on promptly rather
        // than waiting out a long poll.
        if (httpd_serve(&hd, 200, handle_request, nullptr) < 0) break;
    }

    g_running.store(false);
    std::printf("\nstopping — sending a stop to the gimbal\n");
    std::fflush(stdout);
    ser.join();
    if (pad.joinable()) pad.join();
    httpd_close(&hd);
    return 0;
}
