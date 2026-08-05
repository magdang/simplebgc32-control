/*
 * gimbal_ctl.cpp — interactive SimpleBGC gimbal control.
 *
 * Two phases:
 *   1. Setup. Choose the serial port and declare how the gimbal is physically
 *      mounted and oriented. Nothing moves during setup.
 *   2. Control. A command loop with direction, angle and speed controls, plus
 *      a live arrow-key mode.
 *
 * The orientation step exists because "pan left" is meaningless until the tool
 * knows which SBGC axis is pan and which way is positive on this mount. Getting
 * that wrong is the single most likely cause of a gimbal slamming into its own
 * hard stop, so it is asked up front rather than guessed.
 *
 * Run with --simulate to exercise everything with no hardware attached; every
 * frame is printed as hex instead of being written to a port.
 */

#include "sbgc_api.h"
#include "gamepad.h"

#include <algorithm>
#include <ctime>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <termios.h>
#include <unistd.h>

namespace {

// ------------------------------------------------------------------ config --

// Which logical control maps onto which SBGC axis, and which way is positive.
struct AxisMap {
    int  axis = SBGC_YAW;   // SBGC_ROLL / SBGC_PITCH / SBGC_YAW
    bool invert = false;
    double min_deg = -180.0;
    double max_deg =  180.0;
};

struct Config {
    std::string port = "/dev/ttyUSB0";
    int    baud = 115200;
    bool   simulate = false;

    AxisMap pan;    // left / right
    AxisMap tilt;   // up / down
    AxisMap roll;   // horizon roll

    double speed_deg_s = 30.0;   // default rate for direction commands
    double step_deg    = 5.0;    // default nudge for arrow keys
    bool   rel_frame   = true;   // angles relative to the robot frame

    // Gamepad
    std::string pad_path;        // empty = auto-detect
    double deadzone   = 0.12;    // matches the robot's existing joy teleop
    double fine_scale = 0.25;    // right stick speed relative to left
    double roll_scale = 0.50;    // trigger roll speed relative to sticks
    double boost      = 2.00;    // RB multiplier

    Config() {
        pan  = { SBGC_YAW,   false, -170.0, 170.0 };
        tilt = { SBGC_PITCH, false,  -90.0,  40.0 };
        roll = { SBGC_ROLL,  false,  -45.0,  45.0 };
    }
};

const char *axis_name(int a)
{
    switch (a) {
        case SBGC_ROLL:  return "ROLL";
        case SBGC_PITCH: return "PITCH";
        case SBGC_YAW:   return "YAW";
        default:         return "?";
    }
}

// ------------------------------------------------------------------- input --

std::string trim(const std::string &s)
{
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::string prompt(const std::string &question, const std::string &def)
{
    std::cout << question;
    if (!def.empty()) std::cout << " [" << def << "]";
    std::cout << ": " << std::flush;

    std::string line;
    if (!std::getline(std::cin, line)) return def;
    line = trim(line);
    return line.empty() ? def : line;
}

bool prompt_yes_no(const std::string &question, bool def)
{
    for (;;) {
        std::string r = prompt(question + " (y/n)", def ? "y" : "n");
        if (r.empty()) return def;
        char c = static_cast<char>(std::tolower(r[0]));
        if (c == 'y') return true;
        if (c == 'n') return false;
        std::cout << "  Please answer y or n.\n";
    }
}

bool parse_double(const std::string &s, double &out)
{
    try {
        size_t idx = 0;
        double v = std::stod(s, &idx);
        if (idx != s.size()) return false;
        out = v;
        return true;
    } catch (...) {
        return false;
    }
}

double prompt_double(const std::string &question, double def)
{
    for (;;) {
        std::ostringstream ds;
        ds << def;
        std::string r = prompt(question, ds.str());
        double v;
        if (parse_double(r, v)) return v;
        std::cout << "  Not a number.\n";
    }
}

int prompt_axis(const std::string &what, int def)
{
    std::cout << "  " << what << " is driven by which gimbal axis?\n"
              << "    1) YAW    2) PITCH    3) ROLL\n";
    for (;;) {
        int d = (def == SBGC_YAW) ? 1 : (def == SBGC_PITCH) ? 2 : 3;
        std::string r = prompt("  choice", std::to_string(d));
        if (r == "1") return SBGC_YAW;
        if (r == "2") return SBGC_PITCH;
        if (r == "3") return SBGC_ROLL;
        std::cout << "  Enter 1, 2 or 3.\n";
    }
}

// ------------------------------------------------------------------- setup --

void configure_axis(const char *label, const char *positive_dir, AxisMap &m)
{
    std::cout << "\n-- " << label << " --\n";
    m.axis = prompt_axis(label, m.axis);
    m.invert = prompt_yes_no(
        std::string("  Is ") + positive_dir + " the NEGATIVE direction on this mount?",
        m.invert);
    m.min_deg = prompt_double("  soft limit min (deg)", m.min_deg);
    m.max_deg = prompt_double("  soft limit max (deg)", m.max_deg);
    if (m.min_deg > m.max_deg) {
        std::cout << "  min > max, swapping.\n";
        std::swap(m.min_deg, m.max_deg);
    }
}

void run_setup(Config &cfg)
{
    std::cout <<
        "\n=====================================================\n"
        " SETUP — describe the mount before anything moves\n"
        "=====================================================\n";

    if (!cfg.simulate) {
        cfg.port = prompt("Serial port", cfg.port);
        std::string b = prompt("Baud rate", std::to_string(cfg.baud));
        try { cfg.baud = std::stoi(b); } catch (...) { /* keep previous */ }
    } else {
        std::cout << "(simulation mode — no port will be opened)\n";
    }

    std::cout <<
        "\nOrientation. Map each control onto a physical axis and say which\n"
        "way is positive. If the camera moves the wrong way later, come back\n"
        "here with 'setup' rather than compensating in your head.\n";

    configure_axis("PAN  (left/right)", "right", cfg.pan);
    configure_axis("TILT (up/down)",    "up",    cfg.tilt);
    configure_axis("ROLL (horizon)",    "clockwise", cfg.roll);

    if (cfg.pan.axis == cfg.tilt.axis || cfg.pan.axis == cfg.roll.axis ||
        cfg.tilt.axis == cfg.roll.axis) {
        std::cout << "\n  WARNING: two controls are mapped to the same axis.\n"
                     "  They will fight each other. Re-run 'setup' to fix.\n";
    }

    std::cout << "\n-- motion defaults --\n";
    cfg.speed_deg_s = prompt_double("  default speed (deg/s)", cfg.speed_deg_s);
    cfg.step_deg    = prompt_double("  arrow-key step (deg)",  cfg.step_deg);
    cfg.rel_frame   = prompt_yes_no(
        "  Measure angles relative to the ROBOT FRAME?\n"
        "  (n = relative to the horizon, i.e. gravity-stabilised)",
        cfg.rel_frame);

    std::cout << "\nSetup complete.\n";
}

void show_config(const Config &cfg)
{
    auto line = [](const char *label, const AxisMap &m) {
        std::printf("  %-5s -> %-5s  %-8s  limits [%.1f, %.1f]\n",
                    label, axis_name(m.axis),
                    m.invert ? "inverted" : "normal",
                    m.min_deg, m.max_deg);
    };
    std::cout << "\nCurrent settings\n";
    std::printf("  port        %s @ %d%s\n", cfg.port.c_str(), cfg.baud,
                cfg.simulate ? "  (SIMULATED)" : "");
    line("PAN",  cfg.pan);
    line("TILT", cfg.tilt);
    line("ROLL", cfg.roll);
    std::printf("  speed       %.1f deg/s\n", cfg.speed_deg_s);
    std::printf("  step        %.1f deg\n", cfg.step_deg);
    std::printf("  angle ref   %s\n",
                cfg.rel_frame ? "relative to frame" : "relative to horizon");
    std::cout << std::endl;
}

// ----------------------------------------------------------------- control --

// Tracks the commanded angle per logical control so relative moves and soft
// limits are meaningful. The board's own IMU reading is authoritative for
// where the gimbal actually is; this is only what we asked for.
struct State {
    double pan_deg = 0.0;
    double tilt_deg = 0.0;
    double roll_deg = 0.0;
};

double clamp_to(const AxisMap &m, double v, bool &clamped)
{
    double c = std::min(std::max(v, m.min_deg), m.max_deg);
    clamped = (c != v);
    return c;
}

// Push the current commanded angles out as one CMD_CONTROL.
bool send_angles(sbgc_t &sb, const Config &cfg, const State &st)
{
    double deg[SBGC_NUM_AXES]  = { 0.0, 0.0, 0.0 };
    double slew[SBGC_NUM_AXES] = { 0.0, 0.0, 0.0 };

    auto apply = [&](const AxisMap &m, double value) {
        deg[m.axis]  = m.invert ? -value : value;
        slew[m.axis] = cfg.speed_deg_s;
    };
    apply(cfg.pan,  st.pan_deg);
    apply(cfg.tilt, st.tilt_deg);
    apply(cfg.roll, st.roll_deg);

    if (sbgc_control_angle(&sb, deg, slew, cfg.rel_frame ? 1 : 0) != 0) {
        std::cout << "  send failed: " << sbgc_last_error(&sb) << "\n";
        return false;
    }
    return true;
}

// Continuous rate command on the logical controls.
bool send_rates(sbgc_t &sb, const Config &cfg,
                double pan_rate, double tilt_rate, double roll_rate)
{
    double rate[SBGC_NUM_AXES] = { 0.0, 0.0, 0.0 };
    rate[cfg.pan.axis]  = cfg.pan.invert  ? -pan_rate  : pan_rate;
    rate[cfg.tilt.axis] = cfg.tilt.invert ? -tilt_rate : tilt_rate;
    rate[cfg.roll.axis] = cfg.roll.invert ? -roll_rate : roll_rate;

    if (sbgc_control_speed(&sb, rate) != 0) {
        std::cout << "  send failed: " << sbgc_last_error(&sb) << "\n";
        return false;
    }
    return true;
}

void on_frame(uint8_t cmd_id, const uint8_t *payload, size_t len, void *user)
{
    (void)user;
    if (cmd_id == SBGC_CMD_GET_ANGLES) {
        sbgc_angles_t a;
        if (sbgc_parse_angles(payload, len, &a) == 0) {
            std::printf("  IMU    roll %7.2f  pitch %7.2f  yaw %7.2f\n",
                        a.imu_deg[SBGC_ROLL], a.imu_deg[SBGC_PITCH],
                        a.imu_deg[SBGC_YAW]);
            std::printf("  target roll %7.2f  pitch %7.2f  yaw %7.2f\n",
                        a.target_deg[SBGC_ROLL], a.target_deg[SBGC_PITCH],
                        a.target_deg[SBGC_YAW]);
        } else {
            std::printf("  CMD_GET_ANGLES payload was %zu bytes, expected 18 "
                        "— not parsed\n", len);
        }
    } else if (cmd_id == SBGC_CMD_ERROR) {
        std::printf("  board reported CMD_ERROR (%zu byte payload)\n", len);
    } else {
        std::printf("  rx cmd=%u len=%zu\n", cmd_id, len);
    }
}

// ---------------------------------------------------- keyboard debug mode --

/*
 * Full-screen keyboard mode. Built for debugging rather than operating: the
 * control legend stays on screen, and every redraw shows the exact bytes of
 * the last frame sent, so you can correlate a keypress with the wire traffic
 * without a second terminal or a logic analyser.
 *
 * Terminals report key *presses* but not releases, so a held-key deadman like
 * the gamepad's LB is not implementable here. Instead:
 *   - lowercase keys make discrete steps  (precise, repeatable, safe)
 *   - uppercase keys start a continuous rate
 *   - space stops, and an idle watchdog stops anyway after a few seconds
 * The watchdog is what keeps this fail-closed: a continuous rate cannot run
 * forever if you walk away from the keyboard.
 */

const double KB_IDLE_STOP_S = 5.0;

double monotonic_s()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) +
           static_cast<double>(ts.tv_nsec) * 1e-9;
}

// RAII raw-mode terminal so any return path restores the shell.
class RawTerm {
public:
    RawTerm() : active_(false)
    {
        if (!isatty(STDIN_FILENO)) return;
        if (tcgetattr(STDIN_FILENO, &saved_) != 0) return;
        struct termios raw = saved_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) active_ = true;
    }
    ~RawTerm()
    {
        if (active_) tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }
    RawTerm(const RawTerm &) = delete;
    RawTerm &operator=(const RawTerm &) = delete;
private:
    struct termios saved_;
    bool active_;
};

std::string kb_screen(const Config &cfg, const State &st, sbgc_t &sb,
                      double pan_rate, double tilt_rate, double roll_rate,
                      const std::string &note, double idle_left)
{
    char hex[3 * SBGC_MAX_FRAME + 1];
    sbgc_format_last_tx(&sb, hex, sizeof(hex));

    // Build the auto-stop notice into a real buffer. Passing .c_str() of a
    // temporary std::string straight into snprintf would dangle.
    char autostop[40] = "";
    if ((pan_rate != 0.0 || tilt_rate != 0.0 || roll_rate != 0.0) &&
        idle_left > 0.0) {
        std::snprintf(autostop, sizeof(autostop), "   auto-stop in %ds",
                      static_cast<int>(idle_left + 0.5));
    }

    char buf[4096];
    int n = std::snprintf(buf, sizeof(buf),
"\033[H\033[2J"
"+----------------------------------------------------------------------+\n"
"|  KEYBOARD DEBUG MODE                                                 |\n"
"+----------------------------------------------------------------------+\n"
"|  STEP (one move per press)          RATE (runs until stopped)        |\n"
"|    a / d   pan  left / right          A / D   pan  left / right      |\n"
"|    w / s   tilt up   / down           W / S   tilt up   / down       |\n"
"|    q / e   roll ccw  / cw             Q / E   roll ccw  / cw         |\n"
"|    arrow keys = a/d/w/s               SPACE   stop all motion        |\n"
"|                                                                      |\n"
"|  PRESETS                            TUNING                           |\n"
"|    h   home                           - / +   speed  -/+ 5 deg/s     |\n"
"|    l   level                          [ / ]   step   -/+ 1 deg       |\n"
"|    m   motors on/off                                                 |\n"
"|                                                                      |\n"
"|  r  read angles from board     x  re-send last frame                 |\n"
"|  ESC or Ctrl-C  exit back to the prompt                              |\n"
"+----------------------------------------------------------------------+\n"
"\n"
"  transport   %-12s %s\n"
"  angle ref   %s\n"
"  speed       %6.1f deg/s        step   %5.1f deg\n"
"\n"
"  estimate    pan %7.1f    tilt %7.1f    roll %7.1f   (deg)\n"
"  rate        pan %7.1f    tilt %7.1f    roll %7.1f   (deg/s)%s\n"
"  limits      pan [%.0f, %.0f]  tilt [%.0f, %.0f]  roll [%.0f, %.0f]\n"
"\n"
"  last tx     %s\n"
"\n"
"  %s\n",
        cfg.simulate ? "SIMULATED" : "serial",
        cfg.simulate ? "" : cfg.port.c_str(),
        cfg.rel_frame ? "relative to frame" : "relative to horizon",
        cfg.speed_deg_s, cfg.step_deg,
        st.pan_deg, st.tilt_deg, st.roll_deg,
        pan_rate, tilt_rate, roll_rate,
        autostop,
        cfg.pan.min_deg, cfg.pan.max_deg,
        cfg.tilt.min_deg, cfg.tilt.max_deg,
        cfg.roll.min_deg, cfg.roll.max_deg,
        hex[0] ? hex : "(nothing sent yet)",
        note.c_str());

    /*
     * snprintf returns the length it WOULD have written, so on truncation n
     * exceeds the buffer and constructing a string of that length reads past
     * it. cfg.port is operator-supplied and uncapped, so this is reachable.
     */
    size_t used = (n < 0) ? 0u : static_cast<size_t>(n);
    if (used >= sizeof(buf)) used = sizeof(buf) - 1;
    return std::string(buf, used);
}

void keyboard_mode(sbgc_t &sb, Config &cfg, State &st)
{
    if (!isatty(STDIN_FILENO)) {
        std::cout << "  keyboard mode needs a terminal.\n";
        return;
    }

    RawTerm raw;
    sbgc_set_quiet(&sb, 1);          // never printf over the HUD

    double pan_rate = 0.0, tilt_rate = 0.0, roll_rate = 0.0;
    std::string note = "ready";
    bool running = true;
    bool dirty = true;

    double last_t = monotonic_s();
    double last_key = last_t;
    double last_draw = 0.0;

    auto stop_all = [&]() {
        pan_rate = tilt_rate = roll_rate = 0.0;
        sbgc_stop(&sb);
    };

    auto step = [&](double dpan, double dtilt, double droll) {
        bool cl = false, any = false;
        st.pan_deg  = clamp_to(cfg.pan,  st.pan_deg  + dpan,  cl); any |= cl;
        st.tilt_deg = clamp_to(cfg.tilt, st.tilt_deg + dtilt, cl); any |= cl;
        st.roll_deg = clamp_to(cfg.roll, st.roll_deg + droll, cl); any |= cl;
        send_angles(sb, cfg, st);
        note = any ? "step (clamped at soft limit)" : "step";
    };

    while (running) {
        double now = monotonic_s();
        double dt = now - last_t;
        last_t = now;
        if (dt > 0.5) dt = 0.5;

        // ---- read every pending key ----
        unsigned char c;
        while (read(STDIN_FILENO, &c, 1) == 1) {
            dirty = true;
            last_key = now;

            if (c == 0x1B) {            // ESC, possibly an arrow sequence
                unsigned char b1, b2;
                if (read(STDIN_FILENO, &b1, 1) != 1) { running = false; break; }
                if (b1 != '[') { running = false; break; }
                if (read(STDIN_FILENO, &b2, 1) != 1) { running = false; break; }
                switch (b2) {
                    case 'A': step(0,  cfg.step_deg, 0); break;
                    case 'B': step(0, -cfg.step_deg, 0); break;
                    case 'C': step( cfg.step_deg, 0, 0); break;
                    case 'D': step(-cfg.step_deg, 0, 0); break;
                    default: break;
                }
                continue;
            }
            if (c == 0x03) { running = false; break; }   // Ctrl-C

            switch (c) {
                // discrete steps
                case 'a': step(-cfg.step_deg, 0, 0); break;
                case 'd': step( cfg.step_deg, 0, 0); break;
                case 'w': step(0,  cfg.step_deg, 0); break;
                case 's': step(0, -cfg.step_deg, 0); break;
                case 'q': step(0, 0, -cfg.step_deg); break;
                case 'e': step(0, 0,  cfg.step_deg); break;

                // continuous rates
                case 'A': pan_rate  = -cfg.speed_deg_s; note = "rate: pan left";  break;
                case 'D': pan_rate  =  cfg.speed_deg_s; note = "rate: pan right"; break;
                case 'W': tilt_rate =  cfg.speed_deg_s; note = "rate: tilt up";   break;
                case 'S': tilt_rate = -cfg.speed_deg_s; note = "rate: tilt down"; break;
                case 'Q': roll_rate = -cfg.speed_deg_s * cfg.roll_scale;
                          note = "rate: roll ccw"; break;
                case 'E': roll_rate =  cfg.speed_deg_s * cfg.roll_scale;
                          note = "rate: roll cw";  break;

                case ' ': stop_all(); note = "stopped"; break;

                // presets
                case 'h': stop_all(); sbgc_home(&sb);  st = State{}; note = "home";  break;
                case 'l': stop_all(); sbgc_level(&sb); st = State{}; note = "level"; break;
                case 'm': {
                    static bool on = false;
                    on = !on;
                    if (on) sbgc_motors_on(&sb); else sbgc_motors_off(&sb);
                    note = on ? "motors ON" : "motors OFF";
                    break;
                }

                // tuning
                case '-': case '_':
                    cfg.speed_deg_s = std::max(1.0, cfg.speed_deg_s - 5.0);
                    note = "speed down"; break;
                case '=': case '+':
                    cfg.speed_deg_s = std::min(200.0, cfg.speed_deg_s + 5.0);
                    note = "speed up"; break;
                case '[':
                    cfg.step_deg = std::max(0.5, cfg.step_deg - 1.0);
                    note = "step down"; break;
                case ']':
                    cfg.step_deg = std::min(90.0, cfg.step_deg + 1.0);
                    note = "step up"; break;

                case 'r':
                    sbgc_request_angles(&sb);
                    note = "requested angles (see 'read' at the prompt for output)";
                    break;
                case 'x':
                    note = "last frame shown below";
                    break;
                default:
                    break;
            }
        }
        if (!running) break;

        // ---- idle watchdog: a rate must not outlive your attention ----
        double idle = now - last_key;
        double idle_left = KB_IDLE_STOP_S - idle;
        bool moving = (pan_rate != 0.0 || tilt_rate != 0.0 || roll_rate != 0.0);
        if (moving && idle_left <= 0.0) {
            stop_all();
            note = "auto-stopped (no key for 5 s)";
            dirty = true;
            moving = false;
        }

        // ---- publish rate and integrate the estimate ----
        if (moving) {
            auto gate = [&](const AxisMap &m, double &pos, double &rate) {
                double next = pos + rate * dt;
                if (next > m.max_deg && rate > 0) { rate = 0.0; next = m.max_deg; }
                if (next < m.min_deg && rate < 0) { rate = 0.0; next = m.min_deg; }
                pos = next;
            };
            gate(cfg.pan,  st.pan_deg,  pan_rate);
            gate(cfg.tilt, st.tilt_deg, tilt_rate);
            gate(cfg.roll, st.roll_deg, roll_rate);
            send_rates(sb, cfg, pan_rate, tilt_rate, roll_rate);
            dirty = true;
        }

        // ---- redraw at ~15 Hz ----
        if (dirty && now - last_draw > 1.0 / 15.0) {
            std::string s = kb_screen(cfg, st, sb, pan_rate, tilt_rate,
                                      roll_rate, note, idle_left);
            ssize_t ignored = write(STDOUT_FILENO, s.data(), s.size());
            (void)ignored;
            last_draw = now;
            dirty = false;
        }

        usleep(10000);   // 10 ms; keeps the loop near 30 Hz of real work
    }

    stop_all();
    sbgc_set_quiet(&sb, 0);
    std::cout << "\n  Left keyboard mode (gimbal stopped).\n";
}

// --------------------------------------------------------- live key mode --

// Raw-mode arrow keys. Each press nudges by cfg.step_deg; 'q' exits.
void live_key_mode(sbgc_t &sb, const Config &cfg, State &st)
{
    if (!isatty(STDIN_FILENO)) {
        std::cout << "  live mode needs a terminal.\n";
        return;
    }

    struct termios saved;
    if (tcgetattr(STDIN_FILENO, &saved) != 0) {
        std::cout << "  cannot set raw mode.\n";
        return;
    }
    struct termios raw = saved;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN]  = 1;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    std::cout << "\n  Live mode. Arrows = pan/tilt, [ ] = roll,\n"
                 "  h = home, space = stop, q = quit back to the prompt.\n";

    bool running = true;
    while (running) {
        unsigned char c;
        if (read(STDIN_FILENO, &c, 1) != 1) break;

        double dpan = 0.0, dtilt = 0.0, droll = 0.0;

        if (c == 0x1B) {                    // escape sequence
            unsigned char b1, b2;
            if (read(STDIN_FILENO, &b1, 1) != 1) break;
            if (b1 != '[') continue;
            if (read(STDIN_FILENO, &b2, 1) != 1) break;
            switch (b2) {
                case 'A': dtilt =  cfg.step_deg; break;   // up
                case 'B': dtilt = -cfg.step_deg; break;   // down
                case 'C': dpan  =  cfg.step_deg; break;   // right
                case 'D': dpan  = -cfg.step_deg; break;   // left
                default: continue;
            }
        } else {
            switch (c) {
                case 'q': running = false; continue;
                case ' ': sbgc_stop(&sb);  continue;
                case 'h': sbgc_home(&sb);
                          st = State{};
                          std::printf("\r  home\n");
                          continue;
                case '[': droll = -cfg.step_deg; break;
                case ']': droll =  cfg.step_deg; break;
                default: continue;
            }
        }

        bool cl = false, any_clamp = false;
        st.pan_deg  = clamp_to(cfg.pan,  st.pan_deg  + dpan,  cl); any_clamp |= cl;
        st.tilt_deg = clamp_to(cfg.tilt, st.tilt_deg + dtilt, cl); any_clamp |= cl;
        st.roll_deg = clamp_to(cfg.roll, st.roll_deg + droll, cl); any_clamp |= cl;

        send_angles(sb, cfg, st);
        std::printf("\r  pan %7.1f  tilt %7.1f  roll %7.1f %s   ",
                    st.pan_deg, st.tilt_deg, st.roll_deg,
                    any_clamp ? "(limit)" : "       ");
        std::fflush(stdout);
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &saved);
    std::cout << "\n  Left live mode.\n";
}

// -------------------------------------------------------------- gamepad --

/*
 * Gamepad rate control at a fixed 30 Hz.
 *
 * Deadman: nothing moves unless LB is held. On release — or if the pad is
 * unplugged, or the loop is exited by any path — a zero-rate command goes out
 * once and the loop stops publishing. This is fail-closed: losing the
 * controller stops the gimbal rather than leaving the last rate running.
 *
 * Soft limits here are open-loop. In rate mode the board integrates internally
 * and this tool only estimates where the camera ended up, so the estimate can
 * drift from truth over a long session. The board's own limits remain the
 * authoritative protection; press A (home) to resynchronise the estimate.
 */
void gamepad_mode(sbgc_t &sb, Config &cfg, State &st)
{
    gp_t gp;
    int rc = cfg.pad_path.empty() ? gp_open_auto(&gp)
                                  : gp_open(&gp, cfg.pad_path.c_str());
    if (rc != 0) {
        std::cout << "  " << gp_last_error(&gp) << "\n";
        return;
    }

    std::cout << "\n  Gamepad: " << gp_name(&gp) << "  (" << gp.path << ")\n"
              << "    HOLD LB to move — nothing happens without it\n"
              << "    left stick   pan / tilt (full speed)\n"
              << "    right stick  pan / tilt (fine, " << (int)(cfg.fine_scale * 100)
              << "%)\n"
              << "    LT / RT      roll counter-clockwise / clockwise\n"
              << "    RB           boost x" << cfg.boost << " while held\n"
              << "    A home   B level   X slower   Y faster\n"
              << "    Back or q    exit gamepad mode\n\n";

    RawTerm raw;   // restores the terminal however we leave this function
    const bool stdin_is_tty = isatty(STDIN_FILENO);

    const double period = 1.0 / 30.0;
    bool running = true;
    bool active = false;        // LB currently held and rates being sent
    double last_tx = 0.0;
    double last_t = monotonic_s();

    while (running) {
        int pr = gp_poll(&gp, 10);
        if (pr < 0) {
            std::cout << "\n  " << gp_last_error(&gp) << " — stopping.\n";
            sbgc_stop(&sb);
            break;
        }

        /*
         * 'q' on the keyboard also exits, for when the pad is out of reach.
         *
         * Guarded by isatty, as keyboard_mode and live_key_mode already are.
         * RawTerm only sets VMIN=0 when stdin is a terminal, so on a pipe or
         * a redirected stdin this read() blocks forever — gp_poll() would
         * never run again and the deadman-release stop would never be sent,
         * leaving the board in SPEED mode at the last rate.
         */
        if (stdin_is_tty) {
            unsigned char key;
            while (read(STDIN_FILENO, &key, 1) == 1) {
                if (key == 'q' || key == 0x03 /* ^C */) running = false;
            }
        }
        if (!running) break;

        double now = monotonic_s();
        double dt = now - last_t;
        last_t = now;
        if (dt > 0.5) dt = 0.5;         // ignore stalls (e.g. suspend)

        // --- edge-triggered buttons ---
        if (gp_button_pressed(&gp, GP_BTN_BACK)) running = false;
        if (gp_button_pressed(&gp, GP_BTN_A)) {
            sbgc_home(&sb);
            st = State{};
            std::printf("\r  home                                        \n");
        }
        if (gp_button_pressed(&gp, GP_BTN_B)) {
            sbgc_level(&sb);
            st = State{};
            std::printf("\r  level                                       \n");
        }
        if (gp_button_pressed(&gp, GP_BTN_X)) {
            cfg.speed_deg_s = std::max(1.0, cfg.speed_deg_s - 5.0);
            std::printf("\r  speed %.0f deg/s                            \n",
                        cfg.speed_deg_s);
        }
        if (gp_button_pressed(&gp, GP_BTN_Y)) {
            cfg.speed_deg_s = std::min(200.0, cfg.speed_deg_s + 5.0);
            std::printf("\r  speed %.0f deg/s                            \n",
                        cfg.speed_deg_s);
        }
        gp_latch_buttons(&gp);
        if (!running) break;

        // --- deadman ---
        bool held = gp_button(&gp, GP_BTN_LB) != 0;
        if (!held) {
            if (active) {          // just released: one stop, then stay quiet
                sbgc_stop(&sb);
                active = false;
                std::printf("\r  [deadman released — stopped]                \n");
            }
            continue;
        }
        if (!active) {
            active = true;
            std::printf("\r  [deadman held]                              \n");
        }

        if (now - last_tx < period) continue;
        last_tx = now;

        // --- sticks and triggers to rates ---
        double lx = gp_axis_signed(&gp, GP_AX_LEFT_X,  cfg.deadzone);
        double ly = gp_axis_signed(&gp, GP_AX_LEFT_Y,  cfg.deadzone);
        double rx = gp_axis_signed(&gp, GP_AX_RIGHT_X, cfg.deadzone);
        double ry = gp_axis_signed(&gp, GP_AX_RIGHT_Y, cfg.deadzone);
        double lt = gp_axis_unit(&gp, GP_AX_LTRIGGER);
        double rt = gp_axis_unit(&gp, GP_AX_RTRIGGER);

        double scale = cfg.speed_deg_s *
                       (gp_button(&gp, GP_BTN_RB) ? cfg.boost : 1.0);

        // evdev sticks report Y positive downwards; negate so up is positive.
        double pan  = (lx + rx * cfg.fine_scale) * scale;
        double tilt = (-ly + -ry * cfg.fine_scale) * scale;
        double roll = (rt - lt) * scale * cfg.roll_scale;

        // --- open-loop soft-limit gating ---
        auto gate = [&](const AxisMap &m, double &pos, double &rate) {
            double next = pos + rate * dt;
            if (next > m.max_deg && rate > 0) { rate = 0.0; next = m.max_deg; }
            if (next < m.min_deg && rate < 0) { rate = 0.0; next = m.min_deg; }
            pos = next;
        };
        gate(cfg.pan,  st.pan_deg,  pan);
        gate(cfg.tilt, st.tilt_deg, tilt);
        gate(cfg.roll, st.roll_deg, roll);

        if (!send_rates(sb, cfg, pan, tilt, roll)) break;

        std::printf("\r  pan %6.1f/s  tilt %6.1f/s  roll %6.1f/s   "
                    "est %6.1f %6.1f %6.1f  ",
                    pan, tilt, roll, st.pan_deg, st.tilt_deg, st.roll_deg);
        std::fflush(stdout);
    }

    sbgc_stop(&sb);
    gp_close(&gp);
    std::cout << "\n  Left gamepad mode (gimbal stopped).\n";
}

// --------------------------------------------------------- command table --

/*
 * Every command is declared once, here, with its own explanation. The table
 * drives the help output, argument validation and the did-you-mean suggestion,
 * so those three can never drift apart from each other or from reality.
 *
 * max_args of -1 means "unlimited".
 */
struct Cmd {
    const char *name;
    const char *alias;     // nullptr if none
    const char *group;
    const char *usage;
    const char *summary;   // one line, shown in the command list
    const char *detail;    // full explanation, shown by 'help <cmd>'
    int min_args;          // not counting the command word itself
    int max_args;
};

const Cmd COMMANDS[] = {
{"left", nullptr, "Direction", "left [deg/s]",
 "Pan left continuously until 'stop'.",
 "Starts a continuous pan to the left at the given rate, or at the current\n"
 "default speed if no rate is given. The gimbal keeps turning until you type\n"
 "'stop', hit a soft limit, or issue an angle command. This is rate control:\n"
 "the board holds position once the rate returns to zero.", 0, 1},

{"right", nullptr, "Direction", "right [deg/s]",
 "Pan right continuously until 'stop'.",
 "Mirror of 'left'. See 'help left'.", 0, 1},

{"up", nullptr, "Direction", "up [deg/s]",
 "Tilt up continuously until 'stop'.",
 "Starts a continuous upward tilt. Watch your tilt soft limit - the default\n"
 "upper bound is deliberately small because most mounts run out of travel\n"
 "sooner going up than going down.", 0, 1},

{"down", nullptr, "Direction", "down [deg/s]",
 "Tilt down continuously until 'stop'.",
 "Mirror of 'up'. See 'help up'.", 0, 1},

{"cw", nullptr, "Direction", "cw [deg/s]",
 "Roll clockwise continuously until 'stop'.",
 "Rolls the camera clockwise. Roll is rarely wanted deliberately on a patrol\n"
 "robot; if the horizon looks tilted, the fix is usually 'level', not this.", 0, 1},

{"ccw", nullptr, "Direction", "ccw [deg/s]",
 "Roll counter-clockwise continuously until 'stop'.",
 "Mirror of 'cw'. See 'help cw'.", 0, 1},

{"stop", nullptr, "Direction", "stop",
 "Zero the rate on every axis.",
 "Sends a speed-mode command with rate zero on all three axes. This is an\n"
 "explicit hold, not a release: the gimbal actively keeps its current\n"
 "orientation rather than going limp. Use 'motors off' to actually release.", 0, 0},

{"pan", nullptr, "Angle", "pan <deg>",
 "Move to an absolute pan angle.",
 "Commands an absolute pan angle in degrees, clamped to the pan soft limits\n"
 "from setup. Whether the angle is measured against the robot frame or the\n"
 "gravity-stabilised horizon depends on the angle reference you chose in\n"
 "setup; 'show' displays which is active.", 1, 1},

{"tilt", nullptr, "Angle", "tilt <deg>",
 "Move to an absolute tilt angle.",
 "As 'pan', for the tilt axis. Positive is up unless you inverted tilt during\n"
 "setup.", 1, 1},

{"roll", nullptr, "Angle", "roll <deg>",
 "Move to an absolute roll angle.",
 "As 'pan', for the roll axis. Positive is clockwise unless inverted.", 1, 1},

{"goto", nullptr, "Angle", "goto <pan> <tilt> [roll]",
 "Move all axes to absolute angles at once.",
 "Sets every axis in a single command frame, so the axes start moving\n"
 "together instead of one command after another. Roll is optional and stays\n"
 "where it is if omitted. All values are clamped to their soft limits.", 2, 3},

{"nudge", nullptr, "Angle", "nudge <pan> <tilt> [roll]",
 "Offset the current angles by a relative amount.",
 "Like 'goto', but the numbers are added to the current commanded position\n"
 "rather than replacing it. Small nudges are the safest way to confirm an\n"
 "axis moves the direction you expect before trusting the gamepad.", 2, 3},

{"home", nullptr, "Angle", "home",
 "Roll level, pitch and yaw neutral to the frame.",
 "Sends the vendor's documented home frame: roll to the horizon, pitch and\n"
 "yaw to neutral relative to the robot frame, as an auto-task. Also resets\n"
 "this tool's internal position estimate, so it is the way to resynchronise\n"
 "after a long session of rate control has let the estimate drift.", 0, 0},

{"level", nullptr, "Angle", "level",
 "Roll and pitch to the horizon, yaw neutral.",
 "Like 'home' but pitch also goes to the horizon rather than to frame-neutral.\n"
 "Use this when the robot is on a slope and you want the camera level with\n"
 "the world instead of with the chassis.", 0, 0},

{"speed", nullptr, "Tuning", "speed <deg/s>",
 "Set the default rate and angle slew speed.",
 "Sets both the rate used by the direction commands and the slew speed used\n"
 "when moving to an absolute angle. Must be greater than zero. Start slow -\n"
 "30 deg/s is unhurried, 90 is brisk, and anything past 150 is hard to aim.", 1, 1},

{"step", nullptr, "Tuning", "step <deg>",
 "Set the per-keypress increment.",
 "Controls how far one keypress moves the gimbal in 'keys' and 'live' modes.\n"
 "Small steps (2-5 deg) are best for verifying directions and limits; larger\n"
 "steps are for getting somewhere quickly.", 1, 1},

{"keys", "kb", "Modes", "keys",
 "Keyboard debug mode with a full-screen HUD.",
 "Full-screen mode showing the control legend, live position estimate, and\n"
 "the exact bytes of the last frame sent. Lowercase keys make discrete steps,\n"
 "uppercase start continuous rates, SPACE stops. A continuous rate auto-stops\n"
 "after 5 seconds without a keypress. This is the mode to use when something\n"
 "is behaving oddly and you need to see the wire traffic.", 0, 0},

{"pad", nullptr, "Modes", "pad [device]",
 "Xbox/gamepad control; hold LB to move.",
 "Opens a gamepad and enters 30 Hz rate control. Nothing moves unless LB is\n"
 "held. Left stick pans and tilts at full speed, right stick does the same at\n"
 "25% for fine framing, triggers roll, RB boosts. Auto-detects the pad unless\n"
 "you pass an event device path such as /dev/input/event5.", 0, 1},

{"pads", nullptr, "Modes", "pads",
 "List detected gamepads.",
 "Scans /dev/input for anything that looks like a gamepad and reports what it\n"
 "found. If this comes up empty with a controller plugged in, the usual cause\n"
 "is device permissions: add yourself to the 'input' group and log back in.", 0, 0},

{"live", nullptr, "Modes", "live",
 "Minimal arrow-key control.",
 "A stripped-down arrow-key mode with no HUD. 'keys' supersedes it and is\n"
 "better for almost everything; this remains for a quick nudge without the\n"
 "screen being cleared.", 0, 0},

{"read", nullptr, "Status", "read",
 "Ask the board for its actual angles.",
 "Requests CMD_GET_ANGLES and prints the IMU angles and target angles the\n"
 "board reports. This is ground truth, unlike the position estimate shown\n"
 "elsewhere, which is dead reckoning from the rates this tool has commanded.", 0, 0},

{"motors", nullptr, "Status", "motors on|off",
 "Power the gimbal motors on or off.",
 "Motors off releases the axes and the camera goes limp - support it before\n"
 "doing this if it can fall against a stop. Motors must be on for any motion\n"
 "command to have an effect, which is a common reason for 'nothing happened'.", 1, 1},

{"show", nullptr, "Status", "show",
 "Print the current settings.",
 "Displays the port, the axis mapping and inversion flags, the soft limits,\n"
 "the speed and step values, and which angle reference is active.", 0, 0},

{"setup", nullptr, "Status", "setup",
 "Re-run the orientation wizard.",
 "Walks through the mount description again: which gimbal axis drives each\n"
 "control, which direction is positive, soft limits, and motion defaults.\n"
 "Nothing moves during setup. Run this whenever the camera goes the wrong way\n"
 "rather than compensating in your head.", 0, 0},

{"help", "?", "Status", "help [command]",
 "List commands, or explain one in detail.",
 "With no argument, lists every command with a one-line summary. With a\n"
 "command name, prints that command's usage and full explanation.", 0, 1},

{"quit", "exit", "Status", "quit",
 "Stop the gimbal and exit.",
 "Sends a stop command, closes the serial port and exits. The gimbal keeps\n"
 "its last position with motors still powered; use 'motors off' first if you\n"
 "want it released.", 0, 0},
};

const size_t N_COMMANDS = sizeof(COMMANDS) / sizeof(COMMANDS[0]);

const Cmd *find_cmd(const std::string &name)
{
    for (size_t i = 0; i < N_COMMANDS; i++) {
        if (name == COMMANDS[i].name) return &COMMANDS[i];
        if (COMMANDS[i].alias && name == COMMANDS[i].alias) return &COMMANDS[i];
    }
    return nullptr;
}

// ------------------------------------------------------------ did you mean --

int edit_distance(const std::string &a, const std::string &b)
{
    const size_t n = a.size(), m = b.size();
    std::vector<int> prev(m + 1), cur(m + 1);
    for (size_t j = 0; j <= m; j++) prev[j] = static_cast<int>(j);

    for (size_t i = 1; i <= n; i++) {
        cur[0] = static_cast<int>(i);
        for (size_t j = 1; j <= m; j++) {
            int cost = (a[i - 1] == b[j - 1]) ? 0 : 1;
            cur[j] = std::min({ prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + cost });
        }
        prev = cur;
    }
    return prev[m];
}

/*
 * Collect plausible corrections for an unrecognised word: anything within a
 * small edit distance, plus anything the word is a prefix of. The distance
 * budget scales with length so short commands do not match everything.
 */
std::vector<std::string> suggestions(const std::string &word)
{
    std::vector<std::string> hits;
    const int budget = std::max(1, static_cast<int>(word.size()) / 3 + 1);

    for (size_t i = 0; i < N_COMMANDS; i++) {
        const std::string name = COMMANDS[i].name;
        bool prefix = name.size() >= word.size() &&
                      name.compare(0, word.size(), word) == 0;
        if (prefix || edit_distance(word, name) <= budget)
            hits.push_back(name);
    }
    return hits;
}

void print_suggestions(const std::string &word)
{
    std::vector<std::string> s = suggestions(word);
    if (s.empty()) return;
    std::cout << "  did you mean: ";
    for (size_t i = 0; i < s.size() && i < 4; i++)
        std::cout << (i ? ", " : "") << s[i];
    std::cout << " ?\n";
}

// ------------------------------------------------------------------ errors --

void err(const std::string &msg)
{
    std::cout << "  error: " << msg << "\n";
}

// Print the offending line with a caret under the bad token.
void err_at(const std::vector<std::string> &tok, size_t bad, const std::string &msg)
{
    std::string line;
    size_t col = 0;
    for (size_t i = 0; i < tok.size(); i++) {
        if (i == bad) col = line.size();
        line += tok[i];
        if (i + 1 < tok.size()) line += ' ';
    }
    size_t width = (bad < tok.size() && !tok[bad].empty()) ? tok[bad].size() : 1;
    std::cout << "  error: " << msg << "\n"
              << "    " << line << "\n"
              << "    " << std::string(col, ' ')
              << std::string(width, '^') << "\n";
}

void show_usage(const Cmd &c)
{
    std::cout << "  usage: " << c.usage << "\n"
              << "         " << c.summary << "\n"
              << "  type 'help " << c.name << "' for the full explanation\n";
}

// ------------------------------------------------------------------- help --

void print_help_for(const Cmd &c)
{
    std::cout << "\n  " << c.name;
    if (c.alias) std::cout << "   (alias: " << c.alias << ")";
    std::cout << "\n  usage: " << c.usage << "\n\n";

    std::istringstream is(std::string(c.detail));
    std::string ln;
    while (std::getline(is, ln)) std::cout << "    " << ln << "\n";
    std::cout << "\n";
}

void print_help()
{
    std::cout << "\nCommands   (type 'help <command>' for a full explanation)\n";

    const char *groups[] = { "Direction", "Angle", "Tuning", "Modes", "Status" };
    const char *notes[]  = {
        "continuous motion, runs until stopped",
        "absolute or relative positioning, clamped to soft limits",
        "defaults used by the commands above",
        "interactive control modes",
        "state, configuration and exit"
    };

    for (size_t g = 0; g < sizeof(groups) / sizeof(groups[0]); g++) {
        std::cout << "\n  " << groups[g] << " - " << notes[g] << "\n";
        for (size_t i = 0; i < N_COMMANDS; i++) {
            if (std::string(COMMANDS[i].group) != groups[g]) continue;
            std::printf("    %-26s %s\n", COMMANDS[i].usage, COMMANDS[i].summary);
        }
    }
    std::cout << "\n";
}

// ---------------------------------------------------------------- dispatch --

/*
 * Argument helpers. Each reports a specific, actionable error rather than
 * silently substituting a default - mistyping a number on a machine that moves
 * should never be indistinguishable from not typing one.
 */
bool need_num(const std::vector<std::string> &tok, size_t i,
              const Cmd &c, double &out)
{
    if (i >= tok.size()) {
        err(std::string("'") + c.name + "' is missing an argument");
        show_usage(c);
        return false;
    }
    if (!parse_double(tok[i], out)) {
        err_at(tok, i, "'" + tok[i] + "' is not a number");
        show_usage(c);
        return false;
    }
    return true;
}

bool opt_num(const std::vector<std::string> &tok, size_t i,
             const Cmd &c, double def, double &out)
{
    if (i >= tok.size()) { out = def; return true; }
    if (!parse_double(tok[i], out)) {
        err_at(tok, i, "'" + tok[i] + "' is not a number");
        show_usage(c);
        return false;
    }
    return true;
}

bool check_arity(const std::vector<std::string> &tok, const Cmd &c)
{
    int n = static_cast<int>(tok.size()) - 1;
    if (n < c.min_args) {
        err(std::string("'") + c.name + "' needs at least " +
            std::to_string(c.min_args) + " argument" +
            (c.min_args == 1 ? "" : "s") + ", got " + std::to_string(n));
        show_usage(c);
        return false;
    }
    if (c.max_args >= 0 && n > c.max_args) {
        err_at(tok, static_cast<size_t>(c.max_args) + 1,
               std::string("'") + c.name + "' takes at most " +
               std::to_string(c.max_args) + " argument" +
               (c.max_args == 1 ? "" : "s") + ", got " + std::to_string(n));
        show_usage(c);
        return false;
    }
    return true;
}

bool handle(const std::vector<std::string> &tok, sbgc_t &sb,
            Config &cfg, State &st, bool &quit)
{
    const std::string &word = tok[0];

    // --- unknown command: refuse and suggest, never carry on silently ---
    const Cmd *cmd = find_cmd(word);
    if (!cmd) {
        err("unknown command '" + word + "'");
        print_suggestions(word);
        std::cout << "  type 'help' to list every command\n";
        return true;
    }

    if (!check_arity(tok, *cmd)) return true;

    const std::string c = cmd->name;   // canonical name, aliases resolved

    if (c == "quit") {
        sbgc_stop(&sb);
        quit = true;
        return true;
    }

    if (c == "help") {
        if (tok.size() == 1) { print_help(); return true; }
        const Cmd *t = find_cmd(tok[1]);
        if (!t) {
            err("no such command '" + tok[1] + "'");
            print_suggestions(tok[1]);
            return true;
        }
        print_help_for(*t);
        return true;
    }

    if (c == "show")  { show_config(cfg); return true; }
    if (c == "setup") { run_setup(cfg); show_config(cfg); return true; }
    if (c == "live")  { live_key_mode(sb, cfg, st); return true; }
    if (c == "keys")  { keyboard_mode(sb, cfg, st); return true; }

    if (c == "pad") {
        if (tok.size() > 1) cfg.pad_path = tok[1];
        gamepad_mode(sb, cfg, st);
        return true;
    }

    if (c == "pads") {
        char path[64];
        if (gp_find(path, sizeof(path)) == 0) {
            gp_t probe;
            if (gp_open(&probe, path) == 0) {
                std::cout << "  found: " << path << "  " << gp_name(&probe) << "\n";
                gp_close(&probe);
            } else {
                std::cout << "  found " << path << " but "
                          << gp_last_error(&probe) << "\n";
            }
        } else {
            std::cout << "  no gamepad detected under /dev/input\n"
                      << "  plug one in, then check: ls -l /dev/input/event*\n";
        }
        return true;
    }

    // --- tuning ---
    if (c == "speed") {
        double v;
        if (!need_num(tok, 1, *cmd, v)) return true;
        if (v <= 0.0) {
            err_at(tok, 1, "speed must be greater than 0");
            std::cout << "  (use 'stop' to halt, not a speed of 0)\n";
            return true;
        }
        if (v > 500.0) {
            err_at(tok, 1, "speed " + tok[1] + " deg/s is implausibly high");
            std::cout << "  refusing; typical values are 20-120 deg/s\n";
            return true;
        }
        cfg.speed_deg_s = v;
        std::printf("  speed = %.1f deg/s\n", cfg.speed_deg_s);
        return true;
    }

    if (c == "step") {
        double v;
        if (!need_num(tok, 1, *cmd, v)) return true;
        if (v <= 0.0) {
            err_at(tok, 1, "step must be greater than 0");
            return true;
        }
        if (v > 90.0) {
            err_at(tok, 1, "step " + tok[1] + " deg is too large");
            std::cout << "  refusing; a keypress should not swing more than 90 deg\n";
            return true;
        }
        cfg.step_deg = v;
        std::printf("  step = %.1f deg\n", cfg.step_deg);
        return true;
    }

    // --- direction: continuous rate ---
    if (c == "left" || c == "right" || c == "up" || c == "down" ||
        c == "cw"   || c == "ccw") {
        double r;
        if (!opt_num(tok, 1, *cmd, cfg.speed_deg_s, r)) return true;
        if (r <= 0.0) {
            err_at(tok, 1, "rate must be greater than 0");
            std::cout << "  (direction comes from the command, not the sign)\n";
            return true;
        }
        if (r > 500.0) {
            err_at(tok, 1, "rate " + tok[1] + " deg/s is implausibly high");
            return true;
        }
        double p = 0, t = 0, ro = 0;
        if      (c == "right") p =  r;
        else if (c == "left")  p = -r;
        else if (c == "up")    t =  r;
        else if (c == "down")  t = -r;
        else if (c == "cw")    ro =  r;
        else                   ro = -r;
        if (send_rates(sb, cfg, p, t, ro))
            std::printf("  %s at %.1f deg/s - 'stop' to halt\n", c.c_str(), r);
        return true;
    }

    if (c == "stop") {
        if (sbgc_stop(&sb) == 0) std::cout << "  stopped\n";
        else err(sbgc_last_error(&sb));
        return true;
    }

    // --- absolute angle, single axis ---
    if (c == "pan" || c == "tilt" || c == "roll") {
        double v;
        if (!need_num(tok, 1, *cmd, v)) return true;

        const AxisMap &m = (c == "pan") ? cfg.pan
                         : (c == "tilt") ? cfg.tilt : cfg.roll;
        bool cl = false;
        double clamped = clamp_to(m, v, cl);
        if (cl) {
            std::printf("  note: %.1f is outside the %s limits [%.1f, %.1f]; "
                        "clamped to %.1f\n",
                        v, c.c_str(), m.min_deg, m.max_deg, clamped);
        }
        if (c == "pan")       st.pan_deg  = clamped;
        else if (c == "tilt") st.tilt_deg = clamped;
        else                  st.roll_deg = clamped;

        if (send_angles(sb, cfg, st))
            std::printf("  pan %.1f  tilt %.1f  roll %.1f\n",
                        st.pan_deg, st.tilt_deg, st.roll_deg);
        return true;
    }

    // --- absolute or relative, all axes ---
    if (c == "goto" || c == "nudge") {
        bool rel = (c == "nudge");
        double p, t, r;
        if (!need_num(tok, 1, *cmd, p)) return true;
        if (!need_num(tok, 2, *cmd, t)) return true;
        if (!opt_num(tok, 3, *cmd, rel ? 0.0 : st.roll_deg, r)) return true;

        bool cl = false, any = false;
        st.pan_deg  = clamp_to(cfg.pan,  rel ? st.pan_deg  + p : p, cl); any |= cl;
        st.tilt_deg = clamp_to(cfg.tilt, rel ? st.tilt_deg + t : t, cl); any |= cl;
        st.roll_deg = clamp_to(cfg.roll, rel ? st.roll_deg + r : r, cl); any |= cl;
        if (any) std::cout << "  note: one or more axes clamped to a soft limit\n";

        if (send_angles(sb, cfg, st))
            std::printf("  pan %.1f  tilt %.1f  roll %.1f\n",
                        st.pan_deg, st.tilt_deg, st.roll_deg);
        return true;
    }

    if (c == "home") {
        if (sbgc_home(&sb) == 0) { st = State{}; std::cout << "  home\n"; }
        else err(sbgc_last_error(&sb));
        return true;
    }

    if (c == "level") {
        if (sbgc_level(&sb) == 0) { st = State{}; std::cout << "  level\n"; }
        else err(sbgc_last_error(&sb));
        return true;
    }

    // --- status ---
    if (c == "read") {
        if (sbgc_request_angles(&sb) != 0) {
            err(sbgc_last_error(&sb));
        } else if (sbgc_poll(&sb, 300, on_frame, nullptr) == 0) {
            if (sb.simulate) {
                std::cout << "  no response (simulated - nothing is connected)\n";
            } else {
                err("no response within 300 ms");
                std::cout << "  the board may be powered off, on a different "
                             "baud rate, or held by the SimpleBGC GUI\n";
            }
        }
        return true;
    }

    if (c == "motors") {
        const std::string &a = tok[1];
        if (a != "on" && a != "off") {
            err_at(tok, 1, "expected 'on' or 'off', got '" + a + "'");
            show_usage(*cmd);
            return true;
        }
        int rc = (a == "on") ? sbgc_motors_on(&sb) : sbgc_motors_off(&sb);
        if (rc == 0) std::cout << "  motors " << a << "\n";
        else err(sbgc_last_error(&sb));
        return true;
    }

    // Unreachable while the table and this dispatch agree; report loudly if not.
    err("command '" + c + "' is declared but not implemented");
    return true;
}

}  // namespace

// ------------------------------------------------------------------- main --

int main(int argc, char **argv)
{
    Config cfg;
    bool skip_setup = false;

    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--simulate" || a == "-s") cfg.simulate = true;
        else if (a == "--port" && i + 1 < argc) cfg.port = argv[++i];
        else if (a == "--baud" && i + 1 < argc) cfg.baud = std::atoi(argv[++i]);
        else if (a == "--pad" && i + 1 < argc) cfg.pad_path = argv[++i];
        else if (a == "--defaults" || a == "-d") skip_setup = true;
        else if (a == "--help" || a == "-h") {
            std::cout <<
              "usage: gimbal_ctl [--port DEV] [--baud N] [--pad DEV]\n"
              "                  [--simulate] [--defaults]\n"
              "  --port DEV   serial port (default /dev/ttyUSB0)\n"
              "  --pad DEV    gamepad event device; omit to auto-detect\n"
              "  --simulate   no hardware; print frames as hex\n"
              "  --defaults   skip the setup wizard and use built-in defaults\n";
            return 0;
        } else {
            std::cerr << "unknown option: " << a << "\n";
            return 2;
        }
    }

    std::cout << "SimpleBGC gimbal control\n";

    if (!skip_setup) run_setup(cfg);
    show_config(cfg);

    sbgc_t sb;
    if (cfg.simulate) {
        sbgc_open_simulated(&sb);
        std::cout << "Simulation mode — frames are printed, not sent.\n";
    } else if (sbgc_open(&sb, cfg.port.c_str(), cfg.baud) != 0) {
        std::cerr << "Could not open port: " << sbgc_last_error(&sb) << "\n"
                  << "Re-run with --simulate to test without hardware.\n";
        return 1;
    } else {
        std::cout << "Opened " << cfg.port << " at " << cfg.baud << ".\n";
    }

    print_help();

    State st;
    bool quit = false;
    std::string line;

    while (!quit) {
        std::cout << "gimbal> " << std::flush;
        if (!std::getline(std::cin, line)) break;

        line = trim(line);
        if (line.empty()) continue;

        std::vector<std::string> tok;
        std::istringstream iss(line);
        for (std::string w; iss >> w; ) tok.push_back(w);
        if (tok.empty()) continue;

        std::transform(tok[0].begin(), tok[0].end(), tok[0].begin(),
                       [](unsigned char ch) {
                           return static_cast<char>(std::tolower(ch));
                       });

        handle(tok, sb, cfg, st, quit);

        // Surface anything the board volunteered (errors, confirmations).
        sbgc_poll(&sb, 0, on_frame, nullptr);
    }

    sbgc_stop(&sb);
    sbgc_close(&sb);
    std::cout << "Bye.\n";
    return 0;
}
