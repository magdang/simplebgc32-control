/*
 * sbgc_api.c — SimpleBGC 32-bit Serial API implementation.
 * See include/sbgc_api.h for the interface and protocol references.
 */

/* cfmakeraw() and CRTSCTS are POSIX extensions that strict -std=cNN hides. */
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif

#include "sbgc_api.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

/* ------------------------------------------------------------------ units -- */

static int16_t saturate_i16(double v)
{
    if (v > 32767.0)  return 32767;
    if (v < -32768.0) return -32768;
    return (int16_t)v;
}

/* Round half away from zero, matching the vendor's worked examples
 * (5 deg/s -> 40.96 -> 41, 90 deg/s -> 737.34 -> 737). */
static double round_half_away(double v)
{
    return (v < 0.0) ? -floor(-v + 0.5) : floor(v + 0.5);
}

int16_t sbgc_deg_to_units(double deg)
{
    return saturate_i16(round_half_away(deg / SBGC_ANGLE_UNIT_DEG));
}

int16_t sbgc_degs_to_units(double deg_per_s)
{
    return saturate_i16(round_half_away(deg_per_s / SBGC_SPEED_UNIT_DEGS));
}

double sbgc_units_to_deg(int16_t units)
{
    return (double)units * SBGC_ANGLE_UNIT_DEG;
}

double sbgc_units_to_degs(int16_t units)
{
    return (double)units * SBGC_SPEED_UNIT_DEGS;
}

/* ------------------------------------------------------------ frame codec -- */

static void put_i16le(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v & 0xFF);
    p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);
}

static int16_t get_i16le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint8_t sum_mod256(const uint8_t *data, size_t len)
{
    uint32_t s = 0;
    for (size_t i = 0; i < len; i++) s += data[i];
    return (uint8_t)(s & 0xFF);
}

int sbgc_encode_frame(uint8_t *out, size_t out_cap,
                      uint8_t cmd_id,
                      const uint8_t *payload, size_t payload_len)
{
    if (!out) return -1;
    if (payload_len > SBGC_MAX_PAYLOAD) return -1;
    if (payload_len > 0 && !payload) return -1;

    size_t need = 4 + payload_len + 1;
    if (out_cap < need) return -1;

    out[0] = SBGC_START_BYTE_V1;
    out[1] = cmd_id;
    out[2] = (uint8_t)payload_len;
    out[3] = (uint8_t)((cmd_id + payload_len) & 0xFF);   /* header checksum */
    if (payload_len) memcpy(out + 4, payload, payload_len);
    out[4 + payload_len] = sum_mod256(payload, payload_len); /* body checksum */

    return (int)need;
}

int sbgc_build_control_payload(uint8_t *out, size_t out_cap,
                               const uint8_t mode[SBGC_NUM_AXES],
                               const int16_t speed[SBGC_NUM_AXES],
                               const int16_t angle[SBGC_NUM_AXES])
{
    if (!out || !mode || !speed || !angle) return -1;
    if (out_cap < 15) return -1;

    /* 3 mode bytes (ROLL, PITCH, YAW) then SPEED/ANGLE interleaved per axis. */
    out[0] = mode[SBGC_ROLL];
    out[1] = mode[SBGC_PITCH];
    out[2] = mode[SBGC_YAW];
    for (int a = 0; a < SBGC_NUM_AXES; a++) {
        put_i16le(out + 3 + a * 4,     speed[a]);
        put_i16le(out + 3 + a * 4 + 2, angle[a]);
    }
    return 15;
}

/* ---------------------------------------------------------------- parser -- */

void sbgc_parser_init(sbgc_parser_t *p)
{
    if (p) { p->len = 0; p->expect = 0; }
}

int sbgc_parser_push(sbgc_parser_t *p, uint8_t byte,
                     uint8_t *cmd_id,
                     const uint8_t **payload, size_t *payload_len)
{
    if (!p) return 0;

    /* Hunting for the start byte. */
    if (p->len == 0) {
        if (byte != SBGC_START_BYTE_V1) return 0;
        p->buf[p->len++] = byte;
        p->expect = 0;
        return 0;
    }

    p->buf[p->len++] = byte;

    /* Header complete: validate it before trusting the length field. */
    if (p->len == 4) {
        uint8_t want = (uint8_t)((p->buf[1] + p->buf[2]) & 0xFF);
        if (want != p->buf[3]) {
            /* Bad header. Resynchronise: the offending byte may itself be a
             * start byte, so re-examine the buffer tail rather than dropping
             * everything blindly. */
            size_t restart = 0;
            for (size_t i = 1; i < p->len; i++) {
                if (p->buf[i] == SBGC_START_BYTE_V1) { restart = i; break; }
            }
            if (restart) {
                memmove(p->buf, p->buf + restart, p->len - restart);
                p->len -= restart;
            } else {
                p->len = 0;
            }
            p->expect = 0;
            return 0;
        }
        p->expect = 4 + (size_t)p->buf[2] + 1;
    }

    if (p->expect && p->len == p->expect) {
        size_t n = p->buf[2];
        uint8_t want = sum_mod256(p->buf + 4, n);
        uint8_t got  = p->buf[4 + n];
        int ok = (want == got);

        if (ok) {
            if (cmd_id)      *cmd_id = p->buf[1];
            if (payload)     *payload = p->buf + 4;
            if (payload_len) *payload_len = n;
        }
        /* Consume the frame either way; a bad body checksum is dropped. */
        p->len = 0;
        p->expect = 0;
        return ok ? 1 : 0;
    }

    if (p->len >= SBGC_MAX_FRAME) { p->len = 0; p->expect = 0; }
    return 0;
}

/* ------------------------------------------------------------- transport -- */

static void set_err(sbgc_t *sb, const char *fmt, ...)
{
    if (!sb) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(sb->last_error, sizeof(sb->last_error), fmt, ap);
    va_end(ap);
}

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return 0;
    }
}

void sbgc_open_simulated(sbgc_t *sb)
{
    if (!sb) return;
    sb->fd = -1;
    sb->simulate = 1;
    sbgc_parser_init(&sb->parser);
    sb->last_error[0] = '\0';
    /* Callers declare sbgc_t on the stack, so anything left untouched here is
     * indeterminate. A garbage `quiet` silently suppresses the simulated frame
     * dump that is the whole point of this mode, and a garbage last_tx_len
     * makes the HUD's "last tx" field paint uninitialised bytes. */
    sb->quiet = 0;
    sb->last_tx_len = 0;
}

int sbgc_open(sbgc_t *sb, const char *device, int baud)
{
    if (!sb || !device) return -1;

    sb->fd = -1;
    sb->simulate = 0;
    sbgc_parser_init(&sb->parser);
    sb->last_error[0] = '\0';
    sb->quiet = 0;          /* see sbgc_open_simulated */
    sb->last_tx_len = 0;

    speed_t sp = baud_to_speed(baud);
    if (sp == 0) { set_err(sb, "unsupported baud rate %d", baud); return -1; }

    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        set_err(sb, "open(%s): %s", device, strerror(errno));
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        set_err(sb, "tcgetattr(%s): %s", device, strerror(errno));
        close(fd);
        return -1;
    }

    cfmakeraw(&tio);                       /* 8N1, no echo, no translation  */
    tio.c_cflag |= (CLOCAL | CREAD);       /* ignore modem lines, enable rx */
    tio.c_cflag &= (unsigned)~CRTSCTS;     /* no hardware flow control      */
    tio.c_cflag &= (unsigned)~CSTOPB;      /* one stop bit                  */
    tio.c_cflag &= (unsigned)~PARENB;      /* no parity                     */
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    if (cfsetispeed(&tio, sp) != 0 || cfsetospeed(&tio, sp) != 0) {
        set_err(sb, "cfsetspeed(%s): %s", device, strerror(errno));
        close(fd);
        return -1;
    }
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        set_err(sb, "tcsetattr(%s): %s", device, strerror(errno));
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    sb->fd = fd;
    return 0;
}

void sbgc_close(sbgc_t *sb)
{
    if (!sb) return;
    if (sb->fd >= 0) close(sb->fd);
    sb->fd = -1;
    sb->simulate = 0;
}

const char *sbgc_last_error(const sbgc_t *sb)
{
    return (sb && sb->last_error[0]) ? sb->last_error : "no error";
}

void sbgc_set_quiet(sbgc_t *sb, int quiet)
{
    if (sb) sb->quiet = quiet ? 1 : 0;
}

size_t sbgc_format_last_tx(const sbgc_t *sb, char *out, size_t out_cap)
{
    if (!sb || !out || out_cap == 0) return 0;
    out[0] = '\0';
    if (sb->last_tx_len == 0) return 0;

    size_t used = 0;
    for (size_t i = 0; i < sb->last_tx_len; i++) {
        /* "XX " is 3 chars; leave room for it plus the terminator. */
        if (used + 4 > out_cap) break;
        int w = snprintf(out + used, out_cap - used, "%02X ", sb->last_tx[i]);
        if (w <= 0) break;
        used += (size_t)w;
    }
    /* Trim the trailing space. */
    if (used > 0 && out[used - 1] == ' ') out[--used] = '\0';
    return used;
}

int sbgc_send(sbgc_t *sb, uint8_t cmd_id,
              const uint8_t *payload, size_t payload_len)
{
    if (!sb) return -1;

    uint8_t frame[SBGC_MAX_FRAME];
    int n = sbgc_encode_frame(frame, sizeof(frame), cmd_id, payload, payload_len);
    if (n < 0) { set_err(sb, "failed to encode command %u", cmd_id); return -1; }

    /* Record before transmitting so a debugging UI can show the exact bytes
     * even if the write subsequently fails. */
    memcpy(sb->last_tx, frame, (size_t)n);
    sb->last_tx_len = (size_t)n;

    if (sb->simulate) {
        if (!sb->quiet) {
            printf("  [sim] tx cmd=%-3u len=%-3zu :", cmd_id, payload_len);
            for (int i = 0; i < n; i++) printf(" %02X", frame[i]);
            printf("\n");
        }
        return 0;
    }

    if (sb->fd < 0) { set_err(sb, "port is not open"); return -1; }

    /* Short writes are possible on a serial fd; loop until the frame is out. */
    size_t off = 0;
    while (off < (size_t)n) {
        ssize_t w = write(sb->fd, frame + off, (size_t)n - off);
        if (w < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN) {
                struct pollfd pfd = { sb->fd, POLLOUT, 0 };
                if (poll(&pfd, 1, 200) > 0) continue;
                set_err(sb, "write timed out");
                return -1;
            }
            set_err(sb, "write: %s", strerror(errno));
            return -1;
        }
        off += (size_t)w;
    }
    return 0;
}

/* Milliseconds on the monotonic clock, for bounding the drain below. */
static long sbgc_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

int sbgc_poll(sbgc_t *sb, int timeout_ms, sbgc_frame_cb cb, void *user)
{
    if (!sb) return -1;
    if (sb->simulate || sb->fd < 0) return 0;

    int frames = 0;
    struct pollfd pfd = { sb->fd, POLLIN, 0 };

    /*
     * The drain below re-polls with a zero timeout, which returns immediately
     * while any byte is waiting. Against a device that transmits continuously
     * — a LiDAR or GPS on the same hub as the gimbal, or a serial line picking
     * up noise — that condition never goes false and this function never
     * returns. It is called from the serial thread, so the loop that carries
     * the motion watchdog would stop running entirely.
     *
     * The caller's timeout therefore bounds the whole call, not just the first
     * poll, and the number of reads is capped as well so a fast producer
     * cannot keep it here even inside that budget.
     */
    const long deadline = sbgc_now_ms() + (timeout_ms > 0 ? timeout_ms : 0);
    int reads = 0;
    const int MAX_READS = 64;          /* 16 KiB at 256 bytes a read */

    for (;;) {
        int pr = poll(&pfd, 1, timeout_ms);
        if (pr < 0) {
            if (errno == EINTR) continue;
            set_err(sb, "poll: %s", strerror(errno));
            return -1;
        }
        if (pr == 0) break;                 /* timed out; nothing more       */

        uint8_t buf[256];
        ssize_t r = read(sb->fd, buf, sizeof(buf));
        if (r < 0) {
            if (errno == EINTR || errno == EAGAIN) break;
            set_err(sb, "read: %s", strerror(errno));
            return -1;
        }
        if (r == 0) break;

        for (ssize_t i = 0; i < r; i++) {
            uint8_t cmd = 0;
            const uint8_t *pl = NULL;
            size_t pl_len = 0;
            if (sbgc_parser_push(&sb->parser, buf[i], &cmd, &pl, &pl_len)) {
                frames++;
                if (cb) cb(cmd, pl, pl_len, user);
            }
        }
        /* Drain whatever else is already buffered, within the budget. */
        timeout_ms = 0;
        if (++reads >= MAX_READS) break;
        if (sbgc_now_ms() >= deadline) break;
    }
    return frames;
}

/* -------------------------------------------------------- high-level ops -- */

int sbgc_control_raw(sbgc_t *sb,
                     const uint8_t mode[SBGC_NUM_AXES],
                     const int16_t speed[SBGC_NUM_AXES],
                     const int16_t angle[SBGC_NUM_AXES])
{
    uint8_t payload[15];
    if (sbgc_build_control_payload(payload, sizeof(payload),
                                   mode, speed, angle) < 0) {
        set_err(sb, "failed to build CMD_CONTROL payload");
        return -1;
    }
    return sbgc_send(sb, SBGC_CMD_CONTROL, payload, sizeof(payload));
}

int sbgc_control_speed(sbgc_t *sb, const double deg_per_s[SBGC_NUM_AXES])
{
    uint8_t mode[SBGC_NUM_AXES];
    int16_t speed[SBGC_NUM_AXES], angle[SBGC_NUM_AXES];

    for (int a = 0; a < SBGC_NUM_AXES; a++) {
        mode[a]  = SBGC_MODE_SPEED;
        speed[a] = sbgc_degs_to_units(deg_per_s[a]);
        angle[a] = 0;
    }
    return sbgc_control_raw(sb, mode, speed, angle);
}

int sbgc_control_angle(sbgc_t *sb, const double deg[SBGC_NUM_AXES],
                       const double slew_deg_s[SBGC_NUM_AXES],
                       int rel_frame)
{
    uint8_t base = rel_frame ? SBGC_MODE_ANGLE_REL_FRAME : SBGC_MODE_ANGLE;
    uint8_t mode[SBGC_NUM_AXES];
    int16_t speed[SBGC_NUM_AXES], angle[SBGC_NUM_AXES];

    for (int a = 0; a < SBGC_NUM_AXES; a++) {
        mode[a]  = base;
        angle[a] = sbgc_deg_to_units(deg[a]);
        /* Speed 0 tells the board to use its own configured slew rate. */
        speed[a] = slew_deg_s ? sbgc_degs_to_units(slew_deg_s[a]) : 0;
    }
    return sbgc_control_raw(sb, mode, speed, angle);
}

int sbgc_stop(sbgc_t *sb)
{
    const double zero[SBGC_NUM_AXES] = { 0.0, 0.0, 0.0 };
    return sbgc_control_speed(sb, zero);
}

int sbgc_home(sbgc_t *sb)
{
    /* Vendor "home position": ROLL to horizon (MODE_ANGLE), PITCH and YAW to
     * neutral relative to the frame, all as auto-tasks, all values zero.
     * Mode bytes 0x42 0x45 0x45. */
    const uint8_t mode[SBGC_NUM_AXES] = {
        SBGC_MODE_ANGLE           | SBGC_CTRL_FLAG_AUTO_TASK,
        SBGC_MODE_ANGLE_REL_FRAME | SBGC_CTRL_FLAG_AUTO_TASK,
        SBGC_MODE_ANGLE_REL_FRAME | SBGC_CTRL_FLAG_AUTO_TASK
    };
    const int16_t zero[SBGC_NUM_AXES] = { 0, 0, 0 };
    return sbgc_control_raw(sb, mode, zero, zero);
}

int sbgc_level(sbgc_t *sb)
{
    /* Vendor "leveled position": ROLL and PITCH to horizon, YAW neutral
     * relative to the frame. Mode bytes 0x42 0x42 0x45. */
    const uint8_t mode[SBGC_NUM_AXES] = {
        SBGC_MODE_ANGLE           | SBGC_CTRL_FLAG_AUTO_TASK,
        SBGC_MODE_ANGLE           | SBGC_CTRL_FLAG_AUTO_TASK,
        SBGC_MODE_ANGLE_REL_FRAME | SBGC_CTRL_FLAG_AUTO_TASK
    };
    const int16_t zero[SBGC_NUM_AXES] = { 0, 0, 0 };
    return sbgc_control_raw(sb, mode, zero, zero);
}

int sbgc_motors_on(sbgc_t *sb)
{
    return sbgc_send(sb, SBGC_CMD_MOTORS_ON, NULL, 0);
}

int sbgc_motors_off(sbgc_t *sb)
{
    return sbgc_send(sb, SBGC_CMD_MOTORS_OFF, NULL, 0);
}

int sbgc_execute_menu(sbgc_t *sb, uint8_t menu_cmd)
{
    return sbgc_send(sb, SBGC_CMD_EXECUTE_MENU, &menu_cmd, 1);
}

int sbgc_calib_gyro(sbgc_t *sb)
{
    /* The board must be still for the duration; see the header. */
    return sbgc_execute_menu(sb, SBGC_MENU_CALIB_GYRO);
}

int sbgc_request_angles(sbgc_t *sb)
{
    return sbgc_send(sb, SBGC_CMD_GET_ANGLES, NULL, 0);
}

int sbgc_parse_angles(const uint8_t *payload, size_t payload_len,
                      sbgc_angles_t *out)
{
    if (!payload || !out) return -1;
    if (payload_len != 18) return -1;   /* checked, never assumed */

    for (int a = 0; a < SBGC_NUM_AXES; a++) {
        const uint8_t *p = payload + a * 6;
        out->imu_deg[a]      = sbgc_units_to_deg(get_i16le(p));
        out->target_deg[a]   = sbgc_units_to_deg(get_i16le(p + 2));
        out->target_deg_s[a] = sbgc_units_to_degs(get_i16le(p + 4));
    }
    return 0;
}
