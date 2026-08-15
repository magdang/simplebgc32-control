/*
 * sbgc_api.h — SimpleBGC 32-bit (BaseCam / AlexMos) Serial API, C interface.
 *
 * Self-contained implementation of the subset of the SimpleBGC32 Serial API
 * needed to drive a camera gimbal: control, angle feedback, menu actions and
 * motor power. No third-party dependencies; POSIX termios only.
 *
 * Protocol reference:
 *   "SBGC32 Serial API Protocol Specification" (v2.6, BaseCam Electronics).
 *   Frame layout and unit scaling in this file were byte-verified against the
 *   vendor's official "SBGC32 API cmd examples" worked hex dumps. See
 *   test/test_sbgc_api.c, which asserts the exact vendor bytes.
 *
 * Protocol version 1 is used deliberately (start byte '>' 0x3E, 8-bit
 * checksums). Version 2 ('$' 0x24, CRC16) exists on firmware 2.68b0+, but v1
 * is understood by every 32-bit firmware and the board locks to v2 permanently
 * once a v2 message is seen. v1 keeps this tool compatible with older boards.
 */

#ifndef SBGC_API_H
#define SBGC_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- limits -- */

#define SBGC_START_BYTE_V1   0x3E   /* '>' */
#define SBGC_MAX_PAYLOAD     255
#define SBGC_MAX_FRAME       (4 + SBGC_MAX_PAYLOAD + 1)

/* ------------------------------------------------------------ command IDs -- */

enum {
    SBGC_CMD_BOARD_INFO     = 86,
    SBGC_CMD_REALTIME_DATA_4 = 25,
    SBGC_CMD_CONTROL        = 67,   /* also CMD_CONFIRM when received     */
    SBGC_CMD_EXECUTE_MENU   = 69,
    SBGC_CMD_GET_ANGLES     = 73,
    SBGC_CMD_MOTORS_ON      = 77,
    SBGC_CMD_MOTORS_OFF     = 109,
    SBGC_CMD_ERROR          = 255
};

/* --------------------------------------------------------- control modes -- */
/*
 * CMD_CONTROL carries one mode byte per axis, in ROLL, PITCH, YAW order.
 * The low bits select the mode; the high bits are flags OR'd on top.
 *
 * This enumerates the protocol's value space, not this library's feature list.
 * The modes it actually sends are NO_CONTROL, SPEED, ANGLE and
 * ANGLE_REL_FRAME; the rest are here so a decoded mode byte can be named
 * rather than printed as a number. Naming a value is not a promise to drive
 * it — see the high-level ops at the bottom of this header for what is
 * genuinely implemented.
 */
enum {
    SBGC_MODE_NO_CONTROL      = 0,  /* leave this axis alone               */
    SBGC_MODE_SPEED           = 1,  /* SPEED_x is a rate, ANGLE_x ignored  */
    SBGC_MODE_ANGLE           = 2,  /* ANGLE_x absolute, SPEED_x = slew    */
    SBGC_MODE_SPEED_ANGLE     = 3,
    SBGC_MODE_RC              = 4,
    SBGC_MODE_ANGLE_REL_FRAME = 5   /* ANGLE_x relative to the frame       */
};

/* Likewise the flag bits: AUTO_TASK is used by home and level, the other is
 * named for decoding. */
enum {
    SBGC_CTRL_FLAG_AUTO_TASK      = 0x40, /* run as a task, confirm on end */
    SBGC_CTRL_FLAG_FORCE_RC_SPEED = 0x80
};

/*
 * Menu actions for CMD_EXECUTE_MENU.
 *
 * Only gyro calibration is issued this way, because it has no dedicated
 * command whose bytes this file has verified. Motor power and home position
 * both do — CMD_MOTORS_ON/OFF and the CMD_CONTROL frames behind sbgc_home()
 * — and a dedicated command says exactly what it will do, where a menu action
 * runs whatever the operator's profile has bound to that button. The menu
 * equivalents are deliberately not offered here.
 */
enum {
    SBGC_MENU_CALIB_GYRO = 9
};


/* ------------------------------------------------------------------ units -- */
/*
 * Wire units. Both were confirmed arithmetically against the vendor examples:
 *   45 deg   -> 2048   (45   / 0.02197265625 = 2048)
 *  -90 deg   -> -4096
 *    5 deg/s -> 41     (5    / 0.1220740379  = 40.96 -> 41)
 *   90 deg/s -> 737    (90   / 0.1220740379  = 737.3 -> 737)
 */
#define SBGC_ANGLE_UNIT_DEG   0.02197265625   /* deg per LSB (= 360/16384) */
#define SBGC_SPEED_UNIT_DEGS  0.1220740379    /* deg/s per LSB             */

/* Convert to wire units, rounding half away from zero and saturating to int16. */
int16_t sbgc_deg_to_units(double deg);
int16_t sbgc_degs_to_units(double deg_per_s);
double  sbgc_units_to_deg(int16_t units);
double  sbgc_units_to_degs(int16_t units);

/* Axis indices into every 3-element array in this API. */
enum { SBGC_ROLL = 0, SBGC_PITCH = 1, SBGC_YAW = 2, SBGC_NUM_AXES = 3 };

/* ------------------------------------------------------------ frame codec -- */
/*
 * These are pure functions with no I/O, so they are unit-testable without a
 * board attached. This is the part that must be exactly right.
 */

/*
 * Build a protocol-v1 frame into `out`.
 *   out       destination buffer
 *   out_cap   capacity of `out`
 *   cmd_id    command ID
 *   payload   payload bytes (may be NULL when payload_len == 0)
 * Returns the number of bytes written, or -1 on bad arguments / short buffer.
 */
int sbgc_encode_frame(uint8_t *out, size_t out_cap,
                      uint8_t cmd_id,
                      const uint8_t *payload, size_t payload_len);

/*
 * Build the 15-byte CMD_CONTROL payload.
 *   mode[3]   per-axis mode byte (mode | flags), ROLL/PITCH/YAW
 *   speed[3]  per-axis speed in wire units
 *   angle[3]  per-axis angle in wire units
 * Returns 15 on success, -1 on bad arguments.
 */
int sbgc_build_control_payload(uint8_t *out, size_t out_cap,
                               const uint8_t mode[SBGC_NUM_AXES],
                               const int16_t speed[SBGC_NUM_AXES],
                               const int16_t angle[SBGC_NUM_AXES]);

/*
 * Incremental parser. Feed it bytes as they arrive; it resynchronises on the
 * start byte and validates both checksums, silently discarding garbage.
 */
typedef struct {
    uint8_t  buf[SBGC_MAX_FRAME];
    size_t   len;
    size_t   expect;   /* total frame length once the header is known, else 0 */
} sbgc_parser_t;

void sbgc_parser_init(sbgc_parser_t *p);

/*
 * Push one byte. Returns 1 when a complete, checksum-valid frame is available,
 * in which case *cmd_id, *payload and *payload_len describe it; the payload
 * pointer aliases internal storage and is valid until the next push. Returns 0
 * when more bytes are needed.
 */
int sbgc_parser_push(sbgc_parser_t *p, uint8_t byte,
                     uint8_t *cmd_id,
                     const uint8_t **payload, size_t *payload_len);

/* ------------------------------------------------------------- transport -- */

typedef struct {
    int  fd;                 /* -1 when closed or simulated                  */
    int  simulate;           /* 1 = no hardware, log frames instead of write */
    int  quiet;              /* 1 = never print; for full-screen UI modes    */

    sbgc_parser_t parser;

    /* Last frame handed to the transport, kept for debugging displays. This
     * is recorded in both live and simulated mode, so a UI can show exactly
     * what went on the wire without duplicating the encoder. */
    uint8_t last_tx[SBGC_MAX_FRAME];
    size_t  last_tx_len;

    char last_error[192];
} sbgc_t;

/*
 * Suppress the simulation-mode frame printing. Full-screen modes must set
 * this, otherwise stray printf output corrupts the display.
 */
void sbgc_set_quiet(sbgc_t *sb, int quiet);

/* Format the last transmitted frame as uppercase hex into `out`. Returns the
 * number of characters written (0 if nothing has been sent yet). */
size_t sbgc_format_last_tx(const sbgc_t *sb, char *out, size_t out_cap);

/*
 * Open `device` at `baud` (typically 115200) in raw 8N1 mode.
 * Returns 0 on success, -1 on failure (see sbgc_last_error).
 */
int  sbgc_open(sbgc_t *sb, const char *device, int baud);

/*
 * Enter simulation mode: no device is opened, frames are encoded and can be
 * inspected but nothing is written. Lets the whole tool be exercised before
 * the gimbal is physically wired.
 */
void sbgc_open_simulated(sbgc_t *sb);

void sbgc_close(sbgc_t *sb);
const char *sbgc_last_error(const sbgc_t *sb);

/* Send an arbitrary command. Returns 0 on success, -1 on write failure. */
int sbgc_send(sbgc_t *sb, uint8_t cmd_id,
              const uint8_t *payload, size_t payload_len);

/*
 * Read and dispatch any pending inbound frames, blocking up to timeout_ms.
 * Returns the number of complete frames consumed, or -1 on read error.
 * `cb` may be NULL to simply drain the port.
 */
typedef void (*sbgc_frame_cb)(uint8_t cmd_id, const uint8_t *payload,
                              size_t payload_len, void *user);
int sbgc_poll(sbgc_t *sb, int timeout_ms, sbgc_frame_cb cb, void *user);

/* -------------------------------------------------------- high-level ops -- */

/* Send CMD_CONTROL with per-axis modes and values already in wire units. */
int sbgc_control_raw(sbgc_t *sb,
                     const uint8_t mode[SBGC_NUM_AXES],
                     const int16_t speed[SBGC_NUM_AXES],
                     const int16_t angle[SBGC_NUM_AXES]);

/*
 * Rate control: command each axis to turn at the given deg/s. Axes given
 * exactly 0.0 are still sent in SPEED mode with rate 0, which is how the
 * gimbal is told to hold still (as opposed to NO_CONTROL, which relinquishes
 * the axis and lets the previous command persist).
 */
int sbgc_control_speed(sbgc_t *sb, const double deg_per_s[SBGC_NUM_AXES]);

/*
 * Absolute angle control. `rel_frame` selects MODE_ANGLE_REL_FRAME (angles
 * measured against the robot frame) instead of MODE_ANGLE (against the
 * horizon). slew_deg_s of 0 means "use the board's configured default speed".
 */
int sbgc_control_angle(sbgc_t *sb, const double deg[SBGC_NUM_AXES],
                       const double slew_deg_s[SBGC_NUM_AXES],
                       int rel_frame);

/* Stop all motion by commanding zero rate on every axis. */
int sbgc_stop(sbgc_t *sb);

/*
 * Home: ROLL to the horizon, PITCH and YAW to neutral relative to the frame,
 * as an auto-task. This reproduces the vendor's documented "home position"
 * frame byte-for-byte (mode bytes 0x42 0x45 0x45, all values zero).
 */
int sbgc_home(sbgc_t *sb);

/*
 * Level: ROLL and PITCH to the horizon, YAW neutral relative to the frame.
 * Vendor mode bytes 0x42 0x42 0x45.
 */
int sbgc_level(sbgc_t *sb);

int sbgc_motors_on(sbgc_t *sb);
int sbgc_motors_off(sbgc_t *sb);
int sbgc_execute_menu(sbgc_t *sb, uint8_t menu_cmd);

/*
 * Gyroscope calibration.
 *
 * Issued as a menu action rather than via the dedicated CMD_CALIB_GYRO. Both
 * exist, but the menu numbering is corroborated by the two constants already
 * in the menu enum above, whereas the dedicated command ID is not
 * byte-verified against any board here — and this file's rule is that
 * unverified bytes do not get sent to hardware.
 *
 * The gimbal MUST be physically still while this runs. Calibrating a moving
 * gimbal writes a wrong zero-rate bias into the board, and the result is a
 * camera that drifts continuously afterwards. That is why it is never done
 * automatically unless explicitly asked for.
 */
int sbgc_calib_gyro(sbgc_t *sb);

/* Request CMD_GET_ANGLES; the response arrives via sbgc_poll. */
int sbgc_request_angles(sbgc_t *sb);

/*
 * Decoded CMD_GET_ANGLES response. The board reports, per axis, the IMU angle,
 * the RC/serial target angle, and the target speed, each as 2s in wire units.
 */
typedef struct {
    double imu_deg[SBGC_NUM_AXES];
    double target_deg[SBGC_NUM_AXES];
    double target_deg_s[SBGC_NUM_AXES];
} sbgc_angles_t;

/*
 * Parse an 18-byte CMD_GET_ANGLES payload. Returns 0 on success, -1 if the
 * payload length is not 18 — the length is checked rather than assumed,
 * because the response layout could not be byte-verified against the vendor
 * examples (only the request side was). A wrong-length payload is reported,
 * never silently misparsed.
 */
int sbgc_parse_angles(const uint8_t *payload, size_t payload_len,
                      sbgc_angles_t *out);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* SBGC_API_H */
