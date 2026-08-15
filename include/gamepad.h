/*
 * gamepad.h — Linux evdev gamepad input. No external dependencies.
 *
 * Reads /dev/input/event* directly. The kernel's xpad driver binds Xbox
 * controllers automatically over USB and Bluetooth, so a plugged-in pad shows
 * up with no setup on the user's part; gp_find() locates it.
 *
 * Axis ranges are discovered per-device with EVIOCGABS rather than assumed.
 * Xbox pads report sticks as -32768..32767 but triggers as 0..255 or 0..1023
 * depending on the driver and model, and third-party pads differ again.
 * Querying the kernel means normalisation is correct for whatever is attached.
 */

#ifndef GAMEPAD_H
#define GAMEPAD_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Logical controls, resolved from evdev codes at open time. */
enum {
    GP_AX_LEFT_X = 0,
    GP_AX_LEFT_Y,
    GP_AX_RIGHT_X,
    GP_AX_RIGHT_Y,
    GP_AX_LTRIGGER,
    GP_AX_RTRIGGER,
    GP_AX_DPAD_X,
    GP_AX_DPAD_Y,
    GP_AX_COUNT
};

enum {
    GP_BTN_A = 0,
    GP_BTN_B,
    GP_BTN_X,
    GP_BTN_Y,
    GP_BTN_LB,
    GP_BTN_RB,
    GP_BTN_BACK,
    GP_BTN_START,
    GP_BTN_COUNT
};

typedef struct {
    int32_t min, max, flat;
    int32_t value;
    int     present;
} gp_axis_t;

typedef struct {
    int       fd;
    char      name[128];
    char      path[64];
    gp_axis_t axis[GP_AX_COUNT];
    uint8_t   button[GP_BTN_COUNT];
    uint8_t   button_prev[GP_BTN_COUNT];
    char      last_error[192];
} gp_t;

/*
 * Scan /dev/input/event* for the first device that looks like a gamepad
 * (has BTN_SOUTH plus at least ABS_X/ABS_Y). Writes the path into `out`.
 * Returns 0 on success, -1 if none found.
 */
int gp_find(char *out, size_t out_cap);

/* Open a specific device. Returns 0 on success, -1 on failure. */
int gp_open(gp_t *gp, const char *path);

/* Auto-detect and open. Returns 0 on success, -1 on failure. */
int gp_open_auto(gp_t *gp);

void gp_close(gp_t *gp);
const char *gp_last_error(const gp_t *gp);
const char *gp_name(const gp_t *gp);

/*
 * Drain pending events, blocking up to timeout_ms for the first one.
 * Returns the number of events consumed, 0 on timeout, or -1 if the device
 * went away (unplugged) — callers must treat -1 as "stop moving immediately".
 */
int gp_poll(gp_t *gp, int timeout_ms);

/*
 * Stick value normalised to -1..1 with a deadzone applied and the result
 * rescaled so it still reaches 1.0 at full deflection. Returns 0.0 for axes
 * the device does not report.
 *
 * `deadzone` is a floor, not the final word: where the kernel reports a larger
 * flat region for the axis, that is used instead, because the device knows its
 * own resting noise better than a single constant can.
 */
double gp_axis_signed(const gp_t *gp, int ax, double deadzone);

/* Trigger value normalised to 0..1. */
double gp_axis_unit(const gp_t *gp, int ax);

int gp_button(const gp_t *gp, int btn);

/* True on the transition to pressed since the last gp_latch_buttons(). */
int gp_button_pressed(const gp_t *gp, int btn);

/* Snapshot current button state so gp_button_pressed() sees fresh edges. */
void gp_latch_buttons(gp_t *gp);

#ifdef __cplusplus
}
#endif

#endif /* GAMEPAD_H */
