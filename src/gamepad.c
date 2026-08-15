/*
 * gamepad.c — Linux evdev gamepad input. See include/gamepad.h.
 */

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif

#include "gamepad.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* ------------------------------------------------------------------ utils -- */

static void gp_set_err(gp_t *gp, const char *fmt, ...)
{
    if (!gp) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(gp->last_error, sizeof(gp->last_error), fmt, ap);
    va_end(ap);
}

#define BITS_PER_LONG  (int)(8 * sizeof(long))
#define NBITS(x)       (((x) - 1) / BITS_PER_LONG + 1)

static int test_bit(const unsigned long *arr, int bit)
{
    return (arr[bit / BITS_PER_LONG] >> (bit % BITS_PER_LONG)) & 1UL;
}

/* Map evdev ABS_* codes onto our logical axis slots. */
static int abs_code_to_slot(int code)
{
    switch (code) {
        case ABS_X:     return GP_AX_LEFT_X;
        case ABS_Y:     return GP_AX_LEFT_Y;
        case ABS_RX:    return GP_AX_RIGHT_X;
        case ABS_RY:    return GP_AX_RIGHT_Y;
        case ABS_Z:     return GP_AX_LTRIGGER;
        case ABS_RZ:    return GP_AX_RTRIGGER;
        case ABS_HAT0X: return GP_AX_DPAD_X;
        case ABS_HAT0Y: return GP_AX_DPAD_Y;
        default:        return -1;
    }
}

static int slot_to_abs_code(int slot)
{
    switch (slot) {
        case GP_AX_LEFT_X:    return ABS_X;
        case GP_AX_LEFT_Y:    return ABS_Y;
        case GP_AX_RIGHT_X:   return ABS_RX;
        case GP_AX_RIGHT_Y:   return ABS_RY;
        case GP_AX_LTRIGGER:  return ABS_Z;
        case GP_AX_RTRIGGER:  return ABS_RZ;
        case GP_AX_DPAD_X:    return ABS_HAT0X;
        case GP_AX_DPAD_Y:    return ABS_HAT0Y;
        default:              return -1;
    }
}

/* Map evdev BTN_* codes onto our logical button slots.
 * Xbox layout under xpad: SOUTH=A, EAST=B, WEST=X, NORTH=Y. */
static int btn_code_to_slot(int code)
{
    switch (code) {
        case BTN_SOUTH:  return GP_BTN_A;
        case BTN_EAST:   return GP_BTN_B;
        case BTN_WEST:   return GP_BTN_X;
        case BTN_NORTH:  return GP_BTN_Y;
        case BTN_TL:     return GP_BTN_LB;
        case BTN_TR:     return GP_BTN_RB;
        case BTN_SELECT: return GP_BTN_BACK;
        case BTN_START:  return GP_BTN_START;
        default:         return -1;
    }
}

static int slot_to_btn_code(int slot)
{
    switch (slot) {
        case GP_BTN_A:     return BTN_SOUTH;
        case GP_BTN_B:     return BTN_EAST;
        case GP_BTN_X:     return BTN_WEST;
        case GP_BTN_Y:     return BTN_NORTH;
        case GP_BTN_LB:    return BTN_TL;
        case GP_BTN_RB:    return BTN_TR;
        case GP_BTN_BACK:  return BTN_SELECT;
        case GP_BTN_START: return BTN_START;
        default:           return -1;
    }
}

/* ------------------------------------------------------------------- find -- */

/* A gamepad, for our purposes, has the A button and at least a left stick. */
static int looks_like_gamepad(int fd)
{
    unsigned long keybit[NBITS(KEY_MAX)];
    unsigned long absbit[NBITS(ABS_MAX)];

    memset(keybit, 0, sizeof(keybit));
    memset(absbit, 0, sizeof(absbit));

    if (ioctl(fd, EVIOCGBIT(EV_KEY, (int)sizeof(keybit)), keybit) < 0) return 0;
    if (ioctl(fd, EVIOCGBIT(EV_ABS, (int)sizeof(absbit)), absbit) < 0) return 0;

    int has_btn  = test_bit(keybit, BTN_SOUTH) || test_bit(keybit, BTN_GAMEPAD);
    int has_stick = test_bit(absbit, ABS_X) && test_bit(absbit, ABS_Y);

    return has_btn && has_stick;
}

int gp_find(char *out, size_t out_cap)
{
    if (!out || out_cap == 0) return -1;

    DIR *d = opendir("/dev/input");
    if (!d) return -1;

    /* Deterministic order: event0, event1, ... so repeated runs agree. */
    int best = -1;
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (strncmp(ent->d_name, "event", 5) != 0) continue;

        /* Require "event" followed only by digits, then rebuild the path from
         * the parsed number. Formatting the raw d_name (up to 255 bytes) into
         * a fixed buffer would be a truncation risk. */
        const char *digits = ent->d_name + 5;
        if (*digits == '\0') continue;
        int all_digits = 1;
        for (const char *p = digits; *p; p++)
            if (*p < '0' || *p > '9') { all_digits = 0; break; }
        if (!all_digits) continue;

        int n = atoi(digits);
        if (n < 0 || n > 9999) continue;

        char path[32];
        snprintf(path, sizeof(path), "/dev/input/event%d", n);
        int fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;

        int ok = looks_like_gamepad(fd);
        close(fd);

        if (ok && (best < 0 || n < best)) best = n;
    }
    closedir(d);

    if (best < 0) return -1;
    snprintf(out, out_cap, "/dev/input/event%d", best);
    return 0;
}

/* ------------------------------------------------------------------- open -- */

int gp_open(gp_t *gp, const char *path)
{
    if (!gp || !path) return -1;

    memset(gp, 0, sizeof(*gp));
    gp->fd = -1;

    int fd = open(path, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        if (errno == EACCES) {
            gp_set_err(gp,
                "open(%s): permission denied. Add yourself to the 'input' "
                "group (sudo usermod -aG input $USER) and log back in.", path);
        } else {
            gp_set_err(gp, "open(%s): %s", path, strerror(errno));
        }
        return -1;
    }

    if (!looks_like_gamepad(fd)) {
        gp_set_err(gp, "%s does not look like a gamepad", path);
        close(fd);
        return -1;
    }

    if (ioctl(fd, EVIOCGNAME((int)sizeof(gp->name)), gp->name) < 0)
        snprintf(gp->name, sizeof(gp->name), "unknown gamepad");

    snprintf(gp->path, sizeof(gp->path), "%s", path);

    /* Ask the kernel for each axis's real range instead of assuming one. */
    for (int slot = 0; slot < GP_AX_COUNT; slot++) {
        int code = slot_to_abs_code(slot);
        if (code < 0) continue;

        /* EVIOCGABS() shifts its argument internally; feed it an unsigned
         * value so the whole expansion stays unsigned and -Wsign-conversion
         * has nothing to complain about. */
        unsigned ucode = (unsigned)code;
        struct input_absinfo info;
        if (ioctl(fd, EVIOCGABS(ucode), &info) == 0 &&
            info.minimum != info.maximum) {
            gp->axis[slot].min     = info.minimum;
            gp->axis[slot].max     = info.maximum;
            gp->axis[slot].flat    = info.flat;
            gp->axis[slot].value   = info.value;
            gp->axis[slot].present = 1;
        }
    }

    gp->fd = fd;
    return 0;
}

int gp_open_auto(gp_t *gp)
{
    if (!gp) return -1;

    char path[64];
    if (gp_find(path, sizeof(path)) != 0) {
        memset(gp, 0, sizeof(*gp));
        gp->fd = -1;
        gp_set_err(gp,
            "no gamepad found under /dev/input. Is it plugged in? "
            "Check with: ls /dev/input/event*");
        return -1;
    }
    return gp_open(gp, path);
}

void gp_close(gp_t *gp)
{
    if (!gp) return;
    if (gp->fd >= 0) close(gp->fd);
    gp->fd = -1;
}

const char *gp_last_error(const gp_t *gp)
{
    return (gp && gp->last_error[0]) ? gp->last_error : "no error";
}

const char *gp_name(const gp_t *gp)
{
    return (gp && gp->name[0]) ? gp->name : "gamepad";
}

/* ------------------------------------------------------------------- poll -- */

/*
 * Re-read the whole device state straight from the kernel, discarding whatever
 * we believed. Required after SYN_DROPPED, where the kernel's per-client queue
 * overflowed and it threw events away: the release we never saw is simply
 * gone, and no amount of further reading will produce it.
 *
 * This matters more here than in a typical evdev client. The event lost may be
 * the release of the deadman button, and a deadman latched down is a camera
 * that keeps turning with the operator's thumb off the controller — the one
 * failure the deadman exists to prevent. Where the state cannot be read back
 * at all, everything is treated as released rather than assumed unchanged.
 */
static void gp_resync(gp_t *gp)
{
    unsigned long keys[NBITS(KEY_MAX)];
    memset(keys, 0, sizeof(keys));

    if (ioctl(gp->fd, EVIOCGKEY((int)sizeof(keys)), keys) >= 0) {
        for (int slot = 0; slot < GP_BTN_COUNT; slot++) {
            int code = slot_to_btn_code(slot);
            if (code >= 0) gp->button[slot] = test_bit(keys, code) ? 1 : 0;
        }
    } else {
        memset(gp->button, 0, sizeof(gp->button));   /* fail closed */
    }

    for (int slot = 0; slot < GP_AX_COUNT; slot++) {
        if (!gp->axis[slot].present) continue;
        int code = slot_to_abs_code(slot);
        if (code < 0) continue;

        unsigned ucode = (unsigned)code;
        struct input_absinfo info;
        if (ioctl(gp->fd, EVIOCGABS(ucode), &info) == 0) {
            gp->axis[slot].value = info.value;
        } else {
            /* Unknown: fall back to the axis's resting position — centred for
             * a stick or hat, fully released for a trigger. */
            gp->axis[slot].value =
                (slot == GP_AX_LTRIGGER || slot == GP_AX_RTRIGGER)
                    ? gp->axis[slot].min
                    : (gp->axis[slot].min + gp->axis[slot].max) / 2;
        }
    }
}

int gp_poll(gp_t *gp, int timeout_ms)
{
    if (!gp || gp->fd < 0) return -1;

    int consumed = 0;
    int dropped  = 0;      /* inside a torn packet, waiting for SYN_REPORT */
    struct pollfd pfd = { gp->fd, POLLIN, 0 };

    for (;;) {
        int pr = poll(&pfd, 1, timeout_ms);
        if (pr < 0) {
            if (errno == EINTR) continue;
            gp_set_err(gp, "poll: %s", strerror(errno));
            return -1;
        }
        if (pr == 0) break;

        /* Unplugged: the device node reports an error/hangup. */
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            gp_set_err(gp, "gamepad disconnected");
            return -1;
        }

        struct input_event ev[64];
        ssize_t r = read(gp->fd, ev, sizeof(ev));
        if (r < 0) {
            if (errno == EAGAIN) break;
            if (errno == EINTR) continue;
            gp_set_err(gp, "read: %s", strerror(errno));
            return -1;   /* ENODEV on unplug */
        }
        if (r == 0) break;
        if (r < (ssize_t)sizeof(struct input_event)) break;

        size_t n = (size_t)r / sizeof(struct input_event);
        for (size_t i = 0; i < n; i++) {
            /*
             * SYN_DROPPED says the queue overflowed. Everything from here to
             * the next SYN_REPORT is the tail of a packet whose start was
             * discarded, so it describes no coherent state and is skipped;
             * the state we missed is then read back from the device.
             */
            if (ev[i].type == EV_SYN) {
                if (ev[i].code == SYN_DROPPED) {
                    dropped = 1;
                } else if (ev[i].code == SYN_REPORT && dropped) {
                    dropped = 0;
                    gp_resync(gp);
                    consumed++;
                }
                continue;
            }
            if (dropped) continue;

            if (ev[i].type == EV_ABS) {
                int slot = abs_code_to_slot(ev[i].code);
                if (slot >= 0 && gp->axis[slot].present) {
                    gp->axis[slot].value = ev[i].value;
                    consumed++;
                }
            } else if (ev[i].type == EV_KEY) {
                int slot = btn_code_to_slot(ev[i].code);
                /* value 2 is autorepeat; treat anything non-zero as held. */
                if (slot >= 0) {
                    gp->button[slot] = ev[i].value ? 1 : 0;
                    consumed++;
                }
            }
        }
        timeout_ms = 0;   /* drain the rest without blocking again */
    }

    /*
     * An overrun whose closing SYN_REPORT has not arrived yet — it can land in
     * the next read. Resync now regardless: waiting would mean carrying the
     * stale button state, possibly a stuck deadman, until it turns up, and a
     * resync is a valid state to hold at any point.
     */
    if (dropped) {
        gp_resync(gp);
        consumed++;
    }
    return consumed;
}

/* ------------------------------------------------------------------ state -- */

double gp_axis_signed(const gp_t *gp, int ax, double deadzone)
{
    if (!gp || ax < 0 || ax >= GP_AX_COUNT) return 0.0;
    const gp_axis_t *a = &gp->axis[ax];
    if (!a->present) return 0.0;

    double mn = (double)a->min, mx = (double)a->max;
    double centre = (mn + mx) / 2.0;
    double half   = (mx - mn) / 2.0;
    if (half <= 0.0) return 0.0;

    double v = ((double)a->value - centre) / half;   /* -1 .. 1 */
    if (v > 1.0) v = 1.0;
    if (v < -1.0) v = -1.0;

    /*
     * The kernel reports each axis's own flat region — the span it considers
     * centred — from EVIOCGABS. Honour it as a floor on the caller's figure: a
     * pad whose sticks rest noisily says so itself, and overriding that with
     * one hard-coded number for every device is how a resting stick creeps.
     * The caller's value still wins when it is the stricter of the two.
     */
    double dz = deadzone;
    if (a->flat > 0) {
        /*
         * A device claiming a flat region that swallows most of its own travel
         * is describing something unusable, and taking it at face value would
         * kill the stick. Cap what will be accepted from the device rather
         * than either trusting it blindly or — as this did — discarding an
         * over-large figure entirely and silently falling back to the caller's,
         * which contradicts the floor this is documented to be.
         */
        const double KERNEL_DZ_MAX = 0.5;
        double kernel_dz = (double)a->flat / half;
        if (kernel_dz > KERNEL_DZ_MAX) kernel_dz = KERNEL_DZ_MAX;
        if (kernel_dz > dz) dz = kernel_dz;
    }

    if (dz > 0.0 && dz < 1.0) {
        double mag = v < 0 ? -v : v;
        if (mag <= dz) return 0.0;
        /* Rescale so the usable range still reaches full deflection. */
        double scaled = (mag - dz) / (1.0 - dz);
        v = (v < 0) ? -scaled : scaled;
    }
    return v;
}

double gp_axis_unit(const gp_t *gp, int ax)
{
    if (!gp || ax < 0 || ax >= GP_AX_COUNT) return 0.0;
    const gp_axis_t *a = &gp->axis[ax];
    if (!a->present) return 0.0;

    double span = (double)a->max - (double)a->min;
    if (span <= 0.0) return 0.0;

    double v = ((double)a->value - (double)a->min) / span;   /* 0 .. 1 */
    if (v > 1.0) v = 1.0;
    if (v < 0.0) v = 0.0;
    return v;
}

int gp_button(const gp_t *gp, int btn)
{
    if (!gp || btn < 0 || btn >= GP_BTN_COUNT) return 0;
    return gp->button[btn] ? 1 : 0;
}

int gp_button_pressed(const gp_t *gp, int btn)
{
    if (!gp || btn < 0 || btn >= GP_BTN_COUNT) return 0;
    return gp->button[btn] && !gp->button_prev[btn];
}

void gp_latch_buttons(gp_t *gp)
{
    if (!gp) return;
    memcpy(gp->button_prev, gp->button, sizeof(gp->button));
}
