/*
 * sbgc_gui_config.h — recover sane defaults from an existing SimpleBGC GUI
 * installation so the tool needs no command-line arguments.
 *
 * The vendor GUI already knows which port and baud were last used, and its
 * exported .profile files describe the mount. Re-deriving that by hand, or
 * re-typing it on every run, is pointless when the answers are sitting on
 * disk. Everything here is best-effort: if nothing is found, built-in
 * defaults apply and the tool still starts.
 *
 * This module only ever READS files. It never writes to the vendor GUI's
 * configuration.
 */

#ifndef SBGC_GUI_CONFIG_H
#define SBGC_GUI_CONFIG_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SBGC_GUI_PATH_MAX 512

typedef struct {
    /* Where the discovered SimpleBGC GUI lives; empty when none was found. */
    char install_dir[SBGC_GUI_PATH_MAX];
    char properties_path[SBGC_GUI_PATH_MAX];
    char profile_path[SBGC_GUI_PATH_MAX];

    /* From conf/bgc.properties. */
    int  have_port;
    char port[128];             /* last.used.port    */
    int  have_baud;
    int  baud;                  /* latest.serial.baud, mapped to a real rate */

    /* From the exported .profile file — the mount description for profile 1.
     * Angles
     * are degrees. `have_limits` is 0 when no profile file was readable. */
    int    have_limits;
    double roll_min, roll_max;
    double pitch_min, pitch_max;
    double yaw_min, yaw_max;

    int    have_motor_cfg;
    int    poles[3];            /* ROLL, PITCH, YAW */
    int    invert[3];

    /* A short note explaining what was and was not found, for the UI. */
    char   summary[512];
} sbgc_gui_config_t;

/*
 * Search the usual places for a SimpleBGC GUI install and populate `out`.
 * `extra_dir` may name an install directory explicitly (or be NULL).
 * Always returns 0 — absence of a GUI install is a normal outcome, not an
 * error. Inspect the have_* flags to see what was recovered.
 */
int sbgc_gui_config_discover(sbgc_gui_config_t *out, const char *extra_dir);

/*
 * Resolve a serial port that may have moved.
 *
 * Unplugging a CH340 and plugging it back in re-enumerates it: /dev/ttyUSB0
 * becomes /dev/ttyUSB1, and anything holding the old path just sees a device
 * that no longer exists. When `want` is missing, this looks in
 * /dev/serial/by-id/ — those names are derived from the adapter itself and
 * survive re-plugging — and returns the first USB serial device found.
 *
 * Writes the path to use into `out`. Returns 1 if that is `want` unchanged,
 * 2 if a different device was substituted, or 0 if nothing usable was found
 * (in which case `out` holds `want`, so the caller still reports a sensible
 * error).
 */
int sbgc_gui_config_resolve_port(const char *want, char *out, size_t out_cap);

#define SBGC_PORT_LIST_MAX 16
#define SBGC_PORT_NAME_MAX 256

typedef struct {
    char path[SBGC_PORT_NAME_MAX];   /* what to open                        */
    char label[SBGC_PORT_NAME_MAX];  /* what to show a human                */
    int  stable;                     /* 1 when this is a by-id path         */
} sbgc_port_t;

/*
 * List the serial ports currently present. by-id entries are listed with the
 * adapter's own name and marked stable, because those are the ones worth
 * choosing: they do not change when the device is re-plugged.
 * Returns how many entries were written.
 */
int sbgc_gui_config_list_ports(sbgc_port_t *out, int max);

#ifdef __cplusplus
}
#endif

#endif /* SBGC_GUI_CONFIG_H */
