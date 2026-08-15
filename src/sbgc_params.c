/*
 * sbgc_params.c — read-only decoding of SimpleBGC configuration and telemetry.
 * See include/sbgc_params.h for how the offsets were verified.
 */

#include "sbgc_params.h"
#include "sbgc_api.h"

#include <string.h>

/* ------------------------------------------------------------- accessors -- */

static int16_t rd_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/*
 * The board reports yaw as an unwrapped 14-bit count, so a reading can come
 * back as 287 deg where -73 deg is meant. Fold everything into (-180, 180] so
 * the UI never shows an angle no mount could reach.
 */
static double wrap180(double deg)
{
    while (deg > 180.0)  deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
}

/* -------------------------------------------------------- profile params -- */

int sbgc_parse_params_3(const uint8_t *b, size_t len, sbgc_params_t *out)
{
    if (!b || !out) return -1;
    if (len != SBGC_PARAMS_3_LEN) return -1;   /* checked, never assumed */

    memset(out, 0, sizeof(*out));

    out->profile_id = b[0];

    /* 1..18 — six bytes per axis, ROLL then PITCH then YAW. */
    for (int a = 0; a < SBGC_PARAM_AXES; a++) {
        const uint8_t *p = b + 1 + a * 6;
        out->pid[a].p      = p[0];
        out->pid[a].i      = p[1];
        out->pid[a].d      = p[2];
        out->pid[a].power  = p[3];
        out->pid[a].invert = p[4];
        out->pid[a].poles  = p[5];
    }

    out->acc_limiter_all = b[19];
    out->ext_fc_gain[0]  = (int8_t)b[20];
    out->ext_fc_gain[1]  = (int8_t)b[21];

    /*
     * 22..45 — eight bytes per axis.
     *
     * min == max == 0 means "no constraint", not "a zero-width range". The
     * 2.6x manual, RC Settings: "To disable constraints, set min=max=0."
     * Callers must treat that pair as absent rather than as a limit.
     *
     * The manual also pins the reference: ROLL and PITCH limits are absolute,
     * measured against the ground, so they are comparable with imu_deg.
     */
    for (int a = 0; a < SBGC_PARAM_AXES; a++) {
        const uint8_t *p = b + 22 + a * 8;
        out->rc[a].rc_min_angle = rd_i16(p);
        out->rc[a].rc_max_angle = rd_i16(p + 2);
        out->rc[a].rc_mode      = p[4];
        out->rc[a].rc_lpf       = p[5];
        out->rc[a].rc_speed     = p[6];
        out->rc[a].rc_follow    = (int8_t)p[7];
    }

    out->gyro_trust   = b[46];
    out->use_model    = b[47];
    out->pwm_freq     = b[48];
    out->serial_speed = b[49];

    for (int a = 0; a < SBGC_PARAM_AXES; a++) out->rc_trim[a] = (int8_t)b[50 + a];
    out->rc_deadband  = b[53];
    out->rc_expo_rate = b[54];
    out->rc_virt_mode = b[55];
    /* 56..63 — RC channel mapping and FC mixing; not surfaced by the GUI. */

    out->follow_mode      = b[64];
    out->follow_deadband  = b[65];
    out->follow_expo_rate = b[66];
    for (int a = 0; a < SBGC_PARAM_AXES; a++)
        out->follow_offset[a] = (int8_t)b[67 + a];

    out->axis_top          = (int8_t)b[70];
    out->axis_right        = (int8_t)b[71];
    out->frame_axis_top    = (int8_t)b[72];
    out->frame_axis_right  = (int8_t)b[73];
    out->frame_imu_pos     = b[74];

    out->gyro_deadband   = b[75];
    out->gyro_sens       = b[76];
    out->i2c_speed_fast  = b[77];
    out->skip_gyro_calib = b[78];
    /* 79..81 — RC_CMD_LOW/MID/HIGH. */

    memcpy(out->menu_btn_cmd, b + 82, 5);
    out->menu_btn_cmd_long = b[87];
    memcpy(out->motor_output, b + 88, SBGC_PARAM_AXES);

    out->bat_threshold_alarm  = rd_i16(b + 91);
    out->bat_threshold_motors = rd_i16(b + 93);
    out->bat_comp_ref         = rd_i16(b + 95);
    out->beeper_modes         = b[97];

    out->follow_roll_mix_start = b[98];
    out->follow_roll_mix_range = b[99];
    memcpy(out->booster_power, b + 100, SBGC_PARAM_AXES);
    memcpy(out->follow_speed,  b + 103, SBGC_PARAM_AXES);
    out->frame_angle_from_motors = b[106];

    memcpy(out->undecoded_107_116, b + 107, 10);

    out->servo_rate                   = b[117];
    out->adaptive_pid_enabled         = b[118];
    out->adaptive_pid_threshold       = b[119];
    out->adaptive_pid_rate            = b[120];
    out->adaptive_pid_recovery_factor = b[121];
    memcpy(out->follow_lpf, b + 122, SBGC_PARAM_AXES);

    out->general_flags1 = rd_u16(b + 125);
    out->profile_flags1 = rd_u16(b + 127);
    out->spektrum_mode  = b[129];
    out->order_of_axes  = b[130];
    out->euler_order    = b[131];
    out->cur_imu        = b[132];
    out->cur_profile_id = b[133];

    return 0;
}

/* -------------------------------------------------------------- realtime -- */

/* The board reports an unconnected RC input as this sentinel, not as zero. */
#define RC_NO_SIGNAL (-10000)

int sbgc_parse_realtime_3(const uint8_t *b, size_t len, sbgc_realtime_t *out)
{
    if (!b || !out) return -1;
    if (len != SBGC_REALTIME_3_LEN) return -1;

    memset(out, 0, sizeof(*out));

    /*
     * 0..11 are the raw IMU samples, interleaved per axis as
     * (ACC, GYRO) x (ROLL, PITCH, YAW) — not two contiguous arrays. Passed
     * through unscaled; see the struct for why.
     */
    for (int a = 0; a < SBGC_PARAM_AXES; a++) {
        out->acc_raw[a]  = rd_i16(b + a * 4);
        out->gyro_raw[a] = rd_i16(b + a * 4 + 2);
    }

    out->serial_err_cnt   = rd_u16(b + 12);
    out->system_error     = rd_u16(b + 14);
    out->system_sub_error = b[16];
    /* 17..19 are RESERVED. */

    int any_signal = 0;
    for (int i = 0; i < 6; i++) {
        out->rc_data[i] = rd_i16(b + 20 + i * 2);
        if (out->rc_data[i] != RC_NO_SIGNAL) any_signal = 1;
    }
    out->rc_signal_present = any_signal;

    for (int a = 0; a < SBGC_PARAM_AXES; a++) {
        out->imu_units[a]     = rd_i16(b + 32 + a * 2);
        out->imu_deg[a]       = wrap180(sbgc_units_to_deg(out->imu_units[a]));
        out->frame_imu_deg[a] = wrap180(sbgc_units_to_deg(rd_i16(b + 38 + a * 2)));
        out->target_deg[a]    = wrap180(sbgc_units_to_deg(rd_i16(b + 44 + a * 2)));
    }

    out->cycle_time_us   = rd_u16(b + 50);
    out->i2c_error_count = rd_u16(b + 52);
    out->error_code      = b[54];   /* deprecated; system_error supersedes it */
    out->battery_volts   = (double)rd_u16(b + 55) * 0.01;
    out->rt_data_flags   = b[57];
    out->cur_imu         = b[58];
    out->cur_profile     = b[59];
    memcpy(out->motor_power, b + 60, SBGC_PARAM_AXES);

    out->motors_on = (out->rt_data_flags & 0x01) ? 1 : 0;

    return 0;
}

/* ------------------------------------------------------------ board info -- */

int sbgc_parse_board_info(const uint8_t *b, size_t len, sbgc_board_info_t *out)
{
    if (!b || !out) return -1;
    if (len != SBGC_BOARD_INFO_LEN) return -1;

    memset(out, 0, sizeof(*out));

    out->board_ver_major = (uint8_t)(b[0] / 10);
    out->board_ver_minor = (uint8_t)(b[0] % 10);

    out->firmware_ver   = rd_u16(b + 1);
    out->firmware_major = (uint8_t)(out->firmware_ver / 1000);
    out->firmware_minor = (uint8_t)((out->firmware_ver % 1000) / 10);
    out->firmware_beta  = (uint8_t)(out->firmware_ver % 10);

    out->state_flags1    = b[3];
    out->board_features  = rd_u16(b + 4);
    out->connection_flag = b[6];

    return 0;
}

/* ----------------------------------------------------------- serial speed -- */

/*
 * Index order is the SimpleBGC GUI's own dropdown. Index 0 was confirmed to be
 * 115200 on this board: the profile reports SERIAL_SPEED 0 and the link only
 * answers at 115200.
 */
static const struct { const char *name; int baud; } SERIAL_SPEEDS[] = {
    { "115200", 115200 },
    { "57600",   57600 },
    { "38400",   38400 },
    { "19200",   19200 },
    { "9600",     9600 },
    { "256000", 256000 }
};

const char *sbgc_serial_speed_name(uint8_t index)
{
    if (index < sizeof(SERIAL_SPEEDS) / sizeof(SERIAL_SPEEDS[0]))
        return SERIAL_SPEEDS[index].name;
    return "unknown";
}


/* --------------------------------------------------------- error codes -- */
/*
 * Two different error fields arrive in REALTIME_DATA_3, and only one of them
 * can honestly be named.
 *
 * SYSTEM_ERROR (offset 14) is a bitmask whose bits the Serial API
 * specification names individually, so naming them here is transcription, not
 * inference. That satisfies this file's standing rule that nothing is reported
 * which cannot be justified.
 *
 * ERROR_CODE (offset 54) is the older one-byte field, and the spec marks it
 * "deprecated, replaced by the SYSTEM_ERROR variable". Turning its value into
 * a name would need a numeric table this project has not checked against a
 * board, and a confidently wrong fault name sends an operator to inspect the
 * wrong part of the gimbal. So it is still decoded, still reported raw, and
 * still not named.
 */

static const struct { uint16_t bit; const char *name; } SYSTEM_ERRORS[] = {
    { SBGC_ERR_NO_SENSOR,      "no sensor" },
    { SBGC_ERR_CALIB_ACC,      "accelerometer not calibrated" },
    { SBGC_ERR_SET_POWER,      "cannot set motor power" },
    { SBGC_ERR_CALIB_POLES,    "motor poles not calibrated" },
    { SBGC_ERR_PROTECTION,     "protection triggered" },
    { SBGC_ERR_SERIAL,         "serial error" },
    { SBGC_ERR_LOW_BAT1,       "battery below first threshold" },
    { SBGC_ERR_LOW_BAT2,       "battery below second threshold" },
    { SBGC_ERR_GUI_VERSION,    "GUI version mismatch" },
    { SBGC_ERR_MISS_STEPS,     "motor missed steps" },
    { SBGC_ERR_SYSTEM,         "system error" },
    { SBGC_ERR_EMERGENCY_STOP, "emergency stop" }
};

const char *sbgc_system_error_name(uint16_t system_error)
{
    if (system_error == 0) return NULL;

    for (size_t i = 0; i < sizeof(SYSTEM_ERRORS) / sizeof(SYSTEM_ERRORS[0]); i++)
        if (system_error & SYSTEM_ERRORS[i].bit)
            return SYSTEM_ERRORS[i].name;

    /*
     * A bit outside the documented set. Saying so is more useful than saying
     * nothing: it means the board knows about a fault this build does not.
     */
    return "unknown error";
}
