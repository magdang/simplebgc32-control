/*
 * sbgc_params.h — read-only decoding of SimpleBGC board configuration and
 * realtime telemetry.
 *
 * Every command declared here is a QUERY. Nothing in this header, or in its
 * implementation, can move the gimbal or change board state: there is no
 * CMD_WRITE_PARAMS, no CMD_CONTROL, no motor command. That is deliberate —
 * the GUI links against this and must be incapable of actuating hardware.
 *
 * ---------------------------------------------------------------- offsets --
 * The field offsets below were not taken on faith from the spec. They were
 * verified against this board (SBGC 3.1, firmware 2.63b0) by cross-checking a
 * live CMD_READ_PARAMS_3 response against the same profile exported by the
 * SimpleBGC GUI to XML. Fields that matched exactly on both sides:
 *
 *   MENU_BTN_CMD_1..5 / _LONG   1,2,3,0,0 / 7
 *   MOTOR_OUTPUT                4,7,3
 *   BAT_THRESHOLD_ALARM         -1080
 *   BAT_THRESHOLD_MOTORS        -990
 *   BAT_COMP_REF                -1260
 *   FOLLOW_OFFSET               -1,-1,10
 *   FOLLOW_SPEED / FOLLOW_LPF   10,10,10 / 3,3,3
 *   FOLLOW_ROLL_MIX_START/RANGE 34 / 20
 *   ADAPTIVE_PID thr/rate/recov 20 / 50 / 6
 *   PROFILE_FLAGS1              144
 *   GYRO_TRUST / PWM_FREQ       30 / 2
 *   SERIAL_SPEED                0  (= 115200, matching the working link)
 *   AXIS_TOP / FRAME_AXIS_*     3 / 3,1
 *
 * Ten bytes in the RC_MEMORY / SERVO_OUT region (offsets 107..116) could not
 * be split unambiguously between those two fields, so they are NOT decoded.
 * They are exposed as raw bytes instead of being guessed at. Anything this
 * file reports, it can justify.
 */

#ifndef SBGC_PARAMS_H
#define SBGC_PARAMS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Read-only command IDs. */
enum {
    SBGC_CMD_READ_PARAMS_3   = 21,
    SBGC_CMD_REALTIME_DATA_3 = 23,
    SBGC_CMD_BOARD_INFO_3    = 20
};

/* Exact payload sizes as returned by firmware 2.63b0. Checked, never assumed. */
#define SBGC_PARAMS_3_LEN     134
#define SBGC_REALTIME_3_LEN    63
#define SBGC_BOARD_INFO_LEN    18

#define SBGC_PARAM_AXES 3   /* ROLL, PITCH, YAW — same order as sbgc_api.h */

/* ----------------------------------------------------------- system error -- */
/*
 * SYSTEM_ERROR bits, from CMD_REALTIME_DATA_3 offset 14.
 *
 * The single-byte ERROR_CODE at offset 54 that this file used to be the only
 * consumer of is marked "deprecated, replaced by the SYSTEM_ERROR variable" in
 * the protocol specification. It is still decoded, because an older firmware
 * may be the only thing that fills it in, but SYSTEM_ERROR is what anything
 * new should report against: it names which fault the board is in, where the
 * deprecated byte only says that there is one.
 */
enum {
    SBGC_ERR_NO_SENSOR      = 1u << 0,
    SBGC_ERR_CALIB_ACC      = 1u << 1,
    SBGC_ERR_SET_POWER      = 1u << 2,
    SBGC_ERR_CALIB_POLES    = 1u << 3,
    SBGC_ERR_PROTECTION     = 1u << 4,
    SBGC_ERR_SERIAL         = 1u << 5,
    SBGC_ERR_LOW_BAT1       = 1u << 6,
    SBGC_ERR_LOW_BAT2       = 1u << 7,
    SBGC_ERR_GUI_VERSION    = 1u << 8,
    SBGC_ERR_MISS_STEPS     = 1u << 9,
    SBGC_ERR_SYSTEM         = 1u << 10,
    SBGC_ERR_EMERGENCY_STOP = 1u << 11
};

/*
 * Name the lowest set bit of a SYSTEM_ERROR word, or NULL when it is zero.
 * Returns a static string; never allocates. Reporting one cause rather than a
 * list is deliberate — these faults cascade, and the first one is the one
 * worth acting on.
 */
const char *sbgc_system_error_name(uint16_t system_error);

/* ------------------------------------------------------------- raw sensor -- */
/*
 * Wire units for the raw IMU samples at the head of CMD_REALTIME_DATA_3.
 * Both are quoted directly from the protocol specification.
 */
#define SBGC_ACC_UNIT_G       (1.0 / 512.0)    /* G per LSB       */
#define SBGC_GYRO_UNIT_DEGS   0.06103701895    /* deg/s per LSB   */

/* --------------------------------------------------------- profile params -- */

typedef struct {
    uint8_t p, i, d;
    uint8_t power;
    uint8_t invert;
    uint8_t poles;
} sbgc_axis_pid_t;

typedef struct {
    int16_t rc_min_angle;   /* degrees */
    int16_t rc_max_angle;   /* degrees */
    uint8_t rc_mode;
    uint8_t rc_lpf;
    uint8_t rc_speed;
    int8_t  rc_follow;
} sbgc_axis_rc_t;

typedef struct {
    uint8_t profile_id;             /* 0-based: 0 = "Profile 1" in the GUI */

    sbgc_axis_pid_t pid[SBGC_PARAM_AXES];
    uint8_t         acc_limiter_all;
    int8_t          ext_fc_gain[2];
    sbgc_axis_rc_t  rc[SBGC_PARAM_AXES];

    uint8_t  gyro_trust;
    uint8_t  use_model;
    uint8_t  pwm_freq;
    uint8_t  serial_speed;          /* index; 0 = 115200 */

    int8_t   rc_trim[SBGC_PARAM_AXES];
    uint8_t  rc_deadband;
    uint8_t  rc_expo_rate;
    uint8_t  rc_virt_mode;

    uint8_t  follow_mode;
    uint8_t  follow_deadband;
    uint8_t  follow_expo_rate;
    int8_t   follow_offset[SBGC_PARAM_AXES];

    int8_t   axis_top, axis_right;
    int8_t   frame_axis_top, frame_axis_right;
    uint8_t  frame_imu_pos;

    uint8_t  gyro_deadband;
    uint8_t  gyro_sens;
    uint8_t  i2c_speed_fast;
    uint8_t  skip_gyro_calib;

    uint8_t  menu_btn_cmd[5];
    uint8_t  menu_btn_cmd_long;
    uint8_t  motor_output[SBGC_PARAM_AXES];

    int16_t  bat_threshold_alarm;   /* 0.01 V; negative = disabled */
    int16_t  bat_threshold_motors;
    int16_t  bat_comp_ref;
    uint8_t  beeper_modes;

    uint8_t  follow_roll_mix_start;
    uint8_t  follow_roll_mix_range;
    uint8_t  booster_power[SBGC_PARAM_AXES];
    uint8_t  follow_speed[SBGC_PARAM_AXES];
    uint8_t  frame_angle_from_motors;

    /* Offsets 107..116 — RC_MEMORY / SERVO_OUT. The split between these two
     * fields could not be pinned down, so the raw bytes are passed through
     * rather than decoded into fields that might be wrong. */
    uint8_t  undecoded_107_116[10];

    uint8_t  servo_rate;
    uint8_t  adaptive_pid_enabled;
    uint8_t  adaptive_pid_threshold;
    uint8_t  adaptive_pid_rate;
    uint8_t  adaptive_pid_recovery_factor;
    uint8_t  follow_lpf[SBGC_PARAM_AXES];

    uint16_t general_flags1;
    uint16_t profile_flags1;
    uint8_t  spektrum_mode;
    uint8_t  order_of_axes;
    uint8_t  euler_order;
    uint8_t  cur_imu;
    uint8_t  cur_profile_id;
} sbgc_params_t;

/*
 * Decode a CMD_READ_PARAMS_3 payload. Returns 0 on success, -1 if the length
 * is not exactly SBGC_PARAMS_3_LEN — a short or long payload means a firmware
 * whose layout this file has not been verified against, and is refused rather
 * than misparsed.
 */
int sbgc_parse_params_3(const uint8_t *payload, size_t len, sbgc_params_t *out);

/* -------------------------------------------------------------- realtime -- */

/*
 * The subset of CMD_REALTIME_DATA_3 that was confirmed against this board.
 * imu_deg and target_deg were validated by comparing them with the
 * independently-parsed CMD_GET_ANGLES response taken moments apart; both
 * agreed to within a sample.
 */
typedef struct {
    /*
     * imu_deg is wrapped into (-180, 180] so the UI never shows an angle no
     * mount could reach. imu_units is the board's own raw count, NOT wrapped.
     *
     * Anything that becomes a command target must use imu_units. The board's
     * yaw is a continuous count, so feeding a wrapped display value back as an
     * absolute angle can name a target a full revolution away — and the gimbal
     * will dutifully drive all the way round to reach it.
     */
    double   imu_deg[SBGC_PARAM_AXES];
    int16_t  imu_units[SBGC_PARAM_AXES];
    double   frame_imu_deg[SBGC_PARAM_AXES];
    double   target_deg[SBGC_PARAM_AXES];

    /*
     * Raw sensor samples, in the board's own units — deliberately NOT scaled
     * here. Scaling them means choosing a coordinate convention, and the
     * specification says the accelerometer is "expressed in END coordinate
     * system, sign is inverted", which is not this file's business to reconcile
     * with whatever frame a caller works in. Convert with SBGC_ACC_UNIT_G and
     * SBGC_GYRO_UNIT_DEGS at the point where the frame is known.
     *
     * Indexed by SBGC_ROLL / SBGC_PITCH / SBGC_YAW like everything else here.
     */
    int16_t  acc_raw[SBGC_PARAM_AXES];
    int16_t  gyro_raw[SBGC_PARAM_AXES];

    int16_t  rc_data[6];        /* ROLL,PITCH,YAW,CMD,FC_ROLL,FC_PITCH      */
    int      rc_signal_present; /* 0 when every channel reads the -10000    */
                                /* "no signal" sentinel                     */

    uint16_t serial_err_cnt;
    uint16_t system_error;      /* SBGC_ERR_* bits; 0 = no error            */
    uint8_t  system_sub_error;  /* emergency-stop reason, Appendix E        */

    uint16_t cycle_time_us;
    uint16_t i2c_error_count;
    uint8_t  error_code;        /* deprecated by the board; see system_error */
    double   battery_volts;     /* BAT_LEVEL is 0.01 V per LSB              */
    uint8_t  rt_data_flags;
    uint8_t  cur_imu;
    uint8_t  cur_profile;       /* 0-based                                   */
    uint8_t  motor_power[SBGC_PARAM_AXES];

    int      motors_on;         /* decoded from rt_data_flags bit 0          */
} sbgc_realtime_t;

/* Returns 0 on success, -1 if len != SBGC_REALTIME_3_LEN. */
int sbgc_parse_realtime_3(const uint8_t *payload, size_t len,
                          sbgc_realtime_t *out);

/* ------------------------------------------------------------ board info -- */

typedef struct {
    uint8_t  board_ver_major;   /* 31 -> 3.1  */
    uint8_t  board_ver_minor;
    uint16_t firmware_ver;      /* 2630 -> 2.63 b0 */
    uint8_t  firmware_major;
    uint8_t  firmware_minor;
    uint8_t  firmware_beta;
    uint8_t  state_flags1;
    uint16_t board_features;
    uint8_t  connection_flag;
} sbgc_board_info_t;

/* Returns 0 on success, -1 if len != SBGC_BOARD_INFO_LEN. */
int sbgc_parse_board_info(const uint8_t *payload, size_t len,
                          sbgc_board_info_t *out);

/* Human-readable name for a serial-speed index (0 = 115200). */
const char *sbgc_serial_speed_name(uint8_t index);


#ifdef __cplusplus
}
#endif

#endif /* SBGC_PARAMS_H */
