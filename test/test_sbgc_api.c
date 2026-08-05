/*
 * test_sbgc_api.c — protocol tests.
 *
 * The CMD_CONTROL cases assert byte-for-byte equality with the payload hex
 * dumps published by BaseCam in "SBGC32 API cmd examples". If these pass, the
 * frames this tool puts on the wire are the frames the vendor documents.
 */

#include "sbgc_api.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;
static int checks = 0;

static void expect_bytes(const char *what,
                         const uint8_t *got, size_t got_len,
                         const uint8_t *want, size_t want_len)
{
    checks++;
    if (got_len != want_len || memcmp(got, want, want_len) != 0) {
        failures++;
        printf("FAIL %s\n  want:", what);
        for (size_t i = 0; i < want_len; i++) printf(" %02X", want[i]);
        printf("\n  got :");
        for (size_t i = 0; i < got_len; i++) printf(" %02X", got[i]);
        printf("\n");
    } else {
        printf("ok   %s\n", what);
    }
}

static void expect_i16(const char *what, int16_t got, int16_t want)
{
    checks++;
    if (got != want) {
        failures++;
        printf("FAIL %s: want %d, got %d\n", what, want, got);
    } else {
        printf("ok   %s (%d)\n", what, got);
    }
}

static void expect_true(const char *what, int cond)
{
    checks++;
    if (!cond) { failures++; printf("FAIL %s\n", what); }
    else printf("ok   %s\n", what);
}

/* Build a CMD_CONTROL payload and compare against the vendor bytes. */
static void check_control(const char *what,
                          uint8_t m_roll, uint8_t m_pitch, uint8_t m_yaw,
                          int16_t s_roll, int16_t a_roll,
                          int16_t s_pitch, int16_t a_pitch,
                          int16_t s_yaw, int16_t a_yaw,
                          const uint8_t *want)
{
    uint8_t mode[3]  = { m_roll, m_pitch, m_yaw };
    int16_t speed[3] = { s_roll, s_pitch, s_yaw };
    int16_t angle[3] = { a_roll, a_pitch, a_yaw };
    uint8_t out[15];

    int n = sbgc_build_control_payload(out, sizeof(out), mode, speed, angle);
    expect_true("payload length is 15", n == 15);
    expect_bytes(what, out, 15, want, 15);
}

int main(void)
{
    printf("== unit conversion (vendor worked values) ==\n");
    expect_i16("45 deg -> 2048",      sbgc_deg_to_units(45.0),   2048);
    expect_i16("-90 deg -> -4096",    sbgc_deg_to_units(-90.0), -4096);
    expect_i16("-45 deg -> -2048",    sbgc_deg_to_units(-45.0), -2048);
    expect_i16("5 deg/s -> 41",       sbgc_degs_to_units(5.0),     41);
    expect_i16("90 deg/s -> 737",     sbgc_degs_to_units(90.0),   737);
    expect_i16("0 deg/s -> 0",        sbgc_degs_to_units(0.0),      0);
    expect_true("round trip 45 deg",
                fabs(sbgc_units_to_deg(2048) - 45.0) < 1e-9);
    expect_true("saturates high", sbgc_deg_to_units(1e9) == 32767);
    expect_true("saturates low",  sbgc_deg_to_units(-1e9) == -32768);

    printf("\n== CMD_CONTROL payloads vs vendor hex dumps ==\n");

    /* "Rotate YAW 45 degrees right with the default speed."
     * 00 00 02 00 00 00 00 00 00 00 00 00 00 00 08 */
    {
        const uint8_t want[15] = {
            0x00, 0x00, 0x02,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x08
        };
        check_control("yaw 45 deg, default speed",
                      SBGC_MODE_NO_CONTROL, SBGC_MODE_NO_CONTROL, SBGC_MODE_ANGLE,
                      0, 0,  0, 0,  0, sbgc_deg_to_units(45.0),
                      want);
    }

    /* "Rotate PITCH 90 degrees up with the speed 5 deg./sec."
     * 00 02 00 00 00 00 00 29 00 00 F0 00 00 00 00 */
    {
        const uint8_t want[15] = {
            0x00, 0x02, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0x29, 0x00, 0x00, 0xF0,
            0x00, 0x00, 0x00, 0x00
        };
        check_control("pitch -90 deg at 5 deg/s",
                      SBGC_MODE_NO_CONTROL, SBGC_MODE_ANGLE, SBGC_MODE_NO_CONTROL,
                      0, 0,
                      sbgc_degs_to_units(5.0), sbgc_deg_to_units(-90.0),
                      0, 0,
                      want);
    }

    /* "Rotate PITCH 45 degrees up relative to the frame with the speed 90
     *  deg./sec ... AUTO_TASK."
     * 00 45 00 00 00 00 00 E1 02 00 F8 00 00 00 00 */
    {
        const uint8_t want[15] = {
            0x00, 0x45, 0x00,
            0x00, 0x00, 0x00, 0x00,
            0xE1, 0x02, 0x00, 0xF8,
            0x00, 0x00, 0x00, 0x00
        };
        check_control("pitch -45 deg rel frame at 90 deg/s, auto task",
                      SBGC_MODE_NO_CONTROL,
                      SBGC_MODE_ANGLE_REL_FRAME | SBGC_CTRL_FLAG_AUTO_TASK,
                      SBGC_MODE_NO_CONTROL,
                      0, 0,
                      sbgc_degs_to_units(90.0), sbgc_deg_to_units(-45.0),
                      0, 0,
                      want);
    }

    /* "Home position" — 42 45 45 then all zeros. */
    {
        const uint8_t want[15] = {
            0x42, 0x45, 0x45,
            0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0
        };
        check_control("home position mode bytes",
                      SBGC_MODE_ANGLE           | SBGC_CTRL_FLAG_AUTO_TASK,
                      SBGC_MODE_ANGLE_REL_FRAME | SBGC_CTRL_FLAG_AUTO_TASK,
                      SBGC_MODE_ANGLE_REL_FRAME | SBGC_CTRL_FLAG_AUTO_TASK,
                      0, 0, 0, 0, 0, 0, want);
    }

    /* "Leveled position" — 42 42 45 then all zeros. */
    {
        const uint8_t want[15] = {
            0x42, 0x42, 0x45,
            0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0
        };
        check_control("level position mode bytes",
                      SBGC_MODE_ANGLE           | SBGC_CTRL_FLAG_AUTO_TASK,
                      SBGC_MODE_ANGLE           | SBGC_CTRL_FLAG_AUTO_TASK,
                      SBGC_MODE_ANGLE_REL_FRAME | SBGC_CTRL_FLAG_AUTO_TASK,
                      0, 0, 0, 0, 0, 0, want);
    }

    printf("\n== frame framing and checksums ==\n");

    /* Vendor example from the protocol spec: read Profile2 is
     * 3E 52 01 53 01 01 (cmd 0x52, payload {0x01}). */
    {
        const uint8_t payload[1] = { 0x01 };
        const uint8_t want[6] = { 0x3E, 0x52, 0x01, 0x53, 0x01, 0x01 };
        uint8_t out[16];
        int n = sbgc_encode_frame(out, sizeof(out), 0x52, payload, 1);
        expect_true("encode returned 6", n == 6);
        expect_bytes("spec example: read Profile2", out, (size_t)(n < 0 ? 0 : n),
                     want, 6);
    }

    /* Zero-payload command (CMD_GET_ANGLES, #73 = 0x49). */
    {
        const uint8_t want[5] = { 0x3E, 0x49, 0x00, 0x49, 0x00 };
        uint8_t out[16];
        int n = sbgc_encode_frame(out, sizeof(out), SBGC_CMD_GET_ANGLES, NULL, 0);
        expect_bytes("CMD_GET_ANGLES frame", out, (size_t)(n < 0 ? 0 : n), want, 5);
    }

    /* Header checksum must wrap modulo 256. */
    {
        uint8_t payload[200];
        memset(payload, 0xFF, sizeof(payload));
        uint8_t out[SBGC_MAX_FRAME];
        int n = sbgc_encode_frame(out, sizeof(out), 200, payload, 200);
        expect_true("large frame encoded", n == 4 + 200 + 1);
        expect_true("header checksum wraps", out[3] == (uint8_t)((200 + 200) & 0xFF));
        expect_true("body checksum wraps",
                    out[4 + 200] == (uint8_t)((200 * 0xFF) & 0xFF));
    }

    /* Buffer too small must fail rather than overflow. */
    {
        uint8_t small[4];
        expect_true("short buffer rejected",
                    sbgc_encode_frame(small, sizeof(small), 1, NULL, 0) == -1);
    }

    printf("\n== parser ==\n");

    /* Round trip: encode then feed byte by byte. */
    {
        uint8_t frame[SBGC_MAX_FRAME];
        const uint8_t payload[3] = { 0xAA, 0xBB, 0xCC };
        int n = sbgc_encode_frame(frame, sizeof(frame), 42, payload, 3);

        sbgc_parser_t p;
        sbgc_parser_init(&p);
        int done = 0;
        uint8_t cmd = 0; const uint8_t *pl = NULL; size_t pl_len = 0;
        for (int i = 0; i < n; i++)
            done = sbgc_parser_push(&p, frame[i], &cmd, &pl, &pl_len);

        expect_true("parser completed a frame", done == 1);
        expect_true("parser cmd id", cmd == 42);
        expect_true("parser payload length", pl_len == 3);
        expect_true("parser payload bytes", pl && memcmp(pl, payload, 3) == 0);
    }

    /* Leading garbage must be skipped and the frame still recovered. */
    {
        uint8_t frame[SBGC_MAX_FRAME];
        const uint8_t payload[2] = { 0x10, 0x20 };
        int n = sbgc_encode_frame(frame, sizeof(frame), 7, payload, 2);

        sbgc_parser_t p;
        sbgc_parser_init(&p);
        const uint8_t junk[5] = { 0x00, 0x99, 0x12, 0xFE, 0x77 };
        uint8_t cmd = 0; const uint8_t *pl = NULL; size_t pl_len = 0;
        int done = 0;
        for (int i = 0; i < 5; i++)
            sbgc_parser_push(&p, junk[i], &cmd, &pl, &pl_len);
        for (int i = 0; i < n; i++)
            done = sbgc_parser_push(&p, frame[i], &cmd, &pl, &pl_len);

        expect_true("resynchronised after garbage", done == 1 && cmd == 7);
    }

    /* A corrupted body checksum must be rejected, not delivered. */
    {
        uint8_t frame[SBGC_MAX_FRAME];
        const uint8_t payload[2] = { 0x10, 0x20 };
        int n = sbgc_encode_frame(frame, sizeof(frame), 7, payload, 2);
        frame[n - 1] ^= 0xFF;

        sbgc_parser_t p;
        sbgc_parser_init(&p);
        uint8_t cmd = 0; const uint8_t *pl = NULL; size_t pl_len = 0;
        int done = 0;
        for (int i = 0; i < n; i++)
            done = sbgc_parser_push(&p, frame[i], &cmd, &pl, &pl_len);

        expect_true("bad body checksum rejected", done == 0);
    }

    printf("\n== CMD_GET_ANGLES decode ==\n");
    {
        uint8_t pl[18];
        memset(pl, 0, sizeof(pl));
        /* ROLL imu = 2048 units = 45 deg */
        pl[0] = 0x00; pl[1] = 0x08;
        /* PITCH imu = -4096 units = -90 deg (offset 6) */
        pl[6] = 0x00; pl[7] = 0xF0;
        /* YAW target speed = 737 units = ~90 deg/s (offset 12+4) */
        pl[16] = 0xE1; pl[17] = 0x02;

        sbgc_angles_t a;
        expect_true("18-byte payload accepted",
                    sbgc_parse_angles(pl, sizeof(pl), &a) == 0);
        expect_true("roll imu = 45 deg",
                    fabs(a.imu_deg[SBGC_ROLL] - 45.0) < 1e-6);
        expect_true("pitch imu = -90 deg",
                    fabs(a.imu_deg[SBGC_PITCH] + 90.0) < 1e-6);
        expect_true("yaw target speed ~= 90 deg/s",
                    fabs(a.target_deg_s[SBGC_YAW] - 90.0) < 0.1);
        expect_true("wrong length rejected",
                    sbgc_parse_angles(pl, 17, &a) == -1);
    }

    printf("\n----------------------------------------\n");
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
