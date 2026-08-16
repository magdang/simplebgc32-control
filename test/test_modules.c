/*
 * test_modules.c — tests for the three modules that had none: the HTTP
 * server, the configuration/telemetry decoder, and the GUI-config discovery.
 *
 * These are the parts a browser, a board and a filesystem talk to directly,
 * so their inputs are the ones this project does not control. Two of the
 * defects fixed on this branch lived in here precisely because nothing
 * exercised them.
 *
 * Everything is checked against a stated source: the payload offsets against
 * the verification recorded in sbgc_params.h, the HTTP behaviour against what
 * httpd.h promises, and the discovery against fixture files written here.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include "httpd.h"
#include "sbgc_api.h"
#include "sbgc_params.h"
#include "sbgc_gui_config.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

/* ----------------------------------------------------------- harness -- */

static int checks = 0, failures = 0;

static void check(const char *what, int cond, const char *extra)
{
    checks++;
    if (cond) {
        printf("ok   %s\n", what);
    } else {
        failures++;
        printf("FAIL %s\n", what);
        if (extra && *extra) printf("     %s\n", extra);
    }
}

static void section(const char *title) { printf("\n== %s ==\n", title); }

/* ------------------------------------------------------- sbgc_params -- */

static void test_board_info(void)
{
    section("CMD_BOARD_INFO decode");

    uint8_t b[SBGC_BOARD_INFO_LEN];
    memset(b, 0, sizeof(b));
    b[0] = 31;                       /* board 3.1                        */
    b[1] = 2630 & 0xFF;              /* firmware 2630 -> 2.63 b0         */
    b[2] = (2630 >> 8) & 0xFF;
    b[3] = 0x05;                     /* state flags                      */
    b[4] = 0x34; b[5] = 0x12;        /* features                         */
    b[6] = 1;                        /* connection flag                  */

    sbgc_board_info_t bi;
    check("a correct-length payload is accepted",
          sbgc_parse_board_info(b, sizeof(b), &bi) == 0, NULL);
    check("board 31 decodes as 3.1",
          bi.board_ver_major == 3 && bi.board_ver_minor == 1, NULL);
    check("firmware 2630 decodes as 2.63 b0",
          bi.firmware_major == 2 && bi.firmware_minor == 63 &&
          bi.firmware_beta == 0, NULL);
    check("features are little-endian", bi.board_features == 0x1234, NULL);
    check("state flags pass through", bi.state_flags1 == 0x05, NULL);

    /* The length is checked rather than assumed — a different layout means a
     * firmware this file has not been verified against. */
    check("a short payload is refused",
          sbgc_parse_board_info(b, sizeof(b) - 1, &bi) == -1, NULL);
    check("a long payload is refused",
          sbgc_parse_board_info(b, sizeof(b) + 1, &bi) == -1, NULL);
    check("a NULL payload is refused",
          sbgc_parse_board_info(NULL, sizeof(b), &bi) == -1, NULL);
}

static void put16(uint8_t *p, int v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(((unsigned)v >> 8) & 0xFF);
}

static void test_realtime(void)
{
    section("CMD_REALTIME_DATA_3 decode");

    uint8_t b[SBGC_REALTIME_3_LEN];
    memset(b, 0, sizeof(b));

    /*
     * Raw IMU samples are interleaved per axis as (ACC, GYRO), not stored as
     * two contiguous arrays. Distinct values per field so a swapped pair or an
     * off-by-one stride cannot pass.
     */
    put16(b + 0,  100);   put16(b + 2,  -200);   /* roll  acc / gyro */
    put16(b + 4,  300);   put16(b + 6,  -400);   /* pitch acc / gyro */
    put16(b + 8,  512);   put16(b + 10,  819);   /* yaw   acc / gyro */

    /*
     * Both of these are deliberately chosen so their HIGH byte is non-zero: a
     * regression to a one-byte read would otherwise decode them correctly and
     * the test would pass. 40000 also exceeds INT16_MAX, so a signed read of
     * the counter shows up as a negative rather than silently agreeing.
     */
    put16(b + 12, (int16_t)40000);         /* serial error count       */
    put16(b + 14, SBGC_ERR_EMERGENCY_STOP);/* 1 << 11 — high byte set  */
    b[16] = 3;                             /* emergency-stop reason    */

    /* No RC receiver: every channel reads the documented sentinel. */
    for (int i = 0; i < 6; i++) put16(b + 20 + i * 2, -10000);

    /* 45 deg on roll, -90 on pitch, and a yaw the board reports unwrapped. */
    put16(b + 32, 2048);            /* roll  45 deg   */
    put16(b + 34, -4096);           /* pitch -90 deg  */
    put16(b + 36, 9102);            /* yaw   ~200 deg */
    put16(b + 50, 1234);            /* cycle time us  */
    put16(b + 52, 7);               /* i2c errors     */
    b[54] = 0;                      /* error code     */
    put16(b + 55, 1234);            /* 12.34 V        */
    b[57] = 0x01;                   /* motors on      */
    b[59] = 2;                      /* profile 3      */

    sbgc_realtime_t rt;
    check("a correct-length payload is accepted",
          sbgc_parse_realtime_3(b, sizeof(b), &rt) == 0, NULL);
    check("no RC signal is detected from the sentinel",
          rt.rc_signal_present == 0, NULL);
    check("motors_on comes from bit 0 of the flags", rt.motors_on == 1, NULL);
    check("battery is 0.01 V per LSB",
          rt.battery_volts > 12.33 && rt.battery_volts < 12.35, NULL);
    check("cycle time passes through", rt.cycle_time_us == 1234, NULL);
    check("i2c error count passes through", rt.i2c_error_count == 7, NULL);
    check("profile index passes through", rt.cur_profile == 2, NULL);

    /*
     * The interleave is the part that is easy to get wrong, so each of the six
     * fields is asserted separately rather than as two arrays.
     */
    check("roll acc/gyro come from the first interleaved pair",
          rt.acc_raw[SBGC_ROLL] == 100 && rt.gyro_raw[SBGC_ROLL] == -200, NULL);
    check("pitch acc/gyro come from the second interleaved pair",
          rt.acc_raw[SBGC_PITCH] == 300 && rt.gyro_raw[SBGC_PITCH] == -400,
          NULL);
    check("yaw acc/gyro come from the third interleaved pair",
          rt.acc_raw[SBGC_YAW] == 512 && rt.gyro_raw[SBGC_YAW] == 819, NULL);

    /*
     * The samples are passed through unscaled, but the documented units have
     * to be usable: 512 counts is exactly 1 G, and 819 counts is ~50 deg/s.
     */
    check("the accelerometer unit is 1/512 G",
          fabs(rt.acc_raw[SBGC_YAW] * SBGC_ACC_UNIT_G - 1.0) < 1e-9, NULL);
    check("the gyroscope unit is ~0.061 deg/s",
          fabs(rt.gyro_raw[SBGC_YAW] * SBGC_GYRO_UNIT_DEGS - 50.0) < 0.05,
          NULL);

    check("the serial error count is decoded from both bytes, unsigned",
          rt.serial_err_cnt == 40000, NULL);
    check("the emergency-stop sub-error is decoded",
          rt.system_sub_error == 3, NULL);

    /*
     * SYSTEM_ERROR is what supersedes the deprecated byte at offset 54, so a
     * board reporting a fault there must be reportable even when that byte is
     * clear — which it is in this payload.
     */
    check("SYSTEM_ERROR is decoded from offset 14, high byte included",
          rt.system_error == SBGC_ERR_EMERGENCY_STOP, NULL);
    check("a deprecated error_code of 0 does not mask a real SYSTEM_ERROR",
          rt.error_code == 0 && rt.system_error != 0, NULL);
    check("a SYSTEM_ERROR bit is named",
          sbgc_system_error_name(rt.system_error) != NULL &&
          strcmp(sbgc_system_error_name(rt.system_error),
                 "emergency stop") == 0, NULL);
    check("no error names nothing", sbgc_system_error_name(0) == NULL, NULL);
    check("the lowest set bit wins when faults cascade",
          strcmp(sbgc_system_error_name(SBGC_ERR_CALIB_ACC |
                                        SBGC_ERR_EMERGENCY_STOP),
                 "accelerometer not calibrated") == 0, NULL);
    check("an undocumented bit is reported rather than ignored",
          strcmp(sbgc_system_error_name(1u << 15), "unknown error") == 0,
          NULL);

    check("roll decodes to 45 deg",
          rt.imu_deg[SBGC_ROLL] > 44.9 && rt.imu_deg[SBGC_ROLL] < 45.1, NULL);
    check("pitch decodes to -90 deg",
          rt.imu_deg[SBGC_PITCH] < -89.9 && rt.imu_deg[SBGC_PITCH] > -90.1,
          NULL);

    /*
     * The raw count is kept alongside the wrapped display value. Everything
     * that has to know where the axis physically is depends on this.
     */
    check("the raw yaw count is preserved unwrapped",
          rt.imu_units[SBGC_YAW] == 9102, NULL);
    check("the displayed yaw is folded into (-180, 180]",
          rt.imu_deg[SBGC_YAW] > -160.1 && rt.imu_deg[SBGC_YAW] < -159.9,
          NULL);

    /* One live channel is enough to say a receiver is present. */
    put16(b + 20, 1500);
    check("a single live channel counts as RC present",
          sbgc_parse_realtime_3(b, sizeof(b), &rt) == 0 &&
          rt.rc_signal_present == 1, NULL);

    check("a wrong-length payload is refused",
          sbgc_parse_realtime_3(b, sizeof(b) - 1, &rt) == -1, NULL);
}

static void test_params(void)
{
    section("CMD_READ_PARAMS_3 decode");

    uint8_t b[SBGC_PARAMS_3_LEN];
    memset(b, 0, sizeof(b));

    b[0] = 1;                       /* profile 2, zero-based            */
    b[1] = 20; b[2] = 5; b[3] = 30; /* roll P I D                       */
    b[4] = 90;                      /* roll power                       */
    b[6] = 14;                      /* roll poles                       */

    /* RC angle limits, 8 bytes per axis from offset 22. Pitch 0..90 is the
     * manual's own example of a camera that may look down but not up. */
    put16(b + 22 + 8 * SBGC_PITCH,     0);
    put16(b + 22 + 8 * SBGC_PITCH + 2, 90);
    put16(b + 22 + 8 * SBGC_YAW,     -170);
    put16(b + 22 + 8 * SBGC_YAW + 2,  170);

    b[46] = 30;                     /* gyro trust, verified value       */
    b[48] = 2;                      /* pwm freq, verified value         */
    b[49] = 2;                      /* serial speed index 2 = 38400     */
    b[78] = 1;                      /* skip gyro calib                  */
    put16(b + 91, -1080);           /* battery alarm, verified value    */
    put16(b + 93,  -990);           /* battery motors-off               */
    b[88] = 4; b[89] = 7; b[90] = 3;/* motor output, verified triple    */
    b[133] = 1;                     /* current profile                  */

    sbgc_params_t p;
    check("a correct-length payload is accepted",
          sbgc_parse_params_3(b, sizeof(b), &p) == 0, NULL);
    check("profile id passes through", p.profile_id == 1, NULL);
    check("roll PID decodes",
          p.pid[SBGC_ROLL].p == 20 && p.pid[SBGC_ROLL].i == 5 &&
          p.pid[SBGC_ROLL].d == 30, NULL);
    check("roll power and poles decode",
          p.pid[SBGC_ROLL].power == 90 && p.pid[SBGC_ROLL].poles == 14, NULL);
    check("pitch RC limits decode",
          p.rc[SBGC_PITCH].rc_min_angle == 0 &&
          p.rc[SBGC_PITCH].rc_max_angle == 90, NULL);
    check("yaw RC limits decode as signed",
          p.rc[SBGC_YAW].rc_min_angle == -170 &&
          p.rc[SBGC_YAW].rc_max_angle == 170, NULL);
    check("gyro trust and pwm freq decode",
          p.gyro_trust == 30 && p.pwm_freq == 2, NULL);
    /* A distinctive value: a zero-filled buffer satisfies == 0 no matter which
     * offset the decoder actually read, so zero would pin nothing. */
    check("serial speed is read from its own offset",
          p.serial_speed == 2 &&
          strcmp(sbgc_serial_speed_name(p.serial_speed), "38400") == 0, NULL);
    check("battery thresholds decode as signed",
          p.bat_threshold_alarm == -1080 && p.bat_threshold_motors == -990,
          NULL);
    check("motor output triple decodes",
          p.motor_output[0] == 4 && p.motor_output[1] == 7 &&
          p.motor_output[2] == 3, NULL);
    check("skip_gyro_calib decodes", p.skip_gyro_calib == 1, NULL);
    check("current profile id decodes", p.cur_profile_id == 1, NULL);

    /*
     * The 134-byte layout is the only one verified. Anything else is a
     * firmware this decoder has not been checked against, and is refused
     * rather than misparsed into confident nonsense.
     */
    check("a shorter payload is refused",
          sbgc_parse_params_3(b, sizeof(b) - 1, &p) == -1, NULL);
    check("a longer payload is refused",
          sbgc_parse_params_3(b, sizeof(b) + 1, &p) == -1, NULL);

    section("serial speed names");
    check("index 0 is 115200",
          strcmp(sbgc_serial_speed_name(0), "115200") == 0, NULL);
    check("index 1 is 57600",
          strcmp(sbgc_serial_speed_name(1), "57600") == 0, NULL);
    check("an out-of-range index is reported as unknown",
          strcmp(sbgc_serial_speed_name(99), "unknown") == 0, NULL);
}

/* ------------------------------------------------------------- httpd -- */

static void test_url_decode(void)
{
    section("percent decoding");

    char out[64];
    httpd_url_decode("plain", out, sizeof(out));
    check("plain text is unchanged", strcmp(out, "plain") == 0, out);

    httpd_url_decode("a%20b", out, sizeof(out));
    check("%20 becomes a space", strcmp(out, "a b") == 0, out);

    httpd_url_decode("a+b", out, sizeof(out));
    check("+ becomes a space", strcmp(out, "a b") == 0, out);

    httpd_url_decode("%2Fdev%2FttyUSB0", out, sizeof(out));
    check("a device path round-trips",
          strcmp(out, "/dev/ttyUSB0") == 0, out);

    httpd_url_decode("%2fdev%2fttyusb0", out, sizeof(out));
    check("lowercase hex digits work",
          strcmp(out, "/dev/ttyusb0") == 0, out);

    /* A trailing escape has nothing to consume. It must be passed through
     * rather than read past the end of the string. */
    httpd_url_decode("abc%", out, sizeof(out));
    check("a trailing bare % is left alone", strcmp(out, "abc%") == 0, out);
    httpd_url_decode("abc%4", out, sizeof(out));
    check("a truncated escape is left alone", strcmp(out, "abc%4") == 0, out);
    httpd_url_decode("a%zzb", out, sizeof(out));
    check("a non-hex escape is left alone", strcmp(out, "a%zzb") == 0, out);

    /* The output is bounded by the caller's buffer, always terminated. */
    char small[5];
    httpd_url_decode("abcdefghij", small, sizeof(small));
    check("output is truncated to the buffer",
          strlen(small) == 4 && strcmp(small, "abcd") == 0, small);
}

static void test_form_value(void)
{
    section("form and query decoding");

    char v[64];
    check("a single field is found",
          httpd_form_value("pan=1.0", "pan", v, sizeof(v)) && strcmp(v, "1.0") == 0,
          v);
    check("a middle field is found",
          httpd_form_value("a=1&pan=0.5&z=9", "pan", v, sizeof(v)) &&
          strcmp(v, "0.5") == 0, v);
    check("a trailing field is found",
          httpd_form_value("a=1&pan=-1", "pan", v, sizeof(v)) &&
          strcmp(v, "-1") == 0, v);
    check("an absent key is reported absent",
          httpd_form_value("a=1&b=2", "pan", v, sizeof(v)) == 0, NULL);

    /*
     * A valueless parameter used to hide every field after it: the search for
     * '=' ran straight past the '&' into the next field, so "flag&pan=1"
     * matched nothing and a rate command read as zero.
     */
    check("a valueless parameter does not hide later fields",
          httpd_form_value("flag&pan=1", "pan", v, sizeof(v)) &&
          strcmp(v, "1") == 0, v);
    check("two valueless parameters still do not",
          httpd_form_value("x&y&pan=2", "pan", v, sizeof(v)) &&
          strcmp(v, "2") == 0, v);

    /* A key that is a prefix of another must not match it. */
    check("a prefix key does not match a longer one",
          httpd_form_value("panic=1", "pan", v, sizeof(v)) == 0, NULL);
    check("a longer key does not match a shorter one",
          httpd_form_value("pan=1", "panic", v, sizeof(v)) == 0, NULL);

    check("an empty value is found and empty",
          httpd_form_value("pan=&tilt=1", "pan", v, sizeof(v)) &&
          v[0] == '\0', v);
    check("values are percent-decoded",
          httpd_form_value("path=%2Fdev%2FttyUSB0", "path", v, sizeof(v)) &&
          strcmp(v, "/dev/ttyUSB0") == 0, v);
    check("an empty body finds nothing",
          httpd_form_value("", "pan", v, sizeof(v)) == 0, NULL);
}

/* One request/response exchange against a real socket. */

struct hit {
    char method[8];
    char path[HTTPD_MAX_PATH];
    char body[HTTPD_MAX_BODY];
    int  calls;
};

static void record(const httpd_request_t *req, httpd_response_t *resp, void *u)
{
    struct hit *h = (struct hit *)u;
    h->calls++;
    snprintf(h->method, sizeof(h->method), "%s", req->method);
    snprintf(h->path, sizeof(h->path), "%s", req->path);
    snprintf(h->body, sizeof(h->body), "%s", req->body);

    static const char body[] = "pong";
    resp->status = 200;
    resp->content_type = "text/plain";
    resp->body = (char *)body;
    resp->body_len = strlen(body);
    resp->owned = 0;
}

/* Connect, send `req`, pump the server until it answers, return the reply. */
static int exchange(httpd_t *h, int port, const char *req, size_t req_len,
                    struct hit *hit, char *reply, size_t reply_cap)
{
    /* Cleared up front. Returning early while leaving the previous reply in
     * the buffer lets a later assertion pass on an older response — a 413 from
     * one case satisfying the next case's 413 check. */
    if (reply_cap) reply[0] = '\0';

    int c = socket(AF_INET, SOCK_STREAM, 0);
    if (c < 0) return -1;

    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons((uint16_t)port);
    inet_pton(AF_INET, "127.0.0.1", &sa.sin_addr);
    if (connect(c, (struct sockaddr *)&sa, sizeof(sa)) != 0) { close(c); return -1; }

    if (write(c, req, req_len) < 0) { close(c); return -1; }

    for (int i = 0; i < 200 && hit->calls == 0; i++) httpd_serve(h, 10, record, hit);

    /* Keep pumping and reading until the reply really stops arriving, not
     * until the first quiet 20 ms — under load a scheduling gap is not the
     * end of a response, and treating it as one makes the suite flaky. */
    size_t used = 0;
    int quiet = 0;
    for (int i = 0; i < 300 && used + 1 < reply_cap && quiet < 25; i++) {
        struct pollfd p = { c, POLLIN, 0 };
        int pr = poll(&p, 1, 20);
        if (pr == 0) { quiet++; httpd_serve(h, 5, record, hit); continue; }
        if (pr < 0) break;
        quiet = 0;
        ssize_t n = read(c, reply + used, reply_cap - used - 1);
        if (n <= 0) break;
        used += (size_t)n;
    }
    reply[used] = '\0';
    close(c);
    return 0;
}

static void test_http_requests(void)
{
    section("HTTP request handling");

    httpd_t h;
    int port = 0, opened = 0;
    for (port = 18400; port < 18500; port++)
        if (httpd_open(&h, "127.0.0.1", port) == 0) { opened = 1; break; }
    if (!opened) { check("could bind a test port", 0, "no free port"); return; }

    char reply[1024];
    struct hit hit;

    memset(&hit, 0, sizeof(hit));
    const char *get = "GET /api/live HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n";
    exchange(&h, port, get, strlen(get), &hit, reply, sizeof(reply));
    check("a GET reaches the handler", hit.calls == 1, NULL);
    check("the method is parsed", strcmp(hit.method, "GET") == 0, hit.method);
    check("the path is parsed", strcmp(hit.path, "/api/live") == 0, hit.path);
    check("the response comes back",
          strstr(reply, "200 OK") && strstr(reply, "pong"), reply);

    /* A POST body must arrive whole, which means honouring Content-Length. */
    memset(&hit, 0, sizeof(hit));
    const char *post =
        "POST /api/control/rate HTTP/1.1\r\nHost: 127.0.0.1\r\n"
        "Content-Type: application/x-www-form-urlencoded\r\n"
        "Content-Length: 17\r\n\r\npan=1.0&tilt=-0.5";
    exchange(&h, port, post, strlen(post), &hit, reply, sizeof(reply));
    check("a POST reaches the handler", hit.calls == 1, NULL);
    check("the body arrives intact",
          strcmp(hit.body, "pan=1.0&tilt=-0.5") == 0, hit.body);

    /* Headers the handler needs for its origin check. */
    memset(&hit, 0, sizeof(hit));
    const char *origin =
        "GET / HTTP/1.1\r\nHost: 127.0.0.1:8080\r\n"
        "Origin: http://127.0.0.1:8080\r\n\r\n";
    exchange(&h, port, origin, strlen(origin), &hit, reply, sizeof(reply));
    check("a request carrying Origin is handled", hit.calls == 1, NULL);

    /* A request line with no version is malformed and must be refused
     * without ever reaching the handler. */
    memset(&hit, 0, sizeof(hit));
    const char *bad = "GARBAGE\r\n\r\n";
    exchange(&h, port, bad, strlen(bad), &hit, reply, sizeof(reply));
    check("a malformed request line is refused as 400",
          strstr(reply, "400") != NULL, reply);

    /*
     * Oversized requests are refused rather than truncated into something
     * that parses as a different request.
     */
    memset(&hit, 0, sizeof(hit));
    size_t big_len = HTTPD_MAX_REQUEST + 2048;
    char *big = malloc(big_len + 1);
    if (big) {
        int n = snprintf(big, big_len, "GET /");
        memset(big + n, 'a', big_len - (size_t)n);
        big[big_len] = '\0';
        exchange(&h, port, big, big_len, &hit, reply, sizeof(reply));
        check("an oversized request is refused as 413",
              strstr(reply, "413") != NULL, reply);
        free(big);
    }

    /*
     * A declared body larger than the handler's buffer must be refused, not
     * accepted and silently truncated — the handler would otherwise act on a
     * different request from the one that was sent.
     */
    memset(&hit, 0, sizeof(hit));
    {
        char over[512];
        int n = snprintf(over, sizeof(over),
            "POST /x HTTP/1.1\r\nHost: h\r\nContent-Length: %d\r\n\r\n",
            HTTPD_MAX_BODY + 16);
        exchange(&h, port, over, (size_t)n, &hit, reply, sizeof(reply));
        check("a body larger than the handler buffer is refused",
              strstr(reply, "413") != NULL && hit.calls == 0, reply);
    }

    memset(&hit, 0, sizeof(hit));
    {
        const char *huge =
            "POST /x HTTP/1.1\r\nHost: h\r\n"
            "Content-Length: 99999999999999999999\r\n\r\n";
        exchange(&h, port, huge, strlen(huge), &hit, reply, sizeof(reply));
        check("an out-of-range Content-Length is refused",
              strstr(reply, "413") != NULL && hit.calls == 0, reply);
    }

    memset(&hit, 0, sizeof(hit));
    {
        const char *nan_len =
            "POST /x HTTP/1.1\r\nHost: h\r\nContent-Length: abc\r\n\r\n";
        exchange(&h, port, nan_len, strlen(nan_len), &hit, reply, sizeof(reply));
        check("a non-numeric Content-Length is refused",
              strstr(reply, "413") != NULL && hit.calls == 0, reply);
    }

    /* Exactly the declared length reaches the handler, no more. */
    memset(&hit, 0, sizeof(hit));
    {
        const char *exact =
            "POST /x HTTP/1.1\r\nHost: h\r\nContent-Length: 5\r\n\r\npan=1&extra=ignored";
        exchange(&h, port, exact, strlen(exact), &hit, reply, sizeof(reply));
        check("only the declared body length is handed over",
              hit.calls == 1 && strcmp(hit.body, "pan=1") == 0, hit.body);
    }

    /* Digits must be the whole value: "5junk" used to parse as 5. */
    memset(&hit, 0, sizeof(hit));
    {
        const char *suffix =
            "POST /x HTTP/1.1\r\nHost: h\r\nContent-Length: 5junk\r\n\r\npan=1";
        exchange(&h, port, suffix, strlen(suffix), &hit, reply, sizeof(reply));
        check("a trailing-garbage Content-Length is refused",
              strstr(reply, "413") != NULL && hit.calls == 0, reply);
    }

    /* Two headers disagreeing about the length is never legitimate. */
    memset(&hit, 0, sizeof(hit));
    {
        const char *dup =
            "POST /x HTTP/1.1\r\nHost: h\r\nContent-Length: 5\r\n"
            "Content-Length: 9\r\n\r\npan=1";
        exchange(&h, port, dup, strlen(dup), &hit, reply, sizeof(reply));
        check("a duplicated Content-Length is refused",
              strstr(reply, "413") != NULL && hit.calls == 0, reply);
    }

    /*
     * Connections that never finish must not stop later ones being served.
     * This is the starvation defect: the server used to read each connection
     * to completion in turn, so half-open sockets were additive.
     */
    int stalled[HTTPD_MAX_CLIENTS + 8];
    int n_stalled = 0;
    for (int i = 0; i < (int)(sizeof(stalled) / sizeof(stalled[0])); i++) {
        int c = socket(AF_INET, SOCK_STREAM, 0);
        if (c < 0) break;
        struct sockaddr_in sa2;
        memset(&sa2, 0, sizeof(sa2));
        sa2.sin_family = AF_INET;
        sa2.sin_port = htons((uint16_t)port);
        inet_pton(AF_INET, "127.0.0.1", &sa2.sin_addr);
        /* Non-blocking connect: the listen backlog is finite and the server
         * only drains it when it is pumped, so a blocking connect past the
         * backlog would wait for a serve call that this loop has not made
         * yet. */
        fcntl(c, F_SETFL, fcntl(c, F_GETFL, 0) | O_NONBLOCK);
        int rc = connect(c, (struct sockaddr *)&sa2, sizeof(sa2));
        if (rc != 0 && errno != EINPROGRESS) { close(c); break; }
        httpd_serve(&h, 5, record, &hit);
        ssize_t w = write(c, "GET /x HTTP/1.1\r\n", 17);   /* never completed */
        (void)w;
        httpd_serve(&h, 5, record, &hit);
        stalled[n_stalled++] = c;
    }
    check("more stalled connections than slots were opened",
          n_stalled > HTTPD_MAX_CLIENTS, NULL);

    memset(&hit, 0, sizeof(hit));
    exchange(&h, port, get, strlen(get), &hit, reply, sizeof(reply));
    check("a real request is still served while they are held open",
          hit.calls == 1 && strstr(reply, "pong") != NULL, reply);

    for (int i = 0; i < n_stalled; i++) close(stalled[i]);
    httpd_close(&h);
}

/* -------------------------------------------------- gui config files -- */

static int write_file(const char *path, const char *text)
{
    FILE *f = fopen(path, "w");
    if (!f) return 0;
    fputs(text, f);
    fclose(f);
    return 1;
}

static void test_gui_config(void)
{
    section("SimpleBGC GUI settings discovery");

    char root[] = "/tmp/sbgc_cfg_XXXXXX";
    if (!mkdtemp(root)) { check("could create a fixture directory", 0, NULL); return; }

    char conf[512], profiles[512], path[600];
    snprintf(conf, sizeof(conf), "%s/conf", root);
    snprintf(profiles, sizeof(profiles), "%s/profiles", root);
    mkdir(conf, 0777);
    mkdir(profiles, 0777);

    snprintf(path, sizeof(path), "%s/bgc.properties", conf);
    write_file(path,
        "# a comment that must be ignored\n"
        "last.used.port = /dev/ttyUSB3\n"
        "latest.serial.baud=1\n"
        "unrelated.key = value\n");

    /*
     * The value lists are deliberately SHORT and followed by another element
     * holding more <int>s. Without a closing bound the reader runs past the
     * end of its own element and pads the list from whatever comes next, so a
     * yaw limit gets read out of a neighbouring field.
     */
    snprintf(path, sizeof(path), "%s/all.profile", profiles);
    write_file(path,
        "<all-profiles>\n"
        "  <profile>\n"
        "    <rcMinAngle>\n"
        "      <int>-30</int><int>-45</int><int>-170</int>\n"
        "    </rcMinAngle>\n"
        "    <rcMaxAngle>\n"
        "      <int>30</int><int>45</int><int>170</int>\n"
        "    </rcMaxAngle>\n"
        "    <somethingElse>\n"
        "      <int>999</int><int>888</int><int>777</int>\n"
        "    </somethingElse>\n"
        "  </profile>\n"
        "</all-profiles>\n");

    /*
     * Stage a COMPETING install where the automatic scan looks, and point HOME
     * at it, before asking for the fixture explicitly. Without this the
     * precedence check proves nothing: on a machine with no SimpleBGC GUI
     * installed — a container, or CI — the scan finds nothing to override with
     * and the test passes even with the bug present.
     */
    char fake_home[] = "/tmp/sbgc_home_XXXXXX";
    const char *real_home = getenv("HOME");
    char saved_home[512] = "";
    if (real_home) snprintf(saved_home, sizeof(saved_home), "%s", real_home);

    int staged = 0;
    if (mkdtemp(fake_home)) {
        setenv("HOME", fake_home, 1);
        char rival[512], rconf[600], rpath[800];
        snprintf(rival, sizeof(rival), "%s/SimpleBGC_GUI_rival", fake_home);
        snprintf(rconf, sizeof(rconf), "%s/conf", rival);
        if (mkdir(rival, 0777) == 0 && mkdir(rconf, 0777) == 0) {
            snprintf(rpath, sizeof(rpath), "%s/bgc.properties", rconf);
            staged = write_file(rpath,
                "last.used.port=/dev/ttyRIVAL\n"
                "latest.serial.baud=4\n");
        }
    }

    sbgc_gui_config_t gc;
    sbgc_gui_config_discover(&gc, root);

    check("a competing install was staged where the scan looks", staged != 0,
          fake_home);
    check("an explicitly named directory wins over the automatic scan",
          strcmp(gc.port, "/dev/ttyUSB3") == 0 && gc.baud == 57600,
          gc.summary);

    check("the install directory is recorded",
          strcmp(gc.install_dir, root) == 0, gc.install_dir);
    check("the last used port is recovered",
          gc.have_port && strcmp(gc.port, "/dev/ttyUSB3") == 0, gc.port);

    /* The property is a dropdown index, not a rate: 1 means 57600. */
    {
        char extra[64];
        snprintf(extra, sizeof(extra), "baud=%d", gc.baud);
        check("the baud index maps to a real rate",
              gc.have_baud && gc.baud == 57600, extra);
    }

    check("profile limits are recovered", gc.have_limits, NULL);
    {
        char extra[128];
        snprintf(extra, sizeof(extra),
                 "roll [%.0f,%.0f] pitch [%.0f,%.0f] yaw [%.0f,%.0f]",
                 gc.roll_min, gc.roll_max, gc.pitch_min, gc.pitch_max,
                 gc.yaw_min, gc.yaw_max);
        check("each axis takes its own pair, in order",
              gc.roll_min == -30 && gc.roll_max == 30 &&
              gc.pitch_min == -45 && gc.pitch_max == 45 &&
              gc.yaw_min == -170 && gc.yaw_max == 170, extra);
        check("no value is taken from a neighbouring element",
              gc.yaw_min != 999 && gc.yaw_max != 999, extra);
    }
    check("the summary names what was found",
          strstr(gc.summary, root) != NULL, gc.summary);

    /*
     * The remaining cases assert that nothing was found. HOME already points
     * at the staging directory above, so remove the rival install from it
     * first — otherwise the scan would legitimately find that one.
     */
    {
        char rpath[800];
        snprintf(rpath, sizeof(rpath),
                 "%s/SimpleBGC_GUI_rival/conf/bgc.properties", fake_home);
        unlink(rpath);
        snprintf(rpath, sizeof(rpath), "%s/SimpleBGC_GUI_rival/conf", fake_home);
        rmdir(rpath);
        snprintf(rpath, sizeof(rpath), "%s/SimpleBGC_GUI_rival", fake_home);
        rmdir(rpath);
    }

    /* A directory with nothing in it is a normal outcome, not an error. */
    char empty[] = "/tmp/sbgc_empty_XXXXXX";
    if (mkdtemp(empty)) {
        sbgc_gui_config_t none;
        sbgc_gui_config_discover(&none, empty);
        check("an empty directory yields no port or limits",
              !none.have_port && !none.have_limits, none.summary);
        check("and says so in the summary",
              strstr(none.summary, "built-in") != NULL, none.summary);
        rmdir(empty);
    }

    /* A path that does not exist must not crash or invent anything. */
    {
        sbgc_gui_config_t missing;
        sbgc_gui_config_discover(&missing, "/tmp/sbgc-does-not-exist-at-all");
        check("a missing directory is handled",
              !missing.have_port && missing.install_dir[0] == '\0',
              missing.summary);
    }

    if (saved_home[0]) setenv("HOME", saved_home, 1);
    rmdir(fake_home);

    section("serial port resolution");
    {
        char out[256];
        /* A port that exists resolves to itself, unchanged. */
        int r = sbgc_gui_config_resolve_port("/dev/null", out, sizeof(out));
        check("an existing device resolves to itself",
              r == 1 && strcmp(out, "/dev/null") == 0, out);

        /*
         * A device that is gone leaves the requested path in `out` so the
         * caller can still report a sensible error, whether or not a
         * substitute was offered.
         */
        r = sbgc_gui_config_resolve_port("/dev/tty-nope-not-here", out,
                                         sizeof(out));
        check("a missing device never reports itself as present", r != 1, out);
        check("out is always populated", out[0] != '\0', out);
    }

    snprintf(path, sizeof(path), "%s/all.profile", profiles);
    unlink(path);
    snprintf(path, sizeof(path), "%s/bgc.properties", conf);
    unlink(path);
    rmdir(profiles);
    rmdir(conf);
    rmdir(root);
}

/* -------------------------------------------------------------- main -- */

int main(void)
{
    printf("test_modules — httpd, sbgc_params and sbgc_gui_config\n");

    test_board_info();
    test_realtime();
    test_params();
    test_url_decode();
    test_form_value();
    test_http_requests();
    test_gui_config();

    printf("\n----------------------------------------\n");
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
