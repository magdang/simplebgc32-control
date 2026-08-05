/*
 * sbgc_probe.c — STRICTLY READ-ONLY SimpleBGC board prober.
 *
 * Sends only query commands. It never sends CMD_CONTROL, CMD_MOTORS_ON/OFF,
 * CMD_EXECUTE_MENU, CMD_WRITE_PARAMS* or anything else that moves the gimbal
 * or mutates board state. The command whitelist below is the enforcement
 * point: sbgc_send() is not called anywhere except through probe_send(),
 * which refuses any command not on the list.
 *
 * Dumps every response as raw hex so the payloads can be parsed offline
 * against the vendor spec, rather than parsed live against a guess.
 */

#define _DEFAULT_SOURCE 1

#include "sbgc_api.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

/* ------------------------------------------------- read-only command IDs -- */

#define CMD_BOARD_INFO        86
#define CMD_BOARD_INFO_3      20
#define CMD_READ_PARAMS_3     21
#define CMD_READ_PARAMS_EXT   82
#define CMD_READ_PARAMS_EXT2 102
#define CMD_READ_PARAMS_EXT3 104
#define CMD_REALTIME_DATA_3   23
#define CMD_REALTIME_DATA_4   25
#define CMD_GET_ANGLES        73

/* Every command this tool is permitted to transmit. Anything else aborts. */
static const uint8_t READ_ONLY_WHITELIST[] = {
    CMD_BOARD_INFO, CMD_BOARD_INFO_3,
    CMD_READ_PARAMS_3, CMD_READ_PARAMS_EXT,
    CMD_READ_PARAMS_EXT2, CMD_READ_PARAMS_EXT3,
    CMD_REALTIME_DATA_3, CMD_REALTIME_DATA_4,
    CMD_GET_ANGLES
};

static int is_read_only(uint8_t cmd)
{
    for (size_t i = 0; i < sizeof(READ_ONLY_WHITELIST); i++)
        if (READ_ONLY_WHITELIST[i] == cmd) return 1;
    return 0;
}

/* ------------------------------------------------------------- reporting -- */

static FILE *g_out;

static void dump(const char *tag, const uint8_t *p, size_t n)
{
    fprintf(g_out, "%s len=%zu\n", tag, n);
    for (size_t i = 0; i < n; i += 16) {
        fprintf(g_out, "  %04zu:", i);
        for (size_t j = 0; j < 16 && i + j < n; j++)
            fprintf(g_out, " %02X", p[i + j]);
        fprintf(g_out, "\n");
    }
    fflush(g_out);
}

struct rxctx {
    int      got;
    uint8_t  cmd;
    uint8_t  payload[SBGC_MAX_PAYLOAD];
    size_t   len;
};

static void on_frame(uint8_t cmd_id, const uint8_t *payload,
                     size_t payload_len, void *user)
{
    struct rxctx *c = (struct rxctx *)user;
    char tag[64];

    snprintf(tag, sizeof(tag), "RX cmd=%u (0x%02X)", cmd_id, cmd_id);
    dump(tag, payload, payload_len);

    /* Keep the last non-error frame for the caller to inspect. */
    if (cmd_id != SBGC_CMD_ERROR) {
        c->got = 1;
        c->cmd = cmd_id;
        c->len = payload_len > sizeof(c->payload) ? sizeof(c->payload)
                                                  : payload_len;
        memcpy(c->payload, payload, c->len);
    } else {
        fprintf(g_out, "  ** board returned CMD_ERROR **\n");
    }
}

/* Send one whitelisted query and collect the reply. */
static int probe_send(sbgc_t *sb, uint8_t cmd,
                      const uint8_t *payload, size_t len,
                      const char *what, struct rxctx *rx)
{
    if (!is_read_only(cmd)) {
        fprintf(stderr, "REFUSED: command %u is not read-only\n", cmd);
        abort();                      /* a bug here could move hardware */
    }

    memset(rx, 0, sizeof(*rx));
    fprintf(g_out, "\n--- %s (cmd %u) ---\n", what, cmd);

    /*
     * Retry rather than send once. Opening the port toggles DTR/RTS on a
     * CH340, which resets the board; a single query fired into that window is
     * simply lost and looks identical to "no board attached". Re-asking is
     * also what the GUI's polling loop does naturally.
     */
    int frames = 0;
    for (int attempt = 0; attempt < 6 && !rx->got; attempt++) {
        if (sbgc_send(sb, cmd, payload, len) != 0) {
            fprintf(g_out, "  TX FAILED: %s\n", sbgc_last_error(sb));
            return -1;
        }
        if (attempt == 0) {
            char hex[3 * SBGC_MAX_FRAME + 1];
            sbgc_format_last_tx(sb, hex, sizeof(hex));
            fprintf(g_out, "TX %s\n", hex);
        }
        for (int i = 0; i < 5 && !rx->got; i++)
            frames += sbgc_poll(sb, 100, on_frame, rx);
    }

    if (!rx->got) fprintf(g_out, "  (no response after 6 attempts)\n");
    return frames;
}

int main(int argc, char **argv)
{
    const char *port = "/dev/ttyUSB0";
    int baud = 115200;
    const char *outpath = NULL;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = argv[++i];
        else if (!strcmp(argv[i], "--baud") && i + 1 < argc) baud = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--out")  && i + 1 < argc) outpath = argv[++i];
    }

    g_out = outpath ? fopen(outpath, "w") : stdout;
    if (!g_out) { perror("fopen"); return 1; }

    sbgc_t sb;
    if (sbgc_open(&sb, port, baud) != 0) {
        fprintf(stderr, "open failed: %s\n", sbgc_last_error(&sb));
        return 1;
    }
    fprintf(g_out, "# READ-ONLY probe of %s @ %d\n", port, baud);

    /* Discard any boot chatter already sitting in the buffer. */
    sbgc_poll(&sb, 200, NULL, NULL);

    struct rxctx rx;

    probe_send(&sb, CMD_BOARD_INFO,   (const uint8_t[]){0}, 1, "BOARD_INFO", &rx);
    probe_send(&sb, CMD_BOARD_INFO_3, NULL, 0, "BOARD_INFO_3", &rx);

    /* Profile 1 is profile_id 0. The spec's own example, read Profile2,
     * uses payload 0x01 — so profiles are zero-indexed on the wire. */
    for (uint8_t prof = 0; prof < 5; prof++) {
        char what[64];
        snprintf(what, sizeof(what), "READ_PARAMS_3 profile %u", prof + 1u);
        probe_send(&sb, CMD_READ_PARAMS_3, &prof, 1, what, &rx);
        if (prof == 0) {
            snprintf(what, sizeof(what), "READ_PARAMS_EXT profile %u", prof + 1u);
            probe_send(&sb, CMD_READ_PARAMS_EXT, &prof, 1, what, &rx);
            snprintf(what, sizeof(what), "READ_PARAMS_EXT2 profile %u", prof + 1u);
            probe_send(&sb, CMD_READ_PARAMS_EXT2, &prof, 1, what, &rx);
            snprintf(what, sizeof(what), "READ_PARAMS_EXT3 profile %u", prof + 1u);
            probe_send(&sb, CMD_READ_PARAMS_EXT3, &prof, 1, what, &rx);
        }
    }

    probe_send(&sb, CMD_REALTIME_DATA_3, NULL, 0, "REALTIME_DATA_3", &rx);
    probe_send(&sb, CMD_REALTIME_DATA_4, NULL, 0, "REALTIME_DATA_4", &rx);
    probe_send(&sb, CMD_GET_ANGLES,      NULL, 0, "GET_ANGLES", &rx);

    sbgc_close(&sb);
    fprintf(g_out, "\n# done\n");
    if (outpath) fclose(g_out);
    return 0;
}
