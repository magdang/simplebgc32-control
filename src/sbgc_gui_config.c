/*
 * sbgc_gui_config.c — read-only recovery of defaults from a SimpleBGC GUI
 * installation. See include/sbgc_gui_config.h.
 */

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif

#include "sbgc_gui_config.h"

#include <dirent.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

/* ----------------------------------------------------------------- util -- */

static int file_exists(const char *path)
{
    struct stat st;
    return path && path[0] && stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

static int dir_exists(const char *path)
{
    struct stat st;
    return path && path[0] && stat(path, &st) == 0 && S_ISDIR(st.st_mode);
}

/*
 * Join "dir/leaf" into a fixed buffer. Returns 0 if the result would be
 * truncated, so a too-long path is skipped rather than silently turned into a
 * different, shorter path that might exist.
 */
static int join_path(char *out, size_t cap, const char *dir, const char *leaf)
{
    int n = snprintf(out, cap, "%s/%s", dir, leaf);
    return (n > 0 && (size_t)n < cap);
}

static void append_summary(sbgc_gui_config_t *c, const char *fmt, ...)
{
    va_list ap;
    size_t used = strlen(c->summary);
    if (used + 2 >= sizeof(c->summary)) return;
    if (used) {
        c->summary[used++] = ';';
        c->summary[used++] = ' ';
        c->summary[used]   = '\0';
    }
    va_start(ap, fmt);
    vsnprintf(c->summary + used, sizeof(c->summary) - used, fmt, ap);
    va_end(ap);
}

/* Trim ASCII whitespace in place. */
static char *trim(char *s)
{
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
    char *e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' ||
                     e[-1] == '\r' || e[-1] == '\n')) *--e = '\0';
    return s;
}

/* ------------------------------------------------------ bgc.properties -- */

/*
 * latest.serial.baud is a dropdown index, not a rate. Index 0 = 115200 was
 * confirmed on this board: its profile reports SERIAL_SPEED 0 and the link
 * only answers at 115200.
 */
static int baud_index_to_rate(int index)
{
    switch (index) {
        case 0: return 115200;
        case 1: return 57600;
        case 2: return 38400;
        case 3: return 19200;
        case 4: return 9600;
        default: return 115200;
    }
}

static void read_properties(sbgc_gui_config_t *c, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return;

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        char *s = trim(line);
        if (*s == '#' || *s == '\0') continue;

        char *eq = strchr(s, '=');
        if (!eq) continue;
        *eq = '\0';
        char *key = trim(s);
        char *val = trim(eq + 1);

        if (strcmp(key, "last.used.port") == 0 && *val) {
            snprintf(c->port, sizeof(c->port), "%s", val);
            c->have_port = 1;
        } else if (strcmp(key, "latest.serial.baud") == 0 && *val) {
            c->baud = baud_index_to_rate(atoi(val));
            c->have_baud = 1;
        }
    }
    fclose(f);
    snprintf(c->properties_path, sizeof(c->properties_path), "%s", path);
}

/* ----------------------------------------------------- .profile parsing -- */

/*
 * The exported profile is XStream XML. Rather than pull in a parser for a
 * handful of integers, scrape the specific tags needed. The format is machine
 * generated and stable, and a miss simply leaves the default in place.
 */

/*
 * Find <tag> ... </tag>. Returns a pointer just past the opening tag and, in
 * `limit`, the start of the matching closing tag.
 *
 * The closing bound matters: without it a value list runs off the end of its
 * own element and keeps consuming <int>s from whatever element follows, so a
 * short list silently gets padded with unrelated numbers. Reading a yaw limit
 * out of a neighbouring element is exactly the kind of confidently-wrong
 * value this file must not produce.
 */
static const char *find_tag(const char *hay, const char *tag,
                            const char **limit)
{
    char open[64], close[64];
    snprintf(open,  sizeof(open),  "<%s>",  tag);
    snprintf(close, sizeof(close), "</%s>", tag);

    const char *p = strstr(hay, open);
    if (!p) return NULL;
    p += strlen(open);

    const char *e = strstr(p, close);
    if (limit) *limit = e;          /* NULL means "unterminated"; treated as EOF */
    return p;
}

/* Read the first `n` <int> values, never looking past `limit`. */
static int read_int_list(const char *p, const char *limit, int *out, int n)
{
    int got = 0;
    while (p && got < n) {
        const char *v = strstr(p, "<int>");
        if (!v || (limit && v >= limit)) break;
        v += 5;
        out[got++] = atoi(v);
        p = v;
    }
    return got;
}

static void read_profile(sbgc_gui_config_t *c, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return;

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 4 * 1024 * 1024) { fclose(f); return; }

    char *buf = (char *)malloc((size_t)sz + 1);
    if (!buf) { fclose(f); return; }
    size_t rd = fread(buf, 1, (size_t)sz, f);
    buf[rd] = '\0';
    fclose(f);

    /* The first <profile> element is profile 1. */
    const char *prof = strstr(buf, "<all-profiles>");
    prof = prof ? strstr(prof, "<profile>") : NULL;

    if (prof) {
        int mn[3], mx[3];
        const char *lmin = NULL, *lmax = NULL;
        const char *pmin = find_tag(prof, "rcMinAngle", &lmin);
        const char *pmax = find_tag(prof, "rcMaxAngle", &lmax);
        if (pmin && pmax &&
            read_int_list(pmin, lmin, mn, 3) == 3 &&
            read_int_list(pmax, lmax, mx, 3) == 3) {
            c->roll_min  = mn[0]; c->roll_max  = mx[0];
            c->pitch_min = mn[1]; c->pitch_max = mx[1];
            c->yaw_min   = mn[2]; c->yaw_max   = mx[2];
            c->have_limits = 1;
        }
    }

    free(buf);
    snprintf(c->profile_path, sizeof(c->profile_path), "%s", path);
}

/* Pick any *.profile in `dir`, preferring one that mentions all profiles. */
static int find_profile_file(const char *dir, char *out, size_t cap)
{
    DIR *d = opendir(dir);
    if (!d) return 0;

    char best[SBGC_GUI_PATH_MAX] = "";
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        const char *dot = strrchr(e->d_name, '.');
        if (!dot || strcmp(dot, ".profile") != 0) continue;
        char full[SBGC_GUI_PATH_MAX];
        if (!join_path(full, sizeof(full), dir, e->d_name)) continue;
        if (!file_exists(full)) continue;
        if (!best[0] || strstr(e->d_name, "all") != NULL)
            snprintf(best, sizeof(best), "%s", full);
    }
    closedir(d);

    if (!best[0]) return 0;
    snprintf(out, cap, "%s", best);
    return 1;
}

/* ------------------------------------------------------------ discovery -- */

/* Consider one candidate install directory; returns 1 if it looked real. */
static int try_install_dir(sbgc_gui_config_t *c, const char *dir)
{
    if (!dir_exists(dir)) return 0;

    char props[SBGC_GUI_PATH_MAX];
    char profdir[SBGC_GUI_PATH_MAX];
    snprintf(props,   sizeof(props),   "%s/conf/bgc.properties", dir);
    snprintf(profdir, sizeof(profdir), "%s/profiles", dir);

    int any = 0;

    if (file_exists(props)) {
        read_properties(c, props);
        any = 1;
    }

    char proffile[SBGC_GUI_PATH_MAX];
    if (dir_exists(profdir) && find_profile_file(profdir, proffile, sizeof(proffile))) {
        read_profile(c, proffile);
        any = 1;
    }

    /*
     * Record the directory the settings actually came from. Keeping the first
     * hit while later ones overwrite the port, baud and limits would name one
     * install in the summary while reporting another one's values, and the
     * operator would have no way to tell.
     */
    if (any) snprintf(c->install_dir, sizeof(c->install_dir), "%s", dir);

    return any;
}

/* ---------------------------------------------------------- port listing -- */

#define BY_ID_DIR "/dev/serial/by-id"

/* Does this look like a serial device node we could talk to a gimbal on? */
static int is_serial_name(const char *name)
{
    return strncmp(name, "ttyUSB", 6) == 0 || strncmp(name, "ttyACM", 6) == 0;
}

int sbgc_gui_config_list_ports(sbgc_port_t *out, int max)
{
    if (!out || max <= 0) return 0;
    int n = 0;

    /*
     * by-id first: those names come from the adapter itself, so they survive
     * a re-plug that would otherwise move ttyUSB0 to ttyUSB1. They are the
     * right thing to pick, so they are offered first.
     */
    DIR *d = opendir(BY_ID_DIR);
    if (d) {
        struct dirent *e;
        while ((e = readdir(d)) != NULL && n < max) {
            if (e->d_name[0] == '.') continue;
            char full[SBGC_PORT_NAME_MAX];
            if (!join_path(full, sizeof(full), BY_ID_DIR, e->d_name)) continue;

            /* Resolve so the label can show which ttyUSB* it currently is. */
            char real[SBGC_PORT_NAME_MAX];
            ssize_t rl = readlink(full, real, sizeof(real) - 1);
            if (rl > 0) {
                real[rl] = '\0';
                const char *base = strrchr(real, '/');
                base = base ? base + 1 : real;
                /*
                 * Build in two steps. Composing this in one snprintf makes
                 * the compiler assume both fields can be maximal, which it
                 * reports as a possible truncation; copying the name first
                 * and appending only if the suffix fits is both clearer and
                 * provably in bounds.
                 */
                snprintf(out[n].label, sizeof(out[n].label), "%s", e->d_name);
                size_t used = strlen(out[n].label);
                size_t need = strlen(base) + 8;   /* "  (now " + name + ")" */
                if (used + need < sizeof(out[n].label))
                    snprintf(out[n].label + used, sizeof(out[n].label) - used,
                             "  (now %s)", base);
            } else {
                snprintf(out[n].label, sizeof(out[n].label), "%s", e->d_name);
            }
            snprintf(out[n].path, sizeof(out[n].path), "%s", full);
            out[n].stable = 1;
            n++;
        }
        closedir(d);
    }

    /* Then the raw nodes, so a device with no by-id entry is still selectable. */
    d = opendir("/dev");
    if (d) {
        struct dirent *e;
        while ((e = readdir(d)) != NULL && n < max) {
            if (!is_serial_name(e->d_name)) continue;
            char full[SBGC_PORT_NAME_MAX];
            if (!join_path(full, sizeof(full), "/dev", e->d_name)) continue;
            snprintf(out[n].path,  sizeof(out[n].path),  "%s", full);
            snprintf(out[n].label, sizeof(out[n].label), "%s", full);
            out[n].stable = 0;
            n++;
        }
        closedir(d);
    }

    return n;
}

int sbgc_gui_config_resolve_port(const char *want, char *out, size_t out_cap)
{
    if (!out || out_cap == 0) return 0;
    if (want && want[0]) snprintf(out, out_cap, "%s", want);
    else out[0] = '\0';

    struct stat st;
    if (want && want[0] && stat(want, &st) == 0) return 1;   /* still there */

    /* Gone. Prefer a stable by-id entry, then any raw node. */
    sbgc_port_t ports[SBGC_PORT_LIST_MAX];
    int n = sbgc_gui_config_list_ports(ports, SBGC_PORT_LIST_MAX);
    for (int pass = 1; pass >= 0; pass--) {
        for (int i = 0; i < n; i++) {
            if (ports[i].stable != pass) continue;
            snprintf(out, out_cap, "%s", ports[i].path);
            return 2;
        }
    }
    return 0;
}

int sbgc_gui_config_discover(sbgc_gui_config_t *out, const char *extra_dir)
{
    if (!out) return 0;
    memset(out, 0, sizeof(*out));

    if (extra_dir && extra_dir[0]) try_install_dir(out, extra_dir);

    const char *home = getenv("HOME");
    if (home) {
        /* Scan the usual download/install parents for SimpleBGC_GUI_* dirs.
         * readdir order is arbitrary, so "last wins" is whichever the
         * filesystem happened to return last — install_dir is set alongside
         * the values it supplied, so the summary always names the install the
         * settings actually came from. */
        const char *parents[] = { "Downloads", ".", "Applications", "opt" };
        for (size_t i = 0; i < sizeof(parents) / sizeof(parents[0]); i++) {
            char parent[SBGC_GUI_PATH_MAX];
            snprintf(parent, sizeof(parent), "%s/%s", home, parents[i]);

            DIR *d = opendir(parent);
            if (!d) continue;
            struct dirent *e;
            while ((e = readdir(d)) != NULL) {
                if (strncmp(e->d_name, "SimpleBGC", 9) != 0) continue;
                char cand[SBGC_GUI_PATH_MAX];
                if (!join_path(cand, sizeof(cand), parent, e->d_name)) continue;
                try_install_dir(out, cand);
            }
            closedir(d);
        }
    }

    if (!out->install_dir[0]) {
        append_summary(out, "no SimpleBGC GUI install found; using built-in defaults");
        return 0;
    }

    append_summary(out, "found %s", out->install_dir);
    if (out->have_port)   append_summary(out, "port %s", out->port);
    if (out->have_baud)   append_summary(out, "baud %d", out->baud);
    if (out->have_limits) append_summary(out, "limits from profile 1");
    else                  append_summary(out, "no saved profile; limits are built-in defaults");

    return 0;
}
