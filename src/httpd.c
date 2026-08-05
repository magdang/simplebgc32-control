/*
 * httpd.c — minimal HTTP/1.1 server. See include/httpd.h for the caveats.
 */

/* strcasestr is a GNU extension; the rest is plain POSIX. */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include "httpd.h"

#include <arpa/inet.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static void set_err(httpd_t *h, const char *fmt, ...)
{
    if (!h) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(h->last_error, sizeof(h->last_error), fmt, ap);
    va_end(ap);
}

const char *httpd_last_error(const httpd_t *h)
{
    return (h && h->last_error[0]) ? h->last_error : "no error";
}

int httpd_open(httpd_t *h, const char *bind_addr, int port)
{
    if (!h) return -1;
    h->fd = -1;
    h->last_error[0] = '\0';

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) { set_err(h, "socket: %s", strerror(errno)); return -1; }

    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port   = htons((uint16_t)port);
    if (inet_pton(AF_INET, bind_addr ? bind_addr : "127.0.0.1", &sa.sin_addr) != 1) {
        set_err(h, "bad bind address '%s'", bind_addr ? bind_addr : "");
        close(fd);
        return -1;
    }

    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
        set_err(h, "bind %s:%d: %s", bind_addr, port, strerror(errno));
        close(fd);
        return -1;
    }
    if (listen(fd, 16) != 0) {
        set_err(h, "listen: %s", strerror(errno));
        close(fd);
        return -1;
    }

    /* Non-blocking accept so httpd_serve() can share a timeout with the
     * caller's own work loop. */
    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);

    h->fd = fd;
    return 0;
}

void httpd_close(httpd_t *h)
{
    if (!h) return;
    if (h->fd >= 0) close(h->fd);
    h->fd = -1;
}

static const char *status_text(int status)
{
    switch (status) {
        case 200: return "OK";
        case 204: return "No Content";
        case 400: return "Bad Request";
        case 403: return "Forbidden";
        case 404: return "Not Found";
        case 405: return "Method Not Allowed";
        case 413: return "Payload Too Large";
        case 500: return "Internal Server Error";
        default:  return "OK";
    }
}

static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

/* Read a whole request with a deadline, so a stalled peer cannot wedge us. */
static int read_request(int cfd, char *buf, size_t cap, size_t *out_len)
{
    size_t used = 0;
    size_t header_end = 0;

    /*
     * One budget for the whole request, not per poll(). Passing a fixed
     * timeout to each poll lets a peer that dribbles one byte every 1.9 s
     * hold this connection open forever — and because the server is
     * single-threaded, that wedges every other client with it.
     */
    const int  TOTAL_MS = 2000;
    const long start_ms = now_ms();

    for (;;) {
        long left = TOTAL_MS - (now_ms() - start_ms);
        if (left <= 0) return -1;

        struct pollfd p = { cfd, POLLIN, 0 };
        int pr = poll(&p, 1, (int)left);
        if (pr <= 0) return -1;

        if (used + 1 >= cap) return -2;           /* too large */
        ssize_t n = read(cfd, buf + used, cap - used - 1);
        if (n <= 0) return -1;
        used += (size_t)n;
        buf[used] = '\0';

        if (!header_end) {
            char *e = strstr(buf, "\r\n\r\n");
            if (e) header_end = (size_t)(e - buf) + 4;
        }
        if (header_end) {
            /* Honour Content-Length so POST bodies arrive intact. */
            size_t want = 0;
            const char *cl = strcasestr(buf, "content-length:");
            if (cl) want = (size_t)strtoul(cl + 15, NULL, 10);
            if (used >= header_end + want) { *out_len = used; return 0; }
        }
    }
}

static void send_response(int cfd, const httpd_response_t *r)
{
    char head[512];
    const char *ctype = r->content_type ? r->content_type : "text/plain";
    int n = snprintf(head, sizeof(head),
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %zu\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n"
        "\r\n",
        r->status, status_text(r->status), ctype, r->body_len);

    if (n > 0) {
        ssize_t w = write(cfd, head, (size_t)n);
        (void)w;
    }
    if (r->body && r->body_len) {
        size_t off = 0;
        while (off < r->body_len) {
            ssize_t w = write(cfd, r->body + off, r->body_len - off);
            if (w <= 0) break;
            off += (size_t)w;
        }
    }
}

int httpd_serve(httpd_t *h, int timeout_ms, httpd_handler cb, void *user)
{
    if (!h || h->fd < 0) return -1;

    struct pollfd p = { h->fd, POLLIN, 0 };
    int pr = poll(&p, 1, timeout_ms);
    if (pr < 0) {
        if (errno == EINTR) return 0;
        set_err(h, "poll: %s", strerror(errno));
        return -1;
    }
    if (pr == 0) return 0;

    int handled = 0;

    /* Drain every connection that is already pending. */
    for (;;) {
        struct sockaddr_in peer;
        socklen_t plen = sizeof(peer);
        int cfd = accept(h->fd, (struct sockaddr *)&peer, &plen);
        if (cfd < 0) break;

        int one = 1;
        setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        static char buf[HTTPD_MAX_REQUEST];
        size_t len = 0;
        int rr = read_request(cfd, buf, sizeof(buf), &len);

        httpd_request_t req;
        memset(&req, 0, sizeof(req));
        httpd_response_t resp;
        memset(&resp, 0, sizeof(resp));

        inet_ntop(AF_INET, &peer.sin_addr, req.peer, sizeof(req.peer));
        req.peer_is_loopback = (strncmp(req.peer, "127.", 4) == 0);

        if (rr == -2) {
            resp.status = 413;
            resp.content_type = "text/plain";
            resp.body = (char *)"request too large";
            resp.body_len = strlen(resp.body);
            send_response(cfd, &resp);
            close(cfd);
            handled++;
            continue;
        }
        if (rr != 0) { close(cfd); continue; }

        /* Request line: METHOD SP PATH SP VERSION */
        const char *sp1 = strchr(buf, ' ');
        const char *sp2 = sp1 ? strchr(sp1 + 1, ' ') : NULL;
        if (!sp1 || !sp2) {
            resp.status = 400;
            resp.content_type = "text/plain";
            resp.body = (char *)"bad request";
            resp.body_len = strlen(resp.body);
            send_response(cfd, &resp);
            close(cfd);
            handled++;
            continue;
        }

        size_t mlen = (size_t)(sp1 - buf);
        if (mlen >= sizeof(req.method)) mlen = sizeof(req.method) - 1;
        memcpy(req.method, buf, mlen);
        req.method[mlen] = '\0';

        size_t plen2 = (size_t)(sp2 - sp1 - 1);
        if (plen2 >= sizeof(req.path)) plen2 = sizeof(req.path) - 1;
        memcpy(req.path, sp1 + 1, plen2);
        req.path[plen2] = '\0';

        /* Pull the two headers the handler needs for origin checks. */
        {
            static const struct { const char *name; size_t len; } want[] = {
                { "\norigin:", 8 }, { "\nhost:", 6 }
            };
            char *dst[2] = { req.origin, req.host };
            size_t cap[2] = { sizeof(req.origin), sizeof(req.host) };
            for (int i = 0; i < 2; i++) {
                const char *hv = strcasestr(buf, want[i].name);
                if (!hv) continue;
                hv += want[i].len;
                while (*hv == ' ' || *hv == '\t') hv++;
                size_t n = 0;
                while (hv[n] && hv[n] != '\r' && hv[n] != '\n' && n + 1 < cap[i]) n++;
                memcpy(dst[i], hv, n);
                dst[i][n] = '\0';
            }
        }

        const char *he = strstr(buf, "\r\n\r\n");
        if (he) {
            const char *body = he + 4;
            size_t blen = len - (size_t)(body - buf);
            if (blen >= sizeof(req.body)) blen = sizeof(req.body) - 1;
            memcpy(req.body, body, blen);
            req.body[blen] = '\0';
            req.body_len = blen;
        }

        resp.status = 200;
        if (cb) cb(&req, &resp, user);
        if (!resp.body) {
            resp.status = resp.status == 200 ? 204 : resp.status;
            resp.body_len = 0;
        }

        send_response(cfd, &resp);
        if (resp.owned && resp.body) free(resp.body);
        close(cfd);
        handled++;
    }

    return handled;
}

/* ---------------------------------------------------------------- forms -- */

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

void httpd_url_decode(const char *in, char *out, size_t out_cap)
{
    if (!out || out_cap == 0) return;
    size_t o = 0;
    for (size_t i = 0; in && in[i] && o + 1 < out_cap; i++) {
        if (in[i] == '%' && hexval(in[i + 1]) >= 0 && hexval(in[i + 2]) >= 0) {
            out[o++] = (char)(hexval(in[i + 1]) * 16 + hexval(in[i + 2]));
            i += 2;
        } else if (in[i] == '+') {
            out[o++] = ' ';
        } else {
            out[o++] = in[i];
        }
    }
    out[o] = '\0';
}

int httpd_form_value(const char *body, const char *key,
                     char *out, size_t out_cap)
{
    if (!body || !key || !out || out_cap == 0) return 0;

    size_t klen = strlen(key);
    const char *p = body;

    while (p && *p) {
        /*
         * Bound the search to this field first. Looking for '=' with a plain
         * strchr scans straight past the '&' into the next field, so a
         * valueless parameter ("flag&pan=1") makes every key after it
         * invisible — and a rate command would then be read as zero.
         */
        const char *amp = strchr(p, '&');
        const char *eq  = strchr(p, '=');
        if (eq && amp && eq > amp) eq = NULL;   /* this field has no value */

        if (eq && (size_t)(eq - p) == klen && strncmp(p, key, klen) == 0) {
            size_t vlen = amp ? (size_t)(amp - eq - 1) : strlen(eq + 1);
            char raw[256];
            if (vlen >= sizeof(raw)) vlen = sizeof(raw) - 1;
            memcpy(raw, eq + 1, vlen);
            raw[vlen] = '\0';
            httpd_url_decode(raw, out, out_cap);
            return 1;
        }
        p = amp ? amp + 1 : NULL;
    }
    return 0;
}
