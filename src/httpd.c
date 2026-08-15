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
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

static void client_free(httpd_client_t *c);

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
    /* Callers declare httpd_t on the stack, so the slots are indeterminate
     * until they are claimed. A garbage fd here would be closed, polled and
     * read as though it were a client. */
    for (int i = 0; i < HTTPD_MAX_CLIENTS; i++) {
        h->client[i].fd = -1;
        h->client[i].len = 0;
        h->client[i].header_end = 0;
        h->client[i].deadline_ms = 0;
        h->client[i].peer[0] = '\0';
    }

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
    for (int i = 0; i < HTTPD_MAX_CLIENTS; i++)
        if (h->client[i].fd >= 0) client_free(&h->client[i]);
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

static void client_free(httpd_client_t *c)
{
    if (c->fd >= 0) close(c->fd);
    c->fd = -1;
    c->len = 0;
    c->header_end = 0;
    c->deadline_ms = 0;
    c->peer[0] = '\0';
}

/*
 * Fold whatever has arrived into the slot's state.
 *
 * Returns 1 when the request is complete, 0 when more is needed, -1 when the
 * peer went away, and -2 when it does not fit. Nothing here blocks: the socket
 * is non-blocking and this is called only after poll() said it was readable,
 * so one stalled peer costs the others nothing. That is the whole point —
 * reading each connection to completion in turn meant every half-open socket
 * cost the next client the full request timeout.
 */
static int client_read(httpd_client_t *c)
{
    if (c->len + 1 >= sizeof(c->buf)) return -2;

    ssize_t n = read(c->fd, c->buf + c->len, sizeof(c->buf) - c->len - 1);
    if (n < 0) return (errno == EAGAIN || errno == EWOULDBLOCK ||
                       errno == EINTR) ? 0 : -1;
    if (n == 0) return -1;                    /* peer closed */

    c->len += (size_t)n;
    c->buf[c->len] = '\0';

    if (!c->header_end) {
        char *e = strstr(c->buf, "\r\n\r\n");
        if (e) c->header_end = (size_t)(e - c->buf) + 4;
    }
    if (c->header_end) {
        /* Honour Content-Length so POST bodies arrive intact. */
        size_t want = 0;
        const char *cl = strcasestr(c->buf, "content-length:");
        if (cl) want = (size_t)strtoul(cl + 15, NULL, 10);
        if (c->len >= c->header_end + want) return 1;
    }
    if (c->len + 1 >= sizeof(c->buf)) return -2;
    return 0;
}

static void send_response(int cfd, const httpd_response_t *r)
{
    /*
     * Requests are read non-blockingly, but the reply is written blocking with
     * a send timeout. Buffering partial writes would mean carrying response
     * state per slot for no practical gain: API replies are a few KB and fit
     * the socket buffer outright, and the one large body — the compiled-in
     * page — only stalls against a peer that asked for it and then stopped
     * reading. The timeout bounds that case instead of letting it hang.
     */
    int fl = fcntl(cfd, F_GETFL, 0);
    if (fl >= 0) fcntl(cfd, F_SETFL, fl & ~O_NONBLOCK);
    struct timeval tv = { HTTPD_CLIENT_TIMEOUT_MS / 1000,
                          (HTTPD_CLIENT_TIMEOUT_MS % 1000) * 1000 };
    setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

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

/* Turn one fully-received request into a response and send it. */
static void dispatch(httpd_client_t *c, httpd_handler cb, void *user)
{
    char *buf = c->buf;

    httpd_request_t req;
    memset(&req, 0, sizeof(req));
    httpd_response_t resp;
    memset(&resp, 0, sizeof(resp));

    snprintf(req.peer, sizeof(req.peer), "%s", c->peer);
    req.peer_is_loopback = (strncmp(req.peer, "127.", 4) == 0);

    /* Request line: METHOD SP PATH SP VERSION */
    const char *sp1 = strchr(buf, ' ');
    const char *sp2 = sp1 ? strchr(sp1 + 1, ' ') : NULL;
    if (!sp1 || !sp2) {
        resp.status = 400;
        resp.content_type = "text/plain";
        resp.body = (char *)"bad request";
        resp.body_len = strlen(resp.body);
        send_response(c->fd, &resp);
        return;
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

    if (c->header_end) {
        const char *body = buf + c->header_end;
        size_t blen = c->len - c->header_end;
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

    send_response(c->fd, &resp);
    if (resp.owned && resp.body) free(resp.body);
}

/*
 * Accept everything pending, then advance every connection that has data.
 *
 * The listener and all in-flight requests share one poll set, so a peer that
 * sends a partial request and stops costs nothing but a slot until its
 * deadline expires. Previously each connection was read to completion before
 * the next was accepted, which made half-open sockets additive: four of them
 * delayed a legitimate request by nearly eight seconds, long enough to starve
 * the browser's rate republisher and trip the motion watchdog.
 */
int httpd_serve(httpd_t *h, int timeout_ms, httpd_handler cb, void *user)
{
    if (!h || h->fd < 0) return -1;

    struct pollfd p[1 + HTTPD_MAX_CLIENTS];
    int slot[1 + HTTPD_MAX_CLIENTS];
    nfds_t n = 0;

    p[n].fd = h->fd; p[n].events = POLLIN; p[n].revents = 0;
    slot[n] = -1;
    n++;

    long now = now_ms();
    int wait_ms = timeout_ms;
    for (int i = 0; i < HTTPD_MAX_CLIENTS; i++) {
        if (h->client[i].fd < 0) continue;
        /* Wake in time to retire the earliest deadline. */
        long left = h->client[i].deadline_ms - now;
        if (left < 0) left = 0;
        if (wait_ms < 0 || left < wait_ms) wait_ms = (int)left;
        p[n].fd = h->client[i].fd; p[n].events = POLLIN; p[n].revents = 0;
        slot[n] = i;
        n++;
    }

    int pr = poll(p, n, wait_ms);
    if (pr < 0) {
        if (errno == EINTR) return 0;
        set_err(h, "poll: %s", strerror(errno));
        return -1;
    }

    int handled = 0;
    now = now_ms();

    /* New connections first, so a burst is not left waiting a whole cycle. */
    if (p[0].revents & POLLIN) {
        for (;;) {
            struct sockaddr_in peer;
            socklen_t plen = sizeof(peer);
            int cfd = accept(h->fd, (struct sockaddr *)&peer, &plen);
            if (cfd < 0) break;

            int free_slot = -1;
            for (int i = 0; i < HTTPD_MAX_CLIENTS; i++)
                if (h->client[i].fd < 0) { free_slot = i; break; }

            /*
             * Out of slots: evict the oldest incomplete request rather than
             * refusing the new one.
             *
             * Deadlines are stamped at accept time, so the oldest slot is the
             * one that has had the longest to finish and has not. A real
             * request completes in milliseconds; a slot still open after that
             * is almost certainly a peer that stopped sending. Refusing the
             * newcomer instead would let enough half-open sockets to fill the
             * table lock every later client out until their deadlines expired,
             * which is the same starvation this rewrite exists to remove.
             */
            if (free_slot < 0) {
                int oldest = 0;
                for (int i = 1; i < HTTPD_MAX_CLIENTS; i++)
                    if (h->client[i].deadline_ms < h->client[oldest].deadline_ms)
                        oldest = i;
                client_free(&h->client[oldest]);
                free_slot = oldest;
            }

            int one = 1;
            setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            int fl = fcntl(cfd, F_GETFL, 0);
            fcntl(cfd, F_SETFL, fl | O_NONBLOCK);

            httpd_client_t *c = &h->client[free_slot];
            c->fd = cfd;
            c->len = 0;
            c->header_end = 0;
            c->deadline_ms = now + HTTPD_CLIENT_TIMEOUT_MS;
            inet_ntop(AF_INET, &peer.sin_addr, c->peer, sizeof(c->peer));
        }
    }

    /* Then advance whatever already had bytes waiting. */
    for (nfds_t k = 1; k < n; k++) {
        int i = slot[k];
        httpd_client_t *c = &h->client[i];
        if (c->fd < 0) continue;

        /*
         * The accept loop above runs after the poll snapshot was taken and may
         * have evicted this slot and handed it to a brand-new connection. The
         * revents in hand describe the socket that used to be here, so acting
         * on them would read — or, on a stale POLLHUP, hang up — the wrong
         * client. The fd is the identity; if it changed, this entry is spent.
         */
        if (c->fd != p[k].fd) continue;

        if (p[k].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            client_free(c);
            continue;
        }
        if (!(p[k].revents & POLLIN)) continue;

        int rr = client_read(c);
        if (rr == 1) {
            dispatch(c, cb, user);
            client_free(c);
            handled++;
        } else if (rr == -2) {
            httpd_response_t resp;
            memset(&resp, 0, sizeof(resp));
            resp.status = 413;
            resp.content_type = "text/plain";
            resp.body = (char *)"request too large";
            resp.body_len = strlen(resp.body);
            send_response(c->fd, &resp);
            client_free(c);
            handled++;
        } else if (rr < 0) {
            client_free(c);
        }
    }

    /* Retire anything that ran out of time. */
    for (int i = 0; i < HTTPD_MAX_CLIENTS; i++)
        if (h->client[i].fd >= 0 && now >= h->client[i].deadline_ms)
            client_free(&h->client[i]);

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
