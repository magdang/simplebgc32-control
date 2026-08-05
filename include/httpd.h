/*
 * httpd.h — a very small HTTP/1.1 server, POSIX sockets only.
 *
 * Exists so the GUI can be a single self-contained binary with no third-party
 * dependencies, matching the rest of this project. It serves one page and a
 * JSON status endpoint to a handful of clients on a trusted network; it is not
 * a general-purpose web server and makes no attempt to be one.
 *
 * Deliberate limits, so nobody mistakes it for more than it is:
 *   - one request per connection (no keep-alive, no pipelining)
 *   - request bodies and headers are capped and oversized requests refused
 *   - no TLS, no auth
 * Bind to loopback unless you actually intend to expose it.
 */

#ifndef HTTPD_H
#define HTTPD_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HTTPD_MAX_REQUEST 8192
#define HTTPD_MAX_PATH     256
#define HTTPD_MAX_BODY     4096

typedef struct {
    char method[8];
    char path[HTTPD_MAX_PATH];
    char body[HTTPD_MAX_BODY];
    size_t body_len;
    /* Peer address, so the handler can distinguish loopback from remote. */
    char peer[64];
    int  peer_is_loopback;

    /*
     * Origin and Host, needed to tell a request from our own page apart from
     * one a different site made on the operator's behalf. A cross-origin POST
     * needs no preflight, so a method check alone does not stop it.
     */
    char origin[128];
    char host[128];
} httpd_request_t;

typedef struct {
    int         status;         /* 200, 403, 404 ...                        */
    const char *content_type;   /* e.g. "application/json"                  */
    /* Body bytes. If `owned` is set the server free()s them after sending. */
    char       *body;
    size_t      body_len;
    int         owned;
} httpd_response_t;

/*
 * Called for each request. Fill in `resp`. Returning without setting a body
 * produces a 204.
 */
typedef void (*httpd_handler)(const httpd_request_t *req,
                              httpd_response_t *resp, void *user);

typedef struct {
    int fd;                 /* listening socket, -1 when closed */
    char last_error[192];
} httpd_t;

/*
 * Bind and listen. `bind_addr` is a dotted-quad; "127.0.0.1" for loopback
 * only, "0.0.0.0" to accept from the network. Returns 0 on success.
 */
int  httpd_open(httpd_t *h, const char *bind_addr, int port);
void httpd_close(httpd_t *h);
const char *httpd_last_error(const httpd_t *h);

/*
 * Serve for up to timeout_ms, dispatching any requests that arrive. Returns
 * the number of requests handled, or -1 on error. Use timeout_ms = 0 to poll.
 */
int  httpd_serve(httpd_t *h, int timeout_ms, httpd_handler cb, void *user);

/* Percent-decode `in` into `out` in place of the usual query parsing. */
void httpd_url_decode(const char *in, char *out, size_t out_cap);

/*
 * Look up a key in an application/x-www-form-urlencoded or query string.
 * Returns 1 and fills `out` when found, 0 otherwise.
 */
int  httpd_form_value(const char *body, const char *key,
                      char *out, size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif /* HTTPD_H */
