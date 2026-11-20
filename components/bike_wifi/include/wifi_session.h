#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define WIFI_LINK_TIMEOUT_US 500000LL
#define WIFI_COMMAND_MAX_AGE_US 300000LL
#define WIFI_REQUEST_MAX 192
typedef struct {
    uint64_t client, token, sequence, board_us;
    char command[128];
} wifi_request_t;
typedef struct {
    uint64_t client, token, sequence;
    uint32_t address;
    uint16_t port;
    int64_t seen_us;
    bool connected;
} wifi_session_t;
typedef enum { WIFI_REJECT, WIFI_HELLO, WIFI_COMMAND } wifi_accept_t;
bool wifi_request_parse(const char *data, size_t size, wifi_request_t *out);
bool wifi_session_alive(const wifi_session_t *s, int64_t now);
wifi_accept_t wifi_session_accept(wifi_session_t *s, const wifi_request_t *r,
                                 uint32_t address, uint16_t port, int64_t now,
                                 uint64_t new_token);
