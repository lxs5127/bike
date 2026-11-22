#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define WIFI_LINK_TIMEOUT_US 500000LL     /* 会话租约为 500 ms */
#define WIFI_COMMAND_MAX_AGE_US 300000LL  /* 普通命令引用的板端时间最长可滞后 300 ms */
#define WIFI_REQUEST_MAX 192

typedef struct {
    uint64_t client;     /* 客户端启动时生成的标识 */
    uint64_t token;      /* hello 使用 0，普通命令使用板端分配的令牌 */
    uint64_t sequence;   /* 同一客户端的报文序号 */
    uint64_t board_us;   /* 客户端已收到的最新 ESP32 时间 */
    char command[128];
} wifi_request_t;

typedef struct {
    uint64_t client, token, sequence;
    uint32_t address;    /* 以下两项都是 recvfrom() 返回的网络字节序 */
    uint16_t port;
    int64_t seen_us;     /* 最近一次通过校验的报文接收时间 */
    bool connected;
} wifi_session_t;

typedef enum {
    WIFI_REJECT,
    WIFI_HELLO,
    WIFI_COMMAND,
} wifi_accept_t;

bool wifi_request_parse(const char *data, size_t size, wifi_request_t *out);
bool wifi_session_alive(const wifi_session_t *s, int64_t now);
wifi_accept_t wifi_session_accept(wifi_session_t *s, const wifi_request_t *r,
                                  uint32_t address, uint16_t port, int64_t now,
                                  uint64_t new_token);
