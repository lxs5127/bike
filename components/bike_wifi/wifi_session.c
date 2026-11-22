#include "wifi_session.h"

#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

static bool number(char **cursor, unsigned base, uint64_t *value)
{
    char *start = *cursor, *end;
    if (!(base == 16 ? isxdigit((unsigned char)*start) : isdigit((unsigned char)*start))) return false;
    errno = 0;
    unsigned long long parsed = strtoull(start, &end, base);
    if (end == start || errno || *end != ' ') return false;
    *value = parsed;
    *cursor = end + 1;
    return true;
}

bool wifi_request_parse(const char *data, size_t size, wifi_request_t *out)
{
    if (!data || !out || size < 16 || size >= WIFI_REQUEST_MAX) return false;

    /* UDP 数据没有字符串结束符。仅接受可打印 ASCII，再在副本末尾补 '\0'。 */
    char text[WIFI_REQUEST_MAX];
    for (size_t i = 0; i < size; ++i) {
        if ((unsigned char)data[i] < 32 || (unsigned char)data[i] > 126) return false;
    }
    memcpy(text, data, size);
    text[size] = 0;

    /* BIKE1 <client> <token> <sequence> <board_us> <command...>
     * 客户端和令牌用十六进制，其余字段用十进制。命令保留空格以携带参数。
     */
    if (strncmp(text, "BIKE1 ", 6)) return false;
    wifi_request_t r = {0};
    char *cursor = text + 6;
    if (!number(&cursor, 16, &r.client) || !number(&cursor, 16, &r.token) ||
        !number(&cursor, 10, &r.sequence) || !number(&cursor, 10, &r.board_us) ||
        !r.client || !r.sequence || !*cursor || strlen(cursor) >= sizeof(r.command)) {
        return false;
    }
    strcpy(r.command, cursor);
    *out = r;
    return true;
}

bool wifi_session_alive(const wifi_session_t *s, int64_t now)
{
    return s->connected && now >= s->seen_us && now - s->seen_us <= WIFI_LINK_TIMEOUT_US;
}

wifi_accept_t wifi_session_accept(wifi_session_t *s, const wifi_request_t *r,
                                  uint32_t address, uint16_t port, int64_t now,
                                  uint64_t new_token)
{
    bool hello = !strcmp(r->command, "hello") && r->token == 0;
    if (!wifi_session_alive(s, now)) {
        /* 只有 hello 可以建立会话。租约失效后重连使用新令牌。 */
        if (!hello || !new_token) return WIFI_REJECT;
        *s = (wifi_session_t){
            .client = r->client,
            .token = new_token,
            .address = address,
            .port = port,
            .connected = true,
        };
    }

    /* 当前客户端用 hello 或命令续约。过滤来源不符、乱序和重复的报文。 */
    if (s->client != r->client || s->address != address || s->port != port ||
        r->sequence <= s->sequence) {
        return WIFI_REJECT;
    }

    /* 客户端回传板端时间，无需同步两端时钟。stop 可跳过时效检查，
     * 但仍需持有当前会话令牌，并满足来源和序号检查。
     */
    if (!hello && (r->token != s->token ||
                   (strcmp(r->command, "stop") &&
                    (r->board_us == 0 || r->board_us > (uint64_t)now ||
                     (uint64_t)now - r->board_us > WIFI_COMMAND_MAX_AGE_US)))) {
        return WIFI_REJECT;
    }
    s->sequence = r->sequence;
    s->seen_us = now;
    return hello ? WIFI_HELLO : WIFI_COMMAND;
}
