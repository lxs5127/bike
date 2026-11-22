/*
 * Wi-Fi 任务独占 UDP 套接字和客户端会话。控制任务只提交快照和事件。
 * 断线结束网络会话，控制任务按原有目标继续运行。
 */
#include "wifi_link.h"
#include "wifi_session.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "cJSON.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

static int s_socket = -1;
static QueueHandle_t s_samples;   /* 单槽快照，新数据覆盖尚未发送的旧快照 */
static QueueHandle_t s_events;    /* 有界队列，保留停机原因和命令执行结果 */
static wifi_command_callback_t s_command;
static atomic_bool s_disconnected;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_ready_until;     /* 对外暴露的租约截止时间，0 表示没有会话 */
static const char *TAG = "bike_wifi";

static void set_deadline(int64_t deadline)
{
    portENTER_CRITICAL(&s_lock);
    s_ready_until = deadline;
    portEXIT_CRITICAL(&s_lock);
}

bool wifi_link_ready(void)
{
    portENTER_CRITICAL(&s_lock);
    int64_t deadline = s_ready_until;
    portEXIT_CRITICAL(&s_lock);
    return deadline > 0 && esp_timer_get_time() <= deadline;
}

void wifi_link_event(const char *format, ...)
{
    if (!s_events) return;

    char message[192];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    /* 事件满时丢弃本条消息，避免控制任务等待网络发送。 */
    xQueueSend(s_events, message, 0);
}

void wifi_link_publish(const wifi_telemetry_t *sample)
{
    if (s_samples) xQueueOverwrite(s_samples, sample);
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)data;
    if (id == WIFI_EVENT_AP_STADISCONNECTED || id == WIFI_EVENT_AP_STOP) {
        /* 事件回调通知网络任务清理会话，peer 仍由网络任务修改。 */
        set_deadline(0);
        atomic_store(&s_disconnected, true);
    }
}

static void send_bytes(const wifi_session_t *peer, const char *bytes)
{
    struct sockaddr_in target = {
        .sin_family = AF_INET,
        .sin_port = peer->port,
        .sin_addr.s_addr = peer->address,
    };
    sendto(s_socket, bytes, strlen(bytes), MSG_DONTWAIT,
           (struct sockaddr *)&target, sizeof(target));
}

static void send_message(const wifi_session_t *peer, const char *type,
                         uint64_t sequence, bool ok, const char *message)
{
    char token[17], client[17], bytes[640];

    /* 64 位标识用十六进制字符串传输，避免 JSON 数字的精度损失。 */
    snprintf(token, sizeof(token), "%016" PRIx64, peer->token);
    snprintf(client, sizeof(client), "%016" PRIx64, peer->client);
    cJSON *json = cJSON_CreateObject();
    if (!json) return;
    cJSON_AddStringToObject(json, "type", type);
    cJSON_AddStringToObject(json, "token", token);
    cJSON_AddStringToObject(json, "client", client);
    cJSON_AddNumberToObject(json, "sequence", (double)sequence);
    cJSON_AddNumberToObject(json, "time_us", (double)esp_timer_get_time());
    cJSON_AddBoolToObject(json, "ok", ok);
    cJSON_AddStringToObject(json, "message", message);
    if (cJSON_PrintPreallocated(json, bytes, sizeof(bytes), false)) {
        send_bytes(peer, bytes);
    }
    cJSON_Delete(json);
}

static void send_sample(const wifi_session_t *peer, const wifi_telemetry_t *s)
{
    char bytes[1200];

    /* time_us 为打包时间，sample_us 为 IMU 采样时间。
     * 客户端用这两个板端时间判断采样是否过期。
     */
    int n = snprintf(bytes, sizeof(bytes),
        "{\"type\":\"sample\",\"token\":\"%016" PRIx64 "\",\"time_us\":%" PRId64
        ",\"imu_valid\":%s,\"sample_us\":%" PRId64
        ",\"acc_g\":[%.6g,%.6g,%.6g],\"gyro_deg_s\":[%.6g,%.6g,%.6g],\"angles_deg\":[%.6g,%.6g,%.6g]"
        ",\"mode\":%d,\"zero_valid\":%s,\"encoder_valid\":%s,\"tilt_deg\":%.6g"
        ",\"speed_turns_s\":%.6g,\"torque_nm\":%.6g,\"wheel_target\":%.6g"
        ",\"kp\":%.6g,\"kd\":%.6g,\"direction\":%d,\"torque_limit\":%.6g,\"flywheel_limit\":%.6g,\"wheel_limit\":%.6g}",
        peer->token, esp_timer_get_time(), s->imu.valid ? "true" : "false", s->imu.timestamp_us,
        (double)s->imu.acceleration_g[0], (double)s->imu.acceleration_g[1], (double)s->imu.acceleration_g[2],
        (double)s->imu.gyro_deg_s[0], (double)s->imu.gyro_deg_s[1], (double)s->imu.gyro_deg_s[2],
        (double)s->imu.angles_deg[0], (double)s->imu.angles_deg[1], (double)s->imu.angles_deg[2],
        s->mode, s->zero_valid ? "true" : "false", s->encoder_valid ? "true" : "false", (double)s->tilt_deg,
        (double)s->speed_turns_s, (double)s->torque_nm, (double)s->wheel_target,
        (double)s->kp, (double)s->kd, s->direction,
        (double)s->torque_limit, (double)s->flywheel_limit, (double)s->wheel_limit);
    if (n > 0 && n < (int)sizeof(bytes)) send_bytes(peer, bytes);
}

static void network_task(void *arg)
{
    (void)arg;
    wifi_session_t peer = {0};
    TickType_t wake = xTaskGetTickCount();
    while (1) {
        if (atomic_exchange(&s_disconnected, false) ||
            (peer.connected && !wifi_session_alive(&peer, esp_timer_get_time()))) {
            peer.connected = false;
            set_deadline(0);
        }

        /* 每轮限制收包数量，给遥测和事件留出发送机会。 */
        for (unsigned i = 0; i < 8; ++i) {
            char bytes[WIFI_REQUEST_MAX];
            struct sockaddr_in source = {0};
            socklen_t length = sizeof(source);
            int n = recvfrom(s_socket, bytes, sizeof(bytes), MSG_DONTWAIT,
                             (struct sockaddr *)&source, &length);
            if (n < 0) break;
            wifi_request_t request;
            if (!wifi_request_parse(bytes, (size_t)n, &request)) continue;
            uint64_t fresh_token = ((uint64_t)esp_random() << 32) | esp_random();
            wifi_accept_t accepted = wifi_session_accept(&peer, &request, source.sin_addr.s_addr,
                                                         source.sin_port, esp_timer_get_time(), fresh_token);
            if (accepted == WIFI_REJECT) continue;
            set_deadline(peer.seen_us + WIFI_LINK_TIMEOUT_US);
            if (accepted == WIFI_HELLO) {
                send_message(&peer, "welcome", request.sequence, true,
                             "Wi-Fi connected; local control is independent of this link.");
            } else {
                /* reply 回应网络请求。根控制程序随后用 event 报告实际执行结果。 */
                char reply[192] = {0};
                bool ok = s_command(request.command, reply, sizeof(reply));
                send_message(&peer, "reply", request.sequence, ok, reply);
            }
        }
        if (wifi_session_alive(&peer, esp_timer_get_time())) {
            wifi_telemetry_t sample;
            if (xQueueReceive(s_samples, &sample, 0) == pdTRUE) {
                send_sample(&peer, &sample);
            }
            char message[192];
            for (unsigned i = 0; i < 4 && xQueueReceive(s_events, message, 0) == pdTRUE; ++i) {
                send_message(&peer, "event", 0, true, message);
            }
        } else {
            /* 无有效会话时清空事件，快照仍可由控制任务继续更新。 */
            xQueueReset(s_events);
        }
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
}

esp_err_t wifi_link_init(wifi_command_callback_t command)
{
    if (!command) return ESP_ERR_INVALID_ARG;
    s_command = command;
    size_t ssid_size = strlen(CONFIG_BIKE_LINK_AP_SSID);
    size_t password_size = strlen(CONFIG_BIKE_LINK_AP_PASSWORD);
    if (!ssid_size || ssid_size > 32 || password_size < 8 || password_size > 63) return ESP_ERR_INVALID_ARG;
    s_samples = xQueueCreate(1, sizeof(wifi_telemetry_t));
    s_events = xQueueCreate(8, 192);
    if (!s_samples || !s_events) return ESP_ERR_NO_MEM;
    ESP_RETURN_ON_ERROR(esp_netif_init(),TAG,"network init");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(),TAG,"event init");
    if (!esp_netif_create_default_wifi_ap()) return ESP_ERR_NO_MEM;
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init),TAG,"Wi-Fi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM),TAG,"Wi-Fi storage");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_event,NULL),TAG,"Wi-Fi events");
    wifi_config_t config = {0};
    memcpy(config.ap.ssid,CONFIG_BIKE_LINK_AP_SSID,ssid_size);
    memcpy(config.ap.password,CONFIG_BIKE_LINK_AP_PASSWORD,password_size);
    config.ap.ssid_len = ssid_size;
    config.ap.channel = CONFIG_BIKE_LINK_AP_CHANNEL;
    config.ap.max_connection = 1;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP),TAG,"AP mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP,&config),TAG,"AP config");
    s_socket = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if (s_socket < 0) return ESP_FAIL;
    struct sockaddr_in local = {.sin_family=AF_INET,.sin_port=htons(BIKE_WIFI_UDP_PORT),
                               .sin_addr.s_addr=htonl(INADDR_ANY)};
    if (bind(s_socket,(struct sockaddr *)&local,sizeof(local))) {
        close(s_socket); s_socket = -1; return ESP_FAIL;
    }
    ESP_RETURN_ON_ERROR(esp_wifi_start(),TAG,"AP start");
    if (xTaskCreate(network_task,"wifi_data",6144,NULL,5,NULL) != pdPASS) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG,"PC link: SSID=%s IP=192.168.4.1 UDP=%d; Wi-Fi telemetry",CONFIG_BIKE_LINK_AP_SSID,BIKE_WIFI_UDP_PORT);
    return ESP_OK;
}
