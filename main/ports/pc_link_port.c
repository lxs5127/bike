#include "pc_link_port.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "lwip/sockets.h"
#include "app_config.h"
#include "bike_protocol.h"
#include "bike_udp_peer.h"

static const char *TAG = "bike_wifi";
static int s_socket = -1;
static portMUX_TYPE s_peer_lock = portMUX_INITIALIZER_UNLOCKED;
static bike_udp_peer_t s_peer;

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "AP_STARTED: hotspot=%s channel=%d IP=192.168.4.1",
                 CONFIG_BIKE_WIFI_AP_SSID, CONFIG_BIKE_WIFI_AP_CHANNEL);
    } else if (id == WIFI_EVENT_AP_STOP) {
        portENTER_CRITICAL(&s_peer_lock);
        s_peer.registered = false;
        portEXIT_CRITICAL(&s_peer_lock);
        ESP_LOGW(TAG, "AP_STOPPED: hotspot stopped");
    } else if (id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Laptop joined hotspot; waiting for UDP HELLO");
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        portENTER_CRITICAL(&s_peer_lock);
        s_peer.registered = false;
        portEXIT_CRITICAL(&s_peer_lock);
        ESP_LOGW(TAG, "Laptop disconnected; command link disabled");
    }
}

esp_err_t pc_link_port_init(void)
{
    size_t ssid_size = strlen(CONFIG_BIKE_WIFI_AP_SSID);
    size_t password_size = strlen(CONFIG_BIKE_WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "stage=configuration SSID=%s channel=%d free_heap=%" PRIu32,
             CONFIG_BIKE_WIFI_AP_SSID, CONFIG_BIKE_WIFI_AP_CHANNEL, esp_get_free_heap_size());
    ESP_LOGI(TAG, "PHY init TX limit=%d dBm (build-time)", CONFIG_ESP_PHY_MAX_WIFI_TX_POWER);
    if (!ssid_size || ssid_size > 32 || password_size < 8 || password_size > 63) {
        ESP_LOGE(TAG, "Invalid AP configuration: SSID must be 1-32 bytes, password 8-63 bytes");
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "stage=nvs_network");
    ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "NVS initialization failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Network initialization failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Event loop initialization failed");
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();
    if (!ap) {
        ESP_LOGE(TAG, "Creating AP network interface failed");
        return ESP_ERR_NO_MEM;
    }
    esp_err_t stopped = esp_netif_dhcps_stop(ap);
    if (stopped != ESP_OK && stopped != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGE(TAG, "AP DHCP stop failed: %s", esp_err_to_name(stopped));
        return stopped;
    }
    /* Set before Wi-Fi start / DHCP server start; stable address for the PC. */
    esp_netif_ip_info_t ip = {0};
    ip.ip.addr = inet_addr("192.168.4.1");
    ip.gw.addr = ip.ip.addr;
    ip.netmask.addr = inet_addr("255.255.255.0");
    ESP_RETURN_ON_ERROR(esp_netif_set_ip_info(ap, &ip), TAG, "AP address configuration failed");
    ESP_RETURN_ON_ERROR(esp_netif_dhcps_start(ap), TAG, "AP DHCP initialization failed");
    ESP_LOGI(TAG, "stage=wifi_driver");
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "Wi-Fi initialization failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "Wi-Fi storage configuration failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                  wifi_event, NULL), TAG, "Wi-Fi event handler failed");
    wifi_config_t config = {0};
    memcpy(config.ap.ssid, CONFIG_BIKE_WIFI_AP_SSID, ssid_size);
    memcpy(config.ap.password, CONFIG_BIKE_WIFI_AP_PASSWORD, password_size);
    config.ap.ssid_len = ssid_size;
    config.ap.channel = CONFIG_BIKE_WIFI_AP_CHANNEL;
    config.ap.max_connection = 1;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    config.ap.pmf_cfg.required = false;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "AP mode configuration failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &config), TAG, "AP configuration failed");

    ESP_LOGI(TAG, "stage=udp_socket");
    s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_socket < 0) {
        ESP_LOGE(TAG, "Creating UDP socket failed: errno=%d", errno);
        return ESP_FAIL;
    }
    struct timeval timeout = {.tv_sec = 0, .tv_usec = 100000};
    const struct sockaddr_in local = {
        .sin_family = AF_INET, .sin_port = htons(BIKE_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (setsockopt(s_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0 ||
        bind(s_socket, (const struct sockaddr *)&local, sizeof(local)) != 0) {
        ESP_LOGE(TAG, "Configuring/binding UDP socket failed: errno=%d", errno);
        close(s_socket);
        s_socket = -1;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "stage=wifi_start");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");
    ESP_LOGI(TAG, "SSID=%s IP=192.168.4.1 UDP=%u; policy=60Hz mock=%d MOTOR_OUTPUT=OFF",
             CONFIG_BIKE_WIFI_AP_SSID, BIKE_UDP_PORT, CONFIG_BIKE_MOCK_HARDWARE);
    return ESP_OK;
}

bool pc_link_port_ready(void)
{
    portENTER_CRITICAL(&s_peer_lock);
    bool ready = bike_udp_peer_alive(&s_peer, esp_timer_get_time());
    portEXIT_CRITICAL(&s_peer_lock);
    return ready;
}

esp_err_t pc_link_port_send(const uint8_t *bytes, size_t size)
{
    portENTER_CRITICAL(&s_peer_lock);
    bool ready = bike_udp_peer_alive(&s_peer, esp_timer_get_time());
    struct sockaddr_in destination = {
        .sin_family = AF_INET, .sin_addr.s_addr = s_peer.address, .sin_port = s_peer.port,
    };
    portEXIT_CRITICAL(&s_peer_lock);
    if (!ready) return ESP_ERR_NOT_FOUND; /* startup/disconnect is a normal wait */
    int sent = sendto(s_socket, bytes, size, MSG_DONTWAIT,
                      (const struct sockaddr *)&destination, sizeof(destination));
    return sent == (int)size ? ESP_OK : ESP_FAIL;
}

int pc_link_port_read(uint8_t *bytes, size_t capacity)
{
    struct sockaddr_in source = {0};
    socklen_t length = sizeof(source);
    int count = recvfrom(s_socket, bytes, capacity, 0, (struct sockaddr *)&source, &length);
    if (count < 0) return errno == EAGAIN || errno == EWOULDBLOCK ? 0 : -1;
    bool hello = bike_validate_packet(bytes, count, BIKE_PACKET_HELLO);
    bool command = bike_validate_packet(bytes, count, BIKE_PACKET_COMMAND);
    if (!hello && !command) return 0;
    uint64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_peer_lock);
    bool accepted = hello ? bike_udp_peer_hello(&s_peer, source.sin_addr.s_addr,
                                               source.sin_port, now_us) :
                            bike_udp_peer_command(&s_peer, source.sin_addr.s_addr,
                                                  source.sin_port, now_us);
    portEXIT_CRITICAL(&s_peer_lock);
    /* HELLO registers return IP/port only; it never refreshes command validity. */
    return command && accepted ? count : 0;
}
