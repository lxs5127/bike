#include "wifi_diagnostic.h"

#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "wifi_diag";

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "AP_STARTED: hotspot=%s", CONFIG_BIKE_WIFI_AP_SSID);
    } else if (id == WIFI_EVENT_AP_STOP) {
        ESP_LOGW(TAG, "AP_STOPPED");
    }
}

/* Same basic startup order as examples/wifi/getting_started/softAP.
 * Deliberately no UDP, custom DHCP, bike timers, ports or NVS erase.
 */
static esp_err_t start_softap(void)
{
    const size_t ssid_len = strlen(CONFIG_BIKE_WIFI_AP_SSID);
    const size_t password_len = strlen(CONFIG_BIKE_WIFI_AP_PASSWORD);
    if (ssid_len == 0 || ssid_len > 32 || password_len < 8 || password_len > 63) {
        ESP_LOGE(TAG, "Invalid SSID/password length");
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "stage=nvs_network");
    ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "NVS init failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Network init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Event loop init failed");
    if (!esp_netif_create_default_wifi_ap()) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "stage=wifi_driver");
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "Wi-Fi driver init failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "Wi-Fi storage failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                  wifi_event, NULL), TAG, "Wi-Fi events failed");
    wifi_config_t config = {0};
    memcpy(config.ap.ssid, CONFIG_BIKE_WIFI_AP_SSID, ssid_len);
    memcpy(config.ap.password, CONFIG_BIKE_WIFI_AP_PASSWORD, password_len);
    config.ap.ssid_len = ssid_len;
    config.ap.channel = CONFIG_BIKE_WIFI_AP_CHANNEL;
    config.ap.max_connection = 1;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "AP mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &config), TAG, "AP config failed");
    ESP_LOGI(TAG, "stage=wifi_start: enabling radio now");
    return esp_wifi_start();
}

void wifi_diagnostic_run(void)
{
    ESP_LOGI(TAG, "BIKE_FW=wifi-diagnostic-v2 reset_reason=%d MOTOR_OUTPUT=OFF",
             (int)esp_reset_reason());
    ESP_LOGI(TAG, "PHY init TX limit=%d dBm (build-time)", CONFIG_ESP_PHY_MAX_WIFI_TX_POWER);
    ESP_LOGI(TAG, "Ten-second RF_OFF baseline, then minimal SoftAP; no policy/UDP/bike tasks");
    for (unsigned second = 0; second < 10; ++second) {
        ESP_LOGI(TAG, "RF_OFF alive=%us free_heap=%" PRIu32,
                 second, esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    const esp_err_t result = start_softap();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi diagnostic startup failed: %s", esp_err_to_name(result));
    }
    for (unsigned second = 0; ; ++second) {
        ESP_LOGI(TAG, "%s alive=%us free_heap=%" PRIu32,
                 result == ESP_OK ? "WIFI_START_RETURNED_OK" : "WIFI_START_FAILED",
                 second, esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
