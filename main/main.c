#include "esp_log.h"
#include "sdkconfig.h"
#if CONFIG_BIKE_WIFI_DIAGNOSTIC
#include "wifi_diagnostic.h"
#else
#include "bike_app.h"
#endif

void app_main(void)
{
#if CONFIG_BIKE_WIFI_DIAGNOSTIC
    wifi_diagnostic_run();
#else
    ESP_LOGI("main", "BIKE_FW=wifi-udp-v4; laptop inference; MOTOR_OUTPUT=OFF");
#if CONFIG_BIKE_IMU_JY901P
    ESP_LOGI("main", "IMU=JY901P I2C SDA=%d SCL=%d addr=0x%02x; MOTOR_OUTPUT=OFF",
             CONFIG_BIKE_IMU_SDA_GPIO, CONFIG_BIKE_IMU_SCL_GPIO, CONFIG_BIKE_IMU_I2C_ADDRESS);
#if CONFIG_BIKE_MOCK_HARDWARE
    ESP_LOGI("main", "FEEDBACK_MODE=real_imu_mock_drive; flywheel speed=0 (synthetic)");
#endif
#elif CONFIG_BIKE_MOCK_HARDWARE
#if CONFIG_BIKE_MOCK_MOTION
    ESP_LOGI("main", "MOCK_INPUT=sine_roll amplitude=2deg period=4s speed=0; scripted input only");
#else
    ESP_LOGI("main", "MOCK_INPUT=fixed_roll roll=0.2deg speed=0");
#endif
#endif
    const esp_err_t result = bike_app_start();
    if (result != ESP_OK) {
        ESP_LOGE("main", "Framework startup failed: %s; motor remains disabled",
                 esp_err_to_name(result));
    } else {
        ESP_LOGI("main", "Bike tasks started; waiting for laptop Wi-Fi/UDP connection");
    }
#endif
}
