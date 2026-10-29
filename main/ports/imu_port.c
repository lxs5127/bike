#include "imu_port.h"

#include <math.h>
#include <stddef.h>
#include "sdkconfig.h"
#include "esp_timer.h"
#include "app_config.h"

#if CONFIG_BIKE_IMU_JY901P
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* JY901P 标准寄存器协议：地址为 7 位，每个寄存器为 16 位、小端存储。
 * 参考：https://github.com/WITMOTION/WitIIC_JY901/tree/main/ESP32/main */
#define JY901P_REG_RATE       0x03U
#define JY901P_REG_BANDWIDTH  0x1fU
#define JY901P_REG_GX         0x37U
#define JY901P_REG_ROLL       0x3dU
#define JY901P_REG_YAW        0x3fU
#define JY901P_REG_KEY        0x69U
#define JY901P_KEY_UNLOCK     0xb588U
#define JY901P_RATE_200HZ     0x0bU
/* 当前 FreeRTOS tick 为 10 ms，超时须至少一个 tick，不能直接填 5 ms。 */
#define JY901P_TIMEOUT_MS     20
#define JY901P_PI             3.14159265358979323846f

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_device;
static bool s_ready;

static uint16_t read_u16_le(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
}

static int32_t read_i16_le(const uint8_t *bytes)
{
    uint16_t value = read_u16_le(bytes);
    return value < 0x8000U ? (int32_t)value : (int32_t)value - 65536;
}

static esp_err_t read_registers(uint8_t reg, uint8_t *bytes, size_t size)
{
    /* 写入寄存器地址后重复 START，连续读取低字节、高字节。 */
    return i2c_master_transmit_receive(s_device, &reg, 1, bytes, size, JY901P_TIMEOUT_MS);
}

static esp_err_t write_register(uint8_t reg, uint16_t value)
{
    const uint8_t bytes[] = {reg, (uint8_t)value, (uint8_t)(value >> 8)};
    return i2c_master_transmit(s_device, bytes, sizeof(bytes), JY901P_TIMEOUT_MS);
}

static esp_err_t configure_register(uint8_t reg, uint16_t value)
{
    /* 图片中的解锁值 0xB588 在线上发送为 0x88、0xB5。每项配置单独解锁。 */
    esp_err_t result = write_register(JY901P_REG_KEY, JY901P_KEY_UNLOCK);
    if (result != ESP_OK) return result;
    vTaskDelay(pdMS_TO_TICKS(20));
    result = write_register(reg, value);
    if (result != ESP_OK) return result;
    vTaskDelay(pdMS_TO_TICKS(20));
    uint8_t bytes[2];
    result = read_registers(reg, bytes, sizeof(bytes));
    if (result != ESP_OK) return result;
    if (read_u16_le(bytes) != value) {
        ESP_LOGE("imu", "JY901P register 0x%02x readback mismatch", reg);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static void release_i2c(void)
{
    s_ready = false;
    if (s_device) {
        i2c_master_bus_rm_device(s_device);
        s_device = NULL;
    }
    if (s_bus) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
}

static esp_err_t jy901p_init(void)
{
    if (s_ready) return ESP_OK;
    if (CONFIG_BIKE_IMU_SDA_GPIO == CONFIG_BIKE_IMU_SCL_GPIO ||
        !GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_BIKE_IMU_SDA_GPIO) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_BIKE_IMU_SCL_GPIO)) return ESP_ERR_INVALID_ARG;
    const i2c_master_bus_config_t bus = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_BIKE_IMU_SDA_GPIO,
        .scl_io_num = CONFIG_BIKE_IMU_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t result = i2c_new_master_bus(&bus, &s_bus);
    if (result != ESP_OK) return result;
    const i2c_device_config_t device = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        /* ESP-IDF 自动处理读写位；这里传 0x50，而不是 0xA0/0xA1。 */
        .device_address = CONFIG_BIKE_IMU_I2C_ADDRESS,
        .scl_speed_hz = CONFIG_BIKE_IMU_I2C_HZ,
    };
    result = i2c_master_bus_add_device(s_bus, &device, &s_device);
    if (result != ESP_OK) goto fail;
    /* 等待传感器上电稳定；此时尚未创建采样和控制任务。 */
    vTaskDelay(pdMS_TO_TICKS(500));
    result = i2c_master_probe(s_bus, CONFIG_BIKE_IMU_I2C_ADDRESS, JY901P_TIMEOUT_MS);
    if (result != ESP_OK) goto fail;
    result = configure_register(JY901P_REG_RATE, JY901P_RATE_200HZ);
    if (result != ESP_OK) goto fail;
    result = configure_register(JY901P_REG_BANDWIDTH, CONFIG_BIKE_IMU_BANDWIDTH);
    if (result != ESP_OK) goto fail;
    /* 只设置本次运行参数，不反复写 Flash，不自动校准或执行 Z 轴归零。 */
    vTaskDelay(pdMS_TO_TICKS(20));
    s_ready = true;
    ESP_LOGI("imu", "JY901P ready: I2C=%dHz rate=200Hz bandwidth_code=%d; X forward, Y left, Z up",
             CONFIG_BIKE_IMU_I2C_HZ, CONFIG_BIKE_IMU_BANDWIDTH);
    return ESP_OK;

fail:
    ESP_LOGE("imu", "JY901P init failed: %s (SDA=%d SCL=%d addr=0x%02x)",
             esp_err_to_name(result), CONFIG_BIKE_IMU_SDA_GPIO,
             CONFIG_BIKE_IMU_SCL_GPIO, CONFIG_BIKE_IMU_I2C_ADDRESS);
    release_i2c();
    return result;
}
#endif

#if !CONFIG_BIKE_IMU_JY901P && CONFIG_BIKE_MOCK_HARDWARE && CONFIG_BIKE_MOCK_MOTION
static int64_t s_start_us;
#endif

esp_err_t imu_port_init(void)
{
#if CONFIG_BIKE_IMU_JY901P
    return jy901p_init();
#elif CONFIG_BIKE_MOCK_HARDWARE
#if CONFIG_BIKE_MOCK_MOTION
    s_start_us = esp_timer_get_time();
#endif
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t imu_port_read(imu_sample_t *sample)
{
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
#if CONFIG_BIKE_IMU_JY901P
    /* 失败时绝不把旧数据标记为有效，也不沿用上一次读取的时间戳。 */
    *sample = (imu_sample_t) {0};
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    /* 一次连续读取 GX/GY/GZ、HX/HY/HZ、Roll/Pitch/Yaw，共 9 个寄存器。
     * 控制只用 body-X 的 GX 与 Roll；其余轴保留在本次原始缓冲区中。 */
    uint8_t bytes[(JY901P_REG_YAW - JY901P_REG_GX + 1U) * 2U];
    const int64_t started_us = esp_timer_get_time();
    esp_err_t result = read_registers(JY901P_REG_GX, bytes, sizeof(bytes));
    if (result != ESP_OK) return result;
    if (esp_timer_get_time() - started_us > BIKE_MAX_SENSOR_AGE_US) return ESP_ERR_TIMEOUT;
    const int32_t gx = read_i16_le(bytes);
    const int32_t roll = read_i16_le(bytes + (JY901P_REG_ROLL - JY901P_REG_GX) * 2U);
    *sample = (imu_sample_t) {
        /* 图片公式：角度 raw/32768*180°，角速度 raw/32768*2000°/s。
         * 发布前换成 rad、rad/s；模块须按车体 X前/Y左/Z上安装。 */
        .roll_rad = (float)roll / 32768.0f * JY901P_PI,
        .roll_rate_rad_s = (float)gx / 32768.0f * (2000.0f * JY901P_PI / 180.0f),
        /* IIC 数据块没有逐帧采样时间戳。用读取开始时间减一个 200 Hz 周期
         * 估计采样时刻，不把完成时刻当作采样时刻；不含内部融合滤波延迟。 */
        .timestamp_us = started_us - BIKE_SENSOR_PERIOD_US,
        .valid = true,
    };
    return ESP_OK;
#elif CONFIG_BIKE_MOCK_HARDWARE
    const int64_t now_us = esp_timer_get_time();
    float roll = 0.0034906585f; /* fixed fallback: 0.2 degrees */
    float rate = 0.0f;
#if CONFIG_BIKE_MOCK_MOTION
    /* Reduce time modulo one period before converting to float, so the
     * waveform retains precision during long runs. Gyro is d(roll)/dt. */
    const float two_pi = 6.28318530718f;
    const float phase = two_pi * (float)((now_us - s_start_us) % BIKE_MOCK_ROLL_PERIOD_US)
                        / (float)BIKE_MOCK_ROLL_PERIOD_US;
    roll = BIKE_MOCK_ROLL_AMPLITUDE_RAD * sinf(phase);
    rate = BIKE_MOCK_ROLL_AMPLITUDE_RAD * two_pi * 1000000.0f
           / (float)BIKE_MOCK_ROLL_PERIOD_US * cosf(phase);
#endif
    *sample = (imu_sample_t) {
        .roll_rad = roll,
        .roll_rate_rad_s = rate,
        .timestamp_us = now_us,
        .valid = true,
    };
    return ESP_OK;
#else
    *sample = (imu_sample_t) {0};
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
