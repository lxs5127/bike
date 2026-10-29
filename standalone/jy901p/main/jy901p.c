/*
 * JY901P 独立 IIC 示例，适用于 ESP32 / ESP-IDF 5.4。
 * 所有引脚、参数、初始化、读取、换算和串口打印均在本文件中。
 *
 * 接线：SDA -> GPIO21，SCL -> GPIO22，VCC -> 3.3V，GND -> GND。
 * SDA/SCL 各需约 4.7kΩ 上拉到 3.3V（模块已有上拉时无需重复添加）。
 *
 * 在 standalone/jy901p 目录运行 idf.py build flash monitor。
 * 本文件包含独立 app_main，作为该示例工程的唯一应用入口编译。
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ==================== 用户参数 ==================== */
#define JY901P_SDA_GPIO         GPIO_NUM_21
#define JY901P_SCL_GPIO         GPIO_NUM_22
#define JY901P_I2C_PORT         I2C_NUM_0
#define JY901P_I2C_ADDRESS      0x50U      /* 7 位地址，无需左移 */
#define JY901P_I2C_CLOCK_HZ     100000U
#define JY901P_I2C_TIMEOUT_MS   20         /* 有限等待，至少一个 RTOS tick */
#define JY901P_PRINT_PERIOD_MS  10         /* 每 10 ms 读取并打印一组，即 100 Hz */
#define JY901P_OUTPUT_RATE      0x0009U    /* 模块输出率：100 Hz */
#define JY901P_BANDWIDTH        0x0002U    /* SDK 标称带宽：94 Hz */

/* 标准协议寄存器：每个寄存器占 2 字节，低字节在前。
 * 官方参考：https://github.com/WITMOTION/WitIIC_JY901/tree/main/ESP32/main */
#define JY901P_REG_RATE         0x03U
#define JY901P_REG_BANDWIDTH    0x1fU
#define JY901P_REG_AX           0x34U
#define JY901P_REG_GX           0x37U
#define JY901P_REG_HX           0x3aU
#define JY901P_REG_ROLL         0x3dU
#define JY901P_REG_YAW          0x3fU
#define JY901P_REG_KEY          0x69U
#define JY901P_UNLOCK_KEY       0xb588U

typedef struct {
    float acceleration_g[3];  /* X/Y/Z 加速度，单位 g */
    float gyro_deg_s[3];      /* X/Y/Z 角速度，单位 °/s */
    float angle_deg[3];       /* Roll/Pitch/Yaw，单位 ° */
    int32_t magnetic_raw[3];  /* X/Y/Z 磁场原始计数 */
    bool valid;              /* 完整读数成功后才置为 true */
} jy901p_sample_t;

static const char *TAG = "JY901P";
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_device;
static bool s_initialized;

/* 显式恢复 16 位有符号数，正确处理负角度和负角速度。 */
static int32_t jy901p_signed_le(const uint8_t *bytes)
{
    const uint16_t raw = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    return raw < 0x8000U ? (int32_t)raw : (int32_t)raw - 65536;
}

static esp_err_t jy901p_read_registers(uint8_t reg, uint8_t *bytes, size_t count)
{
    /* START + 写地址 + 寄存器地址 + 重复 START + 读地址 + 数据 + STOP。 */
    return i2c_master_transmit_receive(s_device, &reg, 1, bytes, count,
                                       JY901P_I2C_TIMEOUT_MS);
}

static esp_err_t jy901p_write_register(uint8_t reg, uint16_t value)
{
    /* 例如解锁寄存器的发送内容为：0x69、0x88、0xB5。 */
    const uint8_t bytes[] = {reg, (uint8_t)value, (uint8_t)(value >> 8)};
    return i2c_master_transmit(s_device, bytes, sizeof(bytes), JY901P_I2C_TIMEOUT_MS);
}

/* 每次配置先解锁，写入后再回读，确认模块接受了参数。 */
static esp_err_t jy901p_configure(uint8_t reg, uint16_t value)
{
    esp_err_t result = jy901p_write_register(JY901P_REG_KEY, JY901P_UNLOCK_KEY);
    if (result != ESP_OK) return result;
    vTaskDelay(pdMS_TO_TICKS(20));
    result = jy901p_write_register(reg, value);
    if (result != ESP_OK) return result;
    vTaskDelay(pdMS_TO_TICKS(20));
    uint8_t bytes[2];
    result = jy901p_read_registers(reg, bytes, sizeof(bytes));
    if (result != ESP_OK) return result;
    const uint16_t actual = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    if (actual != value) {
        ESP_LOGE(TAG, "Register 0x%02x: expected=0x%04x actual=0x%04x",
                 (unsigned)reg, (unsigned)value, (unsigned)actual);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static void jy901p_deinit(void)
{
    s_initialized = false;
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
    if (s_initialized) return ESP_OK;
    if (JY901P_SDA_GPIO == JY901P_SCL_GPIO ||
        !GPIO_IS_VALID_OUTPUT_GPIO(JY901P_SDA_GPIO) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(JY901P_SCL_GPIO)) return ESP_ERR_INVALID_ARG;

    const i2c_master_bus_config_t bus = {
        .i2c_port = JY901P_I2C_PORT,
        .sda_io_num = JY901P_SDA_GPIO,
        .scl_io_num = JY901P_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t result = i2c_new_master_bus(&bus, &s_bus);
    if (result != ESP_OK) return result;
    const i2c_device_config_t device = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = JY901P_I2C_ADDRESS,
        .scl_speed_hz = JY901P_I2C_CLOCK_HZ,
    };
    result = i2c_master_bus_add_device(s_bus, &device, &s_device);
    if (result != ESP_OK) goto fail;

    /* 等待模块上电稳定，再探测地址并配置输出率、滤波带宽。 */
    vTaskDelay(pdMS_TO_TICKS(500));
    result = i2c_master_probe(s_bus, JY901P_I2C_ADDRESS, JY901P_I2C_TIMEOUT_MS);
    if (result != ESP_OK) goto fail;
    result = jy901p_configure(JY901P_REG_RATE, JY901P_OUTPUT_RATE);
    if (result != ESP_OK) goto fail;
    result = jy901p_configure(JY901P_REG_BANDWIDTH, JY901P_BANDWIDTH);
    if (result != ESP_OK) goto fail;
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 参数在每次初始化时重设，不自动保存 Flash、校准或执行 Z 轴归零。 */
    s_initialized = true;
    return ESP_OK;

fail:
    jy901p_deinit();
    return result;
}

static esp_err_t jy901p_read(jy901p_sample_t *sample)
{
    if (!sample) return ESP_ERR_INVALID_ARG;
    *sample = (jy901p_sample_t) {0};
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    /* AX 到 Yaw 连续 12 个寄存器，共 24 字节。读取失败不发布有效数据。
     * 这是模块当前寄存器读数，不提供内部采样时间或跨寄存器原子快照保证。 */
    uint8_t bytes[(JY901P_REG_YAW - JY901P_REG_AX + 1U) * 2U];
    esp_err_t result = jy901p_read_registers(JY901P_REG_AX, bytes, sizeof(bytes));
    if (result != ESP_OK) return result;

    for (unsigned i = 0; i < 3; ++i) {
        const int32_t acc = jy901p_signed_le(bytes + i * 2U);
        const int32_t gyro = jy901p_signed_le(bytes + (JY901P_REG_GX - JY901P_REG_AX + i) * 2U);
        const int32_t mag = jy901p_signed_le(bytes + (JY901P_REG_HX - JY901P_REG_AX + i) * 2U);
        const int32_t angle = jy901p_signed_le(bytes + (JY901P_REG_ROLL - JY901P_REG_AX + i) * 2U);
        /* 按图片的标准协议换算：±16g、±2000°/s、±180°。 */
        sample->acceleration_g[i] = (float)acc / 32768.0f * 16.0f;
        sample->gyro_deg_s[i] = (float)gyro / 32768.0f * 2000.0f;
        sample->angle_deg[i] = (float)angle / 32768.0f * 180.0f;
        sample->magnetic_raw[i] = mag;
    }
    sample->valid = true;
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t result = jy901p_init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Init failed: %s; check SDA=21 SCL=22 addr=0x50 and power",
                 esp_err_to_name(result));
        return;
    }
    ESP_LOGI(TAG, "Ready: SDA=%d SCL=%d addr=0x%02x IIC=%uHz; output=100Hz print=100Hz console=%dbaud",
             JY901P_SDA_GPIO, JY901P_SCL_GPIO, JY901P_I2C_ADDRESS,
             (unsigned)JY901P_I2C_CLOCK_HZ, CONFIG_ESP_CONSOLE_UART_BAUDRATE);

    /* 按绝对 tick 周期调度，避免将 IIC 读取和打印耗时累加进 10 ms 周期。
     * 串口配置为 460800 baud，保证完整文本输出有足够带宽。 */
    TickType_t previous_wake = xTaskGetTickCount();
    while (1) {
        jy901p_sample_t sample;
        result = jy901p_read(&sample);
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "acc[g] X=%.3f Y=%.3f Z=%.3f | gyro[deg/s] X=%.3f Y=%.3f Z=%.3f | angle[deg] Roll=%.3f Pitch=%.3f Yaw=%.3f",
                     (double)sample.acceleration_g[0], (double)sample.acceleration_g[1], (double)sample.acceleration_g[2],
                     (double)sample.gyro_deg_s[0], (double)sample.gyro_deg_s[1], (double)sample.gyro_deg_s[2],
                     (double)sample.angle_deg[0], (double)sample.angle_deg[1], (double)sample.angle_deg[2]);
        } else {
            ESP_LOGW(TAG, "Read failed: %s", esp_err_to_name(result));
        }
        xTaskDelayUntil(&previous_wake, pdMS_TO_TICKS(JY901P_PRINT_PERIOD_MS));
    }
}
