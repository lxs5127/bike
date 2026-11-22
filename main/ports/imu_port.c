#include "imu_port.h"

#include <math.h>
#include <stddef.h>

#include "app_config.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#if CONFIG_BIKE_IMU_JY901P
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* JY901P 使用 7 位 I2C 地址，寄存器值为 16 位小端。寄存器地址按 16 位值递增。
 * 参考：https://github.com/WITMOTION/WitIIC_JY901/tree/main/ESP32/main */
#define JY901P_REG_RATE       0x03U
#define JY901P_REG_BANDWIDTH  0x1fU
#define JY901P_REG_AX         0x34U
#define JY901P_REG_GX         0x37U
#define JY901P_REG_ROLL       0x3dU
#define JY901P_REG_YAW        0x3fU
#define JY901P_REG_KEY        0x69U
#define JY901P_KEY_UNLOCK     0xb588U
#define JY901P_RATE_200HZ     0x0bU
/* I2C 超时至少覆盖一个 FreeRTOS tick，兼容使用 100 Hz tick 的旧配置。 */
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
    /* 从二进制补码恢复负值，避免超范围的无符号到有符号转换。 */
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
    /* 每项寄存器配置都要单独解锁，再回读确认传感器接受了配置。 */
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
    /* 创建顺序为总线、设备。初始化中途出错时按反序释放已有资源。 */
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
        !GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_BIKE_IMU_SCL_GPIO)) {
        return ESP_ERR_INVALID_ARG;
    }
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
    /* 采样率和带宽只写到本次运行的寄存器。 */
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
    /* 先清除输出，读取失败时调用方收到无效样本和空时间戳。 */
    *sample = (imu_sample_t) {0};
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    /* 同一次 IIC 读取加速度、角速度、磁场、姿态，共 12 个寄存器。
     * 本地控制仍使用 GX/Roll；完整三轴数据用于 Wi-Fi 3D 显示。 */
    uint8_t bytes[(JY901P_REG_YAW - JY901P_REG_AX + 1U) * 2U];
    const int64_t started_us = esp_timer_get_time();
    esp_err_t result = read_registers(JY901P_REG_AX, bytes, sizeof(bytes));
    if (result != ESP_OK) return result;

    /* I2C 操作成功也可能已耗时过久，控制端需要能及时使用的数据。 */
    if (esp_timer_get_time() - started_us > BIKE_MAX_SENSOR_AGE_US) return ESP_ERR_TIMEOUT;

    const int32_t gx = read_i16_le(bytes + (JY901P_REG_GX - JY901P_REG_AX) * 2U);
    const int32_t roll = read_i16_le(bytes + (JY901P_REG_ROLL - JY901P_REG_AX) * 2U);
    *sample = (imu_sample_t) {
        /* 原始角度和角速度的满量程分别为 180° 和 2000°/s。控制端使用弧度。 */
        .roll_rad = (float)roll / 32768.0f * JY901P_PI,
        .roll_rate_rad_s = (float)gx / 32768.0f * (2000.0f * JY901P_PI / 180.0f),
        /* 用读取开始时刻减去一个 200 Hz 输出周期，保守估计样本时间。
         * 这个估计只覆盖寄存器更新间隔，传感器内部融合延迟另计。 */
        .timestamp_us = started_us - BIKE_SENSOR_PERIOD_US,
        .valid = true,
    };
    /* 可视化沿用传感器三轴量程，单位依次为 g、°/s、°。 */
    for (unsigned i = 0; i < 3; ++i) {
        sample->acceleration_g[i] = (float)read_i16_le(bytes + i * 2U) / 32768.f * 16.f;
        sample->gyro_deg_s[i] = (float)read_i16_le(bytes + (3U + i) * 2U) / 32768.f * 2000.f;
        sample->angles_deg[i] = (float)read_i16_le(bytes + (9U + i) * 2U) / 32768.f * 180.f;
    }
    return ESP_OK;
#elif CONFIG_BIKE_MOCK_HARDWARE
    const int64_t now_us = esp_timer_get_time();
    float roll = 0.0034906585f; /* 模拟回退值：0.2°。 */
    float rate = 0.0f;
#if CONFIG_BIKE_MOCK_MOTION
    /* 先取周期内相位，再转浮点，避免运行时间过长降低精度。
     * 用横滚角的导数生成角速度，使两个模拟量保持一致。 */
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
