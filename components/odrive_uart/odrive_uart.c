#include "odrive_uart.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define LINE_CAPACITY 96  /* ASCII 行缓冲容量，包含结尾的空字符。 */
#define TWO_PI 6.2831853071795864769f

static SemaphoreHandle_t s_mutex;
static bool s_initialized;
static bool s_rx_fault;  /* 接收失步时冻结查询，保留写入路径。 */
static bool s_tx_fault;  /* 命令只写了一部分时冻结后续发送。 */

/* 把毫秒超时向上换算为 tick，保留低 tick 频率下的等待时间。 */
static TickType_t timeout_ticks(void)
{
    return (ODRIVE_UART_TIMEOUT_MS + portTICK_PERIOD_MS - 1U) / portTICK_PERIOD_MS;
}

static esp_err_t take_bus(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    /* 查询必须独占从发出请求到取完响应的整个过程，写入也使用同一把锁。 */
    return xSemaphoreTake(s_mutex, timeout_ticks()) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* 使用有时限的 FIFO 发送，避免串口等待长期占用控制任务。
 * ESP_OK 表示最后一个字节已经发出。需要设备回包的接口会另行接收。 */
static esp_err_t send_locked(const char *format, ...)
{
    if (s_tx_fault) return ESP_ERR_INVALID_STATE;
    char bytes[LINE_CAPACITY];
    va_list args;
    va_start(args, format);
    int count = vsnprintf(bytes, sizeof(bytes), format, args);
    va_end(args);
    if (count <= 0 || count >= (int)sizeof(bytes)) return ESP_ERR_INVALID_SIZE;

    esp_err_t result = uart_wait_tx_done(ODRIVE_UART_NUM, timeout_ticks());
    if (result != ESP_OK) return result;
    if (uart_tx_chars(ODRIVE_UART_NUM, bytes, (uint32_t)count) != count) {
        /* ODrive 可能已收下半条命令。锁存故障，避免下一条命令与它拼接。 */
        s_tx_fault = true;
        return ESP_FAIL;
    }
    return uart_wait_tx_done(ODRIVE_UART_NUM, timeout_ticks());
}

static esp_err_t stop_locked(unsigned motor)
{
    /* 请求退出闭环后也尝试清零目标，避免下次进入闭环时沿用上次的指令。 */
    esp_err_t idle = send_locked("w axis%u.requested_state 1\n", motor);
    esp_err_t zero = motor == ODRIVE_MOTOR0 ? send_locked("c 0 0\n") : send_locked("v 1 0 0\n");
    return idle != ESP_OK ? idle : zero;
}

esp_err_t odrive_uart_init(void)
{
    if (s_initialized) return ESP_OK;
    if (uart_is_driver_installed(ODRIVE_UART_NUM)) return ESP_ERR_INVALID_STATE;
    if (ODRIVE_UART_TX_GPIO == ODRIVE_UART_RX_GPIO ||
        !GPIO_IS_VALID_OUTPUT_GPIO(ODRIVE_UART_TX_GPIO) ||
        !GPIO_IS_VALID_GPIO(ODRIVE_UART_RX_GPIO)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    /* 接收使用环形缓冲区，发送直接使用硬件 FIFO。 */
    esp_err_t result = uart_driver_install(ODRIVE_UART_NUM, 512, 0, 0, NULL, 0);
    if (result != ESP_OK) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
        return result;
    }
    const uart_config_t config = {
        .baud_rate = ODRIVE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    result = uart_param_config(ODRIVE_UART_NUM, &config);
    if (result != ESP_OK) goto fail;
    result = uart_set_pin(ODRIVE_UART_NUM, ODRIVE_UART_TX_GPIO, ODRIVE_UART_RX_GPIO,
                         UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (result != ESP_OK) goto fail;
    result = uart_flush_input(ODRIVE_UART_NUM);
    if (result != ESP_OK) goto fail;
    s_rx_fault = s_tx_fault = false;

    /* 初始化阶段由调用方独占。先请求两轴退出闭环并清零目标，再开放其他接口。 */
    result = stop_locked(ODRIVE_MOTOR0);
    esp_err_t second = stop_locked(ODRIVE_MOTOR1);
    if (result == ESP_OK) result = second;
    if (result != ESP_OK) goto fail;
    s_initialized = true;
    return ESP_OK;

fail:
    uart_driver_delete(ODRIVE_UART_NUM);
    vSemaphoreDelete(s_mutex);
    s_mutex = NULL;
    return result;
}

esp_err_t odrive_uart_deinit(void)
{
    if (!s_initialized) return ESP_OK;
    /* 即使停机指令发送失败，也尝试释放串口，便于上层重新初始化。 */
    esp_err_t result = odrive_uart_stop_all();
    esp_err_t deleted = uart_driver_delete(ODRIVE_UART_NUM);
    if (deleted != ESP_OK) return deleted;
    s_initialized = false;
    vSemaphoreDelete(s_mutex);
    s_mutex = NULL;
    return result;
}

esp_err_t odrive_uart_set_motor0_torque(float torque_nm)
{
    if (!isfinite(torque_nm)) return ESP_ERR_INVALID_ARG;
    esp_err_t result = take_bus();
    if (result != ESP_OK) return result;
    result = send_locked("c 0 %.9g\n", (double)torque_nm);
    xSemaphoreGive(s_mutex);
    return result;
}

esp_err_t odrive_uart_set_motor1_velocity(float turns_s)
{
    if (!isfinite(turns_s)) return ESP_ERR_INVALID_ARG;
    esp_err_t result = take_bus();
    if (result != ESP_OK) return result;
    result = send_locked("v 1 %.9g 0\n", (double)turns_s);
    xSemaphoreGive(s_mutex);
    return result;
}

esp_err_t odrive_uart_set_motor1_rpm(float rpm)
{
    return odrive_uart_set_motor1_velocity(rpm / 60.0f);
}

esp_err_t odrive_uart_set_motor1_rad_s(float rad_s)
{
    return odrive_uart_set_motor1_velocity(rad_s / TWO_PI);
}

esp_err_t odrive_uart_enable(unsigned motor)
{
    if (motor > ODRIVE_MOTOR1) return ESP_ERR_INVALID_ARG;
    esp_err_t result = take_bus();
    if (result != ESP_OK) return result;

    /* 先清零旧目标，再选择直通输入和轴对应的控制模式，最后请求闭环。 */
    result = stop_locked(motor);
    if (result == ESP_OK) result = send_locked("w axis%u.controller.config.input_mode 1\n", motor);
    if (result == ESP_OK) {
        result = send_locked("w axis%u.controller.config.control_mode %u\n", motor,
                             motor == ODRIVE_MOTOR0 ? 1U : 2U);
    }
    if (result == ESP_OK) result = send_locked("w axis%u.requested_state 8\n", motor);
    xSemaphoreGive(s_mutex);
    return result;
}

esp_err_t odrive_uart_stop(unsigned motor)
{
    if (motor > ODRIVE_MOTOR1) return ESP_ERR_INVALID_ARG;
    esp_err_t result = take_bus();
    if (result != ESP_OK) return result;
    result = stop_locked(motor);
    xSemaphoreGive(s_mutex);
    return result;
}

esp_err_t odrive_uart_stop_all(void)
{
    esp_err_t result = take_bus();
    if (result != ESP_OK) return result;
    esp_err_t first = stop_locked(ODRIVE_MOTOR0);
    esp_err_t second = stop_locked(ODRIVE_MOTOR1);
    xSemaphoreGive(s_mutex);
    return first != ESP_OK ? first : second;
}

static esp_err_t line_locked(char *line)
{
    /* 超时约束整行响应，逐字节读取时按剩余时间等待。接受 LF 或 CRLF。 */
    const int64_t deadline = esp_timer_get_time() + ODRIVE_UART_TIMEOUT_MS * 1000LL;
    size_t used = 0;
    while (esp_timer_get_time() < deadline) {
        int64_t remaining = deadline - esp_timer_get_time();
        if (remaining <= 0) break;
        TickType_t ticks = (TickType_t)((remaining + portTICK_PERIOD_MS * 1000LL - 1) /
                                       (portTICK_PERIOD_MS * 1000LL));
        uint8_t byte;
        int count = uart_read_bytes(ODRIVE_UART_NUM, &byte, 1, ticks);
        if (count < 0) return ESP_FAIL;
        if (!count) continue;
        if (byte == '\r') continue;
        if (byte == '\n') {
            if (!used) continue;
            line[used] = '\0';
            return ESP_OK;
        }
        if (byte < 32 || byte > 126) return ESP_ERR_INVALID_RESPONSE;
        if (used >= LINE_CAPACITY - 1) return ESP_ERR_INVALID_SIZE;
        line[used++] = (char)byte;
    }
    return ESP_ERR_TIMEOUT;
}

static const char *skip_space(const char *text)
{
    while (isspace((unsigned char)*text)) ++text;
    return text;
}

static bool parse_float(const char **text, float *value)
{
    char *end;
    errno = 0;
    *value = strtof(*text, &end);
    if (end == *text || errno == ERANGE || !isfinite(*value)) return false;
    *text = end;
    return true;
}

/* ASCII 响应没有请求编号，迟到的响应可能被误认成新值。
 * 因此查询超时或收到坏帧后锁存接收故障，由上层重新初始化。 */
static esp_err_t query_locked(const char *command, char *line)
{
    if (s_rx_fault) return ESP_ERR_INVALID_STATE;
    esp_err_t result = uart_flush_input(ODRIVE_UART_NUM);
    if (result == ESP_OK) result = send_locked("%s", command);
    if (result == ESP_OK) result = line_locked(line);
    if (result != ESP_OK) s_rx_fault = true;
    return result;
}

esp_err_t odrive_uart_read_feedback(unsigned motor, odrive_uart_feedback_t *feedback)
{
    if (!feedback) return ESP_ERR_INVALID_ARG;
    *feedback = (odrive_uart_feedback_t) {0};
    if (motor > ODRIVE_MOTOR1) return ESP_ERR_INVALID_ARG;
    esp_err_t result = take_bus();
    if (result != ESP_OK) return result;
    char command[16], line[LINE_CAPACITY];
    snprintf(command, sizeof(command), "f %u\n", motor);
    result = query_locked(command, line);
    if (result == ESP_OK) {
        const char *cursor = line;
        float position, velocity;
        if (!parse_float(&cursor, &position) || !isspace((unsigned char)*cursor) ||
            !parse_float(&cursor, &velocity) || *skip_space(cursor)) {
            s_rx_fault = true;
            result = ESP_ERR_INVALID_RESPONSE;
        } else {
            *feedback = (odrive_uart_feedback_t) {
                .position_turns = position, .velocity_turns_s = velocity,
                .received_at_us = esp_timer_get_time(), .valid = true,
            };
        }
    }
    xSemaphoreGive(s_mutex);
    return result;
}

esp_err_t odrive_uart_read_velocity(unsigned motor, float *turns_s, int64_t *received_at_us)
{
    if (!turns_s || !received_at_us) return ESP_ERR_INVALID_ARG;
    *turns_s = 0.f;
    *received_at_us = 0;
    if (motor > ODRIVE_MOTOR1) return ESP_ERR_INVALID_ARG;
    esp_err_t result = take_bus();
    if (result != ESP_OK) return result;
    char command[64], line[LINE_CAPACITY];
    snprintf(command, sizeof(command), "r axis%u.encoder.vel_estimate\n", motor);
    result = query_locked(command, line);
    if (result == ESP_OK) {
        const char *cursor = line;
        float value;
        if (!parse_float(&cursor, &value) || *skip_space(cursor)) {
            s_rx_fault = true;
            result = ESP_ERR_INVALID_RESPONSE;
        } else {
            *turns_s = value;
            *received_at_us = esp_timer_get_time();
        }
    }
    xSemaphoreGive(s_mutex);
    return result;
}

static esp_err_t read_uint_property_locked(const char *property, uint32_t *value)
{
    char command[64], line[LINE_CAPACITY];
    int count = snprintf(command, sizeof(command), "r %s\n", property);
    if (count <= 0 || count >= (int)sizeof(command)) return ESP_ERR_INVALID_SIZE;
    esp_err_t result = query_locked(command, line);
    if (result != ESP_OK) return result;
    const char *start = skip_space(line);
    char *end;
    errno = 0;
    unsigned long parsed = strtoul(start, &end, 0);
    if (*start == '-' || end == start || *skip_space(end) || errno == ERANGE || parsed > UINT32_MAX) {
        s_rx_fault = true;
        return ESP_ERR_INVALID_RESPONSE;
    }
    *value = (uint32_t)parsed;
    return ESP_OK;
}

static esp_err_t read_uint_locked(unsigned motor, const char *property, uint32_t *value)
{
    char name[48];
    int count = snprintf(name, sizeof(name), "axis%u.%s", motor, property);
    if (count <= 0 || count >= (int)sizeof(name)) return ESP_ERR_INVALID_SIZE;
    return read_uint_property_locked(name, value);
}

esp_err_t odrive_uart_read_axis_state(unsigned motor, odrive_uart_axis_state_t *state)
{
    if (!state) return ESP_ERR_INVALID_ARG;
    *state = (odrive_uart_axis_state_t) {0};
    if (motor > ODRIVE_MOTOR1) return ESP_ERR_INVALID_ARG;
    esp_err_t result = take_bus();
    if (result != ESP_OK) return result;
    odrive_uart_axis_state_t received = {0};
    result = read_uint_locked(motor, "error", &received.axis_error);
    if (result == ESP_OK) result = read_uint_locked(motor, "current_state", &received.current_state);
    if (result == ESP_OK) {
        received.valid = true;
        *state = received;
    }
    xSemaphoreGive(s_mutex);
    return result;
}

esp_err_t odrive_uart_feed_watchdog(unsigned motor)
{
    if (motor > ODRIVE_MOTOR1) return ESP_ERR_INVALID_ARG;
    esp_err_t result = take_bus();
    if (result != ESP_OK) return result;
    result = send_locked("u %u\n", motor);
    xSemaphoreGive(s_mutex);
    return result;
}

esp_err_t odrive_uart_clear_errors(void)
{
    esp_err_t result = take_bus();
    if (result != ESP_OK) return result;
    result = send_locked("sc\n");
    xSemaphoreGive(s_mutex);
    return result;
}
