#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 保存被通知任务的句柄 */
static TaskHandle_t s_event_task_handle = NULL;

/* 等待并处理事件 */
static void event_task(void *arg)
{
    while (1) {
        printf("处理任务：等待事件...\n");

        uint32_t count = ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        printf("处理任务：收到 %" PRIu32 " 次通知\n", count);

        /* 在这里处理事件 */
        printf("处理任务：执行一次事件处理\n");
    }
}

/* 模拟按键或传感器事件 */
static void event_generator_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        printf("模拟任务：产生事件\n");

        xTaskNotifyGive(s_event_task_handle);
    }
}

void app_main(void)
{
    BaseType_t result;

    /* 先创建接收任务，并保存它的句柄 */
    result = xTaskCreate(
        event_task,
        "event_handler",
        4096,
        NULL,
        5,
        &s_event_task_handle
    );
    assert(result == pdPASS);

    /* 再创建通知发送任务 */
    result = xTaskCreate(
        event_generator_task,
        "event_generator",
        4096,
        NULL,
        5,
        NULL
    );
    assert(result == pdPASS);
}