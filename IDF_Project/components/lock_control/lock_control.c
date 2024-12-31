#include "lock_control.h"
#include "button.h"
#include "ws2812b_led.h"

#define LOCK_CONTROL_TAG "LOCK_CONTROL"

// 门锁状态
static lock_status_t current_lock_state = LOCK_STATE_NORMAL;
static QueueHandle_t lock_command_queue; // 队列句柄
static bool is_switch_lock = false;

void lock_control_init(void)
{
    // 创建队列，长度为 5，每条消息 4 字节
    lock_command_queue = xQueueCreate(5, sizeof(lock_command_t));
    if (lock_command_queue == NULL)
    {
        ESP_LOGE(LOCK_CONTROL_TAG, "Failed to create command queue");
        return;
    }

    // 创建任务
    xTaskCreate(lock_control_task, "lock_control_task", 2048, NULL, 10, NULL);
}

// 任务：处理门锁命令
void lock_control_task(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // lock_command_t received_command;

    // while (1)
    // {

    //     if (xQueueReceive(lock_command_queue, &received_command, portMAX_DELAY))
    //     {
    //         ESP_LOGI(LOCK_CONTROL_TAG, "Received command: %d", received_command);

    //         switch (received_command)
    //         {
    //         case LOCK_CMD_SINGLE_OPEN:
    //             single_click_toogle();
    //             break;

    //         case LOCK_CMO_NORMAL:
    //             lock_set_normal();
    //             break;

    //         case LOCK_CMD_ALWAYS_OPEN:
    //             double_click_always_open();
    //             // 不会延时自动关闭，保持常开模式
    //             break;

    //         case LOCK_CMD_LOCK:
    //             lock_set_lock();
    //             break;

    //         default:
    //             break;
    //         }
    //     }
    //     // 等待接收命令
}

// 发送命令到队列
void lock_send_command(lock_command_t cmd)
{
    if (lock_command_queue != NULL)
    {
        ESP_LOGI(LOCK_CONTROL_TAG, "Sending command to queue: %d", cmd);
        xQueueSend(lock_command_queue, &cmd, portMAX_DELAY);
    }
    is_switch_lock = true;
}

void lock_set_lock(void)
{
    ESP_LOGI(LOCK_CONTROL_TAG, "Locking door, lockstate: %d", current_lock_state);
    gpio_set_level(CTL_D0, 1);
    gpio_set_level(OUTPUT_LED_D5, 1);
    gpio_set_level(OUTPUT_LED_D4, 1);
    current_lock_state = LOCK_STATE_LOCKED;
}

void lock_set_open(void)
{
    ESP_LOGI(LOCK_CONTROL_TAG, "Opening door, lockstate: %d", LOCK_STATE_OPEN);
    gpio_set_level(CTL_LOCK, 1);
    gpio_set_level(OUTPUT_LED_D4, 0);
    gpio_set_level(OUTPUT_LED_D5, 1);
    current_lock_state = LOCK_STATE_OPEN;
}

void lock_set_normal(void)
{
    ESP_LOGI(LOCK_CONTROL_TAG, "Setting door to normal, lockstate: %d", LOCK_STATE_NORMAL);
    gpio_set_level(CTL_LOCK, 0);
    gpio_set_level(CTL_D0, 0);
    gpio_set_level(OUTPUT_LED_D4, 0);
    gpio_set_level(OUTPUT_LED_D5, 0);
    current_lock_state = LOCK_STATE_NORMAL;
}

lock_status_t lock_get_status(void)
{
    return current_lock_state;
}
void single_click_toogle(void)
{
    if (is_switch_lock)
    {
        ESP_LOGI(LOCK_CONTROL_TAG, "Another operation is already in progress, exiting...");
        return;
    }

    if (lock_get_status() == LOCK_STATE_NORMAL)
    {
        lock_set_open();
        ws2812b_switch_effect(LED_EFFECT_SINGLE_OPEN_DOOR);

        vTaskDelay(pdMS_TO_TICKS(5 * 60 * 1000)); // 5 秒后自动关闭
        lock_set_normal();
        ws2812b_switch_effect(LED_EFFECT_DEFAULT_STATE);
    }
    else if (lock_get_status() == LOCK_STATE_OPEN)
    {
        lock_set_normal();
        ws2812b_switch_effect(LED_EFFECT_DEFAULT_STATE);
    }
    else
    {
        lock_set_normal();
    }

    // 操作完成，清除标志位
    is_switch_lock = false;
}

void double_click_always_open(void)
{
    if (is_switch_lock)
    {
        ESP_LOGI(LOCK_CONTROL_TAG, "Another operation is already in progress, interrupting...");
        is_switch_lock = false; // 中断当前操作
    }

    if (lock_get_status() == LOCK_STATE_NORMAL)
    {
        lock_set_open();
        ws2812b_switch_effect(LED_EFFECT_ALWAYS_OPEN_MODE);

        // 常开模式，无需延时
    }
    else if (lock_get_status() == LOCK_STATE_OPEN)
    {
        lock_set_normal();
        ws2812b_switch_effect(LED_EFFECT_DEFAULT_STATE);
    }
    else
    {
        lock_set_normal();
    }

    // 操作完成，清除标志位
    is_switch_lock = false;
}