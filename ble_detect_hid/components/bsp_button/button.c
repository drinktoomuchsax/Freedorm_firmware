#include "button.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "multi_button.h"
#include "esp_mac.h"

#include "ble_module.h"
#include "ws2812b_led.h"

void flip_gpio(int gpio_num)
{
    if (gpio_num < GPIO_NUM_0 || gpio_num >= GPIO_NUM_MAX)
    {
        ESP_LOGE("LED_FLIP", "Invalid GPIO number");
        return;
    }

    // 检查当前状态并翻转
    if (led_state_mask & (1 << gpio_num))
    {
        gpio_set_level(gpio_num, 0);
        led_state_mask &= ~(1 << gpio_num); // 清除该位
    }
    else
    {
        gpio_set_level(gpio_num, 1);
        led_state_mask |= (1 << gpio_num); // 设置该位
    }
}

uint32_t led_state_mask = 0; // 在这里初始化，位图，记录每个 GPIO 的当前状态

uint8_t read_button_GPIO(uint8_t button_id)
{
    return gpio_get_level(PAIRING_BUTTON_GPIO);
}

void button_task(void *arg)
{
    while (1)
    {
        button_ticks();                // 按键状态检测
        vTaskDelay(pdMS_TO_TICKS(10)); // 每 5 毫秒调用一次
    }
}

static ws2812b_effect_t current_effect = DEFAULT_EFFECT;

void BTN1_SINGLE_CLICK_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Single click detected");
    flip_gpio(OUTPUT_LED_D4); // 翻转 LED 状态
    flip_gpio(CTL_LOCK);

    // 定义并初始化要发送的数据

    // 切换到下一个效果
    current_effect = (current_effect + 1) % sizeof(ws2812b_effect_t); // 假设有5种效果，循环切换

    ws2812b_queue_data_t effect_data = {
        .effect_args = {
            .color_rgb = {255, 0, 0}, // 红色
            .direction = LED_DIRECTION_TOP_DOWN,
            .loop_mode = LED_MODE_LOOP,
        },
        .current_effect = LED_EFFECT_BREATHING_WAVE, // 呼吸波效果
    };

    // 发送数据到队列
    if (xQueueSend(effect_queue, &effect_data, portMAX_DELAY) == pdPASS)
    {
        ESP_LOGI(BUTTON_TAG, "Effect data sent to queue.");
    }
    else
    {
        ESP_LOGI(BUTTON_TAG, "Failed to send effect data to queue.");
    }
}

void BTN1_DOUBLE_CLICK_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Double click detected");
    for (int i = 0; i < 10; i++)
    {
        flip_gpio(OUTPUT_LED_D4); // 翻转 LED 状态
        vTaskDelay(100 / portTICK_PERIOD_MS);
        flip_gpio(OUTPUT_LED_D5); // 翻转 LED 状态
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    xSemaphoreGive(pairing_semaphore);
}

void BTN1_LONG_PRESS_START_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Long press start detected");
    flip_gpio(OUTPUT_LED_D5); // 翻转 LED 状态
    // flip_gpio(CTL_D0);
}