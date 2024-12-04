#include "button.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "multi_button.h"
#include "esp_mac.h"

void flip_led(int gpio_num)
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

void BTN1_SINGLE_CLICK_Handler(void *btn)
{
    ESP_LOGI(LED_TAG, "Single click detected");
    flip_led(OUTPUT_LED_D4); // 翻转 LED 状态
    // 进入配对模式
}

void BTN1_DOUBLE_CLICK_Handler(void *btn)
{
    ESP_LOGI(LED_TAG, "Double click detected");
    for (int i = 0; i < 10; i++)
    {
        flip_led(OUTPUT_LED_D4); // 翻转 LED 状态
        vTaskDelay(100 / portTICK_PERIOD_MS);
        flip_led(OUTPUT_LED_D5); // 翻转 LED 状态
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void BTN1_LONG_PRESS_START_Handler(void *btn)
{
    ESP_LOGI(LED_TAG, "Long press hold detected");
    flip_led(OUTPUT_LED_D5); // 翻转 LED 状态

    ESP_LOGI(LED_TAG, "Long press detected, entering pairing mode.");
    // xSemaphoreGive(pairing_semaphore);
}