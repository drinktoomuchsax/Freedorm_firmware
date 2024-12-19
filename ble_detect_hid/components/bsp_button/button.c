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
#include "_freedorm_main.h"

// 宏定义
#define BUTTON_TAG "BUTTON"

// 定义全局变量
uint32_t led_state_mask = 0; // 在这里初始化，位图，记录每个 GPIO 的当前状态

// 定义局部变量

static ws2812b_effect_t current_effect_btn = DEFAULT_EFFECT;
static ws2812b_queue_data_t effect_data = {
    .effect_args = {
        .color_rgb = {255, 0, 0}, // 红色
        .direction = LED_DIRECTION_TOP_DOWN,
        .loop_mode = LED_MODE_LOOP,
    },
    .current_effect = DEFAULT_EFFECT,
};

// 定义函数

void flip_gpio(int gpio_num)
{
    if (gpio_num < GPIO_NUM_0 || gpio_num >= GPIO_NUM_MAX)
    {
        ESP_LOGE(BUTTON_TAG, "Invalid GPIO number");
        return;
    }

    esp_err_t res = ESP_OK;
    // 检查当前状态并翻转
    if (led_state_mask & (1 << gpio_num))
    {
        res = gpio_set_level(gpio_num, 0);
        led_state_mask &= ~(1 << gpio_num); // 清除该位
        if (res != ESP_OK)
        {
            ESP_LOGE(BUTTON_TAG, "Failed to set GPIO level");
        }
        else
        {
            ESP_LOGI(BUTTON_TAG, "GPIO %d set to 0", gpio_num);
        }
    }
    else
    {
        res = gpio_set_level(gpio_num, 1);
        led_state_mask |= (1 << gpio_num); // 设置该位
        if (res != ESP_OK)
        {
            ESP_LOGE(BUTTON_TAG, "Failed to set GPIO level");
        }
        else
        {
            ESP_LOGI(BUTTON_TAG, "GPIO %d set to 1", gpio_num);
        }
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

void BTN1_PRESS_REPEAT_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Press repeat detected\n");
}

void BTN1_SINGLE_CLICK_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Single click detected\n");
    flip_gpio(OUTPUT_LED_D4); // 翻转 LED 状态
    flip_gpio(CTL_LOCK);

    // 切换到下一个效果
    ESP_LOGI(BUTTON_TAG, "WS2812B_NUMBER_OF_EFFECTS: %d", WS2812B_NUMBER_OF_EFFECTS);
    if (current_effect_btn == DEFAULT_EFFECT)
    {
        current_effect_btn = LED_EFFECT_RAINBOW_BREATHING_ALL;
    }
    else if (current_effect_btn == LED_EFFECT_RAINBOW_BREATHING_ALL)
    {
        current_effect_btn = LED_EFFECT_BLE_TRY_PAIRING;
    }
    else if (current_effect_btn == LED_EFFECT_BLE_TRY_PAIRING)
    {
        current_effect_btn = LED_EFFECT_RAINBOW_BREATHING_ALL;
    }
    else
    {
        current_effect_btn = DEFAULT_EFFECT;
    }

    // current_effect_btn = (current_effect_btn + 1) % WS2812B_NUMBER_OF_EFFECTS;

    ESP_LOGI(BUTTON_TAG, "Switch to effect: %d", current_effect_btn);
    effect_data = (ws2812b_queue_data_t){
        .current_effect = current_effect_btn,
        .effect_args = {
            .color_rgb = WHITE_RGB, // 根据需要设置颜色
            .direction = LED_DIRECTION_TOP_DOWN,
            .loop_mode = LED_MODE_LOOP,
        },
    };
    ESP_LOGI(BUTTON_TAG, "Sending effect data: current_effect=%d", effect_data.current_effect);

    // 通知LED线程切换效果

    xTaskNotify(xLedTaskHandle, 0, eIncrement); // 通知 LED 任务

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
    // flip_gpio(OUTPUT_LED_D5); // 翻转 LED 状态
    // flip_gpio(CTL_D0);
}

void BTN1_LONG_PRESS_HOLD_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Long press hold detected");
}