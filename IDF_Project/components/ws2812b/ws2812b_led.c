/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

#include "ws2812b_led.h"
#include "lock_control.h" //T IME_RECOVER_TEMP_OPEN
#include <math.h>         // 用于指数计算

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM GPIO_NUM_1    // GPIO number for the LED strip

#define WS2812B_LED_NUMBERS 6
#define EXAMPLE_CHASE_SPEED_MS 50
#define ENUM_TO_STRING(name) #name

// 定义全局变量
QueueHandle_t effect_queue = NULL;
TaskHandle_t xLedTaskHandle = NULL; // 初始化为 NULL

// 定义局部变量
static rmt_channel_handle_t led_chan; // 配置 RMT 通道
static rmt_encoder_handle_t led_encoder;
static rmt_transmit_config_t tx_config;

static uint8_t led_strip_pixels[WS2812B_LED_NUMBERS * 3]; // ws2812b LED 灯带的像素数据

static const char *TAG = "WS2812B_LED"; // 定义日志标签

static uint32_t notify_count = 0;

static ws2812b_queue_data_t received_effect_queue_data = {
    // 定义全局参数变量并初始化
    .effect_args = {
        .color_rgb = WHITE_RGB, // 默认Freedorm蓝色
        .direction = LED_DIRECTION_TOP_DOWN,
        .loop_mode = LED_MODE_LOOP,
    },
    .current_effect = DEFAULT_EFFECT,
};

static ws2812b_state_effect_t current_effect_btn = DEFAULT_EFFECT;
static ws2812b_queue_data_t effect_data = {
    .effect_args = {
        .color_rgb = {255, 0, 0}, // 红色
        .direction = LED_DIRECTION_TOP_DOWN,
        .loop_mode = LED_MODE_LOOP,
    },
    .current_effect = DEFAULT_EFFECT,
};

static ws2812b_state_effect_t ws2812b_current_effect = DEFAULT_EFFECT;

static esp_err_t res;

static volatile bool is_switch_effect = false; // 标志位，表示是否需要切换效果

// 函数声明

/**
 * @brief 在这个函数里切换效果，通过修改 current_effect 的值来切换效果
 *
 * @param arg
 */
static void ws2812b_effect_task(void *arg);

/**
 * @brief 为指定索引的 LED 设置颜色
 *
 * @param index
 * @param r
 * @param g
 * @param b
 */
void ws2812b_set_color(int index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 选择一个颜色，设置所有 LED 为这个颜色
 *
 * @param rgb_color
 * @param time_ms 持续时间，单位 ms，在持续时间内可以切换效果
 */
static void ws2812b_led_set_color_all(ws2812b_color_rgb_t rgb_color, uint16_t time_ms);

/**
 * @brief 所有 LED 同时显示相同颜色，并逐渐变换 HSV 色环的 Hue 值
 *
 */
static void ws2812b_led_rainbow_all(void);

/**
 * @brief 彩虹的动态滚动效果，hsv中的h值从0到360，s和v值固定
 *
 */
static void ws2812b_led_rainbow_wave(void);

/**
 * @brief 一排led灯呼吸灯效果，从一端到另一端，类似水波，需要预先设置颜色
 *
 * @param color_rgb 呼吸灯颜色
 * @param direction 呼吸灯方向, 从上到下或者从下到上
 * @param loop_mode 循环模式, 循环或者来回
 */
static void ws2812b_led_breathing_wave(ws2812b_color_rgb_t color_rgb, ws2812b_direction_t direction, ws2812b_loop_mode_t loop_mode);

/**
 * @brief 所有led灯同时有呼吸灯效果，亮度颜色都统一，需要设置颜色
 *
 * @param color_rgb 呼吸灯颜色
 * @param breath_time_ms 呼吸灯最小正周期时间，单位 ms
 * @param start_from_max 是否从最大亮度开始
 */
static void ws2812b_led_breathing_all(ws2812b_color_rgb_t color_rgb, uint16_t breath_time_ms, bool start_from_max);

/**
 * @brief 彩虹效果加呼吸灯效果，每个LED颜色亮度统一，不需要设置颜色
 *
 * @param breath_time_ms 呼吸灯跑完一圈huv色环时间，单位 ms
 */
static void ws2812b_led_rainbow_breathing_all(uint16_t loop_time_ms);

/**
 * @brief 彩虹效果加波浪呼吸灯效果，每个LED颜色亮度统一，不需要设置颜色
 *
 */
static void ws2812b_led_rainbow_breathing_wave(void);

/**
 * @brief 像流星一样的效果，从一端到滑道另一端，亮度逐渐减小，非常快速，需要预先设置颜色
 *
 * @param color_rgb 颜色
 * @param metror_time_ms 流星效果持续时间，单位 ms
 * @param direction 流星方向
 * @param accumulate 是否累积效果，如果为 true，效果会叠加，否则会清除之前的效果
 *
 */
static void ws2812b_led_meteor(ws2812b_color_rgb_t color_rgb, uint16_t metror_time_ms, ws2812b_direction_t direction, bool accumulate);

/**
 * @brief 乱闪，最杀马特的一集，用来提示门锁上了
 *
 */
static void ws2812b_led_random_color(void);

/**
 * @brief 闪烁效果
 *
 * @param color_rgb     颜色
 * @param flash_hold_time_ms 最小正周期持续时间, 单位 ms，不超过1分钟
 * @param light_on_duty_cycle 亮灯时间占空比，0-100
 * @param flash_count 闪烁次数
 */
static void ws2812b_led_blink(ws2812b_color_rgb_t color_rgb, uint16_t flash_hold_time_ms, uint8_t light_on_duty_cycle, uint8_t flash_count);

/**
 * @brief 像瀑布一样的效果，一开始都是暗的，然后从顶端向底端LED依次亮起，亮起后的轨迹不会消失
 *
 * @param color_rgb 颜色
 * @param waterfall_hold_time_ms 效果时间，单位 ms
 */
static void ws2812b_led_waterfall(ws2812b_color_rgb_t color_rgb, uint16_t waterfall_hold_time_ms);

/**
 * @brief 实现日出效果的灯光动态变化，从顶端到底端逐渐增加亮度。
 *
 * 功能描述：
 * - 效果开始时，顶端的 LED 逐渐变亮，而底端的 LED 完全熄灭。
 * - 随着效果进行，每个 LED 的亮度逐步增加，不同 LED 之间保持亮度差。
 * - 当某颗 LED 的亮度达到最大值时，其亮度不再变化，而下方的 LED 继续变亮。
 * - 整体效果是从顶端到底端逐步呈现日出的渐亮效果。
 *
 * @param color_rgb 指定 LED 显示的颜色（RGB 格式）。
 * @param sunrise_hold_time_ms 效果持续的总时间，单位为毫秒 (ms)。
 */
static void ws2812b_led_sunrise(ws2812b_color_rgb_t color_rgb, uint16_t sunrise_hold_time_ms);

/**
 * @brief 关闭 LED 灯带，不发光
 *
 */
static void ws2812b_shutdown(void);

// 函数定义

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
static void
led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i)
    {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

// 刷入数据到 LED 灯带
static void flash_led_strip()
{
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

static ws2812b_color_rgb_t adjust_brightness(ws2812b_color_rgb_t *color_rgb, int brightness)
{
    ws2812b_color_rgb_t adjusted_color = {
        .red = color_rgb->red * brightness / 100,
        .green = color_rgb->green * brightness / 100,
        .blue = color_rgb->blue * brightness / 100,
    };

    return adjusted_color;
}

static const char *get_effect_name(ws2812b_state_effect_t effect)
{
    switch (effect)
    {
    case LED_EFFECT_DEBUG:
        return ENUM_TO_STRING(LED_EFFECT_DEBUG);
    case LED_EFFECT_DEFAULT_STATE:
        return ENUM_TO_STRING(LED_EFFECT_DEFAULT_STATE);
    case LED_EFFECT_SINGLE_OPEN_DOOR:
        return ENUM_TO_STRING(LED_EFFECT_SINGLE_OPEN_DOOR);
    case LED_EFFECT_ALWAYS_OPEN_MODE:
        return ENUM_TO_STRING(LED_EFFECT_ALWAYS_OPEN_MODE);
    case LED_EFFECT_OPEN_MODE_END:
        return ENUM_TO_STRING(LED_EFFECT_OPEN_MODE_END);
    case LED_EFFECT_CONFIRM_FACTORY_RESET:
        return ENUM_TO_STRING(LED_EFFECT_CONFIRM_FACTORY_RESET);
    case LED_EFFECT_FACTORY_RESETTING:
        return ENUM_TO_STRING(LED_EFFECT_FACTORY_RESETTING);
    case LED_EFFECT_DEVICE_INITIALIZED:
        return ENUM_TO_STRING(LED_EFFECT_DEVICE_INITIALIZED);
    case LED_EFFECT_BLE_TRY_PAIRING:
        return ENUM_TO_STRING(LED_EFFECT_BLE_TRY_PAIRING);
    case LED_EFFECT_BLE_PAIRING_MODE:
        return ENUM_TO_STRING(LED_EFFECT_BLE_PAIRING_MODE);
    case LED_EFFECT_BLE_CONNECTED_FIRST_TIME:
        return ENUM_TO_STRING(LED_EFFECT_BLE_CONNECTED_FIRST_TIME);
    case LED_EFFECT_VISITOR_CODE_OPEN_DOOR:
        return ENUM_TO_STRING(LED_EFFECT_VISITOR_CODE_OPEN_DOOR);
    case LED_EFFECT_VISITOR_CODE_TIME_EXPIRED:
        return ENUM_TO_STRING(LED_EFFECT_VISITOR_CODE_TIME_EXPIRED);
    case LED_EFFECT_OPEN_BLUETOOTH_NEARBY:
        return ENUM_TO_STRING(LED_EFFECT_OPEN_BLUETOOTH_NEARBY);
    case LED_EFFECT_OPEN_BLUETOOTH_FINISHED:
        return ENUM_TO_STRING(LED_EFFECT_OPEN_BLUETOOTH_FINISHED);
    case LED_EFFECT_LOCK_DOOR:
        return ENUM_TO_STRING(LED_EFFECT_LOCK_DOOR);
    case LED_EFFECT_POWER_ON_ANIMATION:
        return ENUM_TO_STRING(LED_EFFECT_POWER_ON_ANIMATION);
    case LED_EFFECT_FIRST_POWER_ON_ACTIVATE:
        return ENUM_TO_STRING(LED_EFFECT_FIRST_POWER_ON_ACTIVATE);
    default:
        return "Unknown Effect";
    }
}

void loop_ws2812b_effect()
{
    current_effect_btn = (current_effect_btn + 1) % WS2812B_NUMBER_OF_EFFECTS;
    ws2812b_switch_effect(current_effect_btn);
}

void ws2812b_switch_effect(ws2812b_state_effect_t effect)
{
    ESP_LOGI(TAG, "Sent! Switch to effect: %s", get_effect_name(effect));

    ws2812b_current_effect = effect;
    is_switch_effect = true;
    // effect_data = (ws2812b_queue_data_t){
    //     .current_effect = effect,
    //     .effect_args = {
    //         .color_rgb = WHITE_RGB, // 根据需要设置颜色
    //         .direction = LED_DIRECTION_TOP_DOWN,
    //         .loop_mode = LED_MODE_LOOP,
    //     },
    // };

    // 通知LED线程切换效果
    // xTaskNotify(xLedTaskHandle, 0, eIncrement);
    // if (xQueueSend(effect_queue, &effect_data, portMAX_DELAY) == pdPASS)
    // {
    //     ESP_LOGI(TAG, "Effect data sent to queue.");
    // }
    // else
    // {
    //     ESP_LOGI(TAG, "Failed to send effect data to queue.");
    // }
}

// 从队列中接收转换效果命令
static void queue_receive_from_button(void)
{
    // if (xTaskNotifyWait(0, 0, &notify_count, pdMS_TO_TICKS(10)) == pdPASS) // 说明需要转换效果了
    // {
    //     // 不要打印任何东西，需要低延时
    //     // 用Notify做通知而不用queue是因为，通知时间开销更小，能够满足无极变色的需求
    //     if (effect_queue == NULL)
    //     {
    //         ESP_LOGE(TAG, "Effect queue not initialized!");
    //     }

    //     if (xQueueReceive(effect_queue, &received_effect_queue_data, 10) == pdPASS)
    //     {
    //         // 更新当前效果
    //         is_switch_effect = true;
    //         ws2812b_current_effect = received_effect_queue_data.current_effect;
    //         ESP_LOGI(TAG, "Received! Switch to effect: %s", get_effect_name(ws2812b_current_effect));
    //     }
    // }

    return;
}

// 检查是否需要切换效果
static bool check_effect_switch(void)
{
    if (is_switch_effect)
    {
        ESP_LOGI(TAG, "Effect switched during transition!");
        is_switch_effect = false;
        return true; // 指示需要切换
    }
    return false; // 不需要切换
}

void ws2812b_led_init(void)
{
    ESP_LOGI(TAG, "Create RMT TX channel");
    led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");
    led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    // ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    esp_err_t err = rmt_new_led_strip_encoder(&encoder_config, &led_encoder);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install LED strip encoder: %s", esp_err_to_name(err));
        return; // 提前返回，避免使用未初始化的 encoder
    }

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "WB2812B led strip initialized");
    // 使用复合字面量
    tx_config = (rmt_transmit_config_t){
        .loop_count = 0,
    };

    // 初始化队列
    effect_queue = xQueueCreate(1, sizeof(ws2812b_queue_data_t));

    // 初始化随机数生成器（如果在主任务中已初始化，可以移除）
    srand(time(NULL));

    // 创建效果任务
    xTaskCreate(ws2812b_effect_task, "ws2812b_effect_task", 2048, NULL, 5, &xLedTaskHandle);
}

static void ws2812b_effect_task(void *arg)
{

    while (1)
    {
        // 检查队列是否有新的效果命令
        queue_receive_from_button();

        // 执行当前效果
        switch (ws2812b_current_effect)
        {
        case LED_EFFECT_DEBUG:
            // 用于调试
            ws2812b_led_waterfall((ws2812b_color_rgb_t)GREEN_RGB, 500);

            break;
        case LED_EFFECT_DEFAULT_STATE:
            ws2812b_led_rainbow_breathing_all(15 * 1000); // 15秒走完一圈色环
            break;

        case LED_EFFECT_SINGLE_OPEN_DOOR:
            ws2812b_led_waterfall((ws2812b_color_rgb_t)GREEN_RGB, 250);
            ws2812b_led_set_color_all((ws2812b_color_rgb_t)GREEN_RGB, TIME_RECOVER_TEMP_OPEN);
            break;

        case LED_EFFECT_ALWAYS_OPEN_MODE:
            ws2812b_led_breathing_all((ws2812b_color_rgb_t)GREEN_RGB, 5600, true);
            break;

        case LED_EFFECT_OPEN_MODE_END:
            ws2812b_led_blink((ws2812b_color_rgb_t)GREEN_RGB, 200, 50, 3);
            break;

        case LED_EFFECT_CONFIRM_FACTORY_RESET:
            ws2812b_led_meteor((ws2812b_color_rgb_t)GREEN_RGB, 500, LED_DIRECTION_TOP_DOWN, true);
            break;

        case LED_EFFECT_FACTORY_RESETTING:
            ws2812b_led_breathing_all((ws2812b_color_rgb_t)RED_RGB, 500, false);
            break;

        case LED_EFFECT_DEVICE_INITIALIZED:
            ws2812b_led_blink((ws2812b_color_rgb_t)RED_RGB, 200, 50, 3);
            break;

        case LED_EFFECT_BLE_TRY_PAIRING:
            ws2812b_led_meteor((ws2812b_color_rgb_t)BLUE_RGB, 500, LED_DIRECTION_TOP_DOWN, true);
            break;

        case LED_EFFECT_BLE_PAIRING_MODE:
            ws2812b_led_breathing_all((ws2812b_color_rgb_t)BLUE_RGB, 1000, false);
            break;

        case LED_EFFECT_BLE_CONNECTED_FIRST_TIME:
            ws2812b_led_blink((ws2812b_color_rgb_t)BLUE_RGB, 200, 50, 3);
            break;

        case LED_EFFECT_OPEN_BLUETOOTH_NEARBY:
            ws2812b_led_waterfall((ws2812b_color_rgb_t)BLUE_RGB, 1000);
            break;

        case LED_EFFECT_OPEN_BLUETOOTH_FINISHED:
            ws2812b_led_blink((ws2812b_color_rgb_t)BLUE_RGB, 200, 50, 3);
            break;

        case LED_EFFECT_VISITOR_CODE_OPEN_DOOR:
            ws2812b_led_waterfall((ws2812b_color_rgb_t)PURPLE_RGB, 500);
            break;

        case LED_EFFECT_VISITOR_CODE_TIME_EXPIRED:
            ws2812b_led_blink((ws2812b_color_rgb_t)PURPLE_RGB, 200, 50, 3);
            break;

        case LED_EFFECT_LOCK_DOOR:
            ws2812b_led_random_color();
            break;
        case LED_EFFECT_POWER_ON_ANIMATION:
            ws2812b_shutdown();
            break;

        case LED_EFFECT_FIRST_POWER_ON_ACTIVATE:
            ws2812b_led_sunrise((ws2812b_color_rgb_t)BLUE_RGB, 6 * 1000);
            break;

        default:
            ESP_LOGW(TAG, "Unknown effect: %d", ws2812b_current_effect);
            ws2812b_current_effect = DEFAULT_EFFECT;
            break;
        }
        // 延迟一小段时间，避免任务占用过多 CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ws2812b_set_color(int index, uint8_t r, uint8_t g, uint8_t b)
{
    // 确保索引在有效范围内
    if (index < 0 || index >= WS2812B_LED_NUMBERS)
    {
        ESP_LOGE(TAG, "Index %d is out of range", index);
        return;
    }

    // 设置指定索引的 LED 颜色 (RGB)
    // LED 的 RGB 数据是按照 Green, Blue, Red 顺序存储的
    led_strip_pixels[index * 3 + 0] = g; // Green
    led_strip_pixels[index * 3 + 1] = b; // Blue
    led_strip_pixels[index * 3 + 2] = r; // Red

    // 刷新数据到 LED 灯带
    flash_led_strip();
}

static void ws2812b_led_set_color_all(ws2812b_color_rgb_t rgb_color, uint16_t time_ms)
{
    // 设置所有 LED 的颜色为相同的 RGB
    for (int i = 0; i < WS2812B_LED_NUMBERS; i++)
    {
        led_strip_pixels[i * 3 + 0] = rgb_color.green;
        led_strip_pixels[i * 3 + 1] = rgb_color.blue;
        led_strip_pixels[i * 3 + 2] = rgb_color.red;
    }

    // 刷新颜色到灯带
    flash_led_strip();

    uint16_t elapsed_time = 0;             // 记录已经持续的时间
    const uint16_t CHECK_INTERVAL_MS = 10; // 每隔 10ms 检测一次

    while (elapsed_time < time_ms)
    {
        queue_receive_from_button();

        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
        elapsed_time += CHECK_INTERVAL_MS;

        if (check_effect_switch())
        {
            return;
        }
    }
}

static void ws2812b_led_rainbow_all(void)
{
    queue_receive_from_button();

    // 可配置变量
    int hue_start = 0;            // 起始 Hue 值
    int hue_end = 360;            // 结束 Hue 值
    int transition_steps = 360;   // 颜色变化的步数（覆盖整个色环）
    int TRANSITION_DELAY_MS = 20; // 每一步的延迟时间 (ms)
    int saturation = 100;         // 饱和度
    int value = 100;              // 亮度

    for (int step = 0; step < transition_steps; step++)
    {
        // 计算当前 Hue 值
        int current_hue = hue_start + step * (hue_end - hue_start) / transition_steps;

        // 将 HSV 转换为 RGB
        uint32_t red, green, blue;
        led_strip_hsv2rgb(current_hue, saturation, value, &red, &green, &blue);

        // 设置所有 LED 的颜色为相同的 RGB
        for (int i = 0; i < WS2812B_LED_NUMBERS; i++)
        {
            led_strip_pixels[i * 3 + 0] = green;
            led_strip_pixels[i * 3 + 1] = blue;
            led_strip_pixels[i * 3 + 2] = red;
        }

        // 刷新颜色到灯带
        flash_led_strip();

        if (check_effect_switch())
        {
            return;
        }

        // 延迟控制彩虹变化速度
        vTaskDelay(pdMS_TO_TICKS(TRANSITION_DELAY_MS));
    }
}

static void ws2812b_led_rainbow_wave(void)
{
    queue_receive_from_button();

    uint32_t hue_start = 0;                            // 起始 Hue 值
    uint32_t hue_end = 360;                            // 结束 Hue 值
    uint32_t hue_step = hue_end / WS2812B_LED_NUMBERS; // 每个 LED 的 Hue 步长
    uint32_t saturation = 100;                         // 饱和度（固定为最大值）
    uint32_t value = 100;                              // 亮度（固定为最大值）

    for (int i = 0; i < WS2812B_LED_NUMBERS; i++)
    {
        // 计算当前 LED 的 Hue 值
        uint32_t current_hue = hue_start + i * hue_step;

        // 将 HSV 转换为 RGB
        uint32_t red, green, blue;
        led_strip_hsv2rgb(current_hue, saturation, value, &red, &green, &blue);

        // 设置 LED 的颜色
        led_strip_pixels[i * 3 + 0] = green;
        led_strip_pixels[i * 3 + 1] = blue;
        led_strip_pixels[i * 3 + 2] = red;
    }

    // 刷新颜色到灯带
    flash_led_strip();

    if (check_effect_switch())
    {
        return;
    }

    // 延迟控制彩虹的移动速度
    vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));

    // 更新起始 Hue 值，使彩虹效果动态移动
    hue_start = (hue_start + hue_step) % hue_end;
}

static void ws2812b_led_breathing_wave(ws2812b_color_rgb_t color_rgb, ws2812b_direction_t direction, ws2812b_loop_mode_t loop_mode)
{
    queue_receive_from_button();

    // 可配置变量
    int brightness_min = 10;       // 最低亮度
    int brightness_max = 100;      // 最高亮度
    int transition_steps = 100;    // 亮度变化的步数
    int TRANSITION_DELAY_MS = 100; // 每一步的延迟时间 (ms)

    // 根据方向初始化 LED 索引范围
    int start = (direction == LED_DIRECTION_TOP_DOWN) ? 0 : WS2812B_LED_NUMBERS - 1;
    int end = (direction == LED_DIRECTION_TOP_DOWN) ? WS2812B_LED_NUMBERS : -1;
    int step = (direction == LED_DIRECTION_TOP_DOWN) ? 1 : -1;

    // 循环控制（一次完整的波动效果）
    for (int t = 0; t < WS2812B_LED_NUMBERS; t++) // 每次波动从一个LED开始
    {
        for (int i = start; i != end; i += step)
        {
            // 根据当前位置和时间t计算亮度
            int distance = abs(i - t);
            int brightness = brightness_max - (brightness_max - brightness_min) * distance / WS2812B_LED_NUMBERS;

            // 限制亮度范围
            if (brightness < brightness_min)
                brightness = brightness_min;

            // 根据亮度调整颜色
            ws2812b_color_rgb_t adjusted_color = {
                .red = color_rgb.red * brightness / 100,
                .green = color_rgb.green * brightness / 100,
                .blue = color_rgb.blue * brightness / 100,
            };

            // 设置当前LED颜色
            led_strip_pixels[i * 3 + 0] = adjusted_color.green;
            led_strip_pixels[i * 3 + 1] = adjusted_color.blue;
            led_strip_pixels[i * 3 + 2] = adjusted_color.red;
        }

        // 刷新LED数据
        flash_led_strip();

        if (check_effect_switch())
        {
            return;
        }
        // 延迟控制
        vTaskDelay(pdMS_TO_TICKS(TRANSITION_DELAY_MS));
    }

    // 如果是来回模式，反转方向
    if (loop_mode == LED_LOOP_MODE_PINGPONG)
    {
        ws2812b_led_breathing_wave(color_rgb,
                                   (direction == LED_DIRECTION_TOP_DOWN) ? LED_DIRECTION_TOP_DOWN : LED_DIRECTION_TOP_DOWN,
                                   LED_LOOP_MODE_SINGLE);
    }
}

static void ws2812b_led_breathing_all(ws2812b_color_rgb_t color_rgb, uint16_t breath_time_ms, bool start_from_max)
{
    if (breath_time_ms == 0)
    {
        ESP_LOGE(TAG, "Breath time is 0, no effect will be shown!");
        return;
    }

    // 可配置变量
    int brightness_min = 1;     // 最低亮度
    int brightness_max = 100;   // 最高亮度
    int transition_steps = 600; // 亮度变化的步数

    // 调整亮度（呼吸效果）
    for (int step = 0; step <= transition_steps; step++) // 根据步数控制变化
    {
        queue_receive_from_button();

        int brightness;
        float progress = (float)step / transition_steps; // 当前进度
        float power = 2.2;                               // 非线性变化的幂次

        if (progress <= 0.5)
        {
            // 前半段
            if (start_from_max)
            {
                // 从最亮变到最暗
                progress = pow((1 - 2 * progress), power); // 非线性变化
                brightness = brightness_min + (int)((brightness_max - brightness_min) * progress);
            }
            else
            {
                // 从最暗变到最亮
                progress = pow(progress, power); // 非线性变化
                brightness = brightness_min + (int)((brightness_max - brightness_min) * progress);
            }
        }
        else
        {
            // 后半段
            if (start_from_max)
            {
                // 从最暗变到最亮
                progress = pow((2 * progress - 1), power); // 非线性变化
                brightness = brightness_min + (int)((brightness_max - brightness_min) * progress);
            }
            else
            {
                // 从最亮变到最暗
                progress = pow((0.5 - (progress - 0.5)), power); // 非线性变化
                brightness = brightness_min + (int)((brightness_max - brightness_min) * progress);
            }
        }

        // 根据亮度调整颜色
        ws2812b_color_rgb_t adjusted_color = adjust_brightness(&color_rgb, brightness);
        ws2812b_led_set_color_all(adjusted_color, 0);

        if (check_effect_switch())
        {
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(15)); // 每次更新延时，调整变化速度
    }
}

static void ws2812b_led_rainbow_breathing_all(uint16_t loop_time_ms)
{

    // 可配置变量
    int hue_start = 0;                                         // 起始颜色的 Hue 值
    int hue_end = 360;                                         // 结束颜色的 Hue 值
    int brightness_min = 10;                                   // 最低亮度
    int brightness_max = 100;                                  // 最高亮度
    int transition_steps = 1000;                               // 颜色和亮度变化的步数
    int TRANSITION_DELAY_MS = loop_time_ms / transition_steps; // 每一步的延迟时间 (ms)

    // HSV 和 RGB 颜色结构体
    ws2812b_color_hsv_t hsv_color = {0, 100, 0}; // 初始化 HSV 颜色
    ws2812b_color_rgb_t rgb_color = {0, 0, 0};   // 初始化 RGB 颜色

    // 同时改变颜色和亮度
    for (int step = 0; step <= transition_steps; step++) // 根据步数控制变化
    {
        queue_receive_from_button();

        // 计算当前 Hue 和亮度值
        hsv_color.hue = hue_start + step * (hue_end - hue_start) / transition_steps;                                                              // 线性插值计算 Hue
        hsv_color.value = (step <= transition_steps / 2) ? (brightness_min + step * (brightness_max - brightness_min) / (transition_steps / 2)) : // 前半段亮度增加
                              (brightness_max - (step - transition_steps / 2) * (brightness_max - brightness_min) / (transition_steps / 2));      // 后半段亮度减少

        // 将 HSV 转换为 RGB
        led_strip_hsv2rgb(hsv_color.hue, hsv_color.saturation, hsv_color.value,
                          &rgb_color.red, &rgb_color.green, &rgb_color.blue);

        // 设置整排灯带为相同颜色
        for (int i = 0; i < WS2812B_LED_NUMBERS; i++)
        {
            led_strip_pixels[i * 3 + 0] = rgb_color.green;
            led_strip_pixels[i * 3 + 1] = rgb_color.blue;
            led_strip_pixels[i * 3 + 2] = rgb_color.red;
        }

        // 刷新颜色到灯带
        flash_led_strip();

        if (check_effect_switch())
        {
            return;
        }

        // 等待一段时间，展现动态效果
        vTaskDelay(pdMS_TO_TICKS(TRANSITION_DELAY_MS)); // 每次更新延时，调整变化速度
    }
}

static void ws2812b_led_rainbow_breathing_wave(void)
{
    queue_receive_from_button();

    if (check_effect_switch())
    {
        return;
    }
}

static void ws2812b_led_meteor(ws2812b_color_rgb_t color_rgb, uint16_t metror_time_ms, ws2812b_direction_t direction, bool accumulate)
{
    // 初始化 LED 数据
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));

    // 配置参数
    const int TAIL_LENGTH = 2;                                                // 流星尾巴长度
    const int TRANSITION_DELAY_MS = 50;                                       // 每帧的延迟时间 (ms)
    const int METEOR_ACCUMULATE_TIME = metror_time_ms / WS2812B_LED_NUMBERS;  // 每课流星累积时间
    const int METEOR_ACCUMULATE_COUNT = metror_time_ms / TRANSITION_DELAY_MS; // 流星效果循环次数

    // 计算方向
    const int start = (direction == LED_DIRECTION_TOP_DOWN) ? 0 : WS2812B_LED_NUMBERS - 1;
    const int end = (direction == LED_DIRECTION_TOP_DOWN) ? WS2812B_LED_NUMBERS : -1;
    const int step = (direction == LED_DIRECTION_TOP_DOWN) ? 1 : -1;

    int accumulate_leds[WS2812B_LED_NUMBERS] = {0}; // 累积效果的 LED 数组
    int accumulate_threshold = 3;                   // 累积效果的阈值，流星次数
    ws2812b_color_rgb_t adjusted_color = {};
    ws2812b_color_rgb_t tail_color = {};

    for (int meteor_accumulate_count = WS2812B_LED_NUMBERS; meteor_accumulate_count >= 0; meteor_accumulate_count--)
    {

        for (int meteor_loop_count = 0; meteor_loop_count < accumulate_threshold; meteor_loop_count++)
        {
            queue_receive_from_button();

            for (int meteor_head_pos = start; meteor_head_pos != end; meteor_head_pos += step)
            {
                memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
                // 每次更新 LED 带
                for (int led_index = 0; led_index < WS2812B_LED_NUMBERS; led_index++)
                {
                    // 计算亮度衰减
                    int distance_from_meteor = abs(meteor_head_pos - led_index);
                    int brightness = 0;
                    if (meteor_head_pos == led_index)
                    {
                        brightness = 100;
                    }

                    // 调整颜色
                    adjusted_color = adjust_brightness(&color_rgb, brightness);

                    led_strip_pixels[led_index * 3 + 0] = MIN(255, led_strip_pixels[led_index * 3 + 0] + adjusted_color.green);
                    led_strip_pixels[led_index * 3 + 1] = MIN(255, led_strip_pixels[led_index * 3 + 1] + adjusted_color.blue);
                    led_strip_pixels[led_index * 3 + 2] = MIN(255, led_strip_pixels[led_index * 3 + 2] + adjusted_color.red);

                    // 为流星尾巴添加效果
                    for (int tail_pos = 1; tail_pos <= TAIL_LENGTH; tail_pos++)
                    {
                        int tail_index = led_index - tail_pos * step;
                        brightness = brightness / (tail_pos * 3);

                        tail_color = adjust_brightness(&color_rgb, brightness);

                        if (tail_index < 0)
                        {
                            break;
                        }
                        else
                        {
                            led_strip_pixels[tail_index * 3 + 0] = MIN(255, led_strip_pixels[tail_index * 3 + 0] + tail_color.green);
                            led_strip_pixels[tail_index * 3 + 1] = MIN(255, led_strip_pixels[tail_index * 3 + 1] + tail_color.blue);
                            led_strip_pixels[tail_index * 3 + 2] = MIN(255, led_strip_pixels[tail_index * 3 + 2] + tail_color.red);
                        }
                    }
                }

                for (int i = 0; i <= WS2812B_LED_NUMBERS; i++)
                {
                    if (accumulate_leds[i] == 1)
                    {
                        led_strip_pixels[i * 3 + 0] = color_rgb.green;
                        led_strip_pixels[i * 3 + 1] = color_rgb.blue;
                        led_strip_pixels[i * 3 + 2] = color_rgb.red;
                    }
                }

                // 刷新颜色到灯带
                flash_led_strip();

                if (check_effect_switch())
                {
                    return;
                }

                // 延迟控制速度
                vTaskDelay(pdMS_TO_TICKS(TRANSITION_DELAY_MS));
            }
        }

        accumulate_leds[meteor_accumulate_count] = 1;

        if (meteor_accumulate_count == 0)
        {
            ws2812b_led_set_color_all(color_rgb, 0);
        }
    }
    // 最后再亮一会
    vTaskDelay(pdMS_TO_TICKS(2000));
}

static void ws2812b_led_random_color(void)
{
    queue_receive_from_button();

    // 设置 LED 随机闪烁效果
    for (int i = 0; i < WS2812B_LED_NUMBERS; i++)
    {
        // 生成随机的 RGB 颜色
        uint8_t r = rand() % 256; // 随机生成 0-255 的红色分量
        uint8_t g = rand() % 256; // 随机生成 0-255 的绿色分量
        uint8_t b = rand() % 256; // 随机生成 0-255 的蓝色分量

        // 随机熄灭这次的LED
        if (rand() % 4 == 0)
        {
            r = 0;
            g = 0;
            b = 0;
        }

        // 设置 LED 的颜色到 led_strip_pixels 数组 WS2812B 的数据顺序是 G -> R -> B
        led_strip_pixels[i * 3 + 0] = g; // Green
        led_strip_pixels[i * 3 + 1] = r; // Red
        led_strip_pixels[i * 3 + 2] = b; // Blue
    }

    // 刷新 LED 数据到灯带
    flash_led_strip();

    vTaskDelay(pdMS_TO_TICKS(100)); // 延迟控制速度

    if (check_effect_switch())
    {
        return;
    }
}

static void ws2812b_led_blink(ws2812b_color_rgb_t color_rgb, uint16_t flash_hold_time_ms, uint8_t light_on_duty_cycle, uint8_t flash_count)
{
    queue_receive_from_button();

    // 可配置变量
    int TRANSITION_DELAY_MS = 1000 / 60;                                // 闪烁频率 (60Hz)
    int light_on_time = flash_hold_time_ms * light_on_duty_cycle / 100; // 亮灯时间
    int light_off_time = flash_hold_time_ms - light_on_time;            // 灭灯时间

    // 闪烁次数
    for (int i = 0; i < flash_count; i++)
    {
        // 亮灯
        ws2812b_led_set_color_all(color_rgb, 0);
        vTaskDelay(pdMS_TO_TICKS(light_on_time));

        // 灭灯
        ws2812b_led_set_color_all((ws2812b_color_rgb_t){0, 0, 0}, 0);
        vTaskDelay(pdMS_TO_TICKS(light_off_time));

        if (check_effect_switch())
        {
            return;
        }
    }
}

static void ws2812b_led_waterfall(ws2812b_color_rgb_t color_rgb, uint16_t waterfall_hold_time_ms)
{

    int TRANSITION_DELAY_MS = waterfall_hold_time_ms / WS2812B_LED_NUMBERS;

    // 初始化 LED 数据
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));

    // 从顶端到底端逐个点亮
    for (int i = 0; i < WS2812B_LED_NUMBERS; i++)
    {
        queue_receive_from_button();

        // 设置当前 LED 为指定颜色
        led_strip_pixels[i * 3 + 0] = color_rgb.green;
        led_strip_pixels[i * 3 + 1] = color_rgb.blue;
        led_strip_pixels[i * 3 + 2] = color_rgb.red;

        // 刷新颜色到灯带
        flash_led_strip();

        if (check_effect_switch())
        {
            return;
        }

        // 延迟控制速度
        vTaskDelay(pdMS_TO_TICKS(TRANSITION_DELAY_MS));

        if (check_effect_switch())
        {
            return;
        }
    }
}

static void ws2812b_led_sunrise(ws2812b_color_rgb_t color_rgb, uint16_t sunrise_hold_time_ms)
{
    if (sunrise_hold_time_ms == 0)
    {
        ESP_LOGE(TAG, "Sunrise hold time is 0, no effect will be shown!");
        return;
    }

    // 配置参数
    int total_steps = 100;                                  // 亮度渐变的总步数
    int step_delay_ms = sunrise_hold_time_ms / total_steps; // 每步的延迟时间
    int led_count = WS2812B_LED_NUMBERS;                    // 总的 LED 数量

    // 初始化 LED 亮度数组
    float brightness[WS2812B_LED_NUMBERS] = {0};

    for (int step = 0; step < total_steps; step++)
    {
        queue_receive_from_button();
        for (int i = 0; i < led_count; i++)
        {
            // brightness[i] 增加逻辑
            float progress = (float)(step - i * (total_steps / led_count)) / (total_steps / led_count);
            if (progress < 0)
                progress = 0; // 确保亮度不低于0
            if (progress > 1.0f)
                progress = 1.0f; // 确保亮度不高于1.0
            brightness[i] = progress;

            if (brightness[i] > 1.0f) // 限制最大亮度
                brightness[i] = 1.0f;

            // 根据亮度调整颜色
            ws2812b_color_rgb_t adjusted_color = {
                .red = (uint32_t)(color_rgb.red * brightness[i]),
                .green = (uint32_t)(color_rgb.green * brightness[i]),
                .blue = (uint32_t)(color_rgb.blue * brightness[i]),
            };

            // 设置 LED 的颜色
            led_strip_pixels[i * 3 + 0] = adjusted_color.green;
            led_strip_pixels[i * 3 + 1] = adjusted_color.blue;
            led_strip_pixels[i * 3 + 2] = adjusted_color.red;
        }

        // print brightness
        // ESP_LOGI(TAG, "Brightness: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", brightness[0], brightness[1], brightness[2], brightness[3], brightness[4], brightness[5], brightness[6], brightness[7], brightness[8], brightness[9]);

        // 刷新灯带
        flash_led_strip();

        if (check_effect_switch())
        {
            return;
        }

        // 延迟
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    }
}

static void ws2812b_shutdown(void)
{
    // 关闭 LED 灯带，不发光
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
    flash_led_strip();
}
