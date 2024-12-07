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

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM GPIO_NUM_1    // GPIO number for the LED strip

#define WS2812B_LED_NUMBERS 6
#define EXAMPLE_CHASE_SPEED_MS 50

// 配置 RMT 通道
static rmt_channel_handle_t led_chan;
static rmt_encoder_handle_t led_encoder;
static rmt_transmit_config_t tx_config;

// ws2812b LED 灯带的像素数据
static uint8_t led_strip_pixels[WS2812B_LED_NUMBERS * 3];

// 定义日志标签
static const char *TAG = "WS2812B_LED";

// 定义全局队列
QueueHandle_t effect_queue = NULL;

// 定义全局参数变量并初始化
static ws2812b_queue_data_t received_effect_queue_data = {
    .effect_args = {
        .color_rgb = WHITE_RGB, // 默认Freedorm蓝色
        .direction = LED_DIRECTION_TOP_DOWN,
        .loop_mode = LED_MODE_LOOP,
    },
    .current_effect = DEFAULT_EFFECT,
};

static ws2812b_effect_t ws2812b_current_effect = DEFAULT_EFFECT;

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
 */
static void ws2812b_led_set_color_all(ws2812b_color_rgb_t rgb_color);

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
 */
static void ws2812b_led_breathing_all(ws2812b_color_rgb_t color_rgb);

/**
 * @brief 彩虹效果加呼吸灯效果，每个LED颜色亮度统一，不需要设置颜色
 *
 */
static void ws2812b_led_rainbow_breathing_all(void);

/**
 * @brief 彩虹效果加波浪呼吸灯效果，每个LED颜色亮度统一，不需要设置颜色
 *
 */
static void ws2812b_led_rainbow_breathing_wave(void);

/**
 * @brief 像流星一样的效果，从一端到滑道另一端，亮度逐渐减小，非常快速，需要预先设置颜色
 *
 * @param direction 流星方向, 从上到下或者从下到上
 *
 */
static void ws2812b_led_meteor(ws2812b_color_rgb_t color_rgb, ws2812b_direction_t direction);

/**
 * @brief 乱闪，最杀马特的一集，用来提示门锁上了
 *
 */
static void ws2812b_led_random_color(void);

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
static void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
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
    xTaskCreate(ws2812b_effect_task, "ws2812b_effect_task", 2048, NULL, 5, NULL);
}

static void ws2812b_effect_task(void *arg)
{

    while (1)
    {
        // 检查队列是否有新的效果命令

        if (effect_queue == NULL)
        {
            ESP_LOGE(TAG, "Effect queue not initialized!");
        }

        if (xQueueReceive(effect_queue, &received_effect_queue_data, 10) == pdPASS)
        {
            // 更新当前效果
            ESP_LOGI(TAG, "Switch to effect: %d", received_effect_queue_data.current_effect);
            ws2812b_current_effect = received_effect_queue_data.current_effect;
            ESP_LOGI(TAG, "Switch to effect: %d", ws2812b_current_effect);
        }

        // 执行当前效果
        switch (ws2812b_current_effect)
        {
        case LED_EFFECT_RAINBOW_ALL:
            // ws2812b_led_rainbow_all();
            break;

        case LED_EFFECT_RAINBOW_WAVE:
            // ws2812b_led_rainbow_wave();
            break;

        case LED_EFFECT_BREATHING_WAVE:
            // ws2812b_led_breathing_wave(received_effect_queue_data.effect_args.color_rgb,
            //    received_effect_queue_data.effect_args.direction,
            //    received_effect_queue_data.effect_args.loop_mode);
            break;

        case LED_EFFECT_BREATHING_ALL:
            // ws2812b_led_breathing_all(received_effect_queue_data.effect_args.color_rgb);
            break;

        case LED_EFFECT_RAINBOW_BREATHING_ALL:
            // ws2812b_led_rainbow_breathing_all();
            break;

        case LED_EFFECT_RAINBOW_BREATHING_WAVE:
            // ws2812b_led_rainbow_breathing_wave();
            break;
        case LED_EFFECT_METEOR:
            // ws2812b_led_meteor(received_effect_queue_data.effect_args.color_rgb, LED_DIRECTION_TOP_DOWN);
            break;
        case LED_EFFECT_RANDOM_COLOR:
            // ws2812b_led_random_color();
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
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

static void ws2812b_led_set_color_all(ws2812b_color_rgb_t rgb_color)
{
    uint32_t red = rgb_color.red;
    uint32_t green = rgb_color.green;
    uint32_t blue = rgb_color.blue;

    for (int i = 0; i < WS2812B_LED_NUMBERS; i++)
    {
        led_strip_pixels[i * 3 + 0] = green;
        led_strip_pixels[i * 3 + 1] = blue;
        led_strip_pixels[i * 3 + 2] = red;
    }
}

static void ws2812b_led_rainbow_all(void)
{
    // 可配置变量
    int hue_start = 0;            // 起始 Hue 值
    int hue_end = 360;            // 结束 Hue 值
    int transition_steps = 360;   // 颜色变化的步数（覆盖整个色环）
    int transition_delay_ms = 20; // 每一步的延迟时间 (ms)
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
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

        // 延迟控制彩虹变化速度
        vTaskDelay(pdMS_TO_TICKS(transition_delay_ms));
    }
}

static void ws2812b_led_rainbow_wave(void)
{
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

    if (led_chan == NULL || led_encoder == NULL)
    {
        ESP_LOGE(TAG, "led_chan or led_encoder is not initialized properly!");
    }

    // 刷新颜色到灯带
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

    // 延迟控制彩虹的移动速度
    vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));

    // 更新起始 Hue 值，使彩虹效果动态移动
    hue_start = (hue_start + hue_step) % hue_end;
}

static void ws2812b_led_breathing_wave(ws2812b_color_rgb_t color_rgb, ws2812b_direction_t direction, ws2812b_loop_mode_t loop_mode)
{
    // 可配置变量
    int brightness_min = 10;       // 最低亮度
    int brightness_max = 100;      // 最高亮度
    int transition_steps = 100;    // 亮度变化的步数
    int transition_delay_ms = 100; // 每一步的延迟时间 (ms)

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
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

        // 延迟控制
        vTaskDelay(pdMS_TO_TICKS(transition_delay_ms));
    }

    // 如果是来回模式，反转方向
    if (loop_mode == LED_LOOP_MODE_PINGPONG)
    {
        ws2812b_led_breathing_wave(color_rgb,
                                   (direction == LED_DIRECTION_TOP_DOWN) ? LED_DIRECTION_TOP_DOWN : LED_DIRECTION_TOP_DOWN,
                                   LED_LOOP_MODE_SINGLE);
    }
}

static void ws2812b_led_breathing_all(ws2812b_color_rgb_t color_rgb)
{
    // 可配置变量
    int brightness_min = 10;      // 最低亮度
    int brightness_max = 100;     // 最高亮度
    int transition_steps = 500;   // 亮度变化的步数
    int transition_delay_ms = 20; // 每一步的延迟时间 (ms)

    // 调整亮度（呼吸效果）
    for (int step = 0; step <= transition_steps; step++) // 根据步数控制变化
    {
        int brightness = (step <= transition_steps / 2) ? (brightness_min + step * (brightness_max - brightness_min) / (transition_steps / 2)) : // 前半段亮度增加
                             (brightness_max - (step - transition_steps / 2) * (brightness_max - brightness_min) / (transition_steps / 2));      // 后半段亮度减少

        // 根据亮度调整颜色
        ws2812b_color_rgb_t adjusted_color = {
            .red = color_rgb.red * brightness / 100,
            .green = color_rgb.green * brightness / 100,
            .blue = color_rgb.blue * brightness / 100,
        };

        // 设置整排灯带为调整后的颜色
        for (int i = 0; i < WS2812B_LED_NUMBERS; i++)
        {
            led_strip_pixels[i * 3 + 0] = adjusted_color.green;
            led_strip_pixels[i * 3 + 1] = adjusted_color.blue;
            led_strip_pixels[i * 3 + 2] = adjusted_color.red;
        }

        // 刷新颜色到灯带
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

        // 等待一段时间，展现动态效果
        vTaskDelay(pdMS_TO_TICKS(transition_delay_ms)); // 每次更新延时，调整变化速度
    }
}

static void ws2812b_led_rainbow_breathing_all(void)
{
    // 可配置变量
    int hue_start = 0;            // 起始颜色的 Hue 值
    int hue_end = 360;            // 结束颜色的 Hue 值
    int brightness_min = 10;      // 最低亮度
    int brightness_max = 100;     // 最高亮度
    int transition_steps = 500;   // 颜色和亮度变化的步数
    int transition_delay_ms = 20; // 每一步的延迟时间 (ms)

    // HSV 和 RGB 颜色结构体
    ws2812b_color_hsv_t hsv_color = {0, 100, 0}; // 初始化 HSV 颜色
    ws2812b_color_rgb_t rgb_color = {0, 0, 0};   // 初始化 RGB 颜色

    // 同时改变颜色和亮度
    for (int step = 0; step <= transition_steps; step++) // 根据步数控制变化
    {
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
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

        // 等待一段时间，展现动态效果
        vTaskDelay(pdMS_TO_TICKS(transition_delay_ms)); // 每次更新延时，调整变化速度
    }
}

static void ws2812b_led_rainbow_breathing_wave(void)
{
    ; // 未实现TODO
}

static void ws2812b_led_meteor(ws2812b_color_rgb_t color_rgb, ws2812b_direction_t direction)
{
    // 配置参数
    int meteor_length = 3;        // 流星长度
    int fade_factor = 30;         // 衰减因子（控制亮度递减程度）
    int transition_delay_ms = 50; // 每帧的延迟时间 (ms)

    // 初始化 LED 数据
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));

    // 计算方向
    int start = (direction == LED_DIRECTION_TOP_DOWN) ? 0 : WS2812B_LED_NUMBERS - 1;
    int end = (direction == LED_DIRECTION_TOP_DOWN) ? WS2812B_LED_NUMBERS : -1;
    int step = (direction == LED_DIRECTION_TOP_DOWN) ? 1 : -1;

    // 流星效果
    for (int pos = start; pos != end; pos += step)
    {
        // 每次更新 LED 带
        for (int i = 0; i < WS2812B_LED_NUMBERS; i++)
        {
            // 计算亮度衰减
            int distance = abs(pos - i);
            int brightness = (distance < meteor_length) ? (color_rgb.red * (meteor_length - distance) / meteor_length) : 0;

            // 调整颜色
            ws2812b_color_rgb_t adjusted_color = {
                .red = color_rgb.red * brightness / 255,
                .green = color_rgb.green * brightness / 255,
                .blue = color_rgb.blue * brightness / 255,
            };

            // 设置 LED 颜色
            led_strip_pixels[i * 3 + 0] = adjusted_color.green;
            led_strip_pixels[i * 3 + 1] = adjusted_color.blue;
            led_strip_pixels[i * 3 + 2] = adjusted_color.red;
        }

        // 刷新颜色到灯带
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

        // 延迟控制速度
        vTaskDelay(pdMS_TO_TICKS(transition_delay_ms));

        // 添加衰减尾巴效果
        for (int i = 0; i < WS2812B_LED_NUMBERS * 3; i++)
        {
            if (led_strip_pixels[i] > fade_factor)
            {
                led_strip_pixels[i] -= fade_factor;
            }
            else
            {
                led_strip_pixels[i] = 0;
            }
        }
    }
}
static void ws2812b_led_random_color(void)
{
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

        // 设置 LED 的颜色到 led_strip_pixels 数组
        // WS2812B 的数据顺序是 G -> R -> B
        led_strip_pixels[i * 3 + 0] = g; // Green
        led_strip_pixels[i * 3 + 1] = r; // Red
        led_strip_pixels[i * 3 + 2] = b; // Blue
    }

    // 刷新 LED 数据到灯带
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

// static void ws2812b_led_random_color(void)
// {
//     // 初始化随机数生成器（仅需调用一次）
//     srand(time(NULL));

//     // 设置 LED 随机闪烁效果
//     for (int i = 0; i < WS2812B_LED_NUMBERS; i++)
//     {
//         // 生成随机的RGB颜色
//         uint8_t r = rand() % 256; // 随机生成 0-255 的红色分量
//         uint8_t g = rand() % 256; // 随机生成 0-255 的绿色分量
//         uint8_t b = rand() % 256; // 随机生成 0-255 的蓝色分量

//         // 设置 LED 的颜色
//         ws2812b_set_color(i, r, g, b);
//     }

//     // 等待一段时间来创建闪烁效果
//     // 这里假设有一个延时函数，比如 delay_ms(n)，根据实际情况调整
//     vTaskDelay(pdMS_TO_TICKS(100)); // 延时 100ms (可以根据需要调整)

//     // 重新调用函数产生新的乱闪效果
//     // 你可以选择持续调用或者让外部循环控制
//     // 在实际应用中，可以控制循环次数或者按某些条件停止
//     ws2812b_led_random_color();
// }
