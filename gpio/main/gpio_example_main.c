#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// 定义GPIO引脚
#define GPIO_INPUT_IO 0  // GPIO 0 接按键
#define GPIO_OUTPUT_IO 6 // GPIO 6 控制输出
#define LED_4 12

#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_IO)                   // 输入GPIO的位掩码
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_IO | 1ULL << LED_4) // 输出GPIO的位掩码

void app_main(void)
{
    // 配置 GPIO 0 为输入模式
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;     // 不使用中断
    io_conf.mode = GPIO_MODE_INPUT;            // 配置为输入模式
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; // 选择 GPIO 0
    io_conf.pull_up_en = 0;                    // 禁用上拉
    io_conf.pull_down_en = 0;                  // 启用下拉
    gpio_config(&io_conf);

    // 配置 GPIO 6 为输出模式
    io_conf.intr_type = GPIO_INTR_DISABLE;      // 不使用中断
    io_conf.mode = GPIO_MODE_OUTPUT;            // 配置为输出模式
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; // 选择 GPIO 6
    io_conf.pull_up_en = 0;                     // 禁用上拉
    io_conf.pull_down_en = 1;                   // enable下拉
    gpio_config(&io_conf);

    // 主循环
    while (1)
    {
        // 读取 GPIO 0 的电平状态
        int key_state = gpio_get_level(GPIO_INPUT_IO);

        if (key_state == 1)
        {
            // 按键按下，GPIO 6 输出高电平
            gpio_set_level(GPIO_OUTPUT_IO, 1);
            gpio_set_level(LED_4, 1);
        }
        else
        {
            // 按键松开，GPIO 6 输出低电平
            gpio_set_level(GPIO_OUTPUT_IO, 0);
            gpio_set_level(LED_4, 0);
        }

        // 延时 10 毫秒，避免反复检测造成误触
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
