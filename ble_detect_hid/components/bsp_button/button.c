#include "button.h"
#include "ble_module.h"
#include "ws2812b_led.h"
#include "_freedorm_main.h"
#include "lock_control.h"

// 宏定义
#define BUTTON_TAG "BUTTON"
#define LIGHT_EFFECT_DEMO 0

// 定义全局变量
uint32_t led_state_mask = 0; // 在这里初始化，位图，记录每个 GPIO 的当前状态
struct Button btn1;

uint8_t read_button_GPIO(uint8_t button_id)
{
    return gpio_get_level(PAIRING_BUTTON_GPIO);
}

void freedorm_button_init()
{
    uint64_t gpio_intput_sel = (1ULL << PAIRING_BUTTON_GPIO);

    // 初始化按键
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << PAIRING_BUTTON_GPIO;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_config_t input_io_conf = {
        .pin_bit_mask = gpio_intput_sel,      // 配置 GPIO0 为输入
        .mode = GPIO_MODE_INPUT,              // 输入模式
        .pull_up_en = GPIO_PULLUP_DISABLE,    // 上拉
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // 不需要下拉
        .intr_type = GPIO_INTR_DISABLE        // 不需要中断
    };
    gpio_config(&input_io_conf);

    // 初始化按键对象
    button_init(&btn1, read_button_GPIO, 1, 0); // 第三个参数为有效电平 0（低电平有效），第四个参数为按键 ID 艹，tmd debug半天结果是这里参考电平的问题，艹

    button_attach(&btn1, PRESS_REPEAT, BTN1_PRESS_REPEAT_Handler);
    button_attach(&btn1, SINGLE_CLICK, BTN1_SINGLE_CLICK_Handler);
    button_attach(&btn1, DOUBLE_CLICK, BTN1_DOUBLE_CLICK_Handler);
    button_attach(&btn1, LONG_PRESS_START, BTN1_LONG_PRESS_START_Handler);
    button_attach(&btn1, LONG_PRESS_HOLD, BTN1_LONG_PRESS_HOLD_Handler);
    button_start(&btn1);
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

// 单次开门
void BTN1_SINGLE_CLICK_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Single click detected");
    single_click_toogle(); // 单击开门
}

// 双击进入常开模式
void BTN1_DOUBLE_CLICK_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Double click detected");
    double_click_always_open(); // 双击进入常开模式
}

void BTN1_LONG_PRESS_START_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Long press start detected");
    xSemaphoreGive(pairing_semaphore);
    ws2812b_switch_effect(LED_EFFECT_BLE_PAIRING_MODE);
}

void BTN1_LONG_PRESS_HOLD_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Long press hold detected");
}