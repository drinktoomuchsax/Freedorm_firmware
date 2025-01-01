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
QueueHandle_t button_event_queue = NULL;

// 定义局部变量
static bool flag_long_press_start = false;
// static bool flag_ble_start_pairing = false;
static TimerHandle_t long_press_ble_timer = NULL;

// 函数声明
void ble_start_pairing(void);

// 按键事件发送函数
void send_button_event(button_event_t event)
{
    if (button_event_queue != NULL)
    {
        xQueueSend(button_event_queue, &event, 0);
    }
}

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
    button_attach(&btn1, NONE_PRESS, BTN1_NONE_PRESS_HOLD_Handler);
    button_attach(&btn1, PRESS_DOWN, BTN1_PRESS_DOWN_Handler);
    button_attach(&btn1, PRESS_UP, BTN1_PRESS_UP_Handler);

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

// 返回多次点击的次数
uint16_t button_get_multi_click_count()
{
    return btn1.repeat;
}

void BTN1_PRESS_REPEAT_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Press repeat detected, repeat count: %d", btn1.repeat);
    if (btn1.repeat == 10)
    {
        send_button_event(BUTTON_EVENT_MULTI_CLICK); // 10 次点击，发送锁门信号
    }
}

// 单次开门
void BTN1_SINGLE_CLICK_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Single click detected");
    send_button_event(BUTTON_EVENT_SINGLE_CLICK);
}

// 双击进入常开模式
void BTN1_DOUBLE_CLICK_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Double click detected");
    send_button_event(BUTTON_EVENT_DOUBLE_CLICK);
}

void BTN1_LONG_PRESS_START_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Long press start detected");
    long_press_ble_timer = xTimerCreate("long_press_ble_timer", 6000 / portTICK_PERIOD_MS, pdFALSE, NULL, (TimerCallbackFunction_t)ble_start_pairing);
    xTimerStart(long_press_ble_timer, 0);
    ws2812b_switch_effect(LED_EFFECT_BLE_TRY_PAIRING);
    flag_long_press_start = true;
}

void BTN1_LONG_PRESS_HOLD_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Long press hold detected");
}

void BTN1_NONE_PRESS_HOLD_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "None press hold detected");
}

void BTN1_PRESS_DOWN_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Press down detected");
}

void BTN1_PRESS_UP_Handler(void *btn)
{
    ESP_LOGI(BUTTON_TAG, "Press up detected");

    if (flag_long_press_start) // 长按之后的抬起就是长按结束
    {
        ESP_LOGI(BUTTON_TAG, "Long press end detected");
        xTimerStop(long_press_ble_timer, 0);
        xTimerDelete(long_press_ble_timer, 0);
        ws2812b_switch_effect(LED_EFFECT_DEFAULT_STATE);
        flag_long_press_start = false;
    }
}

void ble_start_pairing(void)
{
    xTimerStop(long_press_ble_timer, 0);
    xTimerDelete(long_press_ble_timer, 0);

    flag_long_press_start = false; // 清楚长按标志，防止放手之后去到默认灯效
    xSemaphoreGive(pairing_semaphore);
    ws2812b_switch_effect(LED_EFFECT_BLE_PAIRING_MODE);
}