#ifndef BUTTON_H
#define BUTTON_H

#include "button.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "multi_button.h"
#include "esp_mac.h"

#define PAIRING_BUTTON_GPIO GPIO_NUM_0 // 修改为您的按键GPIO编号

// 定义按键事件枚举
typedef enum
{
    BUTTON_EVENT_SINGLE_CLICK,
    BUTTON_EVENT_DOUBLE_CLICK,
    BUTTON_EVENT_MULTI_CLICK,
    BUTTON_EVENT_LONG_PRESS_START,
    BUTTON_EVENT_LONG_PRESS_HOLD,
    BUTTON_EVENT_PRESS_REPEAT,
    BUTTON_EVENT_PRESS_DOWN,
    BUTTON_EVENT_PRESS_UP,
    BUTTON_EVENT_NONE_UPDATE_LOCK_CONTROL // 没有按键事件，用来更新lock_control状态机
} button_event_t;

extern uint32_t led_state_mask; // 在这里初始化，位图，记录每个 GPIO 的当前状态

// 函数声明

uint8_t read_button_GPIO(uint8_t button_id);
void freedorm_button_init();
void button_task(void *arg);
uint16_t button_get_multi_click_count(void);
void send_button_event(button_event_t event);

void BTN1_PRESS_REPEAT_Handler(void *btn);
void BTN1_SINGLE_CLICK_Handler(void *btn);
void BTN1_DOUBLE_CLICK_Handler(void *btn);
void BTN1_LONG_PRESS_START_Handler(void *btn);
void BTN1_LONG_PRESS_HOLD_Handler(void *btn);
void BTN1_NONE_PRESS_HOLD_Handler(void *btn);
void BTN1_PRESS_DOWN_Handler(void *btn);
void BTN1_PRESS_UP_Handler(void *btn);

#endif
