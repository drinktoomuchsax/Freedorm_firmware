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
    BUTTON_EVENT_SINGLE_CLICK,            // 单击
    BUTTON_EVENT_DOUBLE_CLICK,            // 双击
    BUTTON_EVENT_MULTI_CLICK,             // 大于三次点击
    BUTTON_EVENT_LONG_PRESS_START,        // 长按开始
    BUTTON_EVENT_LONG_PRESS_HOLD_3S,      // 这里是hold 3s，加上长按开始的2s，总共按下5s后会触发
    BUTTON_EVENT_LONG_PRESS_HOLD_4S,      // 同上
    BUTTON_EVENT_LONG_PRESS_HOLD_6S,      // 同上
    BUTTON_EVENT_LONG_PRESS_END,          // 长按结束
    BUTTON_EVENT_PRESS_DOWN,              // 按下按钮
    BUTTON_EVENT_PRESS_UP,                // 释放按钮
    BLE_BUTTON_EVENT_SINGLE_CLICK,        // BLE靠近开门
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
