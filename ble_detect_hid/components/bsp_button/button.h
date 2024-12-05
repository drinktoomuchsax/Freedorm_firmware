#ifndef BUTTON_H
#define BUTTON_H

#include <stdlib.h>

#define PAIRING_BUTTON_GPIO GPIO_NUM_0 // 修改为您的按键GPIO编号
#define OUTPUT_LED_D4 GPIO_NUM_12
#define OUTPUT_LED_D5 GPIO_NUM_13
#define CTL_LOCK GPIO_NUM_6
#define CTL_D0 GPIO_NUM_3
#define BUTTON_TAG "BUTTON"

extern uint32_t led_state_mask; // 在这里初始化，位图，记录每个 GPIO 的当前状态

uint8_t read_button_GPIO(uint8_t button_id);
void button_task(void *arg);
void BTN1_SINGLE_CLICK_Handler(void *btn);
void BTN1_DOUBLE_CLICK_Handler(void *btn);
void BTN1_LONG_PRESS_START_Handler(void *btn);

#endif
