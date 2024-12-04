#ifndef BUTTON_H
#define BUTTON_H

#include <stdlib.h>

#define PAIRING_BUTTON_GPIO GPIO_NUM_0 // 修改为您的按键GPIO编号
#define OUTPUT_LED_D4 GPIO_NUM_12
#define OUTPUT_LED_D5 GPIO_NUM_13
#define LED_TAG "LED"

static uint32_t led_state_mask = 0; // 位图，记录每个 GPIO 的当前状态

void flip_led(int gpio_num);
uint8_t read_button_GPIO(uint8_t button_id);
void button_task(void *arg);
void BTN1_SINGLE_CLICK_Handler(void *btn);
void BTN1_DOUBLE_CLICK_Handler(void *btn);
void BTN1_LONG_PRESS_START_Handler(void *btn);

#endif
