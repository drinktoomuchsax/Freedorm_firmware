#ifndef LOCK_CONTROL_H
#define LOCK_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"

#define CTL_LOCK GPIO_NUM_6
#define CTL_D0 GPIO_NUM_3
#define OUTPUT_LED_D4 GPIO_NUM_12
#define OUTPUT_LED_D5 GPIO_NUM_13

#define TIME_RECOVER_TEMP_OPEN 10 * 60 * 1000      // 定义超时时间 (ms)
#define TIME_BLE_RECOVER_TEMP_OPEN 0.5 * 60 * 1000 // 定义超时时间 (ms)
#define TIME_RECOVER_LOCK 10 * 60 * 1000           // 定义超时时间 (ms)

typedef enum
{
    STATE_POWER_ON_BLACK = 0,
    STATE_NORAML_DEFAULT, // 门正常状态，未锁定，未打开，使用校园卡开门
    STATE_TEMP_OPEN,      // 门展示打开状态，直接推开门，持续“TIME_RECOVER_TEMP_OPEN”秒
    STATE_ALWAYS_OPEN,    // 门打开状态，直接推开门
    STATE_LOCKED,         // 门锁定状态，任何卡都刷不开
    STATE_TEMP_OPEN_END,
    STATE_LOCK_END,
    STATE_BLE_TEMP_OPEN,
    STATE_BLE_TEMP_OPEN_END,
} lock_status_t;

// 定义门锁控制命令类型
typedef enum
{
    LOCK_CMO_NORMAL,
    LOCK_CMD_SINGLE_OPEN, // 开门
    LOCK_CMD_LOCK,        // 关门
    LOCK_CMD_ALWAYS_OPEN  // 常开模式
} lock_command_t;

extern QueueHandle_t button_event_queue;

// 门锁模块相关函数
void lock_control_init(void);
void lock_control_task(void *pvParameters);
void transition_to_state(lock_status_t new_state);

#endif // LOCK_CONTROL_H