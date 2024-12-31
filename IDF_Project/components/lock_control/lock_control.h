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

typedef enum
{
    LOCK_STATE_NORMAL = 0, // 门正常状态，未锁定，未打开，使用校园卡开门
    LOCK_STATE_OPEN,       // 门打开状态，直接推开门
    LOCK_STATE_LOCKED,     // 门锁定状态，任何卡都刷不开
} lock_status_t;

// 定义门锁控制命令类型
typedef enum
{
    LOCK_CMO_NORMAL,
    LOCK_CMD_SINGLE_OPEN, // 开门
    LOCK_CMD_LOCK,        // 关门
    LOCK_CMD_ALWAYS_OPEN  // 常开模式
} lock_command_t;

// 门锁模块相关函数
void lock_control_init(void);
void lock_control_task(void *pvParameters);
void lock_send_command(lock_command_t cmd);
lock_status_t lock_get_status(void);

void lock_set_lock(void);
void lock_set_open(void);
void lock_set_normal(void);

void single_click_toogle(void);
void double_click_always_open(void);

#endif // LOCK_CONTROL_H