#ifndef LOCK_CONTROL_H
#define LOCK_CONTROL_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

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
    STATE_NORAML_DEFAULT,          // 门正常状态，未锁定，未打开，使用校园卡开门
    STATE_TEMP_OPEN,               // 门展示打开状态，直接推开门，持续“TIME_RECOVER_TEMP_OPEN”秒
    STATE_ALWAYS_OPEN,             // 门打开状态，直接推开门
    STATE_LOCKED,                  // 门锁定状态，任何卡都刷不开
    STATE_TEMP_OPEN_END,           // 按键单次开门结束
    STATE_LOCK_END,                // 锁门结束阶段，关掉恢复锁门的定时器
    STATE_BLE_TEMP_OPEN,           // 蓝牙靠近开门，显示灯效
    STATE_BLE_TEMP_OPEN_END,       // 蓝牙靠近开门结束，显示结束灯效，
    STATE_BLE_PAIRING_PREPARE,     // 蓝牙配对准备状态，长按进入此状态，继续长按进入配对状态
    STATE_BLE_PAIRING_IN_PROGRESS, // 蓝牙配对中状态，公共广播，等待连接
    STATE_BLE_PAIRING_TIME_OUT,    // 蓝牙配对超时, 2分钟后自动退出配对状态
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
lock_status_t get_current_lock_state();

#endif // LOCK_CONTROL_H