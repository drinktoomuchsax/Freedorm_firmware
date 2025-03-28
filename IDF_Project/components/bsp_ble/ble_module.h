#ifndef BLE_MODULE_H
#define BLE_MODULE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_mac.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#define MAX_WHITELIST_SIZE 10

#define MAX_CONNECTIONS 4

#define RSSI_SAMPLE_COUNT_PER_SEC 4                    // 每秒采样RSSI次数，使用偶数
#define RSSI_SAMPLE_COUNT RSSI_SAMPLE_COUNT_PER_SEC * 3 // 保留最近5秒的RSSI数据
#define RSSI_OFFSET 10                                 // RSSI 上下浮动偏移量，用于计算 RSSI 阈值

extern SemaphoreHandle_t pairing_semaphore;

typedef struct
{
    uint8_t num_of_devices;
    esp_bd_addr_t bd_addr[MAX_WHITELIST_SIZE];
    esp_ble_addr_type_t bd_addr_type[MAX_WHITELIST_SIZE];
} freedorm_ble_whitelist_t;


typedef struct
{
    esp_bd_addr_t remote_bda; // 设备地址
    uint16_t conn_id;         // 连接 ID
    bool is_ble_sec_conn;     // 是否是 BLE 安全连接
} rssi_monitor_params_t;

typedef struct
{
    TaskHandle_t task_handle; // 监控任务句柄
    esp_bd_addr_t remote_bda; // 设备地址

    int8_t smoothed_rssi;                     // 平滑的 RSSI 值
    float distance_history[RSSI_SAMPLE_COUNT];    // RSSI 历史数据
    uint16_t rssi_index;                      // RSSI 历史索引
    uint8_t dude_rssi_index;                  // 滑动窗口索引
    uint8_t rssi_count;                       // 滑动窗口中的值数量
    float distance;                           // 估算的距离, 单位米
    bool is_approaching;                      // 是否在靠近门锁
} rssi_task_info_t;

static rssi_task_info_t rssi_task_list[MAX_CONNECTIONS] = {0};

void ble_module_init(void);
static esp_err_t load_freedorm_whitelist_from_nvs(freedorm_ble_whitelist_t *list);
static esp_err_t save_freedorm_whitelist_to_nvs(freedorm_ble_whitelist_t *list);
static esp_err_t delete_freedorm_whitelist_from_nvs(void);
static esp_err_t add_device_to_freedorm_whitelist(freedorm_ble_whitelist_t *whitelist, esp_bd_addr_t addr, esp_ble_addr_type_t addr_type);

#endif // BLE_MODULE_H
