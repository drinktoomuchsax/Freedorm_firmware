/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

#include "esp_task_wdt.h"
#include "multi_button.h"
#include "button.h"

#include "ble_module.h"

/**
 * BRIEF:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

/**
 * NOTE: 蓝牙连接策略，维护两个白名单，一个是ble模块的白名单，一个是gap的广播白名单。ble模块白名单记录所有连接过的设备，gap广播白名单记录当前允许连接的设备。通过滚动更新gap广播白名单，实现多设备连接。
 * 每次连接任务只有一个-获取RSSI值，RSSI超过门限值则开门。
 *
 *  DONE: 0.能不能绑定成功 (2025-01-07)
 *  TODO: 1.能不能存下来白名单
 *  TODO: 2.能不能快速重连，不用每次都配对
 *  TODO: 3.白名单策略是否可以正常工作
 *  DONE: 4.能不能在连接时读取RSSI值，不用每次都读取 (2025-01-06)
 *  DONE: 5.创建两个特性，一个是Wi-Fi SSID，一个是Wi-Fi 密码，用于传输Wi-Fi信息 (2025-01-06)
 */

#define BLE_TAG "FREEDORM_BLE"
#define BLE_GAP_TAG "FREEDORM_BLE_GAP"
#define BLE_GATT_TAG "FREEDORM_BLE_GATT"
#define BLE_WHITELIST_TAG "FREEDORM_BLE_WHITELIST"

static uint16_t hid_conn_id = 0;
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

// UUID 定义
#define SERVICE_UUID 0xFF69
#define CHAR_UUID_WIFI_SSID 0xFF70 // Wi-Fi SSID 特性 UUID
#define CHAR_UUID_WIFI_PASS 0xFF71 // Wi-Fi 密码特性 UUID

#define GATTS_NUM_HANDLE 8
#define CHARACTERISTIC_VAL_LEN 512
#define MAX_WHITELIST_SIZE 10
#define FREEDORM_DEVICE_NAME "Freedorm Pro (SB76)"

static freedorm_ble_whitelist_t whitelist = {.num_of_devices = 0};
static bool pairing_mode = false;

static uint32_t wifi_ssid[CHARACTERISTIC_VAL_LEN] = {0}; // 存储 Wi-Fi SSID，小程序传过来的是unicode编码
static uint32_t wifi_pass[CHARACTERISTIC_VAL_LEN] = {0}; // 存储 Wi-Fi 密码，小程序传过来的是unicode编码

static bool is_ble_sec_conn = false;
static esp_bd_addr_t last_connected_bda = {0};    // 已连接蓝牙设备地址，用于存储连接设备的地址
static esp_ble_addr_type_t last_con_bda_type = 0; // 已连接蓝牙设备地址类型，用于存储连接设备的地址
static int8_t rssi_threshold = -60;               // RSSI 阈值，超过这个值则开门
SemaphoreHandle_t pairing_semaphore = NULL;

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

// GATT 服务结构体
static struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle_ssid;
    uint16_t char_handle_pass;
    esp_bt_uuid_t char_uuid_ssid;
    esp_bt_uuid_t char_uuid_pass;
} gl_profile = {
    .gatts_cb = NULL,
    .gatts_if = ESP_GATT_IF_NONE,
};

static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0x12,
    0x18,
    0x00,
    0x00,
};

static esp_ble_adv_data_t freedorm_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,   // HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t freedorm_pairing_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// 快速重连RSSI广播参数
static esp_ble_adv_params_t freedorm_fast_recon_rssi_adv_params = {

    .adv_int_min = 0x20,                                     // 广播间隔短，方便设备快速发现
    .adv_int_max = 0x30,                                     // 广播间隔短，方便设备快速发现
    .adv_type = ADV_TYPE_DIRECT_IND_HIGH,                    // 高速直接广播，快速重连
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,                   // 和上面的广播参数一样，不然对方设备无法连接
    .peer_addr = {0},                                        // 定向广播时，需要对方地址，之后再根据白名单配置
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,                  // 定向广播时，需要对方地址，之后再根据白名单配置
    .channel_map = ADV_CHNL_ALL,                             // 所有通道都广播
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST, // 允许白名单设备扫描，允许任何设备连接
};

/* 加载白名单 */
static esp_err_t load_freedorm_whitelist_from_nvs(freedorm_ble_whitelist_t *list)
{
    ESP_LOGI(BLE_WHITELIST_TAG, "Loading Freedorm whitelist from NVS...");
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(BLE_WHITELIST_TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    size_t required_size = sizeof(freedorm_ble_whitelist_t);
    err = nvs_get_blob(nvs_handle, "whitelist", list, &required_size);
    nvs_close(nvs_handle);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW(BLE_WHITELIST_TAG, "Freedorm whitelist not found in NVS. Initializing empty Freedorm whitelist.");
        list->num_of_devices = 0;
        memset(list->bd_addr, 0, sizeof(list->bd_addr));
        memset(list->bd_addr_type, 0, sizeof(list->bd_addr_type));
        return ESP_OK;
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE(BLE_WHITELIST_TAG, "Failed to load Freedorm whitelist from NVS: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(BLE_WHITELIST_TAG, "Freedorm whitelist loaded successfully. Devices: %d", list->num_of_devices);
    }
    return err;
}

/* 保存白名单 */
static esp_err_t save_freedorm_whitelist_to_nvs(freedorm_ble_whitelist_t *list)
{
    ESP_LOGI(BLE_WHITELIST_TAG, "Saving Freedorm whitelist to NVS...");
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(BLE_WHITELIST_TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(nvs_handle, "whitelist", list, sizeof(freedorm_ble_whitelist_t));
    if (err != ESP_OK)
    {
        ESP_LOGE(BLE_WHITELIST_TAG, "Failed to write Freedorm whitelist to NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err == ESP_OK)
    {
        ESP_LOGI(BLE_WHITELIST_TAG, "Freedorm whitelist saved successfully.");
    }
    else
    {
        ESP_LOGE(BLE_WHITELIST_TAG, "Failed to commit Freedorm whitelist to NVS: %s", esp_err_to_name(err));
    }
    return err;
}

/* 删除白名单 */
static esp_err_t delete_freedorm_whitelist_from_nvs()
{
    ESP_LOGI(BLE_WHITELIST_TAG, "Deleting Freedorm whitelist from NVS...");
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(BLE_WHITELIST_TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_erase_key(nvs_handle, "whitelist");
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW(BLE_WHITELIST_TAG, "Freedorm whitelist not found in NVS, nothing to delete.");
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE(BLE_WHITELIST_TAG, "Failed to erase Freedorm whitelist from NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err == ESP_OK)
    {
        ESP_LOGI(BLE_WHITELIST_TAG, "Freedorm whitelist deleted successfully.");
    }
    else
    {
        ESP_LOGE(BLE_WHITELIST_TAG, "Failed to commit NVS changes: %s", esp_err_to_name(err));
    }
    return err;
}

/* 添加设备到白名单 */
static esp_err_t add_device_to_freedorm_whitelist(freedorm_ble_whitelist_t *whitelist, esp_bd_addr_t addr, esp_ble_addr_type_t addr_type)
{
    ESP_LOGI(BLE_WHITELIST_TAG, "Adding device to Freedorm whitelist...");

    // 检查是否已达到白名单最大值
    if (whitelist->num_of_devices >= MAX_WHITELIST_SIZE)
    {
        ESP_LOGW(BLE_WHITELIST_TAG, "Freedorm whitelist is full. Cannot add more devices.");
        return ESP_ERR_NO_MEM;
    }

    // 检查设备是否已存在（防止重复添加）
    for (uint8_t i = 0; i < whitelist->num_of_devices; i++)
    {
        if (memcmp(whitelist->bd_addr[i], addr, sizeof(esp_bd_addr_t)) == 0)
        {
            ESP_LOGW(BLE_WHITELIST_TAG, "Device is already in the Freedorm whitelist.");
            return ESP_OK; // 已存在，直接返回
        }
    }

    // 添加设备
    uint8_t index = whitelist->num_of_devices;
    memcpy(whitelist->bd_addr[index], addr, sizeof(esp_bd_addr_t));
    whitelist->bd_addr_type[index] = addr_type;
    whitelist->num_of_devices++;

    ESP_LOGI(BLE_WHITELIST_TAG, "Device added successfully into Freedorm whitelist. Total devices: %d", whitelist->num_of_devices);

    // 保存到 NVS
    return save_freedorm_whitelist_to_nvs(whitelist);
}

/* 从白名单中移除设备 */
static esp_err_t remove_device_from_freedorm_whitelist(freedorm_ble_whitelist_t *whitelist, esp_bd_addr_t addr)
{
    ESP_LOGI(BLE_WHITELIST_TAG, "Removing device from Freedorm whitelist...");

    bool found = false;

    // 遍历白名单查找设备
    for (uint8_t i = 0; i < whitelist->num_of_devices; i++)
    {
        if (memcmp(whitelist->bd_addr[i], addr, sizeof(esp_bd_addr_t)) == 0)
        {
            found = true;

            // 将后面的设备前移覆盖删除的设备
            for (uint8_t j = i; j < whitelist->num_of_devices - 1; j++)
            {
                memcpy(whitelist->bd_addr[j], whitelist->bd_addr[j + 1], sizeof(esp_bd_addr_t));
                whitelist->bd_addr_type[j] = whitelist->bd_addr_type[j + 1];
            }

            // 清除最后一个设备的数据
            memset(whitelist->bd_addr[whitelist->num_of_devices - 1], 0, sizeof(esp_bd_addr_t));
            whitelist->bd_addr_type[whitelist->num_of_devices - 1] = 0;

            whitelist->num_of_devices--;
            ESP_LOGI(BLE_WHITELIST_TAG, "Device removed successfully form Freedorm whitelist. Total devices: %d", whitelist->num_of_devices);
            break;
        }
    }

    if (!found)
    {
        ESP_LOGW(BLE_WHITELIST_TAG, "Device not found in the Freedorm whitelist.");
        return ESP_ERR_NOT_FOUND;
    }

    // 保存到 NVS
    return save_freedorm_whitelist_to_nvs(whitelist);
}

/* 打印白名单 */
static void print_freedorm_whitelist(freedorm_ble_whitelist_t *whitelist)
{
    ESP_LOGI(BLE_WHITELIST_TAG, "Printing Freedorm whitelist. Total devices: %d", whitelist->num_of_devices);

    for (uint8_t i = 0; i < whitelist->num_of_devices; i++)
    {
        printf("Device %d: Address = ", i + 1);
        for (int j = 0; j < 6; j++)
        {
            printf("%02X", whitelist->bd_addr[i][j]);
            if (j < 5)
                printf(":");
        }
        printf(", Addr Type = %d\n", whitelist->bd_addr_type[i]);
    }
}

// 添加设备到 GAP 白名单
static void add_device_to_gap_whitelist(esp_bd_addr_t bd_addr, esp_ble_addr_type_t addr_type)
{
    esp_err_t err = esp_ble_gap_update_whitelist(ESP_BLE_WHITELIST_ADD, bd_addr, addr_type);
    if (err == ESP_OK)
    {
        ESP_LOGI(BLE_WHITELIST_TAG, "Device added to GAP whitelist: %02X:%02X:%02X:%02X:%02X:%02X",
                 bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);
    }
    else
    {
        ESP_LOGE(BLE_WHITELIST_TAG, "Failed to add device to GAP whitelist: %s", esp_err_to_name(err));
    }
}

static void print_gap_whitelist_size()
{
    uint8_t num_devices = 0;

    // 获取白名单中的设备数量
    esp_err_t err = esp_ble_gap_get_whitelist_size(&num_devices);
    if (err != ESP_OK)
    {
        ESP_LOGE(BLE_WHITELIST_TAG, "Failed to get GAP whitelist size: %s", esp_err_to_name(err));
        return;
    }
    else
    {
        ESP_LOGI(BLE_WHITELIST_TAG, "GAP whitelist size: %d", num_devices);
    }

    if (num_devices == 0)
    {
        ESP_LOGW(BLE_WHITELIST_TAG, "GAP whitelist is empty.");
        return;
    }
}

/* 清空白名单 */
static esp_err_t clear_whitelist(freedorm_ble_whitelist_t *whitelist)
{
    ESP_LOGI(BLE_WHITELIST_TAG, "Clearing whitelist...");
    whitelist->num_of_devices = 0;
    memset(whitelist->bd_addr, 0, sizeof(whitelist->bd_addr));
    memset(whitelist->bd_addr_type, 0, sizeof(whitelist->bd_addr_type));

    // 从 NVS 删除白名单
    return delete_freedorm_whitelist_from_nvs();
}

static bool is_last_connected_bda_valid()
{
    // 如果数组的内容全为 0，则认为地址无效
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
    {
        if (last_connected_bda[i] != 0)
        {
            return true; // 有效地址
        }
    }
    return false; // 地址无效
}

// 在需要时主动断开连接
static void disconnect_from_central()
{
    if (is_last_connected_bda_valid())
    { // 确保有有效的连接
        esp_err_t err = esp_ble_gap_disconnect(last_connected_bda);
        if (err == ESP_OK)
        {
            ESP_LOGI("BLE", "Disconnected from Central device successfully.");
        }
        else
        {
            ESP_LOGE("BLE", "Failed to disconnect (err: %s)", esp_err_to_name(err));
        }
    }
    else
    {
        ESP_LOGW("BLE", "No active connection to disconnect.");
    }
}

static void start_directed_advertising()
{
    if ((last_con_bda_type != 0) && is_last_connected_bda_valid())
    {
        ESP_LOGI(BLE_GAP_TAG, "Starting directed advertising for quick reconnect...");

        // 动态设置目标设备地址
        memcpy(freedorm_fast_recon_rssi_adv_params.peer_addr, last_connected_bda, sizeof(esp_bd_addr_t));
        freedorm_fast_recon_rssi_adv_params.peer_addr_type = last_con_bda_type;

        ESP_LOGI(BLE_GAP_TAG, "Directed advertising target: %02X:%02X:%02X:%02X:%02X:%02X",
                 freedorm_fast_recon_rssi_adv_params.peer_addr[0], freedorm_fast_recon_rssi_adv_params.peer_addr[1],
                 freedorm_fast_recon_rssi_adv_params.peer_addr[2], freedorm_fast_recon_rssi_adv_params.peer_addr[3],
                 freedorm_fast_recon_rssi_adv_params.peer_addr[4], freedorm_fast_recon_rssi_adv_params.peer_addr[5]);
        ESP_LOGI(BLE_GAP_TAG, "Directed advertising target type: %d", freedorm_fast_recon_rssi_adv_params.peer_addr_type);

        // 启动定向广播
        esp_err_t err = esp_ble_gap_start_advertising(&freedorm_fast_recon_rssi_adv_params);
        if (err == ESP_OK)
        {
            ESP_LOGI(BLE_GAP_TAG, "Directed advertising started successfully.");
        }
        else
        {
            ESP_LOGE(BLE_GAP_TAG, "Failed to start directed advertising: %s", esp_err_to_name(err));
        }
    }
    else
    {
        ESP_LOGW(BLE_GAP_TAG, "No connected device info. Directed advertising not started.");
    }
}

// 工具函数：将 UTF-32 转换为 UTF-8
static void utf32_to_utf8(const uint32_t *utf32, uint8_t *utf8, size_t utf8_len)
{
    size_t utf8_index = 0;
    for (size_t i = 0; utf32[i] != 0x0000; i++)
    { // 遍历 UTF-32 数据
        if (utf32[i] < 0x80)
        { // 单字节 UTF-8
            if (utf8_index < utf8_len - 1)
                utf8[utf8_index++] = (uint8_t)utf32[i];
        }
        else if (utf32[i] < 0x800)
        { // 双字节 UTF-8
            if (utf8_index < utf8_len - 2)
            {
                utf8[utf8_index++] = (uint8_t)(0xC0 | ((utf32[i] >> 6) & 0x1F));
                utf8[utf8_index++] = (uint8_t)(0x80 | (utf32[i] & 0x3F));
            }
        }
        else if (utf32[i] < 0x10000)
        { // 三字节 UTF-8
            if (utf8_index < utf8_len - 3)
            {
                utf8[utf8_index++] = (uint8_t)(0xE0 | ((utf32[i] >> 12) & 0x0F));
                utf8[utf8_index++] = (uint8_t)(0x80 | ((utf32[i] >> 6) & 0x3F));
                utf8[utf8_index++] = (uint8_t)(0x80 | (utf32[i] & 0x3F));
            }
        }
        else
        { // 四字节 UTF-8
            if (utf8_index < utf8_len - 4)
            {
                utf8[utf8_index++] = (uint8_t)(0xF0 | ((utf32[i] >> 18) & 0x07));
                utf8[utf8_index++] = (uint8_t)(0x80 | ((utf32[i] >> 12) & 0x3F));
                utf8[utf8_index++] = (uint8_t)(0x80 | ((utf32[i] >> 6) & 0x3F));
                utf8[utf8_index++] = (uint8_t)(0x80 | (utf32[i] & 0x3F));
            }
        }
    }
    utf8[utf8_index] = '\0'; // 添加字符串结束符
}

// 使用 UTF-32 转换并打印
static void print_utf32_as_utf8(const uint32_t *utf32)
{
    uint8_t utf8_buffer[128]; // 足够大的缓冲区
    utf32_to_utf8(utf32, utf8_buffer, sizeof(utf8_buffer));
    ESP_LOGI(BLE_TAG, "UTF-32 as UTF-8: %s", utf8_buffer);
}

static void log_bonded_dev(void)
{
    int bond_dev_num = esp_ble_get_bond_device_num();
    esp_ble_bond_dev_t bond_dev_list[bond_dev_num];
    esp_ble_get_bond_device_list(&bond_dev_num, bond_dev_list);
    ESP_LOGI(BLE_TAG, "Bonded devices: %d", bond_dev_num);
    for (int i = 0; i < bond_dev_num; i++)
    {
        ESP_LOGI(BLE_TAG, "Bonded device %d: %02x:%02x:%02x:%02x:%02x:%02x",
                 i,
                 bond_dev_list[i].bd_addr[0], bond_dev_list[i].bd_addr[1], bond_dev_list[i].bd_addr[2],
                 bond_dev_list[i].bd_addr[3], bond_dev_list[i].bd_addr[4], bond_dev_list[i].bd_addr[5]);
    }

    for (int i = 0; i < bond_dev_num; i++)
    {
        esp_ble_remove_bond_device(bond_dev_list[i].bd_addr);
    }
}

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event)
    {
    case ESP_HIDD_EVENT_REG_FINISH:
    {
        if (param->init_finish.state == ESP_HIDD_INIT_OK)
        {
            // esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
            esp_ble_gap_set_device_name(FREEDORM_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&freedorm_adv_data);
        }
        break;
    }
    case ESP_BAT_EVENT_REG:
    {
        break;
    }
    case ESP_HIDD_EVENT_DEINIT_FINISH:
        break;
    case ESP_HIDD_EVENT_BLE_CONNECT:
    {
        ESP_LOGI(BLE_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
        hid_conn_id = param->connect.conn_id;
        break;
    }
    case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT:
    {
        ESP_LOGI(BLE_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
        ESP_LOG_BUFFER_HEX(BLE_TAG, param->vendor_write.data, param->vendor_write.length);
        break;
    }
    case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT:
    {
        ESP_LOGI(BLE_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
        ESP_LOG_BUFFER_HEX(BLE_TAG, param->led_write.data, param->led_write.length);
        break;
    }
    default:
        break;
    }
    return;
}

static void read_rssi_value(void *pvParameters)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while (1)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        if (1)
        {
            esp_ble_gap_read_rssi(last_connected_bda);
        }
    }
}

static void unlock_if_rssi_valid(int8_t rssi)
{
    if (rssi > rssi_threshold)
    {
        // ESP_LOGI(BLE_TAG, "RSSI value is valid, unlocking door.");
        // 在此执行开门操作
    }
    else
    {
        // ESP_LOGI(BLE_TAG, "RSSI value is invalid, not unlocking door.");
    }
}

/* 配对模式任务 */
static void pairing_mode_task(void *arg)
{
    while (1)
    {
        if (xSemaphoreTake(pairing_semaphore, portMAX_DELAY) == pdTRUE)
        {
            // 进入配对模式
            pairing_mode = true;
            ESP_LOGI(BLE_TAG, "Entering pairing mode.");

            // 设置广播参数允许所有设备连接
            freedorm_pairing_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
            esp_ble_gap_stop_advertising();
            esp_ble_gap_start_advertising(&freedorm_pairing_adv_params);

            // 配对模式持续60秒
            vTaskDelay(60000 / portTICK_PERIOD_MS);

            // 退出配对模式
            pairing_mode = false;
            ESP_LOGI(BLE_TAG, "Exiting pairing mode.");

            // 恢复广播参数为只允许白名单设备连接
            freedorm_pairing_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST;
            esp_ble_gap_stop_advertising();
            esp_ble_gap_start_advertising(&freedorm_pairing_adv_params);
        }
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&freedorm_pairing_adv_params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
        {
            ESP_LOGD(BLE_GAP_TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: // 配对之后触发的事件，在这里储存白名单
        is_ble_sec_conn = true;
        memcpy(last_connected_bda, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        add_device_to_gap_whitelist(last_connected_bda, last_con_bda_type);
        last_con_bda_type = param->ble_security.auth_cmpl.addr_type;

        ESP_LOGI(BLE_GAP_TAG, "remote BD_ADDR: %08x%04x",
                 (last_connected_bda[0] << 24) + (last_connected_bda[1] << 16) + (last_connected_bda[2] << 8) + last_connected_bda[3],
                 (last_connected_bda[4] << 8) + last_connected_bda[5]);
        ESP_LOGI(BLE_GAP_TAG, "address type = %d", last_con_bda_type);
        ESP_LOGI(BLE_GAP_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success)
        {
            ESP_LOGE(BLE_GAP_TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }

        // 添加进入白名单
        add_device_to_freedorm_whitelist(&whitelist, last_connected_bda, last_con_bda_type);
        print_freedorm_whitelist(&whitelist);
        print_gap_whitelist_size();

        break;
    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT:
        ESP_LOGI(BLE_GAP_TAG, "ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT, rssi %d", param->read_rssi_cmpl.rssi);
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(BLE_GAP_TAG, "Advertising start failed: %s", esp_err_to_name(param->adv_start_cmpl.status));
        }
        else
        {
            ESP_LOGI(BLE_GAP_TAG, "Advertising start successfully.");
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(BLE_GAP_TAG, "Advertising stop failed: %s", esp_err_to_name(param->adv_stop_cmpl.status));
        }
        else
        {
            ESP_LOGI(BLE_GAP_TAG, "Advertising stop successfully.");
        }
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(BLE_GATT_TAG, "ESP_GATTS_REG_EVT");
        gl_profile.service_id.is_primary = true;
        gl_profile.service_id.id.inst_id = 0x00;
        gl_profile.service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile.service_id.id.uuid.uuid.uuid16 = SERVICE_UUID;

        esp_ble_gatts_create_service(gatts_if, &gl_profile.service_id, GATTS_NUM_HANDLE);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(BLE_GATT_TAG, "Service created, handle: %d", param->create.service_handle);
        gl_profile.service_handle = param->create.service_handle;

        esp_ble_gatts_start_service(gl_profile.service_handle);

        // 添加 Wi-Fi SSID 特性
        gl_profile.char_uuid_ssid.len = ESP_UUID_LEN_16;
        gl_profile.char_uuid_ssid.uuid.uuid16 = CHAR_UUID_WIFI_SSID;
        if (esp_ble_gatts_add_char(gl_profile.service_handle, &gl_profile.char_uuid_ssid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL))
        {
            ESP_LOGI(BLE_GATT_TAG, "Failed to add WIFI SSID characteristic");
        }

        // 添加 Wi-Fi 密码特性
        gl_profile.char_uuid_pass.len = ESP_UUID_LEN_16;
        gl_profile.char_uuid_pass.uuid.uuid16 = CHAR_UUID_WIFI_PASS;
        if (esp_ble_gatts_add_char(gl_profile.service_handle, &gl_profile.char_uuid_pass,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL))
        {
            ESP_LOGE(BLE_GATT_TAG, "Failed to add WIFI PASS characteristic");
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.status != ESP_GATT_OK)
        {
            ESP_LOGE(BLE_GATT_TAG, "Failed to add characteristic, status: %d", param->add_char.status);
        }
        ESP_LOGI(BLE_GATT_TAG, "Characteristic added, attr_handle: %d", param->add_char.attr_handle);
        if (param->add_char.char_uuid.uuid.uuid16 == CHAR_UUID_WIFI_SSID)
        {
            gl_profile.char_handle_ssid = param->add_char.attr_handle;
        }
        else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_UUID_WIFI_PASS)
        {
            gl_profile.char_handle_pass = param->add_char.attr_handle;
        }
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(BLE_GATT_TAG, "Write event received, handle: %d, value len: %d", param->write.handle, param->write.len);
        ESP_LOG_BUFFER_HEX(BLE_TAG, param->write.value, param->write.len);
        if (param->write.handle == gl_profile.char_handle_ssid)
        {
            memcpy(wifi_ssid, param->write.value, param->write.len);
            wifi_ssid[param->write.len] = '\0'; // 确保字符串以 '\0' 结尾
            // ESP_LOGI(BLE_GATT_TAG, "Received Wi-Fi SSID: %s", wifi_ssid);
            print_utf32_as_utf8(wifi_ssid);
        }
        else if (param->write.handle == gl_profile.char_handle_pass)
        {
            memcpy(wifi_pass, param->write.value, param->write.len);
            wifi_pass[param->write.len] = '\0'; // 确保字符串以 '\0' 结尾
            // ESP_LOGI(BLE_GATT_TAG, "Received Wi-Fi Password: %s", wifi_pass);
            print_utf32_as_utf8(wifi_pass);
        }

        if (param->write.need_rsp)
        {
            // 主机发起的是 Write Request，需要发送响应
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->write.handle;                        // 属性句柄
            rsp.attr_value.len = param->write.len;                              // 响应数据长度
            memcpy(rsp.attr_value.value, param->write.value, param->write.len); // 响应数据

            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_OK, &rsp);

            ESP_LOGI(BLE_TAG, "Write Command received, response sent");
        }
        else
        {
            // 主机发起的是 Write Command，无需响应
            ESP_LOGI(BLE_TAG, "Write Command received, no response needed");
        }
        break;

    case ESP_GATTS_CONNECT_EVT: // 设备连接之后触发的事件，在配对之前
    {
        ESP_LOGI(BLE_GATT_TAG, "Device connected");
        // 打印连接设备的所有信息
        ESP_LOGI(BLE_GATT_TAG, "Connection ID: %d", param->connect.conn_id);
        ESP_LOGI(BLE_GATT_TAG, "Link role: %s", param->connect.link_role == 0 ? "master" : "slave");
        ESP_LOGI(BLE_GATT_TAG, "Remote device address: %02x:%02x:%02x:%02x:%02x:%02x",
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        ESP_LOGI(BLE_GATT_TAG, "Connection handle: %d", param->connect.conn_handle);
        ESP_LOGI(BLE_GATT_TAG, "BLE address type: %d", param->connect.ble_addr_type);

        // 保存连接设备的地址
        memcpy(last_connected_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

        is_ble_sec_conn = true;
    }
    break;

    case ESP_GATTS_DISCONNECT_EVT: // 设备断开连接
        is_ble_sec_conn = false;
        ESP_LOGI(BLE_GATT_TAG, "Device disconnected, restarting advertising...");

        vTaskDelay(3000 / portTICK_PERIOD_MS);
        start_directed_advertising();
        // esp_ble_gap_start_advertising(&freedorm_pairing_adv_params);
        break;

    default:
        break;
    }
}

void ble_module_init(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "%s enable controller failed", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    if ((ret = esp_hidd_profile_init()) != ESP_OK)
    {
        ESP_LOGE(BLE_TAG, "%s init bluedroid failed", __func__);
    }

    /// register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gatts_app_register(0);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND; // bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;       // set the IO capability to No output No input
    uint8_t key_size = 16;                          // the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    pairing_semaphore = xSemaphoreCreateBinary();
    xTaskCreate(&pairing_mode_task, "pairing_mode_task", 2048, NULL, 5, NULL);

    xTaskCreate(&read_rssi_value, "rssi_task", 2048, NULL, 5, NULL);
}
