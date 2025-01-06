/*
 *这部分代码由ESP-IDF的BLE HID示例修改而来，github.com/espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/ble/ble_hid_device_demo
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

// #include "esp_bt_defs.h"
#include "driver/gpio.h"
#include "hid_dev.h"

#include "esp_task_wdt.h"
#include "multi_button.h"
#include "button.h"

#include "ble_module.h"

/**
 * brief: 蓝牙连接策略，维护两个白名单，一个是ble模块的白名单，一个是gap的广播白名单。ble模块白名单记录所有连接过的设备，gap广播白名单记录当前允许连接的设备。通过滚动更新gap广播白名单，实现多设备连接。
 * 每次连接任务只有一个-获取RSSI值，RSSI超过门限值则开门。
 *
 *  TODO: 1.能不能存下来白名单
 *  TODO: 2.能不能快速重连，不用每次都配对
 *  TODO: 3.白名单策略是否可以正常工作
 *  TODO: 4.能不能在连接时读取RSSI值，不用每次都读取
 *  DONE: 5.创建两个特性，一个是Wi-Fi SSID，一个是Wi-Fi 密码，用于传输Wi-Fi信息
 */

#define BLE_TAG "FREEDORM_BLE"
#define GATT_TAG "BLE_GATT_SERVICE"

// UUID 定义
#define SERVICE_UUID 0xFF69
#define CHAR_UUID_WIFI_SSID 0xFF70 // Wi-Fi SSID 特性 UUID
#define CHAR_UUID_WIFI_PASS 0xFF71 // Wi-Fi 密码特性 UUID

#define GATTS_NUM_HANDLE 8
#define DEVICE_NAME "Freedorm Pro (AE86)"
#define CHARACTERISTIC_VAL_LEN 512

static uint8_t wifi_ssid[CHARACTERISTIC_VAL_LEN] = {0}; // 存储 Wi-Fi SSID
static uint8_t wifi_pass[CHARACTERISTIC_VAL_LEN] = {0}; // 存储 Wi-Fi 密码

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static bool send_volum_up = false;
static esp_bd_addr_t connected_bd_addr; // 用于存储连接设备的地址
SemaphoreHandle_t pairing_semaphore = NULL;

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

#define HIDD_DEVICE_NAME "Freedorm Lite (CC69)"
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

// 广播数据配置
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// 广播参数配置
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

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

static esp_ble_adv_data_t freedorm_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x0180,   // HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), // Always set the discoverable mode and BLE only
};

// 配对模式广播参数
static esp_ble_adv_params_t freedorm_pairing_adv_params = {
    // 广播间隔小，方便设备快速发现
    .adv_int_min = 0x20,                                     // 20*0.625ms
    .adv_int_max = 0x30,                                     // 30*0.625ms
    .adv_type = ADV_TYPE_IND,                                // Connectable and Scannable Undirected Advertising，一种可连接、可扫描的无方向广播，与任何主机建立连接的场景，比如配对
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,                   // 固化的公共地址，配对先暂时用这个
    .channel_map = ADV_CHNL_ALL,                             // 所有通道都广播
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_WLST_CON_ANY, // 允许任何设备扫描和连接
    // .peer_addr =                                          //非定向广播，不需要对方地址
    //.peer_addr_type       =                                //非定向广播，不需要对方地址
};

// 快速重连RSSI广播参数
static esp_ble_adv_params_t freedorm_fast_recon_rssi_adv_params = {

    .adv_int_min = 0x10,                                     // 广播间隔短，方便设备快速发现
    .adv_int_max = 0x25,                                     // 广播间隔短，方便设备快速发现
    .adv_type = ADV_TYPE_DIRECT_IND_HIGH,                    // 高速直接广播，快速重连
    .own_addr_type = BLE_ADDR_TYPE_RPA_PUBLIC,               // 基于公有地址的可分辨私有地址，兼顾隐私和快速重连，配对后使用这个
    .peer_addr = {0x20, 0x03, 0x06, 0x21, 0x69, 0x69},       // 定向广播时，需要对方地址，之后再根据白名单配置
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,                  // 定向广播时，需要对方地址，之后再根据白名单配置
    .channel_map = ADV_CHNL_ALL,                             // 所有通道都广播
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_WLST_CON_ANY, // 允许白名单设备扫描，允许任何设备连接
};

#define MAX_WHITELIST_SIZE 10

static whitelist_t whitelist;
static bool pairing_mode = false;
static esp_bd_addr_t connected_bd_addr;

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void pairing_mode_task(void *arg);
// 蓝牙传WIFI回调函数
static void dude_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* 加载白名单 */
esp_err_t load_whitelist_from_nvs(whitelist_t *list)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
        return err;

    size_t required_size = sizeof(whitelist_t);
    err = nvs_get_blob(nvs_handle, "whitelist", list, &required_size);
    nvs_close(nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        // 白名单不存在，初始化
        list->num_devices = 0;
        memset(list->devices, 0, sizeof(list->devices));
        return ESP_OK;
    }
    return err;
}

/* 保存白名单 */
esp_err_t save_whitelist_to_nvs(whitelist_t *list)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_set_blob(nvs_handle, "whitelist", list, sizeof(whitelist_t));
    if (err != ESP_OK)
    {
        nvs_close(nvs_handle);
        return err;
    }
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return err;
}

esp_err_t delete_whitelist_from_nvs()
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_erase_key(nvs_handle, "whitelist");
    if (err != ESP_OK)
    {
        nvs_close(nvs_handle);
        return err;
    }
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return err;
}

/* 配对模式任务 */
void pairing_mode_task(void *arg)
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

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event)
    {
    case ESP_HIDD_EVENT_REG_FINISH:
    {
        if (param->init_finish.state == ESP_HIDD_INIT_OK)
        {
            esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
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
    case ESP_HIDD_EVENT_BLE_DISCONNECT:
    {
        sec_conn = false;
        ESP_LOGI(BLE_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
        esp_ble_gap_start_advertising(&freedorm_pairing_adv_params);
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
            ESP_LOGD(BLE_TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        memcpy(connected_bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "Device authenticated: %08x%04x",
                 (connected_bd_addr[0] << 24) + (connected_bd_addr[1] << 16) + (connected_bd_addr[2] << 8) + connected_bd_addr[3],
                 (connected_bd_addr[4] << 8) + connected_bd_addr[5]);

        // 检查设备是否已在白名单中
        bool device_in_whitelist = false;
        for (int i = 0; i < whitelist.num_devices; i++)
        {
            if (memcmp(whitelist.devices[i], connected_bd_addr, sizeof(esp_bd_addr_t)) == 0)
            {
                device_in_whitelist = true;
                break;
            }
        }

        // 如果设备不在白名单，添加并保存
        if (!device_in_whitelist)
        {
            if (whitelist.num_devices < MAX_WHITELIST_SIZE)
            {
                memcpy(whitelist.devices[whitelist.num_devices], connected_bd_addr, sizeof(esp_bd_addr_t));
                whitelist.num_devices++;
                save_whitelist_to_nvs(&whitelist);
                esp_ble_gap_update_whitelist(true, connected_bd_addr, BLE_WL_ADDR_TYPE_PUBLIC);
                ESP_LOGI(BLE_TAG, "Device added to whitelist.");
            }
            else
            {
                ESP_LOGW(BLE_TAG, "Whitelist is full, cannot add device.");
            }
        }

        // 如果不在配对模式，设置广播参数为只允许白名单设备
        if (!pairing_mode)
        {
            freedorm_pairing_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST;
            esp_ble_gap_stop_advertising();
            esp_ble_gap_start_advertising(&freedorm_pairing_adv_params);
        }
        break;

    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT: // 处理读取 RSSI 的回调
        if (param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(BLE_TAG, "RSSI for connected device: %d", param->read_rssi_cmpl.rssi);
            // 在此根据 RSSI 值执行开门操作
        }
        else
        {
            ESP_LOGE(BLE_TAG, "Failed to read RSSI, status: %d", param->read_rssi_cmpl.status);
        }
        break;

    default:
        break;
    }
}

void hid_demo_task(void *pvParameters)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while (1)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        if (sec_conn)
        {
            ESP_LOGI(BLE_TAG, "Read RSSI value");
            send_volum_up = true;
            // uint8_t key_vaule = {HID_KEY_A};
            // esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);
            // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (send_volum_up)
            {
                esp_ble_gap_read_rssi(connected_bd_addr); // 读取 RSSI 值
                send_volum_up = false;
                // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
                // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
                // vTaskDelay(3000 / portTICK_PERIOD_MS);
                // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);

                // // send keyboard value "Hello Freedorm !"
                // uint8_t key_vaule[] = {HID_KEY_H, HID_KEY_E, HID_KEY_L, HID_KEY_L, HID_KEY_O, HID_KEY_SPACEBAR, HID_KEY_F, HID_KEY_R, HID_KEY_E, HID_KEY_E, HID_KEY_D, HID_KEY_O, HID_KEY_R, HID_KEY_M};
                // esp_hidd_send_keyboard_value(hid_conn_id, 0, key_vaule, sizeof(key_vaule));
            }
        }
    }
}

static void dude_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(BLE_TAG, "Advertisement data set complete, start advertising.");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(BLE_TAG, "Advertising started successfully");
        }
        else
        {
            ESP_LOGE(BLE_TAG, "Failed to start advertising, error code: %d", param->adv_start_cmpl.status);
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
        ESP_LOGI(BLE_TAG, "ESP_GATTS_REG_EVT");
        gl_profile.service_id.is_primary = true;
        gl_profile.service_id.id.inst_id = 0x00;
        gl_profile.service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile.service_id.id.uuid.uuid.uuid16 = SERVICE_UUID;

        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);

        esp_ble_gatts_create_service(gatts_if, &gl_profile.service_id, GATTS_NUM_HANDLE);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(BLE_TAG, "Service created, handle: %d", param->create.service_handle);
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
            ESP_LOGI(BLE_TAG, "Failed to add WIFI SSID characteristic");
        }

        // 添加 Wi-Fi 密码特性
        gl_profile.char_uuid_pass.len = ESP_UUID_LEN_16;
        gl_profile.char_uuid_pass.uuid.uuid16 = CHAR_UUID_WIFI_PASS;
        if (esp_ble_gatts_add_char(gl_profile.service_handle, &gl_profile.char_uuid_pass,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL))
        {
            ESP_LOGE(BLE_TAG, "Failed to add WIFI PASS characteristic");
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.status != ESP_GATT_OK)
        {
            ESP_LOGE(BLE_TAG, "Failed to add characteristic, status: %d", param->add_char.status);
        }
        ESP_LOGI(BLE_TAG, "Characteristic added, attr_handle: %d", param->add_char.attr_handle);
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
        ESP_LOGI(BLE_TAG, "Write event received, handle: %d, value len: %d", param->write.handle, param->write.len);
        if (param->write.handle == gl_profile.char_handle_ssid)
        {
            memcpy(wifi_ssid, param->write.value, param->write.len);
            wifi_ssid[param->write.len] = '\0'; // 确保字符串以 '\0' 结尾
            ESP_LOGI(BLE_TAG, "Received Wi-Fi SSID: %s", wifi_ssid);
        }
        else if (param->write.handle == gl_profile.char_handle_pass)
        {
            memcpy(wifi_pass, param->write.value, param->write.len);
            wifi_pass[param->write.len] = '\0'; // 确保字符串以 '\0' 结尾
            ESP_LOGI(BLE_TAG, "Received Wi-Fi Password: %s", wifi_pass);
        }
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

    // 加载白名单
    if (load_whitelist_from_nvs(&whitelist) == ESP_OK)
    {
        ESP_LOGI(BLE_TAG, "Whitelist loaded, num_devices: %d", whitelist.num_devices);
        for (int i = 0; i < whitelist.num_devices; i++)
        {
            esp_ble_gap_update_whitelist(true, whitelist.devices[i], BLE_WL_ADDR_TYPE_PUBLIC);
            ESP_LOGI(BLE_TAG, "Device %d: %08x%04x", i, (whitelist.devices[i][0] << 24) + (whitelist.devices[i][1] << 16) + (whitelist.devices[i][2] << 8) + whitelist.devices[i][3], (whitelist.devices[i][4] << 8) + whitelist.devices[i][5]);
        }
    }

    // 设置广播参数
    if (whitelist.num_devices == 0)
    {
        freedorm_pairing_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
    }
    else
    {
        freedorm_pairing_adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST;
    }

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

    // 注册回调函数

    esp_ble_gap_register_callback(dude_gap_event_handler);
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gatts_app_register(0);
    // esp_ble_gap_register_callback(gap_event_handler);
    // esp_hidd_register_callbacks(hidd_event_callback);

    /* 设置安全参数 */
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND; // bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;       // set the IO capability to No output No input
    uint8_t key_size = 16;                          // the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    pairing_semaphore = xSemaphoreCreateBinary();
    xTaskCreate(&pairing_mode_task, "pairing_mode_task", 2048, NULL, 5, NULL);

    // xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);
}
