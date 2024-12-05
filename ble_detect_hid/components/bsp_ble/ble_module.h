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

extern SemaphoreHandle_t pairing_semaphore;

typedef struct
{
    uint8_t num_devices;
    esp_bd_addr_t devices[MAX_WHITELIST_SIZE];
} whitelist_t;

void ble_module_init(void);
esp_err_t load_whitelist_from_nvs(whitelist_t *list);
esp_err_t save_whitelist_to_nvs(whitelist_t *list);
esp_err_t delete_whitelist_from_nvs(void);

#endif // BLE_MODULE_H
