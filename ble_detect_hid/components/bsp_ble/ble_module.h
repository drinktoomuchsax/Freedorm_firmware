#ifndef BLE_MODULE_H
#define BLE_MODULE_H

#include "esp_system.h"
#include "esp_bt_defs.h"

#define MAX_WHITELIST_SIZE 10

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
