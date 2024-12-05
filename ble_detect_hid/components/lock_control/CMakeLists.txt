idf_component_register(
    SRCS "lock_control.c"
    INCLUDE_DIRS "."
    REQUIRES driver bsp_ble bsp_button
    PRIV_REQUIRES   freertos
                    esp_event
                    log
                    nvs_flash
)
