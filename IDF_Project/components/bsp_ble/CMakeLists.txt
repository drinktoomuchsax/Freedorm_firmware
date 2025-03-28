idf_component_register(
                        SRCS    "ble_module.c"
                                "esp_hidd_prf_api.c"
                                "hid_dev.c"
                                "hid_device_le_prf.c"
                                "kalman_filter.c"
                        INCLUDE_DIRS "."
                        REQUIRES bt
                        PRIV_REQUIRES   bsp_button 
                                        driver
                                        freertos
                                        esp_system
                                        esp_wifi
                                        esp_event
                                        log
                                        nvs_flash
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)



