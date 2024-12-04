idf_component_register(
                        SRCS    "ble_module.c"
                                "esp_hidd_prf_api.c"
                                "hid_dev.c"
                                "hid_device_le_prf.c"
                        INCLUDE_DIRS "."
                        PRIV_REQUIRES   bsp_button 
                                        driver
                                        freertos
                                        esp_system
                                        esp_wifi
                                        esp_event
                                        log
                                        nvs_flash
                                        bt
                                        driver
                                        esp_common
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)



