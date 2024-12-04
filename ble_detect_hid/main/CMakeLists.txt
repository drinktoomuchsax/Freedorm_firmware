idf_component_register(SRCS     "_freedorm_main.c"
                                "esp_hidd_prf_api.c"
                                "hid_dev.c"
                                "hid_device_le_prf.c"
                                "./bsp_ble/ble_module.c"        # 添加外部库源文件
                    INCLUDE_DIRS "."
                                 "./bsp_ble"
                    
                    REQUIRES bsp_button
                    PRIV_REQUIRES   freertos
                                    esp_system
                                    esp_wifi
                                    esp_event
                                    log
                                    nvs_flash
                                    bt
                                    driver
                                    esp_common)           # 添加外部库头文件目录

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
