idf_component_register(SRCS     "_freedorm_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES    bsp_button 
                                bsp_ble 
                                ws2812b 
                                lock_control
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
