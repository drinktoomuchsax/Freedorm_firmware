idf_component_register(
                        SRCS   "lock_control.c"
                        INCLUDE_DIRS    "."
                        PRIV_REQUIRES   driver 
                                        bsp_ble
                                        freertos
                                        esp_event
                                        log
                                        nvs_flash
                                        main
                                        bsp_button
                                        ws2812b
)
