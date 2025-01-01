idf_component_register(SRCS "ws2812b_led.c" "led_strip_encoder.c"
                       PRIV_REQUIRES    esp_driver_rmt
                                        esp_system
                                        driver
                                        lock_control
                       INCLUDE_DIRS ".")
