idf_component_register(SRCS "freedorm_mqtt.c"   # 你的 MQTT 客户端代码
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES    mqtt
                                        esp_wifi
                                        nvs_flash
                                        esp_netif
                                        )         # 这里引用 MQTT 组件
