#ifndef FREEDORM_MQTT_H
#define FREEDORM_MQTT_H

#include "esp_err.h"

// 初始化 MQTT 客户端并启动
esp_err_t mqtt_start(void);

#endif // FREEDORM_MQTT_H
