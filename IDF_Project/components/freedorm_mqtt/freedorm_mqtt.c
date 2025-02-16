#include "freedorm_mqtt.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_system.h"

// MQTT 客户端对象
static esp_mqtt_client_handle_t client;

// MQTT 配置结构体
static esp_mqtt_client_config_t mqtt_cfg = {
    .broker =
        {
            .address =
                {
                    .uri = "mqtt://",                     // 使用一个公共的 MQTT Broker
                    .port = 1883,                         // 设置端口号，默认为 1883
                    .transport = MQTT_TRANSPORT_OVER_TCP, // 设置传输方式，TCP 是常见的选项
                },
        },
    // 你可以在此添加更多配置，比如用户名、密码等
};

// MQTT 事件处理回调
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("MQTT", "MQTT Connected");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI("MQTT", "MQTT Disconnected");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI("MQTT", "Subscribed to topic: %s", event->topic);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI("MQTT", "Unsubscribed from topic");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("MQTT", "Message Published");
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI("MQTT", "Received Data: %s", event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE("MQTT", "MQTT Error: %d", event->error_handle->error_type);
        break;
    default:
        ESP_LOGI("MQTT", "Unhandled event: %d", event->event_id);
        break;
    }
    return ESP_OK;
}

// mqtt_start函数，初始化并启动 MQTT 客户端
esp_err_t mqtt_start(void)
{
    ESP_LOGI("MQTT", "Starting MQTT Client...");

    // 初始化 MQTT 客户端
    client = esp_mqtt_client_init(&mqtt_cfg);

    if (client == NULL)
    {
        ESP_LOGE("MQTT", "Failed to create MQTT client");
        return ESP_ERR_NO_MEM;
    }

    // 注册 MQTT 事件处理回调
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    // 启动 MQTT 客户端
    esp_err_t err = esp_mqtt_client_start(client);
    if (err != ESP_OK)
    {
        ESP_LOGE("MQTT", "Failed to start MQTT client: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}
