#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_tls.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#define WIFI_SSID "SUSTech-wifi"
#define WIFI_PASSWORD ""  // 无密码

#define CAS_LOGIN_URL "https://cas.sustech.edu.cn/cas/login?service=http%3A%2F%2F172.16.16.20%3A803%2Fsustc_cas.php"
#define CAS_POST_URL "https://cas.sustech.edu.cn/cas/login?service=http%3A%2F%2F172.16.16.20%3A803%2Fsustc_cas.php"

#define USERNAME "你的学号"
#define PASSWORD "你的密码"

static const char *TAG = "CAS_Auth";

// 根证书（根据学校 CAS 服务器情况获取）
extern const uint8_t server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
extern const uint8_t server_root_cert_pem_end[]   asm("_binary_server_root_cert_pem_end");

extern const uint8_t cas_cert_pem_start[] asm("_binary_cas_sustech_pem_start");
extern const uint8_t cas_cert_pem_end[] asm("_binary_cas_sustech_pem_end");

esp_tls_cfg_t cfg = {
    .cacert_buf = cas_cert_pem_start,
    .cacert_bytes = cas_cert_pem_end - cas_cert_pem_start,
};


static char session_cookie[512] = {0};  // 存储 Cookie

// 解析 execution 参数
char *extract_execution(const char *html) {
    char *execution = strstr(html, "name=\"execution\" value=\"");
    if (!execution) {
        ESP_LOGE(TAG, "未找到 execution");
        return NULL;
    }
    execution += 24; // 跳过 `name="execution" value="`
    char *end = strchr(execution, '"');
    if (!end) return NULL;
    size_t len = end - execution;
    
    char *exec_value = (char *)malloc(len + 1);
    if (!exec_value) return NULL;
    strncpy(exec_value, execution, len);
    exec_value[len] = '\0';

    ESP_LOGI(TAG, "解析到 execution: %s", exec_value);
    return exec_value;
}

// 解析 Set-Cookie 头
void extract_cookie(const char *response) {
    char *cookie_start = strstr(response, "Set-Cookie: ");
    if (!cookie_start) {
        ESP_LOGE(TAG, "未找到 Set-Cookie");
        return;
    }
    cookie_start += 12; // 跳过 "Set-Cookie: "
    char *cookie_end = strchr(cookie_start, ';');
    if (!cookie_end) return;
    
    size_t len = cookie_end - cookie_start;
    strncpy(session_cookie, cookie_start, len);
    session_cookie[len] = '\0';

    ESP_LOGI(TAG, "获取到 Cookie: %s", session_cookie);
}

// 发送 HTTPS 请求
static char *https_request(const char *url, const char *request) {
    esp_tls_cfg_t cfg = {
        .cacert_buf = server_root_cert_pem_start,
        .cacert_bytes = server_root_cert_pem_end - server_root_cert_pem_start,
    };

    esp_tls_t *tls = esp_tls_init();  // 初始化 TLS 结构体
    if (!tls) {
        ESP_LOGE(TAG, "TLS 结构体分配失败");
        return NULL;
    }

    // 调用函数时，传入 `tls` 作为第三个参数
    int ret = esp_tls_conn_http_new_sync(url, &cfg, tls);
    if (ret != 1) {
        ESP_LOGE(TAG, "TLS 连接失败");
        esp_tls_conn_destroy(tls);
        return NULL;
    }

    int written = esp_tls_conn_write(tls, request, strlen(request));
    if (written < 0) {
        ESP_LOGE(TAG, "发送请求失败");
        esp_tls_conn_destroy(tls);
        return NULL;
    }

    char *response = (char *)malloc(4096);
    if (!response) {
        esp_tls_conn_destroy(tls);
        return NULL;
    }

    int len = esp_tls_conn_read(tls, response, 4096 - 1);
    if (len < 0) {
        ESP_LOGE(TAG, "读取响应失败");
        free(response);
        esp_tls_conn_destroy(tls);
        return NULL;
    }
    response[len] = '\0';
    esp_tls_conn_destroy(tls);

    return response;
}


// CAS 登录
void cas_login(void) {
    ESP_LOGI(TAG, "开始 CAS 登录流程...");

    // 1. 发送 GET 请求，获取 execution 参数和 Cookie
    char get_request[256];
    // snprintf(get_request, sizeof(get_request),
    //          "GET %s HTTP/1.1\r\n"
    //          "Host: cas.sustech.edu.cn\r\n"
    //          "User-Agent: ESP32-C3\r\n"
    //          "Connection: close\r\n\r\n", CAS_LOGIN_URL);

    char *response = https_request(CAS_LOGIN_URL, get_request);
    if (!response) {
        ESP_LOGE(TAG, "获取 CAS 登录页面失败");
        return;
    }

    char *execution = extract_execution(response);
    extract_cookie(response);
    free(response);
    if (!execution) {
        ESP_LOGE(TAG, "未能获取 execution 参数");
        return;
    }

    // 2. 发送 POST 请求，提交用户名和密码
    char post_data[512];
    // snprintf(post_data, sizeof(post_data),
    //          "username=%s&password=%s&execution=%s&_eventId=submit&geolocation=",
    //          USERNAME, PASSWORD, execution);
    free(execution);

    char post_request[1024];
    // snprintf(post_request, sizeof(post_request),
    //          "POST %s HTTP/1.1\r\n"
    //          "Host: cas.sustech.edu.cn\r\n"
    //          "User-Agent: ESP32-C3\r\n"
    //          "Content-Type: application/x-www-form-urlencoded\r\n"
    //          "Content-Length: %d\r\n"
    //          "Cookie: %s\r\n"
    //          "Connection: close\r\n\r\n"
    //          "%s", CAS_POST_URL, (int)strlen(post_data), session_cookie, post_data);

    response = https_request(CAS_POST_URL, post_request);
    if (!response) {
        ESP_LOGE(TAG, "登录请求失败");
        return;
    }

    // 3. 检查是否登录成功
    if (strstr(response, "success")) {
        ESP_LOGI(TAG, "CAS 认证成功！");
    } else {
        ESP_LOGE(TAG, "CAS 认证失败！");
    }

    free(response);
}

// 连接 Wi-Fi
void wifi_init(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect()); // 使用 IDF 提供的 Wi-Fi 连接函数
}

// 主函数
void app_main(void) {
    wifi_init();
    vTaskDelay(5000 / portTICK_PERIOD_MS); // 等待 Wi-Fi 连接
    cas_login();
}
