#include "weather.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"

static const char *TAG = "WEATHER";

/* 用于在 HTTP 事件回调中动态收集响应数据 */
typedef struct {
    char *data;
    size_t len;
    size_t capacity;
} http_response_t;

/**
 * @brief HTTP 事件回调（在 esp_http_client_perform 内部触发）
 *        通过 HTTP_EVENT_ON_DATA 事件逐块收集响应体
 */
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    http_response_t *resp = (http_response_t *)evt->user_data;
    if (!resp) return ESP_OK;

    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (evt->data && evt->data_len > 0) {
                size_t needed = resp->len + evt->data_len + 1;
                if (needed > resp->capacity) {
                    resp->capacity = needed + 256;
                    char *new_data = realloc(resp->data, resp->capacity);
                    if (!new_data) return ESP_ERR_NO_MEM;
                    resp->data = new_data;
                }
                memcpy(resp->data + resp->len, evt->data, evt->data_len);
                resp->len += evt->data_len;
                resp->data[resp->len] = '\0';
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

/**
 * @brief 将中文字符串进行 URL 编码（%XX 格式）
 */
static char *url_encode(const char *str)
{
    if (!str) return NULL;
    long len = strlen(str);
    char *encoded = malloc(len * 3 + 1);
    if (!encoded) return NULL;
    char *p = encoded;
    for (int i = 0; i < len; i++) {
        unsigned char c = str[i];
        if ((c >= '0' && c <= '9') ||
            (c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') ||
            c == '-' || c == '_' || c == '.' || c == '~') {
            *p++ = c;
        } else {
            sprintf(p, "%%%02X", c);
            p += 3;
        }
    }
    *p = '\0';
    return encoded;
}

/**
 * @brief 发起 HTTPS GET 请求到 uapis.cn 天气 API，返回原始响应字符串
 *        使用事件回调方式收集响应数据，避免 esp_http_client_read 读取不到的问题
 */
static char *http_get_weather(const char *city)
{
    char *encoded_city = url_encode(city);
    if (!encoded_city) {
        ESP_LOGE(TAG, "URL 编码失败");
        return NULL;
    }

    char url[512];
    snprintf(url, sizeof(url),
             "https://%s/api/v1/misc/weather?city=%s&adcode=330382&extended=true&forecast=false&hourly=false&minutely=false&indices=false&lang=zh",
             WEATHER_API_HOST, encoded_city);
    free(encoded_city);

    ESP_LOGI(TAG, "请求 URL: %s", url);

    /* 初始化响应收集结构体 */
    http_response_t resp = {
        .data = NULL,
        .len = 0,
        .capacity = 0,
    };

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = http_event_handler,
        .user_data = &resp,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .skip_cert_common_name_check = true,
        .timeout_ms = 30000,
        .keep_alive_enable = false,
        .buffer_size = 1024,
        .buffer_size_tx = 512,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "HTTP 客户端初始化失败");
        free(resp.data);
        return NULL;
    }

    /* 设置请求头：认证 + 模拟浏览器 */
    esp_http_client_set_header(client, "Authorization",
                               "Bearer " WEATHER_API_KEY);
    esp_http_client_set_header(client, "User-Agent",
                               "Mozilla/5.0 (Windows NT 10.0; Win64; x64)");

    ESP_LOGI(TAG, "开始连接（超时30秒）...");
    esp_err_t err = esp_http_client_perform(client);

    int http_status = -1;
    if (err == ESP_OK) {
        http_status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "HTTP 状态码: %d", http_status);
        ESP_LOGI(TAG, "Content-Length: %lld",
                 (long long)esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP 请求失败: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        free(resp.data);
        return NULL;
    }

    esp_http_client_cleanup(client);

    if (resp.len == 0) {
        ESP_LOGE(TAG, "响应体为空");
        free(resp.data);
        return NULL;
    }

    ESP_LOGI(TAG, "收到 %u 字节响应", resp.len);
    return resp.data;
}

/**
 * @brief 获取指定城市的天气信息并打印到串口
 */
esp_err_t weather_fetch_and_print(const char *city)
{
    if (!city || strlen(city) == 0) {
        ESP_LOGE(TAG, "城市名称为空");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = weather_fetch_json(city);
    if (!root) {
        return ESP_FAIL;
    }

    cJSON *city_name   = cJSON_GetObjectItem(root, "city");
    cJSON *weather     = cJSON_GetObjectItem(root, "weather");
    cJSON *temperature = cJSON_GetObjectItem(root, "temperature");
    cJSON *humidity    = cJSON_GetObjectItem(root, "humidity");
    cJSON *wind_dir    = cJSON_GetObjectItem(root, "wind_direction");
    cJSON *wind_power  = cJSON_GetObjectItem(root, "wind_power");
    cJSON *report_time = cJSON_GetObjectItem(root, "report_time");

    ESP_LOGI(TAG, "========== 实时天气 ==========");
    if (city_name && cJSON_IsString(city_name))
        ESP_LOGI(TAG, "城市: %s", city_name->valuestring);
    if (weather && cJSON_IsString(weather))
        ESP_LOGI(TAG, "天气: %s", weather->valuestring);
    if (temperature && cJSON_IsNumber(temperature))
        ESP_LOGI(TAG, "温度: %.0f℃", temperature->valuedouble);
    if (humidity && cJSON_IsNumber(humidity))
        ESP_LOGI(TAG, "湿度: %.0f%%", humidity->valuedouble);
    if (wind_dir && cJSON_IsString(wind_dir))
        ESP_LOGI(TAG, "风向: %s", wind_dir->valuestring);
    if (wind_power && cJSON_IsString(wind_power))
        ESP_LOGI(TAG, "风力: %s", wind_power->valuestring);
    if (report_time && cJSON_IsString(report_time))
        ESP_LOGI(TAG, "发布时间: %s", report_time->valuestring);
    ESP_LOGI(TAG, "==============================");

    cJSON_Delete(root);
    return ESP_OK;
}

/**
 * @brief 获取天气数据并返回解析后的 cJSON 对象
 *        调用者必须使用 cJSON_Delete() 释放返回的对象
 */
cJSON *weather_fetch_json(const char *city)
{
    if (!city || strlen(city) == 0) {
        ESP_LOGE(TAG, "城市名称为空");
        return NULL;
    }

    char *response = http_get_weather(city);
    if (!response) {
        ESP_LOGE(TAG, "获取天气数据失败");
        return NULL;
    }

    cJSON *root = cJSON_Parse(response);
    free(response);

    if (!root) {
        ESP_LOGE(TAG, "天气 JSON 解析失败");
        return NULL;
    }

    return root;
}