#include "LLM.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LLM";

#define RESPONSE_BUF_SIZE 32768

/* 用于在 HTTP 事件回调中动态收集响应数据 */
typedef struct {
    char *data;
    size_t len;
    size_t capacity;
} http_response_t;

/**
 * @brief HTTP 事件回调，通过 HTTP_EVENT_ON_DATA 逐块收集响应体
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
 * @brief 发送提示词到火山引擎大模型网关，获取返回文本
 */
static esp_err_t llm_http_post(const char *system_prompt, const char *user_prompt,
                                char *response_buf, int buf_len)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", LLM_MODEL_NAME);

    cJSON *messages = cJSON_CreateArray();

    /* system 消息：设定角色和输出格式 */
    cJSON *sys_msg = cJSON_CreateObject();
    cJSON_AddStringToObject(sys_msg, "role", "system");
    cJSON_AddStringToObject(sys_msg, "content", system_prompt);
    cJSON_AddItemToArray(messages, sys_msg);

    /* user 消息：传入实际天气数据 */
    cJSON *usr_msg = cJSON_CreateObject();
    cJSON_AddStringToObject(usr_msg, "role", "user");
    cJSON_AddStringToObject(usr_msg, "content", user_prompt);
    cJSON_AddItemToArray(messages, usr_msg);

    cJSON_AddItemToObject(root, "messages", messages);

    cJSON_AddNumberToObject(root, "max_tokens", 100);

    char *post_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!post_data) {
        ESP_LOGE(TAG, "JSON 生成失败");
        return ESP_FAIL;
    }

    /* 初始化响应收集结构体 */
    http_response_t resp = {
        .data = NULL,
        .len = 0,
        .capacity = 0,
    };

    esp_http_client_config_t config = {
        .url = LLM_API_URL,
        .method = HTTP_METHOD_POST,
        .event_handler = http_event_handler,
        .user_data = &resp,
        .skip_cert_common_name_check = true,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 60000,
        .buffer_size = 4096,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "HTTP 客户端初始化失败");
        free(post_data);
        free(resp.data);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    char auth_buf[256];
    snprintf(auth_buf, sizeof(auth_buf), "Bearer %s", LLM_API_KEY);
    esp_http_client_set_header(client, "Authorization", auth_buf);

    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    free(post_data);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP 请求失败: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        free(resp.data);
        return err;
    }

    int status = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "大模型 HTTP 状态码: %d", status);

    esp_http_client_cleanup(client);

    if (resp.len == 0) {
        ESP_LOGE(TAG, "大模型响应体为空");
        free(resp.data);
        return ESP_FAIL;
    }

    if (status != 200) {
        ESP_LOGE(TAG, "大模型返回异常状态码: %d, 响应: %s", status, resp.data);
        free(resp.data);
        return ESP_FAIL;
    }

    /* 复制到输出缓冲区 */
    size_t copy_len = (resp.len < (size_t)(buf_len - 1)) ? resp.len : (size_t)(buf_len - 1);
    memcpy(response_buf, resp.data, copy_len);
    response_buf[copy_len] = '\0';

    ESP_LOGD(TAG, "大模型原始响应: %s", resp.data);
    free(resp.data);
    return ESP_OK;
}

/**
 * @brief 从大模型返回 JSON 中提取 choices[0].message.content 内的综合控制指令
 *        content 内容为 JSON 格式：
 *        {"servo": 25, "fan": true, "wet": false, "reason": "温度舒适，无需除湿"}
 */
static esp_err_t llm_parse_response(const char *response, llm_command_t *out_cmd)
{
    if (!out_cmd) return ESP_ERR_INVALID_ARG;
    out_cmd->servo_angle = 65;
    out_cmd->fan_on = false;
    out_cmd->wet_on = false;
    snprintf(out_cmd->reason, sizeof(out_cmd->reason), "默认");

    cJSON *root = cJSON_Parse(response);
    if (!root) {
        ESP_LOGE(TAG, "大模型返回 JSON 解析失败，原始响应(前300字节): %.300s", response);
        return ESP_FAIL;
    }

    cJSON *choices = cJSON_GetObjectItem(root, "choices");
    if (!choices || !cJSON_IsArray(choices)) {
        ESP_LOGE(TAG, "返回数据无 choices 数组");
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    cJSON *choice0 = cJSON_GetArrayItem(choices, 0);
    if (!choice0) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    cJSON *message = cJSON_GetObjectItem(choice0, "message");
    if (!message) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    cJSON *content = cJSON_GetObjectItem(message, "content");
    if (!content || !cJSON_IsString(content)) {
        ESP_LOGE(TAG, "返回数据无 content 字段");
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "大模型返回内容: %s", content->valuestring);

    /* 解析 content 中的 JSON：{"servo": 25, "fan": true/false, "wet": true/false, "reason": "..."} */
    /* 先尝试直接解析，如果失败则尝试提取 ```json ... ``` 包裹内容 */
    const char *content_str = content->valuestring;
    cJSON *content_json = cJSON_Parse(content_str);
    if (!content_json) {
        /* 尝试匹配 ```json 代码块 */
        const char *start = strstr(content_str, "```json");
        if (start) {
            start += 7; /* 跳过 ```json */
            while (*start == '\n' || *start == '\r') start++;
            const char *end = strstr(start, "```");
            if (end) {
                size_t json_len = end - start;
                char *json_str = malloc(json_len + 1);
                if (json_str) {
                    memcpy(json_str, start, json_len);
                    json_str[json_len] = '\0';
                    content_json = cJSON_Parse(json_str);
                    free(json_str);
                }
            }
        }
        if (!content_json) {
            ESP_LOGW(TAG, "content 无法解析为 JSON，使用默认值");
            cJSON_Delete(root);
            return ESP_OK;
        }
    }

    cJSON *servo_item = cJSON_GetObjectItem(content_json, "servo");
    cJSON *fan_item   = cJSON_GetObjectItem(content_json, "fan");
    cJSON *wet_item   = cJSON_GetObjectItem(content_json, "wet");
    cJSON *reason_item = cJSON_GetObjectItem(content_json, "reason");

    if (servo_item && cJSON_IsNumber(servo_item)) {
        int angle = (int)servo_item->valuedouble;
        if (angle == 65 || angle == 45 || angle == 25) {
            out_cmd->servo_angle = angle;
        } else {
            out_cmd->servo_angle = (angle > 55) ? 65 : (angle > 35 ? 45 : 25);
            ESP_LOGW(TAG, "舵机角度 %d 不是标准值(65/45/25)，就近取 %d", angle, out_cmd->servo_angle);
        }
    } else {
        out_cmd->servo_angle = 65;
        ESP_LOGW(TAG, "未找到 servo 字段或类型错误，默认 65°");
    }

    if (fan_item && cJSON_IsBool(fan_item)) {
        out_cmd->fan_on = cJSON_IsTrue(fan_item);
    } else {
        out_cmd->fan_on = false;
        ESP_LOGW(TAG, "未找到 fan 字段，默认关闭风扇");
    }

    if (wet_item && cJSON_IsBool(wet_item)) {
        out_cmd->wet_on = cJSON_IsTrue(wet_item);
    } else {
        out_cmd->wet_on = false;
        ESP_LOGW(TAG, "未找到 wet 字段，默认关闭加湿器");
    }

    if (reason_item && cJSON_IsString(reason_item)) {
        snprintf(out_cmd->reason, sizeof(out_cmd->reason), "%s", reason_item->valuestring);
        ESP_LOGI(TAG, "大模型理由: %s", out_cmd->reason);
    }

    cJSON_Delete(content_json);
    cJSON_Delete(root);
    return ESP_OK;
}

/**
 * @brief 融合外网气象与本地传感器数据，获取大模型下发的综合控制指令
 *        内部自动重试 3 次应对网络抖动
 */
esp_err_t llm_get_command(int temperature, int humidity,
                          const char *wind_power, const char *weather_desc,
                          int indoor_temp, int indoor_humi,
                          const char *rain_status,
                          float local_wind,
                          llm_command_t *out_cmd)
{
    if (!out_cmd) return ESP_ERR_INVALID_ARG;

    out_cmd->servo_angle = 65;
    out_cmd->fan_on = false;
    out_cmd->wet_on = false;
    snprintf(out_cmd->reason, sizeof(out_cmd->reason), "默认");

    const char *wind = wind_power ? wind_power : "未知";
    const char *wthr = weather_desc ? weather_desc : "未知";
    const char *rain = rain_status ? rain_status : "未知";

    /* system 提示 */
    const char *system_prompt =
        "你是智能家居室内微气候调控专家。"
        "输出一行JSON，reason不超过8字，禁止解释，禁止换行，禁止markdown。"
        "只输出JSON，不要其他任何内容。";

    /* user 提示：融合外网气象 + 本地传感器数据 */
    char user_prompt[1024];
    snprintf(user_prompt, sizeof(user_prompt),
             "【外网室外气象】%s, 室外温度:%d°C, 室外湿度:%d%%, 风力:%s。\n"
             "【本地室内传感器】室内温度:%d°C, 室内湿度:%d%%。\n"
             "【本地室外传感器】是否下雨:%s, 本地实测风速:%.1fm/s。\n"
             "任务：融合外网气象与本地内外传感器数据，综合调控室内环境并做好室外防护。\n"
             "规则：\n"
             "- 舒适：全开窗(65°)，关闭风扇与加湿器\n"
             "- 温湿度异常：室内温度>30°C或室内湿度>70%%半开窗(45°)并开风扇除湿，<60%%开加湿器\n"
             "- 恶劣天气防护：下雨/大风>5m/s全关窗(25°)，关风扇关加湿器\n"
             "请输出JSON：{\"servo\":65, \"fan\":false, \"wet\":false, \"reason\":\"简短理由\"}\n"
             "只输出JSON，不要其他内容。",
             wthr, temperature, humidity, wind,
             indoor_temp, indoor_humi,
             rain, local_wind);

    ESP_LOGI(TAG, "发送到大模型的提示词: %s", user_prompt);

    char *response = malloc(RESPONSE_BUF_SIZE);
    if (!response) {
        ESP_LOGE(TAG, "内存分配失败");
        return ESP_ERR_NO_MEM;
    }
    memset(response, 0, RESPONSE_BUF_SIZE);

    /* 重试 3 次，间隔 5 秒，应对网络抖动 */
    esp_err_t ret = ESP_FAIL;
    for (int attempt = 1; attempt <= 3; attempt++) {
        ESP_LOGI(TAG, "大模型请求 第%d/3 次", attempt);
        ret = llm_http_post(system_prompt, user_prompt,
                            response, RESPONSE_BUF_SIZE);
        if (ret == ESP_OK) break;

        ESP_LOGW(TAG, "大模型请求第%d次失败: %s", attempt, esp_err_to_name(ret));
        if (attempt < 3) {
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "大模型请求重试3次均失败");
        free(response);
        return ret;
    }

    ret = llm_parse_response(response, out_cmd);
    free(response);
    return ret;
}