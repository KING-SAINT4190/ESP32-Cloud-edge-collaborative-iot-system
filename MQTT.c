/**
 * @file MQTT.c
 * @brief OneNET Studio MQTT 上传实现（轻量级，直接使用 lwIP BSD sockets）
 *        依赖：lwip + cJSON
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include "MQTT.h"
#include "esp_log.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "ONENET_MQTT";

/* ======================== 配置常量 ======================== */

#define ONENET_MQTT_PORT    1883

/* 认证密钥 */
#define ONENET_PASSWORD \
    "version=2018-10-31&res=products%2FIq4m2MA5vx%2Fdevices%2Fwulian&et=1814851227&method=md5&sign=UrKXnqPYOzSmUJImCXKnfQ%3D%3D"

/* OneNET 属性上报主题 */
#define ONENET_TOPIC_PROPERTY_POST \
    "$sys/" ONENET_PRODUCT_ID "/" ONENET_DEVICE_NAME "/thing/property/post"

/* 自增消息 ID */
static uint32_t s_msg_id = 0;

/* 套接字和连接状态 */
static int s_sock = -1;
static bool s_connected = false;

/* 后台任务句柄 */
static TaskHandle_t s_mqtt_task = NULL;

/* ======================== MQTT 协议编解码 ======================== */

/**
 * @brief MQTT 剩余长度编码（最大 268435455 字节）
 * @return 编码后的字节数
 */
static int mqtt_encode_remaining(uint8_t *buf, uint32_t len)
{
    int i = 0;
    do {
        buf[i] = len % 128;
        len /= 128;
        if (len > 0) buf[i] |= 0x80;
        i++;
    } while (len > 0);
    return i;
}

/**
 * @brief 构建 MQTT CONNECT 报文
 * @param buf  输出缓冲区（需足够大，建议 512 字节）
 * @return 报文总长度
 */
static int build_connect_packet(uint8_t *buf)
{
    const char *client_id = ONENET_DEVICE_NAME;   /* OneNET 要求 Client ID = 设备名 */
    const char *username  = ONENET_PRODUCT_ID;
    const char *password  = ONENET_PASSWORD;

    uint16_t id_len   = (uint16_t)strlen(client_id);
    uint16_t user_len = (uint16_t)strlen(username);
    uint16_t pass_len = (uint16_t)strlen(password);

    /* 可变头部 + payload 长度 */
    uint32_t payload_len = 2 + id_len;                     /* Client ID */
    uint8_t flags = 0x02;                                  /* Clean Session */
    if (user_len > 0) { payload_len += 2 + user_len; flags |= 0x80; }
    if (pass_len > 0) { payload_len += 2 + pass_len; flags |= 0x40; }

    uint32_t var_header_len = 10;                          /* "MQTT" + 4 + flags + keepalive */
    uint32_t remaining = var_header_len + payload_len;

    uint8_t *p = buf;

    /* 固定头部 */
    *p++ = 0x10;                                           /* CONNECT */
    p += mqtt_encode_remaining(p, remaining);              /* 剩余长度 */

    /* 可变头部 */
    *p++ = 0x00; *p++ = 0x04;                              /* 协议名长度 */
    *p++ = 'M'; *p++ = 'Q'; *p++ = 'T'; *p++ = 'T';      /* "MQTT" */
    *p++ = 0x04;                                           /* 协议级别 (MQTT 3.1.1) */
    *p++ = flags;                                          /* 连接标志 */
    *p++ = 0x00; *p++ = 0x3C;                              /* Keep Alive: 60 秒 */

    /* Payload: Client ID */
    *p++ = (uint8_t)(id_len >> 8);
    *p++ = (uint8_t)(id_len & 0xFF);
    memcpy(p, client_id, id_len);
    p += id_len;

    /* Payload: Username */
    if (user_len > 0) {
        *p++ = (uint8_t)(user_len >> 8);
        *p++ = (uint8_t)(user_len & 0xFF);
        memcpy(p, username, user_len);
        p += user_len;
    }

    /* Payload: Password */
    if (pass_len > 0) {
        *p++ = (uint8_t)(pass_len >> 8);
        *p++ = (uint8_t)(pass_len & 0xFF);
        memcpy(p, password, pass_len);
        p += pass_len;
    }

    return (int)(p - buf);
}

/**
 * @brief 构建 MQTT PUBLISH 报文（QoS 1）
 * @param buf     输出缓冲区
 * @param topic   主题字符串
 * @param payload 消息体
 * @return 报文总长度
 */
static int build_publish_packet(uint8_t *buf, const char *topic,
                                 const char *payload)
{
    uint16_t topic_len = (uint16_t)strlen(topic);
    uint32_t payload_len = (uint32_t)strlen(payload);
    uint32_t remaining = 2 + topic_len + 2 + payload_len;  /* topic + packetId + payload */

    uint8_t *p = buf;

    /* 固定头部: QoS 1, DUP=0, Retain=0 */
    *p++ = 0x32;
    p += mqtt_encode_remaining(p, remaining);

    /* 主题 */
    *p++ = (uint8_t)(topic_len >> 8);
    *p++ = (uint8_t)(topic_len & 0xFF);
    memcpy(p, topic, topic_len);
    p += topic_len;

    /* 报文标识符 */
    uint16_t packet_id = (uint16_t)(++s_msg_id & 0xFFFF);
    *p++ = (uint8_t)(packet_id >> 8);
    *p++ = (uint8_t)(packet_id & 0xFF);

    /* Payload */
    memcpy(p, payload, payload_len);
    p += payload_len;

    return (int)(p - buf);
}

/**
 * @brief 读取 MQTT 固定头部并等待完整报文
 * @param sock  套接字
 * @param buf   接收缓冲区
 * @param bufsz 缓冲区大小
 * @return >0: 报文类型字节; -1: 错误
 */
static int mqtt_read_packet(int sock, uint8_t *buf, int bufsz)
{
    /* 读第一个字节（报文类型） */
    uint8_t header;
    int ret = recv(sock, &header, 1, 0);
    if (ret <= 0) return -1;

    /* 读剩余长度 */
    uint8_t enc[4];
    uint32_t remaining = 0;
    int multiplier = 1;
    for (int i = 0; i < 4; i++) {
        ret = recv(sock, &enc[i], 1, 0);
        if (ret <= 0) return -1;
        remaining += (enc[i] & 0x7F) * multiplier;
        multiplier *= 128;
        if (!(enc[i] & 0x80)) break;
    }

    /* 读剩余部分 */
    int total = 1 + (int)((uint8_t*)&enc[4] - (uint8_t*)enc);  /* 已经读的编码字节数 */
    total += (int)remaining;

    if (total > bufsz) {
        ESP_LOGE(TAG, "MQTT 报文过大: %d > %d", total, bufsz);
        return -1;
    }

    buf[0] = header;
    memcpy(&buf[1], enc, (size_t)((uint8_t*)&enc[4] - (uint8_t*)enc));
    int pos = 1 + (int)((uint8_t*)&enc[4] - (uint8_t*)enc);
    while (pos < total) {
        ret = recv(sock, buf + pos, total - pos, 0);
        if (ret <= 0) return -1;
        pos += ret;
    }

    return header;
}

/**
 * @brief 构建并发送 OneNET 属性上报 JSON 报文
 */
static int send_property_publish(int sock, const char *window_status,
                                  bool fan_on, bool spray_on)
{
    /* 构建 JSON payload */
    cJSON *root = cJSON_CreateObject();
    char id_buf[16];
    snprintf(id_buf, sizeof(id_buf), "%u", (unsigned)(++s_msg_id));
    cJSON_AddStringToObject(root, "id", id_buf);
    cJSON_AddStringToObject(root, "version", "1.0");

    cJSON *params = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "params", params);

    cJSON *window_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(window_obj, "value",
                             window_status ? window_status : "未知");
    cJSON_AddItemToObject(params, "window", window_obj);

    cJSON *fan_obj = cJSON_CreateObject();
    cJSON_AddBoolToObject(fan_obj, "value", fan_on);
    cJSON_AddItemToObject(params, "FAN", fan_obj);

    cJSON *spray_obj = cJSON_CreateObject();
    cJSON_AddBoolToObject(spray_obj, "value", spray_on);
    cJSON_AddItemToObject(params, "spray", spray_obj);

    char *payload = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!payload) return -1;

    ESP_LOGI(TAG, "发布: window=%s, FAN=%s, spray=%s",
             window_status ? window_status : "NULL",
             fan_on ? "true" : "false",
             spray_on ? "true" : "false");

    uint8_t *buf = malloc(2048);
    if (!buf) {
        ESP_LOGE(TAG, "分配发布缓冲区失败");
        free(payload);
        return -1;
    }
    int len = build_publish_packet(buf, ONENET_TOPIC_PROPERTY_POST, payload);
    free(payload);

    if (len > 2048) {
        ESP_LOGE(TAG, "发布报文过大: %d", len);
        free(buf);
        return -1;
    }

    int ret = send(sock, buf, len, 0);
    free(buf);
    if (ret < 0) {
        ESP_LOGE(TAG, "发布 send 失败: %d", ret);
        return -1;
    }
    return 0;
}

/* ======================== MQTT 后台任务 ======================== */

static void mqtt_task(void *arg)
{
    ESP_LOGI(TAG, "MQTT 后台任务已启动");

    while (1) {
        /* 等待 WiFi 连接就绪（检查是否有可用网络接口） */
        struct sockaddr_in test;
        socklen_t test_len = sizeof(test);
        int test_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (test_sock < 0) {
            ESP_LOGD(TAG, "网络未就绪，10秒后重试");
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        close(test_sock);

        int sock = -1;

        /* ---- 解析地址 ---- */
        struct addrinfo hints = {
            .ai_family = AF_INET,
            .ai_socktype = SOCK_STREAM,
        };
        struct addrinfo *res;
        char port_str[8];
        snprintf(port_str, sizeof(port_str), "%d", ONENET_MQTT_PORT);

        int err = getaddrinfo(ONENET_MQTT_HOST, port_str, &hints, &res);
        if (err != 0 || !res) {
            ESP_LOGE(TAG, "DNS 解析失败: %s", ONENET_MQTT_HOST);
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        /* ---- TCP 连接 ---- */
        sock = socket(res->ai_family, res->ai_socktype, 0);
        if (sock < 0) {
            ESP_LOGE(TAG, "创建 socket 失败");
            freeaddrinfo(res);
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        struct timeval timeout = { .tv_sec = 5, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        struct sockaddr_in *addr = (struct sockaddr_in *)res->ai_addr;
        ESP_LOGI(TAG, "正在连接 %s:%d ...", ONENET_MQTT_HOST, ONENET_MQTT_PORT);
        if (connect(sock, (struct sockaddr *)addr, sizeof(*addr)) < 0) {
            ESP_LOGE(TAG, "TCP 连接失败");
            close(sock);
            freeaddrinfo(res);
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        freeaddrinfo(res);
        ESP_LOGI(TAG, "TCP 连接成功");

        /* ---- 发送 CONNECT ---- */
        uint8_t conn_buf[512];
        int conn_len = build_connect_packet(conn_buf);
        if (send(sock, conn_buf, conn_len, 0) != conn_len) {
            ESP_LOGE(TAG, "CONNECT 发送失败");
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        /* ---- 等待 CONNACK ---- */
        uint8_t resp_buf[256];
        int pkt_type = mqtt_read_packet(sock, resp_buf, sizeof(resp_buf));
        if (pkt_type < 0 || (pkt_type & 0xF0) != 0x20) {
            ESP_LOGE(TAG, "CONNACK 接收失败, type=0x%02X", pkt_type);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        /* CONNACK: 固定头(2) + 可变头(4)，第3字节=0表示接受 */
        if (resp_buf[3] != 0x00) {
            ESP_LOGE(TAG, "MQTT 连接被拒绝, code=0x%02X", resp_buf[3]);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(30000));
            continue;
        }

        s_sock = sock;
        s_connected = true;
        ESP_LOGI(TAG, "OneNET MQTT 已连接！");

        /* ---- 主循环：等待上传请求，用 select 可靠检测连接 ---- */
        while (1) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(sock, &rfds);
            struct timeval tv = { .tv_sec = 30, .tv_usec = 0 };

            int sel_ret = select(sock + 1, &rfds, NULL, NULL, &tv);
            if (sel_ret < 0) {
                ESP_LOGW(TAG, "select 错误, errno=%d", errno);
                break;
            }
            if (sel_ret == 0) {
                /* 30 秒无数据，发送 PINGREQ 保活 */
                uint8_t ping[] = { 0xC0, 0x00 };
                int sret = send(sock, ping, sizeof(ping), 0);
                if (sret < 0) {
                    ESP_LOGW(TAG, "PINGREQ 发送失败, errno=%d", errno);
                    break;
                }
                continue;
            }
            /* 有数据可读，处理 PUBACK / PINGRESP 等 */
            int pkt = mqtt_read_packet(sock, resp_buf, sizeof(resp_buf));
            if (pkt < 0) {
                ESP_LOGW(TAG, "读报文失败, errno=%d", errno);
                break;
            }
        }

        /* ---- 断开 ---- */
        s_connected = false;
        s_sock = -1;
        close(sock);
        ESP_LOGW(TAG, "MQTT 已断开，10 秒后重连");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/* ======================== 对外接口 ======================== */

esp_err_t onenet_mqtt_init(void)
{
    if (s_mqtt_task) {
        ESP_LOGW(TAG, "MQTT 任务已运行，跳过");
        return ESP_OK;
    }

    BaseType_t ret = xTaskCreate(mqtt_task, "mqtt_onenet",
                                  8192, NULL, 4, &s_mqtt_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 MQTT 任务失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OneNET MQTT 后台任务已创建");
    return ESP_OK;
}

bool onenet_mqtt_is_connected(void)
{
    return s_connected;
}

void onenet_upload_all(const char *window_status, bool fan_on, bool spray_on)
{
    if (!s_connected || s_sock < 0) {
        ESP_LOGD(TAG, "MQTT 未连接，跳过上传");
        return;
    }
    send_property_publish(s_sock, window_status, fan_on, spray_on);
}

void onenet_upload_window(const char *window_status)
{
    onenet_upload_all(window_status, false, false);
}

void onenet_upload_fan(bool on)
{
    onenet_upload_all("未知", on, false);
}

void onenet_upload_spray(bool on)
{
    onenet_upload_all("未知", false, on);
}
