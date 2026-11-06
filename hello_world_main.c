#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "weather.h"
#include "LLM.h"
#include "cJSON.h"
#include "SG90.h"
#include "BUZZER.h"
#include "redray.h"
#include "OLED.h"
#include "DHT11.h"
#include "Wind_Sensor.h"
#include "fan.h"
#include "wet.h"
#include "KNN.h"
#include "rain.h"
#include "MQTT.h"
#include <lwip/inet.h>

#define WIFI_SSID       "oppo_saint"
#define WIFI_PASS       "88888888"

#define WIFI_CONNECTED_BIT  BIT0

static const char *TAG = "MAIN";
static EventGroupHandle_t s_wifi_event_group;

/* WiFi 是否已初始化（懒加载标志） */
static bool s_wifi_inited = false;

/* 模式标志：false=红外遥控模式，true=自动模式 */
static bool s_auto_mode = false;

/* 当前舵机角度，-1 表示未知（上电初始态） */
static int s_current_angle = -1;

/* DHT11 上次读取的 tick 计数，用于控制读取频率 */
static TickType_t s_last_dht_tick = 0;

/* 前向声明 */
static void sync_time(void);
static bool wifi_wait_connected(int timeout_ms);

/* ======================== WiFi 事件处理 ======================== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi 启动，正在连接...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi 断开，尝试重连...");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "获得 IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief 懒加载 WiFi 初始化（仅首次真正初始化，后续调用跳过）
 *        非阻塞：启动连接后立即返回
 */
static void wifi_lazy_init(void)
{
    if (s_wifi_inited) return;
    s_wifi_inited = true;

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi 已启动，后台连接中...");
}

/**
 * @brief 等待 WiFi 连接（阻塞），最多等 timeout_ms
 * @return true=已连接, false=超时
 */
static bool wifi_wait_connected(int timeout_ms)
{
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(timeout_ms));
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

/* ======================== 舵机控制（带音效反馈） ======================== */

static void set_servo_angle_with_sound(int angle)
{
    if (angle == s_current_angle) return;

    Servo_SetAngle(angle);
    s_current_angle = angle;

    switch (angle) {
        case 65:
            Buzzer_StartupTone();
            break;
        case 45:
            Buzzer_ConfirmTone();
            break;
        case 25:
            Buzzer_Stop();
            break;
    }
}

/* ======================== MQTT 状态上报辅助 ======================== */

/**
 * @brief 舵机角度 → 窗口状态字符串（最长10字符）
 */
static const char *angle_to_window_str(int angle)
{
    switch (angle) {
        case 65: return "全开";
        case 45: return "半开";
        case 25: return "关闭";
        default: return "未知";
    }
}

/* ======================== KNN 决策 ======================== */

/**
 * @brief 将 KNN 预测结果转换为通用的控制指令结构体
 */
static void knn_to_llm_command(const knn_result_t *knn, llm_command_t *cmd)
{
    if (!knn || !cmd) return;

    /* 窗户状态 → 舵机角度: 0=关闭(25°), 1=半开(45°), 2=全开(65°) */
    if (knn->window_open) {
        cmd->servo_angle = knn->window_half ? 45 : 65;
    } else {
        cmd->servo_angle = 25;
    }

    cmd->fan_on = knn->fan_on;
    cmd->wet_on = knn->humidifier_on;
    snprintf(cmd->reason, sizeof(cmd->reason), "KNN本地决策");
}

/* ======================== 大模型指令执行 ======================== */

static void execute_command(const llm_command_t *cmd)
{
    if (!cmd) return;

    ESP_LOGI(TAG, ">>> 执行控制指令: 舵机=%d°, 风扇=%s, 加湿器=%s, 理由=%s",
             cmd->servo_angle,
             cmd->fan_on ? "开" : "关",
             cmd->wet_on ? "开" : "关",
             cmd->reason);

    /* 舵机控制 */
    set_servo_angle_with_sound(cmd->servo_angle);

    /* 风扇控制 */
    fan_set(cmd->fan_on);

    /* 加湿器控制 */
    wet_set(cmd->wet_on);

    /* 上报状态到 OneNET */
    onenet_upload_all(angle_to_window_str(cmd->servo_angle),
                      cmd->fan_on, cmd->wet_on);
}

/* ======================== OLED 显示更新 ======================== */

static void update_oled_display(void)
{
    TickType_t now = xTaskGetTickCount();

    /* 每 2 秒读取一次 DHT11 和风速 */
    if (now - s_last_dht_tick >= pdMS_TO_TICKS(2000)) {
        s_last_dht_tick = now;

        /* DHT11 温湿度 */
        dht11_t dht;
        if (DHT11_Read(&dht) == ESP_OK) {
            char line1[32];
            snprintf(line1, sizeof(line1), "Temp:%dC Humi:%d%%", dht.temperature, dht.humidity);
            OLED_ShowString(0, 1, line1);
        } else {
            OLED_ShowString(0, 1, "DHT11 Err       ");
        }

        /* 风速传感器 */
        float wind_speed = 0;
        if (wind_sensor_read_speed(&wind_speed, 500) == ESP_OK) {
            char line2[32];
            snprintf(line2, sizeof(line2), "Wind:%.1f m/s     ", wind_speed);
            OLED_ShowString(0, 2, line2);
        } else {
            OLED_ShowString(0, 2, "Wind Err        ");
        }

        /* 第4行：加湿器和风扇状态 */
        char line3[32];
        snprintf(line3, sizeof(line3), "Fan:%s Wet:%s   ",
                 fan_get_state() ? "ON" : "OFF",
                 wet_get_state() ? "ON" : "OFF");
        OLED_ShowString(0, 3, line3);
    }

    /* 第一行：显示当前工作模式 */
    if (s_auto_mode) {
        OLED_ShowString(0, 0, "Auto Mode       ");
    } else {
        OLED_ShowString(0, 0, "Manual Mode     ");
    }
}

/* ======================== 红外遥控回调 ======================== */

static void ir_callback(uint16_t user_code, uint8_t key_code, bool repeat, void *user_data)
{
    if (repeat) return;

    if (s_auto_mode) {
        if (key_code == 0x47) {
            s_auto_mode = false;
            Buzzer_StartupTone();
            ESP_LOGI(TAG, "====================================");
            ESP_LOGI(TAG, "切换到【红外遥控模式】");
            ESP_LOGI(TAG, "  0x0C → 25°  0x5E → 65°  0x18 → 45°");
            ESP_LOGI(TAG, "  0x08 → 加湿器开关  0x1C → 风扇开关");
            ESP_LOGI(TAG, "  0x45 → 切换到自动模式");
            ESP_LOGI(TAG, "====================================");
            OLED_ShowString(0, 0, "Manual Mode     ");
        }
    } else {
        switch (key_code) {
            case 0x0C:
                ESP_LOGI(TAG, "红外遥控: 转到 65°(全开)");
                set_servo_angle_with_sound(65);
                onenet_upload_all("全开",
                                  fan_get_state(), wet_get_state());
                break;
            case 0x5E:
                ESP_LOGI(TAG, "红外遥控: 转到 25°(全关)");
                set_servo_angle_with_sound(25);
                onenet_upload_all("关闭",
                                  fan_get_state(), wet_get_state());
                break;
            case 0x18:
                ESP_LOGI(TAG, "红外遥控: 转到 45°(半开)");
                set_servo_angle_with_sound(45);
                onenet_upload_all("半开",
                                  fan_get_state(), wet_get_state());
                break;
            case 0x08: {
                bool state = !wet_get_state();
                wet_set(state);
                ESP_LOGI(TAG, "红外遥控: 加湿器 %s", state ? "打开" : "关闭");
                onenet_upload_spray(state);
                break;
            }
            case 0x1C: {
                bool state = !fan_get_state();
                fan_set(state);
                ESP_LOGI(TAG, "红外遥控: 风扇 %s", state ? "打开" : "关闭");
                onenet_upload_fan(state);
                break;
            }
            case 0x45:
                /* 懒加载 WiFi（首次才真正初始化） */
                wifi_lazy_init();
                Buzzer_StartupTone();
                ESP_LOGI(TAG, "====================================");
                ESP_LOGI(TAG, "切换到【自动模式】- 等待 WiFi 连接...");
                ESP_LOGI(TAG, "  0x47 → 切换回红外遥控模式");
                ESP_LOGI(TAG, "====================================");
                /* 等待最多 15 秒连 WiFi */
                if (wifi_wait_connected(15000)) {
                    ESP_LOGI(TAG, "WiFi 连接成功，同步 NTP 时间...");
                    sync_time();
                    /* WiFi 就绪后初始化 MQTT 连接 OneNET */
                    onenet_mqtt_init();
                    ESP_LOGI(TAG, "自动模式已准备就绪");
                } else {
                    ESP_LOGW(TAG, "WiFi 连接超时，将使用 KNN 本地决策");
                }
                s_auto_mode = true;
                OLED_ShowString(0, 0, "Auto Mode        ");
                break;
            default:
                ESP_LOGW(TAG, "红外遥控: 未定义键码 0x%02X", key_code);
                break;
        }
    }
}

/* ======================== SNTP 时间同步 ======================== */

static void sync_time(void)
{
    ESP_LOGI(TAG, "正在同步 NTP 时间...");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_setservername(1, "time.nist.gov");
    sntp_init();

    /* 最多等待 30 秒 */
    int retry = 60;
    while (retry-- > 0) {
        time_t now = 0;
        time(&now);
        if (now > 1735689600) {  /* 2025-01-01 之后 */
            struct tm tm;
            localtime_r(&now, &tm);
            ESP_LOGI(TAG, "NTP 时间同步成功: %04d-%02d-%02d %02d:%02d:%02d",
                     tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                     tm.tm_hour, tm.tm_min, tm.tm_sec);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGW(TAG, "NTP 时间同步超时，继续执行...");
}

/* ======================== 主工作任务 ======================== */

static void auto_work_task(void *arg)
{
    /* 风扇必须在最前面初始化，最小化启动时 GPIO 高阻窗口 */
    fan_init();
    ESP_LOGI(TAG, "风扇已初始化");

    Servo_Init();
    Buzzer_Init();
    OLED_Init();
    DHT11_Init();
    ESP_LOGI(TAG, "舵机 + 蜂鸣器 + OLED + DHT11 初始化完成");

    ESP_ERROR_CHECK(wind_sensor_init(WIND_SENSOR_DEFAULT_ADDR, WIND_SENSOR_BAUDRATE));
    ESP_LOGI(TAG, "风速传感器初始化完成");

    wet_init();
    ESP_LOGI(TAG, "加湿器已初始化");

    ESP_ERROR_CHECK(redray_init(11, ir_callback, NULL));
    ESP_ERROR_CHECK(redray_start());
    ESP_LOGI(TAG, "红外接收已启动（GPIO 11）");

    knn_init();
    ESP_LOGI(TAG, "KNN 决策模型已加载");

    ESP_ERROR_CHECK(rain_init(1, 2));
    ESP_LOGI(TAG, "雨水传感器已初始化（AO=GPIO1, DO=GPIO2）");

    /* 初始显示 */
    OLED_ShowString(0, 0, "Manual Mode     ");
    OLED_ShowString(0, 1, "Servo:65 Init   ");
    set_servo_angle_with_sound(65);

    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "系统已启动，默认【红外遥控模式】");
    ESP_LOGI(TAG, "  0x0C → 25°  0x5E → 65°  0x18 → 45°");
    ESP_LOGI(TAG, "  0x45 → 切换到自动模式");
    ESP_LOGI(TAG, "====================================");

    while (1) {
        update_oled_display();

        if (s_auto_mode) {
            /* 首次进入自动模式且 WiFi 未初始化时触发懒加载 */
            if (!s_wifi_inited) {
                wifi_lazy_init();
            }

            /* 检查 WiFi 连接状态（非阻塞） */
            wifi_ap_record_t ap_info;
            bool wifi_ok = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);

            if (!wifi_ok) {
                ESP_LOGW(TAG, "WiFi 已断开，触发后台重连，使用 KNN 本地决策");
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                esp_wifi_connect();  /* 非阻塞，event handler 后台重连 */
            }

            if (wifi_ok) {
                /* WiFi 可用时确保 MQTT 已初始化（幂等，仅首次生效） */
                onenet_mqtt_init();

                /* ========== 在线模式：天气 API + 大模型 ========== */

                ESP_LOGI(TAG, "--- 正在获取天气数据 ---");
                cJSON *weather = weather_fetch_json("南京");
                if (!weather) {
                    ESP_LOGE(TAG, "获取天气失败，60秒后重试");
                    vTaskDelay(pdMS_TO_TICKS(60000));
                    continue;
                }

                cJSON *temp_obj = cJSON_GetObjectItem(weather, "temperature");
                cJSON *humi_obj = cJSON_GetObjectItem(weather, "humidity");
                cJSON *wind_obj = cJSON_GetObjectItem(weather, "wind_power");
                cJSON *wthr_obj = cJSON_GetObjectItem(weather, "weather");

                int temperature = (temp_obj && cJSON_IsNumber(temp_obj)) ? (int)temp_obj->valuedouble : 0;
                int humidity = (humi_obj && cJSON_IsNumber(humi_obj)) ? (int)humi_obj->valuedouble : 0;

                char wind_power_buf[32] = "未知";
                char weather_desc_buf[32] = "未知";
                if (wind_obj && cJSON_IsString(wind_obj))
                    strlcpy(wind_power_buf, wind_obj->valuestring, sizeof(wind_power_buf));
                if (wthr_obj && cJSON_IsString(wthr_obj))
                    strlcpy(weather_desc_buf, wthr_obj->valuestring, sizeof(weather_desc_buf));

                ESP_LOGI(TAG, "天气: %s, 温度: %d℃, 湿度: %d%%, 风力: %s",
                         weather_desc_buf, temperature, humidity, wind_power_buf);

                cJSON_Delete(weather);

                /* 读取本地传感器：室内 DHT11 + 雨水 + 本地风速 */
                dht11_t dht;
                int indoor_temp = 25, indoor_humi = 50;
                if (DHT11_Read(&dht) == ESP_OK) {
                    indoor_temp = dht.temperature;
                    indoor_humi = dht.humidity;
                }
                int rain_adc = 0;
                rain_read_analog_raw(&rain_adc);
                const char *rain_str = (rain_adc < 2000) ? "有雨" : "无雨";

                float local_wind_mps = 0.0f;
                wind_sensor_read_speed(&local_wind_mps, 500);
                char local_wind_str[16];
                snprintf(local_wind_str, sizeof(local_wind_str), "%.1fm/s", local_wind_mps);

                ESP_LOGI(TAG, "室内: %d°C %d%%, 雨水ADC=%d (%s), 本地风速=%s",
                         indoor_temp, indoor_humi, rain_adc, rain_str, local_wind_str);

                ESP_LOGI(TAG, "--- 正在请求大模型云端指令 ---");
                llm_command_t cmd;
                esp_err_t llm_ret = llm_get_command(temperature, humidity,
                                                    wind_power_buf, weather_desc_buf,
                                                    indoor_temp, indoor_humi, rain_str,
                                                    local_wind_mps,
                                                    &cmd);
                if (llm_ret == ESP_OK) {
                    ESP_LOGI(TAG, "云端下发指令: 舵机=%d°, 风扇=%s, 加湿器=%s",
                             cmd.servo_angle,
                             cmd.fan_on ? "开" : "关",
                             cmd.wet_on ? "开" : "关");
                    execute_command(&cmd);
                } else {
                    ESP_LOGE(TAG, "大模型请求失败，使用默认指令");
                    llm_command_t default_cmd = { .servo_angle = 65, .fan_on = false, .wet_on = false };
                    snprintf(default_cmd.reason, sizeof(default_cmd.reason), "大模型请求失败");
                    execute_command(&default_cmd);
                }
            } else {
                /* ========== 离线模式：本地传感器 + KNN 决策 ========== */

                ESP_LOGI(TAG, "--- 读取本地传感器 ---");

                /* DHT11 温湿度 */
                dht11_t dht;
                float temp = 25.0f, humi = 50.0f;
                if (DHT11_Read(&dht) == ESP_OK) {
                    temp = (float)dht.temperature;
                    humi = (float)dht.humidity;
                    ESP_LOGI(TAG, "本地温度: %.0f℃, 湿度: %.0f%%", temp, humi);
                } else {
                    ESP_LOGW(TAG, "DHT11 读取失败，使用默认值");
                }

                /* 雨水 ADC 值 */
                int rain_adc = 0;
                if (rain_read_analog_raw(&rain_adc) == ESP_OK) {
                    ESP_LOGI(TAG, "雨水 ADC: %d", rain_adc);
                } else {
                    ESP_LOGW(TAG, "雨水传感器读取失败，使用默认值 0");
                }

                /* 风速 */
                float wind_mps = 0.0f;
                if (wind_sensor_read_speed(&wind_mps, 500) == ESP_OK) {
                    ESP_LOGI(TAG, "风速: %.1f m/s", wind_mps);
                } else {
                    ESP_LOGW(TAG, "风速读取失败，使用默认值");
                }

                /* KNN 本地决策 */
                ESP_LOGI(TAG, "--- KNN 本地决策 ---");
                knn_result_t knn_res = knn_model_predict_detailed(
                    temp, humi, (float)rain_adc, wind_mps);

                uint32_t _win = knn_res.window_open ? (knn_res.window_half ? 1 : 2) : 0;
                uint32_t _fan = knn_res.fan_on ? 1 : 0;
                uint32_t _wet = knn_res.humidifier_on ? 1 : 0;
                ESP_LOGI(TAG, "KNN 标签=%u (窗=%u 扇=%u 湿=%u)",
                         knn_res.label, _win, _fan, _wet);

                llm_command_t cmd;
                knn_to_llm_command(&knn_res, &cmd);
                execute_command(&cmd);
            }

            ESP_LOGI(TAG, "--- 60 秒后再次获取 ---");
            vTaskDelay(pdMS_TO_TICKS(60000));
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

/* ======================== 主函数 ======================== */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 尽早初始化风扇 GPIO 为输出低电平，避免启动时高阻态导致 MOSFET 误开 */
    fan_init();

    xTaskCreate(auto_work_task, "auto_work", 16384, NULL, 5, NULL);
}