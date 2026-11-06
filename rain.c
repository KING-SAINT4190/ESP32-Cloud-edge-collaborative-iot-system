/**
 * @file rain.c
 * @brief 雨水传感器模块实现 (Raindrop Sensor)
 */


#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include <stdbool.h>
#include "rain.h"
#include "esp_check.h"
static const char *TAG = "RAIN";

// 静态变量，用于保存模块状态
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static int s_ao_pin = -1;
static int s_do_pin = -1;

// ADC 通道配置 (GPIO1 默认对应 ADC1_CHANNEL_0)
// 用户可根据实际接线修改宏定义
#define ADC_UNIT        ADC_UNIT_1
#define ADC_CHANNEL     ADC_CHANNEL_0
#define ADC_BITWIDTH    ADC_BITWIDTH_12
#define ADC_ATTEN       ADC_ATTEN_DB_12

/**
 * @brief 获取 ADC 原始读数
 * 
 * @param adc_handle ADC 单元句柄
 * @param channel    ADC 通道
 * @param raw_value  输出原始值
 * @return
 *      - ESP_OK: 成功
 *      - ESP_ERR_INVALID_ARG: 参数无效
 */
static esp_err_t adc_read_raw(adc_oneshot_unit_handle_t adc_handle,
                              adc_channel_t channel, int *raw_value) {
    if (adc_handle == NULL || raw_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return adc_oneshot_read(adc_handle, channel, raw_value);
}

esp_err_t rain_init(int ao_pin, int do_pin) {
    // 参数检查
    if (ao_pin < 0 || do_pin < 0) {
        ESP_LOGE(TAG, "Invalid pin number");
        return ESP_ERR_INVALID_ARG;
    }

    // 保存引脚号
    s_ao_pin = ao_pin;
    s_do_pin = do_pin;

    // -------------------- 配置 ADC (AO) --------------------
    // ADC 初始化配置
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_config, &s_adc_handle),
                        TAG, "ADC 初始化失败");

    // ADC 通道配置
    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL, &chan_config),
                        TAG, "ADC 通道配置失败");

    // -------------------- 配置 GPIO (DO) --------------------
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << do_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "GPIO DO 配置失败");

    ESP_LOGI(TAG, "雨水传感器初始化成功: AO=GPIO%d, DO=GPIO%d", ao_pin, do_pin);
    return ESP_OK;
}

esp_err_t rain_read_analog_raw(int *raw_value) {
    if (s_adc_handle == NULL) {
        ESP_LOGE(TAG, "ADC 未初始化，请先调用 rain_init");
        return ESP_ERR_INVALID_STATE;
    }
    if (raw_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return adc_read_raw(s_adc_handle, ADC_CHANNEL, raw_value);
}

esp_err_t rain_read_digital(bool *is_wet) {
    if (s_do_pin < 0) {
        ESP_LOGE(TAG, "GPIO 未初始化，请先调用 rain_init");
        return ESP_ERR_INVALID_STATE;
    }
    if (is_wet == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 读取数字引脚电平
    // DO 输出低电平表示检测到雨水（有水），高电平表示干燥
    int level = gpio_get_level(s_do_pin);
    *is_wet = (level == 0);

    return ESP_OK;
}

esp_err_t rain_deinit(void) {
    // 释放 ADC 资源
    if (s_adc_handle) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }

    // 重置引脚号
    s_ao_pin = -1;
    s_do_pin = -1;

    ESP_LOGI(TAG, "雨水传感器已反初始化");
    return ESP_OK;
}
