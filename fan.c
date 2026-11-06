#include "fan.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "FAN";
static bool s_fan_on = false;

esp_err_t fan_init(void)
{
    /* 配置为推挽输出，初始高电平（反相电路：HIGH=关断，LOW=导通） */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FAN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO 配置失败: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(FAN_PIN, 1);       /* HIGH = 关断（反相电路） */
    s_fan_on = false;
    ESP_LOGI(TAG, "风扇已初始化 (GPIO %d, 反相驱动), 默认关闭", FAN_PIN);
    return ESP_OK;
}

void fan_on(void)
{
    gpio_set_level(FAN_PIN, 0);       /* LOW = 导通（反相电路） */
    s_fan_on = true;
    ESP_LOGI(TAG, "风扇打开");
}

void fan_off(void)
{
    gpio_set_level(FAN_PIN, 1);       /* HIGH = 关断（反相电路） */
    s_fan_on = false;
    ESP_LOGI(TAG, "风扇关闭");
}

void fan_set(bool state)
{
    if (state) {
        fan_on();
    } else {
        fan_off();
    }
}

bool fan_get_state(void)
{
    return s_fan_on;
}
