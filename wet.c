#include "wet.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "WET";
static bool s_wet_on = false;

esp_err_t wet_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WET_PIN),
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

    gpio_set_level(WET_PIN, 0);
    s_wet_on = false;
    ESP_LOGI(TAG, "加湿器已初始化 (GPIO %d), 初始关闭", WET_PIN);
    return ESP_OK;
}

void wet_on(void)
{
    gpio_set_level(WET_PIN, 1);
    s_wet_on = true;
    ESP_LOGI(TAG, "加湿器打开");
}

void wet_off(void)
{
    gpio_set_level(WET_PIN, 0);
    s_wet_on = false;
    ESP_LOGI(TAG, "加湿器关闭");
}

void wet_set(bool state)
{
    if (state) {
        wet_on();
    } else {
        wet_off();
    }
}

bool wet_get_state(void)
{
    return s_wet_on;
}
