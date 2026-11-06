#include "DHT11.h"

static const char *TAG = "DHT11";
// 全版本通用临界区锁
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// 读取1字节数据（标准28us延时）
static uint8_t read_byte(void) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        while (gpio_get_level(DHT11_DATA_PIN) == 0);
        esp_rom_delay_us(28);
        data <<= 1;
        if (gpio_get_level(DHT11_DATA_PIN)) data |= 1;
        while (gpio_get_level(DHT11_DATA_PIN) == 1);
    }
    return data;
}

// 初始化
void DHT11_Init(void) {
    gpio_reset_pin(DHT11_DATA_PIN);
    gpio_set_pull_mode(DHT11_DATA_PIN, GPIO_PULLUP_ONLY);   /* DHT11 数据线需要上拉 */
    vTaskDelay(pdMS_TO_TICKS(2000));
}

// 主读取函数（带临界区 + 重试）
esp_err_t DHT11_Read(dht11_t *result) {
    uint8_t buf[5] = {0};
    int retry = 3; // 失败自动重试3次

    while (retry--) {
        // 0. 进入临界区（全版本兼容，无报错）
        taskENTER_CRITICAL(&mux);

        // 1. 发送起始信号
        gpio_set_direction(DHT11_DATA_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(DHT11_DATA_PIN, 0);
        esp_rom_delay_us(20000);
        gpio_set_level(DHT11_DATA_PIN, 1);
        esp_rom_delay_us(30);
        gpio_set_direction(DHT11_DATA_PIN, GPIO_MODE_INPUT);

        // 2. 等待应答
        if (gpio_get_level(DHT11_DATA_PIN)) {
            taskEXIT_CRITICAL(&mux);
            continue;
        }
        esp_rom_delay_us(80);
        if (!gpio_get_level(DHT11_DATA_PIN)) {
            taskEXIT_CRITICAL(&mux);
            continue;
        }
        esp_rom_delay_us(80);

        // 3. 读取40位数据
        for (int i = 0; i < 5; i++) {
            buf[i] = read_byte();
        }

        // 4. 退出临界区
        taskEXIT_CRITICAL(&mux);

        // 4. 校验 + 数据合法性判断
        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4] && buf[0] <= 100) {
            result->humidity = buf[0];
            result->temperature = buf[2];
            ESP_LOGI(TAG, "读取成功：T=%d℃, H=%d%%", result->temperature, result->humidity);
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGE(TAG, "读取失败");
    return ESP_FAIL;
}